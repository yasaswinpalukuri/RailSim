#include "railsim/interlocking.hpp"

#include <algorithm>
#include <string>

namespace railsim {
std::string_view to_string(MoveResult result) {
    switch (result) {
        case MoveResult::Granted:
            return "granted";
        case MoveResult::OutOfService:
            return "block_out_of_service";
        case MoveResult::ReservedByOther:
            return "reserved_by_other_train";
        case MoveResult::OccupiedByOther:
            return "occupied_by_other_train";
        case MoveResult::NotReserved:
            return "block_not_reserved";
        case MoveResult::NotAdjacent:
            return "block_not_adjacent";
        case MoveResult::NotHolder:
            return "not_holder_of_block";
        case MoveResult::StillOccupied:
            return "block_still_occupied";
    }
    return "unknown";
}

Interlocking::Interlocking(const TrackGraph& graph, EventLogger& logger)
    : graph_(graph), logger_(logger), states_(graph.block_count()) {}

MoveResult Interlocking::reserve(TrainId train, BlockId block) {
    BlockState& state = states_.at(block.value);

    if (state.holder == train) {
        return MoveResult::Granted;
    }
    if (state.holder.has_value()) {
        return reject(EventType::ReserveRejected, train, block,
                      state.occupied ? MoveResult::OccupiedByOther : MoveResult::ReservedByOther);
    }
    if (graph_.block(block).status != BlockStatus::InService) {
        return reject(EventType::ReserveRejected, train, block, MoveResult::OutOfService);
    }

    state.holder = train;
    record(EventType::Reserved, train, block);
    return MoveResult::Granted;
}

MoveResult Interlocking::enter(TrainId train, BlockId block) {
    BlockState& target = states_.at(block.value);
    const std::optional<BlockId> current = position(train);

    if (current == block) {
        return MoveResult::Granted;
    }
    if (!target.holder.has_value()) {
        return reject(EventType::EnterRejected, train, block, MoveResult::NotReserved);
    }
    if (target.holder != train) {
        return reject(EventType::EnterRejected, train, block,
                      target.occupied ? MoveResult::OccupiedByOther : MoveResult::ReservedByOther);
    }
    // Checked again here: the block may have been closed after it was reserved.
    if (graph_.block(block).status != BlockStatus::InService) {
        return reject(EventType::EnterRejected, train, block, MoveResult::OutOfService);
    }

    if (current.has_value()) {
        const std::vector<BlockId>& neighbours = graph_.neighbours(*current);
        if (std::find(neighbours.begin(), neighbours.end(), block) == neighbours.end()) {
            return reject(EventType::EnterRejected, train, block, MoveResult::NotAdjacent);
        }
        states_[current->value] = BlockState{};
        record(EventType::Left, train, *current);
    }

    target.occupied = true;
    positions_.insert_or_assign(train.value, block);
    record(EventType::Entered, train, block);
    return MoveResult::Granted;
}

MoveResult Interlocking::release(TrainId train, BlockId block) {
    BlockState& state = states_.at(block.value);

    if (state.holder != train) {
        return reject(EventType::ReleaseRejected, train, block, MoveResult::NotHolder);
    }
    if (state.occupied) {
        return reject(EventType::ReleaseRejected, train, block, MoveResult::StillOccupied);
    }

    state.holder.reset();
    record(EventType::Released, train, block);
    return MoveResult::Granted;
}

std::optional<TrainId> Interlocking::holder(BlockId block) const {
    return states_.at(block.value).holder;
}

bool Interlocking::is_occupied(BlockId block) const {
    return states_.at(block.value).occupied;
}

std::optional<BlockId> Interlocking::position(TrainId train) const {
    const auto it = positions_.find(train.value);
    if (it == positions_.end()) {
        return std::nullopt;
    }
    return it->second;
}

void Interlocking::record(EventType type, TrainId train, BlockId block) {
    logger_.log(type, train_label(train), graph_.block(block).name);
}

MoveResult Interlocking::reject(EventType type, TrainId train, BlockId block, MoveResult reason) {
    logger_.log(type, train_label(train), graph_.block(block).name, std::string(to_string(reason)));
    return reason;
}

}  // namespace railsim
