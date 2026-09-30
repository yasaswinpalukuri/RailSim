#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "railsim/event_logger.hpp"
#include "railsim/track_graph.hpp"
#include "railsim/types.hpp"

namespace railsim {

enum class MoveResult : std::uint8_t {
    Granted,
    OutOfService,     // the block is closed
    ReservedByOther,  // another train holds the block but is not in it
    OccupiedByOther,  // another train is in the block
    NotReserved,      // enter without a reservation
    NotAdjacent,      // enter a block that is not linked to the train's current block
    NotHolder,        // release a block the train does not hold
    StillOccupied,    // release the block the train is standing in
};

[[nodiscard]] std::string_view to_string(MoveResult result);

// The safety authority. It owns which train holds and occupies each block and
// is the only component allowed to change that.
//
// Rules:
//   - a block has at most one holder, and only the holder can occupy it;
//   - a train must reserve a block before entering it;
//   - a train is in exactly one block, and leaving a block releases it.
// Every request is denied unless all checks pass, and every rejection is
// logged with its reason.
//
// Passing a BlockId that is not in the graph throws std::out_of_range.
// Blocks must not be added to the graph after the interlocking is built.
class Interlocking {
public:
    Interlocking(const TrackGraph& graph, EventLogger& logger);

    // Reserving a block the train already holds is granted and changes nothing.
    [[nodiscard]] MoveResult reserve(TrainId train, BlockId block);

    // Moves the train into `block`. The first call places the train on the
    // network; later calls must target a neighbour of its current block.
    [[nodiscard]] MoveResult enter(TrainId train, BlockId block);

    // Gives up a reservation the train holds but is not standing in.
    [[nodiscard]] MoveResult release(TrainId train, BlockId block);

    [[nodiscard]] std::optional<TrainId> holder(BlockId block) const;
    [[nodiscard]] bool is_occupied(BlockId block) const;
    [[nodiscard]] std::optional<BlockId> position(TrainId train) const;

private:
    // One holder field for both "reserved" and "occupied": two trains in the
    // same block cannot even be represented.
    struct BlockState {
        std::optional<TrainId> holder;
        bool occupied{false};
    };

    void record(EventType type, TrainId train, BlockId block);
    MoveResult reject(EventType type, TrainId train, BlockId block, MoveResult reason);

    const TrackGraph& graph_;
    EventLogger& logger_;
    std::vector<BlockState> states_;                       // indexed by BlockId::value
    std::unordered_map<std::size_t, BlockId> positions_;   // keyed by TrainId::value
};

}  // namespace railsim
