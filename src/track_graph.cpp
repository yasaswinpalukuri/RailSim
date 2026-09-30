#include "railsim/track_graph.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace railsim {

std::optional<BlockId> TrackGraph::add_block(std::string name, double length_m,
                                             BlockStatus status) {
    if (!std::isfinite(length_m) || length_m <= 0.0) {
        return std::nullopt;
    }
    if (name.empty() || block_index_.count(name) != 0) {
        return std::nullopt;
    }

    const BlockId id{blocks_.size()};
    block_index_.emplace(name, id);
    blocks_.push_back(Block{std::move(name), length_m, status});
    adjacency_.emplace_back();
    return id;
}

bool TrackGraph::add_link(BlockId first, BlockId second) {
    if (!contains(first) || !contains(second) || first == second) {
        return false;
    }

    auto& from_first = adjacency_[first.value];
    if (std::find(from_first.begin(), from_first.end(), second) != from_first.end()) {
        return false;
    }

    from_first.push_back(second);
    adjacency_[second.value].push_back(first);
    ++link_count_;
    return true;
}

bool TrackGraph::add_station(std::string station_name, BlockId block) {
    if (station_name.empty() || !contains(block)) {
        return false;
    }
    return station_index_.emplace(std::move(station_name), block).second;
}

std::optional<BlockId> TrackGraph::find_block(const std::string& name) const {
    const auto it = block_index_.find(name);
    if (it == block_index_.end()) {
        return std::nullopt;
    }
    return it->second;
}

std::optional<BlockId> TrackGraph::find_station(const std::string& name) const {
    const auto it = station_index_.find(name);
    if (it == station_index_.end()) {
        return std::nullopt;
    }
    return it->second;
}

const Block& TrackGraph::block(BlockId id) const {
    return blocks_.at(id.value);
}

const std::vector<BlockId>& TrackGraph::neighbours(BlockId id) const {
    return adjacency_.at(id.value);
}

void TrackGraph::set_status(BlockId id, BlockStatus status) {
    blocks_.at(id.value).status = status;
}

}  // namespace railsim
