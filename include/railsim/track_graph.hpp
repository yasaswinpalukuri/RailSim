#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "railsim/types.hpp"

namespace railsim {

struct Block {
    std::string name;
    double length_m{};
    BlockStatus status{BlockStatus::InService};
};

// Undirected graph of track blocks stored as an adjacency list.
// Blocks are nodes; a link means a train can move directly between two blocks.
class TrackGraph {
public:
    // Returns nullopt if the name is empty or already used, or the length is
    // not a positive finite number.
    [[nodiscard]] std::optional<BlockId> add_block(std::string name, double length_m,
                                                   BlockStatus status = BlockStatus::InService);

    // Links two blocks in both directions. Returns false for an unknown id,
    // a self-link, or a link that already exists.
    [[nodiscard]] bool add_link(BlockId first, BlockId second);

    // Names a block as a station. Returns false if the station name is empty
    // or already used, or the block id is unknown.
    [[nodiscard]] bool add_station(std::string station_name, BlockId block);

    [[nodiscard]] std::optional<BlockId> find_block(const std::string& name) const;
    [[nodiscard]] std::optional<BlockId> find_station(const std::string& name) const;

    [[nodiscard]] bool contains(BlockId id) const { return id.value < blocks_.size(); }

    // These three throw std::out_of_range for an unknown id: passing one is a
    // bug in the caller, not an expected runtime condition.
    [[nodiscard]] const Block& block(BlockId id) const;
    [[nodiscard]] const std::vector<BlockId>& neighbours(BlockId id) const;
    void set_status(BlockId id, BlockStatus status);

    [[nodiscard]] std::size_t block_count() const { return blocks_.size(); }
    [[nodiscard]] std::size_t link_count() const { return link_count_; }
    [[nodiscard]] std::size_t station_count() const { return station_index_.size(); }

private:
    std::vector<Block> blocks_;                    // indexed by BlockId::value
    std::vector<std::vector<BlockId>> adjacency_;  // indexed by BlockId::value
    std::unordered_map<std::string, BlockId> block_index_;
    std::unordered_map<std::string, BlockId> station_index_;
    std::size_t link_count_{0};
};

}  // namespace railsim
