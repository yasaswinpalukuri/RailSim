#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace railsim {

// Strong type: a BlockId cannot be passed where a raw index or a TrainId is
// expected. The value is the block's index inside TrackGraph.
struct BlockId {
    std::size_t value{};

    friend bool operator==(BlockId lhs, BlockId rhs) { return lhs.value == rhs.value; }
    friend bool operator!=(BlockId lhs, BlockId rhs) { return !(lhs == rhs); }
};

struct TrainId {
    std::size_t value{};

    friend bool operator==(TrainId lhs, TrainId rhs) { return lhs.value == rhs.value; }
    friend bool operator!=(TrainId lhs, TrainId rhs) { return !(lhs == rhs); }
};

// Name used for a train in the event log, e.g. "T1".
inline std::string train_label(TrainId train) {
    return "T" + std::to_string(train.value);
}

// One train of a scenario: where it starts, where it must go, how fast it wants to run.
struct TrainSpec {
    BlockId start;
    BlockId destination;
    double cruise_speed_mps{};
};

// Infrastructure state, fixed by the track file or by maintenance.
// Occupancy is dynamic state and belongs to the interlocking, not here.
enum class BlockStatus : std::uint8_t { InService, OutOfService };

}  // namespace railsim
