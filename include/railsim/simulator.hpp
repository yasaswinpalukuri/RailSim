#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "railsim/event_logger.hpp"
#include "railsim/interlocking.hpp"
#include "railsim/track_graph.hpp"
#include "railsim/train.hpp"
#include "railsim/types.hpp"

namespace railsim {

struct SimulationConfig {
    double dt_s{0.5};
    std::uint64_t max_ticks{4000};
    TrainConfig train;
};

struct SimulationResult {
    std::uint64_t ticks{};
    std::size_t trains{};
    std::size_t arrived{};
    std::size_t safety_violations{};

    [[nodiscard]] bool all_arrived() const { return arrived == trains; }
};

// Fixed time-step simulation of several trains on one track network.
//
// Each tick, trains are processed in id order (T1 first), which makes a run
// fully repeatable. For every train that has not arrived:
//   1. if it holds no block ahead, plan a route and ask the interlocking for
//      the next block on it;
//   2. move the train, telling it that its movement authority ends at the far
//      end of the last block it holds;
//   3. when it crosses into the next block, ask the interlocking to enter it.
// After all trains have moved, an independent check counts trains per block.
//
// A train that has arrived stays in its destination block.
class Simulator {
public:
    // Places train i on specs[i].start as TrainId{i + 1}.
    // Throws std::invalid_argument if dt_s is not positive, a block is too
    // short to stop in (not longer than twice the safety margin), or a train
    // cannot be placed; std::out_of_range if a spec names an unknown block.
    Simulator(const TrackGraph& graph, const std::vector<TrainSpec>& specs, EventLogger& logger,
              const SimulationConfig& config = {});

    // Advances one tick. Returns true while the simulation should continue.
    bool step();

    // Steps until every train has arrived or max_ticks is reached.
    SimulationResult run();

    [[nodiscard]] SimulationResult result() const;
    [[nodiscard]] const Interlocking& interlocking() const { return interlocking_; }

    [[nodiscard]] std::size_t train_count() const { return trains_.size(); }
    [[nodiscard]] BlockId block_of(std::size_t index) const { return trains_.at(index).current; }
    [[nodiscard]] double speed_of(std::size_t index) const {
        return trains_.at(index).train.speed_mps();
    }

private:
    struct TrainState {
        TrainState(const Train& new_train, const TrainSpec& new_spec)
            : train(new_train), spec(new_spec), current(new_spec.start) {}

        Train train;
        TrainSpec spec;
        BlockId current;                // block the train is in
        double offset_m{0.0};           // distance travelled inside `current`
        std::optional<BlockId> next;    // block ahead that this train holds
        std::optional<BlockId> refused; // block ahead that was refused and is still held by another train
        std::vector<BlockId> route;     // last planned route, starting at `current`
        bool arrived{false};
        bool no_route{false};
    };

    void update_authority(TrainState& state);
    void move(TrainState& state);
    void check_safety();
    [[nodiscard]] bool is_blocked_for(const TrainState& state, BlockId block) const;
    [[nodiscard]] bool finished() const;
    [[nodiscard]] std::string route_text(const std::vector<BlockId>& route) const;
    void log(EventType type, const TrainState& state, std::string reason = "");

    const TrackGraph& graph_;
    EventLogger& logger_;
    SimulationConfig config_;
    Interlocking interlocking_;
    std::vector<TrainState> trains_;
    std::uint64_t tick_{0};
    std::size_t safety_violations_{0};
    bool timed_out_{false};
};

}  // namespace railsim
