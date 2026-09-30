#include "railsim/simulator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "railsim/route_planner.hpp"

namespace railsim {
namespace {

// Below this speed a train inside its destination block counts as stopped.
constexpr double kArrivalSpeedMps = 0.1;

}  // namespace

Simulator::Simulator(const TrackGraph& graph, const std::vector<TrainSpec>& specs,
                     EventLogger& logger, const SimulationConfig& config)
    : graph_(graph), logger_(logger), config_(config), interlocking_(graph, logger) {
    if (!std::isfinite(config.dt_s) || config.dt_s <= 0.0) {
        throw std::invalid_argument("Simulator: dt_s must be positive");
    }

    // A train stops two safety margins before the end of its authority, so it
    // could never enter, or arrive in, a block that is not longer than that.
    const double shortest_usable = 2.0 * config.train.safety_margin_m;
    for (std::size_t i = 0; i < graph.block_count(); ++i) {
        if (graph.block(BlockId{i}).length_m <= shortest_usable) {
            throw std::invalid_argument("Simulator: block '" + graph.block(BlockId{i}).name +
                                        "' is not longer than twice the safety margin");
        }
    }

    trains_.reserve(specs.size());
    for (std::size_t i = 0; i < specs.size(); ++i) {
        const TrainId id{i + 1};
        const TrainSpec& spec = specs[i];
        if (!graph.contains(spec.start) || !graph.contains(spec.destination)) {
            throw std::out_of_range("Simulator: train refers to an unknown block");
        }
        if (interlocking_.reserve(id, spec.start) != MoveResult::Granted ||
            interlocking_.enter(id, spec.start) != MoveResult::Granted) {
            throw std::invalid_argument("Simulator: cannot place " + train_label(id) +
                                        " on its start block");
        }
        trains_.emplace_back(Train(id, config.train), spec);
    }
}

bool Simulator::step() {
    if (finished()) {
        return false;
    }

    logger_.set_tick(tick_);
    for (TrainState& state : trains_) {
        if (!state.arrived) {
            update_authority(state);
            move(state);
        }
    }
    check_safety();
    ++tick_;

    if (tick_ >= config_.max_ticks && !finished()) {
        timed_out_ = true;
        for (const TrainState& state : trains_) {
            if (!state.arrived) {
                log(EventType::Timeout, state, "did_not_arrive");
            }
        }
    }
    return !finished();
}

SimulationResult Simulator::run() {
    while (step()) {
    }
    return result();
}

SimulationResult Simulator::result() const {
    SimulationResult result;
    result.ticks = tick_;
    result.trains = trains_.size();
    result.arrived = static_cast<std::size_t>(
        std::count_if(trains_.begin(), trains_.end(),
                      [](const TrainState& state) { return state.arrived; }));
    result.safety_violations = safety_violations_;
    return result;
}

// Makes sure the train holds the next block of its route, if it can get it.
void Simulator::update_authority(TrainState& state) {
    const TrainId id = state.train.id();

    // A refusal is remembered only while the other train still holds the block.
    const std::optional<BlockId> refused = state.refused;
    if (refused.has_value() && !interlocking_.holder(*refused).has_value()) {
        state.refused.reset();
    }

    // Give back a block that was closed after this train reserved it.
    const std::optional<BlockId> held = state.next;
    if (held.has_value() && graph_.block(*held).status != BlockStatus::InService) {
        (void)interlocking_.release(id, *held);
        state.next.reset();
    }

    // Once the next block is held the route is locked up to it: replanning
    // under a moving train could take away the track it needs to stop on.
    if (state.next.has_value() || state.current == state.spec.destination) {
        return;
    }

    const std::optional<Route> route =
        find_route(graph_, state.current, state.spec.destination,
                   [this, &state](BlockId block) { return is_blocked_for(state, block); });
    if (!route.has_value()) {
        if (!state.no_route) {
            state.no_route = true;
            log(EventType::NoRoute, state, "no_usable_route_to_destination");
        }
        return;
    }
    state.no_route = false;

    if (route->blocks != state.route) {
        state.route = route->blocks;
        log(EventType::RoutePlanned, state, route_text(state.route));
    }

    // blocks[0] is the current block; there is a blocks[1] because the train
    // is not yet in its destination block.
    const BlockId candidate = route->blocks.at(1);
    if (state.refused == candidate) {
        return;  // already refused and still held: asking again would only repeat the log entry
    }
    if (interlocking_.reserve(id, candidate) == MoveResult::Granted) {
        state.next = candidate;
    } else {
        state.refused = candidate;
    }
}

void Simulator::move(TrainState& state) {
    const double length = graph_.block(state.current).length_m;
    const std::optional<BlockId> next = state.next;

    // Movement authority: to the end of the current block, plus the block ahead if held.
    double authority = length - state.offset_m;
    if (next.has_value()) {
        authority += graph_.block(*next).length_m;
    }

    const bool was_braking = state.train.emergency_brake_active();
    // Inside its destination block the train's only job is to stop.
    const bool at_destination = state.current == state.spec.destination;
    const double target_speed = at_destination ? 0.0 : state.spec.cruise_speed_mps;
    const StepResult moved = state.train.step(target_speed, authority, config_.dt_s);
    if (moved.emergency_brake && !was_braking) {
        log(EventType::EmergencyBrake, state, "too_close_to_end_of_authority");
    }

    state.offset_m += moved.distance_m;
    if (state.offset_m >= length && moved.distance_m > 0.0) {
        if (next.has_value() &&
            interlocking_.enter(state.train.id(), *next) == MoveResult::Granted) {
            state.offset_m -= length;
            state.current = *next;
            state.next.reset();
            if (!state.route.empty()) {
                state.route.erase(state.route.begin());
            }
        } else {
            // The train reached the end of its block without permission to
            // go further. It is held at the boundary and the event is counted.
            state.offset_m = length;
            ++safety_violations_;
            log(EventType::SafetyViolation, state, "passed_end_of_authority");
        }
    }

    if (state.current == state.spec.destination && state.train.speed_mps() < kArrivalSpeedMps) {
        state.arrived = true;
        log(EventType::Arrived, state);
    }
}

// Independent of the interlocking: uses only where the simulator itself
// believes each train is.
void Simulator::check_safety() {
    std::vector<std::size_t> trains_in_block(graph_.block_count(), 0);
    for (const TrainState& state : trains_) {
        if (++trains_in_block[state.current.value] == 2) {
            ++safety_violations_;
            log(EventType::SafetyViolation, state, "two_trains_in_block");
        }
    }
}

// The destination is never treated as blocked, so a train approaches an
// occupied destination and waits outside it.
bool Simulator::is_blocked_for(const TrainState& state, BlockId block) const {
    if (block == state.spec.destination) {
        return false;
    }
    if (state.refused == block) {
        return true;
    }
    return interlocking_.is_occupied(block) && interlocking_.holder(block) != state.train.id();
}

bool Simulator::finished() const {
    if (timed_out_) {
        return true;
    }
    return std::all_of(trains_.begin(), trains_.end(),
                       [](const TrainState& state) { return state.arrived; });
}

std::string Simulator::route_text(const std::vector<BlockId>& route) const {
    std::string text;
    for (const BlockId id : route) {
        if (!text.empty()) {
            text += '>';
        }
        text += graph_.block(id).name;
    }
    return text;
}

void Simulator::log(EventType type, const TrainState& state, std::string reason) {
    logger_.log(type, train_label(state.train.id()), graph_.block(state.current).name,
                std::move(reason));
}

}  // namespace railsim
