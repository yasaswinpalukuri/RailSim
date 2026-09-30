#pragma once

#include "railsim/pid_controller.hpp"
#include "railsim/types.hpp"

namespace railsim {

struct TrainConfig {
    double max_speed_mps{20.0};
    double max_acceleration_mps2{1.0};
    double service_brake_mps2{1.0};    // strongest braking the speed controller may ask for
    double emergency_brake_mps2{2.5};  // fixed deceleration of the emergency brake
    double safety_margin_m{20.0};      // distance to keep clear in front of a danger point
    PidGains gains{0.8, 0.2, 0.0};
};

struct StepResult {
    double distance_m{};           // distance travelled during this step, never negative
    bool emergency_brake{false};   // true if the emergency brake was applied during this step
};

// Speed and braking of one train. It knows nothing about the track: the
// caller says how far ahead the nearest danger point is (an occupied block or
// the end of the blocks reserved for this train) and gets back how far the
// train moved.
//
// Two independent layers:
//   1. Speed control: a PID tracks the target speed, which is capped by a
//      braking curve so the train stops two safety margins before the danger
//      point using only the service brake.
//   2. Protection: if the train is moving and can no longer stop one safety
//      margin before the danger point even with the emergency brake, the
//      emergency brake takes over and stays on until the train is at rest.
class Train {
public:
    // Throws std::invalid_argument if any limit is not a positive finite
    // number, or the emergency brake is weaker than the service brake.
    Train(TrainId id, const TrainConfig& config);

    // Advances the train by dt_s seconds. Use infinity for
    // `distance_to_danger_m` when the line ahead is clear. A NaN distance is
    // treated as danger. Throws std::invalid_argument if dt_s is not a
    // positive finite number.
    StepResult step(double target_speed_mps, double distance_to_danger_m, double dt_s);

    [[nodiscard]] TrainId id() const { return id_; }
    [[nodiscard]] double speed_mps() const { return speed_mps_; }
    [[nodiscard]] bool emergency_brake_active() const { return emergency_brake_; }

private:
    [[nodiscard]] double braking_curve_speed(double distance_to_danger_m) const;

    TrainId id_;
    TrainConfig config_;
    PidController controller_;
    double speed_mps_{0.0};
    bool emergency_brake_{false};
};

}  // namespace railsim
