#include "railsim/train.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace railsim {
namespace {

bool is_positive(double value) {
    return std::isfinite(value) && value > 0.0;
}

const TrainConfig& validated(const TrainConfig& config) {
    if (!is_positive(config.max_speed_mps) || !is_positive(config.max_acceleration_mps2) ||
        !is_positive(config.service_brake_mps2) || !is_positive(config.emergency_brake_mps2) ||
        !is_positive(config.safety_margin_m)) {
        throw std::invalid_argument("Train: every limit must be a positive finite number");
    }
    if (config.emergency_brake_mps2 < config.service_brake_mps2) {
        throw std::invalid_argument("Train: emergency brake must not be weaker than service brake");
    }
    return config;
}

}  // namespace

Train::Train(TrainId id, const TrainConfig& config)
    : id_(id),
      config_(validated(config)),
      controller_(config.gains, -config.service_brake_mps2, config.max_acceleration_mps2) {}

StepResult Train::step(double target_speed_mps, double distance_to_danger_m, double dt_s) {
    if (!std::isfinite(dt_s) || dt_s <= 0.0) {
        throw std::invalid_argument("Train: dt_s must be positive");
    }

    // Protection layer. Written as "not clear" so that a NaN distance, for
    // which every comparison is false, counts as danger.
    const double emergency_stop_distance =
        speed_mps_ * speed_mps_ / (2.0 * config_.emergency_brake_mps2);
    const bool clear = distance_to_danger_m > emergency_stop_distance + config_.safety_margin_m;
    if (speed_mps_ <= 0.0) {
        emergency_brake_ = false;  // latched until standstill
    } else if (!clear) {
        emergency_brake_ = true;
    }

    double acceleration = 0.0;
    if (emergency_brake_) {
        acceleration = -config_.emergency_brake_mps2;
        controller_.reset();  // do not let the integral grow while the PID is overridden
    } else {
        const double wanted = std::clamp(target_speed_mps, 0.0, config_.max_speed_mps);
        const double target = std::min(wanted, braking_curve_speed(distance_to_danger_m));
        acceleration = controller_.update(target, speed_mps_, dt_s);
    }

    const double new_speed =
        std::clamp(speed_mps_ + acceleration * dt_s, 0.0, config_.max_speed_mps);
    const double distance = 0.5 * (speed_mps_ + new_speed) * dt_s;
    speed_mps_ = new_speed;
    return StepResult{distance, emergency_brake_};
}

// Highest speed from which the train can still stop two safety margins before
// the danger point at half the service brake rate. Planning with half the
// rate leaves the PID room to follow the curve without saturating.
double Train::braking_curve_speed(double distance_to_danger_m) const {
    const double room = distance_to_danger_m - 2.0 * config_.safety_margin_m;
    if (!(room > 0.0)) {
        return 0.0;
    }
    return std::sqrt(config_.service_brake_mps2 * room);
}

}  // namespace railsim
