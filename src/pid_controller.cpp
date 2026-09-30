#include "railsim/pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace railsim {
namespace {

bool is_valid_gain(double gain) {
    return std::isfinite(gain) && gain >= 0.0;
}

}  // namespace

PidController::PidController(PidGains gains, double output_min, double output_max)
    : gains_(gains), output_min_(output_min), output_max_(output_max) {
    if (!is_valid_gain(gains.kp) || !is_valid_gain(gains.ki) || !is_valid_gain(gains.kd)) {
        throw std::invalid_argument("PidController: gains must be finite and not negative");
    }
    if (!(output_min < output_max)) {
        throw std::invalid_argument("PidController: output_min must be below output_max");
    }
}

double PidController::update(double setpoint, double measurement, double dt_s) {
    if (!std::isfinite(dt_s) || dt_s <= 0.0) {
        throw std::invalid_argument("PidController: dt_s must be positive");
    }

    const double error = setpoint - measurement;

    double derivative = 0.0;
    if (previous_measurement_.has_value()) {
        derivative = -(measurement - *previous_measurement_) / dt_s;
    }
    previous_measurement_ = measurement;

    const double candidate_integral = integral_ + error * dt_s;
    const double unclamped =
        gains_.kp * error + gains_.ki * candidate_integral + gains_.kd * derivative;
    const double output = std::clamp(unclamped, output_min_, output_max_);

    // Anti-windup: keep the new integral only if the actuator could deliver
    // what was asked. While saturated, integrating further would only build
    // up error that has to be unwound later as overshoot.
    const bool saturated = unclamped < output_min_ || unclamped > output_max_;
    if (!saturated) {
        integral_ = candidate_integral;
    }
    return output;
}

void PidController::reset() {
    integral_ = 0.0;
    previous_measurement_.reset();
}

}  // namespace railsim
