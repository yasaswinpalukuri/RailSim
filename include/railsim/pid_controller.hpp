#pragma once

#include <optional>

namespace railsim {

struct PidGains {
    double kp{};  // proportional: reacts to the current error
    double ki{};  // integral: removes the error that remains over time
    double kd{};  // derivative: damps fast changes
};

// Discrete PID controller with a clamped output.
//
//   - Anti-windup: the integral is frozen while the output is saturated.
//   - The derivative acts on the measurement, not the error, so a sudden
//     change of setpoint does not produce an output spike.
//
// Pure arithmetic with no I/O or clock: time is passed in as `dt_s`.
class PidController {
public:
    // Throws std::invalid_argument if a gain is negative or not finite, or
    // if output_min is not below output_max.
    PidController(PidGains gains, double output_min, double output_max);

    // Returns the clamped output for one time step.
    // Throws std::invalid_argument if dt_s is not a positive finite number.
    [[nodiscard]] double update(double setpoint, double measurement, double dt_s);

    // Forgets the integral and the previous measurement.
    void reset();

private:
    PidGains gains_;
    double output_min_;
    double output_max_;
    double integral_{0.0};
    std::optional<double> previous_measurement_;
};

}  // namespace railsim
