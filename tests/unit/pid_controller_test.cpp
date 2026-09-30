#include "railsim/pid_controller.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace railsim {
namespace {

constexpr double kWide = 1000.0;  // output limit large enough never to clamp

TEST(PidControllerTest, ProportionalTermScalesTheError) {
    PidController pid(PidGains{2.0, 0.0, 0.0}, -kWide, kWide);

    EXPECT_DOUBLE_EQ(pid.update(10.0, 4.0, 0.1), 12.0);
    EXPECT_DOUBLE_EQ(pid.update(10.0, 13.0, 0.1), -6.0);
}

TEST(PidControllerTest, IntegralTermAccumulatesErrorOverTime) {
    PidController pid(PidGains{0.0, 0.5, 0.0}, -kWide, kWide);

    // Constant error of 4 for three steps of 0.5 s: integral = 2, 4, 6.
    EXPECT_DOUBLE_EQ(pid.update(4.0, 0.0, 0.5), 1.0);
    EXPECT_DOUBLE_EQ(pid.update(4.0, 0.0, 0.5), 2.0);
    EXPECT_DOUBLE_EQ(pid.update(4.0, 0.0, 0.5), 3.0);
}

TEST(PidControllerTest, DerivativeTermOpposesARisingMeasurement) {
    PidController pid(PidGains{0.0, 0.0, 1.0}, -kWide, kWide);

    EXPECT_DOUBLE_EQ(pid.update(0.0, 1.0, 0.5), 0.0);  // no previous measurement yet
    EXPECT_DOUBLE_EQ(pid.update(0.0, 2.0, 0.5), -2.0);
    EXPECT_DOUBLE_EQ(pid.update(0.0, 2.0, 0.5), 0.0);
}

TEST(PidControllerTest, SetpointJumpCausesNoDerivativeKick) {
    PidController pid(PidGains{0.0, 0.0, 5.0}, -kWide, kWide);
    (void)pid.update(0.0, 3.0, 0.1);

    EXPECT_DOUBLE_EQ(pid.update(100.0, 3.0, 0.1), 0.0);
}

TEST(PidControllerTest, OutputIsClampedToLimits) {
    PidController pid(PidGains{1.0, 0.0, 0.0}, -1.0, 2.0);

    EXPECT_DOUBLE_EQ(pid.update(100.0, 0.0, 0.1), 2.0);
    EXPECT_DOUBLE_EQ(pid.update(-100.0, 0.0, 0.1), -1.0);
}

TEST(PidControllerTest, IntegralDoesNotWindUpWhileSaturated) {
    PidController pid(PidGains{1.0, 1.0, 0.0}, -1.0, 1.0);

    // A long period with a large error keeps the output pinned at the limit.
    for (int i = 0; i < 1000; ++i) {
        EXPECT_DOUBLE_EQ(pid.update(10.0, 0.0, 0.1), 1.0);
    }

    // As soon as the error changes sign the output must follow. With windup,
    // the stored integral (about 1000) would hold it at +1 for a long time.
    EXPECT_TRUE(pid.update(0.0, 0.5, 0.1) < 0.0);
}

TEST(PidControllerTest, ResetForgetsIntegralAndPreviousMeasurement) {
    PidController pid(PidGains{0.0, 1.0, 1.0}, -kWide, kWide);
    (void)pid.update(5.0, 0.0, 1.0);
    (void)pid.update(5.0, 1.0, 1.0);

    pid.reset();

    // Integral restarts from zero and there is no derivative on the first call.
    EXPECT_DOUBLE_EQ(pid.update(2.0, 0.0, 1.0), 2.0);
}

TEST(PidControllerTest, RejectsInvalidConstruction) {
    EXPECT_THROW(PidController(PidGains{-1.0, 0.0, 0.0}, -1.0, 1.0), std::invalid_argument);
    EXPECT_THROW(PidController(PidGains{0.0, std::nan(""), 0.0}, -1.0, 1.0), std::invalid_argument);
    EXPECT_THROW(PidController(PidGains{1.0, 0.0, 0.0}, 1.0, 1.0), std::invalid_argument);
    EXPECT_THROW(PidController(PidGains{1.0, 0.0, 0.0}, 2.0, 1.0), std::invalid_argument);
}

TEST(PidControllerTest, RejectsInvalidTimeStep) {
    PidController pid(PidGains{1.0, 0.0, 0.0}, -1.0, 1.0);

    EXPECT_THROW((void)pid.update(1.0, 0.0, 0.0), std::invalid_argument);
    EXPECT_THROW((void)pid.update(1.0, 0.0, -0.1), std::invalid_argument);
    EXPECT_THROW((void)pid.update(1.0, 0.0, std::numeric_limits<double>::infinity()),
                 std::invalid_argument);
}

TEST(PidControllerTest, DrivesASimplePlantToTheSetpoint) {
    // Plant: the output is an acceleration, the measurement is a speed.
    PidController pid(PidGains{0.8, 0.2, 0.0}, -1.0, 1.0);
    const double dt = 0.1;
    double speed = 0.0;
    double highest = 0.0;

    for (int i = 0; i < 1200; ++i) {
        speed += pid.update(15.0, speed, dt) * dt;
        highest = std::fmax(highest, speed);
    }

    EXPECT_TRUE(std::fabs(speed - 15.0) < 0.05);
    EXPECT_TRUE(highest < 15.5);  // little overshoot thanks to anti-windup
}

}  // namespace
}  // namespace railsim
