#include "railsim/train.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace railsim {
namespace {

constexpr double kClear = std::numeric_limits<double>::infinity();
constexpr double kDt = 0.1;

// Runs the train on a clear line until `seconds` have passed.
void cruise(Train& train, double target_speed, double seconds) {
    const int steps = static_cast<int>(std::lround(seconds / kDt));
    for (int i = 0; i < steps; ++i) {
        (void)train.step(target_speed, kClear, kDt);
    }
}

TEST(TrainTest, StartsAtRest) {
    const Train train(TrainId{1}, TrainConfig{});

    EXPECT_EQ(train.id(), TrainId{1});
    EXPECT_DOUBLE_EQ(train.speed_mps(), 0.0);
    EXPECT_FALSE(train.emergency_brake_active());
}

TEST(TrainTest, ReachesAndHoldsTargetSpeed) {
    Train train(TrainId{1}, TrainConfig{});

    cruise(train, 15.0, 60.0);

    EXPECT_TRUE(std::fabs(train.speed_mps() - 15.0) < 0.1);
}

TEST(TrainTest, AccelerationAndSpeedStayWithinLimits) {
    const TrainConfig config;
    Train train(TrainId{1}, config);
    double distance = 0.0;

    for (int i = 0; i < 1000; ++i) {
        const double before = train.speed_mps();
        const StepResult result = train.step(1000.0, kClear, kDt);  // far above max speed
        const double change = train.speed_mps() - before;

        ASSERT_TRUE(change <= config.max_acceleration_mps2 * kDt + 1e-9);
        ASSERT_TRUE(train.speed_mps() <= config.max_speed_mps);
        ASSERT_TRUE(result.distance_m >= 0.0);
        distance += result.distance_m;
    }

    EXPECT_TRUE(std::fabs(train.speed_mps() - config.max_speed_mps) < 0.1);
    EXPECT_TRUE(distance > 0.0);
}

TEST(TrainTest, SlowsToANewLowerTargetWithServiceBrakeOnly) {
    const TrainConfig config;
    Train train(TrainId{1}, config);
    cruise(train, 20.0, 60.0);

    for (int i = 0; i < 600; ++i) {
        const double before = train.speed_mps();
        const StepResult result = train.step(5.0, kClear, kDt);

        ASSERT_FALSE(result.emergency_brake);
        ASSERT_TRUE(before - train.speed_mps() <= config.service_brake_mps2 * kDt + 1e-9);
    }
    EXPECT_TRUE(std::fabs(train.speed_mps() - 5.0) < 0.1);
}

TEST(TrainTest, StopsBeforeDangerPointWithoutEmergencyBrake) {
    const TrainConfig config;
    Train train(TrainId{1}, config);
    const double danger_at = 800.0;
    double travelled = 0.0;

    for (int i = 0; i < 3000; ++i) {
        const StepResult result = train.step(20.0, danger_at - travelled, kDt);
        travelled += result.distance_m;
        ASSERT_FALSE(result.emergency_brake);
    }

    EXPECT_TRUE(train.speed_mps() < 0.01);
    EXPECT_TRUE(travelled > 500.0);  // it did approach, not just sit still
    EXPECT_TRUE(travelled <= danger_at - config.safety_margin_m);
}

TEST(TrainTest, EmergencyBrakeStopsTrainBeforeSuddenObstacle) {
    const TrainConfig config;
    Train train(TrainId{1}, config);
    cruise(train, 20.0, 60.0);
    const double speed = train.speed_mps();

    // An obstacle appears exactly at the emergency threshold for this speed.
    const double danger_at = speed * speed / (2.0 * config.emergency_brake_mps2) +
                             config.safety_margin_m;
    double travelled = 0.0;
    bool braked = false;
    for (int i = 0; i < 300; ++i) {
        const StepResult result = train.step(20.0, danger_at - travelled, kDt);
        travelled += result.distance_m;
        braked = braked || result.emergency_brake;
    }

    EXPECT_TRUE(braked);
    EXPECT_DOUBLE_EQ(train.speed_mps(), 0.0);
    EXPECT_TRUE(travelled < danger_at);
}

TEST(TrainTest, EmergencyBrakeStaysOnUntilStandstill) {
    const TrainConfig config;
    Train train(TrainId{1}, config);
    cruise(train, 20.0, 60.0);

    ASSERT_TRUE(train.step(20.0, 30.0, kDt).emergency_brake);

    // The obstacle disappears, but the brake must not release while moving.
    while (train.speed_mps() > 0.0) {
        const double before = train.speed_mps();
        const StepResult result = train.step(20.0, kClear, kDt);
        ASSERT_TRUE(result.emergency_brake);
        ASSERT_TRUE(train.speed_mps() < before);
    }

    // At rest with a clear line the brake releases and the train moves again.
    const StepResult result = train.step(20.0, kClear, kDt);
    EXPECT_FALSE(result.emergency_brake);
    EXPECT_TRUE(train.speed_mps() > 0.0);
}

TEST(TrainTest, DoesNotStartTowardsACloseDangerPoint) {
    const TrainConfig config;
    Train train(TrainId{1}, config);

    for (int i = 0; i < 100; ++i) {
        const StepResult result = train.step(20.0, config.safety_margin_m, kDt);
        ASSERT_DOUBLE_EQ(result.distance_m, 0.0);
        ASSERT_FALSE(result.emergency_brake);  // nothing to brake: it never moved
    }
}

TEST(TrainTest, UnknownDistanceIsTreatedAsDanger) {
    const double unknown = std::numeric_limits<double>::quiet_NaN();

    Train standing(TrainId{1}, TrainConfig{});
    EXPECT_DOUBLE_EQ(standing.step(20.0, unknown, kDt).distance_m, 0.0);

    Train moving(TrainId{2}, TrainConfig{});
    cruise(moving, 20.0, 60.0);
    EXPECT_TRUE(moving.step(20.0, unknown, kDt).emergency_brake);
}

TEST(TrainTest, NegativeTargetSpeedMeansStop) {
    Train train(TrainId{1}, TrainConfig{});
    cruise(train, 10.0, 30.0);

    cruise(train, -5.0, 60.0);

    EXPECT_DOUBLE_EQ(train.speed_mps(), 0.0);
}

TEST(TrainTest, RejectsInvalidConfigAndTimeStep) {
    TrainConfig zero_speed;
    zero_speed.max_speed_mps = 0.0;
    EXPECT_THROW(Train(TrainId{1}, zero_speed), std::invalid_argument);

    TrainConfig weak_emergency;
    weak_emergency.emergency_brake_mps2 = 0.5;
    EXPECT_THROW(Train(TrainId{1}, weak_emergency), std::invalid_argument);

    Train train(TrainId{1}, TrainConfig{});
    EXPECT_THROW((void)train.step(10.0, kClear, 0.0), std::invalid_argument);
}

}  // namespace
}  // namespace railsim
