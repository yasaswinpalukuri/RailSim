#include "railsim/event_logger.hpp"

#include <sstream>

#include <gtest/gtest.h>

namespace railsim {
namespace {

TEST(EventLoggerTest, StartsEmptyAtTickZero) {
    const EventLogger logger;
    EXPECT_TRUE(logger.events().empty());
    EXPECT_EQ(logger.tick(), 0U);
}

TEST(EventLoggerTest, RecordsEventsInOrderWithCurrentTick) {
    EventLogger logger;
    logger.log(EventType::Reserved, "T1", "A");
    logger.set_tick(7);
    logger.log(EventType::EnterRejected, "T2", "A", "occupied_by_other_train");

    ASSERT_EQ(logger.events().size(), 2U);

    const Event& first = logger.events()[0];
    EXPECT_EQ(first.tick, 0U);
    EXPECT_EQ(first.train, "T1");
    EXPECT_EQ(first.block, "A");
    EXPECT_EQ(first.type, EventType::Reserved);
    EXPECT_TRUE(first.reason.empty());

    const Event& second = logger.events()[1];
    EXPECT_EQ(second.tick, 7U);
    EXPECT_EQ(second.type, EventType::EnterRejected);
    EXPECT_EQ(second.reason, "occupied_by_other_train");
}

TEST(EventLoggerTest, WritesCsvHeaderAndOneLinePerEvent) {
    std::ostringstream sink;
    EventLogger logger(&sink);
    logger.set_tick(3);
    logger.log(EventType::Entered, "T1", "B");
    logger.log(EventType::ReserveRejected, "T2", "B", "occupied_by_other_train");

    EXPECT_EQ(sink.str(), "tick,train,block,event,reason\n"
                          "3,T1,B,entered,\n"
                          "3,T2,B,reserve_rejected,occupied_by_other_train\n");
}

TEST(EventLoggerTest, EveryEventTypeHasAName) {
    EXPECT_EQ(to_string(EventType::Reserved), "reserved");
    EXPECT_EQ(to_string(EventType::ReserveRejected), "reserve_rejected");
    EXPECT_EQ(to_string(EventType::Entered), "entered");
    EXPECT_EQ(to_string(EventType::EnterRejected), "enter_rejected");
    EXPECT_EQ(to_string(EventType::Left), "left");
    EXPECT_EQ(to_string(EventType::Released), "released");
    EXPECT_EQ(to_string(EventType::ReleaseRejected), "release_rejected");
    EXPECT_EQ(to_string(EventType::RoutePlanned), "route_planned");
    EXPECT_EQ(to_string(EventType::NoRoute), "no_route");
    EXPECT_EQ(to_string(EventType::EmergencyBrake), "emergency_brake");
    EXPECT_EQ(to_string(EventType::Arrived), "arrived");
    EXPECT_EQ(to_string(EventType::Timeout), "timeout");
    EXPECT_EQ(to_string(EventType::SafetyViolation), "safety_violation");
}

}  // namespace
}  // namespace railsim
