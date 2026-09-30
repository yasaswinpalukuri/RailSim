#include <cstddef>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "railsim/config_loader.hpp"
#include "railsim/event_logger.hpp"
#include "railsim/simulator.hpp"

namespace railsim {
namespace {

// Replays a log and returns the tick of the first moment two trains were in
// the same block, or -1 if that never happened. It uses nothing but the
// "entered" and "left" events, so it checks the safety rule independently of
// the interlocking and the simulator.
long long first_collision_tick(const std::vector<Event>& events) {
    std::map<std::string, std::set<std::string>> trains_in_block;
    for (const Event& event : events) {
        if (event.type == EventType::Entered) {
            std::set<std::string>& trains = trains_in_block[event.block];
            trains.insert(event.train);
            if (trains.size() > 1) {
                return static_cast<long long>(event.tick);
            }
        } else if (event.type == EventType::Left) {
            trains_in_block[event.block].erase(event.train);
        }
    }
    return -1;
}

std::size_t count_events(const std::vector<Event>& events, EventType type) {
    std::size_t count = 0;
    for (const Event& event : events) {
        if (event.type == type) {
            ++count;
        }
    }
    return count;
}

bool has_event(const std::vector<Event>& events, EventType type, const std::string& train,
               const std::string& block, const std::string& reason) {
    for (const Event& event : events) {
        if (event.type == type && event.train == train && event.block == block &&
            event.reason == reason) {
            return true;
        }
    }
    return false;
}

LoadResult load_scenario(const std::string& name) {
    return load_track_file(std::string(RAILSIM_SCENARIO_DIR) + "/" + name);
}

// Single line with no passing place: A --- B --- C --- D.
TrackGraph single_line() {
    TrackGraph graph;
    const BlockId a = *graph.add_block("A", 300.0);
    const BlockId b = *graph.add_block("B", 300.0);
    const BlockId c = *graph.add_block("C", 300.0);
    const BlockId d = *graph.add_block("D", 300.0);
    EXPECT_TRUE(graph.add_link(a, b));
    EXPECT_TRUE(graph.add_link(b, c));
    EXPECT_TRUE(graph.add_link(c, d));
    return graph;
}

TEST(LogReplayTest, DetectsTwoTrainsInOneBlock) {
    const std::vector<Event> safe{
        Event{0, "T1", "A", EventType::Entered, ""},
        Event{5, "T1", "A", EventType::Left, ""},
        Event{5, "T2", "A", EventType::Entered, ""},
    };
    EXPECT_EQ(first_collision_tick(safe), -1);

    const std::vector<Event> unsafe{
        Event{0, "T1", "A", EventType::Entered, ""},
        Event{7, "T2", "A", EventType::Entered, ""},
    };
    EXPECT_EQ(first_collision_tick(unsafe), 7);
}

TEST(ScenarioTest, SingleTrainRunsFromStartToDestination) {
    const TrackGraph graph = single_line();
    EventLogger logger;
    Simulator simulator(graph, {TrainSpec{BlockId{0}, BlockId{3}, 15.0}}, logger);

    const SimulationResult result = simulator.run();

    EXPECT_TRUE(result.all_arrived());
    EXPECT_EQ(result.safety_violations, 0U);
    EXPECT_EQ(simulator.block_of(0), BlockId{3});
    EXPECT_TRUE(simulator.speed_of(0) < 0.1);
    EXPECT_TRUE(has_event(logger.events(), EventType::RoutePlanned, "T1", "A", "A>B>C>D"));
    EXPECT_TRUE(has_event(logger.events(), EventType::Arrived, "T1", "D", ""));
    EXPECT_EQ(count_events(logger.events(), EventType::EmergencyBrake), 0U);
    EXPECT_EQ(count_events(logger.events(), EventType::Entered), 4U);
}

TEST(ScenarioTest, TrainAlreadyAtDestinationArrivesImmediately) {
    const TrackGraph graph = single_line();
    EventLogger logger;
    Simulator simulator(graph, {TrainSpec{BlockId{1}, BlockId{1}, 15.0}}, logger);

    const SimulationResult result = simulator.run();

    EXPECT_TRUE(result.all_arrived());
    EXPECT_EQ(result.ticks, 1U);
}

TEST(ScenarioTest, TwoTrainsCompetingForOneBlock) {
    const LoadResult scenario = load_scenario("two_trains_conflict.txt");
    ASSERT_TRUE(scenario.ok());
    EventLogger logger;
    Simulator simulator(*scenario.graph, scenario.trains, logger);

    const SimulationResult result = simulator.run();

    EXPECT_TRUE(result.all_arrived());
    EXPECT_EQ(result.safety_violations, 0U);
    EXPECT_EQ(first_collision_tick(logger.events()), -1);

    // The losing request is refused and the reason is in the log.
    EXPECT_TRUE(has_event(logger.events(), EventType::ReserveRejected, "T2", "J",
                          "reserved_by_other_train"));
    EXPECT_EQ(count_events(logger.events(), EventType::ReserveRejected), 1U);
    EXPECT_TRUE(has_event(logger.events(), EventType::Arrived, "T1", "E1", ""));
    EXPECT_TRUE(has_event(logger.events(), EventType::Arrived, "T2", "E2", ""));
}

TEST(ScenarioTest, OpposingTrainsPassUsingTheLoop) {
    const LoadResult scenario = load_scenario("passing_loop.txt");
    ASSERT_TRUE(scenario.ok());
    EventLogger logger;
    Simulator simulator(*scenario.graph, scenario.trains, logger);

    const SimulationResult result = simulator.run();

    EXPECT_TRUE(result.all_arrived());
    EXPECT_EQ(result.safety_violations, 0U);
    EXPECT_EQ(first_collision_tick(logger.events()), -1);
    EXPECT_TRUE(has_event(logger.events(), EventType::Arrived, "T1", "E", ""));
    EXPECT_TRUE(has_event(logger.events(), EventType::Arrived, "T2", "A", ""));
}

TEST(ScenarioTest, HeadOnTrainsOnSingleLineDeadlockButStaySafe) {
    const TrackGraph graph = single_line();
    EventLogger logger;
    SimulationConfig config;
    config.max_ticks = 600;
    Simulator simulator(graph,
                        {TrainSpec{BlockId{0}, BlockId{3}, 15.0},
                         TrainSpec{BlockId{3}, BlockId{0}, 15.0}},
                        logger, config);

    const SimulationResult result = simulator.run();

    EXPECT_EQ(result.arrived, 0U);
    EXPECT_EQ(result.ticks, 600U);
    EXPECT_EQ(result.safety_violations, 0U);
    EXPECT_EQ(first_collision_tick(logger.events()), -1);
    EXPECT_EQ(count_events(logger.events(), EventType::Timeout), 2U);
    EXPECT_EQ(count_events(logger.events(), EventType::SafetyViolation), 0U);
}

TEST(ScenarioTest, SafetyRuleHoldsOnEveryTickWithManyTrains) {
    // Ring of eight blocks with a chord; four trains cross each other's paths.
    TrackGraph graph;
    std::vector<BlockId> ring;
    for (std::size_t i = 0; i < 8; ++i) {
        const double length = 200.0 + 25.0 * static_cast<double>(i);
        ring.push_back(*graph.add_block("R" + std::to_string(i), length));
    }
    for (std::size_t i = 0; i < 8; ++i) {
        ASSERT_TRUE(graph.add_link(ring[i], ring[(i + 1) % 8]));
    }
    ASSERT_TRUE(graph.add_link(ring[1], ring[5]));

    const std::vector<TrainSpec> trains{
        TrainSpec{ring[1], ring[4], 18.0},
        TrainSpec{ring[0], ring[3], 12.0},
        TrainSpec{ring[6], ring[7], 15.0},
        TrainSpec{ring[5], ring[2], 20.0},
    };
    EventLogger logger;
    Simulator simulator(graph, trains, logger);

    bool running = true;
    while (running) {
        running = simulator.step();

        std::set<std::size_t> occupied;
        for (std::size_t i = 0; i < simulator.train_count(); ++i) {
            const BlockId block = simulator.block_of(i);
            ASSERT_TRUE(occupied.insert(block.value).second);
            // The interlocking agrees with where the simulator thinks the train is.
            ASSERT_EQ(simulator.interlocking().position(TrainId{i + 1}), block);
            ASSERT_EQ(simulator.interlocking().holder(block), TrainId{i + 1});
        }
    }

    EXPECT_EQ(simulator.result().safety_violations, 0U);
    EXPECT_EQ(first_collision_tick(logger.events()), -1);
    EXPECT_TRUE(simulator.result().all_arrived());
    // The trains really did get in each other's way.
    EXPECT_TRUE(count_events(logger.events(), EventType::ReserveRejected) >= 1U);
}

TEST(ScenarioTest, TrainStopsShortOfABlockClosedAhead) {
    TrackGraph graph = single_line();
    const BlockId b{1};
    EventLogger logger;
    SimulationConfig config;
    config.max_ticks = 400;
    Simulator simulator(graph, {TrainSpec{BlockId{0}, BlockId{3}, 15.0}}, logger, config);

    // Let the train reserve B and start moving, then close B in front of it.
    for (int i = 0; i < 10; ++i) {
        ASSERT_TRUE(simulator.step());
    }
    ASSERT_EQ(simulator.interlocking().holder(b), TrainId{1});
    graph.set_status(b, BlockStatus::OutOfService);

    const SimulationResult result = simulator.run();

    EXPECT_EQ(result.arrived, 0U);
    EXPECT_EQ(result.safety_violations, 0U);
    EXPECT_EQ(simulator.block_of(0), BlockId{0});
    EXPECT_TRUE(simulator.speed_of(0) < 0.1);
    EXPECT_FALSE(simulator.interlocking().holder(b).has_value());
    EXPECT_TRUE(has_event(logger.events(), EventType::Released, "T1", "B", ""));
    EXPECT_TRUE(has_event(logger.events(), EventType::NoRoute, "T1", "A",
                          "no_usable_route_to_destination"));
}

TEST(ScenarioTest, SameInputGivesTheSameLog) {
    std::string logs[2];
    for (std::string& log : logs) {
        const LoadResult scenario = load_scenario("passing_loop.txt");
        ASSERT_TRUE(scenario.ok());
        std::ostringstream sink;
        EventLogger logger(&sink);
        Simulator simulator(*scenario.graph, scenario.trains, logger);
        (void)simulator.run();
        log = sink.str();
    }

    EXPECT_FALSE(logs[0].empty());
    EXPECT_EQ(logs[0], logs[1]);
}

TEST(ScenarioTest, RejectsInvalidSetups) {
    const TrackGraph graph = single_line();
    EventLogger logger;
    const TrainSpec spec{BlockId{0}, BlockId{3}, 15.0};

    // Two trains on the same start block.
    EXPECT_THROW(Simulator(graph, {spec, spec}, logger), std::invalid_argument);

    EXPECT_THROW(Simulator(graph, {TrainSpec{BlockId{0}, BlockId{99}, 15.0}}, logger),
                 std::out_of_range);

    SimulationConfig bad_step;
    bad_step.dt_s = 0.0;
    EXPECT_THROW(Simulator(graph, {spec}, logger, bad_step), std::invalid_argument);

    TrackGraph short_block;
    ASSERT_TRUE(short_block.add_block("Tiny", 30.0).has_value());
    EXPECT_THROW(Simulator(short_block, {}, logger), std::invalid_argument);
}

}  // namespace
}  // namespace railsim
