#include "railsim/route_planner.hpp"

#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace railsim {
namespace {

// Same layout as scenarios/simple_line.txt:
//
//   A(200) --- B(300) --- C(250) --- E(200)
//              |  \                 /
//         S(100)    ---- D(400) ----
//    (out of service)
class RoutePlannerTest : public ::testing::Test {
protected:
    void SetUp() override {
        a = *graph.add_block("A", 200.0);
        b = *graph.add_block("B", 300.0);
        c = *graph.add_block("C", 250.0);
        d = *graph.add_block("D", 400.0);
        e = *graph.add_block("E", 200.0);
        s = *graph.add_block("S", 100.0, BlockStatus::OutOfService);

        EXPECT_TRUE(graph.add_link(a, b));
        EXPECT_TRUE(graph.add_link(b, c));
        EXPECT_TRUE(graph.add_link(b, d));
        EXPECT_TRUE(graph.add_link(c, e));
        EXPECT_TRUE(graph.add_link(d, e));
        EXPECT_TRUE(graph.add_link(b, s));
    }

    // Route as a string of block names, e.g. "A B C E"; "" when there is no route.
    std::string names(const std::optional<Route>& route) const {
        std::string text;
        if (route) {
            for (const BlockId id : route->blocks) {
                text += (text.empty() ? "" : " ") + graph.block(id).name;
            }
        }
        return text;
    }

    TrackGraph graph;
    BlockId a, b, c, d, e, s;
};

TEST_F(RoutePlannerTest, FindsShortestRoute) {
    const auto route = find_route(graph, a, e);

    ASSERT_TRUE(route.has_value());
    EXPECT_EQ(names(route), "A B C E");
    EXPECT_DOUBLE_EQ(route->length_m, 950.0);
}

TEST_F(RoutePlannerTest, RouteIsSymmetric) {
    const auto route = find_route(graph, e, a);

    ASSERT_TRUE(route.has_value());
    EXPECT_EQ(names(route), "E C B A");
    EXPECT_DOUBLE_EQ(route->length_m, 950.0);
}

TEST_F(RoutePlannerTest, AvoidsBlockedBlock) {
    const auto route = find_route(graph, a, e, [this](BlockId id) { return id == c; });

    ASSERT_TRUE(route.has_value());
    EXPECT_EQ(names(route), "A B D E");
    EXPECT_DOUBLE_EQ(route->length_m, 1100.0);
}

TEST_F(RoutePlannerTest, AvoidsOutOfServiceBlock) {
    graph.set_status(c, BlockStatus::OutOfService);

    EXPECT_EQ(names(find_route(graph, a, e)), "A B D E");
}

TEST_F(RoutePlannerTest, NoRouteWhenEveryPathIsBlocked) {
    const auto route = find_route(graph, a, e, [this](BlockId id) { return id == c || id == d; });

    EXPECT_FALSE(route.has_value());
}

TEST_F(RoutePlannerTest, NoRouteToOutOfServiceDestination) {
    EXPECT_FALSE(find_route(graph, a, s).has_value());
}

TEST_F(RoutePlannerTest, NoRouteToBlockedDestination) {
    EXPECT_FALSE(find_route(graph, a, e, [this](BlockId id) { return id == e; }).has_value());
}

TEST_F(RoutePlannerTest, StartBlockIsNeverFiltered) {
    // The train asking for the route occupies its own start block.
    EXPECT_EQ(names(find_route(graph, a, e, [this](BlockId id) { return id == a; })), "A B C E");

    // A train standing on a block that was taken out of service can still leave.
    EXPECT_EQ(names(find_route(graph, s, a)), "S B A");
}

TEST_F(RoutePlannerTest, RouteToSelfIsSingleBlock) {
    const auto route = find_route(graph, c, c, [](BlockId) { return true; });

    ASSERT_TRUE(route.has_value());
    EXPECT_EQ(names(route), "C");
    EXPECT_DOUBLE_EQ(route->length_m, 250.0);
}

TEST_F(RoutePlannerTest, NoRouteToDisconnectedBlock) {
    const BlockId island = *graph.add_block("Island", 100.0);

    EXPECT_FALSE(find_route(graph, a, island).has_value());
}

TEST_F(RoutePlannerTest, UnknownIdThrows) {
    EXPECT_THROW((void)find_route(graph, a, BlockId{99}), std::out_of_range);
    EXPECT_THROW((void)find_route(graph, BlockId{99}, a), std::out_of_range);
}

TEST(RoutePlannerStandaloneTest, PrefersShorterLengthOverFewerBlocks) {
    TrackGraph graph;
    const BlockId x = *graph.add_block("X", 100.0);
    const BlockId y = *graph.add_block("Y", 100.0);
    const BlockId longer = *graph.add_block("Long", 1000.0);
    const BlockId p = *graph.add_block("P", 100.0);
    const BlockId q = *graph.add_block("Q", 100.0);
    ASSERT_TRUE(graph.add_link(x, longer));
    ASSERT_TRUE(graph.add_link(longer, y));
    ASSERT_TRUE(graph.add_link(x, p));
    ASSERT_TRUE(graph.add_link(p, q));
    ASSERT_TRUE(graph.add_link(q, y));

    const auto route = find_route(graph, x, y);

    ASSERT_TRUE(route.has_value());
    const std::vector<BlockId> expected{x, p, q, y};
    EXPECT_EQ(route->blocks, expected);
    EXPECT_DOUBLE_EQ(route->length_m, 400.0);
}

TEST(RoutePlannerStandaloneTest, EqualRoutesResolveTheSameWayEveryTime) {
    TrackGraph graph;
    const BlockId start = *graph.add_block("Start", 100.0);
    const BlockId upper = *graph.add_block("Upper", 100.0);
    const BlockId lower = *graph.add_block("Lower", 100.0);
    const BlockId end = *graph.add_block("End", 100.0);
    ASSERT_TRUE(graph.add_link(start, lower));
    ASSERT_TRUE(graph.add_link(start, upper));
    ASSERT_TRUE(graph.add_link(lower, end));
    ASSERT_TRUE(graph.add_link(upper, end));

    const std::vector<BlockId> expected{start, upper, end};
    for (int run = 0; run < 5; ++run) {
        const auto route = find_route(graph, start, end);
        ASSERT_TRUE(route.has_value());
        EXPECT_EQ(route->blocks, expected);
    }
}

}  // namespace
}  // namespace railsim
