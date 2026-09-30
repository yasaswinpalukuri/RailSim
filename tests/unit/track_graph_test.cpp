#include "railsim/track_graph.hpp"

#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace railsim {
namespace {

TEST(TrackGraphTest, StartsEmpty) {
    const TrackGraph graph;
    EXPECT_EQ(graph.block_count(), 0U);
    EXPECT_EQ(graph.link_count(), 0U);
    EXPECT_FALSE(graph.contains(BlockId{0}));
}

TEST(TrackGraphTest, AddBlockAssignsSequentialIds) {
    TrackGraph graph;
    const auto a = graph.add_block("A", 100.0);
    const auto b = graph.add_block("B", 250.0, BlockStatus::OutOfService);

    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(b.has_value());
    EXPECT_EQ(a->value, 0U);
    EXPECT_EQ(b->value, 1U);
    EXPECT_EQ(graph.block(*b).name, "B");
    EXPECT_DOUBLE_EQ(graph.block(*b).length_m, 250.0);
    EXPECT_EQ(graph.block(*a).status, BlockStatus::InService);
    EXPECT_EQ(graph.block(*b).status, BlockStatus::OutOfService);
}

TEST(TrackGraphTest, AddBlockRejectsDuplicateAndEmptyNames) {
    TrackGraph graph;
    ASSERT_TRUE(graph.add_block("A", 100.0).has_value());

    EXPECT_FALSE(graph.add_block("A", 50.0).has_value());
    EXPECT_FALSE(graph.add_block("", 50.0).has_value());
    EXPECT_EQ(graph.block_count(), 1U);
    EXPECT_DOUBLE_EQ(graph.block(BlockId{0}).length_m, 100.0);
}

TEST(TrackGraphTest, AddBlockRejectsInvalidLengths) {
    TrackGraph graph;
    EXPECT_FALSE(graph.add_block("zero", 0.0).has_value());
    EXPECT_FALSE(graph.add_block("negative", -5.0).has_value());
    EXPECT_FALSE(graph.add_block("nan", std::numeric_limits<double>::quiet_NaN()).has_value());
    EXPECT_FALSE(graph.add_block("inf", std::numeric_limits<double>::infinity()).has_value());
    EXPECT_EQ(graph.block_count(), 0U);
}

TEST(TrackGraphTest, LinkIsVisibleFromBothEnds) {
    TrackGraph graph;
    const BlockId a = *graph.add_block("A", 100.0);
    const BlockId b = *graph.add_block("B", 100.0);

    ASSERT_TRUE(graph.add_link(a, b));

    ASSERT_EQ(graph.neighbours(a).size(), 1U);
    ASSERT_EQ(graph.neighbours(b).size(), 1U);
    EXPECT_EQ(graph.neighbours(a)[0], b);
    EXPECT_EQ(graph.neighbours(b)[0], a);
    EXPECT_EQ(graph.link_count(), 1U);
}

TEST(TrackGraphTest, AddLinkRejectsSelfDuplicateAndUnknown) {
    TrackGraph graph;
    const BlockId a = *graph.add_block("A", 100.0);
    const BlockId b = *graph.add_block("B", 100.0);
    ASSERT_TRUE(graph.add_link(a, b));

    EXPECT_FALSE(graph.add_link(a, a));
    EXPECT_FALSE(graph.add_link(a, b));
    EXPECT_FALSE(graph.add_link(b, a));
    EXPECT_FALSE(graph.add_link(a, BlockId{99}));
    EXPECT_EQ(graph.link_count(), 1U);
    EXPECT_EQ(graph.neighbours(a).size(), 1U);
}

TEST(TrackGraphTest, FindBlockByName) {
    TrackGraph graph;
    const BlockId a = *graph.add_block("A", 100.0);

    ASSERT_TRUE(graph.find_block("A").has_value());
    EXPECT_EQ(*graph.find_block("A"), a);
    EXPECT_FALSE(graph.find_block("missing").has_value());
}

TEST(TrackGraphTest, StationsMapToBlocks) {
    TrackGraph graph;
    const BlockId a = *graph.add_block("A", 100.0);

    ASSERT_TRUE(graph.add_station("Central", a));
    EXPECT_FALSE(graph.add_station("Central", a));
    EXPECT_FALSE(graph.add_station("", a));
    EXPECT_FALSE(graph.add_station("Nowhere", BlockId{99}));

    ASSERT_TRUE(graph.find_station("Central").has_value());
    EXPECT_EQ(*graph.find_station("Central"), a);
    EXPECT_FALSE(graph.find_station("Nowhere").has_value());
    EXPECT_EQ(graph.station_count(), 1U);
}

TEST(TrackGraphTest, SetStatusChangesBlock) {
    TrackGraph graph;
    const BlockId a = *graph.add_block("A", 100.0);

    graph.set_status(a, BlockStatus::OutOfService);
    EXPECT_EQ(graph.block(a).status, BlockStatus::OutOfService);
}

TEST(TrackGraphTest, UnknownIdThrows) {
    TrackGraph graph;
    EXPECT_THROW((void)graph.block(BlockId{0}), std::out_of_range);
    EXPECT_THROW((void)graph.neighbours(BlockId{0}), std::out_of_range);
    EXPECT_THROW(graph.set_status(BlockId{0}, BlockStatus::OutOfService), std::out_of_range);
}

}  // namespace
}  // namespace railsim
