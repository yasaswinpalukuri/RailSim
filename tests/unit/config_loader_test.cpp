#include "railsim/config_loader.hpp"

#include <sstream>
#include <string>

#include <gtest/gtest.h>

namespace railsim {
namespace {

LoadResult load(const std::string& text) {
    std::istringstream input(text);
    return load_track(input);
}

// Loads text that must fail with exactly one error and returns that error.
ParseError single_error(const std::string& text) {
    const LoadResult result = load(text);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.errors.size(), 1U);
    return result.errors.empty() ? ParseError{} : result.errors[0];
}

bool mentions(const ParseError& error, const std::string& text) {
    return error.message.find(text) != std::string::npos;
}

TEST(ConfigLoaderTest, LoadsBlocksLinksAndStations) {
    const LoadResult result = load("block A 100\n"
                                   "block B 250.5 out_of_service\n"
                                   "link A B\n"
                                   "station Central A\n");

    ASSERT_TRUE(result.ok());
    EXPECT_TRUE(result.errors.empty());

    const TrackGraph& graph = *result.graph;
    EXPECT_EQ(graph.block_count(), 2U);
    EXPECT_EQ(graph.link_count(), 1U);

    const BlockId a = *graph.find_block("A");
    const BlockId b = *graph.find_block("B");
    EXPECT_DOUBLE_EQ(graph.block(b).length_m, 250.5);
    EXPECT_EQ(graph.block(a).status, BlockStatus::InService);
    EXPECT_EQ(graph.block(b).status, BlockStatus::OutOfService);
    ASSERT_EQ(graph.neighbours(a).size(), 1U);
    EXPECT_EQ(graph.neighbours(a)[0], b);
    EXPECT_EQ(*graph.find_station("Central"), a);
}

TEST(ConfigLoaderTest, IgnoresCommentsBlankLinesAndExtraWhitespace) {
    const LoadResult result = load("# header comment\n"
                                   "\n"
                                   "   block   A   100   # trailing comment\n"
                                   "\t\n"
                                   "block B 100\r\n");

    ASSERT_TRUE(result.ok());
    EXPECT_EQ(result.graph->block_count(), 2U);
}

TEST(ConfigLoaderTest, ReportsLineNumberOfError) {
    const ParseError error = single_error("block A 100\n"
                                          "\n"
                                          "signal A\n");
    EXPECT_EQ(error.line, 3U);
    EXPECT_TRUE(mentions(error, "unknown keyword 'signal'"));
}

TEST(ConfigLoaderTest, RejectsInvalidBlockLengths) {
    EXPECT_TRUE(mentions(single_error("block A abc\n"), "positive number"));
    EXPECT_TRUE(mentions(single_error("block A 12x\n"), "positive number"));
    EXPECT_TRUE(mentions(single_error("block A 0\n"), "positive number"));
    EXPECT_TRUE(mentions(single_error("block A -5\n"), "positive number"));
    EXPECT_TRUE(mentions(single_error("block A inf\n"), "positive number"));
}

TEST(ConfigLoaderTest, RejectsMalformedBlockLines) {
    EXPECT_TRUE(mentions(single_error("block A\n"), "expected: block"));
    EXPECT_TRUE(mentions(single_error("block A 100 closed\n"), "unknown block option"));
    EXPECT_TRUE(mentions(single_error("block A 100 out_of_service extra\n"), "expected: block"));
}

TEST(ConfigLoaderTest, RejectsDuplicateBlock) {
    const ParseError error = single_error("block A 100\nblock A 200\n");
    EXPECT_EQ(error.line, 2U);
    EXPECT_TRUE(mentions(error, "duplicate block 'A'"));
}

TEST(ConfigLoaderTest, RejectsBadLinks) {
    EXPECT_TRUE(mentions(single_error("block A 100\nlink A\n"), "expected: link"));
    EXPECT_TRUE(mentions(single_error("block A 100\nlink A B\n"), "unknown block 'B'"));
    EXPECT_TRUE(mentions(single_error("block A 100\nlink A A\n"), "cannot link to itself"));
    EXPECT_TRUE(mentions(single_error("block A 100\nblock B 100\nlink A B\nlink B A\n"),
                         "duplicate link"));
}

TEST(ConfigLoaderTest, LinkBeforeBlockDeclarationIsAnError) {
    const ParseError error = single_error("block A 100\nlink A B\nblock B 100\n");
    EXPECT_EQ(error.line, 2U);
}

TEST(ConfigLoaderTest, RejectsBadStations) {
    EXPECT_TRUE(mentions(single_error("block A 100\nstation Central\n"), "expected: station"));
    EXPECT_TRUE(mentions(single_error("block A 100\nstation Central B\n"), "unknown block 'B'"));
    EXPECT_TRUE(mentions(single_error("block A 100\nstation X A\nstation X A\n"),
                         "duplicate station 'X'"));
}

TEST(ConfigLoaderTest, LoadsTrainsInFileOrder) {
    const LoadResult result = load("block A 100\nblock B 100\nlink A B\n"
                                   "station West A\nstation East B\n"
                                   "train West East 15\n"
                                   "train East West 12.5\n");

    ASSERT_TRUE(result.ok());
    ASSERT_EQ(result.trains.size(), 2U);
    EXPECT_EQ(result.trains[0].start, *result.graph->find_block("A"));
    EXPECT_EQ(result.trains[0].destination, *result.graph->find_block("B"));
    EXPECT_DOUBLE_EQ(result.trains[0].cruise_speed_mps, 15.0);
    EXPECT_DOUBLE_EQ(result.trains[1].cruise_speed_mps, 12.5);
}

TEST(ConfigLoaderTest, RejectsBadTrains) {
    const std::string track = "block A 100\nblock B 100\nstation West A\nstation East B\n";

    EXPECT_TRUE(mentions(single_error(track + "train West East\n"), "expected: train"));
    EXPECT_TRUE(mentions(single_error(track + "train West Nowhere 10\n"), "unknown station"));
    EXPECT_TRUE(mentions(single_error(track + "train West East fast\n"), "positive number"));
    EXPECT_TRUE(mentions(single_error(track + "train West East 0\n"), "positive number"));
    EXPECT_TRUE(mentions(single_error(track + "train West East 10\ntrain West East 10\n"),
                         "already starts"));
    EXPECT_TRUE(load(track + "train West East 10\ntrain West East 10\n").trains.empty());
}

TEST(ConfigLoaderTest, CollectsEveryErrorAndReturnsNoGraph) {
    const LoadResult result = load("block A 100\n"
                                   "block B -1\n"
                                   "bogus\n"
                                   "link A Z\n");

    EXPECT_FALSE(result.ok());
    EXPECT_FALSE(result.graph.has_value());
    ASSERT_EQ(result.errors.size(), 3U);
    EXPECT_EQ(result.errors[0].line, 2U);
    EXPECT_EQ(result.errors[1].line, 3U);
    EXPECT_EQ(result.errors[2].line, 4U);
}

TEST(ConfigLoaderTest, EmptyInputIsAnError) {
    const ParseError error = single_error("# only a comment\n");
    EXPECT_EQ(error.line, 0U);
    EXPECT_TRUE(mentions(error, "no blocks"));
}

TEST(ConfigLoaderTest, MissingFileIsAnError) {
    const LoadResult result = load_track_file("/nonexistent/railsim/track.txt");
    EXPECT_FALSE(result.ok());
    ASSERT_EQ(result.errors.size(), 1U);
    EXPECT_TRUE(mentions(result.errors[0], "cannot open"));
}

TEST(ConfigLoaderTest, LoadsExampleScenarioFile) {
    const LoadResult result = load_track_file(std::string(RAILSIM_SCENARIO_DIR) + "/simple_line.txt");

    ASSERT_TRUE(result.ok());
    EXPECT_EQ(result.graph->block_count(), 6U);
    EXPECT_EQ(result.graph->link_count(), 6U);
    EXPECT_EQ(result.graph->station_count(), 2U);
    EXPECT_EQ(result.graph->block(*result.graph->find_block("S")).status,
              BlockStatus::OutOfService);
}

}  // namespace
}  // namespace railsim
