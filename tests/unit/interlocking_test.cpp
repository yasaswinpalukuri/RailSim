#include "railsim/interlocking.hpp"

#include <cstddef>
#include <random>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

namespace railsim {
namespace {

// A --- B --- C, plus S (out of service) linked to B.
class InterlockingTest : public ::testing::Test {
protected:
    void SetUp() override {
        a = *graph.add_block("A", 100.0);
        b = *graph.add_block("B", 100.0);
        c = *graph.add_block("C", 100.0);
        s = *graph.add_block("S", 100.0, BlockStatus::OutOfService);
        EXPECT_TRUE(graph.add_link(a, b));
        EXPECT_TRUE(graph.add_link(b, c));
        EXPECT_TRUE(graph.add_link(b, s));
    }

    // Reserves and enters in one go; the test fails if either step is refused.
    void place(Interlocking& interlocking, TrainId train, BlockId block) {
        EXPECT_EQ(interlocking.reserve(train, block), MoveResult::Granted);
        EXPECT_EQ(interlocking.enter(train, block), MoveResult::Granted);
    }

    const Event& last_event() const { return logger.events().back(); }

    TrackGraph graph;
    EventLogger logger;
    BlockId a, b, c, s;
    TrainId t1{1};
    TrainId t2{2};
};

TEST_F(InterlockingTest, AllBlocksStartFree) {
    const Interlocking interlocking(graph, logger);

    EXPECT_FALSE(interlocking.holder(a).has_value());
    EXPECT_FALSE(interlocking.is_occupied(a));
    EXPECT_FALSE(interlocking.position(t1).has_value());
}

TEST_F(InterlockingTest, ReserveFreeBlockIsGrantedAndLogged) {
    Interlocking interlocking(graph, logger);

    EXPECT_EQ(interlocking.reserve(t1, a), MoveResult::Granted);

    EXPECT_EQ(interlocking.holder(a), t1);
    EXPECT_FALSE(interlocking.is_occupied(a));
    ASSERT_EQ(logger.events().size(), 1U);
    EXPECT_EQ(last_event().type, EventType::Reserved);
    EXPECT_EQ(last_event().train, "T1");
    EXPECT_EQ(last_event().block, "A");
}

TEST_F(InterlockingTest, ReservingAgainIsGrantedWithoutANewEvent) {
    Interlocking interlocking(graph, logger);
    ASSERT_EQ(interlocking.reserve(t1, a), MoveResult::Granted);

    EXPECT_EQ(interlocking.reserve(t1, a), MoveResult::Granted);
    EXPECT_EQ(logger.events().size(), 1U);
}

TEST_F(InterlockingTest, ReserveRejectedWhenHeldByAnotherTrain) {
    Interlocking interlocking(graph, logger);
    ASSERT_EQ(interlocking.reserve(t1, a), MoveResult::Granted);

    EXPECT_EQ(interlocking.reserve(t2, a), MoveResult::ReservedByOther);

    EXPECT_EQ(interlocking.holder(a), t1);
    EXPECT_EQ(last_event().type, EventType::ReserveRejected);
    EXPECT_EQ(last_event().train, "T2");
    EXPECT_EQ(last_event().reason, "reserved_by_other_train");
}

TEST_F(InterlockingTest, ReserveRejectedWhenOccupiedByAnotherTrain) {
    Interlocking interlocking(graph, logger);
    place(interlocking, t1, a);

    EXPECT_EQ(interlocking.reserve(t2, a), MoveResult::OccupiedByOther);
    EXPECT_EQ(last_event().reason, "occupied_by_other_train");
}

TEST_F(InterlockingTest, ReserveRejectedWhenOutOfService) {
    Interlocking interlocking(graph, logger);

    EXPECT_EQ(interlocking.reserve(t1, s), MoveResult::OutOfService);

    EXPECT_FALSE(interlocking.holder(s).has_value());
    EXPECT_EQ(last_event().reason, "block_out_of_service");
}

TEST_F(InterlockingTest, EnterWithoutReservationIsRejected) {
    Interlocking interlocking(graph, logger);

    EXPECT_EQ(interlocking.enter(t1, a), MoveResult::NotReserved);

    EXPECT_FALSE(interlocking.is_occupied(a));
    EXPECT_FALSE(interlocking.position(t1).has_value());
    EXPECT_EQ(last_event().type, EventType::EnterRejected);
    EXPECT_EQ(last_event().reason, "block_not_reserved");
}

TEST_F(InterlockingTest, EnterBlockHeldByAnotherTrainIsRejected) {
    Interlocking interlocking(graph, logger);
    place(interlocking, t1, a);
    ASSERT_EQ(interlocking.reserve(t2, b), MoveResult::Granted);

    EXPECT_EQ(interlocking.enter(t1, b), MoveResult::ReservedByOther);
    EXPECT_EQ(interlocking.position(t1), a);

    place(interlocking, t2, b);
    EXPECT_EQ(interlocking.enter(t1, b), MoveResult::OccupiedByOther);
    EXPECT_EQ(interlocking.position(t1), a);
}

TEST_F(InterlockingTest, FirstEnterPlacesTheTrain) {
    Interlocking interlocking(graph, logger);

    place(interlocking, t1, b);

    EXPECT_EQ(interlocking.position(t1), b);
    EXPECT_TRUE(interlocking.is_occupied(b));
    EXPECT_EQ(last_event().type, EventType::Entered);
}

TEST_F(InterlockingTest, MovingFreesThePreviousBlock) {
    Interlocking interlocking(graph, logger);
    place(interlocking, t1, a);
    ASSERT_EQ(interlocking.reserve(t1, b), MoveResult::Granted);

    EXPECT_EQ(interlocking.enter(t1, b), MoveResult::Granted);

    EXPECT_EQ(interlocking.position(t1), b);
    EXPECT_FALSE(interlocking.holder(a).has_value());
    EXPECT_FALSE(interlocking.is_occupied(a));
    EXPECT_EQ(interlocking.reserve(t2, a), MoveResult::Granted);

    // The move is logged as "left A" followed by "entered B".
    const std::size_t count = logger.events().size();
    ASSERT_TRUE(count >= 3U);
    EXPECT_EQ(logger.events()[count - 3].type, EventType::Left);
    EXPECT_EQ(logger.events()[count - 3].block, "A");
    EXPECT_EQ(logger.events()[count - 2].type, EventType::Entered);
    EXPECT_EQ(logger.events()[count - 2].block, "B");
}

TEST_F(InterlockingTest, EnterNonAdjacentBlockIsRejected) {
    Interlocking interlocking(graph, logger);
    place(interlocking, t1, a);
    ASSERT_EQ(interlocking.reserve(t1, c), MoveResult::Granted);

    EXPECT_EQ(interlocking.enter(t1, c), MoveResult::NotAdjacent);

    EXPECT_EQ(interlocking.position(t1), a);
    EXPECT_FALSE(interlocking.is_occupied(c));
    EXPECT_EQ(last_event().reason, "block_not_adjacent");
}

TEST_F(InterlockingTest, EnterBlockClosedAfterReservationIsRejected) {
    Interlocking interlocking(graph, logger);
    place(interlocking, t1, a);
    ASSERT_EQ(interlocking.reserve(t1, b), MoveResult::Granted);
    graph.set_status(b, BlockStatus::OutOfService);

    EXPECT_EQ(interlocking.enter(t1, b), MoveResult::OutOfService);
    EXPECT_EQ(interlocking.position(t1), a);
}

TEST_F(InterlockingTest, EnterCurrentBlockIsGrantedWithoutANewEvent) {
    Interlocking interlocking(graph, logger);
    place(interlocking, t1, a);
    const std::size_t count = logger.events().size();

    EXPECT_EQ(interlocking.enter(t1, a), MoveResult::Granted);
    EXPECT_EQ(logger.events().size(), count);
}

TEST_F(InterlockingTest, ReleaseGivesUpAnUnusedReservation) {
    Interlocking interlocking(graph, logger);
    ASSERT_EQ(interlocking.reserve(t1, b), MoveResult::Granted);

    EXPECT_EQ(interlocking.release(t1, b), MoveResult::Granted);

    EXPECT_FALSE(interlocking.holder(b).has_value());
    EXPECT_EQ(last_event().type, EventType::Released);
    EXPECT_EQ(interlocking.reserve(t2, b), MoveResult::Granted);
}

TEST_F(InterlockingTest, ReleaseRejectedForNonHolderAndForOccupiedBlock) {
    Interlocking interlocking(graph, logger);
    place(interlocking, t1, a);

    EXPECT_EQ(interlocking.release(t2, a), MoveResult::NotHolder);
    EXPECT_EQ(interlocking.release(t2, b), MoveResult::NotHolder);
    EXPECT_EQ(interlocking.release(t1, a), MoveResult::StillOccupied);

    EXPECT_EQ(interlocking.holder(a), t1);
    EXPECT_TRUE(interlocking.is_occupied(a));
    EXPECT_EQ(last_event().type, EventType::ReleaseRejected);
    EXPECT_EQ(last_event().reason, "block_still_occupied");
}

TEST_F(InterlockingTest, UnknownBlockThrows) {
    Interlocking interlocking(graph, logger);
    const BlockId unknown{99};

    EXPECT_THROW((void)interlocking.reserve(t1, unknown), std::out_of_range);
    EXPECT_THROW((void)interlocking.enter(t1, unknown), std::out_of_range);
    EXPECT_THROW((void)interlocking.release(t1, unknown), std::out_of_range);
    EXPECT_THROW((void)interlocking.holder(unknown), std::out_of_range);
}

// Throws random requests from three trains at a ring of six blocks and checks
// the safety rule after every single request, whatever the answer was.
TEST(InterlockingPropertyTest, RandomRequestsNeverPutTwoTrainsInOneBlock) {
    const std::size_t block_count = 6;
    const std::size_t train_count = 3;

    TrackGraph graph;
    std::vector<BlockId> blocks;
    for (std::size_t i = 0; i < block_count; ++i) {
        blocks.push_back(*graph.add_block("R" + std::to_string(i), 100.0));
    }
    for (std::size_t i = 0; i < block_count; ++i) {
        ASSERT_TRUE(graph.add_link(blocks[i], blocks[(i + 1) % block_count]));
    }

    EventLogger logger;
    Interlocking interlocking(graph, logger);
    std::mt19937 random(12345);  // fixed seed: a failure can be replayed exactly
    std::size_t granted_moves = 0;

    for (int step = 0; step < 20000; ++step) {
        const TrainId train{random() % train_count};
        const BlockId block = blocks[random() % block_count];
        const std::optional<BlockId> before = interlocking.position(train);

        switch (random() % 3) {
            case 0:
                (void)interlocking.reserve(train, block);
                break;
            case 1:
                if (interlocking.enter(train, block) == MoveResult::Granted && before != block) {
                    ++granted_moves;
                }
                break;
            default:
                (void)interlocking.release(train, block);
                break;
        }

        std::vector<std::size_t> trains_in_block(block_count, 0);
        for (std::size_t t = 0; t < train_count; ++t) {
            const std::optional<BlockId> position = interlocking.position(TrainId{t});
            if (position.has_value()) {
                ++trains_in_block[position->value];
                ASSERT_EQ(interlocking.holder(*position), TrainId{t});
                ASSERT_TRUE(interlocking.is_occupied(*position));
            }
        }
        for (std::size_t i = 0; i < block_count; ++i) {
            ASSERT_TRUE(trains_in_block[i] <= 1U);
            ASSERT_EQ(interlocking.is_occupied(blocks[i]), trains_in_block[i] == 1U);
        }
    }

    // Guards against a test that passes only because nothing ever moved.
    EXPECT_TRUE(granted_moves > 100U);
}

}  // namespace
}  // namespace railsim
