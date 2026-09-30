#pragma once

#include <functional>
#include <optional>
#include <vector>

#include "railsim/track_graph.hpp"
#include "railsim/types.hpp"

namespace railsim {

struct Route {
    std::vector<BlockId> blocks;  // from start to destination, both included
    double length_m{};            // sum of the lengths of every block in `blocks`
};

// Answers "is this block unavailable right now?" (occupied, reserved, ...).
// The planner asks through this callback so it does not depend on whoever
// owns that state.
using BlockPredicate = std::function<bool(BlockId)>;

// Shortest route by total block length (Dijkstra).
//
// A block is skipped if it is out of service or `is_blocked` returns true for
// it. The start block is never skipped: the train asking for a route is
// already standing on it. Returns nullopt when no usable route exists.
// When two routes are equally short, the one found first is kept, so the
// result is the same on every run.
//
// Throws std::out_of_range if `from` or `to` is not a block of `graph`.
[[nodiscard]] std::optional<Route> find_route(const TrackGraph& graph, BlockId from, BlockId to,
                                              const BlockPredicate& is_blocked = nullptr);

}  // namespace railsim
