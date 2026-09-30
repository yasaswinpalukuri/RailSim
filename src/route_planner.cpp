#include "railsim/route_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

namespace railsim {
namespace {

bool is_usable(const TrackGraph& graph, BlockId id, const BlockPredicate& is_blocked) {
    if (graph.block(id).status != BlockStatus::InService) {
        return false;
    }
    return !(is_blocked && is_blocked(id));
}

}  // namespace

std::optional<Route> find_route(const TrackGraph& graph, BlockId from, BlockId to,
                                const BlockPredicate& is_blocked) {
    if (!graph.contains(from) || !graph.contains(to)) {
        throw std::out_of_range("find_route: unknown block id");
    }
    if (from != to && !is_usable(graph, to, is_blocked)) {
        return std::nullopt;
    }

    const std::size_t count = graph.block_count();
    std::vector<double> distance(count, std::numeric_limits<double>::infinity());
    // previous[i] is the block the search reached block i from. Only entries on
    // a found path are ever read, so the initial value does not matter.
    std::vector<BlockId> previous(count, from);

    // Min-heap of (distance, block index). An entry is never updated in place;
    // a shorter distance is pushed as a new entry and the old one goes stale.
    using Entry = std::pair<double, std::size_t>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<>> frontier;

    distance[from.value] = graph.block(from).length_m;
    frontier.emplace(distance[from.value], from.value);

    while (!frontier.empty()) {
        const auto [current_distance, current] = frontier.top();
        frontier.pop();

        if (current_distance > distance[current]) {
            continue;  // stale entry
        }
        if (current == to.value) {
            break;  // the first time the destination is popped, its distance is final
        }

        for (const BlockId next : graph.neighbours(BlockId{current})) {
            if (!is_usable(graph, next, is_blocked)) {
                continue;
            }
            const double candidate = current_distance + graph.block(next).length_m;
            if (candidate < distance[next.value]) {
                distance[next.value] = candidate;
                previous[next.value] = BlockId{current};
                frontier.emplace(candidate, next.value);
            }
        }
    }

    if (std::isinf(distance[to.value])) {
        return std::nullopt;
    }

    Route route;
    route.length_m = distance[to.value];
    route.blocks.push_back(to);
    for (BlockId step = to; step != from;) {
        step = previous[step.value];
        route.blocks.push_back(step);
    }
    std::reverse(route.blocks.begin(), route.blocks.end());
    return route;
}

}  // namespace railsim
