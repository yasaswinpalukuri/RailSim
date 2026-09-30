#pragma once

#include <cstddef>
#include <iosfwd>
#include <optional>
#include <string>
#include <vector>

#include "railsim/track_graph.hpp"

namespace railsim {

// Track file format, one statement per line ('#' starts a comment):
//
//   block   <name> <length_m> [out_of_service]
//   link    <block> <block>
//   station <station_name> <block>
//   train   <from_station> <to_station> <cruise_speed_mps>
//
// A block must be declared before a link or station refers to it, and a
// station before a train refers to it. Trains are numbered T1, T2, ... in
// file order.

struct ParseError {
    std::size_t line{};  // 1-based; 0 means the error is about the file as a whole
    std::string message;
};

// All or nothing: `graph` and `trains` are set only when `errors` is empty, so
// a partly loaded scenario can never reach the simulator.
struct LoadResult {
    std::optional<TrackGraph> graph;
    std::vector<TrainSpec> trains;
    std::vector<ParseError> errors;

    [[nodiscard]] bool ok() const { return graph.has_value(); }
};

// Reads the whole stream and reports every error found, not just the first.
[[nodiscard]] LoadResult load_track(std::istream& input);

[[nodiscard]] LoadResult load_track_file(const std::string& path);

}  // namespace railsim
