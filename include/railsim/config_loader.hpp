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
//
// A block must be declared before a link or station refers to it.

struct ParseError {
    std::size_t line{};  // 1-based; 0 means the error is about the file as a whole
    std::string message;
};

// All or nothing: `graph` is set only when `errors` is empty, so a partly
// loaded network can never reach the simulator.
struct LoadResult {
    std::optional<TrackGraph> graph;
    std::vector<ParseError> errors;

    [[nodiscard]] bool ok() const { return graph.has_value(); }
};

// Reads the whole stream and reports every error found, not just the first.
[[nodiscard]] LoadResult load_track(std::istream& input);

[[nodiscard]] LoadResult load_track_file(const std::string& path);

}  // namespace railsim
