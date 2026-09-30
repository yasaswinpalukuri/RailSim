#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>

#include "railsim/config_loader.hpp"
#include "railsim/route_planner.hpp"

namespace {

void print_summary(const railsim::TrackGraph& graph) {
    std::cout << graph.block_count() << " blocks, " << graph.link_count() << " links, "
              << graph.station_count() << " stations\n";

    for (std::size_t i = 0; i < graph.block_count(); ++i) {
        const railsim::BlockId id{i};
        const railsim::Block& block = graph.block(id);

        std::cout << "  " << block.name << " (" << block.length_m << " m";
        if (block.status == railsim::BlockStatus::OutOfService) {
            std::cout << ", out of service";
        }
        std::cout << ") ->";
        for (const railsim::BlockId neighbour : graph.neighbours(id)) {
            std::cout << ' ' << graph.block(neighbour).name;
        }
        std::cout << '\n';
    }
}

// Prints the shortest route between two stations. Returns the process exit code.
int print_route(const railsim::TrackGraph& graph, const std::string& from_name,
                const std::string& to_name) {
    const std::optional<railsim::BlockId> from = graph.find_station(from_name);
    const std::optional<railsim::BlockId> to = graph.find_station(to_name);
    if (!from) {
        std::cerr << "unknown station '" << from_name << "'\n";
    }
    if (!to) {
        std::cerr << "unknown station '" << to_name << "'\n";
    }
    if (!from || !to) {
        return EXIT_FAILURE;
    }

    const std::optional<railsim::Route> route = railsim::find_route(graph, *from, *to);
    if (!route) {
        std::cerr << "no route from '" << from_name << "' to '" << to_name << "'\n";
        return EXIT_FAILURE;
    }

    std::cout << "route " << from_name << " -> " << to_name << " (" << route->length_m << " m):";
    for (const railsim::BlockId id : route->blocks) {
        std::cout << ' ' << graph.block(id).name;
    }
    std::cout << '\n';
    return EXIT_SUCCESS;
}

}  // namespace

// The signature of main is fixed by the C++ standard.
// cppcheck-suppress constParameter
int main(int argc, char* argv[]) {
    if (argc != 2 && argc != 4) {
        std::cerr << "usage: railsim <track-file> [<from-station> <to-station>]\n";
        return EXIT_FAILURE;
    }

    const std::string path = argv[1];
    const railsim::LoadResult result = railsim::load_track_file(path);
    // Tested directly (not via ok()) so clang-tidy can see the dereference below is checked.
    if (!result.graph) {
        for (const railsim::ParseError& error : result.errors) {
            std::cerr << path << ':' << error.line << ": " << error.message << '\n';
        }
        return EXIT_FAILURE;
    }

    if (argc == 4) {
        return print_route(*result.graph, argv[2], argv[3]);
    }
    print_summary(*result.graph);
    return EXIT_SUCCESS;
}
