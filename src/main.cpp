#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <string>

#include "railsim/config_loader.hpp"

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

}  // namespace

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "usage: railsim <track-file>\n";
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

    print_summary(*result.graph);
    return EXIT_SUCCESS;
}
