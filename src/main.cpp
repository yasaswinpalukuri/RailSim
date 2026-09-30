#include <cstddef>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include "railsim/config_loader.hpp"
#include "railsim/event_logger.hpp"
#include "railsim/route_planner.hpp"
#include "railsim/simulator.hpp"

namespace {

// Exit codes: 1 for bad usage or a bad input file, 2 for a simulation that
// did not end with every train arrived and no safety violation.
constexpr int kExitUnsuccessfulRun = 2;

void print_usage() {
    std::cerr << "usage:\n"
                 "  railsim show  <scenario-file>\n"
                 "  railsim route <scenario-file> <from-station> <to-station>\n"
                 "  railsim run   <scenario-file> [<log-file.csv>]\n";
}

int show(const railsim::TrackGraph& graph) {
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
    return EXIT_SUCCESS;
}

int route(const railsim::TrackGraph& graph, const std::string& from_name,
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

    const std::optional<railsim::Route> found = railsim::find_route(graph, *from, *to);
    if (!found) {
        std::cerr << "no route from '" << from_name << "' to '" << to_name << "'\n";
        return EXIT_FAILURE;
    }

    std::cout << "route " << from_name << " -> " << to_name << " (" << found->length_m << " m):";
    for (const railsim::BlockId id : found->blocks) {
        std::cout << ' ' << graph.block(id).name;
    }
    std::cout << '\n';
    return EXIT_SUCCESS;
}

// Writes the event log to `log_sink` and a one-line summary to `summary`.
int run(const railsim::TrackGraph& graph, const std::vector<railsim::TrainSpec>& trains,
        std::ostream& log_sink, std::ostream& summary) {
    if (trains.empty()) {
        std::cerr << "the scenario file defines no trains\n";
        return EXIT_FAILURE;
    }

    railsim::EventLogger logger(&log_sink);
    railsim::Simulator simulator(graph, trains, logger);
    const railsim::SimulationResult result = simulator.run();

    summary << result.arrived << " of " << result.trains << " trains arrived after "
            << result.ticks << " ticks, " << result.safety_violations << " safety violations\n";
    return result.all_arrived() && result.safety_violations == 0 ? EXIT_SUCCESS
                                                                  : kExitUnsuccessfulRun;
}

int dispatch(int argc, char* argv[]) {
    const std::string command = argc >= 2 ? argv[1] : "";
    const bool known = (command == "show" && argc == 3) || (command == "route" && argc == 5) ||
                       (command == "run" && (argc == 3 || argc == 4));
    if (!known) {
        print_usage();
        return EXIT_FAILURE;
    }

    const std::string path = argv[2];
    const railsim::LoadResult scenario = railsim::load_track_file(path);
    // Tested directly (not via ok()) so clang-tidy can see the dereferences below are checked.
    if (!scenario.graph) {
        for (const railsim::ParseError& error : scenario.errors) {
            std::cerr << path << ':' << error.line << ": " << error.message << '\n';
        }
        return EXIT_FAILURE;
    }

    if (command == "show") {
        return show(*scenario.graph);
    }
    if (command == "route") {
        return route(*scenario.graph, argv[3], argv[4]);
    }
    if (argc == 3) {
        return run(*scenario.graph, scenario.trains, std::cout, std::cerr);
    }

    std::ofstream log_file(argv[3]);
    if (!log_file) {
        std::cerr << "cannot write '" << argv[3] << "'\n";
        return EXIT_FAILURE;
    }
    return run(*scenario.graph, scenario.trains, log_file, std::cout);
}

}  // namespace

// The signature of main is fixed by the C++ standard.
// cppcheck-suppress constParameter
int main(int argc, char* argv[]) {
    try {
        return dispatch(argc, argv);
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
