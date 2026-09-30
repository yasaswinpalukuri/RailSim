#include "railsim/config_loader.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <istream>
#include <sstream>
#include <utility>

namespace railsim {
namespace {

using Tokens = std::vector<std::string>;

// Drops everything from '#' onwards, then splits on whitespace.
Tokens tokenize(const std::string& line) {
    std::istringstream stream(line.substr(0, line.find('#')));
    Tokens tokens;
    std::string token;
    while (stream >> token) {
        tokens.push_back(token);
    }
    return tokens;
}

// Accepts only text that is a finite number from start to end ("12x" fails).
std::optional<double> parse_number(const std::string& text) {
    char* end = nullptr;
    const double value = std::strtod(text.c_str(), &end);
    if (end == text.c_str() || *end != '\0' || !std::isfinite(value)) {
        return std::nullopt;
    }
    return value;
}

class Parser {
public:
    void parse_line(std::size_t line_number, const std::string& line) {
        line_number_ = line_number;
        const Tokens tokens = tokenize(line);
        if (tokens.empty()) {
            return;
        }

        const std::string& keyword = tokens[0];
        if (keyword == "block") {
            parse_block(tokens);
        } else if (keyword == "link") {
            parse_link(tokens);
        } else if (keyword == "station") {
            parse_station(tokens);
        } else if (keyword == "train") {
            parse_train(tokens);
        } else {
            error("unknown keyword '" + keyword + "'");
        }
    }

    LoadResult finish() {
        if (errors_.empty() && graph_.block_count() == 0) {
            errors_.push_back(ParseError{0, "track file defines no blocks"});
        }

        LoadResult result;
        if (errors_.empty()) {
            result.graph = std::move(graph_);
            result.trains = std::move(trains_);
        }
        result.errors = std::move(errors_);
        return result;
    }

private:
    void error(std::string message) {
        errors_.push_back(ParseError{line_number_, std::move(message)});
    }

    // Looks a block up by name and records an error if it was never declared.
    std::optional<BlockId> require_block(const std::string& name) {
        const auto id = graph_.find_block(name);
        if (!id) {
            error("unknown block '" + name + "'");
        }
        return id;
    }

    void parse_block(const Tokens& tokens) {
        if (tokens.size() != 3 && tokens.size() != 4) {
            error("expected: block <name> <length_m> [out_of_service]");
            return;
        }
        if (tokens.size() == 4 && tokens[3] != "out_of_service") {
            error("unknown block option '" + tokens[3] + "'");
            return;
        }

        const auto length = parse_number(tokens[2]);
        if (!length || *length <= 0.0) {
            error("block length must be a positive number, got '" + tokens[2] + "'");
            return;
        }

        const auto status =
            tokens.size() == 4 ? BlockStatus::OutOfService : BlockStatus::InService;
        if (!graph_.add_block(tokens[1], *length, status)) {
            error("duplicate block '" + tokens[1] + "'");
        }
    }

    void parse_link(const Tokens& tokens) {
        if (tokens.size() != 3) {
            error("expected: link <block> <block>");
            return;
        }

        const auto first = require_block(tokens[1]);
        const auto second = require_block(tokens[2]);
        if (!first || !second) {
            return;
        }
        if (*first == *second) {
            error("block '" + tokens[1] + "' cannot link to itself");
            return;
        }
        if (!graph_.add_link(*first, *second)) {
            error("duplicate link '" + tokens[1] + "' - '" + tokens[2] + "'");
        }
    }

    void parse_station(const Tokens& tokens) {
        if (tokens.size() != 3) {
            error("expected: station <station_name> <block>");
            return;
        }

        const auto block = require_block(tokens[2]);
        if (!block) {
            return;
        }
        if (!graph_.add_station(tokens[1], *block)) {
            error("duplicate station '" + tokens[1] + "'");
        }
    }

    std::optional<BlockId> require_station(const std::string& name) {
        const auto id = graph_.find_station(name);
        if (!id) {
            error("unknown station '" + name + "'");
        }
        return id;
    }

    void parse_train(const Tokens& tokens) {
        if (tokens.size() != 4) {
            error("expected: train <from_station> <to_station> <cruise_speed_mps>");
            return;
        }

        const auto start = require_station(tokens[1]);
        const auto destination = require_station(tokens[2]);
        const auto speed = parse_number(tokens[3]);
        if (!speed || *speed <= 0.0) {
            error("train speed must be a positive number, got '" + tokens[3] + "'");
        }
        if (!start || !destination || !speed || *speed <= 0.0) {
            return;
        }

        for (const TrainSpec& other : trains_) {
            if (other.start == *start) {
                error("another train already starts at station '" + tokens[1] + "'");
                return;
            }
        }
        trains_.push_back(TrainSpec{*start, *destination, *speed});
    }

    TrackGraph graph_;
    std::vector<TrainSpec> trains_;
    std::vector<ParseError> errors_;
    std::size_t line_number_{0};
};

}  // namespace

LoadResult load_track(std::istream& input) {
    Parser parser;
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        parser.parse_line(line_number, line);
    }
    return parser.finish();
}

LoadResult load_track_file(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        LoadResult result;
        result.errors.push_back(ParseError{0, "cannot open '" + path + "'"});
        return result;
    }
    return load_track(file);
}

}  // namespace railsim
