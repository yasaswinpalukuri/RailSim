#pragma once

#include <cstdint>
#include <iosfwd>
#include <string>
#include <string_view>
#include <vector>

namespace railsim {

enum class EventType : std::uint8_t {
    Reserved,
    ReserveRejected,
    Entered,
    EnterRejected,
    Left,
    Released,
    ReleaseRejected,
    RoutePlanned,
    NoRoute,
    EmergencyBrake,
    Arrived,
    Timeout,
    SafetyViolation,
};

[[nodiscard]] std::string_view to_string(EventType type);

struct Event {
    std::uint64_t tick{};
    std::string train;   // empty when the event is not about a train
    std::string block;   // empty when the event is not about a block
    EventType type{};
    std::string reason;  // empty unless something was rejected or needs explaining
};

// Records every event in memory and, if a sink is given, also writes it as one
// CSV line: tick,train,block,event,reason
//
// The sink is not owned. It must outlive the logger; the caller's
// std::ofstream closes the file when it goes out of scope (RAII).
class EventLogger {
public:
    explicit EventLogger(std::ostream* sink = nullptr);

    // Simulation time stamped on every event logged from now on.
    void set_tick(std::uint64_t tick) { tick_ = tick; }
    [[nodiscard]] std::uint64_t tick() const { return tick_; }

    void log(EventType type, std::string train, std::string block, std::string reason = "");

    [[nodiscard]] const std::vector<Event>& events() const { return events_; }

private:
    std::ostream* sink_;
    std::uint64_t tick_{0};
    std::vector<Event> events_;
};

}  // namespace railsim
