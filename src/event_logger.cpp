#include "railsim/event_logger.hpp"

#include <ostream>
#include <utility>

namespace railsim {

std::string_view to_string(EventType type) {
    switch (type) {
        case EventType::Reserved:
            return "reserved";
        case EventType::ReserveRejected:
            return "reserve_rejected";
        case EventType::Entered:
            return "entered";
        case EventType::EnterRejected:
            return "enter_rejected";
        case EventType::Left:
            return "left";
        case EventType::Released:
            return "released";
        case EventType::ReleaseRejected:
            return "release_rejected";
    }
    return "unknown";
}

EventLogger::EventLogger(std::ostream* sink) : sink_(sink) {
    if (sink_ != nullptr) {
        *sink_ << "tick,train,block,event,reason\n";
    }
}

void EventLogger::log(EventType type, std::string train, std::string block, std::string reason) {
    events_.push_back(Event{tick_, std::move(train), std::move(block), type, std::move(reason)});

    if (sink_ != nullptr) {
        const Event& event = events_.back();
        // Flushed per line: if the program dies, the log still ends at the last event.
        *sink_ << event.tick << ',' << event.train << ',' << event.block << ','
               << to_string(event.type) << ',' << event.reason << '\n'
               << std::flush;
    }
}

}  // namespace railsim
