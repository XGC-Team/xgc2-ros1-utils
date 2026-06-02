#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <utility>
#include <vector>

namespace controller_runtime {

struct RuntimeEvent {
    uint32_t id{0};
    double stamp{0.0};
    std::string source;
};

class EventQueue {
public:
    void push(RuntimeEvent event) { events_.push_back(std::move(event)); }

    void push(const std::vector<RuntimeEvent>& events) {
        for (const auto& event : events) {
            events_.push_back(event);
        }
    }

    bool empty() const { return events_.empty(); }
    std::size_t size() const { return events_.size(); }

    std::vector<RuntimeEvent> drain() {
        std::vector<RuntimeEvent> result;
        result.reserve(events_.size());
        while (!events_.empty()) {
            result.push_back(std::move(events_.front()));
            events_.pop_front();
        }
        return result;
    }

private:
    std::deque<RuntimeEvent> events_;
};

}  // namespace controller_runtime
