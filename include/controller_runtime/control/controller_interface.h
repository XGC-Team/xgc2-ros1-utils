#pragma once

#include <string>

#include "controller_runtime/scheduler/task_gate.h"
#include "controller_runtime/time/tick_context.h"

namespace controller_runtime {

struct ControllerTiming {
    double compute_period_s{0.02};
    double command_publish_period_s{0.02};
    double debug_publish_period_s{0.2};
    ClockDomain compute_clock{ClockDomain::Ros};
    ClockDomain command_clock{ClockDomain::Wall};
    ClockDomain debug_clock{ClockDomain::Wall};
};

struct ControllerSpec {
    std::string name;
    ControllerTiming timing;
    bool publish_command_heartbeat{true};
    bool publish_zero_on_exit{true};
};

struct ControllerResult {
    bool ok{false};
    double compute_time_ms{0.0};
    std::string reason;
};

class IController {
public:
    virtual ~IController() = default;

    virtual ControllerSpec spec() const = 0;
    virtual void onActivate(const TickContext&) {}
    virtual ControllerResult compute(const TickContext&) = 0;
    virtual void onDeactivate(const TickContext&) {}
    virtual void reset() {}
};

}  // namespace controller_runtime
