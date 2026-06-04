#pragma once

#include <algorithm>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "controller_runtime/time/tick_context.h"

namespace controller_runtime {

enum class ClockDomain {
    Wall,
    Ros,
};

enum class DueReason {
    None,
    FirstRun,
    Period,
    Dirty,
};

enum class SkipReason {
    None,
    NotDue,
    RosClockInvalid,
    StateDisabled,
};

using DirtySet = std::unordered_set<std::string>;

struct TaskSpec {
    std::string name;
    ClockDomain clock_domain{ClockDomain::Ros};
    double period_s{0.0};
    double min_period_s{0.0};
    double timeout_s{0.0};
    bool run_on_dirty{true};
    std::vector<std::string> dirty_dependencies;
    std::vector<std::string> enabled_states;
};

struct TaskDecision {
    bool due{false};
    DueReason due_reason{DueReason::None};
    SkipReason skip_reason{SkipReason::None};
};

class TaskGate {
public:
    explicit TaskGate(TaskSpec spec = TaskSpec()) : spec_(std::move(spec)) { normalize(); }

    const TaskSpec& spec() const { return spec_; }
    void setSpec(TaskSpec spec) {
        spec_ = std::move(spec);
        normalize();
        reset();
    }

    TaskDecision evaluate(const TickContext& ctx, const DirtySet& dirty) const {
        TaskDecision decision;
        const auto now = selectedTime(ctx);
        if (!now.has_value()) {
            decision.skip_reason = SkipReason::RosClockInvalid;
            return decision;
        }

        if (!last_run_s_.has_value()) {
            decision.due = true;
            decision.due_reason = DueReason::FirstRun;
            return decision;
        }

        const double elapsed = *now - *last_run_s_;
        constexpr double kTimeEpsilon = 1e-9;
        const bool period_due = spec_.period_s <= 0.0 || elapsed + kTimeEpsilon >= spec_.period_s;
        if (period_due) {
            decision.due = true;
            decision.due_reason = DueReason::Period;
            return decision;
        }

        const bool dirty_due = spec_.run_on_dirty && dirtyMatches(dirty) &&
                               elapsed + kTimeEpsilon >= spec_.min_period_s;
        if (dirty_due) {
            decision.due = true;
            decision.due_reason = DueReason::Dirty;
            return decision;
        }

        decision.skip_reason = SkipReason::NotDue;
        return decision;
    }

    bool ready(const TickContext& ctx, const DirtySet& dirty) const {
        return evaluate(ctx, dirty).due;
    }

    void markRun(const TickContext& ctx) {
        const auto now = selectedTime(ctx);
        if (now.has_value()) {
            last_run_s_ = *now;
        }
    }

    void reset() { last_run_s_.reset(); }
    std::optional<double> lastRunSec() const { return last_run_s_; }

private:
    void normalize() {
        spec_.period_s = std::max(0.0, spec_.period_s);
        spec_.min_period_s = std::max(0.0, spec_.min_period_s);
        spec_.timeout_s = std::max(0.0, spec_.timeout_s);
    }

    std::optional<double> selectedTime(const TickContext& ctx) const {
        if (spec_.clock_domain == ClockDomain::Wall) {
            return ctx.wall_now.toSec();
        }
        if (!ctx.ros_time_valid) {
            return std::nullopt;
        }
        return ctx.ros_now.toSec();
    }

    bool dirtyMatches(const DirtySet& dirty) const {
        if (dirty.empty()) {
            return false;
        }
        if (spec_.dirty_dependencies.empty()) {
            return true;
        }
        for (const auto& dependency : spec_.dirty_dependencies) {
            if (dirty.find(dependency) != dirty.end()) {
                return true;
            }
        }
        return false;
    }

    TaskSpec spec_;
    std::optional<double> last_run_s_;
};

inline const char* toString(DueReason reason) {
    switch (reason) {
    case DueReason::None:
        return "none";
    case DueReason::FirstRun:
        return "first_run";
    case DueReason::Period:
        return "period";
    case DueReason::Dirty:
        return "dirty";
    }
    return "unknown";
}

inline const char* toString(SkipReason reason) {
    switch (reason) {
    case SkipReason::None:
        return "none";
    case SkipReason::NotDue:
        return "not_due";
    case SkipReason::RosClockInvalid:
        return "ros_clock_invalid";
    case SkipReason::StateDisabled:
        return "state_disabled";
    }
    return "unknown";
}

}  // namespace controller_runtime
