#pragma once

#include <algorithm>
#include <chrono>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "controller_runtime/scheduler/task_gate.h"

namespace controller_runtime {

struct TaskStats {
    std::string name;
    uint64_t run_count{0};
    uint64_t skip_count{0};
    double last_compute_ms{0.0};
    double max_compute_ms{0.0};
    DueReason last_due_reason{DueReason::None};
    SkipReason last_skip_reason{SkipReason::None};
};

class ModuleScheduler {
public:
    using Callback = std::function<void(const TickContext&, const DirtySet&)>;

    void addTask(TaskSpec spec, Callback callback) {
        Task task;
        task.gate = TaskGate(std::move(spec));
        task.callback = std::move(callback);
        task.stats.name = task.gate.spec().name;
        tasks_.push_back(std::move(task));
    }

    void run(const TickContext& ctx, const DirtySet& dirty) { run(ctx, dirty, std::string{}); }

    void run(const TickContext& ctx, const DirtySet& dirty, const std::string& active_state) {
        for (auto& task : tasks_) {
            if (!stateEnabled(task.gate.spec(), active_state)) {
                ++task.stats.skip_count;
                task.stats.last_skip_reason = SkipReason::StateDisabled;
                continue;
            }
            const TaskDecision decision = task.gate.evaluate(ctx, dirty);
            if (!decision.due) {
                ++task.stats.skip_count;
                task.stats.last_skip_reason = decision.skip_reason;
                continue;
            }

            const auto start = Clock::now();
            task.callback(ctx, dirty);
            const auto elapsed =
                std::chrono::duration<double, std::milli>(Clock::now() - start).count();
            task.gate.markRun(ctx);
            ++task.stats.run_count;
            task.stats.last_compute_ms = elapsed;
            task.stats.max_compute_ms = std::max(task.stats.max_compute_ms, elapsed);
            task.stats.last_due_reason = decision.due_reason;
            task.stats.last_skip_reason = SkipReason::None;
        }
    }

    std::vector<TaskStats> stats() const {
        std::vector<TaskStats> result;
        result.reserve(tasks_.size());
        for (const auto& task : tasks_) {
            result.push_back(task.stats);
        }
        return result;
    }

private:
    using Clock = std::chrono::steady_clock;

    static bool stateEnabled(const TaskSpec& spec, const std::string& active_state) {
        if (spec.enabled_states.empty() || active_state.empty()) {
            return true;
        }
        return std::find(spec.enabled_states.begin(), spec.enabled_states.end(), active_state) !=
               spec.enabled_states.end();
    }

    struct Task {
        TaskGate gate;
        Callback callback;
        TaskStats stats;
    };

    std::vector<Task> tasks_;
};

}  // namespace controller_runtime
