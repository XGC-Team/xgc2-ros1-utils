#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <thread>
#include <utility>

#include <ros/ros.h>

#include "controller_runtime/time/tick_context.h"

namespace controller_runtime {

struct LoopControllerOptions {
    double frequency_hz{500.0};
    double overrun_warn_period_s{1.0};
    double ros_jump_forward_threshold_s{1.0};
};

class LoopController {
public:
    explicit LoopController(LoopControllerOptions options = LoopControllerOptions())
        : options_(options) {
        if (options_.frequency_hz <= 0.0) {
            options_.frequency_hz = 500.0;
        }
        if (options_.ros_jump_forward_threshold_s <= 0.0) {
            options_.ros_jump_forward_threshold_s = 1.0;
        }
        period_ = std::chrono::duration<double>(1.0 / options_.frequency_hz);
    }

    void requestStop() { stop_requested_.store(true); }
    bool stopRequested() const { return stop_requested_.load(); }

    template<typename TickCallback, typename AfterTickCallback>
    void run(TickCallback&& tick, AfterTickCallback&& after_tick) {
        stop_requested_.store(false);
        while (ros::ok() && !stop_requested_.load()) {
            const auto loop_start = Clock::now();

            ros::spinOnce();
            tick(makeTickContext());
            after_tick();

            const Duration elapsed = Clock::now() - loop_start;
            const Duration remaining = period_ - elapsed;
            if (remaining > Duration::zero()) {
                std::this_thread::sleep_for(remaining);
            } else {
                warnOverrun(elapsed);
            }
        }
    }

    template<typename TickCallback>
    void run(TickCallback&& tick) {
        run(std::forward<TickCallback>(tick), [] {});
    }

    TickContext makeTickContext() {
        TickContext ctx;
        ctx.seq = seq_++;
        ctx.wall_now = ros::WallTime::now();
        ctx.ros_now = ros::Time::now();
        ctx.ros_time_valid = !ctx.ros_now.isZero();

        if (!last_wall_now_.isZero()) {
            ctx.wall_dt = (ctx.wall_now - last_wall_now_).toSec();
        }

        if (ctx.ros_time_valid && last_ros_time_valid_) {
            ctx.ros_dt = (ctx.ros_now - last_ros_now_).toSec();
            ctx.ros_time_jumped_back = ctx.ros_dt < 0.0;
            ctx.ros_time_jumped_forward =
                ctx.ros_dt > options_.ros_jump_forward_threshold_s;
        }

        last_wall_now_ = ctx.wall_now;
        if (ctx.ros_time_valid) {
            last_ros_now_ = ctx.ros_now;
            last_ros_time_valid_ = true;
        }
        return ctx;
    }

private:
    using Clock = std::chrono::steady_clock;
    using Duration = std::chrono::duration<double>;

    void warnOverrun(const Duration& elapsed) {
        const ros::WallTime now = ros::WallTime::now();
        if (last_overrun_warning_.isZero() ||
            (now - last_overrun_warning_).toSec() >= options_.overrun_warn_period_s) {
            ROS_WARN("[LoopController] loop overrun: target %.1f Hz, elapsed %.3f ms",
                     options_.frequency_hz,
                     elapsed.count() * 1000.0);
            last_overrun_warning_ = now;
        }
    }

    LoopControllerOptions options_;
    Duration period_{1.0 / 500.0};
    ros::WallTime last_overrun_warning_;
    ros::WallTime last_wall_now_;
    ros::Time last_ros_now_;
    bool last_ros_time_valid_{false};
    uint64_t seq_{0};
    std::atomic<bool> stop_requested_{false};
};

}  // namespace controller_runtime
