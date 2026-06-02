#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <utility>

#include <ros/ros.h>

namespace ros1_utils {

struct LoopControllerOptions {
    double frequency_hz{500.0};
    double overrun_warn_period_s{1.0};
    std::string runtime_clock{"ros"};
};

class LoopController {
public:
    explicit LoopController(LoopControllerOptions options = LoopControllerOptions())
        : options_(options) {
        if (options_.frequency_hz <= 0.0) {
            options_.frequency_hz = 500.0;
        }
        if (options_.runtime_clock != "ros" && options_.runtime_clock != "wall") {
            ROS_WARN("[LoopController] unknown runtime_clock '%s', using ros",
                     options_.runtime_clock.c_str());
            options_.runtime_clock = "ros";
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
            tick(runtimeNow());
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

private:
    using Clock = std::chrono::steady_clock;
    using Duration = std::chrono::duration<double>;

    ros::Time runtimeNow() const {
        if (options_.runtime_clock == "wall") {
            return ros::Time(ros::WallTime::now().toSec());
        }
        return ros::Time::now();
    }

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
    std::atomic<bool> stop_requested_{false};
};

class PeriodicGate {
public:
    explicit PeriodicGate(double period_s = 0.0)
        : period_s_(std::max(0.0, period_s)) {}

    void setPeriod(double period_s) { period_s_ = std::max(0.0, period_s); }

    bool ready(const ros::Time& now) const {
        return last_run_.isZero() || period_s_ <= 0.0 ||
               (now - last_run_).toSec() >= period_s_;
    }

    void markRun(const ros::Time& now) { last_run_ = now; }
    const ros::Time& lastRun() const { return last_run_; }

private:
    double period_s_{0.0};
    ros::Time last_run_;
};

class DirtyPeriodicGate {
public:
    explicit DirtyPeriodicGate(double period_s = 0.0)
        : periodic_(period_s) {}

    void setPeriod(double period_s) { periodic_.setPeriod(period_s); }
    void markDirty() { dirty_ = true; }
    void clearDirty() { dirty_ = false; }

    bool ready(const ros::Time& now) const {
        return dirty_ || periodic_.ready(now);
    }

    void markRun(const ros::Time& now) {
        periodic_.markRun(now);
        dirty_ = false;
    }

private:
    PeriodicGate periodic_;
    bool dirty_{false};
};

}  // namespace ros1_utils
