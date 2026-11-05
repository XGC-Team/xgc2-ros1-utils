#pragma once

#include <ros/callback_queue_interface.h>
#include <ros/node_handle.h>
#include <ros/timer.h>
#include <state_machine/owner_schedule.hpp>

#include <cstdint>
#include <functional>
#include <utility>

namespace ros1_utils {

// The caller owns the existing queue and serializes request/begin/cancel.
class QueuedOwnerTurn {
    class Callback final : public ros::CallbackInterface {
    public:
        explicit Callback(std::function<void()> turn) : turn_(std::move(turn)) {}
        CallResult call() override {
            turn_();
            return Success;
        }

    private:
        std::function<void()> turn_;
    };

public:
    QueuedOwnerTurn(ros::CallbackQueueInterface& queue, std::function<void()> turn)
        : queue_(queue), callback_(new Callback(std::move(turn))) {}
    QueuedOwnerTurn(const QueuedOwnerTurn&) = delete;
    QueuedOwnerTurn& operator=(const QueuedOwnerTurn&) = delete;
    void request() {
        turn_.request([this] { queue_.addCallback(callback_, id()); });
    }
    void beginTurn() { turn_.beginTurn(); }
    void cancel() {
        turn_.cancel([this] { queue_.removeByID(id()); });
    }

private:
    uint64_t id() const { return reinterpret_cast<uintptr_t>(this); }
    ros::CallbackQueueInterface& queue_;
    ros::CallbackInterfacePtr callback_;
    state_machine::QueuedTurn turn_;
};

template <class Time> class DomainDeadline {
public:
    DomainDeadline(ros::NodeHandle& nh, std::function<void()> notify) : notify_(std::move(notify)) {
        timer_ = nh.createTimer(ros::Duration(1), &DomainDeadline::fired, this, true, false);
    }
    DomainDeadline(const DomainDeadline&) = delete;
    DomainDeadline& operator=(const DomainDeadline&) = delete;
    template <class Delay> void arm(const Time& deadline, Delay&& delay) {
        arm_.arm(deadline, [&](const Time&) {
            timer_.setPeriod(delay(), true);
            timer_.start();
        });
    }
    void cancel() {
        // ROS stop is unconditional, even after the one-shot was consumed.
        timer_.stop();
        arm_.consumed();
    }

private:
    void fired(const ros::TimerEvent&) {
        arm_.consumed();
        notify_();
    }
    ros::Timer timer_;
    state_machine::DeadlineArm<Time> arm_;
    std::function<void()> notify_;
};

}  // namespace ros1_utils
