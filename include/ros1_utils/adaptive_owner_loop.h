#pragma once

#include <ros/callback_queue.h>
#include <ros/poll_manager.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <state_machine/owner_loop.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <utility>

namespace ros1_utils {

class AdaptiveOwnerLoop {
    class Queue : public ros::CallbackQueue {
    public:
        explicit Queue(std::shared_ptr<state_machine::OwnerWake> wake) : wake_(std::move(wake)) {}
        void addCallback(const ros::CallbackInterfacePtr& callback, uint64_t id = 0) override {
            ros::CallbackQueue::addCallback(callback, id);
            wake_->notify();
        }
        void dispatchOne() { callOne(ros::WallDuration(0)); }
        bool empty() { return isEmpty(); }

    private:
        std::shared_ptr<state_machine::OwnerWake> wake_;
    };
    struct ShutdownConnection {
        std::shared_ptr<state_machine::OwnerWake> wake;
        ~ShutdownConnection() { wake->stop(); }
    };

public:
    AdaptiveOwnerLoop()
        : wake_(std::make_shared<state_machine::OwnerWake>()), queue_(wake_), loop_(*wake_) {}
    AdaptiveOwnerLoop(const AdaptiveOwnerLoop&) = delete;
    AdaptiveOwnerLoop& operator=(const AdaptiveOwnerLoop&) = delete;
    ~AdaptiveOwnerLoop() {
        clock_.shutdown();
        shutdown_.disconnect();
        queue_.disable();
        queue_.clear();
    }
    // Attach before subscriptions; destroy after the existing producers stop.
    void attach(ros::NodeHandle& nh, bool domain_clock = true) {
        nh.setCallbackQueue(&queue_);
        simulated_ = domain_clock && ros::Time::isSimTime();
        domain_now_ = ros::Time::now().toSec();
        if (simulated_) {
            // Use the clock sample actually delivered on this owner's queue.
            clock_ = nh.subscribe<rosgraph_msgs::Clock>(
                "/clock", 1, [this](const rosgraph_msgs::Clock::ConstPtr& sample) {
                    domain_now_ = sample->clock.toSec();
                });
        }
        // Noetic shutdown disconnects all slots. Token destruction observes it
        // without another thread or periodic wake, even after ros::ok changes.
        auto token = std::make_shared<ShutdownConnection>();
        token->wake = wake_;
        shutdown_ = ros::PollManager::instance()->addPollThreadListener([token] {});
    }
    ros::CallbackQueueInterface* queue() { return &queue_; }
    bool externalClock() const { return simulated_; }
    double now() const { return simulated_ ? domain_now_ : ros::Time::now().toSec(); }
    static double wallNow() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }
    void notify() { wake_->notify(); }
    template <class Work, class Demand> void run(Work&& work, Demand&& demand) {
        loop_.run(
            queue_, std::forward<Work>(work), std::forward<Demand>(demand),
            [this](const auto& next) {
                return state_machine::projectOwnerDeadline(
                    next, simulated_, [this] { return now(); }, [] { return wallNow(); });
            },
            [] { return ros::ok() && !ros::isShuttingDown(); });
    }

private:
    std::shared_ptr<state_machine::OwnerWake> wake_;
    Queue queue_;
    state_machine::OwnerLoop loop_;
    boost::signals2::connection shutdown_;
    ros::Subscriber clock_;
    bool simulated_{false};
    double domain_now_{0};
};

}  // namespace ros1_utils
