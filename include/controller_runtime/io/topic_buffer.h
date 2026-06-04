#pragma once

#include <optional>

#include <ros/ros.h>

namespace controller_runtime {

template <typename MsgT> class TopicBuffer {
public:
    void update(const MsgT& msg, const ros::Time& msg_stamp,
                const ros::WallTime& receive_wall_time) {
        latest_ = msg;
        msg_stamp_ = msg_stamp;
        receive_wall_time_ = receive_wall_time;
        ++generation_;
        dirty_this_tick_ = true;
    }

    bool valid() const { return latest_.has_value(); }
    const std::optional<MsgT>& latest() const { return latest_; }
    uint64_t generation() const { return generation_; }
    bool dirtyThisTick() const { return dirty_this_tick_; }
    ros::Time msgStamp() const { return msg_stamp_; }
    ros::WallTime receiveWallTime() const { return receive_wall_time_; }

    bool hasNewSince(uint64_t last_seen_generation) const {
        return generation_ != last_seen_generation;
    }

    void clearTickDirty() { dirty_this_tick_ = false; }

private:
    std::optional<MsgT> latest_;
    uint64_t generation_{0};
    bool dirty_this_tick_{false};
    ros::Time msg_stamp_;
    ros::WallTime receive_wall_time_;
};

}  // namespace controller_runtime
