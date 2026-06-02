#pragma once

#include <optional>
#include <string>

#include "controller_runtime/io/topic_buffer.h"
#include "controller_runtime/scheduler/task_gate.h"
#include "controller_runtime/time/tick_context.h"

namespace controller_runtime {

struct InputSnapshot {
    TickContext tick;
    uint64_t pose_generation{0};
    uint64_t twist_generation{0};
    uint64_t command_generation{0};
    DirtySet dirty;
};

template <typename PoseMsgT, typename TwistMsgT, typename CommandMsgT>
class InputStore {
public:
    void updatePose(const PoseMsgT& msg,
                    const ros::Time& msg_stamp,
                    const ros::WallTime& receive_wall_time) {
        pose_.update(msg, msg_stamp, receive_wall_time);
    }

    void updateTwist(const TwistMsgT& msg,
                     const ros::Time& msg_stamp,
                     const ros::WallTime& receive_wall_time) {
        twist_.update(msg, msg_stamp, receive_wall_time);
    }

    void updateCommand(const CommandMsgT& msg,
                       const ros::Time& msg_stamp,
                       const ros::WallTime& receive_wall_time) {
        command_.update(msg, msg_stamp, receive_wall_time);
    }

    InputSnapshot snapshot(const TickContext& ctx) const {
        InputSnapshot out;
        out.tick = ctx;
        out.pose_generation = pose_.generation();
        out.twist_generation = twist_.generation();
        out.command_generation = command_.generation();
        if (pose_.dirtyThisTick()) {
            out.dirty.insert("pose");
        }
        if (twist_.dirtyThisTick()) {
            out.dirty.insert("twist");
        }
        if (command_.dirtyThisTick()) {
            out.dirty.insert("command");
        }
        return out;
    }

    void endTick() {
        pose_.clearTickDirty();
        twist_.clearTickDirty();
        command_.clearTickDirty();
    }

    const TopicBuffer<PoseMsgT>& pose() const { return pose_; }
    const TopicBuffer<TwistMsgT>& twist() const { return twist_; }
    const TopicBuffer<CommandMsgT>& command() const { return command_; }

private:
    TopicBuffer<PoseMsgT> pose_;
    TopicBuffer<TwistMsgT> twist_;
    TopicBuffer<CommandMsgT> command_;
};

}  // namespace controller_runtime
