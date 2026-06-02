#pragma once

#include <cstdint>

#include <ros/ros.h>

namespace controller_runtime {

struct TickContext {
    uint64_t seq{0};

    ros::Time ros_now;
    ros::WallTime wall_now;

    bool ros_time_valid{false};
    bool ros_time_jumped_back{false};
    bool ros_time_jumped_forward{false};

    double wall_dt{0.0};
    double ros_dt{0.0};
};

}  // namespace controller_runtime
