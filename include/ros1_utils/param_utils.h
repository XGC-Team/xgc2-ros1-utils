#pragma once

#include <string>
#include <type_traits>
#include <vector>

#include <Eigen/Dense>
#include <ros/ros.h>

namespace ros1_utils {

template <typename T>
bool getParamWithLog(ros::NodeHandle& nh, const std::string& param_name, T& value,
                     const std::string& description) {
    const bool found = nh.getParam(param_name, value);

    if (!found) {
        ROS_WARN("[ROS1Utils] Parameter '%s' not found, using default", param_name.c_str());
    }

    if constexpr (std::is_same_v<T, bool>) {
        ROS_INFO("[ROS1Utils] %s: %s", description.c_str(), value ? "enabled" : "disabled");
    } else if constexpr (std::is_same_v<T, std::string>) {
        ROS_INFO("[ROS1Utils] %s: %s", description.c_str(), value.c_str());
    } else if constexpr (std::is_floating_point_v<T>) {
        ROS_INFO("[ROS1Utils] %s: %.2f", description.c_str(), value);
    } else {
        ROS_INFO("[ROS1Utils] %s: %d", description.c_str(), static_cast<int>(value));
    }

    return found;
}

inline bool getVector3ParamWithLog(ros::NodeHandle& nh, const std::string& param_name,
                                   Eigen::Vector3d& value, const std::string& description) {
    std::vector<double> raw;
    if (!nh.getParam(param_name, raw)) {
        ROS_WARN("[ROS1Utils] Parameter '%s' not found, using default", param_name.c_str());
        return false;
    }
    if (raw.size() != 3) {
        ROS_WARN("[ROS1Utils] Parameter '%s' must contain 3 values, using default",
                 param_name.c_str());
        return false;
    }

    value << raw[0], raw[1], raw[2];
    ROS_INFO("[ROS1Utils] %s: [%.3f, %.3f, %.3f]", description.c_str(), value.x(), value.y(),
             value.z());
    return true;
}

}  // namespace ros1_utils
