#pragma once

#include <string>

#include <ros/ros.h>

namespace ros1_utils {

struct VrpnTrackerTopics {
    std::string robot_name;
    std::string pose_topic;
    std::string twist_topic;
};

inline std::string nodeNamespaceBasename(std::string node_namespace) {
    while (!node_namespace.empty() && node_namespace.front() == '/') {
        node_namespace.erase(0, 1);
    }
    while (!node_namespace.empty() && node_namespace.back() == '/') {
        node_namespace.pop_back();
    }
    const std::size_t slash_pos = node_namespace.rfind('/');
    if (slash_pos != std::string::npos) {
        node_namespace = node_namespace.substr(slash_pos + 1);
    }
    return node_namespace;
}

inline std::string stripTrailingSlash(std::string value) {
    while (value.size() > 1 && value.back() == '/') {
        value.pop_back();
    }
    return value;
}

inline VrpnTrackerTopics makeVrpnTrackerTopics(ros::NodeHandle& nh,
                                               ros::NodeHandle& private_nh,
                                               const std::string& default_vrpn_client_node = "/vrpn_client_node") {
    std::string robot_name = nodeNamespaceBasename(nh.getNamespace());
    private_nh.param<std::string>("robot_name", robot_name, robot_name);
    robot_name = nodeNamespaceBasename(robot_name);

    std::string vrpn_client_node;
    private_nh.param<std::string>("vrpn_client_node", vrpn_client_node, default_vrpn_client_node);
    vrpn_client_node = stripTrailingSlash(vrpn_client_node.empty() ? default_vrpn_client_node : vrpn_client_node);

    VrpnTrackerTopics topics;
    topics.robot_name = robot_name;
    topics.pose_topic = vrpn_client_node + "/" + topics.robot_name + "/pose";
    topics.twist_topic = vrpn_client_node + "/" + topics.robot_name + "/twist";
    private_nh.param<std::string>("vrpn_pose_topic", topics.pose_topic, topics.pose_topic);
    private_nh.param<std::string>("vrpn_twist_topic", topics.twist_topic, topics.twist_topic);
    return topics;
}

}  // namespace ros1_utils
