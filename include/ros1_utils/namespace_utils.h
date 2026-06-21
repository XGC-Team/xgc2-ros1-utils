#pragma once

#include <cctype>
#include <string>

#include <ros/ros.h>

namespace ros1_utils {

inline std::string nameFromNamespacePrefix(const std::string& node_namespace,
                                           const std::string& prefix) {
    const std::size_t pos = node_namespace.rfind(prefix);
    if (pos == std::string::npos) {
        return "";
    }

    std::size_t end = pos + prefix.size();
    while (end < node_namespace.size() &&
           std::isdigit(static_cast<unsigned char>(node_namespace[end]))) {
        ++end;
    }
    if (end == pos + prefix.size()) {
        return "";
    }
    return node_namespace.substr(pos + 1, end - pos - 1);
}

inline std::string currentNameFromNamespacePrefix(const std::string& prefix) {
    return nameFromNamespacePrefix(ros::this_node::getNamespace(), prefix);
}

}  // namespace ros1_utils
