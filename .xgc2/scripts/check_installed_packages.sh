#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-melodic}"
PREFIX="/opt/ros/${ROS_DISTRO}"

dpkg -s ros-melodic-xgc2-ros1-utils >/dev/null
rospack find ros1_utils >/dev/null
test -f "${PREFIX}/include/ros1_utils/namespace_utils.h"
test -f "${PREFIX}/include/ros1_utils/param_utils.h"
test -f "${PREFIX}/include/ros1_utils/time_utils.h"
test -f "${PREFIX}/include/ros1_utils/topic_stats.h"
test ! -e "${PREFIX}/include/ros1_utils/loop_controller.h"
test ! -e "${PREFIX}/include/ros1_utils/vrpn_topics.h"
test ! -e "${PREFIX}/include/controller_runtime/time/loop_controller.h"
test ! -e "${PREFIX}/include/control_utils/ugv_identification.h"
test ! -e "${PREFIX}/lib/libros1_utils_ugv_identification.so"

downstream_ws="$(mktemp -d)"
cleanup() {
  rm -rf "${downstream_ws}"
}
trap cleanup EXIT

mkdir -p "${downstream_ws}/src/ros1_utils_downstream_smoke/src"
cat > "${downstream_ws}/src/ros1_utils_downstream_smoke/package.xml" <<'EOF'
<?xml version="1.0"?>
<package format="2">
  <name>ros1_utils_downstream_smoke</name>
  <version>0.0.0</version>
  <description>Downstream compile smoke test for ros1_utils deb exports.</description>
  <maintainer email="apt@example.com">XGC2</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>ros1_utils</depend>
</package>
EOF

cat > "${downstream_ws}/src/ros1_utils_downstream_smoke/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.0.2)
project(ros1_utils_downstream_smoke)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ros1_utils
)

catkin_package()

add_executable(${PROJECT_NAME}
  src/main.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
EOF

cat > "${downstream_ws}/src/ros1_utils_downstream_smoke/src/main.cpp" <<'EOF'
#include <deque>

#include <ros/ros.h>

#include "ros1_utils/namespace_utils.h"
#include "ros1_utils/param_utils.h"
#include "ros1_utils/time_utils.h"
#include "ros1_utils/topic_stats.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros1_utils_downstream_smoke", ros::init_options::AnonymousName);

    if (ros1_utils::nameFromNamespacePrefix("/swarm/uav12/controller", "/uav") !=
        "uav12") {
        return 1;
    }

    ros1_utils::PositionQualityConfig quality_config;
    quality_config.window_size = 3;
    ros1_utils::PositionQualityDetector detector(quality_config);
    detector.process(1.0, 2.0, 3.0);
    const auto repeated = detector.process(1.0, 2.0, 3.0);
    if (repeated.frame_is_valid) {
        return 2;
    }

    const std::deque<double> dt_window{0.1, 0.2, 0.3};
    if (ros1_utils::TopicStatsManager::calculateJitter(dt_window) <= 0.0) {
        return 3;
    }
    if (ros1_utils::samplePeriodSec(true, 1.0, 1.25) != 0.25) {
        return 4;
    }

    return 0;
}
EOF

source "${PREFIX}/setup.bash"
cd "${downstream_ws}"
catkin_make \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG" \
  -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG"
source devel/setup.bash
./devel/lib/ros1_utils_downstream_smoke/ros1_utils_downstream_smoke

echo "Installed package check passed"
