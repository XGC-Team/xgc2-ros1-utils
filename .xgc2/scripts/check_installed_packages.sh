#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
PREFIX="/opt/ros/${ROS_DISTRO}"

dpkg -s ros-noetic-xgc2-ros1-utils >/dev/null
dpkg -s libxgc2-math-dev >/dev/null
rospack find ros1_utils >/dev/null
test -f "${PREFIX}/include/ros1_utils/loop_controller.h"
test -f "${PREFIX}/include/ros1_utils/namespace_utils.h"
test -f "${PREFIX}/include/ros1_utils/param_utils.h"
test -f "${PREFIX}/include/ros1_utils/topic_stats.h"
test -f "${PREFIX}/include/ros1_utils/vrpn_topics.h"
test -f "${PREFIX}/include/controller_runtime/control/controller_interface.h"
test -f "${PREFIX}/include/controller_runtime/event/event_queue.h"
test -f "${PREFIX}/include/controller_runtime/io/input_store.h"
test -f "${PREFIX}/include/controller_runtime/io/topic_buffer.h"
test -f "${PREFIX}/include/controller_runtime/scheduler/module_scheduler.h"
test -f "${PREFIX}/include/controller_runtime/scheduler/task_gate.h"
test -f "${PREFIX}/include/controller_runtime/time/loop_controller.h"
test -f "${PREFIX}/include/controller_runtime/time/tick_context.h"
test -f "${PREFIX}/include/control_utils/butterworth_filter.h"
test -f "${PREFIX}/include/control_utils/ugv_identification.h"
test -f /usr/include/xgc2_math/filter/butterworth_filter.hpp
test -f "${PREFIX}/lib/libros1_utils_ugv_identification.so"

while IFS= read -r file; do
  if ! file -b "${file}" | grep -q '^ELF'; then
    continue
  fi
  if ! ldd "${file}" | awk '/not found/ {missing=1} END {exit missing ? 1 : 0}'; then
    echo "missing shared library dependency in ${file}" >&2
    ldd "${file}" >&2 || true
    exit 1
  fi
done < <(find "${PREFIX}/lib" -maxdepth 1 -type f -name 'libros1_utils_*.so' | sort -u)

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
#include <cmath>
#include <vector>

#include <ros/ros.h>

#include "control_utils/butterworth_filter.h"
#include "control_utils/ugv_identification.h"
#include "controller_runtime/control/controller_interface.h"
#include "controller_runtime/event/event_queue.h"
#include "controller_runtime/io/input_store.h"
#include "controller_runtime/io/topic_buffer.h"
#include "controller_runtime/scheduler/module_scheduler.h"
#include "controller_runtime/scheduler/task_gate.h"
#include "controller_runtime/time/loop_controller.h"
#include "controller_runtime/time/tick_context.h"
#include "ros1_utils/loop_controller.h"
#include "ros1_utils/namespace_utils.h"
#include "ros1_utils/param_utils.h"
#include "ros1_utils/topic_stats.h"
#include "ros1_utils/vrpn_topics.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros1_utils_downstream_smoke", ros::init_options::AnonymousName);

    control_utils::SecondOrderButterworthLowPass filter(5.0);
    const double filtered = filter.filter(1.0, 0.01);
    if (!std::isfinite(filtered)) {
        return 1;
    }

    const double wrapped = control_utils::ugv_identification::normalizeAngle(4.0);
    if (!std::isfinite(wrapped)) {
        return 2;
    }

    controller_runtime::TickContext tick;
    tick.seq = 1;
    tick.ros_now = ros::Time(1.0);
    tick.wall_now = ros::WallTime(1.0);
    tick.ros_time_valid = true;

    controller_runtime::TaskGate gate;
    controller_runtime::DirtySet dirty;
    if (!gate.ready(tick, dirty)) {
        return 3;
    }

    controller_runtime::EventQueue events;
    controller_runtime::RuntimeEvent event;
    event.id = 1;
    event.stamp = tick.ros_now.toSec();
    event.source = "downstream_smoke";
    events.push(event);
    if (events.drain().empty()) {
        return 4;
    }

    ros1_utils::LoopControllerOptions options;
    options.frequency_hz = 100.0;
    ros1_utils::LoopController loop(options);
    loop.requestStop();

    const std::string topic = ros1_utils::stripTrailingSlash("/vrpn_client_node/");
    if (topic != "/vrpn_client_node") {
        return 5;
    }

    if (ros1_utils::nameFromNamespacePrefix("/swarm/uav12/controller", "/uav") !=
        "uav12") {
        return 6;
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
