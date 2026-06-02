#!/usr/bin/env bash
set -euo pipefail

dpkg -s ros-noetic-xgc2-ros1-utils >/dev/null
rospack find ros1_utils >/dev/null
test -d /opt/ros/noetic/include/ros1_utils
test -d /opt/ros/noetic/include/controller_runtime
test -d /opt/ros/noetic/include/control_utils
test -f /opt/ros/noetic/lib/libros1_utils_ugv_identification.so
