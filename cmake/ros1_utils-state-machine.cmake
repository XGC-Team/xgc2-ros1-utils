# Resolve the public SDK dependency at the consumer's installed prefix, not at
# this package's build path. Catkin collects these variables after CFG_EXTRAS.
find_package(xgc2_state_machine 0.2.0 REQUIRED CONFIG COMPONENTS workers)
list(APPEND ros1_utils_INCLUDE_DIRS ${XGC2_STATE_MACHINE_INCLUDE_DIRS})
list(APPEND ros1_utils_LIBRARIES xgc2_state_machine::workers)
list(REMOVE_DUPLICATES ros1_utils_INCLUDE_DIRS)
list(REMOVE_DUPLICATES ros1_utils_LIBRARIES)
