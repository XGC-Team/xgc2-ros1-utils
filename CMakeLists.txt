cmake_minimum_required(VERSION 3.0.2)
project(ros1_utils)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosgraph_msgs
)
find_package(Eigen3 REQUIRED)
find_package(xgc2_state_machine 0.2.0 REQUIRED CONFIG COMPONENTS workers)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rosgraph_msgs
  DEPENDS EIGEN3
  CFG_EXTRAS ros1_utils-state-machine.cmake
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(${PROJECT_NAME}_headers_test
    test/ros1_utils_headers_test.cpp
  )
  if(TARGET ${PROJECT_NAME}_headers_test)
    target_compile_features(${PROJECT_NAME}_headers_test PUBLIC cxx_std_17)
    target_link_libraries(${PROJECT_NAME}_headers_test
      ${catkin_LIBRARIES}
      xgc2_state_machine::workers
    )
  endif()
  add_executable(${PROJECT_NAME}_owner_scheduling_test test/owner_scheduling_test.cpp)
  target_compile_features(${PROJECT_NAME}_owner_scheduling_test PRIVATE cxx_std_17)
  target_include_directories(${PROJECT_NAME}_owner_scheduling_test PRIVATE ${GTEST_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME}_owner_scheduling_test
    ${catkin_LIBRARIES} ${GTEST_LIBRARIES} xgc2_state_machine::workers)
  add_rostest(test/owner_scheduling.test DEPENDENCIES ${PROJECT_NAME}_owner_scheduling_test)
endif()
