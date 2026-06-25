cmake_minimum_required(VERSION 3.0.2)
project(ros1_utils)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  DEPENDS EIGEN3
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
  catkin_add_gtest(${PROJECT_NAME}_headers_test
    test/ros1_utils_headers_test.cpp
  )
  if(TARGET ${PROJECT_NAME}_headers_test)
    target_compile_features(${PROJECT_NAME}_headers_test PUBLIC cxx_std_17)
    target_link_libraries(${PROJECT_NAME}_headers_test
      ${catkin_LIBRARIES}
    )
  endif()
endif()
