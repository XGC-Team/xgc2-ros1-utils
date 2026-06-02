cmake_minimum_required(VERSION 3.0.2)
project(ros1_utils)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros1_utils_ugv_identification
  CATKIN_DEPENDS roscpp
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_library(ros1_utils_ugv_identification
  src/ugv_identification.cpp
)
target_compile_features(ros1_utils_ugv_identification PUBLIC cxx_std_17)
target_compile_definitions(ros1_utils_ugv_identification PUBLIC CONTROL_UTILS_WITH_CERES)
target_link_libraries(ros1_utils_ugv_identification
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(TARGETS ros1_utils_ugv_identification
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_loop_controller_test
    test/loop_controller_test.cpp
  )
  if(TARGET ${PROJECT_NAME}_loop_controller_test)
    target_compile_features(${PROJECT_NAME}_loop_controller_test PUBLIC cxx_std_17)
    target_link_libraries(${PROJECT_NAME}_loop_controller_test
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(${PROJECT_NAME}_controller_runtime_test
    test/controller_runtime_test.cpp
  )
  if(TARGET ${PROJECT_NAME}_controller_runtime_test)
    target_compile_features(${PROJECT_NAME}_controller_runtime_test PUBLIC cxx_std_17)
    target_link_libraries(${PROJECT_NAME}_controller_runtime_test
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(${PROJECT_NAME}_butterworth_filter_test
    test/butterworth_filter_test.cpp
  )
  if(TARGET ${PROJECT_NAME}_butterworth_filter_test)
    target_compile_features(${PROJECT_NAME}_butterworth_filter_test PUBLIC cxx_std_17)
    target_link_libraries(${PROJECT_NAME}_butterworth_filter_test
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(${PROJECT_NAME}_butterworth_filter_integration_test
    test/butterworth_filter_integration_test.cpp
  )
  if(TARGET ${PROJECT_NAME}_butterworth_filter_integration_test)
    target_compile_features(${PROJECT_NAME}_butterworth_filter_integration_test PUBLIC cxx_std_17)
    target_link_libraries(${PROJECT_NAME}_butterworth_filter_integration_test
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(${PROJECT_NAME}_ugv_identification_test
    test/ugv_identification_test.cpp
  )
  if(TARGET ${PROJECT_NAME}_ugv_identification_test)
    target_compile_features(${PROJECT_NAME}_ugv_identification_test PUBLIC cxx_std_17)
    target_link_libraries(${PROJECT_NAME}_ugv_identification_test
      ros1_utils_ugv_identification
      ${catkin_LIBRARIES}
      ${CERES_LIBRARIES}
    )
  endif()

  catkin_add_gtest(${PROJECT_NAME}_ugv_identification_integration_test
    test/ugv_identification_integration_test.cpp
  )
  if(TARGET ${PROJECT_NAME}_ugv_identification_integration_test)
    target_compile_features(${PROJECT_NAME}_ugv_identification_integration_test PUBLIC cxx_std_17)
    target_link_libraries(${PROJECT_NAME}_ugv_identification_integration_test
      ros1_utils_ugv_identification
      ${catkin_LIBRARIES}
      ${CERES_LIBRARIES}
    )
  endif()
endif()
