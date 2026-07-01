cmake_minimum_required(VERSION 3.20)

project(sort_backend
  VERSION 1.0.0
  DESCRIPTION "C++ implementation of Simple Online and Realtime Tracking (SORT) backend"
  LANGUAGES CXX)

# Default to a Release build if the user doesn't specify one
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type specified, defaulting to 'Release'.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

option(BUILD_EXAMPLE "Build the sort_main example (requires OpenCV)" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(hungarian REQUIRED)

# Create library
add_library(${PROJECT_NAME} STATIC
  src/kalman_filter.cpp
  src/kalman_box_tracker.cpp
  src/utils.cpp
  src/sort_backend.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Eigen3::Eigen
  PRIVATE
    hungarian::hungarian)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall
  -Wextra
  -Wpedantic)

# Optionally install the library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib)

# Install header files
install(DIRECTORY include/
  DESTINATION include)

# Export targets and dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 hungarian) # Build static library, so downstream packages need to find these

# Build example
if(BUILD_EXAMPLE)
  find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs imgproc)

  add_executable(${PROJECT_NAME}_example example/sort_main.cpp)

  target_include_directories(${PROJECT_NAME}_example PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

  target_link_libraries(${PROJECT_NAME}_example PRIVATE
    ${PROJECT_NAME}
    Eigen3::Eigen
    opencv_highgui
    opencv_imgcodecs
    opencv_imgproc)

  target_compile_features(${PROJECT_NAME}_example PRIVATE cxx_std_17)

  target_compile_options(${PROJECT_NAME}_example PRIVATE
    -Wall
    -Wextra
    -Wpedantic)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  set(test_sources
    test_utils
    test_kalman_filter
    test_kalman_box_tracker
    test_sort_backend)

  foreach(test_src ${test_sources})
    ament_add_gtest(${PROJECT_NAME}_${test_src} test/${test_src}.cpp)
    target_link_libraries(${PROJECT_NAME}_${test_src}
      ${PROJECT_NAME}
      Eigen3::Eigen)
  endforeach()
endif()

ament_package()
