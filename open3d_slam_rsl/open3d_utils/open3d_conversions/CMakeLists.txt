cmake_minimum_required(VERSION 3.5.0)
project(open3d_conversions)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-O3)

set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  sensor_msgs
  open3d_catkin
  open3d_slam_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)

# Declare the catkin package.
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES open3d_conversions
  CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS EIGEN3
)

# Build the open3d_conversions library
add_library(open3d_conversions src/open3d_conversions.cpp)

# Use target_include_directories to ensure the include paths are set for the target.
target_include_directories(open3d_conversions
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(open3d_conversions ${catkin_LIBRARIES})

# Install targets and include directories
install(TARGETS open3d_conversions
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/open3d_conversions/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_open3d_conversions test/test_open3d_conversions.cpp)
  target_link_libraries(test_open3d_conversions open3d_conversions ${catkin_LIBRARIES})
endif ()
