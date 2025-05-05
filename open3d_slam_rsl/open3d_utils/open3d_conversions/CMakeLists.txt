cmake_minimum_required(VERSION 3.5)
project(open3d_conversions)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --------------------------------------------------------------------------- #
#  Build flags                                                                #
# --------------------------------------------------------------------------- #
add_compile_options(
  -O3 -std=c++17 -mtune=native -funroll-loops -fopenmp
  -fopt-info-vec-optimized -fno-math-errno
)

# --------------------------------------------------------------------------- #
#  Catkin & 3rd-party dependencies                                            #
# --------------------------------------------------------------------------- #
set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  sensor_msgs
  open3d_catkin
  open3d_slam_msgs
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGE_DEPENDENCIES})
find_package(Eigen3 REQUIRED)

# --------------------------------------------------------------------------- #
#  Package definition                                                         #
# --------------------------------------------------------------------------- #
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES open3d_conversions
  CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS EIGEN3
)

# --------------------------------------------------------------------------- #
#  Build library                                                              #
# --------------------------------------------------------------------------- #
add_library(open3d_conversions src/open3d_conversions.cpp)
add_dependencies(open3d_conversions ${catkin_EXPORTED_TARGETS})

target_include_directories(open3d_conversions
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(open3d_conversions
  ${catkin_LIBRARIES}
)

# --------------------------------------------------------------------------- #
#  Install                                                                    #
# --------------------------------------------------------------------------- #
install(TARGETS open3d_conversions
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/open3d_conversions/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
