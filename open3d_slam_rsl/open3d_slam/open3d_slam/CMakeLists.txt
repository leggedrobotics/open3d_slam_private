cmake_minimum_required(VERSION 3.5)
project(open3d_slam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-std=c++17 -O3 -mtune=native -flto -funroll-loops -fopenmp -fopt-info-vec-optimized -fno-math-errno)

option(ENABLE_PROFILING "Enable profiling with ProfilerScopeGuard" OFF)

if(ENABLE_PROFILING)
  add_definitions(-DENABLE_PROFILING)
endif()

set(SRC_FILES
  src/Mapper.cpp
  src/helpers.cpp
  src/time.cpp
  src/math.cpp
  src/Odometry.cpp
  src/Voxel.cpp
  src/croppers.cpp
  src/Submap.cpp
  src/SubmapCollection.cpp
  src/Transform.cpp
  src/TransformInterpolationBuffer.cpp
  src/AdjacencyMatrix.cpp
  src/PlaceRecognition.cpp
  src/OptimizationProblem.cpp
  src/SlamWrapper.cpp
  src/constraint_builders.cpp
  src/output.cpp
  src/MotionCompensation.cpp
  src/VoxelHashMap.cpp
  src/ScanToMapRegistration.cpp
  src/CloudRegistration.cpp
)

set(CATKIN_PACKAGE_DEPENDENCIES
  open3d_catkin
  # libpointmatcher
  # pointmatcher_ros
  open3d_conversions
  eigen_conversions
  nav_msgs
  geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost QUIET REQUIRED COMPONENTS serialization filesystem system program_options)
find_package(TBB REQUIRED)


find_package(OpenMP QUIET)
if (OpenMP_FOUND)
  add_compile_options(${OpenMP_CXX_FLAGS})
  add_definitions(-Dopen3d_slam_OPENMP_FOUND=${OpenMP_FOUND})
endif()

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

get_filename_component(_THIS_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
file(REAL_PATH "${_THIS_DIR}/../../thirdparty/small_gicp/include" SMALL_GICP_INCLUDE_DIR)


catkin_package(
  INCLUDE_DIRS 
    include 
    ${SMALL_GICP_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS} 
    ${Boost_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${OpenMP_CXX_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${SRC_FILES}
)

# Use target_include_directories to ensure that small_gicp is available during compilation.
target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${SMALL_GICP_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${OpenMP_CXX_INCLUDE_DIRS}
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
  ${OpenMP_CXX_LIBRARIES}
  TBB::tbb
)

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_clang_tooling(
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_FIX
    #CT_ATTACH_TO_ALL # Uncomment to attach to all targets during build, takes time
  )
endif(cmake_clang_tools_FOUND)
