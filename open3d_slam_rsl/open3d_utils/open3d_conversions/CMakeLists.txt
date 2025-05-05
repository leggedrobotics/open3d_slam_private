##############################################################################
#  open3d_conversions ‒ CMakeLists.txt                                       #
##############################################################################

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
    # pointmatcher_ros
    open3d_catkin
    open3d_slam_msgs
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGE_DEPENDENCIES})
find_package(Eigen3 REQUIRED)

# --------------------------------------------------------------------------- #
#  Locate libfmt installed by open3d_catkin                                   #
#  ↳ <ws>/devel/lib/libfmt.a  and headers in <ws>/devel/include               #
# --------------------------------------------------------------------------- #
if(NOT CATKIN_DEVEL_PREFIX)
    message(FATAL_ERROR
        "CATKIN_DEVEL_PREFIX is not defined – this should never happen when "
        "building with catkin_tools.")
endif()

# Fix path resolution to point to <workspace>/devel
get_filename_component(WS_DEVEL_DIR "${CATKIN_DEVEL_PREFIX}/../.." ABSOLUTE)

set(FMT_LIBRARY     "${WS_DEVEL_DIR}/lib/libfmt.a")
set(FMT_INCLUDE_DIR "${WS_DEVEL_DIR}/include")


message(STATUS "CATKIN_DEVEL_PREFIX : ${CATKIN_DEVEL_PREFIX}")
message(STATUS "Workspace devel dir : ${WS_DEVEL_DIR}")
message(STATUS "fmt library         : ${FMT_LIBRARY}")
message(STATUS "fmt headers         : ${FMT_INCLUDE_DIR}")

if(NOT EXISTS "${FMT_LIBRARY}")
    message(FATAL_ERROR
        "libfmt.a not found at ${FMT_LIBRARY}\n"
        "Did open3d_catkin build/install successfully?")
endif()

# --------------------------------------------------------------------------- #
#  Package definition                                                         #
# --------------------------------------------------------------------------- #
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES    open3d_conversions
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
        ${FMT_INCLUDE_DIR}
)

target_link_libraries(open3d_conversions
    ${catkin_LIBRARIES}
    ${FMT_LIBRARY}
)

# --------------------------------------------------------------------------- #
#  Install                                                                    #
# --------------------------------------------------------------------------- #
install(TARGETS open3d_conversions
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/open3d_conversions/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
