cmake_minimum_required(VERSION 3.8)
project(liorf)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g")

add_compile_options(-Wall -pthread)

# ament_cmake_auto discovers every ROS dependency declared in package.xml and
# wires their include directories and libraries into the targets below, so the
# dependency list lives in exactly one place. Names passed as REQUIRED fail the
# configure step when missing; anything else is skipped silently, which is why
# the packages below are listed explicitly.
find_package(ament_cmake_auto REQUIRED)

# PCL pulls in VTK, whose Resolute export references JsonCpp::JsonCpp without
# loading jsoncpp's package config first. Define the imported target before any
# ROS dependency can transitively discover PCL/VTK.
find_package(jsoncpp CONFIG REQUIRED)

ament_auto_find_build_dependencies(REQUIRED
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_conversions
  cv_bridge
  rosidl_default_generators
)

# Dependencies whose CMake package name differs from the rosdep key in
# package.xml (GTSAM vs gtsam, Eigen3 vs eigen, ...), so ament_auto cannot
# resolve them. These are found and linked explicitly.
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)
if(TARGET GeographicLib::GeographicLib)
  set(GeographicLib_LIBRARIES GeographicLib::GeographicLib)
endif()

# Lyrical does not currently publish a libnabo binary. Keep the dependency
# pinned with the package and build only its C++ library. A clear error here is
# more useful than a later find_package or missing-header failure when someone
# cloned without --recursive.
set(LIBNABO_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/libnabo")
if(NOT EXISTS "${LIBNABO_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "The libnabo submodule is missing. Run: git submodule update --init --recursive")
endif()
set(LIBNABO_BUILD_DOXYGEN OFF CACHE BOOL "" FORCE)
set(LIBNABO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(LIBNABO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(LIBNABO_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
set(SHARED_LIBS OFF CACHE BOOL "" FORCE)
add_subdirectory("${LIBNABO_SOURCE_DIR}" third_party/libnabo EXCLUDE_FROM_ALL)
set(NABO_LIBRARIES libnabo::nabo)

###############
## Interfaces #
###############

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CloudInfo.msg"
  "msg/ContextInfo.msg"
  "srv/SaveMap.srv"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs sensor_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ament_auto_add_* already puts ``include`` on the include path.
include_directories(
  lib
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  src/liorf-DiSO/third_parties
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

file(GLOB common_lib "lib/*.cpp")

###########
## Build ##
###########

# Static so the executables stay self-contained; ament_auto links every library
# added here into every executable, and an archive contributes nothing to a
# binary that does not reference it.

# Maximum Clique Solver (used by the map fusion nodes)
ament_auto_add_library(fast_max-clique_finder STATIC
  src/liorf-DiSO/third_parties/fast_max-clique_finder/src/findClique.cpp
  src/liorf-DiSO/third_parties/fast_max-clique_finder/src/findCliqueHeu.cpp
  src/liorf-DiSO/third_parties/fast_max-clique_finder/src/utils.cpp
  src/liorf-DiSO/third_parties/fast_max-clique_finder/src/graphIO.cpp
)
target_compile_options(fast_max-clique_finder PRIVATE -w)

ament_auto_add_library(solid STATIC
  src/liorf-DiSO/third_parties/SOLiD/solid.cpp
)
target_link_libraries(solid ${PCL_LIBRARIES})

ament_auto_add_library(scan_context STATIC
  src/liorf-DiSO/third_parties/scanContext/scanContext.cpp
)
target_link_libraries(scan_context ${PCL_LIBRARIES})

# <!-- liorf_yjz_lucky_boy -->
# Range Image Projection
ament_auto_add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp ${common_lib})
target_link_libraries(${PROJECT_NAME}_imageProjection
  ${cpp_typesupport_target} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Mapping Optimization
ament_auto_add_executable(${PROJECT_NAME}_mapOptmization src/mapOptmization.cpp ${common_lib} include/Scancontext.cpp)
target_link_libraries(${PROJECT_NAME}_mapOptmization
  ${cpp_typesupport_target} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} OpenMP::OpenMP_CXX
  ${GeographicLib_LIBRARIES} gtsam)

# IMU Preintegration
ament_auto_add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp ${common_lib})
target_link_libraries(${PROJECT_NAME}_imuPreintegration
  ${cpp_typesupport_target} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

# Map Fusion (SOLiD)
ament_auto_add_executable(${PROJECT_NAME}_mapFusion src/liorf-DiSO/mapFusion_so.cpp)
target_link_libraries(${PROJECT_NAME}_mapFusion
  ${cpp_typesupport_target} ${PCL_LIBRARIES} ${NABO_LIBRARIES}
  gtsam fast_max-clique_finder solid)

# Map Fusion (Scan Context)
ament_auto_add_executable(${PROJECT_NAME}_mapFusionSC src/liorf-DiSO/mapFusion_sc.cpp)
target_link_libraries(${PROJECT_NAME}_mapFusionSC
  ${cpp_typesupport_target} ${PCL_LIBRARIES} ${NABO_LIBRARIES}
  gtsam fast_max-clique_finder scan_context)

#########
## Test #
#########

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_observable_scan_match
    test/test_observable_scan_match.cpp)
  target_include_directories(test_observable_scan_match PRIVATE include)
  target_link_libraries(test_observable_scan_match Eigen3::Eigen)
endif()

#############
## Install ##
#############

# Not INSTALL_TO_SHARE: config/doc holds the README media and must be excluded.
install(DIRECTORY launch config src/liorf-DiSO/config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "doc" EXCLUDE
)

# Runtime dependency of the generated interfaces; it is an exec_depend, so
# ament_auto_package() does not pick it up on its own.
ament_export_dependencies(rosidl_default_runtime)

# ament_auto_package installs the executables to lib/${PROJECT_NAME}, the
# libraries to lib, and the headers to include/${PROJECT_NAME}.
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
