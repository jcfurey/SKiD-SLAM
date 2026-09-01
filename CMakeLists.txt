cmake_minimum_required(VERSION 3.24)
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
find_package(TBB REQUIRED)
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

# The paper registration pipeline is intentionally built from pinned
# submodules.  KISS-Matcher normally downloads ROBIN during configure, and
# ROBIN in turn downloads PMC and xenium.  Define those targets locally first
# so a recursive clone can be configured without network access.
set(SKID_THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
function(skid_require_submodule relative_path marker)
  if(NOT EXISTS "${SKID_THIRD_PARTY_DIR}/${relative_path}/${marker}")
    message(FATAL_ERROR
      "The ${relative_path} submodule is missing or incomplete. "
      "Run: git submodule update --init --recursive")
  endif()
endfunction()

skid_require_submodule(kiss_matcher cpp/kiss_matcher/CMakeLists.txt)
skid_require_submodule(small_gicp CMakeLists.txt)
skid_require_submodule(robin CMakeLists.txt)
skid_require_submodule(pmc CMakeLists.txt)
skid_require_submodule(xenium CMakeLists.txt)

set(PMC_BUILD_SHARED OFF CACHE BOOL "" FORCE)
add_subdirectory("${SKID_THIRD_PARTY_DIR}/pmc" third_party/pmc EXCLUDE_FROM_ALL)
set_property(TARGET pmc PROPERTY POSITION_INDEPENDENT_CODE ON)
target_compile_options(pmc PRIVATE -w)

add_library(xenium INTERFACE)
target_include_directories(xenium SYSTEM INTERFACE
  $<BUILD_INTERFACE:${SKID_THIRD_PARTY_DIR}/xenium>
)

add_library(skid_robin STATIC
  third_party/robin/src/core.cpp
  third_party/robin/src/graph.cpp
  third_party/robin/src/graph_core.cpp
  third_party/robin/src/graph_io.cpp
  third_party/robin/src/graph_solvers.cpp
  third_party/robin/src/pkc.cpp
  third_party/robin/src/problems.cpp
  third_party/robin/src/robin.cpp
  third_party/robin/src/utils.cpp
)
add_library(robin::robin ALIAS skid_robin)
set_property(TARGET skid_robin PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(skid_robin SYSTEM PUBLIC
  $<BUILD_INTERFACE:${SKID_THIRD_PARTY_DIR}/robin/include>
)
target_link_libraries(skid_robin
  PUBLIC Eigen3::Eigen xenium
  PRIVATE OpenMP::OpenMP_CXX pmc
)
target_compile_options(skid_robin PRIVATE -w)

find_library(SKID_LZ4_LIBRARY NAMES lz4 REQUIRED)
add_library(kiss_matcher_core STATIC
  third_party/kiss_matcher/cpp/kiss_matcher/core/kiss_matcher/ROBINMatching.cpp
  third_party/kiss_matcher/cpp/kiss_matcher/core/kiss_matcher/FasterPFH.cpp
  third_party/kiss_matcher/cpp/kiss_matcher/core/kiss_matcher/KISSMatcher.cpp
  third_party/kiss_matcher/cpp/kiss_matcher/core/kiss_matcher/GncSolver.cpp
)
add_library(kiss_matcher::kiss_matcher_core ALIAS kiss_matcher_core)
set_property(TARGET kiss_matcher_core PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(kiss_matcher_core SYSTEM PUBLIC
  $<BUILD_INTERFACE:${SKID_THIRD_PARTY_DIR}/kiss_matcher/cpp/kiss_matcher/core>
)
target_link_libraries(kiss_matcher_core
  PUBLIC Eigen3::Eigen robin::robin TBB::tbb OpenMP::OpenMP_CXX
  PRIVATE ${SKID_LZ4_LIBRARY}
)
target_compile_options(kiss_matcher_core PRIVATE -w)

add_library(small_gicp STATIC
  third_party/small_gicp/src/small_gicp/registration/registration.cpp
  third_party/small_gicp/src/small_gicp/registration/registration_helper.cpp
)
set_property(TARGET small_gicp PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(small_gicp SYSTEM PUBLIC
  $<BUILD_INTERFACE:${SKID_THIRD_PARTY_DIR}/small_gicp/include>
)
target_link_libraries(small_gicp
  PUBLIC Eigen3::Eigen OpenMP::OpenMP_CXX
)
target_compile_options(small_gicp PRIVATE -w)

###############
## Interfaces #
###############

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CloudInfo.msg"
  "msg/ContextInfo.msg"
  "msg/LoopConstraint.msg"
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

add_library(skid_registration STATIC src/skid_registration.cpp)
target_include_directories(skid_registration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(skid_registration
  PUBLIC Eigen3::Eigen
  PRIVATE kiss_matcher::kiss_matcher_core small_gicp
)

add_library(skid_pose_uncertainty STATIC src/skid_pose_uncertainty.cpp)
target_include_directories(skid_pose_uncertainty PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(skid_pose_uncertainty PUBLIC Eigen3::Eigen gtsam)

# SOLiD place recognition shared by intra- and inter-robot loop detection.
add_library(skid_loop_detection STATIC src/skid_loop_detection.cpp)
target_include_directories(skid_loop_detection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(skid_loop_detection PUBLIC Eigen3::Eigen)

# Isolates SOLiD's global PointType/PointXYZIRPYT declarations to one
# translation unit so the mapping node can build descriptors too.
add_library(skid_solid_descriptor STATIC src/skid_solid_descriptor.cpp)
target_include_directories(skid_solid_descriptor
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/liorf-DiSO/third_parties
)
target_link_libraries(skid_solid_descriptor
  PUBLIC skid_loop_detection ${PCL_LIBRARIES}
  PRIVATE solid
)

# <!-- liorf_yjz_lucky_boy -->
# Range Image Projection
ament_auto_add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp ${common_lib})
target_link_libraries(${PROJECT_NAME}_imageProjection
  ${cpp_typesupport_target} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Mapping Optimization
ament_auto_add_executable(${PROJECT_NAME}_mapOptmization src/mapOptmization.cpp ${common_lib})
target_link_libraries(${PROJECT_NAME}_mapOptmization
  ${cpp_typesupport_target} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} OpenMP::OpenMP_CXX
  ${GeographicLib_LIBRARIES} gtsam skid_pose_uncertainty
  skid_registration skid_loop_detection skid_solid_descriptor)

# IMU Preintegration
ament_auto_add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp ${common_lib})
target_link_libraries(${PROJECT_NAME}_imuPreintegration
  ${cpp_typesupport_target} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}
  ${GeographicLib_LIBRARIES} gtsam)

# Map Fusion (SOLiD)
ament_auto_add_executable(${PROJECT_NAME}_mapFusion src/liorf-DiSO/mapFusion_so.cpp)
target_link_libraries(${PROJECT_NAME}_mapFusion
  ${cpp_typesupport_target} ${PCL_LIBRARIES} ${NABO_LIBRARIES}
  gtsam fast_max-clique_finder solid skid_registration skid_pose_uncertainty
  skid_loop_detection)

# Map Fusion (Scan Context)
ament_auto_add_executable(${PROJECT_NAME}_mapFusionSC src/liorf-DiSO/mapFusion_sc.cpp)
target_link_libraries(${PROJECT_NAME}_mapFusionSC
  ${cpp_typesupport_target} ${PCL_LIBRARIES} ${NABO_LIBRARIES}
  gtsam fast_max-clique_finder scan_context skid_pose_uncertainty)

#########
## Test #
#########

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_observable_scan_match
    test/test_observable_scan_match.cpp)
  target_include_directories(test_observable_scan_match PRIVATE include)
  target_link_libraries(test_observable_scan_match Eigen3::Eigen)

  ament_add_gtest(test_geographic_frames
    test/test_geographic_frames.cpp)
  target_include_directories(test_geographic_frames PRIVATE include)
  target_link_libraries(test_geographic_frames
    Eigen3::Eigen ${GeographicLib_LIBRARIES})

  ament_add_gtest(test_skid_registration
    test/test_skid_registration.cpp)
  target_link_libraries(test_skid_registration skid_registration)

  ament_add_gtest(test_skid_pose_uncertainty
    test/test_skid_pose_uncertainty.cpp)
  target_link_libraries(test_skid_pose_uncertainty skid_pose_uncertainty)

  ament_add_gtest(test_loop_constraint_utils
    test/test_loop_constraint_utils.cpp)
  target_link_libraries(test_loop_constraint_utils
    ${cpp_typesupport_target} skid_pose_uncertainty)

  ament_add_gtest(test_skid_loop_detection
    test/test_skid_loop_detection.cpp)
  target_link_libraries(test_skid_loop_detection skid_loop_detection)

  ament_add_gtest(test_skid_solid_descriptor
    test/test_skid_solid_descriptor.cpp)
  target_link_libraries(test_skid_solid_descriptor skid_solid_descriptor)

  ament_add_gtest(test_skid_registration_params
    test/test_skid_registration_params.cpp)
  target_link_libraries(test_skid_registration_params skid_registration)
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
