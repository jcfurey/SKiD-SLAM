cmake_minimum_required(VERSION 3.8)
project(liorf)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g")

add_compile_options(-Wall -pthread)

# ament / ROS 2
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)

# third party
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(libnabo REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)
if(TARGET GeographicLib::GeographicLib)
  set(GeographicLib_LIBRARIES GeographicLib::GeographicLib)
endif()

# nabo exports the imported target ``libnabo::nabo`` since 1.1; older releases
# only set the plain ``nabo`` library name.
if(TARGET libnabo::nabo)
  set(NABO_LIBRARIES libnabo::nabo)
else()
  set(NABO_LIBRARIES ${libnabo_LIBRARIES})
endif()

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

set(ROS_DEPENDENCIES
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
)

include_directories(
  lib
  include
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

# Maximum Clique Solver (used by the map fusion node)
add_library(fast_max-clique_finder
  src/liorf-DiSO/third_parties/fast_max-clique_finder/src/findClique.cpp
  src/liorf-DiSO/third_parties/fast_max-clique_finder/src/findCliqueHeu.cpp
  src/liorf-DiSO/third_parties/fast_max-clique_finder/src/utils.cpp
  src/liorf-DiSO/third_parties/fast_max-clique_finder/src/graphIO.cpp
)
target_compile_options(fast_max-clique_finder PRIVATE -w)

add_library(solid
  src/liorf-DiSO/third_parties/SOLiD/solid.cpp
)
ament_target_dependencies(solid pcl_conversions sensor_msgs)
target_link_libraries(solid ${PCL_LIBRARIES})

add_library(scan_context
  src/liorf-DiSO/third_parties/scanContext/scanContext.cpp
)
ament_target_dependencies(scan_context pcl_conversions sensor_msgs)
target_link_libraries(scan_context ${PCL_LIBRARIES})

# <!-- liorf_yjz_lucky_boy -->
# Range Image Projection
add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp ${common_lib})
ament_target_dependencies(${PROJECT_NAME}_imageProjection ${ROS_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_imageProjection
  ${cpp_typesupport_target} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Mapping Optimization
add_executable(${PROJECT_NAME}_mapOptmization src/mapOptmization.cpp ${common_lib} include/Scancontext.cpp)
ament_target_dependencies(${PROJECT_NAME}_mapOptmization ${ROS_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_mapOptmization
  ${cpp_typesupport_target} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} OpenMP::OpenMP_CXX
  ${GeographicLib_LIBRARIES} gtsam)

# IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp ${common_lib})
ament_target_dependencies(${PROJECT_NAME}_imuPreintegration ${ROS_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_imuPreintegration
  ${cpp_typesupport_target} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

# Map Fusion (SOLiD)
add_executable(${PROJECT_NAME}_mapFusion src/liorf-DiSO/mapFusion_so.cpp)
ament_target_dependencies(${PROJECT_NAME}_mapFusion ${ROS_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_mapFusion
  ${cpp_typesupport_target} ${PCL_LIBRARIES} ${NABO_LIBRARIES}
  gtsam fast_max-clique_finder solid)

# Map Fusion (Scan Context)
add_executable(${PROJECT_NAME}_mapFusionSC src/liorf-DiSO/mapFusion_sc.cpp)
ament_target_dependencies(${PROJECT_NAME}_mapFusionSC ${ROS_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_mapFusionSC
  ${cpp_typesupport_target} ${PCL_LIBRARIES} ${NABO_LIBRARIES}
  gtsam fast_max-clique_finder scan_context)

#############
## Install ##
#############

install(TARGETS
  ${PROJECT_NAME}_imageProjection
  ${PROJECT_NAME}_mapOptmization
  ${PROJECT_NAME}_imuPreintegration
  ${PROJECT_NAME}_mapFusion
  ${PROJECT_NAME}_mapFusionSC
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS fast_max-clique_finder solid scan_context
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config src/liorf-DiSO/config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "doc" EXCLUDE
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime ${ROS_DEPENDENCIES})

ament_package()
