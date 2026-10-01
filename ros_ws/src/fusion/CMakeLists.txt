cmake_minimum_required(VERSION 3.8)
project(fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

# GTSAM is no longer required since object_factor_graph_fusion_node was removed.
# Re-enable if you re-add the GTSAM factor-graph fusion node:
# if(DEFINED ENV{HOME})
#   list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/.local/gtsam-4.2")
# endif()
# find_package(GTSAM REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rosidl_default_generators REQUIRED)
set(msg_files
  "msg/VehicleDynamics.msg"
  "msg/TrackedObject.msg"
  "msg/Track.msg"
  "msg/RemoteObjectState.msg"
  "msg/Vel.msg"
  "msg/Acc.msg"
  "msg/GPS.msg"
  "msg/TrackedObjectList.msg"
  "msg/VehicleCommand.msg"
  "msg/TripleVectorWps.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs
  DEPENDENCIES builtin_interfaces
  DEPENDENCIES visualization_msgs
  DEPENDENCIES sensor_msgs
  DEPENDENCIES rosgraph_msgs
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

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
endif()

find_package(rclcpp REQUIRED)
include_directories(./include/eigen-3.4.0)
include_directories(./include/hungarian)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(Fusion src/fusion_node.cpp src/utils.cpp src/utils.hpp src/kalman.hpp src/kalman.cpp)

# === Trimmed (2026-10-01): only the Fusion executable remains.
# Previously also built: ControlGen, ControlGen2, ControlGen3 (maneuver models),
# kalman_test (unit test), object_factor_graph_fusion_node (GTSAM offline fusion).

target_link_libraries(Fusion "${cpp_typesupport_target}")

ament_target_dependencies(Fusion rclcpp tf2)

install(TARGETS
  Fusion
  DESTINATION lib/${PROJECT_NAME})

ament_package()
