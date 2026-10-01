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

# GTSAM is only required by object_factor_graph_fusion_node.
# Uncomment if you restore that node AND have GTSAM built locally.
if(DEFINED ENV{HOME})
  list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/.local/gtsam-4.2")
endif()
find_package(GTSAM)

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

# === Maneuver models (re-added 2026-10-01) ===
# ControlGen / ControlGen2 / ControlGen3 are path trackers using
# pure-pursuit + NonlinCircleFit (MATLAB-generated C code).
#
# Each requires the MATLAB codegen library at src/codegen/lib/.
# If codegen/ is missing, the build will skip that target.
# To restore codegen: copy src/codegen/lib/{WayPtsAhead2,NonlinCircleFit}/
# from a backup (USB or git history).

# MATLAB codegen source list — shared across all 3 ControlGen variants
# Based on what codegen/lib/{WayPtsAhead2,NonlinCircleFit}/ actually contains.
# Run a `find codegen/lib -name '*.cpp' -o -name '*.h'` to regenerate this list
# if MATLAB Coder emits new files.
set(CONTROLGEN_CODEGEN_SOURCES
  # WayPtsAhead2 — pure-pursuit lookahead waypoint picker
  src/codegen/lib/WayPtsAhead2/WayPtsAhead2.cpp
  src/codegen/lib/WayPtsAhead2/WayPtsAhead2.h
  src/codegen/lib/WayPtsAhead2/WayPtsAhead2_data.cpp
  src/codegen/lib/WayPtsAhead2/WayPtsAhead2_data.h
  src/codegen/lib/WayPtsAhead2/WayPtsAhead2_initialize.cpp
  src/codegen/lib/WayPtsAhead2/WayPtsAhead2_initialize.h
  src/codegen/lib/WayPtsAhead2/WayPtsAhead2_terminate.cpp
  src/codegen/lib/WayPtsAhead2/WayPtsAhead2_terminate.h
  src/codegen/lib/WayPtsAhead2/WayPtsAhead2_types.h
  src/codegen/lib/WayPtsAhead2/spline.cpp
  src/codegen/lib/WayPtsAhead2/spline.h
  src/codegen/lib/WayPtsAhead2/rtwtypes.h

  # NonlinCircleFit — circular arc fitting
  src/codegen/lib/NonlinCircleFit/NonlinCircleFit.cpp
  src/codegen/lib/NonlinCircleFit/NonlinCircleFit.h
  src/codegen/lib/NonlinCircleFit/NonlinCircleFit_data.cpp
  src/codegen/lib/NonlinCircleFit/NonlinCircleFit_data.h
  src/codegen/lib/NonlinCircleFit/NonlinCircleFit_initialize.cpp
  src/codegen/lib/NonlinCircleFit/NonlinCircleFit_initialize.h
  src/codegen/lib/NonlinCircleFit/NonlinCircleFit_terminate.cpp
  src/codegen/lib/NonlinCircleFit/NonlinCircleFit_terminate.h
  src/codegen/lib/NonlinCircleFit/NonlinCircleFit_types.h

  # MATLAB runtime helpers (Inf / NaN checks)
  src/codegen/lib/NonlinCircleFit/rt_nonfinite.cpp
  src/codegen/lib/NonlinCircleFit/rt_nonfinite.h
  src/codegen/lib/NonlinCircleFit/rtGetInf.cpp
  src/codegen/lib/NonlinCircleFit/rtGetInf.h
  src/codegen/lib/NonlinCircleFit/rtGetNaN.cpp
  src/codegen/lib/NonlinCircleFit/rtGetNaN.h
)

# Build ControlGen, ControlGen2, ControlGen3 only if codegen is present
foreach(_codegen_src ${CONTROLGEN_CODEGEN_SOURCES})
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_codegen_src}")
    message(WARNING "codegen source missing: ${_codegen_src} \u2014 skipping ControlGen builds. Restore src/codegen/lib/ from backup to enable.")
    set(_CONTROLGEN_AVAILABLE FALSE)
    break()
  endif()
endforeach()
if(_CONTROLGEN_AVAILABLE)
  add_executable(ControlGen   src/ControlGenerator.cpp  ${CONTROLGEN_CODEGEN_SOURCES})
  add_executable(ControlGen2  src/ControlGenerator2.cpp ${CONTROLGEN_CODEGEN_SOURCES})
  add_executable(ControlGen3  src/ControlGenerator3.cpp ${CONTROLGEN_CODEGEN_SOURCES})
endif()

add_executable(Fusion src/fusion_node.cpp src/utils.cpp src/utils.hpp src/kalman.hpp src/kalman.cpp)
add_executable(kalman_test src/kalman_test.cpp src/kalman.cpp)

# GTSAM-backed factor-graph fusion (optional)
if(GTSAM_FOUND)
  add_executable(object_factor_graph_fusion_node
    src/object_factor_graph_fusion_node.cpp
  )
  target_link_libraries(object_factor_graph_fusion_node "${cpp_typesupport_target}" gtsam)
  set_target_properties(object_factor_graph_fusion_node PROPERTIES
    BUILD_RPATH "$<TARGET_FILE_DIR:gtsam>"
    INSTALL_RPATH "$<TARGET_FILE_DIR:gtsam>"
  )
else()
  message(WARNING "GTSAM not found \u2014 skipping object_factor_graph_fusion_node")
endif()

target_link_libraries(Fusion "${cpp_typesupport_target}")
target_link_libraries(kalman_test "${cpp_typesupport_target}")
if(_CONTROLGEN_AVAILABLE)
  target_link_libraries(ControlGen "${cpp_typesupport_target}")
  target_link_libraries(ControlGen2 "${cpp_typesupport_target}")
  target_link_libraries(ControlGen3 "${cpp_typesupport_target}")
endif()
if(GTSAM_FOUND)
  target_link_libraries(object_factor_graph_fusion_node "${cpp_typesupport_target}" gtsam)
endif()

ament_target_dependencies(Fusion rclcpp tf2)
ament_target_dependencies(kalman_test rclcpp tf2)
if(_CONTROLGEN_AVAILABLE)
  ament_target_dependencies(ControlGen rclcpp tf2)
  ament_target_dependencies(ControlGen2 rclcpp tf2)
  ament_target_dependencies(ControlGen3 rclcpp tf2)
endif()
if(GTSAM_FOUND)
  ament_target_dependencies(object_factor_graph_fusion_node rclcpp tf2)
endif()

# === Install set ===
set(_INSTALL_TARGETS Fusion kalman_test)
if(_CONTROLGEN_AVAILABLE)
  list(APPEND _INSTALL_TARGETS ControlGen ControlGen2 ControlGen3)
endif()
if(GTSAM_FOUND)
  list(APPEND _INSTALL_TARGETS object_factor_graph_fusion_node)
endif()
install(TARGETS ${_INSTALL_TARGETS} DESTINATION lib/${PROJECT_NAME})

ament_package()
