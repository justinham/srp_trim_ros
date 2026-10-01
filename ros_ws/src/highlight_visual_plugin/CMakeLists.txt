cmake_minimum_required(VERSION 3.5)
project(highlight_visual_plugin)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

set(SOURCES 
  src/highlight_visual_plugin.cpp
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(${PROJECT_NAME} SHARED src/highlight_visual_plugin.cpp)

target_link_libraries(${PROJECT_NAME}
  ${GAZEBO_LIBRARIES}
  ${rclcpp_LIBRARIES}
  Qt5::Widgets
)

ament_target_dependencies(${PROJECT_NAME}
  "gazebo_ros"
  "rclcpp"
  "std_msgs"
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

ament_package()