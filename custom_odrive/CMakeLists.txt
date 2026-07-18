cmake_minimum_required(VERSION 3.8)
project(custom_odrive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlMessage.msg"
  "msg/ControllerStatus.msg"
  "msg/ODriveStatus.msg"
  "srv/AxisState.srv"
)
ament_export_dependencies(rosidl_default_runtime)

include_directories(include)
include_directories(../odrive_base/include)

add_executable(custom_odrive_node
  ../odrive_base/src/epoll_event_loop.cpp
  ../odrive_base/src/socket_can.cpp
  src/custom_odrive_node.cpp
  src/main.cpp
  include/custom_odrive_node.hpp)

ament_target_dependencies(custom_odrive_node
  rclcpp
  std_srvs
)

target_compile_features(custom_odrive_node PRIVATE cxx_std_20)

install(
  TARGETS custom_odrive_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(custom_odrive_node "${cpp_typesupport_target}")

ament_package()
