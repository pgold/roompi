cmake_minimum_required(VERSION 3.5)
project(roompi_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PanTilt.msg"
)

ament_package()
