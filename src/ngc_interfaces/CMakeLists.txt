cmake_minimum_required(VERSION 3.8)
project(ngc_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}

"msg/Eta.msg"
"msg/Nu.msg"
"msg/NuDot.msg"
"msg/Tau.msg"
"msg/Wind.msg"
"msg/ThrusterSignals.msg"
"msg/Route.msg"
"msg/GNSS.msg"
"msg/HeadingDevice.msg"
"msg/SystemMode.msg"
"msg/OtterStatus.msg"
"msg/ButtonControl.msg"
"msg/Waypoint.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
