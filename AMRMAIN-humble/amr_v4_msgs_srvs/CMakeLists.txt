cmake_minimum_required(VERSION 3.5)
project(amr_v4_msgs_srvs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/Motor.msg"
  "msg/Robot.msg"
  "msg/Pin.msg"
  "msg/Mode.msg"
  "msg/Error.msg"
  "msg/Charging.msg"
  "msg/Battery.msg"
  "msg/Lights.msg"
)
set(srv_files
  "srv/Linefollower.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

install(TARGETS
  DESTINATION lib/${PROJECT_NAME})

ament_package()
