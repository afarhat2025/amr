cmake_minimum_required(VERSION 3.5)
project(diffdrive_ctre)

#message("Your architecture is " ${CMAKE_LIBRARY_ARCHITECTURE})
#message("Your path " ${PROJECT_SOURCE_DIR})

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -Og")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")


# Prefer -lpthread
set(THREADS_PREFER_PTHREAD_FLAG ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(phoenix6 REQUIRED)
find_package(Threads REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(amr_v4_msgs_srvs)
#find_package(JetsonGPIO)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(gpiod REQUIRED)

add_compile_definitions(UNIT_LIB_ENABLE_IOSTREAM)
add_compile_definitions(UNIT_LIB_DISABLE_FMT)

## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/diffbot_system.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  transmission_interface
  hardware_interface
  pluginlib
  amr_v4_msgs_srvs
  std_msgs
  sensor_msgs
)

#ctre binary library
target_link_libraries(${PROJECT_NAME} phoenix6)
target_link_libraries(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_Phoenix.so)
target_link_libraries(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixCCI.so)
#target_link_libraries(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/lib/arm64/libCTRE_PhoenixTools.so)
target_link_libraries(${PROJECT_NAME} Threads::Threads)
#target_link_libraries(${PROJECT_NAME} JetsonGPIO::JetsonGPIO)
target_link_libraries(${PROJECT_NAME} gpiod)
#target_link_libraries(${PROJECT_NAME} jetgpio)
pluginlib_export_plugin_description_file(hardware_interface diffdrive_ctre.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)
ament_package()
