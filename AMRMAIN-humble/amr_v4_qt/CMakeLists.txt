cmake_minimum_required(VERSION 3.5)
project(amr_v4_qt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(amr_v4_msgs_srvs REQUIRED)
find_package(Qt5 COMPONENTS Core Quick Widgets Network UiTools REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rviz_common)
find_package(rviz_default_plugins)
find_package(yaml-cpp REQUIRED)

# Check for mongocxx and bsoncxx packages with pkg-config
pkg_check_modules(MONGOCXX REQUIRED libmongocxx>=3.7.0 libmongocxx<=3.8.0)
pkg_check_modules(BSONCXX REQUIRED libbsoncxx>=3.7.0 libbsoncxx<=3.8.0)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
  amr_v4_msgs_srvs
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR YES)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/main_gui.cpp
    src/main_screen.cpp
    src/motor_screen.cpp
    src/home_screen.cpp
    src/scrolling_label.cpp
    src/amr_node.cpp
)

# Wrap QObject-derived classes with moc
set(MOC_HEADERS
  include/main_gui.h
  include/motor_screen.h
  include/home_screen.h
  include/ui_motor_screen.h
  include/ui_main_screen.h
  include/ui_home_screen.h
  include/main_screen.h
  include/scrolling_label.h
  include/amr_node.h
)

QT5_WRAP_CPP(MOC_GENERATED_FILES ${MOC_HEADERS})

# Link the generated moc files to the target
target_sources(${PROJECT_NAME} PRIVATE ${MOC_GENERATED_FILES})

file(COPY assets DESTINATION ${CMAKE_BINARY_DIR})

# Include MongoDB headers from v_noabi directories
target_include_directories(${PROJECT_NAME}
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  /usr/local/include/mongocxx/v_noabi
  /usr/local/include/bsoncxx/v_noabi
  #/usr/local/include/bsoncxx/v_noabi/bsoncxx/third_party/mnmlstc
)

# Link libraries, including MongoDB
target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Quick
  Qt5::Widgets
  Qt5::UiTools
  rviz_common::rviz_common
  rviz_default_plugins::rviz_default_plugins
  yaml-cpp
  ${CUDA_LIBRARIES}
  ${MONGOCXX_LIBRARIES}
  ${BSONCXX_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  rviz_common
  rviz_default_plugins
)

target_compile_options(${PROJECT_NAME}
    PRIVATE
    -Wall -Wextra
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

set_target_properties(${PROJECT_NAME} PROPERTIES AUTOMOC TRUE)

ament_package()
