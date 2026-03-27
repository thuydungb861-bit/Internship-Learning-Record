cmake_minimum_required(VERSION 3.8)
project(hwt906p_driver)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(hwt906p_node
  src/hwt906p_node.cpp
  src/hwt906p_parser.cpp
)

target_compile_features(hwt906p_node PUBLIC cxx_std_17)
target_include_directories(hwt906p_node PUBLIC include)
ament_target_dependencies(hwt906p_node rclcpp sensor_msgs)

install(TARGETS hwt906p_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
