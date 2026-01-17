cmake_minimum_required(VERSION 3.5)
project(cxd5602pwbimu_localizer_node)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
  set(CMAKE_C_STANDARD_REQUIRED ON)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(cxd5602pwbimu_localizer_component SHARED
  src/localizer_component.cpp
)
target_include_directories(cxd5602pwbimu_localizer_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cxd5602pwbimu_localizer_component
  rclcpp rclcpp_components sensor_msgs geometry_msgs tf2_ros)

rclcpp_components_register_nodes(cxd5602pwbimu_localizer_component
  "cxd5602pwbimu_localizer_node::LocalizerComponent")

install(TARGETS cxd5602pwbimu_localizer_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
add_executable(localizer_node src/localizer_node.cpp)
target_link_libraries(localizer_node
  cxd5602pwbimu_localizer_component
)
ament_target_dependencies(localizer_node
  rclcpp
)
install(TARGETS localizer_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
ament_package()