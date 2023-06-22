cmake_minimum_required(VERSION 3.8)
project(3dmap_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(PCL 1.2 REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

ament_auto_find_build_dependencies()

include_directories(include
                    Boost
                    ${PCL_INCLUDE_DIRS} 
                    ${EIGEN3_INCLUDE_DIRS}
)
link_directories(
  Boost
  ${PCL_LIBRARY_DIRS}
  ${EIGEN3_INCLUDE_DIRS})

add_definitions(${PCL_DEFINITIONS})

# add_executable(publish_pc src/publish_pc.cpp)
# ament_target_dependencies(publish_pc rclcpp pcl_conversions sensor_msgs)

ament_auto_add_library(map_server_component SHARED src/map_server_component.cpp)
rclcpp_components_register_nodes(map_server_component "map_server::MapServerComponent")
ament_auto_add_executable(map_server
  src/map_server_component.cpp
  src/map_server_node.cpp
)

# target_link_libraries(publish_pc ${PCL_LIBRARIES})
target_link_libraries(map_server ${PCL_LIBRARIES})

install(DIRECTORY map DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

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

ament_auto_package()
