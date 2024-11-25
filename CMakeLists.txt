cmake_minimum_required(VERSION 3.8)
project(rototui_parking_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# CMAKE Prefix Path Addition
set(CMAKE_PREFIX_PATH "~/autoware/install")


# find dependencies and packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_cmake REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(autoware_map_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(tier4_planning_msgs REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_extension REQUIRED)
find_package(autoware_route_handler REQUIRED)
find_package(rototui_interfaces REQUIRED)
find_package(autoware_freespace_planning_algorithms REQUIRED)
find_package(autoware_freespace_planner REQUIRED)

autoware_package() 



add_executable(parking_planner_service src/parking_planner_service.cpp)
ament_target_dependencies(parking_planner_service rclcpp rototui_interfaces)

add_executable(parking_planner_client src/parking_planner_client.cpp)
ament_target_dependencies(parking_planner_client rclcpp rototui_interfaces)



add_executable(parking_space_planner_node src/parking_space_planner.cpp)

target_include_directories(parking_space_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(parking_space_planner_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


ament_target_dependencies(parking_space_planner_node
                          rclcpp 
                          tf2
                          tf2_ros
                          tf2_geometry_msgs
                          nav_msgs
                          sensor_msgs
                          autoware_map_msgs
                          autoware_planning_msgs
                          autoware_perception_msgs
                          autoware_adapi_v1_msgs
                          tier4_planning_msgs
                          autoware_route_handler
                          lanelet2_core
                          lanelet2_extension
                          rototui_interfaces
                          autoware_freespace_planning_algorithms
                          autoware_freespace_planner  
                        )



install(TARGETS
  parking_space_planner_node
  parking_planner_client
  parking_planner_service
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
