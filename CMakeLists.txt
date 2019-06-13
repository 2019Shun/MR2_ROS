cmake_minimum_required(VERSION 2.8.3)
project(mr2_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
  message_generation
)

# add_message_files(
#   FILES
#   Point.msg
# )

add_service_files(
  FILES
  Point.srv
  CourtMode.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES mr2_ros
 CATKIN_DEPENDS   geometry_msgs pcl_ros roscpp rospy sensor_msgs std_msgs tf visualization_msgs message_runtime 
 DEPENDS system_lib
)  

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(field_detector_node src/field_detector_node.cpp src/detector_manager.cpp src/detector.cpp)
target_link_libraries(field_detector_node ${catkin_LIBRARIES})
add_dependencies(field_detector_node ${PROJECT_NAME}_gencpp)

add_executable(plane_estimation_node src/plane_estimation_node.cpp)
target_link_libraries(plane_estimation_node ${catkin_LIBRARIES})

add_executable(field_position_node src/field_position_node.cpp)
target_link_libraries(field_position_node ${catkin_LIBRARIES})
add_dependencies(field_position_node ${PROJECT_NAME}_gencpp)