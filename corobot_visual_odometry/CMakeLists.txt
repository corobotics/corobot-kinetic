cmake_minimum_required(VERSION 2.8.3)
project(corobot_visual_odometry)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
  cv_bridge
)


catkin_package(
  CATKIN_DEPENDS roscpp std_msgs cv_bridge
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

add_definitions(-std=c++11)

add_executable(corobot_visual_odometry_node src/corobot_visual_odometry_node.cpp)

target_link_libraries(corobot_visual_odometry_node 
	${catkin_LIBRARIES}
	${PCL_LIBRARIES})
