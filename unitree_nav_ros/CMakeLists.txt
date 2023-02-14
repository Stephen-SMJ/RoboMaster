cmake_minimum_required(VERSION 3.0.2)
project(unitree_nav_ros)

add_compile_options(-std=c++11)
add_compile_options(-std=c++14)
set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4/)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	tf
	#unitree_legged_msgs
	#pcl_conversions
	#pcl_ros
	airsim_ros_pkgs
	OpenCV
	cv_bridge
	pcl_ros
	cmake_modules
  sensor_msgs
  image_transport
	pcl_conversions
)

find_package(PCL REQUIRED)

add_library(
	global_planner
	src/global_planner.cpp
)

add_library(
	astar
	src/astar.cpp
)

add_library(
	spline_planner
	src/spline_planner.cpp
)

catkin_package(
	LIBRARIES visionutil rosutil
	INCLUDE_DIRS include
	# unitree_legged_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS}  ${OpenCv_INCLUDE_DIRS})

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

# add_dependencies(${PROJECT_NAME} unitree_legged_msgs_gencpp)
add_executable(unitree_nav_ros src/UnitreeNav.cc src/UnitreeNav_node.cc)

target_link_libraries(unitree_nav_ros

	${YAML_CPP_LIBRARIES}
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	${PCL_LIBRARIES}
	global_planner
	astar
	spline_planner
)
