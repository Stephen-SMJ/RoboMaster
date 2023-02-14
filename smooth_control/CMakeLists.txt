cmake_minimum_required(VERSION 3.0.2)
project(smooth_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(smooth_control src/smooth_control.cc)

target_link_libraries(smooth_control
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)
