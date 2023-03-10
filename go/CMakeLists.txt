cmake_minimum_required(VERSION 3.0.2)
project(go)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4/)

find_package(catkin REQUIRED COMPONENTS
airsim_ros_pkgs
message_generation
nav_msgs
# nodelet
roscpp
rospy
sensor_msgs
std_msgs
geographic_msgs
geometry_msgs
std_srvs
tf2
tf2_ros
tf2_sensor_msgs
tf2_geometry_msgs
image_transport
OpenCV
pcl_ros
cv_bridge
)
find_package(PCL 1.8 REQUIRED )


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp std_msgs # nodelet 
)

include_directories(include ${catkin_INCLUDE_DIRS}  ${OpenCv_INCLUDE_DIRS})


add_executable(go src/go.cpp )
target_link_libraries(go ${catkin_LIBRARIES} ${OpenCv_LIBS} ${Boost_LIBRARIES})

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  # PATTERN ".git" EXCLUDE
)
target_link_libraries(go ${PCL_LIBRARIES})