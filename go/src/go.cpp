#ifndef _GO_CPP_
#define _GO_CPP_

#include "go.hpp"

using namespace std;
using namespace cv;

Visualize::Visualize()
{
	// cv::namedWindow("view");
	// cv::startWindowThread();
	camera_left_sub_ = n.subscribe("/airsim_node/drone_1/front_left/Scene",10, &Visualize::cameraLeftCallback,this);
	camera_right_sub_ = n.subscribe("/airsim_node/drone_1/front_right/Scene",10, &Visualize::cameraRightCallback,this);
	camera_bottom_sub_ = n.subscribe("/airsim_node/drone_1/bottom_center/Scene",10, &Visualize::camerabottomCallback,this);
	camera_point_pub_ = n.advertise<sensor_msgs::PointCloud>("/scan_point",10);
}  

Visualize::~Visualize()
{
	cv::destroyAllWindows();
  ROS_INFO("VISULIZE NODE CLOSED");
}

void Visualize::Execute()
{
  ros::Rate loop_rate(ROS_RATE_HZ);

  while (ros::ok())
  {
		// cout << "img_left_.data " << img_left_.data.size() << endl;
		// cv::imshow("left", img_left_);
		// cv::imshow("right", img_right_);

		updateEnvironment(img_left_, img_right_);
		
    loop_rate.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char ** argv) 
{
  ROS_INFO("VISUALIZE PATH");
  ros::init(argc, argv, "visulize_node");
  Visualize MyVisualize;
  MyVisualize.Execute();

  return 0;
}
#endif