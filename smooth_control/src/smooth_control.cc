/*
 * Copyright 2022
 *
 * Author : Jingrao.Zhang@geely.com
 * Date : 2022-03-22
 * 
 * Description : smooth the velocity
 * uniformly accelerated motion
 * uniformly retarded motion
 * monitor speed and publish "/cmd_vel" to controller_node
 */

#include "smooth_control/smooth_control.h"

using namespace std;
using namespace tf;

SmoothControl::SmoothControl()
{
  odom_sub_ = n.subscribe("/airsim_node/drone_1/odom_local_ned", 1, &SmoothControl::OdomCallBack, this);
  target_sub_  = n.subscribe("/target", 1, &SmoothControl::TargetCallBack, this);
  id_sub_  = n.subscribe("/id", 1, &SmoothControl::IdCallBack, this);
  id_ = 0;
	reference_line_ = 
		{ {0, 0, 0, 0, 0},
			{14.87, -0.79, -2.74, -25, -25},
			{37.68, -12.27, -1.02, -10, -10},
			{67.42, -12.94, -0.3, 10, 10},
			{94.44, -8.05, 0.02, -30, -30},
			{113.95, -35.7, -0.34, -70, -70},
			{121.77, -67.73, -3.83, -85, -85},
			{121.80, -96, -7.39, -100, -100}
		};
}  

SmoothControl::~SmoothControl()
{
  ROS_INFO("SMOOTH CONTROL NODE CLOSED");
}

void SmoothControl::Execute()
{
  ros::Rate loop_rate(ROS_RATE_HZ);

  while (ros::ok())
  {
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z));
    tf::quaternionMsgToTF(odom_.pose.pose.orientation, q);
    transform.setRotation(q);
    tfBroadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "real_base_link"));

    transform.setOrigin(tf::Vector3(0.26, 0, 0));
    q = tf::createQuaternionFromRPY(0, 0, 0);
    transform.setRotation(q);
    tfBroadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "real_base_link", "camera"));

    if (id_ != 0)
    {
      AirSim_target_.x = reference_line_[id_][0];
      AirSim_target_.y = reference_line_[id_][1];
      AirSim_target_.z = reference_line_[id_][2];
    }

    transform.setOrigin(tf::Vector3(0, 0, 0));
    // tf::Quaternion q;
    // tf::quaternionMsgToTF(odom_.pose.pose.orientation, q);
    double pitch_target = atan(AirSim_target_.z / AirSim_target_.x);
    double yaw_target = atan(AirSim_target_.y / AirSim_target_.x);
    cout << "pitch_target " << pitch_target << endl;
    cout << "yaw_target " << yaw_target << endl;
    q = tf::createQuaternionFromRPY(0, -pitch_target, yaw_target);
    transform.setRotation(q);
    tfBroadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "real_base_link", "base_link"));

    loop_rate.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char ** argv) 
{
  ROS_INFO("BEGIN SMOOTH CONTROL PATH");
  ros::init(argc, argv, "smooth_control_node");
  SmoothControl MySmoothControl;
  MySmoothControl.Execute();

  return 0;
}