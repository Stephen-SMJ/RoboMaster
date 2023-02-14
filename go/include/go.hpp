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

#ifndef LIB_VISULIZE_H
#define LIB_VISULIZE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "airsim_ros_pkgs/VelCmd.h"
#include "airsim_ros_pkgs/PoseCmd.h"
#include "airsim_ros_pkgs/Takeoff.h"
#include "airsim_ros_pkgs/Land.h"
#include "airsim_ros_pkgs/GPSYaw.h"
#include "nav_msgs/Odometry.h"
#include  "sensor_msgs/Imu.h"
#include <time.h>
#include <stdlib.h>
#include "Eigen/Dense"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <ros/callback_queue.h>
#include <boost/thread/thread.hpp>
#include <airsim_ros_pkgs/SetLocalPosition.h>
#include <geometry_msgs/Point.h>
#include <pcl/point_types.h> 
#include <pcl/visualization/cloud_viewer.h>
#include <mutex>
#include <ros/ros.h>
#include <tf/tf.h>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <cfloat>
#include <fstream>
#include <ctime>
#include <string>
#include <bits/stdc++.h>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <list>
#include <math.h>
#include <assert.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/Image.h>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include <pcl/point_types.h> 
#include <pcl/visualization/cloud_viewer.h>


using namespace std;

// 124
struct TreeNode {
  int val;
  TreeNode *left;
  TreeNode *right;
  TreeNode() : val(0), left(nullptr), right(nullptr) {}
  TreeNode(int x) : val(x), left(nullptr), right(nullptr) {}
  TreeNode(int x, TreeNode *left, TreeNode *right) : val(x), left(left), right(right) {}
};

class Visualize //速度控制类
{
    public:
    Visualize();
    ~Visualize();
    /** Functions **/
    /**
     * @brief 管理unitree_nav_node节点的运行
     * 程序运行
     */
    void Execute();

    private:
    /** Const **/
    const int ROS_RATE_HZ = 50; //ROS更新频率

    /** Node Handle **/
    ros::NodeHandle n; // 节点句柄

		ros::Subscriber camera_left_sub_, camera_right_sub_, camera_bottom_sub_;
		ros::Publisher camera_point_pub_;
		cv::Mat img_left_;
		cv::Mat img_right_;

    /** Parameters **/

    /** Variables **/
		boost::recursive_mutex mutex_;
		
		void camerabottomCallback(const sensor_msgs::Image::ConstPtr& Input)
		{
			cv::Mat img_bottom_ = cv_bridge::toCvShare(Input, sensor_msgs::image_encodings::TYPE_8UC3)->image;
		}

		void cameraLeftCallback(const sensor_msgs::Image::ConstPtr& Input)
		{
			boost::unique_lock<boost::recursive_mutex> lock(mutex_);

			cout << "===cameraLeftCallback:===" << endl;
			img_left_ = cv_bridge::toCvShare(Input, sensor_msgs::image_encodings::TYPE_8UC3)->image;
		}
		void cameraRightCallback(const sensor_msgs::Image::ConstPtr& Input)
		{
			boost::unique_lock<boost::recursive_mutex> lock(mutex_);

			cout << "===cameraRightCallback:===" << endl;
			img_right_ = cv_bridge::toCvShare(Input, sensor_msgs::image_encodings::TYPE_8UC3)->image;
		}
		sensor_msgs::PointCloud camera_pointcloud_;

		void updateEnvironment(cv::Mat left, cv::Mat right)
		{
			boost::unique_lock<boost::recursive_mutex> lock(mutex_);
			
			camera_pointcloud_.points.clear();
			if (img_left_.rows == 0 || img_right_.rows == 0)
				return;
			cout << "===updateEnvironment===" << endl;
			
			double fx = 320, fy = 320, cx = 320, cy = 240;
    	double b = 0.095;
			cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);

			cv::Mat disparity_sgbm, disparity;
			sgbm->compute(left, right, disparity_sgbm);
			disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);

			// 定义点云使用的格式：这里用的是XYZRGB
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr road_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			
			// 根据视差和相机模型计算每一个点的三维坐标, 并添加到PCL点云中
			for (int v = 0; v < left.rows; v+=5)
				for (int u = 0; u < left.cols; u+=5) {
					if (disparity.at<float>(v, u) <= 2.0 || disparity.at<float>(v, u) >= 96.0) 
							continue;

					double depth = fx * b / (disparity.at<float>(v, u));
					pcl::PointXYZRGB p;
					// if (depth > 3) continue;

					p.x = depth * (u - cx) / fx;
					p.y = depth * (v - cy) / fy;
					p.z = depth;
					p.b = left.at<cv::Vec3b>(v, u)[0];
					p.g = left.at<cv::Vec3b>(v, u)[1];
					p.r = left.at<cv::Vec3b>(v, u)[2];
					
					road_cloud->points.push_back(p);
					geometry_msgs::Point32 p32;
					// p32.y = p.x * 0.00011785;
					// p32.z = p.y * 0.00011785;
					// p32.x = p.z * 0.00011785;
					p32.y = p.x;
					p32.z = p.y;
					p32.x = p.z;
					cout << "p32 " << p32.x << " " << p32.y << " " << p32.z << endl;
					camera_pointcloud_.points.push_back(p32);
				}
			
			//可视化视差图
			// cv::imshow("disparity", disparity / 96.0);
			// cv::waitKey(0);
			
			//可视化三维点云
			// road_cloud->height = 1;
			// road_cloud->width = road_cloud->points.size();
			// pcl::visualization::CloudViewer viewer("Cloud Viewer");
			// viewer.showCloud(road_cloud);
			// while(!viewer.wasStopped()){}
			camera_pointcloud_.header.frame_id = "camera";
			cout << "camera_pointcloud_ " << camera_pointcloud_.points.size() << endl;
			camera_point_pub_.publish(camera_pointcloud_);
			// cout << "img_left_ " << img_left_.rows << endl;
			// ros::shutdown();
			// cv::imshow("left", img_left_);
			// cv::imshow("right", img_right_);
/*
			// 内参
    double fx = 320, fy = 320, cx = 320, cy = 240;
    // 间距
    double b = 0.09;  // (注意此处的间距为双目相机左右光圈的间距)

    // // 读取图像
    // cv::Mat left = cv::imread(left_file, 0);
    // cv::Mat right = cv::imread(right_file, 0);
    // cv::Mat disparity = cv::imread(disparity_file, 0); // disparty 为CV_8U,单位为像素
    cv::imshow("real_right",right);

		cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);
		cv::Mat disparity_sgbm, disparity;
		sgbm->compute(left, right, disparity_sgbm);
		disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);
    if(left.data)
        cout<<"loaded left.png"<<endl;
    cv::imshow("left",left);
    cv::imshow("right",right);
    cv::imshow("disparity",disparity);
    cv::waitKey(0);
    cv::destroyAllWindows();

    // 生成点云
    vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> pointcloud;

    // TODO 根据双目模型计算点云
    // 如果你的机器慢，请把后面的v++和u++改成v+=2, u+=2
    for (int v = 0; v < left.rows; v++)
        for (int u = 0; u < left.cols; u++) {

            Eigen::Vector4d point(0, 0, 0, left.at<uchar>(v, u) / 255.0); // 前三维为xyz,第四维为颜色

            // start your code here (~6 lines)
            // 根据双目模型计算 point 的位置
            unsigned int d=disparity.ptr<unsigned short>(v)[u];
            if(d==0)
            {
                cout<<"d==0"<<endl;
                continue;
            }
            point[2]=(fx*b*1000)/d;
            point[1]=(v-cy)*point[2]/fy;
            point[0]=(u-cx)*point[2]/fx;
            cout<<"point = [ "<<point[0]<<" "<<point[1]<<" "<<point[2]<<" "<<point[3]<<" ]"<<endl;
            pointcloud.push_back(point);
            // end  your code here
        }
			cout<<"点云共有 : "<<pointcloud.size()<<"个点"<<endl;
			*/
		}
}; // Visualize

#endif // LIB_VISULIZE_H
