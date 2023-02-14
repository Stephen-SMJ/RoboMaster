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

#ifndef LIB_SMOOTH_CONTROL_H
#define LIB_SMOOTH_CONTROL_H

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <iostream>
#include <fstream>
#include <string>
#include <fstream>
#include <ctime>
#include <bits/stdc++.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int8.h>

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

class SmoothControl //速度控制类
{
    public:
    SmoothControl();
    ~SmoothControl();
    /** Functions **/
    /**
     * @brief 管理unitree_nav_node节点的运行
     * 程序运行
     */
    void Execute();

    private:
    /** Const **/
    const int ROS_RATE_HZ = 20; //ROS更新频率

    /** Node Handle **/
    ros::NodeHandle n; // 节点句柄

    /** Subscribers **/
    ros::Subscriber target_sub_; // 订阅未经处理的速度话题
    ros::Subscriber odom_sub_;
    ros::Subscriber id_sub_;

    /** Publishers **/
    ros::Publisher cmd_vel_pub_; // 发布处理过后的速度话题
    ros::Publisher mode_pub_; // 发布处理过后的速度话题

    /** Parameters **/

    /** Variables **/
    geometry_msgs::Twist last_vel_; // 上一帧速度
    geometry_msgs::Twist target_vel_; // 目标速度
    ros::Time twist_clock_; //计时器
    nav_msgs::Odometry last_odom_;
		std::queue<nav_msgs::Odometry> odom_queue_;
    nav_msgs::Odometry odom_;
    tf::TransformBroadcaster tfBroadcaster_;
    int id_;
    void OdomCallBack(const nav_msgs::Odometry::ConstPtr& msg)
    {
      odom_ = *msg; 
    }
		geometry_msgs::Point32 AirSim_target_;
    void IdCallBack(const std_msgs::Int8::ConstPtr& msg)
    {
      id_ = msg->data;
    }
    void TargetCallBack(const geometry_msgs::Point32::ConstPtr& odom)
		{
			AirSim_target_ = *odom;
      AirSim_target_.x += 0.26;
		}

    vector<vector<float>> reference_line_ = vector<vector<float>>(8, vector<float>(5));

    void play();
    // 827
    const vector<int> d = {0, -1, 0, 1, 0};
    bool valid(int n, int x, int y);
    int dfs(const vector<vector<int>> &grid, int x, int y, vector<vector<int>> &tag, int t);
    int largestIsland(vector<vector<int>>& grid);
    // 124
    int maxSum = INT_MIN;
    int maxGain(TreeNode* node);
    int maxPathSum(TreeNode* root);
    bool isListValid(vector<int> List);




}; // SmoothControl

#endif // LIB_SMOOTH_CONTROL_H
