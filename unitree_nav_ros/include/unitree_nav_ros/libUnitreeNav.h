/*
 * Copyright 2022
 *
 * Author : Jingrao.Zhang@geely.com
 * Date : 2022-03-22
 * 
 * Description : Navigation for A1 quadruped robot.
 */

#ifndef LIB_UNITREENAV_H
#define LIB_UNITREENAV_H

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

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_listener.h>

#include <gazebo_msgs/ModelStates.h>

// add by zjr. 2022.03.28
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include <unitree_nav_ros/global_planner.h>

// add by zjr. 2022.04.14
#include "unitree_nav_ros/spline_planner.h"
#include <std_msgs/String.h>

#include "airsim_ros_pkgs/VelCmd.h"
#include "airsim_ros_pkgs/PoseCmd.h"

#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>

struct DestinationState // 目标点位姿
{
	geometry_msgs::Pose pose; // 位置
	float x, y, z;
	double roll, yaw, pitch; // 姿态
	double E_yaw;
	double tolerance_angle; // 目标点与机器人的方位角
	double distance_robot_goal; // 目标点与机器人的距离
};

class UnitreeNav // 机器人导航类
{
	public:
		UnitreeNav();
		~UnitreeNav();
		/**
		 * @brief 管理unitree_nav_node节点的运行
		 */
		void Manager();

	private:

		// add by zjr. 2022.0328

		/** Dynamic libraries **/
		// 全局规划动态库
		std::shared_ptr<GlobalPlanner> global_planner_;
		// add by zjr. 2022.04.14
		std::shared_ptr<SplinePlanner> spline_planner_;

		/** enum **/
		enum Robot_index : const int // 机器人状态
		{
			WAITTING = 1, // 正在等待任务
			TURN_TARGET, // 正在转向目标点
			WALKING, // 正在行走
			ADJUST_ORIENTATION, // 正在调整姿态
			SLOW_DOWN, // 正在减速
			PASS
		};

		/** enum **/
		enum FlyIndex : const int // 机器人状态
		{
			GOAL_1 = 0,
			GOAL_2,
			GOAL_3,
			GOAL_4,
			GOAL_5,
			GOAL_6,
			GOAL_7
		};

		/** Const **/
		const int ROS_RATE_HZ = 20; //ROS更新频率
		const double PI       = 3.1415926; // PI

		/** Node Handle **/
		ros::NodeHandle n; // 节点句柄

		/** Subscribers **/
		ros::Subscriber goal_sub_; // 订阅目标点
		ros::Subscriber state_sub_; // 订阅Gazebo中机器人的位姿和速度信息
		ros::Subscriber odom_sub_; // 订阅odom
		ros::Subscriber target_sub_; // 订阅odom
		
		ros::Subscriber voice_sub_;

		/** Publishers **/
		ros::Publisher cmd_vel_pub_; // 发布速度
		ros::Publisher chasing_pub_; // 发布前向点
		ros::Publisher cmd_pub_; // 发布前向点
		ros::Publisher pose_pub_; // 发布前向点
		ros::Publisher target_pub_; // 
		ros::Publisher reference_circle_id_pub_; // 

		/** Tf **/
		tf::TransformListener* listener_; // TF接收

		/** Parameters **/
		std::string gazebo_robot_name_; // 仿真中机器人的名字
		double first_turn_target_tolerance_; // 首次转向目标点的Yaw角容许误差
		double arrive_tolerance_; //到达目标点的位置容许误差

		/** Variables **/
		int robot_index_; // 机器人状态
		RobotState robot_current_state_; // 机器人位姿
		DestinationState robot_target_map_; // 目标点map系下位姿
		DestinationState robot_target_base_; //目标点base系下位姿

		// add by zjr. 2022.03.28
		nav_msgs::Odometry odom_; // 里程计信息
		sensor_msgs::PointCloud global_path_; // 全局路径点云

		geometry_msgs::Twist untreated_cmd_;
		geometry_msgs::Point32 spline_target_;
		vector<DestinationState> AirSim_global_path_ = vector<DestinationState>(8);

		/** Functions **/
		/**
		 * @brief 程序运行
		 */
		void execute();

		/**
		 * @brief 更新机器人位姿和目标点在base系下的位姿，进行状态机转换，并判断机器人是否到达目标点
		 * @return true or false
		 */
		bool isGoalReached();

		/**
		 * @brief 更新机器人位姿
		 */
		bool updateRobotLocation();

		/**
		 * @brief 更新目标点在base系下的位姿
		 */
		bool updateGoalLocation();

		/**
		 * @brief 坐标系转换
		 * @param frame 当前坐标系
		 * @param target_frame 转换到此坐标系
		 * @param input 代转换坐标
		 * @param input 转换后坐标
		 * @param listener TF
		 * @return true or false
		 */
		bool transformPosePositioin(std::string frame,
																std::string target_frame,
																geometry_msgs::Pose& input,
																geometry_msgs::Pose& output,
																tf::TransformListener* listener);

		/**
		 * @brief 速度规划
		 * @return twist
		 */
		geometry_msgs::Twist applySpeedFollow(geometry_msgs::Point32 target, double yaw);

		/**
		 * @brief 读取config文件
		 * @return true or false
		 */
		bool isReadConfig();

		/**
		 * @brief 打印config文件到终端
		 * @return true ot false
		 */
		void printConfig();

		// add by zjr. 2022.03.28
		/**
		 * @brief 四元数初始化
		 * @param Path 待初始化的四元数is_navigation_begin_
		 * @return 前向点位置
		 */
		int findCurrentGoalRoute(const sensor_msgs::PointCloud& Path, 
														 const RobotState& Robot, 
														 double lookahead);
		
		geometry_msgs::Point32 getTemporaryTargetInBaseLink();

		/** Inline Function **/ 
		/**
		 * @brief 四元数初始化
		 * @param velocity 待初始化的四元数
		 */
		inline void initializeQuaternion(geometry_msgs::Quaternion& quaternion)
		{
			quaternion.x = 0;
			quaternion.y = 0;
			quaternion.z = 0;
			quaternion.w = 1;
		}

		/**
		 * @brief 速度初始化
		 * @param velocity 待初始化的速度
		 */
		inline void initializeVelocity(geometry_msgs::Twist& velocity)
		{
			velocity.linear.x = 0;
			velocity.linear.y = 0;
			velocity.angular.z = 0;
		}

		/**
		 * @brief 返回符号
		 * @param Input 待判断的数据
		 */
		inline double ComputeSign(double Input)
		{
			if (Input == 0) return 0;
			return (Input > 0) ? 1 : -1;
		} 

		/**
		 * @brief 判断机器人是否静止
		 * @return true or false
		 */
		inline bool isStop()
		{
			if (hypot(odom_.twist.twist.linear.x, odom_.twist.twist.linear.y) == 0)
				return true;
			return false;
		}

		/** Callbacks **/
    nav_msgs::OccupancyGrid global_costmap_;

		/**
		 * @brief 目标点回调
		 * @param goal Rviz中的2D Nav Goal
		 */
		void GoalCallBack(const geometry_msgs::PoseStamped::ConstPtr& goal)
		{
			robot_target_map_.pose.position.x = goal->pose.position.x;
			robot_target_map_.pose.position.y = goal->pose.position.y;
			initializeQuaternion(robot_target_map_.pose.orientation);
			robot_index_ = TURN_TARGET;
			updateRobotLocation();
      // zjr 2022.06.22
			global_path_ = global_planner_->getGlobalPlanner(robot_target_map_.pose, robot_current_state_);
		}   

		/**
		 * @brief Gazebo中机器人位姿回调
		 * @param state 机器人位姿
		 */
		void StateCallBack(const gazebo_msgs::ModelStates::ConstPtr& state)
		{
			for (int i = 0; i < state->name.size(); ++i)
			{
				if (state->name[i] == gazebo_robot_name_)
				{
					robot_current_state_.pose = state->pose[i];
					robot_current_state_.yaw = tf::getYaw(state->pose[i].orientation);
					continue;
				}
			} 
		}

		/**
		 * @brief odom回调
		 * @param odom odom
		 */
		void OdomCallBack(const nav_msgs::Odometry::ConstPtr& odom)
		{
			odom_ = *odom;
		}
		geometry_msgs::Pose AirSim_target_;
		void TargetCallBack(const geometry_msgs::Point32::ConstPtr& odom)
		{
			geometry_msgs::Pose target_base, target_b;
			target_base.position.x = odom->x;
			target_base.position.y = odom->y;
			target_base.position.z = odom->z;
			initializeQuaternion(target_base.orientation);
			transformPosePositioin("camera", "base_link", target_base, target_b, listener_);

			AirSim_target_ = target_b;

			// sensor_msgs::PointCloud chasing;
			// chasing.points.push_back(AirSim_target_);
			// chasing.header.frame_id = "base_link";
			// chasing_pub_.publish(chasing);
		}

		inline double hypot3(double x, double y, double z)
		{
			return sqrt(x*x + y*y + z*z);
		}
		
}; // UnitreeNav

#endif // LIB_UNITREENAV_H