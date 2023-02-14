/*
 * Copyright 2022
 *
 * Author : Jingrao.Zhang@geely.com
 * Date : 2022-03-22
 * 
 * Description : Navigation for A1 quadruped robot. 
 * The robot can move to destination assigned in Rviz by topic
 * "/move_base_simple/goal".
 */

#include "unitree_nav_ros/libUnitreeNav.h"

using namespace std;

UnitreeNav::UnitreeNav()
{
	// 订阅目标点信息
	goal_sub_    = n.subscribe("/move_base_simple/goal", 1, &UnitreeNav::GoalCallBack, this);
	// 订阅GAZEBO中机器人信息
	state_sub_   = n.subscribe("/gazebo/model_states", 1, &UnitreeNav::StateCallBack, this);
	// 订阅里程计
	odom_sub_    = n.subscribe("/airsim_node/drone_1/odom_local_ned", 1, &UnitreeNav::OdomCallBack, this);
	target_sub_  = n.subscribe("/target", 1, &UnitreeNav::TargetCallBack, this);
	// 发布未经处理的速度到spline_control节点
	cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("plan", 1);
	cmd_pub_ = n.advertise<airsim_ros_pkgs::VelCmd>("/airsim_node/drone_1/vel_cmd_body_frame", 1);
	// 发布前向点
	chasing_pub_ = n.advertise<sensor_msgs::PointCloud>("chasing", 1);
	pose_pub_    = n.advertise<airsim_ros_pkgs::PoseCmd>("/airsim_node/drone_1/pose_cmd_body_frame", 1);
	target_pub_  = n.advertise<geometry_msgs::Pose>("/MPC_target", 1);
	reference_circle_id_pub_  = n.advertise<std_msgs::Int8>("/id", 1);
	
	listener_    = new tf::TransformListener(n, ros::Duration(5.0), true);
	// 机器人初始状态是等待
	robot_index_ = WAITTING;
	// 读取并打印config文件
	isReadConfig();
	// 打印Config文件
	printConfig();

	// add by zjr. 2022.0329
	// 链接到全局规划
	global_planner_ = std::make_shared<GlobalPlanner>();
	// add by zjr. 2022.04.14
	spline_planner_ = std::make_shared<SplinePlanner>();

	vector<vector<float>> reference_line(8, vector<float>(5));
	reference_line = 
		{ {0, 0, 0, 0, 0},
			{14.87, -0.79, -2.74, -25, -25},
			{37.68, -12.27, -1.02, -10, -10},
			{67.42, -12.94, -0.3, 10, 10},
			{94.44, -8.05, 0.02, -30, -30},
			{113.95, -35.7, -0.34, -70, -70},
			{121.77, -67.73, -3.83, -85, -85},
			{121.80, -96, -7.39, -100, -100}
		};

	for (int i = 0; i < 8; ++i)
	{
		AirSim_global_path_[i].x = reference_line[i][0];
		AirSim_global_path_[i].y = reference_line[i][1];
		AirSim_global_path_[i].z = reference_line[i][2];
		AirSim_global_path_[i].yaw = reference_line[i][3];
		AirSim_global_path_[i].E_yaw = reference_line[i][4];
		geometry_msgs::Point32 p;
		p.x = AirSim_global_path_[i].x;
		p.y = AirSim_global_path_[i].y;
		p.z = AirSim_global_path_[i].z;
		global_path_.points.push_back(p);
	}
}

UnitreeNav::~UnitreeNav()
{
	ROS_INFO("UNITREE NAV NODE CLOSED");
}

void UnitreeNav::Manager()
{
	ros::Rate loop_rate(20);
	
	while (ros::ok())
	{
		// global_planner_->initGlobalPlanner();
		// // updateGoalLocation();
		// //初始化全局规划

		// if (global_path_.points.size() > 0)
		// {
			execute();
		// }
		
		loop_rate.sleep();
		ros::spinOnce();
	}
}

bool UnitreeNav::isReadConfig()
{
	std::ifstream yaml_file;
	std::string config_name = "/home/zjr/catkin_ws/src/config/cfg/unitree_nav.txt";

	// 打开读取txt
	yaml_file.open(config_name);
	if (!yaml_file.is_open())
		return false;

	std::string str;
	while (std::getline(yaml_file, str))
	{
		std::stringstream ss(str);
		std::string yaml_info;
		ss >> yaml_info;
		if (yaml_info == "gazebo_robot_name") ss >> gazebo_robot_name_;
		else if (yaml_info == "first_turn_target_tolerance") ss >> first_turn_target_tolerance_;
		else if (yaml_info == "arrive_tolerance") ss >> arrive_tolerance_;
	}

	yaml_file.close();
	return true;
}

void UnitreeNav::execute()
{
	geometry_msgs::Pose MPC_target;

	if (!updateRobotLocation())
	{
		ROS_ERROR("CANNOT GET ROBOT LOCATION!");
	}

	int reference_target_global = findCurrentGoalRoute(global_path_, robot_current_state_, 1);

	int closed_aim;
	int closed_next_aim;
	double close_distance = 999;
	for (int i = 0; i < global_path_.points.size(); ++i)
	{
		if (hypot3(robot_current_state_.pose.position.x - global_path_.points[i].x,
							 robot_current_state_.pose.position.y - global_path_.points[i].y,
							 robot_current_state_.pose.position.z - global_path_.points[i].z) < close_distance)
		{
			close_distance = hypot3(robot_current_state_.pose.position.x - global_path_.points[i].x,
												robot_current_state_.pose.position.y - global_path_.points[i].y,
												robot_current_state_.pose.position.z - global_path_.points[i].z);
			closed_aim = i;
		}
	}

	if (reference_target_global > closed_aim)
	{
		closed_next_aim  = closed_aim + 1;
	}
	else closed_next_aim = closed_aim;

	int reference_target = closed_next_aim;

	geometry_msgs::Pose target_m;
	static geometry_msgs::Pose last_MPC_target = target_m;

	target_m.position.x = global_path_.points[reference_target].x;
	target_m.position.y = global_path_.points[reference_target].y;
	target_m.position.z = global_path_.points[reference_target].z;
	
	// 前向点转换到base_link系下
	geometry_msgs::Pose target_b_ref;
	initializeQuaternion(target_m.orientation);
	transformPosePositioin("map", "base_link", target_m, target_b_ref, listener_);

	double distance_robot_to_reference_circle = hypot3(target_b_ref.position.x, target_b_ref.position.y, target_b_ref.position.z);
	if (hypot3(robot_current_state_.pose.position.x - global_path_.points[closed_aim].x,
						 robot_current_state_.pose.position.y - global_path_.points[closed_aim].y,
						 robot_current_state_.pose.position.z - global_path_.points[closed_aim].z) < 1) robot_index_ = PASS;
	else robot_index_ = WALKING;
	geometry_msgs::Pose target_base;
	if (hypot3(AirSim_target_.position.x, AirSim_target_.position.y, AirSim_target_.position.z) < 1 &&
		  robot_index_ == PASS &&
			AirSim_target_.position.x > 0)
	{
		std_msgs::Int8 id;
		id.data = reference_target;
		reference_circle_id_pub_.publish(id);
		target_base = AirSim_target_;
	}
	else 
	{
		target_base = target_b_ref;
		std_msgs::Int8 id;
		id.data = 0;
		reference_circle_id_pub_.publish(id);
	}

	// add by zjr. 2022.04.14
	double distance_robot_goal = hypot3(target_base.position.x, target_base.position.y, target_base.position.z);
	geometry_msgs::Point32 target;
	target.x = target_base.position.x;
	target.y = target_base.position.y;
	target.z = 1;
	sensor_msgs::PointCloud chasing;
	chasing.points.push_back(target);
	spline_target_ = spline_planner_->runSplineControl(target, distance_robot_goal);
	
	target_base.position.x = spline_target_.x;
	target_base.position.y = spline_target_.y;
	target_base.position.z = spline_target_.z;
	
	// 前向点转换到base_link系下
	geometry_msgs::Pose target_real_base;
	initializeQuaternion(target_base.orientation);
	transformPosePositioin("base_link", "real_base_link", target_base, target_real_base, listener_);
	MPC_target = target_real_base;
	
	double odom_angle = (tf::getYaw(odom_.pose.pose.orientation) / PI) * 180;
	double odom_euro = tf::getYaw(odom_.pose.pose.orientation);
	double delta_angle = AirSim_global_path_[reference_target].yaw - odom_angle;
	double delta_euro = (delta_angle / 180) * PI;

	if (fabs(delta_angle) > 5)
	{
		if (hypot3(AirSim_target_.position.x, AirSim_target_.position.y, AirSim_target_.position.z) > 3)
		{
			MPC_target.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, delta_euro);
		}
		else 
			initializeQuaternion(MPC_target.orientation);	
	}
	else 
	{
		initializeQuaternion(MPC_target.orientation);	
	}

	// if (MPC_target.position.x != last_MPC_target.position.x || 
	// 		MPC_target.position.y != last_MPC_target.position.y || 
	// 		MPC_target.position.z != last_MPC_target.position.z ||
	// 		tf::getYaw(last_MPC_target.orientation) != tf::getYaw(MPC_target.orientation))
	{
		target_pub_.publish(MPC_target);
		// cout << "=================" << endl;
		// cout << "MPC_target " << MPC_target.position.x << " " << MPC_target.position.y << " " << MPC_target.position.z << endl;
 		// cout << "target Airsim " << hypot3(AirSim_target_.position.x, AirSim_target_.position.y, AirSim_target_.position.z) << endl;
		// cout << "reference_target " << reference_target << endl;
		// cout << "AirSim_global_path_[reference_target].yaw " << AirSim_global_path_[reference_target].yaw << endl;
		// cout << "yaw in real_base " << tf::getYaw(MPC_target.orientation) << endl;
		// cout << "Angle " << (tf::getYaw(MPC_target.orientation) / PI) * 180 << endl;
		// cout << "odom " << tf::getYaw(odom_.pose.pose.orientation) << endl;
		// cout << "odom anglu " << (tf::getYaw(odom_.pose.pose.orientation) / PI) * 180 << endl;
		// cout << "delta yaw " << AirSim_global_path_[reference_target].yaw -odom_angle << endl;
		// cout << "publish " << delta_euro << endl;
		last_MPC_target = MPC_target;
	}
	geometry_msgs::Point32 tt;
	tt.x = MPC_target.position.x;
	tt.y = MPC_target.position.y;
	tt.z = MPC_target.position.z;
	double yy = atan2(tt.y, tt.x);
	chasing.points.push_back(tt);
	
	chasing.header.frame_id = "base_link";
	chasing_pub_.publish(chasing);
	cout << "yy " << yy << endl;
	applySpeedFollow(tt, yy);
}

bool UnitreeNav::updateRobotLocation()
{
	tf::StampedTransform transform;

	try 
	{
		listener_->lookupTransform("map", "real_base_link", ros::Time(0), transform);
	}
	catch (tf::TransformException &ex)
	{
		std::cerr << ex.what() << '\n';
		return false;
	}

	robot_current_state_.pose.position.x = transform.getOrigin().x();
	robot_current_state_.pose.position.y = transform.getOrigin().y();
	robot_current_state_.pose.position.z = transform.getOrigin().z();
	robot_current_state_.yaw = transform.getOrigin().z();
	// robot_current_state_.pose.position.z = tf::getYaw(transform.getRotation());
	return true;
}

bool UnitreeNav::updateGoalLocation()
{
	if (!transformPosePositioin("map", "base_link", robot_target_map_.pose, robot_target_base_.pose, listener_))
		return false;
	// 计算目标点与机器人的方位角
	robot_target_base_.tolerance_angle =
		atan2(robot_target_base_.pose.position.y, robot_target_base_.pose.position.x);
	// 计算目标点与机器人的距离
	robot_target_base_.distance_robot_goal =
		hypot(robot_target_base_.pose.position.x, robot_target_base_.pose.position.y);
	robot_target_map_.tolerance_angle = robot_target_map_.tolerance_angle;
	robot_target_map_.distance_robot_goal = robot_target_map_.distance_robot_goal;

	return true;
}

bool UnitreeNav::transformPosePositioin(std::string frame,
                                        std::string target_frame,
                                        geometry_msgs::Pose& input,
                                        geometry_msgs::Pose& output,
                                        tf::TransformListener* listener)
{
	geometry_msgs::PoseStamped input_s, output_s;
	input_s.header.frame_id = frame;
	input_s.pose = input;
	output_s.header.frame_id = target_frame;
	
	try
	{
		input_s.header.stamp = ros::Time(0);
		input_s.header.frame_id = frame;
		listener->transformPose(target_frame, input_s, output_s);
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
		return false;
	}

	output = output_s.pose;

	return true;
}

bool UnitreeNav::isGoalReached()
{
	// 更新机器人位姿
	if (!updateRobotLocation())
	{
		ROS_ERROR("CANNOT GET ROBOT LOCATION!");
		return true;
	}
	// 机器人为等待状态时默认到达目标点
	if (robot_index_ == WAITTING)
		return true;

	// 更新目标点在base系下位姿
	if (!updateGoalLocation())
	{
		ROS_ERROR("CANNOT GET GOAL POSE IN BASE_LINK !");
		return false;
	}

	// 机器人处于首次转向状态
	if (robot_index_ == TURN_TARGET)
	{   
		// add zjr. 0823 
		if (robot_target_base_.distance_robot_goal <= 0.2)
		{
			// 机器人状态置为减速
			robot_index_ = SLOW_DOWN;
			
			// 是否减速完成
			if (isStop())
			{
				// 返回到达目标点
				return true;
				robot_index_ == WAITTING;
			}
		}
	}
	// 机器人处于行走状态
	else if (robot_index_ == WALKING) 
	{   
		// 判断机器人是否到达目标点
		if (robot_target_base_.distance_robot_goal <= 0.2)
		{
			// 机器人状态置为减速
			robot_index_ = SLOW_DOWN; 
			// 是否减速完成
			if (isStop()) 
			{
				// 返回到达目标点
				return true; 
				robot_index_ == WAITTING;
			}
		}
	}

	return false;
}

geometry_msgs::Point32 UnitreeNav::getTemporaryTargetInBaseLink()
{
	// 获取全局路径map系下前向点
	int num = findCurrentGoalRoute(global_path_, robot_current_state_, 1);
	
	geometry_msgs::Point32 target;
	geometry_msgs::Pose target_m;
	target_m.position.x = global_path_.points[num].x;
	target_m.position.y = global_path_.points[num].y;
	
	// 前向点转换到base_link系下
	geometry_msgs::Pose target_b;
	initializeQuaternion(target_m.orientation);
	transformPosePositioin("map", "base_link", target_m, target_b, listener_);

  target.x = target_b.position.x;
  target.y = target_b.position.y;

	return target;
}

// add by zjr. 2022.03.28
geometry_msgs::Twist UnitreeNav::applySpeedFollow(geometry_msgs::Point32 target, double yaw)
{
	geometry_msgs::Twist twist;
	airsim_ros_pkgs::VelCmd objective_vel;
	objective_vel.twist.linear.x =
	2 *
			ComputeSign(target.x) * ((fabs(target.x) > 2) ? 2 : (1 * fabs(target.x)));
	// objective_vel.twist.linear.y =
	// 		ComputeSign(target.y) * ((fabs(target.y) > 2) ? 2 : (1 * fabs(target.y)));
	objective_vel.twist.linear.z =
	2 *
			ComputeSign(target.z) * ((fabs(target.z) > 2) ? 2 : (1 * fabs(target.z)));
	objective_vel.twist.angular.z = 2 * yaw;

	cmd_pub_.publish(objective_vel);

	// airsim_ros_pkgs::PoseCmd objective_pose;
	// objective_pose.roll = double(0.0);
	// objective_pose.pitch = double(atan(target.z / target.x));
	// objective_pose.yaw = double(atan(target.y / target.x));
	// objective_pose.throttle = 0.0;
	
	// pose_pub_.publish(objective_pose);

return twist;

/*


	cout << "target " << target.x << " " << target.y << endl;
	if (robot_index_ == TURN_TARGET)
	{
	// 判断机器人是否首次转向完成
		// 机器人状态置为行走
		if (fabs(atan2(target.y, target.x)) < 0.1)
			robot_index_ = WALKING;
	}
	ROS_WARN("applySpeedFollow");
	// 发布前向点
	sensor_msgs::PointCloud chasing;
	chasing.points.push_back(target);
	chasing.header.frame_id = "base_link";
	chasing_pub_.publish(chasing);
	// geometry_msgs::Point32 target = getTemporaryTargetInBaseLink();

	// 计算距离和角度差
	geometry_msgs::Twist objective_vel;
	initializeVelocity(objective_vel);
	// if (spline_planner_->isRobotAttachObstacle()) 
	// 	return objective_vel;
	double length_t = hypot(target.x, target.y);
	double angle_t = atan2(target.y, target.x);
	std::cout << "robot_index_ " << robot_index_ << std::endl;
	// 机器人处于首次转向状态
	if (robot_index_ == TURN_TARGET) 
	{
		// 角速度规划
		objective_vel.angular.z =
			((fabs(angle_t) > 0.5) ? 0.6 : (1.2 * fabs(angle_t))) * ComputeSign(angle_t);
		cout << "angle_t " << angle_t << endl;
	}
	// 机器人处于行走状态
	else if (robot_index_ == WALKING) 
	{
		cout << "angle_t " << angle_t << endl;
		// 线速度规划
		objective_vel.linear.x =
			(length_t > 0.5) ? 0.5 : (1 * fabs(length_t));
		// 角速度规划
		objective_vel.angular.z =
			((fabs(angle_t) > 0.5) ? 0.7 : (1.4 * fabs(angle_t))) * ComputeSign(angle_t);

		if (fabs(angle_t) > 0.5) objective_vel.linear.x = 0;
	} 
	else if (robot_index_ == SLOW_DOWN)
	{
		// 线速度规划
		objective_vel.linear.x =
			(length_t > 0.2) ? 0.2 : (1 * fabs(length_t));
		// 角速度规划
		objective_vel.angular.z =
			((fabs(angle_t) > 0.2) ? 0.28 : (1.4 * fabs(angle_t))) * ComputeSign(angle_t);
	}
	cout << "objective_vel " << objective_vel.linear.x << " " << objective_vel.angular.z << endl;
	return objective_vel;
	*/
}
// add by zjr. 2022.03.28
int UnitreeNav::findCurrentGoalRoute(const sensor_msgs::PointCloud& path, 
																	 	 const RobotState& robotState, 
																		 double lookahead) 
{
	geometry_msgs::Point32 Robot;
	Robot.x = robotState.pose.position.x;
	Robot.y = robotState.pose.position.y;
  geometry_msgs::Point32 current_goal;
  int output = -1;
  double min_value = DBL_MAX;
  double min_index_i = -1;
  double min_index_j = -1;
	// 全局路径为空返回
  if (path.points.size() <= 0)
    return 0;

	// 遍历全局路径
  for (int i = 0; i < path.points.size()-1; ++i) 
	{
		// 前向距离
    double lookahead_applied = lookahead;
		// 计算到每个连续三个点的距离
    double distance_i 
      = hypot((path.points.at(i).x - Robot.x),(path.points.at(i).y - Robot.y));
    double distance_j 
      = hypot((path.points.at(i+1).x - Robot.x),(path.points.at(i+1).y - Robot.y));
    double distance_ij 
      = hypot((path.points.at(i).x - path.points.at(i+1).x)
      ,(path.points.at(i).y - path.points.at(i+1).y));

		// 计算距离
    double value_ij = (distance_i + distance_j - distance_ij) / distance_ij;

		// 对比上一次计算结果
    if(value_ij < min_value) 
		{
      min_value = value_ij;
      min_index_i = i;
      min_index_j = i + 1;
			// 静态存储本次结果
      if(distance_j < lookahead_applied && min_index_j < path.points.size()-1) 
        min_index_j++;
    }
  }

	// 避免结果非法
  if(min_index_i == -1) min_index_i = path.points.size() - 1;
  if(min_index_j == -1) min_index_j = path.points.size() - 1;
  output = min_index_j;
	// 向后顺眼
	// (output == path.points.size()-1) ? output : output += 1;

  return output;
}

void UnitreeNav::printConfig()
{
	std::cout << "==================================" << std::endl;
	std::cout << "          Unitree Config          " << std::endl;
	std::cout << "==================================" << std::endl;
	std::cout << "gazebo_robot_name : " << gazebo_robot_name_ << std::endl;
	std::cout << "first_turn_target_tolerance : " << first_turn_target_tolerance_ << std::endl;
	std::cout << "arrive_tolerance : " << arrive_tolerance_ << std::endl;
	std::cout << "==================================" << std::endl;
}