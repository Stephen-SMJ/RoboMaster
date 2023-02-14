/*
 * Copyright 2022
 *
 * Author : Jingrao.Zhang@geely.com
 * Date : 2022-05-16
 * 
 * Description : Global Planner for A1 quadruped robot.
 * Load map from slam or map_server node
 * Generate global costmap
 * link to A*
 * Flody smooth
 * Circle smooth
 * Cut the global path by 0.5m
 */

#ifndef UNITREE_NAV_ROS_GLOBALL_PLANNER_H_
#define UNITREE_NAV_ROS_GLOBALL_PLANNER_H_

#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <list>
#include <math.h>
#include <assert.h>
#include <algorithm>
#include <yaml-cpp/yaml.h>

#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>

#include <unitree_nav_ros/astar.h>

// 机器人位姿
struct RobotState
{
	// 位置
	geometry_msgs::Pose pose;
	// 姿态
	double roll, yaw, pitch;
};

struct Point2
{
  //点坐标，这里为了方便按照C++的数组来计算，x代表横排，y代表竖列
  double x, y;
	double slope;
  //parent的坐标，用指针，从而简化代码
  Point2 *parent;
  //变量初始化
  Point2(double _x, double _y, double _slope) :x(_x), y(_y), slope(_slope), parent(nullptr)
  {}
};

struct Point3
{
	double x, y, z;
};

// 圆弧平滑结构体
struct Circle
{
	// 圆心
	Point3 origin; 
	// 半径
	double r; 
};

// 地图信息
// map.yaml
// GlobalPlanner.yaml
struct Map
{
	// 图像名字
	std::string image; 
	// 分辨率
	double resolution; 
	// 原点
	std::vector<double> origin; 
	double negate;
	double occupied_thresh;
	double free_thresh;
	// 与障碍物安全距离
	double safe_expand_size; 
	double costmap_resolution_;
};

// 全局规划类
class GlobalPlanner
{
  public:
    GlobalPlanner();
    ~GlobalPlanner();
		/**
		 * @brief 初始化GlobalPlanner
		 */
    void initGlobalPlanner();
		/**
		 * @brief 获得全局规划
		 * @param destination 目标点，link是map
		 * @param robot_current_state_ 机器人信息
		 * @return 全局规划点云
		 */
    sensor_msgs::PointCloud getGlobalPlanner(const geometry_msgs::Pose& destination, 
																						 const RobotState& robot_current_state_);
	
		/**
		 * @brief 检测道路上的障碍物
		 * @param start 起点
		 * @param end 终点
		 * @param costmap 全局珊格地图
		 * @return true or false
		 */
		bool isRoutObstructed(const geometry_msgs::Point32& start,
													const geometry_msgs::Point32& end,
													const nav_msgs::OccupancyGrid& costmap);
    // add by zjr. 2022.06.21
    nav_msgs::OccupancyGrid getGlobalCostmap();
    /**
		 * @brief Flody第一次平滑
		 * @param original_path 原始A*路径
		 * @return Flody第一次平滑列表
		 */
		std::list<Point2 *> floyd1Smooth(const sensor_msgs::PointCloud& original_path);

    /**
		 * @brief Flody第二次平滑
		 * @param floyd_list Flody第一次平滑列表
		 * @return Flody第二次平滑列表
		 */
		sensor_msgs::PointCloud floyd2Smooth(const std::list<Point2 *>& floyd_list);
		
		/**
		 * @brief 圆弧平滑
		 * @param flody_path Flody第二次平滑列表
		 * @return 圆弧平滑路经点云
		 */
		sensor_msgs::PointCloud circleSmoothPath(const sensor_msgs::PointCloud& flody_path);

    /**
		 * @brief 切割全局规划路径
		 * @param path 全局规划路径
		 * @param distance 切割单位
		 * @return 指导局部规划的全局路径
		 */
		sensor_msgs::PointCloud cutGlobalPath(const sensor_msgs::PointCloud& path, const double& distance);
    
	private:

		/** Dynamic libraries **/
		// A*动态链接
    std::shared_ptr<Astar> astar_planner_;

    /** const **/
    const double PI = 3.1415926; //PI
		// 标志位：
		// 被占据
		const int OCCUPIED = 10; 
		// 危险区域
		const int DANGEROUS = 15; 
		// 安全区域
		const int SAFE = 30;
		// 空白
		const int EMPTY = 0; 
		// 拐弯
		const int CORNER = 100; 

    /** Node Handle **/
		// 节点句柄
    ros::NodeHandle nh; 

    /** Subscribers **/
		// 订阅地图
    ros::Subscriber map_sub_; 

    /** Publishers **/
		// 发布全局珊格地图
    ros::Publisher global_costmap_pub_; 
		// 发布全局规划点云
    ros::Publisher global_path_pub_; 
		// 发布A*路径
    ros::Publisher Astar_path_pub_; 
		// 发布第一次Flody平滑路径
    ros::Publisher flody_1_path_pub_; 
		// 发布第二次Flody平滑路径
    ros::Publisher flody_2_path_pub_; 
		// 发布指导局部规划的全局规划路径
		ros::Publisher global_guide_pub_;
		// 发布A*网格
		ros::Publisher Astar_mesh_pub_;
		// 发布地图边界点
		ros::Publisher map_pointcloud_pub_;
		
    /** Variables **/
		// 全局地图
    nav_msgs::OccupancyGrid map_; 
		// 全局珊格地图
		nav_msgs::OccupancyGrid costmap_global_; 
		// 初始化标志位
    bool initilize_; 
		// 全局规划起点
		geometry_msgs::Pose start_point_; 
		// 全局规划路径点云
		sensor_msgs::PointCloud global_path_; 
		// 指导局部规划的全局规划路径点云
		sensor_msgs::PointCloud global_guide_path_;
		// 地图信息 
		Map map_msgs_;

    /** Functions **/
		/**
		 * @brief 打印Config文件
		 */
		void printConfig();

		/**
		 * @brief 拓展地图
		 * @param map 全局地图
		 */
    void expandMap(const nav_msgs::OccupancyGrid& map);

		/**
		 * @brief 更新珊格地图
		 * @param environment 地图边界点云
		 * @return true or false
		 */
    bool updateCostmap(const sensor_msgs::PointCloud& environment);

		/**
		 * @brief 初始化全局珊格地图
		 * @param costmap 全局珊格地图
		 */
    void initLocalCostmap(nav_msgs::OccupancyGrid& costmap);

		/**
		 * @brief 设置珊格地图
		 * @param costmap 经过初始化的全局珊格地图
		 * @param boundary 地图边界
		 */
    void setVfhLocalCostmap(nav_msgs::OccupancyGrid& costmap,
  			   								  const sensor_msgs::PointCloud& boundary);

		/**
		 * @brief 获取A*路径点云
		 * @param map 全局珊格地图
		 * @param destination A*搜索终点
		 * @return A*路径点云
		 */
		sensor_msgs::PointCloud aStar(const nav_msgs::OccupancyGrid& map,
																	const geometry_msgs::Pose& destination);

		
		/**
		 * @brief 圆弧插值，+
		 * @param atan_start 插值在极坐标系下起点
		 * @param atan_end 插值终点
		 * @param circle 圆弧
		 * @param path 插值后的路径点云
		 * @param sign 正负号
		 * @return 圆弧平滑路经点云
		 */
		void pushCirclePathAdd(const double& atan_start,
													 const double& atan_end,
													 const Circle& circle,
													 sensor_msgs::PointCloud& path,
													 const int& sign);
		
		/**
		 * @brief 圆弧插值，-
		 * @param atan_start 插值在极坐标系下起点
		 * @param atan_end 插值终点
		 * @param circle 圆弧
		 * @param path 插值后的路径点云
		 * @param sign 正负号
		 * @return 圆弧平滑路经点云
		 */
		void pushCirclePathSub(const double& atan_start,
												   const double& atan_end,
													 const Circle& circle,
													 sensor_msgs::PointCloud& path,
													 const int& sign);
		
		/**
		 * @brief 读取配置文件
		 */
		void readConfig();

		/** inline **/
		/**
		 * @brief 获取三个点中第二个点的夹角
		 * @param x2 y2 第二个点
		 * @param x1 y1 第一个点
		 * @param x3 y3 第三个点
		 * @return 夹角
		 */
		inline double get_angle(double x2, double y2, double x1, double y1, double x3, double y3)
		{
			double theta = 
				((x2 - x1) * (x3 - x1) + (y2 - y1) * (y3 - y1)) / (hypot(x1 - x2, y1 - y2) * hypot(x3 - x1, y3 - y1));
			double acos_theta = acos(theta);
			return fabs(acos_theta);
		}

		/**
		 * @brief 返回符号
		 * @param Input 待判断的值
		 * @return 正负号或0
		 */
		inline double ComputeSign(double Input)
		{
			if (Input == 0) return 0;
			return (Input > 0) ? 1 : -1;
		} 

    /** Callbacks **/
		/**
		 * @brief 地图回调
		 * @param map 地图
		 */
  	void MapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& map)
    {
			if (!initilize_)
			{
				map_ = *map;
				// 处理地图
				expandMap(map_);
			}
    }
}; // GlobalPlanner

#endif //UNITREE_NAV_ROS_GLOBALL_PLANNER_H_