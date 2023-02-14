#ifndef UNITREE_NAV_ROS_SPLINE_PLANNER_H_
#define UNITREE_NAV_ROS_SPLINE_PLANNER_H_

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

#include "unitree_nav_ros/global_planner.h"
#include <sensor_msgs/Image.h>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include <pcl/point_types.h> 
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>

using namespace std;
using namespace cv;

// 样条集合结构体
struct PathGroup
{
	// 样条点云
	sensor_msgs::PointCloud path_pointcloud;
	// 样条id
	int path_id;
};

// 样条映射地图结构体
struct MapToPath
{
	// 地图像素点
	int map_data_num;
	// 对应样条id
	vector<int> path_id;
};

// 样条局部规划类
class SplinePlanner
{
  public:
    SplinePlanner();
    ~SplinePlanner();
		/**
		 * @brief 计算前向点
		 * @param goal 当前目标点
		 * @param distance_to_goal 到达最终终点的距离
		 * @return 返回前向点
		 */
		geometry_msgs::Point32 runSplineControl(const geometry_msgs::Point32& goal,
																						const double& distance_to_goal);
		/**
		 * @brief 判断机器人是否距离障碍物过近
		 * @return true or false
		 */
		bool isRobotAttachObstacle();

  private:
    /** const **/
		const double PI = 3.1415926;

    /** Node Handle **/
		// 节点句柄
    ros::NodeHandle ns; 

    /** Subscribers **/
		// 订阅激光信息
		ros::Subscriber scan_sub_;
    ros::Subscriber marker_sub_;
    ros::Subscriber environment_sub_;
    ros::Subscriber camera_left_sub_, camera_right_sub_;

    /** Publishers **/
		// 发布样条关节点云
		ros::Publisher joint_pub_;
		// 发布单个样条束点云
		ros::Publisher single_path_set_pub_;
		// 发布所有样条束点云
		ros::Publisher path_set_pub_;
		// 发布激光转点云的点云
		ros::Publisher scan_point_pub_;
		// 发布局部珊格地图
		ros::Publisher local_costmap_pub_, costmap_1_pub_, costmap_2_pub_, costmap_3_pub_, costmap_4_pub_, costmap_5_pub_, costmap_6_pub_;
		// 发布安全样条集合点云
		ros::Publisher safe_path_pub_, safe_path_1_pub_, safe_path_2_pub_, safe_path_3_pub_, safe_path_4_pub_, safe_path_5_pub_, safe_path_6_pub_;
		// 发布最优样条点云
		ros::Publisher best_path_pub_;

		/** Tf **/
		// TF接收
		tf::TransformListener* listener_; 

    /** Variables **/
		// Config文件地址
		std::string config_folder_;
		// 1米/2米/3米/4米原始样条集合
		std::vector<PathGroup> path_set_1_;
		std::vector<PathGroup> path_set_2_;
		std::vector<PathGroup> path_set_3_;
		std::vector<PathGroup> path_set_3_1_, path_set_3_2_, path_set_3_3_, path_set_3_4_, path_set_3_5_, path_set_3_6_;
		std::vector<PathGroup> path_set_4_;
		// 1米/2米/3米/4米采样窗口大小
		double path_window_radius_1_;
		double path_window_radius_2_;
		double path_window_radius_3_;
		double path_window_radius_4_;
		// 1米/2米/3米/4米映射到/map下的样条集合
		std::vector<MapToPath> map_to_path_1_;
		std::vector<MapToPath> map_to_path_2_;
		std::vector<MapToPath> map_to_path_3_;
		std::vector<MapToPath> map_to_path_4_;
		// 关节点集合
		std::vector<sensor_msgs::PointCloud> joints_set_;
		// 样条第一/二/三层数量
		int spline_1st_level_;
		int spline_2nd_level_;
		int spline_3rd_level_;
		// 角度
		double path_swap_range_;
		// 关节点云
		sensor_msgs::PointCloud joint_pointcloud_;
		// 插值数量
		double path_vertical_step_;
		// 高度标志位
		double spline_height_;
		// 样条id
		int id_num_;
		// 单个样条束
		std::vector<PathGroup> single_path_set_path_group_;
		// 样条束数量
		int spline_array_num_;
		// 所有样条束
		std::vector<PathGroup> path_set_path_group_;
		// 激光点云
		sensor_msgs::PointCloud scan_point_;
		// 局部珊格地图
		nav_msgs::OccupancyGrid costmap_local_;
		vector<nav_msgs::OccupancyGrid> costmap_local_vec_ = vector<nav_msgs::OccupancyGrid>(7);
		// 局部珊格地图分辨率
		double costmap_resolution_;
		// 障碍物集合（一维）
		vector<int> obstacle_vec_;
		// 安全样条集合
		std::vector<PathGroup> path_safe_set_;
		std::vector<PathGroup> path_safe_set_1_, path_safe_set_2_, path_safe_set_3_, path_safe_set_4_, path_safe_set_5_, path_safe_set_6_;
		// 当殿要使用的原始样条集合
		std::vector<PathGroup> path_set_init_;
		// 最优路径
		sensor_msgs::PointCloud path_best_;
		// 第1到2层最优路径
		sensor_msgs::PointCloud sub_1_path_best_;
		// 第2到3曾最优路径
		sensor_msgs::PointCloud sub_2_path_best_;
		// 选择中间的路权重
		double path_middle_weight_;
		// 选择靠近终点的路权重
		double path_goal_weight_;
		// 递归锁
		boost::recursive_mutex mutex_;
		// 局部目标
		geometry_msgs::Point32 local_goal_;
		// 局部目标子目标
		geometry_msgs::Point32 local_sub_goal_;
		// 采样范围
		int search_range_;
		// 机器人是否接近障碍物
		bool is_robot_close_obstacle_;
		// 选择路径与当前目标点夹角
		double face_goal_angle_;

    /** Functions **/
		/**
		 * @brief 写入MapToPath的TXT文件
		 * @param string_name 文件名后缀
		 * @param costmap 当前珊格地图
		 * @param Path_set 写入的样条集合
		 */
		void writeMapToPathTxt(const std::string& string_name,
													 const nav_msgs::OccupancyGrid& costmap,
													 const std::vector<PathGroup>& Path_set);
		
		/**
		 * @brief 读取CONFIG文件
		 */
		bool readConfig();

		/**
		 * @brief 打印CONFIG文件
		 */
		void printConfig();

		/**
		 * @brief 初始化样条
		 * @param Spline_name 样条名字
		 * @param Path_set 路径束
		 * @param Radius 半径
		 * @return ture or false
		 */
		bool InitSplineCurve(const std::string& Spline_name,
												 std::vector<PathGroup>& Path_set,
												 const double& Radius,
												 std::vector<MapToPath>& Map_to_Path);
		
		/**
		 * @brief 读取样条对应珊格地图id
		 * @param string_name txt文件名
		 * @param map_to_path 映射到map的路径
		 * @return true or false
		 */
		bool ReadAdjacentListTxt(const std::string& string_name, 
														 std::vector<MapToPath> &map_to_path);
		
		/**
		 * @brief 生成样条关节
		 * @param Output 关节点云
		 * @param Path_Radius 采样半径
		 */
		void GenerateSplinesJoints(std::vector<sensor_msgs::PointCloud>& Output,
															 const double& Path_Radius);
		
		/**
		 * @brief 将点云数组转换成点云
		 * @param Input 点云数组
		 * @param frame 目标坐标系
		 * @return 点云
		 */
		sensor_msgs::PointCloud ConvertVectortoPointcloud(const std::vector<PathGroup>& Input,
																											const std::string& frame);
		
		/**
		 * @brief 生成样条路径
		 * @param path_set 样条束
		 * @param joints 关节点
		 */
		void GenerateSplinesPath(std::vector<PathGroup>& path_set, 
														 const std::vector<sensor_msgs::PointCloud>& joints);
		
		/**
		 * @brief 计算样条
		 * @param Output 输出点云
		 * @param Point_1 控制点1
		 * @param Point_2 控制点2
		 * @param Point_3 控制点3
		 * @param Level 
		 */
		void ComputeSplines(sensor_msgs::PointCloud& Output,
												const geometry_msgs::Point32& Point_1,
												const geometry_msgs::Point32& Point_2,
												const geometry_msgs::Point32& Point_3,
												const double& Level);
		
		/**
		 * @brief 
		 * @param Path_set 
		 */
		void GenerateRevoluteSplinePath(vector<PathGroup> &Path_set);
		
		/**
		 * @brief 转换坐标系
		 * @param frame 当前坐标系
		 * @param target_frame 目标坐标系
		 * @param input 待转换坐标
		 * @param output 转换后坐标
		 * @param listener tf
		 * @return true or false
		 */
		bool transformPosePositioin(std::string frame,
																std::string target_frame,
																geometry_msgs::Pose& input,
																geometry_msgs::Pose& output,
																tf::TransformListener* listener);
		
		/**
		 * @brief 更新珊格地图
		 */
		bool updateCostmap();
		
		/**
		 * @brief 初始化珊格地图
		 * @param Costmap 珊格地图
		 */
		void initLocalCostmap(nav_msgs::OccupancyGrid& Costmap);
		void initLocalCostmap(vector<nav_msgs::OccupancyGrid>& Costmap_vec);
		
		/**
		 * @brief 更新珊格地图
		 * @param Costmap 珊格地图
		 * @param Obstacle 障碍物信息
		 */
		void setVfhLocalCostmap(nav_msgs::OccupancyGrid& Costmap, 
														const sensor_msgs::PointCloud& Obstacle);

		void setVfhLocalCostmap(vector<nav_msgs::OccupancyGrid>& Costmap_vec, 
														const vector<sensor_msgs::PointCloud>& Obstacle_vec);

		
		/**
		 * @brief 计算安全样条
		 * @param Cost_map 珊格地图
		 * @param Path_set 所有样条
		 * @param map_to_path 映射到map的样条id
		 */
		std::vector<PathGroup> ComputeSafePath(nav_msgs::OccupancyGrid Cost_map,
																						std::vector<PathGroup> Path_set,
																						std::vector<MapToPath> map_to_path);
		
		/**
		 * @brief 选择最优路径
		 * @param goal 目标点
		 * @return true or false
		 */
		bool selectBestPath(geometry_msgs::Point32 goal);
		
		/**
		 * @brief 选择最好的样条束
		 * @param goal 目标
		 * @return 样条束id
		 */
		int selectBestPathGroup(geometry_msgs::Point32 goal);
		
		/**
		 * @brief 选择最好的单个样条
		 * @param path_group_index 样条束id
		 * @return 单个样条id
		 */
		int selectBestSubPathGroup(int path_group_index);
		
		/**
		 * @brief 计算纯跟踪速度
		 * @param Goal_Route 目标半径
		 * @param Goal_Plan 目标点
		 * @param Pp_command 速度系数
		 */
		void ComputePurePursuitCommand(geometry_msgs::Point32 Goal_Route, 
																	 geometry_msgs::Point32 Goal_Plan,
																	 geometry_msgs::Twist& Pp_command);
		
		/**
		 * @brief 
		 * @param goal 
		 * @return 
		 */
		int decideSearchRange(geometry_msgs::Point32 goal);

		/** inline **/
		/**
		 * @brief 获取对数信息
		 * @param input 数值
		 * @param digit 次幂
		 * @return 数值
		 */
		inline int ComputeDigit(double input, int digit) 
		{
			if(input < pow(10,digit-1)) return 0;

			int reminder = int(input) % int(pow(10,digit));
			int output = reminder/pow(10,digit-1);

			return output;
  	}

		/**
		 * @brief 初始化四元数
		 * @param quaternion 四元数
		 */
		inline void initializeQuaternion(geometry_msgs::Quaternion& quaternion)
		{
			quaternion.x = 0;
			quaternion.y = 0;
			quaternion.z = 0;
			quaternion.w = 1;
		}

		/**
		 * @brief 笛卡尔坐标系点转一维
		 * @param grid 二维
		 * @param point 点
		 * @return 一维坐标
		 */
		inline int convertCartesianToLocalOccupany(nav_msgs::OccupancyGrid grid, geometry_msgs::Point32 point)
		{
			geometry_msgs::Point32 temp_point;
			temp_point.x = point.x - grid.info.origin.position.x;
			temp_point.y = point.y - grid.info.origin.position.y;

			int row_num = temp_point.x / grid.info.resolution;
			int col_num = temp_point.y / grid.info.resolution;

			int costmap_size = grid.info.width * grid.info.height;
			int output = col_num * grid.info.width + row_num;

			if(output > (costmap_size-1)) output = -1;
			return output;
  	}

		/** Callbacks **/
		/**
		 * @brief 激光回调
		 * @param Input 激光
		 */
		void scanCallback(const sensor_msgs::LaserScan::ConstPtr& Input)
		{
			scan_point_.points.clear();
			for (int i = 0; i < Input->ranges.size(); ++i) 
			{
				double current_angle = Input->angle_min + i * Input->angle_increment;
				// 激光变成点云
				if(Input->ranges[i] <= Input->range_min || Input->ranges[i] >= Input->range_max)
					continue;

				geometry_msgs::Point32 final_point;
				geometry_msgs::Pose scan_laser, scan_base;
				scan_laser.position.x = (Input->ranges[i] * cos(current_angle));
				scan_laser.position.y = (Input->ranges[i] * sin(current_angle));
				initializeQuaternion(scan_laser.orientation);
				transformPosePositioin(frame_base_which_, frame_base_which_, scan_laser, scan_base, listener_);
				final_point.x = scan_base.position.x;
				final_point.y = scan_base.position.y;

				scan_point_.points.push_back(final_point);
			}

			scan_point_.header.frame_id = frame_base_which_;
			scan_point_.header.stamp = ros::Time::now();
			scan_point_pub_.publish(scan_point_);
		}

    void markerCallback(const visualization_msgs::MarkerArray::ConstPtr& Input)
    {
      scan_point_.points.clear();
      visualization_msgs::MarkerArray marker_array = *Input;
      for (int i = 0; i < marker_array.markers.size(); ++i)
      {
        geometry_msgs::Point32 final_point;
        final_point.x = marker_array.markers[i].pose.position.x;
        final_point.y = marker_array.markers[i].pose.position.y;
        scan_point_.points.push_back(final_point);
      }
			
      scan_point_.header.frame_id = frame_base_which_;
			scan_point_.header.stamp = ros::Time::now();
			scan_point_pub_.publish(scan_point_);
    }
		string frame_base_which_ = "base_link";
		vector<sensor_msgs::PointCloud> pointcloud_vec_ = vector<sensor_msgs::PointCloud>(7);
		geometry_msgs::Point target_;
		void environmentCallback(const sensor_msgs::PointCloud::ConstPtr& Input)
		{
			scan_point_.points.clear();
			for (int i = 0; i < Input->points.size(); ++i)
			{
				geometry_msgs::Point32 final_point;
				geometry_msgs::Pose scan_laser, scan_base;
				scan_laser.position.x = Input->points[i].x;
				scan_laser.position.y = Input->points[i].y;
				scan_laser.position.z = Input->points[i].z;
				initializeQuaternion(scan_laser.orientation);
				transformPosePositioin(Input->header.frame_id, frame_base_which_, scan_laser, scan_base, listener_);
				if (fabs(scan_base.position.z) > 0.1) continue;
				final_point.x = scan_base.position.x;
				final_point.y = scan_base.position.y;
				final_point.z = 0;
				scan_point_.points.push_back(final_point);
			}

			sensor_msgs::PointCloud2::Ptr scan_front_pointcloud_two(new sensor_msgs::PointCloud2);
			pcl::PCLPointCloud2::Ptr pixel_cloud_filter(new pcl::PCLPointCloud2); 
			sensor_msgs::convertPointCloudToPointCloud2(scan_point_, *scan_front_pointcloud_two);
			pcl_conversions::toPCL(*scan_front_pointcloud_two, *pixel_cloud_filter);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_Radius(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromPCLPointCloud2(*pixel_cloud_filter, *cloud_after_Radius);

			//创建滤波器
			pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;
			//设置输入点云
			radiusoutlier.setInputCloud(cloud_after_Radius);
			//设置半径为100的范围内找临近点
			radiusoutlier.setRadiusSearch(0.2);
			//设置查询点的邻域点集数小于2的删除
			radiusoutlier.setMinNeighborsInRadius(2);
			radiusoutlier.filter(*cloud_after_Radius);

			sensor_msgs::PointCloud2 output2;
			pcl::toROSMsg(*cloud_after_Radius, output2);

			sensor_msgs::convertPointCloud2ToPointCloud(output2, scan_point_);

			scan_point_.header.frame_id = "base_link";
			scan_point_pub_.publish(scan_point_);
		}
		void TargetCallBack(const geometry_msgs::Point::ConstPtr& Input)
		{
			target_ = *Input;
		}
		
		
}; // SplinePlanner

#endif //UNITREE_NAV_ROS_SPLINE_PLANNER_H_