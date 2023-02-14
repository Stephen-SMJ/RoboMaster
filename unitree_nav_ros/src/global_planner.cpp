/*
 * Copyright 2022
 *
 * Author : Jingrao.Zhang@geely.com
 * Date : 2022-04-12
 * 
 * Description : Global Planner for A1 quadruped robot.
 * Load map from slam or map_server node
 * Generate global costmap
 * link to A*
 * Flody smooth
 * Circle smooth
 */

#include "unitree_nav_ros/global_planner.h"

GlobalPlanner::GlobalPlanner()
  : initilize_(false)
{
	ROS_INFO("Global Planner Begin : ");
	// 订阅slam或者map_server node发出的map
	map_sub_            = nh.subscribe("/map", 1, &GlobalPlanner::MapCallBack, this);
	// 发布全局膨胀珊格地图
	global_costmap_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("global_costmap", 1);
	// 发布全局规划路径
	global_path_pub_    = nh.advertise<sensor_msgs::PointCloud>("global_path", 1);
	// 发布A*规划路径
	Astar_path_pub_     = nh.advertise<sensor_msgs::PointCloud>("A_STAR_path", 1);
	// 发布Flody第一次平滑路径
	flody_1_path_pub_   = nh.advertise<sensor_msgs::PointCloud>("FLODY_1_path", 1);
	// 发布Flody第二次平滑路径
	flody_2_path_pub_   = nh.advertise<sensor_msgs::PointCloud>("FLODY_2_path", 1);
	global_guide_pub_   = nh.advertise<sensor_msgs::PointCloud>("global_guide_path", 1);

	// 动态库链接A*
	astar_planner_      = std::make_shared<Astar>();
	
	// 打印全局Config文件
	readConfig();
	printConfig();
}

GlobalPlanner::~GlobalPlanner()
{
	ROS_INFO("Global Planner Finish!!!");
}

void GlobalPlanner::initGlobalPlanner()
{
	if (!initilize_)
	  // 处理地图
		expandMap(map_);

	// 发布全局规划
	global_costmap_pub_.publish(costmap_global_);
}

void GlobalPlanner::readConfig()
{
	// 读取map.yaml信息
	if (nh.getParam("image", map_msgs_.image))
	{
		nh.getParam("resolution", map_msgs_.resolution);
		nh.getParam("origin", map_msgs_.origin);
		nh.getParam("negate", map_msgs_.negate);
		nh.getParam("occupied_thresh", map_msgs_.occupied_thresh);
		nh.getParam("free_thresh", map_msgs_.free_thresh);
		// 读取GlobaoPlanner.yaml信息
		nh.getParam("costmap_resolution_", map_msgs_.costmap_resolution_);
		nh.getParam("safe_expand_size", map_msgs_.safe_expand_size);
	}
	else
		// 读取失败，查看launch里地址
		ROS_WARN("can't load global parameter");
}

sensor_msgs::PointCloud GlobalPlanner::getGlobalPlanner(const geometry_msgs::Pose& destination,
																												const RobotState& robot_current_state_)
{
	// 获取规划起始位置，是机器人所在位置
	start_point_.position.x = robot_current_state_.pose.position.x;
	start_point_.position.y = robot_current_state_.pose.position.y;

	// 清除旧规划
	global_path_.points.clear();
	// 获取A*路径
	sensor_msgs::PointCloud a_star_path = aStar(costmap_global_, destination);
	// 发布A*路径
	a_star_path.header.frame_id = "map";
	Astar_path_pub_.publish(a_star_path);
	// A*路径合法，进行flody平滑
	if (a_star_path.points.size() > 0)
	{
		// Flody第一次平滑，去掉斜率相同的点
		std::list<Point2 *> flody_1_list = floyd1Smooth(a_star_path);
		// Flody第二次平滑，去掉无障碍物通过的点
		sensor_msgs::PointCloud untreated_flody_2_path = floyd2Smooth(flody_1_list);
		sensor_msgs::PointCloud flody_2_path = cutGlobalPath(untreated_flody_2_path, 0.2);
		std::cout << "flody_2_path " << flody_2_path.points.size() << std::endl;
		// 发布Flody第二次平滑路径
		flody_2_path.header.frame_id = "map";
		flody_2_path_pub_.publish(flody_2_path);
		// 第三次平滑，夹角改为圆弧
		global_path_ = circleSmoothPath(flody_2_path);
		global_guide_path_ = cutGlobalPath(global_path_,0.5);
	}
	else
		// A*路径不合法
		ROS_ERROR("INVALID TARGET! TAKE CARE!");

	// 发布全局规划
	global_path_.header.frame_id = "map";
	global_path_pub_.publish(global_path_);

	global_guide_path_.header.frame_id = "map";
	global_guide_pub_.publish(global_guide_path_);

	return global_guide_path_;
}

void GlobalPlanner::expandMap(const nav_msgs::OccupancyGrid& map)
{
	// 珊格地图大小
	int map_size = map.info.width * map.info.height;
	static sensor_msgs::PointCloud material;

	// 提取地图边界
	for (int i = 0; i < map_size; ++i)
	{
		// 地图边界data=100
		if ((int)(map.data.at(i)) == 100)
		{
			geometry_msgs::Point32 map_point;
			map_point.x = (int)(i % map.info.width) * map.info.resolution + map.info.origin.position.x;
			map_point.y = (int)(i / map.info.width) * map.info.resolution + map.info.origin.position.y;
			material.points.push_back(map_point);
		}
	}
	// 地图非空
	if (material.points.size() != 0)
	{
		initilize_ = true;
		// 更新珊格地图
		updateCostmap(material);
	}
}

bool GlobalPlanner::updateCostmap(const sensor_msgs::PointCloud& environment)
{
	// 初始化珊格地图
	initLocalCostmap(costmap_global_);
	// 膨胀扇格地图
	setVfhLocalCostmap(costmap_global_, environment);

  return true;
}

void GlobalPlanner::initLocalCostmap(nav_msgs::OccupancyGrid& costmap)
{
	costmap = map_;
}

void GlobalPlanner::setVfhLocalCostmap(nav_msgs::OccupancyGrid& costmap,
																			 const sensor_msgs::PointCloud& boundary)
{
  int costmap_size = costmap.info.width * costmap.info.height;

	// 遍历所有边界
  for (int s = 0; s < boundary.points.size()-1; ++s)
  {
		// 二维坐标点转换成一维地图坐标系下
    geometry_msgs::Point32 temp_point;
    temp_point.x = boundary.points.at(s).x - costmap.info.origin.position.x;
    temp_point.y = boundary.points.at(s).y - costmap.info.origin.position.y;

    int row_num = temp_point.x / costmap.info.resolution;
    int col_num = temp_point.y / costmap.info.resolution;
    int obstacle_in_map = col_num * costmap.info.width + row_num;

		// 超过地图的点去除
    if (obstacle_in_map < 0 || obstacle_in_map >= costmap_size) continue;
		// 其余设置为“占据”
    costmap.data.at(obstacle_in_map) = OCCUPIED;

		// 遍历膨胀
		// 膨胀大小等于size / 分辨率（单位：m）
		double expand_size = map_msgs_.safe_expand_size / map_msgs_.resolution;
		// 纵向遍历
    for(int i = col_num - expand_size; i <= col_num + expand_size; i++)
    {
			// 纵向超出地图部分去除
      if(i < 0 || i > costmap.info.height - 1) continue;
			// 横向遍历
      for(int j = row_num - expand_size; j <= row_num + expand_size; j++)
      {
				// 横向超出地图部分去除
        if(j < 0 || j > costmap.info.width - 1) continue;
				// 地图边界外去除
				if (costmap.data[i * costmap.info.width + j] == -1)	continue;
				// 空白部分进行SAFE膨胀
        if(costmap.data.at(i * costmap.info.width + j) != OCCUPIED && costmap.data.at(i * costmap.info.width + j) != DANGEROUS)
        {
					// 膨胀边界为圆形
					geometry_msgs::Point32 obstacle_position;
					obstacle_position.x = j * costmap.info.resolution + costmap.info.origin.position.x;
					obstacle_position.y = i * costmap.info.resolution + costmap.info.origin.position.y;
					double obstacle_diatance = 
						hypot(obstacle_position.x - boundary.points[s].x, obstacle_position.y - boundary.points[s].y);
					// 最外层“SAFE”膨胀
					if (obstacle_diatance > expand_size * costmap.info.resolution) continue;
					costmap.data.at(i * costmap.info.width + j) = SAFE;
        }
      }
    }
  }
}

sensor_msgs::PointCloud GlobalPlanner::aStar(const nav_msgs::OccupancyGrid& map,
																						 const geometry_msgs::Pose& destination)
{
	sensor_msgs::PointCloud path;
	sensor_msgs::PointCloud apath;
	// A*点阵
	std::vector<std::vector<int>> maze;
	for (int i = 0; i < map.info.width; i++)
	{
		std::vector<int> element;
		for (int j = 0; j < map.info.height; j++)
		{
			geometry_msgs::Point32 learning_point;
      learning_point.x = i * 0.05 + map.info.origin.position.x;
      learning_point.y = j * 0.05 + map.info.origin.position.y;
			
			int data_num = j * map.info.width + i;
			int map_data_sign = (int)(map.data[data_num]);
			// 空白可通行标记为0
			if (map_data_sign == 0){
				apath.points.push_back(learning_point);
				element.push_back(0);
			}
			// 占据不可通行标记为1
			else
				element.push_back(1);
		}
		maze.push_back(element);
		element.clear();
	}

	// 计算在像素下规划起点和终点位置
	int start_x = (destination.position.x - map.info.origin.position.x) / map.info.resolution;
	int start_y = (destination.position.y - map.info.origin.position.y) / map.info.resolution;
	// 有TF测试：
	int end_x = (start_point_.position.x - map.info.origin.position.x) / map.info.resolution;
	int end_y = (start_point_.position.y - map.info.origin.position.y) / map.info.resolution;
	// 无TF测试：
	// int end_x = (0 - map.info.origin.position.x) / map.info.resolution;	
	// int end_y = (0 - map.info.origin.position.y) / map.info.resolution;

	Point start(start_x, start_y);
	Point end(end_x, end_y);

	// 起点与终点重合
	if (end_x == start_x && end_y == start_y) return path;
	// 动态链接到A*库
	std::cout << "map.info.origin " << map.info.origin.position.x << " " << map.info.origin.position.y << std::endl;
	// A*初始化
	astar_planner_->initAstar(maze, map);
	// 获取A*路径
	path = astar_planner_->getPath(start, end, true);

	return path;
}

std::list<Point2 *> GlobalPlanner::floyd1Smooth(const sensor_msgs::PointCloud& original_path)
{
	std::list<Point2 *> original_list;
	std::list<Point2 *> floyd_list;
	for (int i = 0; i < original_path.points.size(); ++i)
	{
		// 计算斜率，放在原始列表中
		double slope = (i >= original_path.points.size() - 1) ? NULL :
										atan2(original_path.points[i+1].y - original_path.points[i].y,
													original_path.points[i+1].x - original_path.points[i].x);
		original_list.push_back(new Point2(original_path.points[i].x,
																			 original_path.points[i].y,
																			 slope));
	}

	// 遍历原始列表
	while (!original_list.empty())
	{
		auto resPoint = original_list.front();
		// 移除首个元素
		original_list.remove(resPoint);

		// flody列表放入第一个元素
		if (floyd_list.empty()) floyd_list.push_back(resPoint);
		// flody列表放入最后一个元素
		if (original_list.empty()) floyd_list.push_back(resPoint);
		// 斜率不同的元素放入flody列表
		if (resPoint->slope != floyd_list.back()->slope) floyd_list.push_back(resPoint);
	}

	return floyd_list;
}

sensor_msgs::PointCloud GlobalPlanner::floyd2Smooth(const std::list<Point2 *>& floyd_list)
{
	// 列表转点云
	sensor_msgs::PointCloud path;
	for (auto &flody_p : floyd_list)
	{
		geometry_msgs::Point32 p;
		p.x = flody_p->x;
		p.y = flody_p->y;
		path.points.push_back(p);
	}

	// 发布Flody第一次平滑路径
	path.header.frame_id = "map";
	flody_1_path_pub_.publish(path);

	// flody路经点序列位置
	int flody_num = 0;
	// 去除Flody路径点中没有经过障碍物部分的点
	// 遍历起点
	for (int i = 0; i < path.points.size(); ++i)
	{
		// 从第flody_num进行终点遍历
		flody_num = i;
		// 遍历终点
		for (int j = i+1; j < path.points.size(); ++j)
		{
			// 判断是否有障碍物
			if (!isRoutObstructed(path.points[i], path.points[j], costmap_global_))
				// 静态存储flody_num作为终点的位置
				if (flody_num < j)
					flody_num = j;
		}
		i = flody_num;
		// 终点位置记为需要拐弯的终点
		path.points[flody_num].z = CORNER;
	}

	// flody起点和终点需要保存
	path.points[0].z = CORNER;
	path.points[path.points.size() - 1].z = CORNER;

	// 将需要拐弯的点保存成新路径
	sensor_msgs::PointCloud flody_path;
	for (int i = 0; i < path.points.size(); ++i)
		if (path.points[i].z == CORNER) 
			flody_path.points.push_back(path.points[i]);

	return flody_path;
}

bool GlobalPlanner::isRoutObstructed(const geometry_msgs::Point32& start,
																		 const geometry_msgs::Point32& end,
																		 const nav_msgs::OccupancyGrid& costmap)
{
	// 计算两点距离
	double distance = hypot(start.x - end.x, start.y - end.y);
	// 计算插值检查个数
	int inter_num = (int)(distance / costmap.info.resolution);
	// 遍历途中所有点
	for (int i = 0; i <= inter_num; ++i)
	{
		// 检查途中点middle又没有经过障碍物
		Point2 middle(0,0,0);
		double t = (costmap.info.resolution / distance) * i;
		middle.x = start.x * t + end.x * (1 - t);
		middle.y = start.y * t + end.y * (1 - t);
		// 二维坐标转一维地图坐标
		int row_num = (middle.x - costmap.info.origin.position.x) / costmap.info.resolution;
		int col_num = (middle.y - costmap.info.origin.position.y) / costmap.info.resolution;
		int map_data = col_num * costmap.info.width + row_num;
		// 被占据
		if ((int)(costmap.data[map_data]) != EMPTY && (int)(costmap.data[map_data]) != SAFE) 
			return true;
	}
	return false;
}

sensor_msgs::PointCloud GlobalPlanner::circleSmoothPath(const sensor_msgs::PointCloud& flody_path)
{
	sensor_msgs::PointCloud path;
	// 点到点
	if (flody_path.points.size() == 2)
	{
		// 按照0.1m进行插值
		double distance_12 = hypot(flody_path.points[0].x - flody_path.points[1].x,
															 flody_path.points[0].y - flody_path.points[1].y);
		int inter_num_12 = (int)(distance_12 / 0.1);

		for (int j = 0; j <= inter_num_12; ++j)
		{
			geometry_msgs::Point32 p_s;
			double t_12 = (0.1 / distance_12) * j;
			p_s.x = flody_path.points[0].x * (1 - t_12) + flody_path.points[1].x * t_12;
			p_s.y = flody_path.points[0].y * (1 - t_12) + flody_path.points[1].y * t_12;
			path.points.push_back(p_s);
		}

		return path;
	}

	Circle circle;
	// 圆弧半径
	circle.r = 0.5;
	// 记录上一个起点
	geometry_msgs::Point32 last_start = flody_path.points[0];
	for (int i = 1; i < flody_path.points.size(); ++i)
	{
		// 特殊情况：flody路径的最后一个点
		if (i == flody_path.points.size() - 1)
		{
			// 按照0.1进行差插值
			double distance = hypot(last_start.x - flody_path.points[flody_path.points.size() - 1].x,
															last_start.y - flody_path.points[flody_path.points.size() - 1].y);
			int inter_num = (int)(distance / 0.1);

			// 插值
			for (int j = 0; j <= inter_num; ++j)
			{
				geometry_msgs::Point32 p_s;
				double t = (0.1 / distance) * j;
				p_s.x = last_start.x * (1 - t) + flody_path.points[flody_path.points.size() - 1].x * t;
				p_s.y = last_start.y * (1 - t) + flody_path.points[flody_path.points.size() - 1].y * t;
				path.points.push_back(p_s);
			}
			// 跳出循环
			continue;
		}

		// 获得夹角角度
		double angle = get_angle(last_start.x, last_start.y,
														 flody_path.points[i].x, flody_path.points[i].y,
														 flody_path.points[i+1].x, flody_path.points[i+1].y);
		// 计算开始曲线平滑的距离
		double s = (sin(PI / 2 - angle / 2) * circle.r) / sin(angle / 2);
		// 当前点与前一个点距离
		double distance_12 = hypot(last_start.x - flody_path.points[i].x,
															 last_start.y - flody_path.points[i].y) - s;
		// 当前点与后一个点距离
		double distance_23 = hypot(flody_path.points[i].x - flody_path.points[i+1].x,
															 flody_path.points[i].y - flody_path.points[i+1].y) - s;
		// 当前点与前一个点插值个数
		int inter_num_12 = (int)(distance_12 / 0.1);
		// 当前点与后一个点插值个数
		int inter_num_23 = (int)(distance_23 / 0.1);
		// 平滑起点
		double circle_start_x =
			flody_path.points[i].x * (distance_12 / (distance_12 + s)) +
			(last_start.x) * (1 - (distance_12 / (distance_12 + s)));
		double circle_start_y =
			flody_path.points[i].y * (distance_12 / (distance_12 + s)) +
			(last_start.y) * (1 - (distance_12 / (distance_12 + s)));

		// 记录前一个点到到平滑起点
		// 插值
		for (int j = 0; j <= inter_num_12; ++j)
		{
			geometry_msgs::Point32 p_s;
			double t_12 = (0.1 / distance_12) * j;
			p_s.x = last_start.x * (1 - t_12) + circle_start_x * t_12;
			p_s.y = last_start.y * (1 - t_12) + circle_start_y * t_12;
			path.points.push_back(p_s);
		}
		
		// 计算平滑起点垂直于前一个点到当前点函数
		double k1 = (last_start.y - flody_path.points[i].y) /
								(last_start.x - flody_path.points[i].x);
		double k11 = - 1 / k1;
		double b1 = circle_start_y - k11 * circle_start_x;
		// 平滑终点
		double end_x =
			flody_path.points[i].x * (distance_23 / (distance_23+s)) +
			(flody_path.points[i+1].x) * (1 - (distance_23 / (distance_23 + s)));
		double end_y =
			flody_path.points[i].y * (distance_23 / (distance_23+s)) +
			(flody_path.points[i+1].y) * (1 - (distance_23 / (distance_23 + s)));

		// 静态存储本次终点作为下一次起点	
		last_start.x = end_x;
		last_start.y = end_y;

		// 计算平滑终点垂直于当前点到后一个点的函数
		double k2 = (flody_path.points[i].y - flody_path.points[i+1].y) /
								(flody_path.points[i].x - flody_path.points[i+1].x);
		double k22 = - 1 / k2;
		double b2 = end_y - k22 * end_x;

		// 计算圆心位置
		circle.origin.x = (b2 - b1) / (k11 - k22);
		circle.origin.y = k11 * circle.origin.x + b1;

		// 处理斜率等于0的特殊情况
		if (k1 == 0)
		{
			circle.origin.x = circle_start_x;
			circle.origin.y = k22 * circle.origin.x + b2;
		}
		if (k2 == 0)
		{
			circle.origin.x = end_x;
			circle.origin.y = k11 * circle.origin.x + b1;
		}

		// 计算圆弧在极坐标系下的起点和终点
		double atan_1 = atan2(circle_start_y - circle.origin.y, circle_start_x - circle.origin.x);
		double atan_2 = atan2(end_y - circle.origin.y, end_x - circle.origin.x);

		// 根据不同情况进行圆弧插值
		if ((ComputeSign(atan_1) == ComputeSign(atan_2)))
		{
			if (ComputeSign(atan_2 - atan_1) < 0)
				pushCirclePathSub(atan_1, atan_2, circle, path, ComputeSign(atan_2 - atan_1));
			else
				pushCirclePathAdd(atan_1, atan_2, circle, path, ComputeSign(atan_2 - atan_1));
		}
		else
		{
			if (fabs(atan_1) + fabs(atan_2) < fabs(PI - fabs(atan_1)) + fabs(PI - fabs(atan_2)))
			{
				if (ComputeSign(atan_2) < 0)
					pushCirclePathSub(atan_1, atan_2, circle, path, ComputeSign(atan_2));
				else 
					pushCirclePathAdd(atan_1, atan_2, circle, path, ComputeSign(atan_2));
			}
			else 
			{
				if (ComputeSign(atan_1) > 0)
				{
					pushCirclePathAdd(atan_1, PI, circle, path, ComputeSign(atan_1));
					pushCirclePathAdd(-PI, atan_2, circle, path, ComputeSign(atan_1));
				}
				else
				{
					pushCirclePathSub(atan_1, -PI, circle, path, ComputeSign(atan_1));
					pushCirclePathSub(PI, atan_2, circle, path, ComputeSign(atan_1));
				}
			}
		}
	}

	return path;
}

void GlobalPlanner::pushCirclePathAdd(const double& atan_start,
																			const double& atan_end,
																			const Circle& circle,
																			sensor_msgs::PointCloud& path,
																			const int& sign)
{
	// 按照0.2rad进行插值
	double d_a = 0.2;
	if (sign == 1)
	{
		for (double i = atan_start; i <= atan_end; i += d_a)
		{
			geometry_msgs::Point32 p;

			p.x = circle.origin.x + circle.r * cos(i);
			p.y = circle.origin.y + circle.r * sin(i);
			path.points.push_back(p);
		}
	}
	else if (sign == -1)
	{
		for (double i = atan_start; i >= atan_end; i += d_a)
		{
			geometry_msgs::Point32 p;
			p.x = circle.origin.x + circle.r * cos(i);
			p.y = circle.origin.y + circle.r * sin(i);
			path.points.push_back(p);
		}
	}
}

void GlobalPlanner::pushCirclePathSub(const double& atan_start,
																			const double& atan_end,
																			const Circle& circle,
																			sensor_msgs::PointCloud& path,
																			const int& sign)
{
	// 按照0.2rad进行插值
	double d_a = 0.2;
	if (sign == 1)
	{
		for (double i = atan_start; i <= atan_end; i -= d_a)
		{
			geometry_msgs::Point32 p;
			p.x = circle.origin.x + circle.r * cos(i);
			p.y = circle.origin.y + circle.r * sin(i);
			path.points.push_back(p);
		}
	}
	else if (sign == -1)
	{
		for (double i = atan_start; i >= atan_end; i -= d_a)
		{
			geometry_msgs::Point32 p;
			p.x = circle.origin.x + circle.r * cos(i);
			p.y = circle.origin.y + circle.r * sin(i);
			path.points.push_back(p);
		}
	}
}

sensor_msgs::PointCloud GlobalPlanner::cutGlobalPath(const sensor_msgs::PointCloud& path, const double& distance)
{
	sensor_msgs::PointCloud cut_global_path;
	int start_num = 0;
	for (int i = 0; i < path.points.size(); ++i)
	{
		if (i == 0 || i == path.points.size()-1)
			cut_global_path.points.push_back(path.points[i]);
		else 
		{
			double delta_distance = 
				hypot(path.points[start_num].x - path.points[i].x, path.points[start_num].y - path.points[i].y);
			if (delta_distance >= distance)
			{
				cut_global_path.points.push_back(path.points[i]);
				start_num = i;
			}
		}
	}
	return cut_global_path;
}

void GlobalPlanner::printConfig()
{
	std::cout << "==================================" << std::endl;
	std::cout << "       GlobalPlanner Config       " << std::endl;
	std::cout << "==================================" << std::endl;
	std::cout << "image : " << map_msgs_.image << std::endl;
	std::cout << "resolution : " << map_msgs_.resolution << std::endl;
	std::cout << "origin[] : " << map_msgs_.origin[0] << " " << map_msgs_.origin[1] << " " << map_msgs_.origin[2] << std::endl;
	std::cout << "negate : " << map_msgs_.negate << std::endl;
	std::cout << "occupied_thresh : " << map_msgs_.occupied_thresh << std::endl;
	std::cout << "free_thresh : " << map_msgs_.free_thresh << std::endl;
	std::cout << "safe_expand_size : " << map_msgs_.safe_expand_size << std::endl;
	std::cout << "==================================" << std::endl;
}