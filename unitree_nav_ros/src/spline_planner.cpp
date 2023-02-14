#include "unitree_nav_ros/spline_planner.h"

SplinePlanner::SplinePlanner()
{
	ROS_INFO("Spline Planner Begin : ");
	ns.param<string>("config_folder",config_folder_,"");

	ros::NodeHandle private_nh("~");
  // 发布样条关节点云
	joint_pub_ = ns.advertise<sensor_msgs::PointCloud>("/joint",10);
  // 发布单个样条束点云
	single_path_set_pub_ = ns.advertise<sensor_msgs::PointCloud>("/single_path_set",10);
  // 发布所有样条束点云
	path_set_pub_ = ns.advertise<sensor_msgs::PointCloud>("/path_set",10);
  // 发布激光转点云的点云
	scan_point_pub_ = ns.advertise<sensor_msgs::PointCloud>("/scan_to_target_point",10);
  // 发布局部珊格地图
	local_costmap_pub_ = ns.advertise<nav_msgs::OccupancyGrid>("/local_costmap",10);
	costmap_1_pub_ = ns.advertise<nav_msgs::OccupancyGrid>("/costmap_1",10);
	costmap_2_pub_ = ns.advertise<nav_msgs::OccupancyGrid>("/costmap_2",10);
	costmap_3_pub_ = ns.advertise<nav_msgs::OccupancyGrid>("/costmap_3",10);
	costmap_4_pub_ = ns.advertise<nav_msgs::OccupancyGrid>("/costmap_4",10);
	costmap_5_pub_ = ns.advertise<nav_msgs::OccupancyGrid>("/costmap_5",10);
	costmap_6_pub_ = ns.advertise<nav_msgs::OccupancyGrid>("/costmap_6",10);
  // 发布安全样条集合点云
	safe_path_pub_ = ns.advertise<sensor_msgs::PointCloud>("/safe_path",10);
	safe_path_1_pub_ = ns.advertise<sensor_msgs::PointCloud>("/safe_path_1",10);
	safe_path_2_pub_ = ns.advertise<sensor_msgs::PointCloud>("/safe_path_2",10);
	safe_path_3_pub_ = ns.advertise<sensor_msgs::PointCloud>("/safe_path_3",10);
	safe_path_4_pub_ = ns.advertise<sensor_msgs::PointCloud>("/safe_path_4",10);
	safe_path_5_pub_ = ns.advertise<sensor_msgs::PointCloud>("/safe_path_5",10);
	safe_path_6_pub_ = ns.advertise<sensor_msgs::PointCloud>("/safe_path_6",10);
  // 发布最优样条点云
	best_path_pub_ = ns.advertise<sensor_msgs::PointCloud>("/best_path",10);

  // 订阅激光信息
	scan_sub_ = ns.subscribe("/scan",1, &SplinePlanner::scanCallback,this);
	marker_sub_ = ns.subscribe("/marker_array",1, &SplinePlanner::markerCallback,this);
	environment_sub_ = ns.subscribe("/scan_point",1, &SplinePlanner::environmentCallback,this);

	listener_ = new tf::TransformListener(ns, ros::Duration(5.0), true);

  // 读取并打印config文件
	readConfig();
  cout << "===readConfig^ ===" << endl;
	printConfig();
}

SplinePlanner::~SplinePlanner()
{
	ROS_INFO("Spline Planner Finish!!!");
}

void SplinePlanner::writeMapToPathTxt(const std::string& string_name,
																			const nav_msgs::OccupancyGrid& costmap,
													 						const std::vector<PathGroup>& Path_set)
{
	sensor_msgs::PointCloud temp_pointcloud;
  int txt_size = (search_range_ / costmap.info.resolution) * (search_range_ / costmap.info.resolution) * 4 - 1;
  std::vector<std::vector<int>> matrix{txt_size};

	for (int j = 0; j < Path_set.size(); ++j)
	{
		temp_pointcloud.points.clear();

		for(int i = 0; i < Path_set[j].path_pointcloud.points.size(); i++)
			temp_pointcloud.points.push_back(Path_set[j].path_pointcloud.points[i]);

		for (int i = 2; i < temp_pointcloud.points.size()-1; ++i)
		{
			temp_pointcloud.header.frame_id = frame_base_which_;
			geometry_msgs::Point32 start = temp_pointcloud.points[i];
			geometry_msgs::Point32 end = temp_pointcloud.points[i+1];

			// 计算两点距离
			double distance = hypot(start.x - end.x, start.y - end.y);
			// 计算插值检查个数
			int inter_num = (int)(distance / costmap.info.resolution);
			// 遍历途中所有点
			for (int k = 0; k <= inter_num; ++k)
			{
				// 检查途中点middle又没有经过障碍物
				Point2 middle(0,0,0);
				double t = (costmap.info.resolution / distance) * k;
				middle.x = start.x * t + end.x * (1 - t);
				middle.y = start.y * t + end.y * (1 - t);
				// 二维坐标转一维地图坐标
				int row_num = (middle.x - costmap.info.origin.position.x) / costmap.info.resolution;
				int col_num = (middle.y - costmap.info.origin.position.y) / costmap.info.resolution;
				int map_data = col_num * costmap.info.width + row_num;

				if (matrix[map_data].size() >= 1) 
				{
					if (matrix[map_data][matrix[map_data].size()-1] != j)
						matrix[map_data].push_back(j);
				}
				else matrix[map_data].push_back(j);
			}
		}
	}

	std::ofstream ofs;
	ofs.open(string_name, ios::trunc);

  // 写入对应的TXT
	for (int i = 0; i < matrix.size(); ++i)
	{
		ofs << i << "   ";
		if (matrix[i].size() != 0)
		{
			for (int j = 0; j < matrix[i].size(); ++j)
				ofs << matrix[i][j] << " ";
		}
		ofs << endl;
	}
	ofs.close();
}

bool SplinePlanner::readConfig()
{
  path_window_radius_1_	= 1;
  path_window_radius_2_	= 2;
  path_window_radius_3_	= 3;
	path_window_radius_4_	= 4;

	spline_1st_level_ = 1;
	spline_2nd_level_	= 7;
	spline_3rd_level_ = 5;
	path_swap_range_ = 1.57;

	path_vertical_step_ = 0.2;
	spline_height_ = 0;
	id_num_ = 0;

	spline_array_num_ = 15;

	costmap_resolution_ = 0.1;

	path_middle_weight_ = 0;
	path_goal_weight_ = 1;

  search_range_ = 3;

	std::string config_name = config_folder_+ "/cfg/spline_config.txt";
  std::string spline_map_name_1 = config_folder_ + "/cfg/map_to_path_1m.txt";
  std::string spline_map_name_2 = config_folder_ + "/cfg/map_to_path_2m.txt";
  std::string spline_map_name_3 = config_folder_ + "/cfg/map_to_path_3m.txt";
	std::string spline_map_name_4 = config_folder_ + "/cfg/map_to_path_4m.txt";

	cout << "config_name " << config_name << endl;
	cout << "spline_map_name_1 " << spline_map_name_1 << endl;
	cout << "spline_map_name_2 " << spline_map_name_2 << endl;
	cout << "spline_map_name_3 " << spline_map_name_3 << endl;
	cout << "spline_map_name_4 " << spline_map_name_4 << endl;

  // if(!InitSplineCurve(spline_map_name_1,path_set_1_,path_window_radius_1_,map_to_path_1_)) return false;
  // if(!InitSplineCurve(spline_map_name_2,path_set_2_,path_window_radius_2_,map_to_path_2_)) return false;
  if(!InitSplineCurve(spline_map_name_3,path_set_3_,path_window_radius_3_,map_to_path_3_)) {cout << "InitSplineCurve" << endl;return false;}
	// if(!InitSplineCurve(spline_map_name_4,path_set_4_,path_window_radius_4_,map_to_path_4_)) return false;
  return true;
}

bool SplinePlanner::InitSplineCurve(const std::string& spline_name,
																		std::vector<PathGroup> &path_set,
																		const double& radius,
																		std::vector<MapToPath> &map_to_path)
{
  cout << "===InitSplineCurve===" << endl;
  if(!ReadAdjacentListTxt(spline_name, map_to_path)) return false;
  joints_set_.clear();
  path_set.clear();
  // 生成样条关节
  GenerateSplinesJoints(joints_set_, radius);
  // 生承单支样条束
  GenerateSplinesPath(path_set, joints_set_);
  // 生成所有样条
  // 投影到/base_link系下
  GenerateRevoluteSplinePath(path_set);
  cout << "===InitSplineCurve===" << endl;

  return true;
}

bool SplinePlanner::ReadAdjacentListTxt(const std::string& string_name,
																				std::vector<MapToPath> &map_to_path)
{	
  map_to_path.clear();
  std::ifstream node_file;
  node_file.open(string_name);

  if (!node_file.is_open())
	{
  	cout << "Could not find Node File" << endl;
  	cout << "File Not Exist At: " << string_name << endl;
  	return false;
  }

  string str;
	int t = 0;
  // 逐行读取txt
  while (std::getline(node_file, str)) 
	{
    MapToPath sub_map_to_path;
		std::stringstream ss(str);
    bool map_enable = true;
    int sub_path_id;
    while (!ss.eof()) 
		{
      if (map_enable)
				ss >> sub_map_to_path.map_data_num;
      else
        if (ss >> sub_path_id)
          sub_map_to_path.path_id.push_back(sub_path_id);

      map_enable = false;
    }
    map_to_path.push_back(sub_map_to_path);
	}
	node_file.close();

	return true;
}

void SplinePlanner::GenerateSplinesJoints(std::vector<sensor_msgs::PointCloud>& output,
																					const double& path_radius) 
{
  output.clear();
	// 关节数量是3
  const int temp_splines_joints_num = 3;
	// 扇形第一层的开扇角度
  double first_shrink_scale = temp_splines_joints_num * 0.95;
	// 扇形第二层的开扇角度
  double second_shrink_scale = temp_splines_joints_num * 0.55;
	// Path_Radius是搜索距离：1米、2米、3米、4米
	// 单位半径
  double radius_unit = path_radius / temp_splines_joints_num;

	// 分支数量分别是1、7、5
  vector<int> level_nums = {spline_1st_level_, spline_2nd_level_, spline_3rd_level_}; //{1,7,5};
	// 分支初始化{0, 0, 0}
  vector<double> level_unit(temp_splines_joints_num,0);
	// 点云数组:size = 1
  vector<sensor_msgs::PointCloud> temp_vector(level_nums[0]);

  path_swap_range_ = 1.57;

  if(level_nums[0] == 1) level_unit[0] = 0;
  else {
		ROS_ERROR("debug");
		level_unit[0] = path_swap_range_ / double(level_nums[0] - 1);
	}

  // 计算每一层单位角度
  level_unit[1] = 
		path_swap_range_ / (first_shrink_scale * double(level_nums[1] - 1));
  level_unit[2] = 
		level_unit[1] * double(level_nums[1] - 1) / (second_shrink_scale * double(level_nums[2] - 1));
  
	sensor_msgs::PointCloud temp_level;
  // temp_level.header.frame_id = "/base_link";
  // temp_level.header.stamp = ros::Time::now();
  
	geometry_msgs::Point32 temp_joint;

  for (int i = 0; i < temp_vector.size(); ++i)
	{
    double level_0_angle = 0;
    temp_joint.x = radius_unit * 0.5 * cos(level_0_angle);
		// 由radius_unit*1 -> radius_unit*0.5, 缩短第一个节点距离
    // 单位距离0.5米
    temp_joint.y = radius_unit * 0.5 * sin(level_0_angle); 
    temp_joint.z = i+1;
    temp_level.points.push_back(temp_joint);

    for (int j = 0; j < level_nums[1]; ++j)
		{
      double level_1_angle = level_0_angle - ((level_nums[1] - 1) * level_unit[1]/2) + j * level_unit[1];
      // 单位距离1.5米
      temp_joint.x = radius_unit * 1.5 * cos(level_1_angle);
      temp_joint.y = radius_unit * 1.5 * sin(level_1_angle);
      temp_joint.z = i+1 + (j+1) * 10;
      temp_level.points.push_back(temp_joint);

      for (int k = 0; k < level_nums[2]; ++k)
			{
        double level_2_angle = level_1_angle - ((level_nums[2] - 1) * level_unit[2]/2) + k * level_unit[2];
        // 单位距离3米
        temp_joint.x = radius_unit * 3 * cos(level_2_angle);
        temp_joint.y = radius_unit * 3 * sin(level_2_angle);
        temp_joint.z = i+1 + (j+1) * 10 + (k+1) * 100;
        temp_level.points.push_back(temp_joint);
      }
    }
    temp_vector[i] = temp_level;
    temp_level.points.clear();
  }

  output = temp_vector;
	joint_pointcloud_ = temp_vector[0];
	joint_pointcloud_.header.frame_id = "map";
}

void SplinePlanner::GenerateSplinesPath(std::vector<PathGroup>& path_set, 
																				const std::vector<sensor_msgs::PointCloud>& joints)
{
  path_set.clear();
  geometry_msgs::Point32 point_1;
  geometry_msgs::Point32 point_2;
  geometry_msgs::Point32 point_3;

  sensor_msgs::PointCloud path_temp;
  PathGroup path_group_temp;
  path_temp.header.frame_id = "/base_link";
  path_temp.header.stamp = ros::Time::now();
  
  for (int i = 0; i < joints.size(); ++i)
    for (int j = 0; j < joints[i].points.size(); ++j)
		{
      // 根据.z信息，将每层点分开赋值
      if(int(std::log10(joints[i].points[j].z)) == 0) point_1 = joints[i].points[j];
      if(int(std::log10(joints[i].points[j].z)) == 1) point_2 = joints[i].points[j];
      if(int(std::log10(joints[i].points[j].z)) == 2) point_3 = joints[i].points[j];

      // 对应属于同一束样条
      if(ComputeDigit(point_1.z,1) == ComputeDigit(point_2.z,1) && 
				 ComputeDigit(point_1.z,1) == ComputeDigit(point_3.z,1) && 
				 ComputeDigit(point_2.z,2) == ComputeDigit(point_3.z,2))
			{
        //生成单条样条曲线
        ComputeSplines(path_temp,point_1,point_2,point_3,i);

        path_group_temp.path_pointcloud = path_temp;
        path_group_temp.path_pointcloud.channels.resize(1);
        path_group_temp.path_pointcloud.channels[0].name = "path_id";
        path_group_temp.path_pointcloud.channels[0].values.resize(path_temp.points.size(),1);
        // 赋予样条ID
        path_group_temp.path_id = id_num_;
        id_num_++;

        path_set.push_back(path_group_temp);
      }
    }
	single_path_set_path_group_ = path_set;
}

void SplinePlanner::ComputeSplines(sensor_msgs::PointCloud& Output,
																	 const geometry_msgs::Point32& Point_1,
																   const geometry_msgs::Point32& Point_2,
																	 const geometry_msgs::Point32& Point_3,
																	 const double& Level)
{
  Output.points.clear();
  geometry_msgs::Point32 temp_output;
  geometry_msgs::Point32 Point_0;
  vector<geometry_msgs::Point32> Point_group;

  vector<double> path_h(3,0);
  vector<double> path_a(3,0);
  vector<double> path_b(3,0);
  vector<double> path_c(3,0);
  vector<double> path_d(3,0);
  vector<double> path_m(4,0);

  bool x_input;
  bool variable_increase;

  double temp_p_1;
  double temp_p_2;
  
  // 根据从近到远来生成样条
  // 符合生成样条的要求
  if(Point_1.x > 0 && Point_1.x < Point_2.x && Point_2.x < Point_3.x)
	{
    x_input = true; 
    variable_increase = true; 
    Point_group.push_back(Point_0);
    Point_group.push_back(Point_1);
    Point_group.push_back(Point_2);
    Point_group.push_back(Point_3);
  }
  // 不符合
	else if(Point_1.x < 0 && Point_1.x > Point_2.x && Point_2.x > Point_3.x)
	{
    x_input = true; 
    variable_increase = false; 
    Point_group.push_back(Point_3);
    Point_group.push_back(Point_2);
    Point_group.push_back(Point_1);
    Point_group.push_back(Point_0);
  }
  // 不符合
	else if(Point_1.y > 0 && Point_1.y < Point_2.y && Point_2.y < Point_3.y)
	{ 
    x_input = false; 
    variable_increase = true; 
    Point_group.push_back(Point_0);
    Point_group.push_back(Point_1);
    Point_group.push_back(Point_2);
    Point_group.push_back(Point_3);
  }
  // 不符合
	else if(Point_1.y < 0 && Point_1.y > Point_2.y && Point_2.y > Point_3.y)
	{ 
    x_input = false; 
    variable_increase = false; 
    Point_group.push_back(Point_3);
    Point_group.push_back(Point_2);
    Point_group.push_back(Point_1);
    Point_group.push_back(Point_0);
  }

  // 如果符合要求
  if(x_input && variable_increase)
	{
    // 不同层样条的间距
    path_h[0] = Point_group[1].x - Point_group[0].x;
    path_h[1] = Point_group[2].x - Point_group[1].x;
    path_h[2] = Point_group[3].x - Point_group[2].x;

    temp_p_1 = 
			6 * ((Point_group[2].y - Point_group[1].y)/path_h[1] - (Point_group[1].y - Point_group[0].y)/path_h[0]);
    temp_p_2 = 
			6 * ((Point_group[3].y - Point_group[2].y)/path_h[2] - (Point_group[2].y - Point_group[1].y)/path_h[1]);

    path_m[0] = 0;
    path_m[2] = 
			((path_h[0]+path_h[1]) * temp_p_2 * 2 - path_h[1] * temp_p_1) / (4 * (path_h[0] + path_h[1]) * (path_h[1] + path_h[2]) - 
      pow(path_h[1],2));
    path_m[1] = 
			(temp_p_1 - path_h[1] * path_m[2]) / (2 * (path_h[0] + path_h[1]));
    path_m[3] = 0;

    for(int i = 0; i < path_h.size(); i++)
		{
      path_a[i] = Point_group[i].y;
      path_b[i] = (Point_group[i+1].y - Point_group[i].y) / path_h[i] - 
									path_h[i] * path_m[i] / 2 - 
									path_h[i] * (path_m[i+1] - 
									path_m[i]) / 6;
      path_c[i] = path_m[i] / 2;
      path_d[i] = (path_m[i+1] - path_m[i]) / path_h[i] / 6;
    }

    double range = fabs(Point_3.x);
    double iter_num = 15;
    path_vertical_step_ = range / iter_num;
    // 按照要求对样条进行插值
    for (int j = 0; j <= iter_num; j++)
		{
      temp_output.x = j * path_vertical_step_;
      // 符合要求记录插值点
      if (temp_output.x >= Point_group[0].x && temp_output.x <= Point_group[1].x)
        temp_output.y = path_a[0] + 
												path_b[0] * (temp_output.x - Point_group[0].x) + 
												path_c[0] * pow(temp_output.x - Point_group[0].x,2) + 
												path_d[0] * pow(temp_output.x - Point_group[0].x,3);
      else if (temp_output.x >= Point_group[1].x && temp_output.x <= Point_group[2].x)
        temp_output.y = path_a[1] + 
												path_b[1] * (temp_output.x - Point_group[1].x) + 
												path_c[1] * pow(temp_output.x - Point_group[1].x,2) + 
												path_d[1] * pow(temp_output.x - Point_group[1].x,3);
      else if (temp_output.x >= Point_group[2].x && temp_output.x <= Point_group[3].x) 
        temp_output.y = path_a[2] + 
												path_b[2] * (temp_output.x - Point_group[2].x) + 
												path_c[2] * pow(temp_output.x - Point_group[2].x,2) + 
												path_d[2] * pow(temp_output.x - Point_group[2].x,3);
			else 
        continue;

      temp_output.z = spline_height_;
      Output.points.push_back(temp_output);
    }
    spline_height_+=0.1;
  }
}

void SplinePlanner::GenerateRevoluteSplinePath(vector<PathGroup> &Path_set)
{
	// 平均角度10度，0.174rad
  double average_angle = 10.0 / 180 * PI;
	// 单边样条数量(int)(4.5) = 5
  int single_num = spline_array_num_ / 2;
	// 初始位置
  double shrink_rate = -average_angle * single_num;
	// z标志位
  double z_height = 10;
	// 初始化为单条样条
  vector<PathGroup> Path_set_init = Path_set;
	// 曲线等级
  double path_scale = 1000;

  for(int index = 0; index < spline_array_num_; index++)
	{
    double revolute_angle = shrink_rate;
    shrink_rate += average_angle;
    if (index == single_num) continue;

    for (int i = 0; i < Path_set_init.size(); i++)
		{
      PathGroup Path_sub_path;
      for(int j = 0; j < Path_set_init[i].path_pointcloud.points.size(); j++)
			{
        geometry_msgs::Point32 temp_point;
        if(j == 0) 
				{
          temp_point.x = 0;
          temp_point.y = 0;
          temp_point.z = Path_set_init[i].path_pointcloud.points[j].z + z_height; ;
        }
				else 
				{
          double point_distance = 
						hypot(Path_set_init[i].path_pointcloud.points[j].x, Path_set_init[i].path_pointcloud.points[j].y);
          double costheta = 
						Path_set_init[i].path_pointcloud.points[j].x / point_distance;
          double sintheta = 
						Path_set_init[i].path_pointcloud.points[j].y / point_distance;

          temp_point.x = 
						point_distance * (costheta * cos(revolute_angle) - sintheta * sin(revolute_angle));
          temp_point.y = 
						point_distance * (sintheta * cos(revolute_angle) + costheta * sin(revolute_angle));
          temp_point.z = 
						Path_set_init[i].path_pointcloud.points[j].z + z_height;
        }
        Path_sub_path.path_pointcloud.points.push_back(temp_point);
      }

      double temp_path_id = 
				Path_set[i].path_pointcloud.channels[0].values[0] + path_scale;
      
			Path_sub_path.path_pointcloud.channels.resize(1);
      Path_sub_path.path_pointcloud.channels[0].name = "path_id";
      Path_sub_path.path_pointcloud.channels[0].values.resize(Path_sub_path.path_pointcloud.points.size(),1);
      Path_sub_path.path_id = id_num_;
      
			id_num_++;
      Path_set.push_back(Path_sub_path);
    }
    z_height+=10;
    path_scale+=1000;
  }
  id_num_ = 0;
	path_set_path_group_ = Path_set;
}

geometry_msgs::Point32 SplinePlanner::runSplineControl(const geometry_msgs::Point32& goal, const double& distance_to_goal)
{
	boost::unique_lock<boost::recursive_mutex> lock(mutex_);
  // geometry_msgs::Point32 goal;
  // goal.x = 3;
  // goal.y =0.1;
  // goal.z = 0.5;
  // // goal
  // double pitch_goal = atan(goal.z / goal.x);
  // cout << "pitch_goal " << pitch_goal << endl;
  // if (pitch_goal > 0 && pitch_goal < 0.1) frame_base_which_ = "base_link";
  // else
  // {
  // int set_goal = (pitch_goal > 0.5) ? 1 : 
  //   ((pitch_goal > 0.3) ? 2 : 
  //     ((pitch_goal > 0.1) ? 3 : 
  //       ((pitch_goal > -0.1) ? 4 : 
  //         ((pitch_goal > -0.3) ? 5 : 
  //           ((pitch_goal > -0.5) ? 6 : -999)))));

  
  // frame_base_which_ = "base_link" + to_string(set_goal);
  // }
  // cout << "frame_base_which_ " << frame_base_which_ << endl;

  // if (distance_to_goal < 1) 
  //   return goal;
	joint_pub_.publish(joint_pointcloud_);
	single_path_set_pub_.publish(ConvertVectortoPointcloud(single_path_set_path_group_, frame_base_which_));
	path_set_pub_.publish(ConvertVectortoPointcloud(path_set_path_group_, frame_base_which_));

  // costmap_local_ = costmap_local_vec_[set_goal];

	search_range_ = 3;
	updateCostmap();
	path_safe_set_ = ComputeSafePath(costmap_local_, path_set_3_, map_to_path_3_);
//   /*
// 	path_safe_set_1_ = ComputeSafePath(costmap_local_vec_[1], path_set_3_, map_to_path_1_);
// 	path_safe_set_2_ = ComputeSafePath(costmap_local_vec_[2], path_set_3_, map_to_path_1_);
// 	path_safe_set_3_ = ComputeSafePath(costmap_local_vec_[3], path_set_3_, map_to_path_1_);
// 	path_safe_set_4_ = ComputeSafePath(costmap_local_vec_[4], path_set_3_, map_to_path_1_);
// 	path_safe_set_5_ = ComputeSafePath(costmap_local_vec_[5], path_set_3_, map_to_path_1_);
// 	path_safe_set_6_ = ComputeSafePath(costmap_local_vec_[6], path_set_3_, map_to_path_1_);
//   */
// 	// if (path_safe_set_.size() != 0)
// 	// 	search_range_ = 3;
// 	// else 
// 	// 	search_range_ = 0;

	sensor_msgs::PointCloud temp_pointcloud = ConvertVectortoPointcloud(path_safe_set_, frame_base_which_);
	temp_pointcloud.header.frame_id = frame_base_which_;
	safe_path_pub_.publish(temp_pointcloud);

	selectBestPath(goal);
  for (int i = 0; i < sub_2_path_best_.points.size(); ++i)
    sub_2_path_best_.points[i].z = 0;
	sub_2_path_best_.header.frame_id = frame_base_which_;
	if(sub_2_path_best_.points.size() > 0) 
		best_path_pub_.publish(sub_2_path_best_);

// 	/*
// 	writeMapToPathTxt(config_folder_ + "/cfg/map_to_path_1m.txt", costmap_local_, path_set_1_);
// 	writeMapToPathTxt(config_folder_ + "/cfg/map_to_path_2m.txt", costmap_local_, path_set_2_);
// 	writeMapToPathTxt(config_folder_ + "/cfg/map_to_path_3m.txt", costmap_local_, path_set_3_);
// 	writeMapToPathTxt(config_folder_ + "/cfg/map_to_path_4m.txt", costmap_local_, path_set_4_);
// 	*/

  int path_lookahead_index = sub_2_path_best_.points.size()/2.5;

  local_goal_ = goal;
  if (sub_2_path_best_.points.size() > path_lookahead_index)
    local_sub_goal_ = sub_2_path_best_.points[path_lookahead_index];
  return local_sub_goal_;
}
/*
bool SplinePlanner::updateCostmap()
{
  initLocalCostmap(costmap_local_);
	if (!scan_point_.points.empty())
	setVfhLocalCostmap(costmap_local_, scan_point_);

  return true;
}
*/

bool SplinePlanner::updateCostmap()
{
  initLocalCostmap(costmap_local_);
  setVfhLocalCostmap(costmap_local_, scan_point_);

  costmap_local_.header.frame_id = "base_link";
  local_costmap_pub_.publish(costmap_local_);
  return true;
}


void SplinePlanner::initLocalCostmap(nav_msgs::OccupancyGrid& Costmap)
{
  std::vector<signed char> map_data;
  geometry_msgs::Pose map_origin;

  Costmap.header.frame_id = frame_base_which_;
  Costmap.header.stamp = ros::Time::now();
  Costmap.info.resolution = costmap_resolution_;

  Costmap.info.width  = round((6) / Costmap.info.resolution);
  Costmap.info.height = round((6) / Costmap.info.resolution);

  int costmap_size = Costmap.info.width * Costmap.info.height;
  map_data.resize(costmap_size);

  // COSTMAP中心点的位置
  map_origin.position.x = - 3;
  map_origin.position.y = - 3;
  map_origin.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

  Costmap.info.origin = map_origin;
  Costmap.data = map_data;
}

void SplinePlanner::initLocalCostmap(vector<nav_msgs::OccupancyGrid>& Costmap_vec)
{
  for (int i = 1; i < 7; ++i)
  {
    nav_msgs::OccupancyGrid Costmap;
    std::vector<signed char> map_data;
    geometry_msgs::Pose map_origin;

    Costmap.header.frame_id = frame_base_which_ + to_string(i);
    Costmap.header.stamp = ros::Time::now();
    Costmap.info.resolution = costmap_resolution_;

    Costmap.info.width  = round((3 * 1) / Costmap.info.resolution);
    Costmap.info.height = round((3 * 1) / Costmap.info.resolution);

    int costmap_size = Costmap.info.width * Costmap.info.height;
    map_data.resize(costmap_size);

    // COSTMAP中心点的位置
    map_origin.position.x = - 1.5;
    map_origin.position.y = - 1.5;
    map_origin.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

    Costmap.info.origin = map_origin;
    Costmap.data = map_data;

    Costmap_vec[i] = Costmap;
  }
}

void SplinePlanner::setVfhLocalCostmap(nav_msgs::OccupancyGrid& Costmap, const sensor_msgs::PointCloud& Obstacle)
{
	obstacle_vec_.clear();
  int costmap_size = Costmap.info.width * Costmap.info.height;
  int collision_expand_size_ = 3;
	int OCCUPIED = 30;
  is_robot_close_obstacle_ = false;
  for (int s = 0; s < Obstacle.points.size(); ++s) 
  {
    if (fabs(Obstacle.points[s].x) > search_range_ || fabs(Obstacle.points[s].y) > search_range_)
      continue;
    if (fabs(Obstacle.points[s].x) < collision_expand_size_ * Costmap.info.resolution || fabs(Obstacle.points[s].y) < collision_expand_size_ * Costmap.info.resolution)
      is_robot_close_obstacle_ = true;
    
    geometry_msgs::Point32 temp_point;
    temp_point.x = Obstacle.points.at(s).x - Costmap.info.origin.position.x;
    temp_point.y = Obstacle.points.at(s).y - Costmap.info.origin.position.y;

    int row_num = temp_point.x / Costmap.info.resolution;
    int col_num = temp_point.y / Costmap.info.resolution;

    int costmap_size = Costmap.info.width * Costmap.info.height;
    int obstacle_in_map = col_num * Costmap.info.width + row_num;
    // 如果障碍物在检测范围之外
    if (obstacle_in_map < 0 || obstacle_in_map >= costmap_size)
      continue;
    // 障碍物标记被占据
    Costmap.data.at(obstacle_in_map) = OCCUPIED;
		obstacle_vec_.push_back(obstacle_in_map);
    // 如果障碍物的膨胀在检测范围之外
    for(int i = col_num - (collision_expand_size_); i <= col_num + (collision_expand_size_); i++)
    {
      if(i < 0 || i >= Costmap.info.height - 1)
        continue;
      for(int j = row_num - (collision_expand_size_); j <= row_num + (collision_expand_size_); j++)
      {
        if(j < 0 || j >= Costmap.info.width - 1)
          continue;
        // 开始膨胀
        if(Costmap.data.at(i * Costmap.info.width + j) != OCCUPIED)
        {
					geometry_msgs::Point32 obstacle_position;
					obstacle_position.x = 
						(int)((i * Costmap.info.width + j) % Costmap.info.width) * costmap_resolution_ + Costmap.info.origin.position.x;
					obstacle_position.y = 
						(int)((i * Costmap.info.width + j) / Costmap.info.width) * costmap_resolution_ + Costmap.info.origin.position.y;
					double obstacle = hypot(obstacle_position.x - Obstacle.points[s].x, obstacle_position.y - Obstacle.points[s].y);
					
					// The expansion's shape in costmap if round.
					if (obstacle > collision_expand_size_ * costmap_resolution_) continue;
					// This layer is safe layer, robot can gei in this layer to buffer.
					Costmap.data.at(i * Costmap.info.width + j) = OCCUPIED;
					obstacle_vec_.push_back(i * Costmap.info.width + j);
        }
      }
    }
  }
}

void SplinePlanner::setVfhLocalCostmap(vector<nav_msgs::OccupancyGrid>& Costmap_vec, const vector<sensor_msgs::PointCloud>& Obstacle_vec)
{
  for (int i = 1; i < 7; ++i)
  {
    nav_msgs::OccupancyGrid Costmap = Costmap_vec[i];
    obstacle_vec_.clear();
    int costmap_size = Costmap.info.width * Costmap.info.height;
    int collision_expand_size_ = 3;
    int OCCUPIED = 30;
    is_robot_close_obstacle_ = false;
    for (int s = 0; s < Obstacle_vec[i].points.size(); ++s) 
    {
      if (fabs(Obstacle_vec[i].points[s].x) > search_range_ || fabs(Obstacle_vec[i].points[s].y) > search_range_)
        continue;
      if (fabs(Obstacle_vec[i].points[s].x) < collision_expand_size_ * Costmap.info.resolution || fabs(Obstacle_vec[i].points[s].y) < collision_expand_size_ * Costmap.info.resolution)
        is_robot_close_obstacle_ = true;
      
      geometry_msgs::Point32 temp_point;
      temp_point.x = Obstacle_vec[i].points.at(s).x - Costmap.info.origin.position.x;
      temp_point.y = Obstacle_vec[i].points.at(s).y - Costmap.info.origin.position.y;

      int row_num = temp_point.x / Costmap.info.resolution;
      int col_num = temp_point.y / Costmap.info.resolution;

      int costmap_size = Costmap.info.width * Costmap.info.height;
      int obstacle_in_map = col_num * Costmap.info.width + row_num;
      // 如果障碍物在检测范围之外
      if (obstacle_in_map < 0 || obstacle_in_map >= costmap_size)
        continue;
      // 障碍物标记被占据
      Costmap.data.at(obstacle_in_map) = OCCUPIED;
      obstacle_vec_.push_back(obstacle_in_map);
      // 如果障碍物的膨胀在检测范围之外
      for(int i = col_num - (collision_expand_size_); i <= col_num + (collision_expand_size_); i++)
      {
        if(i < 0 || i > Costmap.info.height - 1)
          continue;
        for(int j = row_num - (collision_expand_size_); j <= row_num + (collision_expand_size_); j++)
        {
          if(j < 0 || j > Costmap.info.width - 1)
            continue;
          // 开始膨胀
          if(Costmap.data.at(i * Costmap.info.width + j) != OCCUPIED)
          {
            geometry_msgs::Point32 obstacle_position;
            obstacle_position.x = 
              (int)((i * Costmap.info.width + j) % Costmap.info.width) * costmap_resolution_ + Costmap.info.origin.position.x;
            obstacle_position.y = 
              (int)((i * Costmap.info.width + j) / Costmap.info.width) * costmap_resolution_ + Costmap.info.origin.position.y;
            double obstacle = hypot(obstacle_position.x - Obstacle_vec[i].points[s].x, obstacle_position.y - Obstacle_vec[i].points[s].y);
            
            // The expansion's shape in costmap if round.
            if (obstacle > collision_expand_size_ * costmap_resolution_) continue;
            // This layer is safe layer, robot can gei in this layer to buffer.
            Costmap.data.at(i * Costmap.info.width + j) = OCCUPIED;
            obstacle_vec_.push_back(i * Costmap.info.width + j);
          }
        }
      }
    }
    Costmap_vec[i] = Costmap;
  }
}

std::vector<PathGroup> SplinePlanner::ComputeSafePath(nav_msgs::OccupancyGrid Cost_map,
                                                      std::vector<PathGroup> Path_set,
                                                      std::vector<MapToPath> map_to_path)
{
  std::vector<PathGroup> _path_safe_set;
  path_set_init_.clear();
  path_set_init_ = Path_set;
  vector<int> temp_path_id;

  // 遍历障碍物数组
  for(int i = 0; i < obstacle_vec_.size(); i++)
	{
    // 在TXT中查找对应的样条曲线ID
    int obs_id = obstacle_vec_[i];
    int single_path_id;
    for(int j = 0; j < map_to_path[obs_id].path_id.size(); j++)
		{
      single_path_id = map_to_path[obs_id].path_id[j];
			if (path_set_init_.size() - 1 >= single_path_id)
			{
				path_set_init_[single_path_id].path_pointcloud.channels[0].values.assign(
					path_set_init_[single_path_id].path_pointcloud.points.size(),-1
				);
			}
      // 记录ID

      temp_path_id.push_back(single_path_id);
    } 
  }
  // 记录安全ID
  for (int ii = 0; ii < path_set_init_.size(); ii++) 
	{
    PathGroup temp_path_group;
    if(path_set_init_[ii].path_pointcloud.channels[0].values.front() != -1) 
		{
      double path_group_id = 
				path_set_init_[ii].path_id / (spline_2nd_level_ * spline_3rd_level_);

      temp_path_group.path_id = path_set_init_[ii].path_id;
      temp_path_group.path_pointcloud = path_set_init_[ii].path_pointcloud;
      _path_safe_set.push_back(temp_path_group);
    }
  }

  return _path_safe_set;
}

bool SplinePlanner::selectBestPath(geometry_msgs::Point32 goal)
{
  path_best_.points.clear();
  sub_1_path_best_.points.clear();
  sub_2_path_best_.points.clear();
  if(path_safe_set_.empty()) return false;

  int path_group_index;
  int optimal_index;

  // 目标评分
  path_group_index = selectBestPathGroup(goal);
  // 钻则最优路径
  optimal_index = selectBestSubPathGroup(path_group_index);

  bool enable_push_back = true;
  // 动态窗口查找最优路径
  for(int k = 0; k < path_safe_set_.size(); k++)
	{
    if(path_safe_set_[k].path_id / spline_3rd_level_ == optimal_index)
		{
      sub_1_path_best_.points.insert(
				sub_1_path_best_.points.end(), 
				path_safe_set_[k].path_pointcloud.points.begin(),
				path_safe_set_[k].path_pointcloud.points.end()
			);

			if(enable_push_back) 
			{
				sub_2_path_best_.points = path_safe_set_[k].path_pointcloud.points;
				enable_push_back = false;
			}
    }
  }
  return true;
}

int SplinePlanner::selectBestPathGroup(geometry_msgs::Point32 goal)
{
  face_goal_angle_ = PI;
  // compute path group cost, choose max_feasible_pro fist group path
  int path_group_index;
  vector<double> path_cost(spline_array_num_,0);
  int costmap_size = 
		costmap_local_.info.width * costmap_local_.info.height;
  int check_around = 10;
  double left_distance_min = 1;
  double right_distance_min = 1;
  vector<double> middle_cost(spline_array_num_,0);
  vector<double> goal_cost(spline_array_num_,0);
  // 归一化处理
  double middle_cost_sum = 0; 
  // 归一化处理
  double goal_cost_sum = 0; 

  for(int i = 0; i < path_safe_set_.size(); i++) 
	{
    int path_1st_group_index = 
			path_safe_set_[i].path_id / (spline_2nd_level_ * spline_3rd_level_);

    // 增加路径中间点空旷度的权重,查找横向最短障碍物距离,最长为1
    geometry_msgs::Point32 middle_point = 
			path_safe_set_[i].path_pointcloud.points[path_safe_set_[i].path_pointcloud.points.size()/2];
    // 一维转二维
    int check_id = convertCartesianToLocalOccupany(costmap_local_, middle_point);
    for (int j = check_id; j < check_id+check_around * costmap_local_.info.width; j = j + costmap_local_.info.width)
		{
      if (j >= costmap_size || j < 0) break;
      if (costmap_local_.data[j] > 20)
			{
        left_distance_min = 
					(j-check_id)/costmap_local_.info.width * costmap_local_.info.resolution;
        break;
      }
    }

    for (int k = check_id; k < check_id-check_around * costmap_local_.info.width; k = k - costmap_local_.info.width)
		{
      if(k >= costmap_size || k < 0) break;
      if(costmap_local_.data[k] > 20)
			{
        right_distance_min = 
					(k-check_id)/costmap_local_.info.width * costmap_local_.info.resolution;
        break;
      }
    }

    double middle_distance_min = 
			left_distance_min > right_distance_min ? right_distance_min : left_distance_min;
    middle_cost[path_1st_group_index] += pow(middle_distance_min,1);
    middle_cost_sum += pow(middle_distance_min,1);
    
    // 增加goal目标点权重,朝向角在45°范围内权重为N倍
    geometry_msgs::Point32 candidate_point = path_safe_set_[i].path_pointcloud.points.back();
    double candidate_distance = 
			hypot(candidate_point.x,candidate_point.y);
    double path_goal_distance = 
			hypot((candidate_point.y - goal.y),(candidate_point.x - goal.x));
    if (path_goal_distance == 0) 
			path_goal_distance = pow(10,-10);

    double goal_distance = hypot(goal.x,goal.y);
    double face_goal_angle = acos((pow(goal_distance, 2) + pow(candidate_distance, 2) -
                                   pow(path_goal_distance, 2)) / (2 * goal_distance * candidate_distance));
    // 记录当前束中与目标点差角最小值
    if (fabs(face_goal_angle) < fabs(face_goal_angle_)) 
      face_goal_angle_ = face_goal_angle;

    // 如果角度差小于PI/6
    if (face_goal_angle <= PI/6)
		{
      goal_cost[path_1st_group_index] += 1.0 / pow(path_goal_distance,1);
      goal_cost_sum += 1.0 / pow(path_goal_distance,1);
    } 
		else 
		{
      goal_cost[path_1st_group_index] += 1.0 / pow(path_goal_distance,3);
      goal_cost_sum += 1.0 / pow(path_goal_distance,3);     
    }    
  }

  for(int m = 0; m < path_cost.size(); m++)
    path_cost[m] = 
			path_middle_weight_ * middle_cost[m]/middle_cost_sum + path_goal_weight_ * goal_cost[m]/goal_cost_sum;

  double max_feasible_prob = -1;
  for (int j = 0; j < path_cost.size(); j++)
    if (path_cost[j] > max_feasible_prob)
		{
      max_feasible_prob = path_cost[j];
      path_group_index = j;
    }

  return path_group_index;
}

int SplinePlanner::selectBestSubPathGroup(int path_group_index)
{
  static int last_index = -1;
  int optimal_index;
  vector<int> sub_path_index;
  vector<int> sub_path_cost(spline_2nd_level_,0);

  // 遍历安全样条束
  for (int k = 0; k < path_safe_set_.size(); k++) 
    if (path_safe_set_[k].path_id / (spline_2nd_level_ * spline_3rd_level_) == path_group_index)
		{
      int check_id = 
				path_safe_set_[k].path_id % (spline_2nd_level_ * spline_3rd_level_);
      int check_index = 
				check_id / spline_3rd_level_;
      sub_path_cost[check_index] += 1;
    }

  int sub_max_feasible_prob = INT_MIN;
  // 遍历第二层样条所有值
  for (int ii = 0; ii < sub_path_cost.size(); ii++)
  // 记录最小值
    if (sub_path_cost[ii] > sub_max_feasible_prob)
		{
      optimal_index = path_group_index * spline_2nd_level_ + ii;
      sub_max_feasible_prob = sub_path_cost[ii];
    }

  // 遍历第三层样条所有值
  for (int jj = 0; jj < sub_path_cost.size(); jj++)
    // 记录最小值
    if (sub_path_cost[jj] == sub_max_feasible_prob)
      sub_path_index.push_back(jj);

  int sub_max_feasible_prob_index = sub_path_index.size()/2;
  // 返回最优样条束
  optimal_index = 
		path_group_index * spline_2nd_level_ + sub_path_index[sub_max_feasible_prob_index];
  last_index = optimal_index;
  return optimal_index;
}

sensor_msgs::PointCloud SplinePlanner::ConvertVectortoPointcloud(const std::vector<PathGroup>& Input,
																																 const std::string& frame)
{
  sensor_msgs::PointCloud temp_pointcloud;

  if(Input.empty()) return temp_pointcloud;

  temp_pointcloud.channels.resize(1);
  temp_pointcloud.channels[0].name = "path_id";
  // 遍历数组
  for(int j = 0; j < Input.size(); j++)
    // 遍历数组中的点云
    for(int i = 0; i < Input[j].path_pointcloud.points.size(); i++)
		{
      temp_pointcloud.points.push_back(Input[j].path_pointcloud.points[i]);
      temp_pointcloud.channels[0].values.push_back(Input[j].path_pointcloud.channels[0].values[i]);
    }

  for (int i = 0; i < temp_pointcloud.points.size(); ++i)
    temp_pointcloud.points[i].z = 0;

	temp_pointcloud.header.frame_id = frame;
  temp_pointcloud.header.stamp = ros::Time::now();
  return temp_pointcloud;
}

bool SplinePlanner::transformPosePositioin(std::string frame,
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

int SplinePlanner::decideSearchRange(geometry_msgs::Point32 goal)
{
  // 根据目标距离觉得样条采样范围
  double distance_to_goal = hypot(goal.x, goal.y);
  double search_range = (distance_to_goal > 4 ? 4 : 
                          ((distance_to_goal > 3 ? 3 : 
                            ((distance_to_goal > 2) ? 2 : 
                              ((distance_to_goal > 1) ? 1 : 0)))));
  return search_range;
}

bool SplinePlanner::isRobotAttachObstacle()
{
  // 返回机器人是否距离障碍物太近
  return is_robot_close_obstacle_;
}

void SplinePlanner::printConfig()
{
	std::cout << "==================================" << std::endl;
	std::cout << "          Spline Config           " << std::endl;
	std::cout << "==================================" << std::endl;

	std::cout << "==================================" << std::endl;
}