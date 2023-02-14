/*
 * Copyright 2022
 *
 * Author : Jingrao.Zhang@geely.com
 * Date : 2022-04-12
 * 
 * Description : A*
 */

#ifndef UNITREE_NAV_ROS_ASTAR_H_
#define UNITREE_NAV_ROS_ASTAR_H_

#include <vector>
#include <iostream>
#include <fstream>
#include <list>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <yaml-cpp/yaml.h>
#include <nav_msgs/OccupancyGrid.h>

struct Point
{
  //点坐标，这里为了方便按照C++的数组来计算，x代表横排，y代表竖列
  int x, y;
  //F=G+H
  int F, G, H;
  //parent的坐标，用指针，从而简化代码
  Point *parent;
  //变量初始化
  Point(int _x, int _y) :x(_x), y(_y), F(0), G(0), H(0), parent(nullptr)
  {}
};

class Astar
{
  public:
    /**
		 * @brief 初始化A*
		 * @param _maze A*点阵
		 * @return A*路径点云
		 */
    void initAstar(std::vector<std::vector<int>> &maze,
                   const nav_msgs::OccupancyGrid& map);

    /**
		 * @brief 获取A*路经点云
		 * @param startPoint A*起点
		 * @param endPoint A*终点
		 * @param isIgnoreCorner 是否忽略对角
		 * @return A*路径点云
		 */
    sensor_msgs::PointCloud getPath(Point &startPoint, 
                                    Point &endPoint, 
                                    bool isIgnoreCorner);

  private:
    /** const **/
    const int kCost1_ = 10;
    const int kCost2_ = 12;

    /** Variables **/
    double range_length_, range_width_; // 长、宽
    double resolution_; // 分辨率

    /** Functions **/
    /**
		 * @brief 寻找A*路径
		 * @param startPoint 起点
		 * @param endPoint 终点
		 * @param isIgnoreCorner 是否忽略对角
		 * @return A*路径点云
		 */
    Point *findPath(Point &startPoint, 
                    Point &endPoint, 
                    bool isIgnoreCorner);

    /**
		 * @brief 获取周围路径点
		 * @param point 中心点
		 * @param isIgnoreCorner 是否忽略对角
		 * @return 可行点数组
		 */
    std::vector<Point *> getSurroundPoints(const Point *point, 
                                           bool isIgnoreCorner);
    
    /**
		 * @brief 判断某点是否可以用于下一步判断
		 * @param point 起点
		 * @param target 目标
		 * @param isIgnoreCorner 是否忽略对角
		 * @return true or false
		 */
    bool isCanreach(const Point *point, 
                    Point *target, 
                    bool isIgnoreCorner) const;
    
    /**
		 * @brief 判断开启/关闭列表中是否包含某点
		 * @param list 待检查列表
		 * @param point 点
		 * @return 点
		 */
    Point *isInList(const std::list<Point *> &list, 
                    const Point *point) const;
    
    /**
		 * @brief 从开启列表中返回F值最小的节点
		 * @return 点
		 */
    Point *getLeastFpoint();
    
    /**
		 * @brief 计算FGH值
		 * @return 点
		 */
    int calcG(Point *temp_start, 
              Point *point);
    int calcH(Point *point, 
              Point *end);
    int calcF(Point *point);

  private:
    std::vector<std::vector<int>> maze_;
    //开启列表
    std::list<Point *> openList_;
    //关闭列表
    std::list<Point *> closeList_;
};

#endif //PATH_ALGORITHM_ASTAR_H
