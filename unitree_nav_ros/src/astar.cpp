
/*
 * Copyright 2022
 *
 * Author : Jingrao.Zhang@geely.com
 * Date : 2022-04-12
 * 
 * Description : A*
 */

#include "unitree_nav_ros/astar.h"

void Astar::initAstar(std::vector<std::vector<int>> &maze,
                      const nav_msgs::OccupancyGrid& map)
{
  maze_ = maze;
	
  range_length_ = map.info.origin.position.x;
  range_width_ = map.info.origin.position.y;
  resolution_ = 0.05;
}

int Astar::calcG(Point *temp_start, Point *point)
{
	// 计算calcG
  int extraG = (abs(point->x - temp_start->x) + abs(point->y - temp_start->y)) == 1 ? kCost1_ : kCost2_;
  int parentG = point->parent == NULL ? 0 : temp_start->G;
  return parentG + extraG;
}

int Astar::calcH(Point *point, Point *end)
{
	// 计算calcH
  return 
		sqrt((double)(end->x - point->x)*(double)(end->x - point->x) + 
				 (double)(end->y - point->y)*(double)(end->y - point->y)) * kCost1_;
}

int Astar::calcF(Point *point)
{
	// 计算calcGF
  return point->G + point->H;
}

Point *Astar::getLeastFpoint()
{
	// 如果OPENLIST是空
  if (!openList_.empty())
  {
    auto resPoint = openList_.front();
    for (auto &point : openList_)
      if (point->F < resPoint->F)
        resPoint = point;
    return resPoint;
  }
  return NULL;
}

Point *Astar::findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
  openList_.push_back(new Point(startPoint.x, startPoint.y));
  delete new Point(startPoint.x, startPoint.y);
  while (!openList_.empty())
  {
    auto curPoint = getLeastFpoint();
		// OPENLIST移除并放入CLOSELIST
    openList_.remove(curPoint);
    closeList_.push_back(curPoint);
    auto surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner);
    for (auto &target : surroundPoints)
    {
      if (!isInList(openList_, target))
      {
        target->parent = curPoint;
				// 计算G、H、F
        target->G = calcG(curPoint, target);
        target->H = calcH(target, &endPoint);
        target->F = calcF(target);

        openList_.push_back(target);
      }
      Point *resPoint = isInList(openList_, &endPoint);
      if (resPoint)
        return resPoint;
    }
  }
  return NULL;
}

sensor_msgs::PointCloud Astar::getPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
  Point *result = findPath(startPoint, endPoint, isIgnoreCorner);
  std::list<Point *> path;
  sensor_msgs::PointCloud a_star_path;
	// 遍历A*路径
  while (result)
  {
		// 根据地图原点修改每个点位置
    geometry_msgs::Point32 a_star_point;
    a_star_point.x = result->x * resolution_ + range_length_;
    a_star_point.y = result->y * resolution_ + range_width_;
    a_star_path.points.push_back(a_star_point);
    path.push_front(result);
    result = result->parent;
  }
  openList_.clear();
  closeList_.clear();
  return a_star_path;
}

Point *Astar::isInList(const std::list<Point *> &list, const Point *point) const
{
  for (auto p : list)
    if (p->x == point->x&&p->y == point->y) return p;
  return NULL;
}

bool Astar::isCanreach(const Point *point, Point *target, bool isIgnoreCorner) const
{
	// 判断是否可以到达周围的点
  if (target->y<0 || target->y>maze_[0].size() - 1 ||
      target->x<0 || target->x>maze_.size() - 1 ||
      maze_[target->x][target->y] == 1 ||
      target->x == point->x && target->y == point->y ||
      isInList(closeList_, target))
    return false;
  else
  {
    if (abs(point->x - target->x) + abs(point->y - target->y) == 1)
      return true;
    else
    {
      if (maze_[target->x][point->y] == 0 && maze_[point->x][target->y] == 0)
        return true;
      else
        return isIgnoreCorner;
    }
  }
}

std::vector<Point *> Astar::getSurroundPoints(const Point *point, bool isIgnoreCorner) 
{
	// 获取周围的点
  std::vector<Point *> surroundPoints;
	// 遍历周围9个点
  for (int x = point->x - 1; x <= point->x + 1; x++)
    for (int y = point->y - 1; y <= point->y + 1; y++)
    {
      Point *target = new Point(x, y);
      if (point->x == x && point->y == y) 
				continue;
			// 递归遍历
      if (isCanreach(point, target, isIgnoreCorner))
        surroundPoints.push_back(target);
    }

  return surroundPoints;
}
