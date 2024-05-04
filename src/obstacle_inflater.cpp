/**
 * @file obstacle_inflater.cpp
 * @author Toshiki Nakamura
 * @brief C++ implementation of obstacle inflater
 * @date 2024-05-04
 * @copyright Copyright (c) 2024
 */

#include <algorithm>
#include <vector>

#include "obstacle_inflater/obstacle_inflater.h"

ObstacleInflater::ObstacleInflater() : private_nh_("~")
{
  private_nh_.param<int>("hz", hz_, 1);
  private_nh_.param<float>("inflation_radius", inflation_radius_, 0.15);

  inflated_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map/inflated", 1);
  raw_map_sub_ = nh_.subscribe("/map", 1, &ObstacleInflater::map_callback, this);

  ROS_INFO_STREAM(ros::this_node::getName() << "node has started..");
  ROS_INFO_STREAM("hz: " << hz_);
  ROS_INFO_STREAM("inflation_radius: " << inflation_radius_);
}

void ObstacleInflater::process()
{
  ros::Rate loop_rate(hz_);
  while (ros::ok())
  {
    ros::spinOnce();
    inflated_map_pub_.publish(inflated_map_);
    loop_rate.sleep();
  }
}

void ObstacleInflater::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg) { inflate_obstacle(*msg); }

void ObstacleInflater::inflate_obstacle(const nav_msgs::OccupancyGrid &map)
{
  inflated_map_ = map;
  if (inflation_radius_ <= map.info.resolution)
  {
    ROS_WARN("inflation_radius is smaller than resolution, so inflation is not executed.");
    return;
  }

  for (int i = 0; i < map.data.size(); i++)
  {
    if (map.data[i] == 100)
      change_surrounding_grid_color(i, inflation_radius_, inflated_map_);
  }
}

void ObstacleInflater::change_surrounding_grid_color(
    const int center_grid_index, const float target_margin, nav_msgs::OccupancyGrid &map)
{
  const std::vector<int> rect_grid_index_list = get_rect_grid_index_list(center_grid_index, target_margin, map);
  for (const auto &index : rect_grid_index_list)
  {
    if (in_margin(index, center_grid_index, target_margin, map))
    {
      if (index < map.data.size())
        map.data[index] = 100;
      else
        ROS_WARN("index out of range");
    }
  }
}

bool ObstacleInflater::in_margin(
    const int index, const int center_grid_index, const float target_margin, const nav_msgs::OccupancyGrid &map)
{
  const float dist = calc_dist_between_grid(center_grid_index, index, map);
  return dist <= target_margin;
}

float ObstacleInflater::calc_dist_between_grid(const int index1, const int index2, const nav_msgs::OccupancyGrid &map)
{
  const int index1_x = calc_grid_index_x(index1, map);
  const int index1_y = calc_grid_index_y(index1, map);
  const int index2_x = calc_grid_index_x(index2, map);
  const int index2_y = calc_grid_index_y(index2, map);

  return hypot(index1_x - index2_x, index1_y - index2_y) * map.info.resolution;
}

std::vector<int> ObstacleInflater::get_rect_grid_index_list(
    const int center_grid_index, const float target_margin, const nav_msgs::OccupancyGrid &map)
{
  const int upper_right_grid_index = get_upper_right_grid_index(center_grid_index, target_margin, map);
  const int lower_left_grid_index = get_lower_left_grid_index(center_grid_index, target_margin, map);

  const int width = calc_grid_index_x(upper_right_grid_index, map) - calc_grid_index_x(lower_left_grid_index, map);
  const int height = calc_grid_index_y(upper_right_grid_index, map) - calc_grid_index_y(lower_left_grid_index, map);

  if (width < 0 || height < 0)
  {
    ROS_WARN("width or height is negative");
    return std::vector<int>();
  }

  std::vector<int> rect_grid_index_list;
  for (int i = 0; i < height; i++)
  {
    int index = lower_left_grid_index + map.info.width * i;
    for (int j = 0; j < width; j++)
    {
      rect_grid_index_list.push_back(index);
      index++;
    }
  }

  return rect_grid_index_list;
}

int ObstacleInflater::get_upper_right_grid_index(
    const int center_grid_index, const float target_margin, const nav_msgs::OccupancyGrid &map)
{
  if (center_grid_index < 0 || map.data.size() <= center_grid_index)
  {
    ROS_WARN("center_grid_index is out of range");
    return -1;
  }

  const int target_margin_cell = static_cast<int>(target_margin / map.info.resolution);

  int index = center_grid_index + target_margin_cell * map.info.width;
  if (map.data.size() <= index)
    index = map.info.width * (map.info.height - 1) + calc_grid_index_x(center_grid_index, map);

  const int max_index = calc_max_grid_index_in_same_line(index, map);
  index = std::min(index + target_margin_cell, max_index);

  return index;
}

int ObstacleInflater::get_lower_left_grid_index(
    const int center_grid_index, const float target_margin, const nav_msgs::OccupancyGrid &map)
{
  if (center_grid_index < 0 || map.data.size() <= center_grid_index)
  {
    ROS_WARN("center_grid_index is out of range");
    return -1;
  }

  const int target_margin_cell = static_cast<int>(target_margin / map.info.resolution);

  int index = center_grid_index - target_margin_cell * map.info.width;
  if (index < 0)
    index = calc_grid_index_x(center_grid_index, map);

  const int min_index = calc_min_grid_index_in_same_line(index, map);
  index = std::max(index - target_margin_cell, min_index);

  return index;
}

int ObstacleInflater::calc_grid_index_x(const int index, const nav_msgs::OccupancyGrid &map)
{
  return index % map.info.width;
}

int ObstacleInflater::calc_grid_index_y(const int index, const nav_msgs::OccupancyGrid &map)
{
  return static_cast<int>(index / map.info.width);
}

int ObstacleInflater::calc_max_grid_index_in_same_line(const int index, const nav_msgs::OccupancyGrid &map)
{
  const int index_y = calc_grid_index_y(index, map);
  return (index_y + 1) * map.info.width - 1;
}

int ObstacleInflater::calc_min_grid_index_in_same_line(const int index, const nav_msgs::OccupancyGrid &map)
{
  const int index_y = calc_grid_index_y(index, map);
  return index_y * map.info.width;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "obstacle_inflater");
  ObstacleInflater obstacle_inflater;
  obstacle_inflater.process();

  return 0;
}
