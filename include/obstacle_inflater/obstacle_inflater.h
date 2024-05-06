/**
 * @file obstacle_inflater.h
 * @author Toshiki Nakamura
 * @brief C++ implementation of obstacle inflater
 * @copyright Copyright (c) 2024
 */

#ifndef OBSTACLE_INFLATER_OBSTACLE_INFLATER_H
#define OBSTACLE_INFLATER_OBSTACLE_INFLATER_H

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <vector>

/**
 * @class ObstacleInflater
 * @brief Class for inflating obstacles
 */
class ObstacleInflater
{
public:
  /**
   * @brief Construct a new Obstacle Inflater object
   */
  ObstacleInflater(void);

  /**
   * @brief Process the inflation
   */
  void process(void);

private:
  /**
   * @brief Callback function for map
   *
   * @param msg map message
   */
  void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

  /**
   * @brief Inflate obstacles
   *
   * @param map map message
   */
  void inflate_obstacle(const nav_msgs::OccupancyGrid &map);

  /**
   * @brief Change surrounding grid color
   *
   * @param center_grid_index center grid index
   * @param target_margin target margin
   * @param map map message
   */
  void
  change_surrounding_grid_color(const int center_grid_index, const float target_margin, nav_msgs::OccupancyGrid &map);

  /**
   * @brief Check if the grid is in the margin
   *
   * @param index grid index
   * @param center_grid_index center grid index
   * @param target_margin target margin
   * @param map map message
   * @return true if the grid is in the margin
   * @return false if the grid is not in the margin
   */
  bool in_margin(
      const int index, const int center_grid_index, const float target_margin, const nav_msgs::OccupancyGrid &map);

  /**
   * @brief Calculate distance between two grids
   *
   * @param index1 grid index 1
   * @param index2 grid index 2
   * @param map map
   * @return float distance between two grids
   */
  float calc_dist_between_grid(const int index1, const int index2, const nav_msgs::OccupancyGrid &map);

  /**
   * @brief Get the list of grid index in the rectangle
   *
   * @param center_grid_index center grid index
   * @param target_margin target margin
   * @param map map
   * @return std::vector<int> list of grid index in the rectangle
   */
  std::vector<int>
  get_rect_grid_index_list(const int center_grid_index, const float target_margin, const nav_msgs::OccupancyGrid &map);

  /**
   * @brief Get the upper right grid index
   *
   * @param center_grid_index center grid index
   * @param target_margin target margin
   * @param map map
   * @return int upper right grid index
   */
  int get_upper_right_grid_index(
      const int center_grid_index, const float target_margin, const nav_msgs::OccupancyGrid &map);

  /**
   * @brief Get the lower left grid index object
   *
   * @param center_grid_index center grid index
   * @param target_margin target margin
   * @param map map
   * @return int lower left grid index
   */
  int get_lower_left_grid_index(
      const int center_grid_index, const float target_margin, const nav_msgs::OccupancyGrid &map);

  /**
   * @brief Calculate grid index x
   *
   * @param index grid index
   * @param map map
   * @return int grid index x
   */
  int calc_grid_index_x(const int index, const nav_msgs::OccupancyGrid &map);

  /**
   * @brief Calculate grid index y
   *
   * @param index grid index
   * @param map map
   * @return int grid index y
   */
  int calc_grid_index_y(const int index, const nav_msgs::OccupancyGrid &map);

  /**
   * @brief Calculate max grid index in the same line
   *
   * @param index grid index
   * @param map map
   * @return int max grid index in the same line
   */
  int calc_max_grid_index_in_same_line(const int index, const nav_msgs::OccupancyGrid &map);

  /**
   * @brief Calculate min grid index in the same line
   *
   * @param index grid index
   * @param map map
   * @return int min grid index in the same line
   */
  int calc_min_grid_index_in_same_line(const int index, const nav_msgs::OccupancyGrid &map);

  int hz_;
  float inflation_radius_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher inflated_map_pub_;
  ros::Subscriber raw_map_sub_;

  nav_msgs::OccupancyGrid inflated_map_;
};

#endif  // OBSTACLE_INFLATER_OBSTACLE_INFLATER_H
