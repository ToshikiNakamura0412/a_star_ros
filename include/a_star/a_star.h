/**
 * @file a_star.h
 * @author Toshiki Nakamura
 * @brief C++ implementation of A* algorithm
 * @copyright Copyright (c) 2024
 */

#ifndef A_STAR_A_STAR_H
#define A_STAR_A_STAR_H

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <optional>
#include <ros/ros.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

/**
 * @struct Node
 * @brief Node Struct
 */
struct Node
{
  int index_x = 0;          // [cell]
  int index_y = 0;          // [cell]
  int parent_index_x = -1;  // [cell]
  int parent_index_y = -1;  // [cell]
  float cost = 0.0;         // f value ( = g + h )
};

/**
 * @struct Motion
 * @brief Motion Struct
 */
struct Motion
{
  int dx;  // [cell]
  int dy;  // [cell]
  float cost;
};

/**
 * @class AStarPlanner
 * @brief A* Planner Class
 */
class AStarPlanner
{
public:
  /**
   * @brief Construct a new A Star Planner object
   */
  AStarPlanner();

  /**
   * @brief Process A* search algorithm
   */
  void process();

private:
  /**
   * @brief Callback function for map
   *
   * @param msg map message
   */
  void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

  /**
   * @brief Callback function for initial pose
   *
   * @param msg initial pose message
   */
  void initial_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

  /**
   * @brief Callback function for goal
   *
   * @param msg goal pose message
   */
  void goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);

  /**
   * @brief Calculate node
   *
   * @param pose pose
   * @param map map
   * @return Node node
   */
  Node pose2node(const geometry_msgs::Pose &pose, const nav_msgs::OccupancyGrid &map);

  /**
   * @brief Check if the node is obstacle
   *
   * @param node node
   * @param map map
   * @return true if the node is obstacle
   * @return false if the node is not obstacle
   */
  bool is_obs(const Node node, const nav_msgs::OccupancyGrid &map);

  /**
   * @brief Check if the node is unknown cell
   *
   * @param node node
   * @param map map
   * @return true if the node is unknown cell
   * @return false if the node is not unknown cell
   */
  bool is_unknown_cell(const Node node, const nav_msgs::OccupancyGrid &map);
  /**
   * @brief execute A* algorithm
   *
   * @param start_node start node
   * @param goal_node goal node
   * @param map map
   * @param w weight of heuristic
   * @return nav_msgs::Path path
   */
  nav_msgs::Path planning(Node start_node, const Node goal_node, const nav_msgs::OccupancyGrid &map, const float w);

  /**
   * @brief Calculate heuristic value
   *
   * @param node node
   * @param goal_node goal node
   * @param w weight of heuristic
   * @return float heuristic value
   */
  float heuristic(const Node &node, const Node &goal_node, const float w);

  /**
   * @brief Check if the node is the same node
   *
   * @param node1 node1
   * @param node2 node2
   * @return true if the node is the same node
   * @return false if the node is not the same node
   */
  bool is_same_node(const Node node1, const Node node2);

  /**
   * @brief Move node between sets
   *
   * @param node node
   * @param set1 set1
   * @param set2 set2
   */
  void move_node_between_sets(const Node node, std::vector<Node> &set1, std::vector<Node> &set2);

  /**
   * @brief Update open set and closed set
   *
   * @param current_node current node
   * @param goal_node goal node
   * @param w weight of heuristic
   * @param map map
   * @param open_set open set
   * @param closed_set closed set
   */
  void update_sets(
      const Node current_node, const Node goal_node, const float w, const nav_msgs::OccupancyGrid &map,
      std::vector<Node> &open_set, std::vector<Node> &closed_set);

  /**
   * @brief Create neighbor nodes
   *
   * @param current_node current node
   * @param goal_node goal node
   * @param w weight of heuristic
   * @return std::vector<Node> neighbor nodes
   */
  std::vector<Node> create_neighbor_nodes(const Node current_node, const Node goal_node, const float w);

  /**
   * @brief Get neighbor node
   *
   * @param current_node current node
   * @param motion motion
   * @param goal_node goal node
   * @param w weight of heuristic
   * @return Node neighbor node
   */
  Node get_neighbor_node(const Node current_node, const Motion motion, const Node goal_node, const float w);

  /**
   * @brief Create motion model
   *
   * @return std::vector<Motion> motion model
   */
  std::vector<Motion> create_motion_model();

  /**
   * @brief Get motion
   *
   * @param dx dx
   * @param dy dy
   * @param cost cost
   * @return Motion motion
   */
  Motion get_motion(const int dx, const int dy, const float cost);

  /**
   * @brief Check if the node is in the set
   *
   * @param node node
   * @param set set of nodes
   * @return true if the node is in the set
   * @return false if the node is not in the set
   */
  bool in_set(const Node node, const std::vector<Node> &set);

  /**
   * @brief Search node index from set
   *
   * @param node node
   * @param set set of nodes
   * @return int node index
   */
  int search_node_index_from_set(const Node node, std::vector<Node> &set);

  /**
   * @brief Get the lowest cost node object
   *
   * @param set set of nodes
   * @return Node lowest cost node
   */
  Node get_lowest_cost_node(const std::vector<Node> &set);

  /**
   * @brief Create path
   *
   * @param current_node current node
   * @param start_node start node
   * @param closed_set closed set
   * @param map map
   * @return nav_msgs::Path path
   */
  nav_msgs::Path create_path(
      Node current_node, const Node start_node, const std::vector<Node> &closed_set,
      const nav_msgs::OccupancyGrid &map);

  /**
   * @brief Calculate pose
   *
   * @param node node
   * @param map map
   * @return geometry_msgs::PoseStamped pose
   */
  geometry_msgs::PoseStamped calc_pose(const Node node, const nav_msgs::OccupancyGrid &map);

  /**
   * @brief Get parent node
   *
   * @param node node
   * @param closed_set closed set
   * @return Node parent node
   */
  Node get_parent_node(const Node node, const std::vector<Node> &closed_set);

  /**
   * @brief Add direction to path
   *
   * @param goal goal pose
   * @param path path
   */
  void add_direction_to_path(const geometry_msgs::Pose &goal, nav_msgs::Path &path);

  /**
   * @brief Calculate direction
   *
   * @param point1 point1
   * @param point2 point2
   * @return geometry_msgs::Quaternion direction
   */
  geometry_msgs::Quaternion calc_direction(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2);

  /**
   * @brief Show node point
   *
   * @param node node
   */
  void show_node_point(const Node node);

  /**
   * @brief Show node set
   *
   * @param set set of nodes
   * @param pub publisher
   */
  void show_node_set(const std::vector<Node> &set, const ros::Publisher &pub);

  int hz_;
  std::string global_frame_;
  float weight_of_heuristic_;
  bool debug_mode_;
  float sleep_time_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher closed_set_pub_;
  ros::Publisher current_node_pub_;
  ros::Publisher neighbor_nodes_pub_;
  ros::Publisher open_set_pub_;
  ros::Publisher path_pub_;
  ros::Subscriber map_sub_;
  ros::Subscriber initial_pose_sub_;
  ros::Subscriber goal_sub_;

  std::optional<nav_msgs::OccupancyGrid> map_;
  std::optional<geometry_msgs::Pose> initial_pose_;
  std::optional<geometry_msgs::Pose> goal_;
};

#endif  // A_STAR_A_STAR_H
