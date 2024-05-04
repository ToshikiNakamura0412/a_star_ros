/**
 * @file a_star.cpp
 * @author Toshiki Nakamura
 * @brief C++ implementation of A* algorithm
 * @date 2024-05-04
 * @copyright Copyright (c) 2024
 */

#include <string>
#include <vector>

#include "a_star/a_star.h"

AStarPlanner::AStarPlanner() : private_nh_("~")
{
  private_nh_.param<int>("hz", hz_, 2);
  private_nh_.param<std::string>("global_frame", global_frame_, std::string("map"));
  private_nh_.param<float>("weight_of_heuristic", weight_of_heuristic_, 1.0);
  private_nh_.param<bool>("debug_mode", debug_mode_, false);
  private_nh_.param<float>("sleep_time", sleep_time_, 0.01);

  path_pub_ = nh_.advertise<nav_msgs::Path>("path", 1);
  map_sub_ = nh_.subscribe("/map", 1, &AStarPlanner::map_callback, this);
  initial_pose_sub_ = nh_.subscribe("/initialpose", 1, &AStarPlanner::initial_pose_callback, this);
  goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &AStarPlanner::goal_callback, this);

  if (debug_mode_)
  {
    current_node_pub_ = nh_.advertise<geometry_msgs::PointStamped>("current_node", 1);
    open_set_pub_ = nh_.advertise<geometry_msgs::PoseArray>("open_set", 1);
    closed_set_pub_ = nh_.advertise<geometry_msgs::PoseArray>("close_set", 1);
    neighbor_nodes_pub_ = nh_.advertise<geometry_msgs::PoseArray>("neighbor_nodes", 1);
  }

  ROS_INFO_STREAM(ros::this_node::getName() << "node has started..");
  ROS_INFO_STREAM("hz: " << hz_);
  ROS_INFO_STREAM("global_frame: " << global_frame_);
  ROS_INFO_STREAM("weight_of_heuristic: " << weight_of_heuristic_);
  ROS_INFO_STREAM("debug_mode: " << debug_mode_);
  ROS_INFO_STREAM("sleep_time: " << sleep_time_);
}

void AStarPlanner::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg) { map_ = *msg; }

void AStarPlanner::initial_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  initial_pose_ = msg->pose.pose;
}

void AStarPlanner::goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg) { goal_ = msg->pose; }

void AStarPlanner::process()
{
  ros::Rate loop_rate(hz_);
  std::optional<nav_msgs::Path> path = std::nullopt;
  while (ros::ok())
  {
    ros::spinOnce();

    if (map_.has_value() && initial_pose_.has_value() && goal_.has_value())
    {
      Node start_node = pose2node(initial_pose_.value(), map_.value());
      Node goal_node = pose2node(goal_.value(), map_.value());
      path = planning(start_node, goal_node, map_.value(), weight_of_heuristic_);
      add_direction_to_path(goal_.value(), path.value());
      initial_pose_.reset();
    }
    if (path.has_value())
    {
      path.value().header.stamp = ros::Time::now();
      path.value().header.frame_id = global_frame_;
      path_pub_.publish(path.value());
    }

    loop_rate.sleep();
  }
}

Node AStarPlanner::pose2node(const geometry_msgs::Pose &pose, const nav_msgs::OccupancyGrid &map)
{
  Node node;
  node.index_x = static_cast<int>((pose.position.x - map.info.origin.position.x) / map.info.resolution);
  node.index_y = static_cast<int>((pose.position.y - map.info.origin.position.y) / map.info.resolution);
  return node;
}

nav_msgs::Path
AStarPlanner::planning(Node start_node, const Node goal_node, const nav_msgs::OccupancyGrid &map, const float w)
{
  std::vector<Node> open_set;
  std::vector<Node> closed_set;

  start_node.cost = heuristic(start_node, goal_node, w);
  open_set.push_back(start_node);

  Node current_node = start_node;
  while (!is_same_node(current_node, goal_node))
  {
    if (open_set.size() == 0)
    {
      ROS_ERROR_STREAM("Open set is empty..");
      exit(EXIT_FAILURE);
    }
    move_node_between_sets(current_node, open_set, closed_set);
    update_sets(current_node, goal_node, w, map, open_set, closed_set);
    current_node = get_lowest_cost_node(open_set);

    show_node_point(current_node);
    show_node_set(open_set, open_set_pub_);
    show_node_set(closed_set, closed_set_pub_);
  }

  return create_path(current_node, start_node, closed_set, map);
}

float AStarPlanner::heuristic(const Node &node, const Node &goal_node, const float w)
{
  const float dx = static_cast<float>(node.index_x - goal_node.index_x);
  const float dy = static_cast<float>(node.index_y - goal_node.index_y);
  return w * hypot(dx, dy);
}

bool AStarPlanner::is_same_node(const Node node1, const Node node2)
{
  if (node1.index_x == node2.index_x && node1.index_y == node2.index_y)
    return true;
  else
    return false;
}

void AStarPlanner::move_node_between_sets(const Node node, std::vector<Node> &set1, std::vector<Node> &set2)
{
  for (int i = 0; i < set1.size(); i++)
  {
    if (is_same_node(node, set1[i]))
    {
      set1.erase(set1.begin() + i);
      set2.push_back(node);
      return;
    }
  }
  ROS_ERROR_STREAM("The same node is not found..");
  exit(EXIT_FAILURE);
}

void AStarPlanner::update_sets(
    const Node current_node, const Node goal_node, const float w, const nav_msgs::OccupancyGrid &map,
    std::vector<Node> &open_set, std::vector<Node> &closed_set)
{
  std::vector<Node> neighbor_nodes = create_neighbor_nodes(current_node, goal_node, w);
  show_node_set(neighbor_nodes, neighbor_nodes_pub_);

  for (const auto &neighbor_node : neighbor_nodes)
  {
    if (is_obs(neighbor_node, map))
      continue;

    if (in_set(neighbor_node, open_set))
    {
      const int node_index = search_node_index_from_set(neighbor_node, open_set);
      if (neighbor_node.cost < open_set[node_index].cost)
      {
        open_set[node_index].cost = neighbor_node.cost;
        open_set[node_index].parent_index_x = neighbor_node.parent_index_x;
        open_set[node_index].parent_index_y = neighbor_node.parent_index_y;
      }
    }
    else if (in_set(neighbor_node, closed_set))
    {
      const int node_index = search_node_index_from_set(neighbor_node, closed_set);
      if (neighbor_node.cost < closed_set[node_index].cost)
      {
        closed_set.erase(closed_set.begin() + node_index);
        open_set.push_back(neighbor_node);
      }
    }
    else
    {
      open_set.push_back(neighbor_node);
    }
  }
}

std::vector<Node> AStarPlanner::create_neighbor_nodes(const Node current_node, const Node goal_node, const float w)
{
  std::vector<Motion> motion_set = create_motion_model();

  std::vector<Node> neighbor_nodes;
  for (int i = 0; i < motion_set.size(); i++)
  {
    Node neighbor_node = get_neighbor_node(current_node, motion_set[i], goal_node, w);
    neighbor_nodes.push_back(neighbor_node);
  }

  return neighbor_nodes;
}

Node AStarPlanner::get_neighbor_node(const Node current_node, const Motion motion, const Node goal_node, const float w)
{
  Node neighbor_node;
  neighbor_node.index_x = current_node.index_x + motion.dx;
  neighbor_node.index_y = current_node.index_y + motion.dy;
  neighbor_node.parent_index_x = current_node.index_x;
  neighbor_node.parent_index_y = current_node.index_y;
  neighbor_node.cost = (current_node.cost - heuristic(current_node, goal_node, w)) +
                       heuristic(neighbor_node, goal_node, w) + motion.cost;

  return neighbor_node;
}

std::vector<Motion> AStarPlanner::create_motion_model()
{
  std::vector<Motion> motion_set;
  motion_set.push_back(get_motion(1, 0, 1));
  motion_set.push_back(get_motion(0, 1, 1));
  motion_set.push_back(get_motion(-1, 0, 1));
  motion_set.push_back(get_motion(0, -1, 1));
  motion_set.push_back(get_motion(-1, -1, sqrt(2)));
  motion_set.push_back(get_motion(-1, 1, sqrt(2)));
  motion_set.push_back(get_motion(1, -1, sqrt(2)));
  motion_set.push_back(get_motion(1, 1, sqrt(2)));

  return motion_set;
}

Motion AStarPlanner::get_motion(const int dx, const int dy, const float cost)
{
  if (1 < abs(dx) || 1 < abs(dy))
  {
    ROS_ERROR_STREAM("The motion is inappropriate..");
    exit(EXIT_FAILURE);
  }

  Motion motion;
  motion.dx = dx;
  motion.dy = dy;
  motion.cost = cost;

  return motion;
}

bool AStarPlanner::is_obs(const Node node, const nav_msgs::OccupancyGrid &map)
{
  const int grid_index = node.index_x + (node.index_y * map.info.width);
  if (grid_index < 0 || map.data.size() <= grid_index)
  {
    ROS_ERROR_STREAM("The grid index is out of range..");
    return true;
  }
  return map.data[grid_index] == 100;
}

bool AStarPlanner::in_set(const Node node, const std::vector<Node> &set)
{
  for (const auto &n : set)
  {
    if (is_same_node(node, n))
      return true;
  }

  return false;
}

int AStarPlanner::search_node_index_from_set(const Node node, std::vector<Node> &set)
{
  for (int i = 0; i < set.size(); i++)
  {
    if (is_same_node(node, set[i]))
      return i;
  }

  ROS_ERROR_STREAM("The node is not found..");
  exit(EXIT_FAILURE);
}

Node AStarPlanner::get_lowest_cost_node(const std::vector<Node> &set)
{
  Node lowest_cost_node = set.front();

  for (const auto &node : set)
  {
    if (node.cost < lowest_cost_node.cost)
      lowest_cost_node = node;
  }

  return lowest_cost_node;
}

nav_msgs::Path AStarPlanner::create_path(
    Node current_node, const Node start_node, const std::vector<Node> &closed_set, const nav_msgs::OccupancyGrid &map)
{
  nav_msgs::Path path;
  path.poses.push_back(calc_pose(current_node, map));
  while (!is_same_node(current_node, start_node))
  {
    show_node_point(current_node);
    current_node = get_parent_node(current_node, closed_set);
    path.poses.push_back(calc_pose(current_node, map));
  }

  reverse(path.poses.begin(), path.poses.end());
  return path;
}

geometry_msgs::PoseStamped AStarPlanner::calc_pose(const Node node, const nav_msgs::OccupancyGrid &map)
{
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = node.index_x * map.info.resolution + map.info.origin.position.x;
  pose.pose.position.y = node.index_y * map.info.resolution + map.info.origin.position.y;
  return pose;
}

Node AStarPlanner::get_parent_node(const Node node, const std::vector<Node> &closed_set)
{
  for (const auto &closed_node : closed_set)
  {
    if (node.parent_index_x == closed_node.index_x && node.parent_index_y == closed_node.index_y)
      return closed_node;
  }
  ROS_ERROR_STREAM("The parent node is not found..");
  exit(EXIT_FAILURE);
}

void AStarPlanner::add_direction_to_path(const geometry_msgs::Pose &goal, nav_msgs::Path &path)
{
  for (int i = 0; i < path.poses.size() - 1; i++)
    path.poses[i].pose.orientation = calc_direction(path.poses[i].pose.position, path.poses[i + 1].pose.position);
  path.poses.back().pose.orientation = goal.orientation;
}

geometry_msgs::Quaternion
AStarPlanner::calc_direction(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2)
{
  const float yaw = atan2(point2.y - point1.y, point2.x - point1.x);
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);

  geometry_msgs::Quaternion q_msg;
  q_msg.x = q.x();
  q_msg.y = q.y();
  q_msg.z = q.z();
  q_msg.w = q.w();

  return q_msg;
}

// for debug
void AStarPlanner::show_node_point(const Node node)
{
  if (debug_mode_)
  {
    geometry_msgs::PointStamped current_node_;
    current_node_.header.stamp = ros::Time::now();
    current_node_.header.frame_id = global_frame_;
    current_node_.point.x = node.index_x * map_.value().info.resolution + map_.value().info.origin.position.x;
    current_node_.point.y = node.index_y * map_.value().info.resolution + map_.value().info.origin.position.y;
    current_node_pub_.publish(current_node_);
    ros::Duration(sleep_time_).sleep();
  }
}

// for debug
void AStarPlanner::show_node_set(const std::vector<Node> &set, const ros::Publisher &pub)
{
  if (debug_mode_)
  {
    geometry_msgs::PoseArray pose_array;
    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = global_frame_;
    for (const auto &node : set)
    {
      geometry_msgs::Pose pose;
      pose.position.x = node.index_x * map_.value().info.resolution + map_.value().info.origin.position.x;
      pose.position.y = node.index_y * map_.value().info.resolution + map_.value().info.origin.position.y;
      pose_array.poses.push_back(pose);
    }
    pub.publish(pose_array);
    ros::Duration(sleep_time_).sleep();
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "obstacle_inflater");
  AStarPlanner a_star_planner;
  a_star_planner.process();

  return 0;
}
