// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Taehun Lim (Darby), Ryan Shim

#ifndef WALL_FOLLOWER_HPP_
#define WALL_FOLLOWER_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define FRONT 0
#define FLEFT 1
#define MLEFT 2
#define LLEFT 3
#define FFEEL 4
#define MFEEL 5
#define LFEEL 6

#define LINEAR_VELOCITY  0.2 // working 0.2
#define ANGULAR_VELOCITY 0.6 // working 0.6

#define NUM_SCANS 360

class WallFollower : public rclcpp::Node
{
public:
  WallFollower();
  ~WallFollower();

private:
  // ROS topic publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // ROS topic subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Variables
  double scan_ranges[7];
  double limits[7];
  int front_min_angle = 0;
  int left_min_angle = 0;

  // double forward_dist_limit = 0.5;
	// double fleft_limit = (sin(DEG2RAD * 30) * forward_dist_limit) / sin(DEG2RAD * 120) - 0.01;
	// double mleft_limit = (sin(DEG2RAD * 30) * forward_dist_limit) / sin(DEG2RAD * 105) - 0.01;
	// double lleft_limit = (sin(DEG2RAD * 30) * forward_dist_limit) / sin(DEG2RAD * 90) - 0.01;

	// double ffeeler_limit = (sin(DEG2RAD * 30) * forward_dist_limit) / sin(DEG2RAD * 145) - 0.01;
	// double mfeeler_limit = (sin(DEG2RAD * 30) * forward_dist_limit) / sin(DEG2RAD * 140) - 0.01;
	// double lfeeler_limit = (sin(DEG2RAD * 30) * forward_dist_limit) / sin(DEG2RAD * 135) - 0.01;

  double side_dist_limit = limits[LLEFT];
	double window_width = 0.1;

  bool debug = true;
  int debug_state = 0;

  // ROS timer
  rclcpp::TimerBase::SharedPtr update_timer_;

  // Function prototypes
  void update_callback();
  void update_cmd_vel(double linear, double angular);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  bool left_close();
  bool left_far();
  //bool right_close();
  bool front_far();
  double min_non_zero(std::vector<float>);
};
#endif  // TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_
