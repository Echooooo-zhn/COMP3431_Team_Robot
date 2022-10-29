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
//
// Modified by Claude Sammut for COMP3431
// Use this code as the basis for a wall follower

#include "wall_follower/wall_follower.hpp"

#include <memory>

using namespace std::chrono_literals;

WallFollower::WallFollower()
: Node("wall_follower_node")
{
	/************************************************************
	** Initialise variables
	************************************************************/
	scan_ranges[0] = 0.0;
	scan_ranges[1] = 0.0;
	scan_ranges[2] = 0.0;
	scan_ranges[3] = 0.0;
	scan_ranges[4] = 0.0;

	/************************************************************
	** Initialise ROS publishers and subscribers
	************************************************************/
	auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

	// Initialise publishers
	cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

	// Initialise subscribers
	scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"scan", \
		rclcpp::SensorDataQoS(), \
		std::bind(
			&WallFollower::scan_callback, \
			this, \
			std::placeholders::_1));
	odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom", qos, std::bind(&WallFollower::odom_callback, this, std::placeholders::_1));

	/************************************************************
	** Initialise ROS timers
	************************************************************/
	update_timer_ = this->create_wall_timer(10ms, std::bind(&WallFollower::update_callback, this));

	RCLCPP_INFO(this->get_logger(), "Wall follower node has been initialised");
}

WallFollower::~WallFollower()
{
	RCLCPP_INFO(this->get_logger(), "Wall follower node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
void WallFollower::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	tf2::Quaternion q(
		msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

}

void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	RCLCPP_INFO(this->get_logger(), "Num scans: %d", msg->ranges.size());

	// Setting up scan data 
	// Creating vectors to store a range of scan data
	std::vector<float>::const_iterator it_first = msg->ranges.begin();
	std::vector<float>::const_iterator it_last = msg->ranges.begin() + 30;
	std::vector<float> forward_ranges(it_first, it_last);
	it_first = msg->ranges.begin() + 330;
	it_last = msg->ranges.end();
	std::vector<float> temp_forward_ranges(it_first, it_last);
	forward_ranges.insert(forward_ranges.end(), temp_forward_ranges.begin(), temp_forward_ranges.end());

	it_first = msg->ranges.begin() + 45;
	it_last = msg->ranges.begin() + 85;
	std::vector<float> left_ranges(it_first, it_last);

	it_first = msg->ranges.begin() + 120;
	it_last = msg->ranges.begin() + 240;
	std::vector<float> back_ranges(it_first, it_last);

	it_first = msg->ranges.begin() + 275;
	it_last = msg->ranges.begin() + 315;
	std::vector<float> right_ranges(it_first, it_last);

	scan_ranges[FRONT] = *std::min_element(forward_ranges.begin(), forward_ranges.end());
	scan_ranges[LEFT] = *std::min_element(left_ranges.begin(), left_ranges.end());
	scan_ranges[RIGHT] = *std::min_element(right_ranges.begin(), right_ranges.end());
	scan_ranges[BACK] = *std::min_element(back_ranges.begin(), back_ranges.end());

	for (int i = 0; i < 4; i++)
	{
		if (std::isinf(scan_ranges[i]))
		{
			scan_ranges[i] = msg->range_max;
		}
	}
}

void WallFollower::update_cmd_vel(double linear, double angular)
{
	geometry_msgs::msg::Twist cmd_vel;
	cmd_vel.linear.x = linear;
	cmd_vel.angular.z = angular;

	cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/
void WallFollower::update_callback()
{
	double forward_dist_limit = 0.5; //working 0.45
	double side_dist_limit = 0.24; //working 2
	double window_width = 0.1; // working 0.21 // experimental 0.15
		
	// Measure and Change confidence level

	// Check if there is space in front 
	RCLCPP_INFO(this->get_logger(), "Forward %lf", scan_ranges[FRONT]);
	if (scan_ranges[FRONT] > forward_dist_limit || scan_ranges[FRONT] == 0) {
		if (scan_ranges[LEFT] < side_dist_limit && (scan_ranges[RIGHT] > side_dist_limit || scan_ranges[RIGHT] == 0)) {
			// Too close to left wall, turning right
			RCLCPP_INFO(this->get_logger(), "Too close to left wall");
			RCLCPP_INFO(this->get_logger(), "%lf < %lf", scan_ranges[LEFT], side_dist_limit);
			update_cmd_vel(LINEAR_VELOCITY, -1 * ANGULAR_VELOCITY);
		} else if (scan_ranges[LEFT] > (side_dist_limit + window_width) || scan_ranges[LEFT] == 0) {
			// Too far from left wall, turning left
			RCLCPP_INFO(this->get_logger(), "Too far from left wall");
			RCLCPP_INFO(this->get_logger(), "Left %lf > %lf", scan_ranges[LEFT], forward_dist_limit + window_width);
			auto speed_reduction = side_dist_limit / scan_ranges[LEFT];
			update_cmd_vel(LINEAR_VELOCITY * speed_reduction, ANGULAR_VELOCITY);
		} else if (scan_ranges[RIGHT] < side_dist_limit && (scan_ranges[LEFT] > side_dist_limit || scan_ranges[LEFT] == 0)) {
			// Too close to right wall, turning left
			RCLCPP_INFO(this->get_logger(), "Too close to right wall");
			update_cmd_vel(LINEAR_VELOCITY, ANGULAR_VELOCITY);
			RCLCPP_INFO(this->get_logger(), "Right %lf < %lf", scan_ranges[RIGHT], side_dist_limit);
		} else {
			// If we're not too close to anything, but there is enough space infront, go forward
			RCLCPP_INFO(this->get_logger(), "Moving Forward");
			update_cmd_vel(LINEAR_VELOCITY, 0);
		}
	} else {
		// No space infront
		/*
		if (scan_ranges[LEFT] > (side_dist_limit + window_width) || scan_ranges[LEFT] == 0) {
			// Check if we can turn left
			auto speed_reduction = (scan_ranges[FRONT] - (forward_dist_limit * 0.6)) / forward_dist_limit;
			update_cmd_vel(LINEAR_VELOCITY * speed_reduction, ANGULAR_VELOCITY);
			RCLCPP_INFO(this->get_logger(), "Turning Left: %lf > %lf", scan_ranges[LEFT], (side_dist_limit + window_width));
		} else if (scan_ranges[RIGHT] > side_dist_limit || scan_ranges[RIGHT] == 0) {
			// Check if we can turn right
			auto speed_reduction = (scan_ranges[FRONT] - (forward_dist_limit * 0.6)) / forward_dist_limit;
			update_cmd_vel(LINEAR_VELOCITY * speed_reduction, -1 * ANGULAR_VELOCITY);
			RCLCPP_INFO(this->get_logger(), "Turning Right");
		} else if (scan_ranges[BACK] > side_dist_limit || scan_ranges[BACK] == 0) {
			// Check if we can turn around
			update_cmd_vel(0, -1 * ANGULAR_VELOCITY);
			RCLCPP_INFO(this->get_logger(), "U TURN");
		} else {
			// STUCK
			update_cmd_vel(0, 0);
			RCLCPP_INFO(this->get_logger(), "STUCK");
		}
		*/
		auto speed_reduction = (scan_ranges[FRONT] - (forward_dist_limit * 0.6)) / forward_dist_limit;
		update_cmd_vel(LINEAR_VELOCITY * speed_reduction, -1 * ANGULAR_VELOCITY);
		RCLCPP_INFO(this->get_logger(), "Turning Right");
	}
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WallFollower>());
	rclcpp::shutdown();

	return 0;
}
