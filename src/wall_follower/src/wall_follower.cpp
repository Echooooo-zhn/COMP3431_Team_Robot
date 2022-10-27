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
	scan_data_[0] = 0.0;
	scan_data_[1] = 0.0;
	scan_data_[2] = 0.0;

	confidence = START_LEVEL;

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
	uint16_t scan_angle[4] = {0, 90, 45, 270};

	RCLCPP_INFO(this->get_logger(), "Num scans: %d", sizeof(msg->ranges));

	for (int num = 0; num < 3; num++)
	{
		if (std::isinf(msg->ranges.at(scan_angle[num])))
		{
			scan_data_[num] = msg->range_max;
		}
		else
		{
			scan_data_[num] = msg->ranges.at(scan_angle[num]);
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
	double check_forward_dist = 0.45; //working 0.45

	double check_side_dist = 0.2; //working 2
	double window_width = 0.21; // working 0.21 // experimental 0.15
		
	// Measure and Change confidence level

	if (confidence > MAX_LEVEL) confidence = MAX_LEVEL;
	if (confidence < 0) confidence = 0;
	RCLCPP_INFO(this->get_logger(), "Confidence: %d", confidence);

	// Check if there is space in front 
	if (scan_data_[CENTER] > check_forward_dist ||  scan_data_[CENTER] == 0.0)
	{
		if (scan_data_[OFF_LEFT] < check_side_dist + 0.1 || scan_data_[OFF_LEFT] <= (check_side_dist - 0.3))
		{
			// Too close to left wall, turning right
			RCLCPP_INFO(this->get_logger(), "Too close to left wall");
			if (confidence < RIGHT_TURN_LEVEL) confidence = RIGHT_TURN_LEVEL;
			confidence++;
		}
		else if (scan_data_[OFF_LEFT] <= (check_side_dist + 0.3))
		{
			// Sees left wall, add to confidence
			confidence += 10;
			if (confidence > RIGHT_TURN_LEVEL) confidence = RIGHT_TURN_LEVEL;
			RCLCPP_INFO(this->get_logger(), "Seeing left 45deg wall");
			
		}
		else if (scan_data_[LEFT] > (check_side_dist + window_width))
		{

			// Move forward a certain distance
			// turn left
			// Too far from left wall, turning left
			confidence--;
			if (confidence > RIGHT_TURN_LEVEL) confidence = RIGHT_TURN_LEVEL;
			RCLCPP_INFO(this->get_logger(), "Too far from left wall");
		} 
		/*
		else if (scan_data_[RIGHT] < check_side_dist)
		{
			// Too close to right wall, turning left
			RCLCPP_INFO(this->get_logger(), "Too close to right wall");
		}*/
		else
		{
			// If we're not too close to anything, but there is enough space infront, go forward
			confidence++;
			if (confidence > RIGHT_TURN_LEVEL) confidence = RIGHT_TURN_LEVEL;
			RCLCPP_INFO(this->get_logger(), "within window");
		}
		
	} else {
		// if (scan_data_[CENTER] < check_forward_dist && scan_data_[CENTER] != 0.0)
		// If there is something in front, turn right
		if (confidence < RIGHT_TURN_LEVEL) confidence = RIGHT_TURN_LEVEL;
		confidence++;
		RCLCPP_INFO(this->get_logger(), "U TURN");
	}

	if (confidence > RIGHT_TURN_LEVEL) {
		// RIGHT
		update_cmd_vel(0.02, -1* ANGULAR_VELOCITY-0.1);
	} else if (confidence > LEFT_TURN_LEVEL) {
		// FORWARD
		update_cmd_vel(LINEAR_VELOCITY, 0.0);
	} else if (confidence <= LEFT_TURN_LEVEL) {
		// LEFT
		update_cmd_vel(0.0, ANGULAR_VELOCITY);
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
