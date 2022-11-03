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
	// We split the left ranges into 3 sections for a more accuracy
	// For example if we find the minimum of the data scanned at 45 to 85 degrees,
	// and since the bot is a square we know that the distance from the edge of the
	// turtlebot to the wall at 85 degrees is shorter than 45 degrees. So in the
	// case that the minimum value in the range is at 45 degrees the bot will closer 
	// than the intended side limit before it turns. This is simply fixed by splitting
	// the ranges in as many parts such that the difference between the start and end
	// degrees is negligible. For maximum effectiveness each degree would have their own
	// side limit however this is not necessary as only a few splits would be more than
	// enough for the difference to become negligible.
	it_first = left_ranges.begin();
	it_last = left_ranges.begin() + 13;
	std::vector<float> fleft_ranges(it_first, it_last);
	it_first = left_ranges.begin() + 14;
	it_last = left_ranges.begin() + 27;
	std::vector<float> mleft_ranges(it_first, it_last);
	it_first = left_ranges.begin() + 28;
	it_last = left_ranges.end();
	std::vector<float> lleft_ranges(it_first, it_last);

	// Dont really need these ranges so removing them to save some time
	// it_first = msg->ranges.begin() + 120;
	// it_last = msg->ranges.begin() + 240;
	// std::vector<float> back_ranges(it_first, it_last);

	// it_first = msg->ranges.begin() + 275;
	// it_last = msg->ranges.begin() + 315;
	// std::vector<float> right_ranges(it_first, it_last);

	scan_ranges[FRONT] = min_non_zero(forward_ranges);
	scan_ranges[FLEFT] = min_non_zero(fleft_ranges);
	scan_ranges[MLEFT] = min_non_zero(mleft_ranges);
	scan_ranges[LLEFT] = min_non_zero(lleft_ranges);
	// scan_ranges[RIGHT] = min_non_zero(right_ranges);
	// scan_ranges[BACK] = min_non_zero(back_ranges);

	for (int i = 0; i < 6; i++)
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

	if (cmd_vel.angular.z > 1){
		RCLCPP_INFO(this->get_logger(), "%lf", cmd_vel.angular.z);
	}

	cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/
void WallFollower::update_callback()
{
	// Check if there is space in front 
	if (front_far()) {
		if (left_close()) {
			// Too close to left wall, turning right
			if (debug) {
				double min = 5.0;
				for (int i = FLEFT; i <= LLEFT; i++) {
					if (scan_ranges[i] < min) {
						min = scan_ranges[i];
					}
				}
				double limit = 0.0;
				if (min == scan_ranges[FLEFT]) {
					limit = fleft_limit;
				} else if (min == scan_ranges[MLEFT]) {
					limit = mleft_limit;
				} else {
					limit = lleft_limit;
				}
				RCLCPP_INFO(this->get_logger(), "Too close to the left wall:  %lf < %lf", min, limit);
				debug_state = 1;
			}
			update_cmd_vel(LINEAR_VELOCITY, -1 * ANGULAR_VELOCITY);
		} else if (left_far()) {
			// Too far from left wall, turning left
			if (debug) {
				double min = 5.0;
				for (int i = FLEFT; i <= LLEFT; i++) {
					if (scan_ranges[i] < min) {
						min = scan_ranges[i];
					}
				}
				double limit = 0.0;
				if (min == scan_ranges[FLEFT]) {
					limit = fleft_limit;
				} else if (min == scan_ranges[MLEFT]) {
					limit = mleft_limit;
				} else {
					limit = lleft_limit;
				}
				RCLCPP_INFO(this->get_logger(), "Too far from left wall:  %lf > %lf", min, limit + window_width);
				debug_state = 2;
			}
			update_cmd_vel(0.5 * LINEAR_VELOCITY, ANGULAR_VELOCITY);
		// } else if (right_close() && !left_close()) {
		// 	// Too close to right wall, turning left
		// 	if (debug) {
		// 		RCLCPP_INFO(this->get_logger(), "Too close to the right wall:  %lf < %lf", scan_ranges[RIGHT], side_dist_limit);
		// 		debug_state = 3;
		// 	}
		// 	update_cmd_vel(LINEAR_VELOCITY, ANGULAR_VELOCITY);
		} else {
			// If we're not too close to anything, but there is enough space infront, go forward
			if (debug) {
				RCLCPP_INFO(this->get_logger(), "Moving Forward:  %lf", scan_ranges[FRONT]);
				debug_state = 4;
			}
			update_cmd_vel(LINEAR_VELOCITY, 0);
		}
	} else {
		// No space infront
		if (debug) {
			RCLCPP_INFO(this->get_logger(), "Forward blocked, turning right:  %lf < %lf", scan_ranges[FRONT], forward_dist_limit);
			debug_state = 5;
		}
		/*
		// This code currently doesnt work for this implementation more logic is needed.

		if (scan_ranges[LEFT] > (side_dist_limit + window_width) || scan_ranges[LEFT] == 0) {
			// Check if we can turn left
			auto speed_reduction = (scan_ranges[FRONT] - (forward_dist_limit * 0.6)) / forward_dist_limit;
			update_cmd_vel(LINEAR_VELOCITY * speed_reduction, ANGULAR_VELOCITY);
			RCLCPP_INFO(this->get_logger(), "\33[2KTurning Left: %lf > %lf", scan_ranges[LEFT], (side_dist_limit + window_width));
		} else if (scan_ranges[RIGHT] > side_dist_limit || scan_ranges[RIGHT] == 0) {
			// Check if we can turn right
			auto speed_reduction = (scan_ranges[FRONT] - (forward_dist_limit * 0.6)) / forward_dist_limit;
			update_cmd_vel(LINEAR_VELOCITY * speed_reduction, -1 * ANGULAR_VELOCITY);
			RCLCPP_INFO(this->get_logger(), "\33[2KTurning Right");
		} else if (scan_ranges[BACK] > side_dist_limit || scan_ranges[BACK] == 0) {
			// Check if we can turn around
			update_cmd_vel(0, -1 * ANGULAR_VELOCITY);
			RCLCPP_INFO(this->get_logger(), "\33[2KU TURN");
		} else {
			// STUCK
			update_cmd_vel(0, 0);
			RCLCPP_INFO(this->get_logger(), "\33[2KSTUCK");
		}
		*/
		// Turn right whilst slowing linear speed based on formula below
		
		auto speed_reduction = (scan_ranges[FRONT] - (forward_dist_limit * 0.5)) / forward_dist_limit;
		if (speed_reduction < 0.0) {speed_reduction = 0.0;}
		update_cmd_vel(LINEAR_VELOCITY * speed_reduction, -1 * ANGULAR_VELOCITY);
	}
}

bool WallFollower::left_close() {
	//return scan_ranges[FLEFT] < fleft_limit;
	return scan_ranges[FLEFT] < fleft_limit ||
	 	   scan_ranges[MLEFT] < mleft_limit ||
		   scan_ranges[LLEFT] < lleft_limit;
}

bool WallFollower::left_far() {
	//return scan_ranges[FLEFT] > fleft_limit + window_width;
	return scan_ranges[FLEFT] >= fleft_limit + window_width ||
		   scan_ranges[MLEFT] >= mleft_limit + window_width ||
		   scan_ranges[LLEFT] >= lleft_limit + window_width;
}

bool WallFollower::right_close() {
	return scan_ranges[RIGHT] < side_dist_limit;
}

bool WallFollower::front_far() {
	return scan_ranges[FRONT] > forward_dist_limit;
}

double WallFollower::min_non_zero(std::vector<float> v) {
	double min = 3.5;
	for (auto item : v) {
		if (item != 0.0 && item < min) {
			min = item;
		}
	}
	return min;
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
