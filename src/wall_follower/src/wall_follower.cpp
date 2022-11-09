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
	scan_ranges[FRONT] = 0.0;
	scan_ranges[FLEFT] = 0.0;
	scan_ranges[MLEFT] = 0.0;
	scan_ranges[LLEFT] = 0.0;
	scan_ranges[FFEEL] = 0.0;
	scan_ranges[MFEEL] = 0.0;
	scan_ranges[LFEEL] = 0.0;

	limits[FRONT] = 0.5;
	limits[FLEFT] = (sin(DEG2RAD * 30) * limits[FRONT]) / sin(DEG2RAD * 120) - 0.01;
	limits[MLEFT] = (sin(DEG2RAD * 30) * limits[FRONT]) / sin(DEG2RAD * 105) - 0.01;
	limits[LLEFT] = (sin(DEG2RAD * 30) * limits[FRONT]) / sin(DEG2RAD * 90) - 0.01;
	limits[FFEEL] = (sin(DEG2RAD * 30) * limits[FRONT]) / sin(DEG2RAD * 145) - 0.01;
	limits[MFEEL] = (sin(DEG2RAD * 30) * limits[FRONT]) / sin(DEG2RAD * 140) - 0.01;
	limits[LFEEL] = (sin(DEG2RAD * 30) * limits[FRONT]) / sin(DEG2RAD * 135) - 0.01;

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

	// Creating front range
	std::vector<float>::const_iterator it_first = msg->ranges.begin();
	std::vector<float>::const_iterator it_last = msg->ranges.begin() + 30;
	std::vector<float> forward_ranges(it_first, it_last);
	it_first = msg->ranges.begin() + 330;
	it_last = msg->ranges.end();
	std::vector<float> temp_forward_ranges(it_first, it_last);
	forward_ranges.insert(forward_ranges.end(), temp_forward_ranges.begin(), temp_forward_ranges.end());

	// Creating left ranges
	it_first = msg->ranges.begin() + 45;
	it_last = msg->ranges.begin() + 90;
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
	it_last = left_ranges.begin() + 15;
	std::vector<float> fleft_ranges(it_first, it_last);
	it_first = left_ranges.begin() + 16;
	it_last = left_ranges.begin() + 30;
	std::vector<float> mleft_ranges(it_first, it_last);
	it_first = left_ranges.begin() + 31;
	it_last = left_ranges.end();
	std::vector<float> lleft_ranges(it_first, it_last);

	// Creating feeler ranges
	// Making ranges from 30 - 45 degrees splitting into 3 because the limit difference is greatest per degree at these ranges
	it_first = msg->ranges.begin() + 31;
	it_last = msg->ranges.begin() + 35;
	std::vector<float> ffeeler_ranges(it_first, it_last);
	it_first = msg->ranges.begin() + 36;
	it_last = msg->ranges.begin() + 40;
	std::vector<float> mfeeler_ranges(it_first, it_last);
	it_first = msg->ranges.begin() + 41;
	it_last = msg->ranges.begin() + 45;
	std::vector<float> lfeeler_ranges(it_first, it_last);

	// Storing the min scan value of each range in an array
	scan_ranges[FRONT] = min_non_zero(forward_ranges);
	scan_ranges[FLEFT] = min_non_zero(fleft_ranges);
	scan_ranges[MLEFT] = min_non_zero(mleft_ranges);
	scan_ranges[LLEFT] = min_non_zero(lleft_ranges);
	scan_ranges[FFEEL] = min_non_zero(ffeeler_ranges);
	scan_ranges[MFEEL] = min_non_zero(mfeeler_ranges);
	scan_ranges[LFEEL] = min_non_zero(lfeeler_ranges);

	// Value bounding
	for (int i = 0; i < 7; i++)
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
				RCLCPP_INFO(this->get_logger(), "Too close to the left wall");
				RCLCPP_INFO(this->get_logger(), "FLEFT: %lf < %lf", scan_ranges[FLEFT], limits[FLEFT]);
				RCLCPP_INFO(this->get_logger(), "MLEFT: %lf < %lf", scan_ranges[MLEFT], limits[MLEFT]);
				RCLCPP_INFO(this->get_logger(), "LLEFT: %lf < %lf\n", scan_ranges[LLEFT], limits[LLEFT]);
			}
			update_cmd_vel(LINEAR_VELOCITY, -1 * ANGULAR_VELOCITY);
		} else if (left_far()) {
			// Too far from left wall, turning left
			if (debug) {
				RCLCPP_INFO(this->get_logger(), "Too far from left wall");
				RCLCPP_INFO(this->get_logger(), "FLEFT: %lf > %lf", scan_ranges[FLEFT], limits[FLEFT] + window_width);
				RCLCPP_INFO(this->get_logger(), "MLEFT: %lf > %lf", scan_ranges[MLEFT], limits[MLEFT] + window_width);
				RCLCPP_INFO(this->get_logger(), "LLEFT: %lf > %lf\n", scan_ranges[LLEFT], limits[LLEFT] + window_width);
			}
			// The linear velocity modifier may need some adjusting, seems to work at half speed.
			update_cmd_vel(LINEAR_VELOCITY * 0.5, ANGULAR_VELOCITY);
		} else {
			// If we're not too close to anything, but there is enough space infront, go forward
			if (debug) {
				RCLCPP_INFO(this->get_logger(), "Moving Forward");
				RCLCPP_INFO(this->get_logger(), "FRONT: %lf > %lf", scan_ranges[FRONT], limits[FRONT]);
				RCLCPP_INFO(this->get_logger(), "FFEEL: %lf > %lf", scan_ranges[FFEEL], limits[FFEEL]);
				RCLCPP_INFO(this->get_logger(), "MFEEL: %lf > %lf", scan_ranges[MFEEL], limits[MFEEL]);
				RCLCPP_INFO(this->get_logger(), "LFEEL: %lf > %lf\n", scan_ranges[LFEEL], limits[LFEEL]);
			}
			update_cmd_vel(LINEAR_VELOCITY, 0);
		}
	} else {
		// No space infront
		if (debug) {
			RCLCPP_INFO(this->get_logger(), "Forward blocked, turning right");
			RCLCPP_INFO(this->get_logger(), "FRONT: %lf < %lf", scan_ranges[FRONT], limits[FRONT]);
			RCLCPP_INFO(this->get_logger(), "FFEEL: %lf < %lf", scan_ranges[FFEEL], limits[FFEEL]);
			RCLCPP_INFO(this->get_logger(), "MFEEL: %lf < %lf", scan_ranges[MFEEL], limits[MFEEL]);
			RCLCPP_INFO(this->get_logger(), "LFEEL: %lf < %lf\n", scan_ranges[LFEEL], limits[LFEEL]);
		}

		// Turn right whilst slowing linear speed based on distance from wall using formula below
		auto speed_reduction = (scan_ranges[front_min_angle] - (limits[FRONT] * 0.5)) / limits[front_min_angle];
		if (speed_reduction < 0.0) {speed_reduction = 0.0;}
		update_cmd_vel(LINEAR_VELOCITY * speed_reduction, -1 * ANGULAR_VELOCITY);
	}
}

bool WallFollower::left_close() {
	return scan_ranges[FLEFT] < limits[FLEFT] ||
	 	   scan_ranges[MLEFT] < limits[MLEFT] ||
		   scan_ranges[LLEFT] < limits[LLEFT];
}

bool WallFollower::left_far() {
	return scan_ranges[FLEFT] >= limits[FLEFT] + window_width ||
		   scan_ranges[MLEFT] >= limits[MLEFT] + window_width ||
		   scan_ranges[LLEFT] >= limits[LLEFT] + window_width;
}

bool WallFollower::front_far() {
	if (scan_ranges[FRONT] <= limits[FRONT]) {
		front_min_angle = FRONT;
		return false;
	} else if (scan_ranges[FFEEL] <= limits[FFEEL]) {
		front_min_angle = FFEEL;
		return false;
	} else if (scan_ranges[MFEEL] <= limits[MFEEL]) {
		front_min_angle = MFEEL;
		return false;
	} else if (scan_ranges[LFEEL] <= limits[LFEEL]) {
		front_min_angle = LFEEL;
		return false;
	} else {
		front_min_angle = -1;
		return true;
	}
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
