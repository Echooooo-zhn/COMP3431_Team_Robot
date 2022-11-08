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
#include <fstream>
#include <iostream>

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

	confidence = MAX_LEVEL;

    linear_velocity = LINEAR_VELOCITY;
    angular_velocity = ANGULAR_VELOCITY;

	robot_pose_ = 0.0;
	prev_robot_pose_ = 0.0;

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

	robot_pose_ = yaw;
}

void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	uint16_t scan_angle[4] = {0, 90, 45, 180, 225};

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

void WallFollower::log_record(double scan_data_, string message)
{
    ofstream location_out;
    string scan = "CENTER: " + string(scan_data_[0]) + "LEFT: " + string(scan_data_[1])+ "OFF_LEFT: " + string(scan_data_[2])+ "RIGHT: " + string(scan_data_[3])+ "OFF_RIGHT: " + string(scan_data_[4]);
    location_out.open("log_record.txt", std::ios::out | std::ios::app);
    if (!location_out.is_open())
    {
        return 0;
    }
    location_out << scan << endl;
    location_out << message << endl;
    location_out.close();
}

/********************************************************************************
** Update functions
********************************************************************************/
void WallFollower::update_callback()
{
	// Current state of the vehicle, i.e. turning left, going straight etc
	static uint8_t turtlebot3_state_num = 0;
	double escape_range = 5.0 * DEG2RAD;
	double check_forward_dist = 0.45;
	double check_side_dist = 0.2;
	double window_width = 0.21;

	// Measure and Change confidence level
	RCLCPP_INFO(this->get_logger(), "Confidence: %d", confidence);

	// Check if there is space in front 
	if (scan_data_[CENTER] > check_forward_dist ||  scan_data_[CENTER] == 0.0)
	{
		if (scan_data_[LEFT] < check_side_dist + 0.1)
		{
			// Too close to left wall, turning right
            RCLCPP_INFO(this->get_logger(), "Too close to left wall");
            log_record(scan_data_, "Too close to left wall");
			prev_robot_pose_ = robot_pose_;
			confidence++;
			update_cmd_vel(linear_velocity, -1 * angular_velocity - 0.1);
		}
		else if (scan_data_[OFF_LEFT] <= (check_side_dist + 0.3))
		{
			// Too close to right wall, turning left
            RCLCPP_INFO(this->get_logger(), "Seeing left 45deg wall");
            log_record(scan_data_, "Seeing left 45deg wall");
			escape_range = 90 * DEG2RAD;
			prev_robot_pose_ = robot_pose_;
			confidence += 10;
		}
		else if (scan_data_[LEFT] > (check_side_dist + window_width))
		{
			// Move forward a certain distance
			// turn left
			// Too far from left wall, turning left
            RCLCPP_INFO(this->get_logger(), "Too far from left wall");
            log_record(scan_data_, "Too far from left wall");
			prev_robot_pose_ = robot_pose_;
			turtlebot3_state_num = TB3_LEFT_TURN;
			confidence--;
		} 
		else
		{
			// If we're not too close to anything, but there is enough space infront, go forward
            RCLCPP_INFO(this->get_logger(), "Within window, moving forward");
            log_record(scan_data_, "Within window, moving forward");
			confidence++;
		}
		
	} else {
		// If there is something in front, turn right
        RCLCPP_INFO(this->get_logger(), "U TURN");
        log_record(scan_data_, "U TURN");
		prev_robot_pose_ = robot_pose_;
		turtlebot3_state_num = TB3_RIGHT_TURN;
		confidence = 0;
	}

	if (confidence > MAX_LEVEL) confidence = MAX_LEVEL;
	if (confidence < 0) confidence = 0;
	
	// Act on confidence level
	if (confidence > LEFT_TURN_LEVEL) {
		// FORWARD
        RCLCPP_INFO(this->get_logger(), "Act on confidence level ==> Move forward");
        log_record(scan_data_, "Act on confidence level ==> Move forward");
        linear_velocity = LINEAR_VELOCITY;
        angular_velocity = ANGULAR_VELOCITY;
		update_cmd_vel(linear_velocity, 0.0);
		turtlebot3_state_num = GET_TB3_DIRECTION;

	} else if (confidence <= LEFT_TURN_LEVEL) {
		// LEFT
        RCLCPP_INFO(this->get_logger(), "Act on confidence level ==> Turn left");
        log_record(scan_data_, "Act on confidence level ==> Turn left");
        linear_velocity -= 0.05
        angular_velocity += 0.05
		update_cmd_vel(linear_velocity, angular_velocity);
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
