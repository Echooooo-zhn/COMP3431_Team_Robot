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
	uint16_t scan_angle[3] = {0, 30, 330};

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
** TurtleBot will follow the left wall to determine the action
********************************************************************************/
void WallFollower::update_callback()
{
	static uint8_t turtlebot3_state_num = 0;
	double escape_range = 30.0 * DEG2RAD;
	double check_forward_dist = 0.7;
	double check_side_dist = 0.6;
	// bool rotate_and_forward = false;

	// Get the state of the turtlebot
	switch (turtlebot3_state_num)
	{
		case GET_TB3_DIRECTION:

			// If left wall detected
			if (scan_data_[LEFT] < check_side_dist)
			{
				// If front wall detected, move forward
				if (scan_data_[CENTER] < check_forward_dist)
				{
					prev_robot_pose_ = robot_pose_;
					turtlebot3_state_num = TB3_DRIVE_FORWARD;
				}
				// If front wall is not detected, turn right
				else
				{
					turtlebot3_state_num = TB3_RIGHT_TURN;
				}

			}
			// If left wall is not detected, rotate and move forward
			else 
			{
				prev_robot_pose_ = robot_pose_;
				
				// If front wall is detected, rotated right
				if (scan_data_[CENTER] < check_forward_dist)
				{
					turtlebot3_state_num = TB3_RIGHT_TURN;
				}
				// If the front wall is not detected, rotated left
				else
				{
					turtlebot3_state_num = TB3_LEFT_TURN;
				}
				
				// // After the rotate, do the move forward action immediately.
				// rotate_and_forward = true;
			}

			break;

		// Turtlebot is moving forward
		case TB3_DRIVE_FORWARD:
			// Keep moving forward until front wall is detected
			if (scan_data_[CENTER] > check_forward_dist)
			{
				update_cmd_vel(LINEAR_VELOCITY, 0.0);
				
			}
			else
			{
				turtlebot3_state_num = GET_TB3_DIRECTION;
			}
			break;

		// Turtlebot is turning right currently
		case TB3_RIGHT_TURN:
			if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range)
			{
				turtlebot3_state_num = GET_TB3_DIRECTION;
			}
			else
			{
				update_cmd_vel(0.0, -1 * ANGULAR_VELOCITY);
			}
			break;

		// Turtlebot is turning left currently
		case TB3_LEFT_TURN:
			if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range)
			{
				// Check the rotate_and_forward flag, if after the rotation,
				// move forward action should be made, set the state num into
				// TB3_DRIVE_FORWARD.
				if (rotate_and_forward)
				{
					turtlebot3_state_num = TB3_DRIVE_FORWARD;
				}
				else
				{
					turtlebot3_state_num = GET_TB3_DIRECTION;
				}
			}
			else
			{
				update_cmd_vel(0.0, ANGULAR_VELOCITY);
			}
			break;

		default:
			turtlebot3_state_num = GET_TB3_DIRECTION;
			break;
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
