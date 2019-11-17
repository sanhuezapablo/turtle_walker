/**
 * Distributed under the BSD License (license terms found in LICENSE or at https://www.freebsd.org/copyright/freebsd-license.html)
 * @file walker.cpp
 * @brief Source code for class walker
 * @author Pablo Sanhueza
 * @copyright 2019 Pablo Sanhueza
 */

#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Twist.h"
#include "walker.hpp"

/**
 * @brief Constructor for walker
 */
walker::walker() {
  /// Obstacles initialized to false
  object = false;

  /// Sensor data
  sensor = n.subscribe<sensor_msgs::
    LaserScan>("/scan", 50, &walker::sensorCallBack, this);

  /// Publishes velocity to robot
  velocity = n.advertise<geometry_msgs::Twist>
    ("/mobile_base/commands/velocity", 1000);
}

/**
 * @brief Method that calls back LaserScan
 */
void walker::sensorCallBack(const sensor_msgs::LaserScan::ConstPtr& msg) {
  for (auto i : msg->ranges) {
    /// Checks obstacle within 1 range
    if (i < 1) {
      object = true;
      return;
    }
  }

  object = false;
}

/**
 * @brief Initilizes msg parameters
 */
void walker::setMsg() {
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
}

/**
 * @brief Pusblishes msg to velocity of robot
 */
void walker::publishMsg() {
  velocity.publish(msg);
}

/**
 * @brief Method that moves robot
 */    
void walker::robotWalk() {
  if (object == true) {
    msg.angular.z = 0.5;
    ROS_INFO_STREAM("Rotating");
  } else {
    msg.linear.x = 1;
    ROS_INFO_STREAM("Moving Forward");
  }
}
