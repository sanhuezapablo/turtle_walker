/**
 * Distributed under the BSD License (license terms found in LICENSE or at https://www.freebsd.org/copyright/freebsd-license.html)
 * @file walker.hpp
 * @brief Header file for walker class.
 * @author Pablo Sanhueza
 * @copyright 2019 Pablo Sanhueza
 */


#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

/**
 * @brief Class for moving robot.
 */
class walker {
 public:
  /**
   * @brief Constructor for walker
   */
  walker();

  /**
   * @brief Method to call back LaserScan
   * @param msg Sensor messages
   * @return none
   */  
  void sensorCallBack(const sensor_msgs::LaserScan::ConstPtr& msg);

  /**
   * @brief Sets initial parameters of msg
   * @param none
   * @return none
   */  
  void setMsg();

  /**
   * @brief Publishes msg to robot velocity
   * @param none
   * @return none
   */ 
  void publishMsg();

  /**
   * @brief Method to move robot
   * @param none
   * @return none
   */ 
  void robotWalk();

 private:
  /// Boolean to detect obstacles
  bool object;

  /// Node handle object
  ros::NodeHandle n;

  /// Subscribe to laser scan topic
  ros::Subscriber sensor;

  /// Publisher object. This will publish to robot velocity
  ros::Publisher velocity;

  /// Geometry msg later passed to robot velocity
  geometry_msgs::Twist msg;
};

#endif  // INCLUDE_WALKER_HPP_
