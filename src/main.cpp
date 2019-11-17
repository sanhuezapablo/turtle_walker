/**
 * Distributed under the BSD License (license terms found in LICENSE or at https://www.freebsd.org/copyright/freebsd-license.html)
 * @file main.cpp
 * @brief Program runs turtlebot walker
 * @author Pablo Sanhueza
 * @copyright 2019 Pablo Sanhueza
 */


#include "walker.hpp"


/**
 * @brief main function
 * @param argc  The argc
 * @param argv  The argv
 * @return none
 */
int main(int argc, char **argv) {
  /// Initializing the node name as walker
  ros::init(argc, argv, "walker");

  /// Walker object
  walker walk;

  /// Publishes at 5 Hz
  ros::Rate loop_rate(5);

  while (ros::ok()) {
    /// Calling setMsg from walker class
    walk.setMsg();

    /// Calling robotWalk from walker class
    walk.robotWalk();

    /// Calling publishMsg from walker class
    walk.publishMsg();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
