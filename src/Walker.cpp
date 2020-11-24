/**
 * Copyright 2020, Sukoon Sarin
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @file Walker.cpp
 * @author Sukoon Sarin
 * @copyright 2020 BSD
 * @brief Implementation of header of Walker.hpp for navigation
 *
 */

#include <iostream>
#include "Walker.hpp"

Walker::Walker(ros::NodeHandle nh) {
  // ROS publisher to velocity topic
  velocity = nh.advertise <geometry_msgs::Twist>
  ("/cmd_vel", 1);
  // ROS subscriber to LaserScan
  depth = nh.subscribe("/scan", 1000,
          &ObstacleDetector::obstacleScanner, &obstacle);

  // velocity.publish(msg);

  // set loop rate
  ros::Rate loop_rate(4);

  while (ros::ok()) {
    // Initialize linear and angular velocities to zeros
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;

    // check if obstacle is close
    if (obstacle.detector() > 0.45) {
      ROS_INFO_STREAM("Forward");
      msg.linear.x = -0.12;
    } else {
        ROS_INFO_STREAM("Turn");
        msg.angular.z = 1.5;
    }
    velocity.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
