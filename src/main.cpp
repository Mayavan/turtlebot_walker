/**
 * @file main.cpp
 * @brief main file for implementation of Walker to compute the 
 * linear and angular velocity based on distance from an obstacle.
 *
 * @author Mayavan
 * @copyright 2018, Mayavan
 *
 * MIT License
 * Copyright (c) 2018 Mayavan
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "turtlebot_walker/Walker.hpp"
#include "geometry_msgs/Twist.h"

/**
 * @brief main function to create the node and publish data to turtlebot based on laser data
 * @param argc Standard main function parameter
 * @param argv Standard main function parameter
 * @return 0 if execution completed successfully
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "walker");
  ros::NodeHandle n;

  // Initialize publisher to send velocity control to turtle bot
  ros::Publisher publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  Walker walker(n);

  ros::Rate loop_rate(4);
  geometry_msgs::Twist command;

  command.linear.x=0;
  command.linear.y=0;
  command.linear.z=0;

  command.angular.x=0;
  command.angular.y=0;
  command.angular.z=0;

  while (n.ok()) {
  	if(walker.getObstacle()){
  		command.linear.x=-0.5;
        publisher.publish(command);
        ros::Duration(4).sleep();
  		command.angular.z = 1;
  		command.linear.x= 0.0;
        publisher.publish(command);
        ros::Duration(2).sleep();
  	}
  	else{
  		command.linear.x=0.5;
  		command.angular.z=0.0;
        publisher.publish(command);
  	}
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}