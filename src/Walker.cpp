/**
 * @file Walker.cpp
 * @brief Class implementation of Walker to compute the 
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


void Walker::updateDistance(const sensor_msgs::LaserScan::ConstPtr& laserData) {
  // Minimum value initialization
  float min = 5;

  // iterate through every point in the laser data to find the closest point
  for (int i = 0; i < laserData->ranges.size(); i++) {
    if (laserData->ranges[i] < min)
      min = laserData->ranges[i];
  }
  distanceToObstacle = min;
  ROS_INFO_STREAM("Distance to Obstacle :" << distanceToObstacle);
}

Walker::Walker(ros::NodeHandle& n) {
  subscriber = n.subscribe("/scan", 10, &Walker::updateDistance, this);
}

bool Walker::getObstacle(){
    // If distance to big go straight else rotate
    if (distanceToObstacle > 0.5) {
      ROS_INFO_STREAM("Moving forward");
      return false;
    } else {
      ROS_INFO_STREAM("Rotating");
      return true;
    }
}