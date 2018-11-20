/************************************************************************
 MIT License
 Copyright (c) 2018 Ajeet Wankhede
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 *************************************************************************/

/**
 *  @copyright MIT License 2018 Ajeet Wankhede
 *  @file    cleaner.hpp
 *  @author  Ajeet Wankhede
 *  @date    11/19/2018
 *  @version 1.0
 *
 *  @brief UMD ENPM 808X, Week 12, Working with Gazebo
 *
 *  @section DESCRIPTION
 *
 *  This assignment implements a simple walker algorithm for a vacuum robot.
 *  This package launches a Gazebo simulation with TurtleBot
 *  This is a header file containing declaration for class Cleaner
 */

#ifndef INCLUDE_CLEANER_HPP_
#define INCLUDE_CLEANER_HPP_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class Cleaner {
 public:
  /**
   *   @brief Default constructor for Cleaner
   *
   *   @param nothing
   *   @return nothing
   */
  Cleaner();
  /**
   *   @brief Default destructor for Cleaner
   *
   *   @param nothing
   *   @return nothing
   */
  ~Cleaner();
  /**
   *   @brief This is a callback function for laser data
   *
   *   @param LaserScan message
   *
   *   @return none
   */
  void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  /**
   *   @brief Use this function to start the robot
   *
   *   @param none
   *
   *   @return none
   */
  void clean();
  bool obstacle;

 private:
  geometry_msgs::Twist msg;
  ros::NodeHandle n;
  ros::Publisher velocity_pub;
  ros::Subscriber sub;
  float speedX, rotateZ;
};

#endif  // INCLUDE_CLEANER_HPP_
