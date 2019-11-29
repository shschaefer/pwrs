/*
The MIT License (MIT)

Copyright (c) 2016 

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
*/

#include "ros/ros.h"
#include "ackermann_msgs/AckermannDrive.h"

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES // for C
#endif
#include <math.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joystick_node");
  
  ros::NodeHandle wigglerNodeHandle;
  ros::Publisher wigglerPublisher = wigglerNodeHandle.advertise<ackermann_msgs::AckermannDrive>("cmd_ack", 100); 
  
  // Rate is measured in Hz
  float rate = 5;
  ros::Rate loop_rate(rate);

  // Setup some basic speed parameters
  float linearSpeed = 2.0; // m/s
  float angularSpeed = 0.5; // rad/s
  float goal_angle = M_PI / 4.0; // 45 deg
  
  bool turning = false;
  float leftOrRight = -1.0;
  int count = 0;
  while (ros::ok())
  {
    ackermann_msgs::AckermannDrive msg;
	
    if (!turning)
    {
      msg.speed = linearSpeed;
      msg.acceleration = 0.0;
      msg.jerk = 0.0;
    }
    else
    {
      msg.steering_angle = leftOrRight * angularSpeed;
      msg.steering_angle_velocity = 1.0;
    }

    wigglerPublisher.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    count++;

    if (count > 25) // arbitary, should be calculated from speed / distance
    {
      if (turning)
      {
        leftOrRight *= -1.0;
      }
	
      count = 0;
      turning != turning;
    }
  }

  return 0;
}