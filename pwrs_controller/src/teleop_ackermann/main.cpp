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
#include <sensor_msgs/Joy.h>

ros::Publisher g_pub;

float getJoyVelocity(uint32_t dwPos)
{
	// Joystick axes are +MAX = 0, -MAX=65535 - Right Handed coordinates
	float velocity = (32768.0 - (float)dwPos) / 32768.0;
	return velocity;
}

void JoyToAckCallback(const sensor_msgs::Joy::ConstPtr& joyMsg)
{
  if (joyMsg->axes.size() > 1)
  {
    ackermann_msgs::AckermannDrive msg;

    msg.speed = getJoyVelocity(joyMsg->axes[0]);
    msg.acceleration = 0.0;
    msg.jerk = 0.0;
    msg.steering_angle = getJoyVelocity(joyMsg->axes[1]);
    msg.steering_angle_velocity = 1.0;

    g_pub.publish(msg);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_ackermann");
  
  ros::NodeHandle nh;
  g_pub = nh.advertise<ackermann_msgs::AckermannDrive>("cmd_ack", 1); 
  ros::Subscriber sub = nh.subscribe("/joy", 1, JoyToAckCallback);
  
  return ros::spin();
}