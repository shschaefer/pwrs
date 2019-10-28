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
#include "std_msgs/Empty.h"

#include <mmsystem.h>
#define ERR_NODEVICE -1

int findJoystick()
{
  JOYINFO joyinfo; 
  UINT wNumDevs, wDeviceID; 
  BOOL bDev1Attached, bDev2Attached; 
 
  if((wNumDevs = joyGetNumDevs()) == 0) 
    return ERR_NODEVICE; 

  bDev1Attached = joyGetPos(JOYSTICKID1,&joyinfo) != JOYERR_UNPLUGGED; 
  bDev2Attached = wNumDevs == 2 && joyGetPos(JOYSTICKID2,&joyinfo) != JOYERR_UNPLUGGED; 

  if(bDev1Attached || bDev2Attached)   // decide which joystick to use 
    wDeviceID = bDev1Attached ? JOYSTICKID1 : JOYSTICKID2; 
  else 
    return ERR_NODEVICE;

  return wDeviceID;
}

float getJoyVelocity(DWORD dwPos)
{
	// Joystick axes are +MAX = 0, -MAX=65535 - Right Handed coordinates
	float velocity = (32768.0 - (float)dwPos) / 32768.0;
	return velocity;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joystick_node");
 
  // Initialize connection to Joystick
  int joyId = findJoystick();
  if (joyId == ERR_NODEVICE)
  {
    ROS_ERROR("Unable to find Joystick device.");
    exit(-1);
  }
 
  ros::NodeHandle joyNodeHandle;
  ros::Publisher joystickPublisher = joyNodeHandle.advertise<ackermann_msgs::AckermannDrive>("cmd_ack", 100); 
  
  // Rate is measured in Hz
  ros::Rate loop_rate(5);

  while (ros::ok())
  {
    JOYINFOEX joyInfo;
    joyInfo.dwSize = sizeof(joyInfo);
    joyInfo.dwFlags = JOY_RETURNALL;
    if (joyGetPosEx(joyId, &joyInfo) == JOYERR_NOERROR)
    {
      ackermann_msgs::AckermannDrive msg;

      move_msg.speed = getJoyVelocity(joyInfo.dwXpos);
      move_msg.acceleration = 0.0;
      move_msg.jerk = 0.0;
      move_msg.steering_angle = getJoyVelocity(joyInfo.dwYpos);
      move_msg.steering_angle_velocity = 1.0;

      joystickPublisher.publish(msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}