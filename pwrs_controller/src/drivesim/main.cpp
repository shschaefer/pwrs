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

#define _USE_MATH_DEFINES // for C
#include <math.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

double velocity = 0.5; // m/s
double robot_L = 0.4318; // Wheelbase - 17 inches
double steering_angle_phi = (15.0 * M_PI) / 180.0; // 15 degrees in radians

void PublishTransform(tf::TransformBroadcaster caster, ros::Time current_time, 
	double x, double y, geometry_msgs::Quaternion odom_quat)
{
    //set the tf transform header
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

	//set the main transform
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    caster.sendTransform(odom_trans);
}

void PublishOdometry(ros::Publisher pub, ros::Time current_time, 
	double x, double y, geometry_msgs::Quaternion odom_quat, double vx, double vy, double vth)
{
    //set the odometry header
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    pub.publish(odom);
}

void PublishJointStates(ros::Publisher pub, ros::Time current_time)
{
	sensor_msgs::JointState states;
	states.header.stamp = current_time;
	//states.name = { "left_steering_mount", "left_front_axle", "right_steering_mount", "right_front_axle",
	//	"left_rear_axle", "right_rear_axle" };
	//states.position = { 0.5, 0.0, 0.5, 0.0, 0.0, 0.0 };
	states.name = { "left_steering_mount", "right_steering_mount" };
	states.position = { steering_angle_phi, steering_angle_phi };
	
	pub.publish(states);
}

void ComputeCircleStep(double *x, double *y, double *th, 
						double *vx, double *vy, double *vth, double dt)
{
	*vx = 0.1;
	*vy = -0.1;
	*vth = 0.1;
  
    double delta_x = (*vx * cos(*th) - *vy * sin(*th)) * dt;
    double delta_y = (*vx * sin(*th) + *vy * cos(*th)) * dt;
    double delta_th = *vth * dt;

    *x += delta_x;
    *y += delta_y;
    *th += delta_th;
}

void ComputeAckermanCircle(double *x, double *y, double *th, 
						double *vx, double *vy, double *vth, double dt)
{
  double delta_m = velocity * dt;
 
  double delta_theta = (delta_m / robot_L) * tan(steering_angle_phi);
  double delta_x = delta_m * cos(*th + delta_theta / 2);
  double delta_y = delta_m * sin(*th + delta_theta / 2);  

  *vx = delta_x / dt;
  *vy = delta_y / dt;
  *vth = delta_theta / dt;
  
  *x += delta_x;
  *y += delta_y;
  *th += delta_theta;  
}

int main(int argc, char** argv){
  ros::init(argc, argv, "drivesim_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  // ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 5);
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("steering", 5);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    ComputeAckermanCircle(&x, &y, &th, &vx, &vy, &vth, dt);
		
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    // Publish the odometry, transform(s) and joint states
    PublishTransform(odom_broadcaster, current_time, x, y, odom_quat);
    PublishOdometry(odom_pub, current_time, x, y, odom_quat, vx, vy, vth);
    PublishJointStates(joint_pub, current_time);
	
    last_time = current_time;
    r.sleep();
  }	
 
  return 0;
}