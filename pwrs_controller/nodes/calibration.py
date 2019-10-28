#!/usr/bin/env python

""" calibration.py

Accurate calibration is a multi-step process.  Build up towards execution of the loop described by:

http://people.tamu.edu/~animodor/papers/(J)Mechatronics2010.pdf

0) Adjust steering and/or calibration offset to ensure the robot can move in a straight line.
1) Calibrate general speed curve - Talon SRX is generally linear
  a) Obtain translations for Arduino to turn velocities into PWM
  b) Estimate velocities based on timed runs at fixed speeds assuming infinite acceleration and braking
2) Calibrate straight line odometry
  a) Given a set of timed travels, find the distance and number pulses - should be a linear fit
    or
  b) Given a set of timed travels, estimate the covariance parameters for EKF
3) Calibrate steering angles
  a) Obtain translations for Arduino to turn angular position into servo control
  b) Test angles against ability to transcribe a circular of known radius per the geometry
  c) Find steering error, repeatability, etc...
4) Calibrate linear control
  a) Given a fixed position, achieve a goal position within some reasonable error vs. odometry
5) Execute CW/CCW Loops to fix errors in tread and wheel diameter - straights are 2r long
  a) Calibrate tread
  b) Calibrate wheel diameters

Model Parameters:
- L = Wheel Base - length between front and rear axle
- b = Track/Tread - distance between the center of wheels on the same axle
- D = Wheel diameter
- phi = Steering angle

The model assumes coordinate axes on the middle of the rear axle.

Code follows pattern defined by ROS rbx1_nav package - https://github.com/pirobot/ros-by-example/
All calibration data is external to this node.  It is sent to the Arduino controller from the launch parameters.
This node must be stopped and restarted to modify calibration and run a new test.

Load the ackermann.ino program and set calibration mode for tests 0-3.  Turn off calibration mode for the
rest, as you will need to be able to utilize odometry.
 
"""

import rospy
from geometry_msgs.msg import Point
import tf

class CalibrateLoop():
    def __init__(self):
        rospy.init_node('calibrate_loop', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        
        # Set our update rate
        self.rate = 10
        r = rospy.Rate(self.rate)
        
        # Set the base test parameters
        self.test_distance = rospy.get_param('~test_distance', 1.0) # meters
        self.speed = rospy.get_param('~speed', 0.15) # meters per second
        self.tolerance = rospy.get_param('~tolerance', 0.01) # meters
        self.test_variant = rospy.get_param('~test_variant', "SPEED_CURVE")

        # Publish Ackerman drive messages to manually control the robot
        self.cmd_ack = rospy.Publisher('/cmd_ack', AckermannDrive, queue_size=5)
 
        # The base frame is base_link for SpongeBot
        self.base_frame = rospy.get_param('~base_frame', '/base_link')

        # The odom frame is /odom for now - will become /odom_ekf when IMU is integrated
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))
            
        rospy.loginfo("Bring up rqt_reconfigure to control the test.")
  
        # Get the starting position from the tf transform between the odom and base frames
        self.position = Point()        
        self.position = self.get_position()
        x_start = self.position.x
        y_start = self.position.y
		
        case self.test_variant:
            switch "SPEED_ODOMETRY":
                move_forward_leg(self.speed, self.test_distance, self.rate)

            switch "STEERING_ANGLE":
                # Circumscribe a circle of known radius three times
		        rotate_leg(self.speed, self.test_distance / 2, 6 * pi, self.rate)

            switch "LINEAR_CONTROL":
                move_forward_controlled(self.speed, self.test_distance, self.rate)
			    
            switch "CAL_LOOP":
                # Four stages in the open loop - out, turn, back, turn
                move_forward_controlled(self.speed, self.test_distance, self.rate)
		        rotate_leg(self.speed, self.test_distance / 2, pi, self.rate)
		        move_forward_controlled(self.speed, self.test_distance, self.rate)
		        rotate_leg(self.speed, self.test_distance / 2, pi, self.rate)

        # Robot may already be stopped
		shutdown()
		self.position = self.get_position()
        rospy.loginfo("TF Position: {0} {1}", self.position.x - x_start, self.position.y - y_start)

    def rotate_leg(speed, radius, arc, update_rate):
        # Initialize the movement command
        move_cmd = AckermannDrive()

        # Set the forward speed
        # Setting no acceleration or jerk will accelerate at maximum rate
        move_cmd.speed = speed
		
		# Set the steering angle - Need to derive from CLMR Kinematics
        # TODO: Check to see that it is within the geometric limits of the robot
        angle = math.atan(wheelbase / radius)
		move_cmd.steering_angle = angle
        
        # Move forward for a time to go the desired distance
		distance = arc * radius
        duration = distance / speed
        ticks = int(duration * update_rate)

        for t in range(ticks):
            self.cmd_ack.publish(move_cmd)
            r.sleep()
            
        # Stop the robot before the next leg
        self.cmd_ack.publish(AckermanDrive())
        rospy.sleep(1)
        
    def move_forward_leg(speed, distance, update_rate):
        # Initialize the movement command
        move_cmd = AckermannDrive()
            
        # Set the forward speed
        # Setting no acceleration or jerk will accelerate at maximum rate
        move_cmd.speed = speed
        
        # Move forward for a time to go the desired distance
        duration = distance / speed
        ticks = int(duration * update_rate)

        for t in range(ticks):
            self.cmd_ack.publish(move_cmd)
            r.sleep()
            
        # Stop the robot before the next leg
        self.cmd_ack.publish(AckermanDrive())
        rospy.sleep(1)

    def move_forward_controlled(speed, distance, update_rate):
        # Initialize the movement command
        move_cmd = AckermannDrive()
            
        # Set the forward speed
        # Setting no acceleration or jerk will accelerate at maximum rate
        move_cmd.speed = speed
		
		# Continue to move until we get close to the mark
        error = distance
		while abs(error) > self.tolerance

            # If we have not achieved our goal, move in the appropriate direction
            move_cmd.speed = copysign(self.speed, -1 * error)	
		    self.cmd_ack.publish(move_cmd)
            r.sleep()
			
            # Get the current position from the tf transform between the odom and base frames
            self.position = self.get_position()
                
            # Compute the Euclidean distance from the target point
            distance_travelled = sqrt(pow((self.position.x - x_start), 2) +
                                      pow((self.position.y - y_start), 2))
               
            # How close are we?
            error =  distance_travelled - distance
                
        # Stop the robot before the next leg
        self.cmd_ack.publish(AckermanDrive())
        rospy.sleep(1)
        
    def get_position(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return Point(*trans)
        
    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_ack.publish(AckermanDrive())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        CalibrateLoop()
        rospy.spin()
    except:
        rospy.loginfo("Calibration terminated.")
