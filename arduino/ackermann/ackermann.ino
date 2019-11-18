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


#include <ros.h>
#include <ros/time.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <rosgraph_msgs/Log.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <ServoSteering.h>
#include <MotorController.h>
#include <SonarRanging.h>
#include <stdio.h>

//
// Compacted all calibration, teleoperation and other modes into
// one single module.  This moves things like joystick control into
// ROS master nodes rather than recomputation in messy Arduino firmware.
//

//
// ROS Node declarations - publishers and subscribers
//
ros::NodeHandle nh;
rosgraph_msgs::Log logMsg;
ros::Publisher logger("serial_log", &logMsg);

nav_msgs::Odometry odom;
ros::Publisher odomPublisher("odom", &odom);

tf::TransformBroadcaster tfCaster;
sensor_msgs::JointState jstate;
ros::Publisher steeringPublisher("steering", &jstate);

sensor_msgs::Range range;
ros::Publisher rangePublisher("range", &range);

sensor_msgs::Imu inertial;
ros::Publisher imuPublisher("inertial", &inertial);

//
// Peripherals - Motor, Steering, Encoders
//
ServoSteering *steering;
const byte servoPin = 11;
float steeringAnglePhi = 0;

MotorController *motorController;
int motorPin = 13; // Leonardo doesn't support PWM on pin 12, but LED is 13!!!

const byte buttonPin = 12;

#define NUM_SONARS 3
SonarRanging *ranger;
const byte sonar1Pin = A3;
const byte sonar2Pin = A4;
const byte sonar3Pin = A5;
const byte sonarPowerPin = 6;
const byte sonarEnablePin = 5;

// Simplified, Firmata-ish protocol with the IMU
#define CALIBRATE         0xA0
#define DATA_REQUEST      0xB0
#define RESPONSE_END      0xFF
typedef union {
  float value;
  byte data[4];
} floatData;

//
// ROSPARAM - Robot constants
//
float robotWheelbase = 0;

//
// Local variables
//
bool connected = false;
int calibrationMode = false;
ros::Time lastTime;
char baseLink[] = "/base_link";
char odomLink[] = "/odom";
char imuLink[] = "/imu";
float odomX = 0;
float odomY = 0;
float odomTheta = 0;


void rosLog(char *logMessage)
{
  if (!connected)
  {
    Serial.println(*logMessage);
  }
  else
  {
    logMsg.msg = logMessage;
    logger.publish(&logMsg);
  }
}

void rosLog(String logMessage)
{
  rosLog(logMessage.c_str());
}

void moveHandler(const ackermann_msgs::AckermannDrive& move_msg)
{
  motorController->SetMotorSpeed(move_msg.speed, move_msg.acceleration, move_msg.jerk);
  steering->Steer(move_msg.steering_angle, move_msg.steering_angle_velocity);
}

ros::Subscriber<ackermann_msgs::AckermannDrive> moveCommand("cmd_ack", &moveHandler );

void ComputeAckermannIncrement(float deltaT, 
	float *velocityX, float *velocityY, float *velocityTheta)
{
  // Get the current control positions
  steeringAnglePhi = steering->GetCurrentAngle();
  float velocity = motorController->GetCurrentSpeed();

  // TODO: Fix the incorrect estimate for odometry
  // Currently using the last known steering and velocity setting
  // This is wrong if either is modified within the interval
  // Small errors also accumulate due to latency to achieve desired setting
  float deltaM = velocity * deltaT;
  float deltaTheta = (deltaM / robotWheelbase) * tan(steeringAnglePhi);
  float deltaX = deltaM * cos(odomTheta + deltaTheta / 2);
  float deltaY = deltaM * sin(odomTheta + deltaTheta / 2);  

  odomX += deltaX;
  odomY += deltaY;
  odomTheta += deltaTheta;
  
  *velocityX = deltaX / deltaT;
  *velocityY = deltaY / deltaT;
  *velocityTheta = deltaTheta / deltaT;
}

void PublishTransform(tf::TransformBroadcaster caster, ros::Time currentTime, 
	float x, float y, geometry_msgs::Quaternion quat)
{
  // Publish a TF transform using the accumulated odometry data
  geometry_msgs::TransformStamped xform;
  xform.header.stamp = currentTime;
  xform.header.frame_id = odomLink;
  xform.child_frame_id = baseLink;
  xform.transform.translation.x = x;
  xform.transform.translation.y = y;
  xform.transform.translation.z = 0.0;
  xform.transform.rotation = quat;

  caster.sendTransform(xform);
}

void PublishOdometry(ros::Publisher publisher, ros::Time currentTime,
	float x, float y, geometry_msgs::Quaternion quat, float vx, float vy, float vth)
{
  // Publish odometry data
  odom.header.stamp = currentTime;
  odom.header.frame_id = odomLink;
  odom.child_frame_id = baseLink;

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = quat;
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;

  publisher.publish(&odom);
}

void PublishJointStates(ros::Publisher publisher, ros::Time currentTime)
{
  char *names[] = { "left_steering_mount", "right_steering_mount" };
  float positions[] = { steeringAnglePhi, steeringAnglePhi };

  jstate.header.stamp = currentTime;
  jstate.name = names;
  jstate.name_length = 2;
  jstate.position = positions;
  jstate.position_length = 2;
  jstate.velocity_length = 0;
  jstate.effort_length = 0;
  
  publisher.publish(&jstate);
}

float readImuFloat()
{
  // TODO: Make insensitive to lost bytes....
  floatData fvalue;
  for (int i = 0; i<4; i++) fvalue.data[i] = Serial2.read();
  return fvalue.value;
}

void ReadAndPublishInertialState(ros::Time currentTime)
{
  inertial.header.stamp = currentTime;
  inertial.header.frame_id = imuLink;

  Serial2.write(DATA_REQUEST);

  // The first byte back should be an echo of the DATA_REQUEST
  int responseByte = Serial2.read();
  if (responseByte != DATA_REQUEST)
  {
    // TODO: Write a diagnostics message
    rosLog("Unable to read IMU data.");

    // Read to the end
    while (Serial2.available())
    {
      responseByte = Serial2.read();
    }

    // TODO: What do we do when the inertial data is no good?
    return;
  }

  inertial.orientation.x = readImuFloat();
  inertial.orientation.y = readImuFloat();
  inertial.orientation.z = readImuFloat();
  inertial.orientation.w = readImuFloat();
  
  inertial.linear_acceleration.x = readImuFloat();
  inertial.linear_acceleration.y = readImuFloat();
  inertial.linear_acceleration.z = readImuFloat();

  inertial.angular_velocity.x = readImuFloat();
  inertial.angular_velocity.y = readImuFloat();
  inertial.angular_velocity.z = readImuFloat();

  // Read the RESPONSE_END byte.
  // Not sure it matters if we test to make sure it is the right value.
  responseByte = Serial2.read();

  imuPublisher.publish(&inertial);
}

void reportOdometry()
{
  ros::Time currentTime = nh.now();

  // Compute odometry given the velocities of the robot
  float dt = currentTime.toSec() - lastTime.toSec();
  float vx, vy, vth;
  ComputeAckermannIncrement(dt, &vx, &vy, &vth);
  
  // All odometry is 6DOF in ROS
  geometry_msgs::Quaternion odomQuat = tf::createQuaternionFromYaw(odomTheta);

  // Publish the odometry, transform(s) and joint states
  PublishTransform(tfCaster, currentTime, odomX, odomY, odomQuat);
  PublishOdometry(odomPublisher, currentTime, odomX, odomY, odomQuat, vx, vy, vth);
  PublishJointStates(steeringPublisher, currentTime);

  lastTime = currentTime;
  nh.spinOnce();
}

void reportSensors()
{
  ros::Time currentTime = nh.now();

  // TODO: How to feedback state from EKF to update IMU estimates
  ReadAndPublishInertialState(currentTime);

  // Take sonar readings
  ranger->Range();

  // Setup header boilerplate
  range.header.stamp = currentTime;
  range.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range.field_of_view = SONAR_FIELD_OF_VIEW;
  range.min_range = SONAR_MIN_RANGE;
  range.max_range = SONAR_MAX_RANGE;

  // Report sonar sensor messages
  for(int i=0; i < NUM_SONARS; i++)
  {
      char ident[8];
      sprintf(ident, "/range%d", i + 1);
      range.header.frame_id = ident;
      range.range = ranger->GetDistance(i);
      rangePublisher.publish(&range);

      nh.spinOnce();
  }
}

void setup()
{
  nh.initNode();

  // Wired up an input button from the ProtoShield
  pinMode(buttonPin, INPUT);

  // Initialize calibration parameters
  float steeringOffset = 0;
  float steeringSlope = 0;
  float steeringTravel = 0;
  float velocitySlope = 0;
  nh.getParam("/spongebot/calibrate", &calibrationMode);
  nh.getParam("/spongebot/wheelbase", &robotWheelbase);
  nh.getParam("/spongebot/velocitySlope", &velocitySlope);
  nh.getParam("/spongebot/steeringOffset", &steeringOffset);
  nh.getParam("/spongebot/steeringSlope", &steeringSlope);
  nh.getParam("/spongebot/steeringTravel", &steeringTravel);

  motorController = new MotorController(motorPin, velocitySlope);

  steering = new ServoSteering(&rosLog, servoPin, steeringSlope, steeringOffset, steeringTravel);

  if (!calibrationMode)
  {
    // Initialize the sonar array
    int sonarPins[3] = { sonar1Pin, sonar2Pin, sonar3Pin };
    ranger = new SonarRanging(&rosLog, sonarEnablePin, sonarPowerPin, NUM_SONARS, sonarPins);
    ranger->Initialize();

    // Initialize the IMU - Serial2 is pin 16/17
    Serial2.begin(115200);
  }

  // Initialize ROS pubs and subs
  tfCaster.init(nh);
  nh.advertise(logger);
  nh.advertise(odomPublisher);
  nh.advertise(steeringPublisher);
  nh.advertise(imuPublisher);
  nh.advertise(rangePublisher);
  nh.subscribe(moveCommand);
  nh.spinOnce();
  
  connected = true;
  lastTime = nh.now();
}

void loop()
{
  // We need a way to effectively spin and delay one cycle - say once per second
  ros::Time currentTime;
  do
  {
    nh.spinOnce();
    delay(1);
    currentTime = nh.now();
  } while (currentTime.toSec() - lastTime.toSec() < 1.0);
  
  reportOdometry();
  
  if (!calibrationMode)
    reportSensors();
}

// Events from the IMU??
//void serialEvent2(){
//
//}