  
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

#define PATRICKBOT
//#define SPONGEBOT

#ifdef PATRICKBOT
#define ADAFRUIT_10DOF_IMU
#endif

#ifdef ADAFRUIT_10DOF_IMU
#include <Adafruit_Sensor_Set.h>
#include <Adafruit_Simple_AHRS.h>
#include <Madgwick.h>
#include <Mahony.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_Simple_AHRS.h>
#include <Adafruit_L3GD20_U.h>
#else
#endif

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
#ifdef SPONGE2
int motorPin = 12;
#else
int motorPin = 13; // Leonardo doesn't support PWM on pin 12, but LED is 13!!!
#endif


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
int calibrationMode = false;
ros::Time lastTime;
char baseLink[] = "/base_link";
char odomLink[] = "/odom_link";
char imuLink[] = "/imu_link";
float odomX = 0;
float odomY = 0;
float odomTheta = 0;

#ifdef ADAFRUIT_10DOF_IMU
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);
Adafruit_L3GD20_Unified       gyro(20);

// Create simple AHRS algorithm using the above sensors.
Adafruit_Simple_AHRS          ahrs(&accel, &mag);

Madgwick filter;
float mag_offsets[3]            = { 2.45F, -4.55F, -26.93F };

float mag_softiron_matrix[3][3] = { {  0.961,  -0.001,  0.025 },
                                    {  0.001,  0.886,  0.015 },
                                    {  0.025,  0.015,  1.176 } };

float mag_field_strength        = 44.12F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
// Raw values converted to rad/s based on 250dps sensitiviy (1 lsb = 0.00875 rad/s)
float rawToDPS = 0.00875F;
float dpsToRad = 0.017453293F;
float gyro_zero_offsets[3]      = { 175.0F * rawToDPS * dpsToRad,
                                   -729.0F * rawToDPS * dpsToRad,
                                    101.0F * rawToDPS * dpsToRad };

#endif


void rosLog(char *logMessage)
{
  if (!nh.connected())
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

#ifndef ADAFRUIT_10DOF_IMU
float readImuFloat()
{
  // TODO: Make insensitive to lost bytes....
  floatData fvalue;
  for (int i = 0; i<4; i++) fvalue.data[i] = Serial2.read();
  return fvalue.value;
}
#endif

void ReadAndPublishInertialState(ros::Time currentTime)
{
  inertial.header.stamp = currentTime;
  inertial.header.frame_id = imuLink;

#ifdef ADAFRUIT_10DOF_IMU

  sensors_event_t gyro_event = {0};
  sensors_event_t accel_event = {0};
  sensors_event_t mag_event = {0};

  gyro.getEvent(&gyro_event);
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

  // Apply mag offset compensation (base values in uTesla)
  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // Apply gyro zero-rate error compensation
  float gx = gyro_event.gyro.x - gyro_zero_offsets[0];
  float gy = gyro_event.gyro.y - gyro_zero_offsets[1];
  float gz = gyro_event.gyro.z - gyro_zero_offsets[2];

  // The filter library expects gyro data in degrees/s, but adafruit sensor
  // uses rad/s so we need to convert them first (or adapt the filter lib
  // where they are being converted)
  gx *= 57.2958F;
  gy *= 57.2958F;
  gz *= 57.2958F;

  // Update the filter
  filter.update(gx, gy, gz,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mx, my, mz);

  float qw, qx, qy, qz;
  filter.getQuaternion(&inertial.orientation.w, &inertial.orientation.x, &inertial.orientation.y, &inertial.orientation.z);

  inertial.linear_acceleration.x = accel_event.acceleration.x;
  inertial.linear_acceleration.y = accel_event.acceleration.y;
  inertial.linear_acceleration.z = accel_event.acceleration.z;

  inertial.angular_velocity.x = gx;
  inertial.angular_velocity.y = gy;
  inertial.angular_velocity.z = gz;

#else
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
  #endif

  imuPublisher.publish(&inertial);
}

void reportOdometry(ros::Time currentTime)
{
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

void reportSensors(ros::Time currentTime)
{
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
  nh.getHardware()->setBaud(115200);
  nh.initNode();

    // Initialize ROS pubs and subs
  tfCaster.init(nh);
  nh.advertise(logger);
  nh.advertise(odomPublisher);
  nh.advertise(steeringPublisher);
  nh.advertise(imuPublisher);
  nh.advertise(rangePublisher);
  nh.subscribe(moveCommand);

  while (!nh.connected())
  {
    nh.spinOnce();
    delay(1000);
  }

  nh.loginfo("Spongebot Getting Parameters 2");

  // Wired up an input button from the ProtoShield
  pinMode(buttonPin, INPUT);

  nh.spinOnce();

  // Initialize calibration parameters
  float steeringOffset = 0;
  float steeringSlope = 0;
  float steeringTravel = 0;
  float velocitySlope = 0;
  if (!nh.getParam("~calibrate", &calibrationMode, 1, 10000))
  {
    nh.loginfo("Could not get calibrate");
  }
  
  if (!nh.getParam("~wheelbase", &robotWheelbase))
  {
    nh.loginfo("Could not get wheelbase");
  }
  if (!nh.getParam("~velocitySlope", &velocitySlope))
  {
    nh.loginfo("Could not get velocitySlope");
  }
  if (!nh.getParam("~steeringOffset", &steeringOffset))
  {
    nh.loginfo("Could not get steeringOffset");
  }
  if (!nh.getParam("~steeringSlope", &steeringSlope))
  {
    nh.loginfo("Could not get steeringSlope");
  }
  if (!nh.getParam("~steeringTravel", &steeringTravel))
  {
    nh.loginfo("Could not get steeringTravel");
  }
  
  nh.loginfo("Finished Getting Parameters");

  motorController = new MotorController(motorPin, velocitySlope);

  steering = new ServoSteering(&rosLog, servoPin, steeringSlope, steeringOffset, steeringTravel);

  if (!calibrationMode)
  {
    // Initialize the sonar array
    int sonarPins[3] = { sonar1Pin, sonar2Pin, sonar3Pin };
    ranger = new SonarRanging(&rosLog, sonarEnablePin, sonarPowerPin, NUM_SONARS, sonarPins);
    ranger->Initialize();

#ifdef ADAFRUIT_10DOF_IMU
    nh.loginfo("Initializing Adafruit IMU");
    accel.begin();
    mag.begin();
    gyro.begin();
    filter.begin(25);
    nh.loginfo("Finished initializing Adafruit IMU");

#else
    // Initialize the IMU - Serial2 is pin 16/17
    Serial2.begin(115200);
    nh.loginfo("Started talking to Serial IMU");
#endif
  }

  nh.loginfo("Finished Setup");

  lastTime = nh.now();
}

const uint32_t kOdometryPublishFrequency = 5; //hz
const uint32_t kIMUPublishFrequency = 100; //hz
const uint32_t kSensorPublishFrequency = 5; //hz

const uint32_t kOdometryPublishMilliDelta = 1000 / kOdometryPublishFrequency; // milliseconds
const uint32_t kIMUPublishMilliDelta = 1000 / kIMUPublishFrequency; //milliseconds
const uint32_t kSensorPublishMilliDelta = 1000 / kSensorPublishFrequency; //milliseconds

void loop()
{
  static uint32_t sOdometryLastPublishMilli = 0;
  static uint32_t sIMULastPublishMilli = 0;
  static uint32_t sSensorLastPublishMilli = 0;

  uint32_t milliTime = millis();
  ros::Time currentROSTime = nh.now();
  

  if (milliTime - sOdometryLastPublishMilli >= kOdometryPublishMilliDelta)
  {
    reportOdometry(currentROSTime);
    sOdometryLastPublishMilli = milliTime;
  }

  if (milliTime - sIMULastPublishMilli >= kIMUPublishMilliDelta)
  {
    // TODO: How to feedback state from EKF to update IMU estimates
    ReadAndPublishInertialState(currentROSTime);
    sIMULastPublishMilli = milliTime;
  }

  if (milliTime - sSensorLastPublishMilli >= sSensorLastPublishMilli)
  {
    if (!calibrationMode)
    {
      reportSensors(currentROSTime);
    }
    sSensorLastPublishMilli = milliTime;
  }

  nh.spinOnce();
}
