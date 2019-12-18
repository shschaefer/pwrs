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

/* Code derived from "MPU9250 Basic Example Code" by Kris Winer (Beerware license) */
/* Leveraging Kris Winer's MPU9250 libraries as adapted by SparkFun for the Arduino */

// We have wired the MPU9250 to an Arduino Pro Mini (3v3) running filtering and integration.
// Then expose a connector which adds two pins to the standard output connector,
// passing serial commands through a Bi-directional level converter to the Mega (5V).
// BLK,  NC, GND, CTS, VCC,  RX,  TX, RST, NC, YYY
// HV,   NC, GND,  NC, 3v3, LV3, LV2,  NC, NC, LV1
// 5VIN, NC, GND,  NC,  NC, HV3, HV2,  NC, NC, HV1

#include "MPU9250.h"

#define TESTING_MODE 0

// Main IMU definitions
int imuInterruptPin = 2;
int powerPin = 10;
bool initialized = false;
MPU9250 myIMU;

// Simplified, Firmata-ish protocol
#define CALIBRATE         0xA0
#define DATA_REQUEST      0x30
#define RESPONSE_END      0xFF
#define MAX_DATA_BYTES 4
byte commandData[MAX_DATA_BYTES];
typedef union {
  float value;
  byte data[4];
} floatData;
int command;
int waiting = 0;

// Testing mode definitions
#if TESTING_MODE == 1
#define AHRS true                 // Set to false for basic data read
int myLed  = 13;                  // Set up pin 13 led for toggling
uint32_t delta_t = 0;              // Used to control display output rate
uint32_t count = 0, sumCount = 0; // used to control display output rate
float sum = 0.0f;
#endif

void setup()
{
  Wire.begin();
  // Wire.setClock(400000L);  // 400 kbit/sec I2C speed
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(115200);

  // Turn on the power indicator LED
  pinMode(powerPin, OUTPUT);
  digitalWrite(powerPin, HIGH);
  
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(imuInterruptPin, INPUT);
  digitalWrite(imuInterruptPin, LOW);

  // Read the WHO_AM_I register, this is a good test of communication
#if TESTING_MODE == 1
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
  
  byte c = myIMU.getDeviceID();
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);
#else
  // When the IMU does not respond, go into listening mode
  // As our state is not initialized, we will return failures to all commands
  if (!myIMU.testConnection()) return;
#endif

#if TESTING_MODE == 1
  // Start by performing self test and reporting values
  float testvals[6];
  myIMU.factorySelfTest(testvals);
  Serial.print("x-axis self test: accelerometer trim within : ");
  Serial.print(testvals[0],1); Serial.println("% of factory value");
  Serial.print("y-axis self test: accelerometer trim within : ");
  Serial.print(testvals[1],1); Serial.println("% of factory value");
  Serial.print("z-axis self test: accelerometer trim within : ");
  Serial.print(testvals[2],1); Serial.println("% of factory value");
  Serial.print("x-axis self test: gyro trim within : ");
  Serial.print(testvals[3],1); Serial.println("% of factory value");
  Serial.print("y-axis self test: gyro trim within : ");
  Serial.print(testvals[4],1); Serial.println("% of factory value");
  Serial.print("z-axis self test: gyro trim within : ");
  Serial.print(testvals[5],1); Serial.println("% of factory value");
  
  // Calibrate gyro and accelerometers, load biases in bias registers
  Serial.println("Calibrating gyro and accel");
  float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};
  myIMU.calibrateGyroAndAccel(gyroBias, accelBias);
  Serial.println("accel biases (mg)"); Serial.println(1000.*accelBias[0]); Serial.println(1000.*accelBias[1]); Serial.println(1000.*accelBias[2]);
  Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
#else
  // TODO: Load calibration from host configuration, not reinitialize???
  myIMU.calibrateGyroAndAccel(NULL,  NULL);
#endif
  
  // Initialize device for active mode read of acclerometer, gyroscope, and
  // temperature
  myIMU.initMPU9250();

  // Initialize device for active mode read of magnetometer
  // Get magnetometer calibration from AK8963 ROM  
  myIMU.initAK8963();

  float mSens[3] = {0, 0, 0}, mBias[3] = {0, 0, 0}, mScale[3] = {0, 0, 0};
#if TESTING_MODE == 1
  Serial.println("Magnetometer Calibration: Wave device in a figure eight until done!");
  delay(2000);
  myIMU.calibrateMagnetometer(mSens, mBias, mScale);

  Serial.println("Magnetomter Calibration values: ");
  Serial.print("X-Axis sensitivity adjustment value ");
  Serial.println(mSens[0], 2);
  Serial.print("Y-Axis sensitivity adjustment value ");
  Serial.println(mSens[1], 2);
  Serial.print("Z-Axis sensitivity adjustment value ");
  Serial.println(mSens[2], 2);
  
  Serial.println("AK8963 mag biases (mG)"); Serial.println(mBias[0]); Serial.println(mBias[1]); Serial.println(mBias[2]); 
  Serial.println("AK8963 mag scale (mG)"); Serial.println(mScale[0]); Serial.println(mScale[1]); Serial.println(mScale[2]); 
  delay(2000); // add delay to see results before serial spew of data
#else
  // User environmental axis correction and scaling in milliGauss
  // From previous calibrations done in testing mode
  // TODO: load from host configuration
  mBias[0] = 294.14;
  mBias[1] = 274.17;
  mBias[2] = -173.37;
  mScale[1] = 0.98;
  mScale[0] = 0.94;
  mScale[2] = 1.09;
  myIMU.setMagnetometerCalibration(mBias, mScale);
#endif

  // Declination adjustment for earth magnetic drift - Sammamish/Seattle
  // TODO: Should we get rid of this and only use the host configuration?
  myIMU.setYawCalibration(15.47f);
  
  initialized = true;
}

void loop()
{ 
  // Run the refresh algorithm
  if (initialized)
    RefreshIMU();
}

void writeFloat(float value)
{
  floatData fvalue;
  fvalue.value = value;
  Serial.write(fvalue.data, 4);
}

void serialEvent() 
{
  while (Serial.available()) 
  {
    int nextByte = Serial.read();
    if (waiting > 0)
      commandData[waiting - 1] = nextByte;
    else
      command = nextByte;
    switch (command) {
    case CALIBRATE:
      if (waiting == MAX_DATA_BYTES)
      {
        // Store a calibration quaternion
        floatData *yaw;
        memcpy(yaw->data, commandData, 4);
        myIMU.setYawCalibration(yaw->value);
      
        // Send the initialization state
        Serial.write((byte)CALIBRATE);
        Serial.write(initialized);
        Serial.write((byte)RESPONSE_END);
        waiting = 0;
      }
      else
        waiting++;
      break;

    case DATA_REQUEST:
      float headingQuaternion[4];
      myIMU.getCurrentOrientation(headingQuaternion);
      Serial.write((byte)DATA_REQUEST);
      for(int i=0; i<4; i++)
      {
        writeFloat(headingQuaternion[i]);
      }
      writeFloat(myIMU.ax);
      writeFloat(myIMU.ay);
      writeFloat(myIMU.az);
      writeFloat(myIMU.gx);
      writeFloat(myIMU.gy);
      writeFloat(myIMU.gz);
      Serial.write((byte)RESPONSE_END);
      command = 0;
      return;

    default:
      // Drop unkonwn packets on the floor and hope to pickup
      // a re-transmit or good command later.
      command = 0;
      waiting = 0;
      break;
    }
  }
}

void RefreshIMU()
{
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.getInterruptStatus() & 0x01)
  {
    myIMU.updateAxes();
#if TESTING_MODE == 1
    sum += myIMU.deltat; // sum for averaging filter update rate
    sumCount++;
  }

  if (!AHRS)
  {
    delta_t = millis() - count;
    if (delta_t > 500)
    {
      // Print acceleration values in milligs!
      Serial.print("X-acceleration: "); Serial.print(1000*myIMU.ax);
      Serial.print(" mg ");
      Serial.print("Y-acceleration: "); Serial.print(1000*myIMU.ay);
      Serial.print(" mg ");
      Serial.print("Z-acceleration: "); Serial.print(1000*myIMU.az);
      Serial.println(" mg ");

      // Print gyro values in degree/sec
      Serial.print("X-gyro rate: "); Serial.print(myIMU.gx, 3);
      Serial.print(" degrees/sec ");
      Serial.print("Y-gyro rate: "); Serial.print(myIMU.gy, 3);
      Serial.print(" degrees/sec ");
      Serial.print("Z-gyro rate: "); Serial.print(myIMU.gz, 3);
      Serial.println(" degrees/sec");

      // Print mag values in degree/sec
      Serial.print("X-mag field: "); Serial.print(myIMU.mx);
      Serial.print(" mG ");
      Serial.print("Y-mag field: "); Serial.print(myIMU.my);
      Serial.print(" mG ");
      Serial.print("Z-mag field: "); Serial.print(myIMU.mz);
      Serial.println(" mG");

      int16_t tempCount = myIMU.readTempData();  // Read the adc values
      // Temperature in degrees Centigrade
      float temperature = ((float) tempCount) / 333.87 + 21.0;
      // Print temperature in degrees Centigrade
      Serial.print("Temperature is ");  Serial.print(temperature, 1);
      Serial.println(" degrees C");

      count = millis();
      digitalWrite(myLed, !digitalRead(myLed));  // toggle led
    } // if (delta_t > 500)
  } // if (!AHRS)
  else
  {
    // Serial print and/or display at 0.5 s rate independent of data rates
    delta_t = millis() - count;

    // update LCD once per half-second independent of read rate
    if (delta_t > 500)
    {
      Serial.print("ax = "); Serial.print((int)1000*myIMU.ax);
      Serial.print(" ay = "); Serial.print((int)1000*myIMU.ay);
      Serial.print(" az = "); Serial.print((int)1000*myIMU.az);
      Serial.println(" mg");

      Serial.print("gx = "); Serial.print( myIMU.gx, 2);
      Serial.print(" gy = "); Serial.print( myIMU.gy, 2);
      Serial.print(" gz = "); Serial.print( myIMU.gz, 2);
      Serial.println(" deg/s");

      Serial.print("mx = "); Serial.print( (int)myIMU.mx );
      Serial.print(" my = "); Serial.print( (int)myIMU.my );
      Serial.print(" mz = "); Serial.print( (int)myIMU.mz );
      Serial.println(" mG");

      float headingQuaternion[4];
      myIMU.getCurrentOrientation(headingQuaternion);
      Serial.print("q0 = "); Serial.print(headingQuaternion[0]);
      Serial.print(" qx = "); Serial.print(headingQuaternion[1]);
      Serial.print(" qy = "); Serial.print(headingQuaternion[2]);
      Serial.print(" qz = "); Serial.println(headingQuaternion[3]);

      float y, p, r;
      myIMU.getCurrentYawPitchRoll(&y, &p, &r);
      Serial.print("Yaw, Pitch, Roll: ");
      Serial.print(y, 2);
      Serial.print(", ");
      Serial.print(p, 2);
      Serial.print(", ");
      Serial.println(r, 2);

      Serial.print("rate = ");
      Serial.print((float)sumCount/sum, 2);
      Serial.println(" Hz");

      count = millis();
      sumCount = 0;
      sum = 0;
    } // if (delta_t > 500)
  } // if (AHRS)
#else
  }
#endif
}
