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

#include "ServoSteering.h"

ServoSteering::ServoSteering(void (*logFunction)(String), int servoPin,
        float slope, int calibrationOffset, float degreesMaxTravel) : 
	logFunc(logFunction),
  steeringSlope(slope),
	offset(calibrationOffset)
{
  maxSteeringAngle = degreesMaxTravel / 2.0;
  minSteeringAngle = -1.0 * maxSteeringAngle;

  servo = new Servo();
  servo->attach(servoPin);
  servo->writeMicroseconds(offset);
}

void ServoSteering::Steer(float steeringAngle, float steeringVelocity)
{
  // Turns are in absolute radians.  Ignore the velocity/acceleration for now.
  if (steeringAngle == steeringAnglePhi)
  {
    return;
  }

  // Clamp the steering to min/max
  if (steeringAngle > maxSteeringAngle){
    steeringAngle = maxSteeringAngle;
  } else if (steeringAngle < minSteeringAngle) {
    steeringAngle = minSteeringAngle;
  }

  int newPosition = int(steeringAngle * steeringSlope) + offset;
  servo->writeMicroseconds(newPosition);

  // TODO: Update the current steering angle during the turn, control the profile
  steeringAnglePhi = steeringAngle;
}

float ServoSteering::GetCurrentAngle()
{
  return steeringAnglePhi;
}