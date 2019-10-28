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

#ifndef _STEPPER_STEERING_H
#define _STEPPER_STEERING_H

#include <Arduino.h>
#include <AccelStepper.h>

class StepperSteering
{
  public:
    StepperSteering(void (*logFunction)(String), byte enablePin, byte stepPin, byte directionPin, 
		byte es1Pin, byte es2Pin, float maxSpeed, float defaultSpeed, float defaultAccel, float calibrationOffset);
	void Calibrate();
	void Steer(float steeringAngle, float steeringVelocity);
	
  private:
    int FindEndstop(byte endstopPin, int direction);
 
    AccelStepper *stepper;
    const int stepsPerRevolution = 3200;
    byte endStop1Pin;
    byte endStop2Pin;
    float steeringAnglePhi = 0;
    int travel = 0;
    int speed = 800;
    int accel = 800;
    float offset = 0;
	float maxSteeringAngle = 10.7549; // In degrees, not radians
	void (*logFunc)(String);
};

#endif // _STEPPER_STEERING_H