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

#include "MotorController.h"

MotorController::MotorController(byte motorPin, float slope) :
	velocitySlope(slope),
	currentSpeed(0.0)
{
  // PWM Motor Driver - Talon SRX
  rwd.attach(motorPin);  
}

void MotorController::SetMotorSpeed(float speed, float acceleration, float jerk)
{
  // Velocity in m/s.
  // Convert to PWM duty cycle percentage -> vel = slope * duty_cycle (require zero crossing)
  // TODO: Ignore acceleration and jerk parameters for now.
  float dutyCycle = speed / velocitySlope;
  // TODO: Should this be clamped to [-1.0, 1.0]???
  currentSpeed = speed;
  
  // TODO: Consider braking vs. coasting scenarios
  int PWMvalue = dutyCycle * 500 + 1500; //scale up to 1000-2000  
  rwd.writeMicroseconds(PWMvalue);
}

float MotorController::GetCurrentSpeed()
{
  return currentSpeed;	
}