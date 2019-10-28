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

#include "StepperSteering.h"

StepperSteering::StepperSteering(void (*logFunction)(String), byte enablePin, byte stepPin, byte directionPin, 
	byte es1Pin, byte es2Pin, float maxSpeed, float defaultSpeed, float defaultAccel, float calibrationOffset) :
	logFunc(logFunction),
	endStop1Pin(es1Pin),
	endStop2Pin(es2Pin),
	offset(calibrationOffset)
{
  stepper = new AccelStepper(AccelStepper::DRIVER, stepPin, directionPin);
  stepper->setEnablePin(enablePin);

  pinMode(endStop1Pin, INPUT_PULLUP);
  pinMode(endStop2Pin, INPUT_PULLUP);
  
  speed = (defaultSpeed / (2 * M_PI)) * stepsPerRevolution;
  accel = (defaultAccel / (2 * M_PI)) * stepsPerRevolution;
 
  stepper->setMaxSpeed((maxSpeed / (2 * M_PI)) * stepsPerRevolution);
  stepper->setCurrentPosition(0);
}

void StepperSteering::Steer(float steeringAngle, float steeringVelocity)
{
  // Turns are in absolute radians.  Ignore the velocity/acceleration for now.
  // TODO: Clamp the steering to min/max
  int newPosition = steeringAngle * travel / M_PI;
  if (steeringAngle == steeringAnglePhi)
  {
    return;
  }
  
  // digitalWrite(enablePin, LOW);
  stepper->setSpeed(speed);
  stepper->setAcceleration(accel);

  stepper->runToNewPosition(newPosition);
	
  // digitalWrite(enablePin, HIGH);

  // TODO: Update the current steering angle during the turn
  steeringAnglePhi = steeringAngle;
}

int StepperSteering::FindEndstop(byte endstopPin, int direction)
{
  bool switchOn = false;
  int movePos = stepsPerRevolution * direction;

  // Enable stepper motion
  // digitalWrite(enablePin, LOW);

  // Set target position
  stepper->moveTo(movePos);

  // Move to target or endstop
  while(!switchOn) {
    if(digitalRead(endstopPin) == HIGH) 
    { 
      switchOn = true; 
    }
    else { stepper->run(); }
  }
  stepper->stop();

  // Disable stepper motion
  // digitalWrite(enablePin, HIGH);

  return stepper->currentPosition();
}

// A NEMA-23 stepper is only 200 steps per revolution.  Big Easy Driver is 16 micro-steps = 3200 steps/rev.
// 360 deg / 3200 steps = .1125 degrees per step (0.0019635 radians)
// We are getting around 170-200 steps from current setup
void StepperSteering::Calibrate()
{
  int maxPos = 1;
  int minPos = -1;

  logFunc(F("Calibrating steering... "));

  stepper->setMaxSpeed(50);
  stepper->setSpeed(25);
  stepper->setAcceleration(50);
  stepper->setCurrentPosition(0);
  
  logFunc(F("FIND SWITCH 1"));
  minPos = FindEndstop(endStop2Pin, minPos);
  String msgString = F("SWITCH FOUND: ");
  msgString += minPos;
  logFunc(msgString);
  
  stepper->setCurrentPosition(0);

  logFunc(F("FIND SWITCH 2"));
  maxPos = FindEndstop(endStop1Pin, maxPos);
  msgString = F("SWITCH FOUND: ");
  msgString += maxPos;
  logFunc(msgString);

  travel = (maxPos)/2;
  msgString = F("Center Position: ");
  msgString += travel;
  logFunc(msgString);

  logFunc(F("CENTERING"));
  stepper->runToNewPosition(travel);
  
  stepper->setCurrentPosition(0);
}