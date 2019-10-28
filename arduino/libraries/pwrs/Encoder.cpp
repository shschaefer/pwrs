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

#include "Encoder.h"
#include <stdio.h>

// Define the singleton instance variable
Encoder *Encoder::instance;

Encoder::Encoder(byte encoder1Pin, byte encoder2Pin)
{
  // Shitty Arduino can't throw...  So we just accept a new instance...
  instance = this;
  
  // Odometry counter - using external interrupt pins
  // +5V to 220 Ohms to LED positive supply
  // Ground to E/D
  // 10K from +5V to arduino pin, pin to receptor positive supply
  attachInterrupt(digitalPinToInterrupt(encoder1Pin), &Encoder::OdometryCounter, RISING);
}

void Encoder::SetMotorDirection(float direction)
{
  if (direction == 0) 
	motorDirection = 1;
  else
	motorDirection = direction / abs(direction);
}

int Encoder::GetEncoderCount()
{
  // Note that we can lose data here
  noInterrupts();
  int reportValue = encoderCount; // TODO: Average the counts?
  encoderCount = 0;
  interrupts();
  
  return reportValue;
}

void Encoder::OdometryCounter()
{
  instance->encoderCount += instance->motorDirection;
}