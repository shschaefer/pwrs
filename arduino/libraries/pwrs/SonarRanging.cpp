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

#include "SonarRanging.h"

SonarRanging::SonarRanging(void (*logFunction)(String), byte enablePin, byte powerPin,  
	int numSensors, int *sensorPins)
  : logFunc(logFunction), enable(enablePin), power(powerPin), sonars(numSensors)
{
  pinMode(enable, INPUT);
  pinMode(power, OUTPUT);
  digitalWrite(power, LOW);
  
  sonarPins = new int[sonars];
  sonarSamples = new float*[sonars];
  for(int i=0; i<sonars; i++)
  {
	sonarSamples[i] = new float[FILTER_SAMPLES];  
    sonarPins[i] = sensorPins[i];
  }
}

void SonarRanging::Initialize()
{
  // Turn on power and wait at least 250ms for the sonars to quiesce
  digitalWrite(power, HIGH);
  delay(300);

  // Start the group ranging - we assume simultaneous ranging for now  
  pinMode(enable, OUTPUT);
  digitalWrite(enable, HIGH);
  delay(25);
  digitalWrite(enable, LOW);  
  pinMode(enable, INPUT);
}

void SonarRanging::Range()
{
  // TODO: shift, then read
  for(int i=0; i<sonars; i++) sonarSamples[i][0] = analogRead(sonarPins[i]);
}

float SonarRanging::GetDistance(int sensorNumber)
{
  // Filter the samples
  if (sensorNumber > sonars) return 0.0;
  
  return (sonarSamples[sensorNumber - 1][0] / 2.0f) * 2.54f;
}