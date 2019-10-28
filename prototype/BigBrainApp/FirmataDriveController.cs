using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Diagnostics;
using Microsoft.Maker.Firmata;

namespace BigBrain
{
    class FirmataDriveController
    {
        const int PWM_FORWARDS = 128;    // Max forward starting velocity which does not stall
        const int PWM_BACKWARDS = 102;   // Max backward starting velocity which does not stall
        const int BIG_INCREMENT = 13;
        const int SMALL_INCREMENT = 1;

        const byte pwmPin = 9;
        UwpFirmata firmata;

        public FirmataDriveController(UwpFirmata firmataIn)
        {
            firmata = firmataIn;
        }

        public void setSpeed(double speedPercent)
        {
            if (speedPercent < 0 - double.Epsilon)
            {
                firmata.sendAnalog(pwmPin, PWM_BACKWARDS);
            }
            else if (speedPercent > 0 + double.Epsilon)
            {
                firmata.sendAnalog(pwmPin, PWM_FORWARDS);
            }
            else
            {
                firmata.sendAnalog(pwmPin, 0);
            }
        }

        // Accelerates to intended velocity gradually, preventing motor from stalling
        public async void FirmataRampUp(ushort velocity, int duration)  // USER INPUT: speed in PWM range (0 - 255), duration in seconds
                                                                        // Duration of 0 inplies that the car will run continuously 
        {
            Stopwatch timer = new Stopwatch();
            duration = duration + 3;
            ushort i;
            //firmata.sendAnalog(pwmPin, velocity);
            //await Task.Delay(3000);
            //firmata.sendAnalog(pwmPin, 0);
            //await Task.Delay(3000);

            // Moving car forwards
            if (velocity > PWM_FORWARDS)
            {
                i = PWM_FORWARDS;   // Sets car to maximum starting forward speed

                while (i < velocity)
                {
                    firmata.sendAnalog(pwmPin, i);
                    await Task.Delay(500);

                    // If close to intended velocity, increment up by 1
                    if ((velocity - i) < BIG_INCREMENT)
                    {
                        i += SMALL_INCREMENT;
                    }

                    // If far from intended velocity, increment up by 5%
                    else
                    {
                        i += BIG_INCREMENT;
                    }
                }

                // If at intended velocity, maintain velocity and continue running for user input duration
                if (i == velocity)
                {
                    timer.Start();
                    while (timer.Elapsed < TimeSpan.FromSeconds(duration))
                    {
                        firmata.sendAnalog(pwmPin, i);
                        Debug.WriteLine(timer.Elapsed);
                    }
                    timer.Stop();
                    firmata.sendAnalog(pwmPin, 0);
                    return;
                    //firmata.sendAnalog(pwmPin, i);
                    //if (duration != 0)
                    //{
                    //    await Task.Delay(duration);
                    //    firmata.sendAnalog(pwmPin, 102);
                    //}

                }
            }

            // Moving car backwards
            else if (velocity < PWM_BACKWARDS)
            {
                i = PWM_BACKWARDS;  // Sets car to maximum starting backwards speed
                while (i > velocity)
                {
                    firmata.sendAnalog(pwmPin, i);
                    await Task.Delay(500);

                    // If close to intended velocity, increment down by 1%
                    if ((i - velocity) < BIG_INCREMENT)
                    {
                        i -= SMALL_INCREMENT;
                    }

                    // If far from intended velocity, increment down by 5%
                    else
                    {
                        i -= BIG_INCREMENT;
                    }
                }

                // If at intended velocity, maintain velocity and continue running
                if (i == (velocity))
                {
                    timer.Start();
                    while (timer.Elapsed < TimeSpan.FromSeconds(duration))
                    {
                        firmata.sendAnalog(pwmPin, 0);
                        Debug.WriteLine(timer.Elapsed);
                    }
                    timer.Stop();
                    firmata.sendAnalog(pwmPin, 0);
                    return;
                    //firmata.sendAnalog(pwmPin, i);
                    //if (duration != 0)
                    //{
                    //    await Task.Delay(duration);
                    //    return;
                    //}
                }
            }

            // If the intended velocity is between 40% and 50% inclusive, the motor does not need to be revved up and can be directly set
            else
            {
                timer.Start();
                while (timer.Elapsed < TimeSpan.FromSeconds(duration))
                {
                    firmata.sendAnalog(pwmPin, velocity);
                    Debug.WriteLine(timer.Elapsed);
                }
                timer.Stop();
                firmata.sendAnalog(pwmPin, 0);
                return;
            }
        }
    }
}