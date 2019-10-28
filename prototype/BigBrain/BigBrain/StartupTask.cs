/*
    Copyright(c) Microsoft Open Technologies, Inc. All rights reserved.

    The MIT License(MIT)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files(the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions :

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Linq;
using System.Text;
using System.Net.Http;
using Windows.ApplicationModel.Background;
//using Windows.Devices.Pwm;
using Windows.System.Threading;
using Microsoft.Maker.RemoteWiring;
using Microsoft.Maker.Serial;
using Microsoft.Maker.Firmata;
using System.Diagnostics;
using RCVNuget;

// The Background Application template is documented at http://go.microsoft.com/fwlink/?LinkID=533884&clcid=0x409

namespace BigBrain
{

    class CarImplementation : ICarImplementation
    {
        StepperController steering;
        FirmataDriveController drive;

        public CarImplementation(StepperController s, FirmataDriveController d)
        {
            steering = s;
            drive = d;
        }

        public void setSpeed(double throttle)
        {
            drive.setSpeed(throttle);
        }

        public void setSteering(double degree)
        {
            steering.rotateToDegree(degree);
        }
    }

    public sealed class StartupTask : IBackgroundTask
    {
        CarRC carRC = new CarRC();
        BackgroundTaskDeferral deferral;
        StepperController step;
        FirmataDriveController drive;
        //PwmPin motorPin;
        //PwmController pwmController;
        //DriveController drive = new DriveController();

        //member variables
        IStream usb;
        UwpFirmata firmata;
        RemoteDevice arduino;
        bool stepperUsbSetup = false;
        bool driveUsbSetup = false;

        public async void Run(IBackgroundTaskInstance taskInstance)
        {

            usb = new UsbSerial("VID_2341", "PID_0042");//Arduino MEGA

            //construct the firmata client
            firmata = new UwpFirmata();
            arduino = new RemoteDevice(firmata);
            firmata.begin(usb);

            //we do not care about inputs in this example
            //firmata.startListening();

            usb.begin(115200, SerialConfig.SERIAL_8N1);
            arduino.DeviceReady += Stepper_Usb_ConnectionEstablished; //usb.ConnectionEstablished
            arduino.DeviceReady += Drive_Usb_ConnectionEstablished; //usb.ConnectionEstablished
            arduino.DeviceConnectionFailed += Arduino_DeviceConnectionFailed;

            deferral = taskInstance.GetDeferral();
            //pwmController = (await PwmController.GetControllersAsync(PwmPCA9685.PwmProviderPCA9685.GetPwmProvider()))[0];
            //pwmController.SetDesiredFrequency(300); // Check if this is the correct frequency for Talon
            //motorPin = pwmController.OpenPin(13);
            //drive.RampUp(motorPin, .34);
            drive = new FirmataDriveController(firmata);
            step = new StepperController(firmata);

            var carImpl = new CarImplementation(step, drive);
            carRC.implementation = carImpl;


            await Task.Run(async () =>
            {
                while (!stepperUsbSetup && !driveUsbSetup)
                {
                    //wait
                    await Task.Delay(1);
                }

                carRC.Start();
            });
        }

        private void Arduino_DeviceConnectionFailed(string message)
        {
            Debug.WriteLine(message);
        }

        public void Stepper_Usb_ConnectionEstablished() //delegate
        {
            stepperUsbSetup = true;
        }

        public void Drive_Usb_ConnectionEstablished() //delegate
        {
            arduino.pinMode(9, PinMode.SERVO);
            driveUsbSetup = true;
        }

        ~StartupTask()
        {
            //motorPin.Stop();
        }
    }

}
