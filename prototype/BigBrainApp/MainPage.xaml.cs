using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;
using Microsoft.Maker.RemoteWiring;
using Microsoft.Maker.Serial;
using Microsoft.Maker.Firmata;
using System.Diagnostics;
using System.Threading.Tasks;
using RCVNuget;

// The Blank Page item template is documented at http://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x409

namespace BigBrain
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {
        public MainPage()
        {
            this.InitializeComponent();
        }

        CarRC carRC = new CarRC();
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

        protected override async void OnNavigatedTo(NavigationEventArgs e)
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

            //pwmController = (await PwmController.GetControllersAsync(PwmPCA9685.PwmProviderPCA9685.GetPwmProvider()))[0];
            //pwmController.SetDesiredFrequency(300); // Check if this is the correct frequency for Talon
            //motorPin = pwmController.OpenPin(13);
            //drive.RampUp(motorPin, .34);
            drive = new FirmataDriveController(firmata);
            step = new StepperController(firmata);

            var carImpl = new CarImplementation(step, drive);
            carRC.implementation = carImpl;


            while (!stepperUsbSetup && !driveUsbSetup)
            {
                //wait
                await Task.Delay(1);
            }

            carRC.Start();
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
    }
}
