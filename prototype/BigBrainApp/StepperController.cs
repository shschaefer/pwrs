using System;
using Microsoft.Maker.RemoteWiring;
using Microsoft.Maker.Serial;
using Microsoft.Maker.Firmata;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BigBrain
{
    class StepperController
    {
        //All angles measured as degrees from center, used as 0 degree reference
        const int MAX_RIGHT = 17; //max right
        const int MAX_LEFT = -88; //max left
        const int MAX_ANGLE = (MAX_LEFT - (2 * MAX_LEFT)) + MAX_RIGHT;
        const int MAX_STEPS = 0; //calculate: DEGREES_PER_STEP * MAX_ANGLE
        const double DEGREES_PER_STEP = 1.8; //Written on Stepper label
        const byte RUN_STEPPER = 0x11;
        const byte STEPPER_SETUP = 0x12;

        public bool usbSetup = false;
        bool isSetUp = false;
        UwpFirmata firmata;

        public StepperController(UwpFirmata firmataIn)
        {
            firmata = firmataIn;
        }

        public void rotateToDegree(double degree)
        {
            byte firmataAngle = angleToPos((int)degree);
            rotateTo(firmataAngle);
        }

        //converts angle to rotate to into steps to rotate
        public byte angleToPos(int rotateToAngle)
        {
            byte pos = (byte)((rotateToAngle / DEGREES_PER_STEP) * 1000);
            return pos;
        }

        public void stepperSetup()
        {
            firmata.beginSysex(STEPPER_SETUP);
            firmata.endSysex();
            isSetUp = true;
        }

        //communicates via Firmata with AccelStepper to move stepper motor
        public void rotateTo(byte pos)
        {
            if (!isSetUp)
            {
                firmata.beginSysex(STEPPER_SETUP);
                firmata.endSysex();
                isSetUp = true;
            }
            firmata.beginSysex(RUN_STEPPER);
            firmata.appendSysex(pos);
            firmata.endSysex();
            return;
        }


    }//end class
}//end namespace 

