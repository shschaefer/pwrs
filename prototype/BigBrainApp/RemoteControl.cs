using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Windows.System.Threading;
using Microsoft.Maker.RemoteWiring;
using Microsoft.Maker.Serial;
using Microsoft.Maker.Firmata;
using System.Diagnostics;
using RCVNuget;

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


}
