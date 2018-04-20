#include "Copter.h"
void _Copter::AltitudeControl()
{
  float StickThrust = 0.00000872 * RCsignal.THROTTLE * RCsignal.THROTTLE - 8.804;
  if (flag.mode == Attitude_mode)
  {
    U1 = StickThrust / 1.0;//(IMU.phi_cos * IMU.theta_cos);
  }
  else if ((flag.mode == Altitude_mode) || (flag.mode == loiter_mode))
  {
    U1 = 0;
  }
}
