#include "Copter.h"
void _Copter::AttitudeControl()
{
  Ctrl_timer1++;
  if (Ctrl_timer1 >= 4)
  {
    Ctrl_timer1 = 0;
    /*Receive Command*/
    if (flag.mode == Attitude_mode || flag.mode == Altitude_mode)
    {
      if ((RCsignal.ROLL < 1485) || (RCsignal.ROLL > 1515))
        Target.phi = (float)(RCsignal.ROLL - 1500) / 20;
      else
        Target.phi = 0;
      Target.phi = data_limitation(Target.phi, -25, 25);
      Target.phi_rad = Rad(Target.phi);
      if ((RCsignal.PITCH < 1485) || (RCsignal.PITCH > 1515))
        Target.theta = (float)(RCsignal.PITCH - 1500) / 20;
      else
        Target.theta = 0;
      Target.theta = data_limitation(Target.theta, -25, 25);
      Target.theta_rad = Rad(Target.theta);
    }
    if (RCsignal.THROTTLE > 1100 )
    {
      if (LockYaw != 1)
      {
        LockYaw = 1;
        Target.psi = IMU.psi;
      }
    }
    else {
      LockYaw = 0;
      Target.psi = IMU.psi;
    }
    if ((RCsignal.YAW > 1515) || (RCsignal.YAW < 1485))
    {
      Target.psi += ((RCsignal.YAW - 1500) / 300.0f);
      if (Target.psi > 180.0f) Target.psi -= 360.0f;
      else if (Target.psi < -180.0f)Target.psi += 360.0f;
    }
    Target.psi_rad = Rad(Target.psi);
    /*---------------------------------------------P Control---------------------------------------------*/
    //phi
    Phicon.Input[0] = Target.phi - IMU.phi_rad;
    Phicon.Output[0] = phiang.Kp * Phicon.Input[0];

    //theta
    Thetacon.Input[0] = Target.theta - IMU.theta_rad;
    Thetacon.Output[0] = thetaang.Kp * Thetacon.Input[0];

    //psi
    if ((Target.psi - IMU.psi_rad) >= M_PI || (Target.psi - IMU.psi_rad) < - M_PI)
    {
      if (Target.psi > 0 && IMU.psi_rad < 0)  Psicon.Input[0] = (-M_PI - IMU.psi_rad) + (Target.psi - M_PI);
      if (Target.psi < 0 && IMU.psi_rad > 0)  Psicon.Input[0] = (M_PI - IMU.psi_rad) + (Target.psi + M_PI);
    }
    else  Psicon.Input[0] = Target.psi - IMU.psi_rad;

    Psicon.Integral += Psicon.Input[0];
    Psicon.Output[0] = psiang.Kp * Psicon.Input[0];
    /*---------------------------------------------------------------------------------------------------*/
  }
  AngularRateControl();
}
void _Copter::AngularRateControl()
{
  //p
  Target.p_rad = Phicon.Output[0];
  Target.p = Degree(Target.p_rad);
  //q
  Target.q_rad = Thetacon.Output[0];
  Target.q = Degree(Target.q_rad);
  //r
  Target.r_rad = Psicon.Output[0];
  Target.r = Degree(Target.r_rad);

  /*---------------------------------------------Angular Rate Control---------------------------------------------*/
  //p
  Pcon.Input[0] = Target.p_rad - IMU.p_rad;
  Pcon.Integral += Pcon.Input[0];
  Pcon.Integral = data_limitation(Pcon.Integral, -70, 70);
  Pcon.Output[0] = Pcon.Input[0] * pvel.Kp + Pcon.Integral * pvel.Ki * innerT + (Pcon.Input[0] - Pcon.Input[1]) * pvel.Kd / innerT;
  Pcon.Output[1] = Pcon.Output[0];
  Pcon.Input[1] = Pcon.Input[0];
  //q
  Qcon.Input[0] = Target.q_rad - IMU.q_rad;
  Qcon.Integral += Qcon.Input[0];
  Qcon.Integral = data_limitation(Qcon.Integral, -70, 70);
  Qcon.Output[0] = Qcon.Input[0] * qvel.Kp + Qcon.Integral * qvel.Ki * innerT + (Qcon.Input[0] - Qcon.Input[1]) * qvel.Kd / innerT;
  Qcon.Output[1] = Qcon.Output[0];
  Qcon.Input[1] = Qcon.Input[0];
  //r
  Rcon.Input[0] = Target.r_rad - IMU.r_rad;
  Rcon.Integral += Rcon.Input[0];
  Rcon.Integral = data_limitation(Rcon.Integral, -30, 30);
  Rcon.Output[0] = Rcon.Input[0] * rvel.Kp + Rcon.Integral * rvel.Ki * innerT + (Rcon.Input[0] - Rcon.Input[1]) * rvel.Kd / innerT;
  Rcon.Output[1] = Rcon.Output[0];
  Rcon.Input[1] = Rcon.Input[0];
  /*----------------------------------------------------------------------------------------------------------------*/
  U2 = Pcon.Output[0];
  U2 = data_limitation(U2, -500, 500);
  U3 = Qcon.Output[0];
  U3 = data_limitation(U3, -500, 500);
  U4 = Rcon.Output[0];
  U4 = data_limitation(U4, -200, 200);

  //U2 = 0;
  // U3 = 0;
  // U4 = 0;
}
