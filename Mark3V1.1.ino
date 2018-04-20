#include "Copter.h"
#include "Comm.h"
_Copter Copter;
void setup()
{
  InitComm(); //Teensy 3.2
  Copter.Copter_Init();
}
void loop()
{
  /*-------------------400Hz data collection---------------------*/
  Serial.print(Copter.gy89.gyro.origin.x); Serial.print('\t');
  Serial.println();
  /*-------------------------------------------------------*/
  RC_refine();
  Copter.AHRS();
  Copter.AttitudeControl();
  Copter.AltitudeControl();
  Copter.InputTransform();
  Copter.command_Comm();
  Copter.Copter_Check();
}

