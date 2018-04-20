#include "Copter.h"
void _Copter::Copter_Init()
{
  Motor_init();
  InitSensor();
  AHRS_filter_init();
  InitControl();
}
void _Copter::Copter_Check()
{
  Loop_Check();
}

