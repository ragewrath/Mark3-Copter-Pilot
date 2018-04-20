#include "Copter.h"
void _Copter::InitControl()
{
  pvel.Kp = (EEPROM.read(11) << 8) | EEPROM.read(10); pvel.Kp /= 10;
  pvel.Ki = (EEPROM.read(13) << 8) | EEPROM.read(12); pvel.Ki /= 10;
  pvel.Kd = (EEPROM.read(15) << 8) | EEPROM.read(14); pvel.Kd /= 10;
  phiang.Kp = (EEPROM.read(17) << 8) | EEPROM.read(16); phiang.Kp /= 100;

  qvel.Kp = (EEPROM.read(19) << 8) | EEPROM.read(18); qvel.Kp /= 10;
  qvel.Ki = (EEPROM.read(21) << 8) | EEPROM.read(20); qvel.Ki /= 10;
  qvel.Kd = (EEPROM.read(23) << 8) | EEPROM.read(22); qvel.Kd /= 10;
  thetaang.Kp = (EEPROM.read(25) << 8) | EEPROM.read(24); thetaang.Kp /= 100;

  rvel.Kp = (EEPROM.read(27) << 8) | EEPROM.read(26); rvel.Kp /= 10;
  rvel.Ki = (EEPROM.read(29) << 8) | EEPROM.read(28); rvel.Ki /= 10;
  rvel.Kd = (EEPROM.read(31) << 8) | EEPROM.read(30); rvel.Kd /= 10;
  psiang.Kp = (EEPROM.read(33) << 8) | EEPROM.read(32); psiang.Kp /= 100;

  Zalt.Kp = (EEPROM.read(41) << 8) | EEPROM.read(40); Zalt.Kp /= 100;
  Vzalt.Kp = (EEPROM.read(43) << 8) | EEPROM.read(42); Vzalt.Kp /= 100;
  Vzalt.Ki = (EEPROM.read(45) << 8) | EEPROM.read(44); Vzalt.Ki /= 100;
  Vzalt.Kd = (EEPROM.read(47) << 8) | EEPROM.read(46); Vzalt.Kd /= 100;

  Xpos.Kp = (EEPROM.read(51) << 8) | EEPROM.read(50); Xpos.Kp /= 1000;
  Vxpos.Kp = (EEPROM.read(53) << 8) | EEPROM.read(52); Vxpos.Kp /= 1000;
  Vxpos.Ki = (EEPROM.read(55) << 8) | EEPROM.read(54); Vxpos.Ki /= 1000;
  Vxpos.Kd = (EEPROM.read(57) << 8) | EEPROM.read(56); Vxpos.Kd /= 1000;

  Ypos.Kp = (EEPROM.read(61) << 8) | EEPROM.read(60); Ypos.Kp /= 1000;
  Vypos.Kp = (EEPROM.read(63) << 8) | EEPROM.read(62); Vypos.Kp /= 1000;
  Vypos.Ki = (EEPROM.read(65) << 8) | EEPROM.read(64); Vypos.Ki /= 1000;
  Vypos.Kd = (EEPROM.read(67) << 8) | EEPROM.read(66); Vypos.Kd /= 1000;
  
  EstimatedG = (EEPROM.read(71) << 8) | EEPROM.read(70); EstimatedG /= 1000;
}
void _Copter::Loop_Check()
{
  Battery_Check();
  Timer_Check();
}
void _Copter::Battery_Check()
{
  voltage = (float)analogRead(A14) * 0.020462;
  voltageavg = voltage * 0.08 + voltageavg * 0.92;
  if (voltageavg < 10.8)
    battery_warning = 1;
}
void _Copter::Timer_Check()
{
  if (gltimer == 200)
  {
    if (glch > 0)
      digitalWrite(LED2, LOW);
    else
      digitalWrite(LED2, HIGH);
    glch *= -1;
    gltimer = 0;
  }
  else
    gltimer++;
  if (micros() - whole_timer > MainLoopPeriod)
    time_out = 1;
  while (micros() - whole_timer < MainLoopPeriod);
  whole_timer = micros();
}

float _Copter::Rad(float angle)
{
  return (angle * M_PI / 180.0);
}
float _Copter::Degree(float rad)
{
  return (rad / M_PI * 180.0);
}
float _Copter::data_limitation(float a, float b, float c)
{
  if (a < b) a = b;
  if (a > c) a = c;
  return a;
}

float _Copter::invSqrt(float number) {
  long i;
  float x2, y;
  const float threehalfs = 1.5F;

  x2 = number * 0.5F;
  y  = number;
  i  = * ( long * ) &y;
  i  = 0x5f3759df - ( i >> 1 );
  y  = * ( float * ) &i;
  y  = y * ( threehalfs - ( x2 * y * y ) );
  return y;
}
