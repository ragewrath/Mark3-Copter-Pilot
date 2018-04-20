#include "Copter.h"
void _Copter::Motor_init()
{
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor3, OUTPUT);
  pinMode(motor4, OUTPUT);
  delay(5);
  analogWriteFrequency(motor1, 400);
  analogWriteFrequency(motor2, 400);
  analogWriteFrequency(motor3, 400);
  analogWriteFrequency(motor4, 400);
  analogWriteResolution(16);
  delay(5);
  InputK1 = 122609.122;
  InputK2 = 0.35355339 * Jxx / (4 * armlength * ThrustCo);
  InputK3 = 0.35355339 * Jyy / (4 * armlength * ThrustCo);
  InputK4 = 54422.24;
  Motor_stop();
}
void _Copter::InputTransform()
{
  omega12 = InputK1 * U1 - InputK2 * U2 + InputK3 * U3 - InputK4 * U4;
  omega22 = InputK1 * U1 + InputK2 * U2 + InputK3 * U3 + InputK4 * U4;
  omega32 = InputK1 * U1 + InputK2 * U2 - InputK3 * U3 - InputK4 * U4;
  omega42 = InputK1 * U1 - InputK2 * U2 - InputK3 * U3 + InputK4 * U4;
  if (omega12 < 0) omega12 = 0;
  if (omega22 < 0) omega22 = 0;
  if (omega32 < 0) omega32 = 0;
  if (omega42 < 0) omega42 = 0;
  omega1 = sqrt(omega12);
  omega2 = sqrt(omega22);
  omega3 = sqrt(omega32);
  omega4 = sqrt(omega42);
  MotorModel(omega1, omega2, omega3, omega4);
}
void _Copter::MotorModel(float omega1, float omega2, float omega3, float omega4)
{
  float param_a = 1150.0, param_b = 4668, param_c = -133500, param_d = 59.48, param_e = 926.1;
  PWM1 = (omega1 * omega1 + param_b * omega1 + param_c) / (param_a * voltageavg + param_d) + param_e;
  PWM2 = (omega2 * omega2 + param_b * omega2 + param_c) / (param_a * voltageavg + param_d) + param_e;
  PWM3 = (omega3 * omega3 + param_b * omega3 + param_c) / (param_a * voltageavg + param_d) + param_e;
  PWM4 = (omega4 * omega4 + param_b * omega4 + param_c) / (param_a * voltageavg + param_d) + param_e;
  if (PWM1 < 1050) PWM1 = 1050;
  if (PWM2 < 1050) PWM2 = 1050;
  if (PWM3 < 1050) PWM3 = 1050;
  if (PWM4 < 1050) PWM4 = 1050;
  if (PWM1 > 1550) PWM1 = 1550;
  if (PWM2 > 1550) PWM2 = 1550;
  if (PWM3 > 1550) PWM3 = 1550;
  if (PWM4 > 1550) PWM4 = 1550;
  MotorRun();
}
void _Copter::Motor_stop()
{
  PWM = pwm_factor * 950;
  analogWrite(motor1, PWM);
  analogWrite(motor2, PWM);
  analogWrite(motor3, PWM);
  analogWrite(motor4, PWM);

  
  Pcon.Integral = 0;
  Qcon.Integral = 0;
  Rcon.Integral = 0;
}
void _Copter::MotorRun()
{
  if ((!flag.turnoff) && (flag.ARMED == 2))
  {
    float inputpwm1 = pwm_factor * PWM1;
    float inputpwm2 = pwm_factor * PWM2;
    float inputpwm3 = pwm_factor * PWM3;
    float inputpwm4 = pwm_factor * PWM4;
    analogWrite(motor1, inputpwm1);
    analogWrite(motor2, inputpwm2);
    analogWrite(motor3, inputpwm3);
    analogWrite(motor4, inputpwm4);
  }
  else
    Motor_stop();
}






