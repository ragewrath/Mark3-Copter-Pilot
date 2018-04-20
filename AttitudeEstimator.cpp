#include "Copter.h"
void _Copter::AHRS()
{
  AHRS_filter();        //Digital Filter
  //integral_r += gy89.gyro.filter.z * 0.0025;
  //Madgwick_MARG_Update();
}
void _Copter::AHRS_filter()
{
  GY89_Dataanl();
  gy89.gyro.filter.x = IIR_filter_apply(L3GD20_DEFAULT_FILTER_FREQ, gy89.gyro.aftcal.x, &gyro_IIRx);
  gy89.gyro.filter.y = IIR_filter_apply(L3GD20_DEFAULT_FILTER_FREQ, gy89.gyro.aftcal.y, &gyro_IIRy);
  gy89.gyro.filter.z = IIR_filter_apply(L3GD20_DEFAULT_FILTER_FREQ, gy89.gyro.aftcal.z, &gyro_IIRz);

  gy89.acc.filter.x = IIR_filter_apply(LSM303D_ACCEL_DEFAULT_DRIVER_FILTER_FREQ, gy89.acc.aftcal.x, &acc_IIRx);
  gy89.acc.filter.y = IIR_filter_apply(LSM303D_ACCEL_DEFAULT_DRIVER_FILTER_FREQ, gy89.acc.aftcal.y, &acc_IIRy);
  gy89.acc.filter.z = IIR_filter_apply(LSM303D_ACCEL_DEFAULT_DRIVER_FILTER_FREQ, gy89.acc.aftcal.z, &acc_IIRz);
}
void _Copter::GY89_Dataanl()
{
  L3GD20read();
  LSM303Dread();
}
void _Copter::AHRS_filter_init()
{
  IIR_set_cutoff_frequency(L3GD20_DEFAULT_RATE, L3GD20_DEFAULT_FILTER_FREQ, &gyro_IIRx);
  IIR_set_cutoff_frequency(L3GD20_DEFAULT_RATE, L3GD20_DEFAULT_FILTER_FREQ, &gyro_IIRy);
  IIR_set_cutoff_frequency(L3GD20_DEFAULT_RATE, L3GD20_DEFAULT_FILTER_FREQ, &gyro_IIRz);

  IIR_set_cutoff_frequency(LSM303D_ACCEL_DEFAULT_RATE, LSM303D_ACCEL_DEFAULT_DRIVER_FILTER_FREQ, &acc_IIRx);
  IIR_set_cutoff_frequency(LSM303D_ACCEL_DEFAULT_RATE, LSM303D_ACCEL_DEFAULT_DRIVER_FILTER_FREQ, &acc_IIRy);
  IIR_set_cutoff_frequency(LSM303D_ACCEL_DEFAULT_RATE, LSM303D_ACCEL_DEFAULT_DRIVER_FILTER_FREQ, &acc_IIRz);
}
void _Copter::IIR_set_cutoff_frequency(float sample_freq, float cutoff_freq, struct _IIR *input_IIR)
{
  if (cutoff_freq <= 0.0f) {
    // no filtering
    return;
  }
  float fr = sample_freq / cutoff_freq;
  float ohm = tanf(M_PI_F / fr);
  float c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;
  input_IIR->b0 = ohm * ohm / c;
  input_IIR->b1 = 2.0f * input_IIR->b0;
  input_IIR->b2 = input_IIR->b0;
  input_IIR->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
  input_IIR->a2 = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
}
float _Copter:: IIR_filter_apply(float cutoff_freq, float sample, struct _IIR *input_IIR)
{
  if (cutoff_freq <= 0.0f) {
    // no filtering
    return sample;
  }
  // do the filtering
  input_IIR->element0 = sample - input_IIR->element1 * input_IIR->a1 - input_IIR->element2 * input_IIR->a2;
  float output = input_IIR->element0 * input_IIR->b0 + input_IIR->element1 * input_IIR->b1 + input_IIR->element2 * input_IIR->b2;

  input_IIR->element2 = input_IIR->element1;
  input_IIR->element1 = input_IIR->element0;

  // return the value.  Should be no need to check limits
  return output;
}

/*---------------------------------------------------------------------------Linear Kalman Filter-----------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------Gradient Descent-----------------------------------------------------------------------------*/
void _Copter::Madgwick_MARG_Update()
{
  float q[4], ypr[3], gx, gy, gz, AHRS_val[6];
  AHRS_val[0] = gy89.acc.filter.y / 8192.0;
  AHRS_val[1] = gy89.acc.filter.x / 8192.0;
  AHRS_val[2] = gy89.acc.filter.z / 8192.0;
  AHRS_val[3] = gy89.gyro.filter.x * -1.0;
  AHRS_val[4] = gy89.gyro.filter.y;
  AHRS_val[5] = gy89.gyro.filter.z * -1.0;

  MahonyAHRSupdateIMU(AHRS_val[3] * M_PI_F / 180, AHRS_val[4] * M_PI_F / 180, AHRS_val[5] * M_PI_F / 180, AHRS_val[0], AHRS_val[1], AHRS_val[2]);

  q[0] = q0;
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;

  gx = 2 * (q[1] * q[3] - q[0] * q[2]);
  gy = 2 * (q[0] * q[1] + q[2] * q[3]);
  gz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

  ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1);
  ypr[1] = atan(gx / sqrt(gy * gy + gz * gz));
  ypr[2] = atan(gy / sqrt(gx * gx + gz * gz));

  IMU.p = gy89.gyro.filter.x;
  IMU.q = -gy89.gyro.filter.y;
  IMU.r = -gy89.gyro.filter.z;
  IMU.p_rad = Rad(IMU.p);
  IMU.q_rad = Rad(IMU.q);
  IMU.r_rad = Rad(IMU.r);

  IMU.phi_rad = -ypr[2];
  IMU.theta_rad = ypr[1];
  IMU.psi_rad = -ypr[0];
  IMU.phi = Degree( -ypr[2]);
  IMU.theta = Degree( ypr[1]);
  IMU.psi = Degree( -ypr[0]);

  IMU.phi_sin = sin(IMU.phi_rad);
  IMU.theta_sin = sin(IMU.theta_rad);
  IMU.psi_sin = sin(IMU.psi_rad);
  IMU.phi_cos = cos(IMU.phi_rad);
  IMU.theta_cos = cos(IMU.theta_rad);
  IMU.psi_cos = cos(IMU.psi_rad);
}
void _Copter::MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 , _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * (1.0f / sampleFreq);
  q1 += qDot2 * (1.0f / sampleFreq);
  q2 += qDot3 * (1.0f / sampleFreq);
  q3 += qDot4 * (1.0f / sampleFreq);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
} 
/*---------------------------------------------------------------------------Complimentary Filter-----------------------------------------------------------------------------*/
void _Copter::MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if (twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  
}

