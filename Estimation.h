#include "Arduino.h"
#define instability_fix 1
#define twoKpDef  (2.0f * 0.5f)
#define twoKiDef  (2.0f * 0.25f)
#define betaDef   0.2f
#define sampleFreq 400
class _EKF
{
  public:
    struct Vector2f {
      float x;
      float y;
    };
    struct Vector3f {
      float x;
      float y;
      float z;
    };
    struct parameters {
      float flow_delay_ms = 5.0f;    ///< optical flow measurement delay relative to the IMU (mSec) - this is to the middle of the optical flow integration interval
      float range_delay_ms = 5.0f;   ///< range finder measurement delay relative to the IMU (mSec)
      float ev_delay_ms = 100.0f;    ///< off-board vision measurement delay relative to the IMU (mSec)
      float auxvel_delay_ms = 0.0f;    ///< auxiliary velocity measurement delay relative to the IMU (mSec)

      // input noise
      float gyro_noise = 1.5e-2f;    ///< IMU angular rate noise used for covariance prediction (rad/sec)
      float accel_noise = 3.5e-1f;   ///< IMU acceleration noise use for covariance prediction (m/sec**2)

      // process noise
      float gyro_bias_p_noise = 1.0e-3f; ///< process noise for IMU rate gyro bias prediction (rad/sec**2)
      float accel_bias_p_noise = 6.0e-3f;  ///< process noise for IMU accelerometer bias prediction (m/sec**3)

      // initialization errors
      float switch_on_gyro_bias = 0.1f;  ///< 1-sigma gyro bias uncertainty at switch on (rad/sec)
      float switch_on_accel_bias = 0.2f; ///< 1-sigma accelerometer bias uncertainty at switch on (m/sec**2)
      float initial_tilt_err = 0.1f;   ///< 1-sigma tilt error after initial alignment using gravity vector (rad)

      // range finder fusion
      float range_noise = 0.1f;    ///< observation noise for range finder measurements (m)
      float range_innov_gate = 5.0f;   ///< range finder fusion innovation consistency gate size (STD)
      float rng_gnd_clearance = 0.1f;    ///< minimum valid value for range when on ground (m)
      float rng_sens_pitch = 0.0f;   ///< Pitch offset of the range sensor (rad). Sensor points out along Z axis when offset is zero. Positive rotation is RH about Y axis.
      float range_noise_scaler = 0.0f;   ///< scaling from range measurement to noise (m/m)
      float vehicle_variance_scaler = 0.0f;  ///< gain applied to vehicle height variance used in calculation of height above ground observation variance
      float max_hagl_for_range_aid = 5.0f; ///< maximum height above ground for which we allow to use the range finder as height source (if range_aid == 1)
      float max_vel_for_range_aid = 1.0f;  ///< maximum ground velocity for which we allow to use the range finder as height source (if range_aid == 1)
      int32_t range_aid = 0;     ///< allow switching primary height source to range finder if certian conditions are met
      float range_aid_innov_gate = 1.0f;   ///< gate size used for innovation consistency checks for range aid fusion
      float range_cos_max_tilt = 0.7071f;  ///< cosine of the maximum tilt angle from the vertical that permits use of range finder data

      // vision position fusion for vicon or otus tracker
      float ev_innov_gate = 5.0f;    ///< vision estimator fusion innovation consistency gate size (STD)

      // optical flow fusion
      float flow_noise = 0.15f;    ///< observation noise for optical flow LOS rate measurements (rad/sec)
      float flow_noise_qual_min = 0.5f;  ///< observation noise for optical flow LOS rate measurements when flow sensor quality is at the minimum useable (rad/sec)
      int32_t flow_qual_min = 1;   ///< minimum acceptable quality integer from  the flow sensor
      float flow_innov_gate = 3.0f;    ///< optical flow fusion innovation consistency gate size (STD)
      float flow_rate_max = 2.5f;    ///< maximum valid optical flow rate (rad/sec)

      // XYZ offset of sensors in body axes (m)
      Vector3f imu_pos_body;      ///< xyz position of IMU in body frame (m)
      Vector3f rng_pos_body;      ///< xyz position of range sensor in body frame (m)
      Vector3f flow_pos_body;     ///< xyz position of range sensor focal point in body frame (m)
      Vector3f ev_pos_body;     ///< xyz position of VI-sensor focal point in body frame (m)

      // output complementary filter tuning
      float vel_Tau = 0.25f;     ///< velocity state correction time constant (1/sec)
      float pos_Tau = 0.25f;     ///< postion state correction time constant (1/sec)

      // accel bias learning control
      float acc_bias_lim = 0.4f;   ///< maximum accel bias magnitude (m/sec**2)
      float acc_bias_learn_acc_lim = 25.0f;  ///< learning is disabled if the magnitude of the IMU acceleration vector is greater than this (m/sec**2)
      float acc_bias_learn_gyr_lim = 3.0f; ///< learning is disabled if the magnitude of the IMU angular rate vector is greater than this (rad/sec)
      float acc_bias_learn_tc = 0.5f;    ///< time constant used to control the decaying envelope filters applied to the accel and gyro magnitudes (sec)

      // multi-rotor drag specific force fusion
      float drag_noise = 2.5f;     ///< observation noise variance for drag specific force measurements (m/sec**2)**2
      float bcoef_x = 25.0f;     ///< ballistic coefficient along the X-axis (kg/m**2)
      float bcoef_y = 25.0f;     ///< ballistic coefficient along the Y-axis (kg/m**2)

      // control of accel error detection and mitigation (IMU clipping)
      float vert_innov_test_lim = 4.5f;  ///< Number of standard deviations allowed before the combined vertical velocity and position test is declared as failed
      int bad_acc_reset_delay_us = 500000; ///< Continuous time that the vertical position and velocity innovation test must fail before the states are reset (uSec)
    };
  private:
};


