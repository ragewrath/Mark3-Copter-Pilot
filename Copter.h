#include "System.h"
#include "Estimation.h"
#include "Sensor.h"
#include "Motor.h"
#include "i2c_t3.h"
#include "EEPROM.h"
#include "Arduino.h"
#include <cmath>
#define LED0 13
#define LED1 31
#define LED2 33
#define Attitude_mode 1
#define Altitude_mode 2
#define loiter_mode 3
class _Copter
{
  public:
    /*----------------------------------------*/
    /*Copter.cpp*/
    void Copter_Init();
    void Copter_Check();
    struct _flag
    {
      uint8_t ARMED = 0;
      uint8_t takingoff = 0;
      int8_t  turnoff = 0;
      uint8_t CRASHED = 0;
      uint8_t AltEmergency = 0;
      int8_t calibratedA = 0;
      int8_t calibratedG = 0;
      int8_t calibrationOn = 0;
      uint8_t mode = 0;
      uint8_t momentstart = 0;
      uint8_t LoiterSwitch = 0;
    };
    _flag flag;
    int16_t gltimer = 0, rltimer = 0;
    int8_t glch = 10, rlch = 10;

    /*Motor.cpp*/
    void MotorModel(float omega1, float omega2, float omega3, float omega4);
    void MotorRun();
    void Motor_init();
    void Motor_stop();
    void InputTransform();
    /*Comm.cpp*/
    void command_Comm();
    void Xbee_comm();
    void Xbee_receive();
    struct _RCsignal
    {
      short  ROLL;
      short  PITCH;
      short  THROTTLE;
      short  YAW;
      short  MODE;
      short  SWITCH;
    };
    _RCsignal RCsignal;
    short Xbee_timer = 0;
    short comorder = 0; /*---Command---*/
    short upporder = 0;
    short datalength = 0;

    /*Sensor.cpp*/
    void InitSensor();
    uint8_t I2Cwrite(uint8_t SENSOR_ADDRESS, uint8_t SENSOR_REGISTER, uint8_t SENSOR_VALUE, bool sendStop);
    uint8_t I2CRead(uint8_t SENSOR_ADDRESS, uint8_t SENSOR_REGISTER, uint8_t nbytes);
    void L3GD20read();
    void L3GD20Cali();
    void LSM303Dread();
    void LSM303DCali(uint8_t point);
    void LSM303DPointRead();
    float GyroCollection[3] = {0, 0, 0};
    float gyro_temp;
    short GyroCaliFlag = 0;
    uint8_t i2cData[30];
    uint8_t i2c1Data[30];
    struct _float {
      float x;
      float y;
      float z;
    };
    struct _lint16 {
      short x;
      short y;
      short z;
    };
    struct _trans {
      struct _lint16 origin;
      struct _float filter;
      struct _float histor;
      struct _float aftcal;
      struct _float quietf;
      struct _float radian;
    };
    struct _sensor {
      struct _trans acc;
      struct _trans gyro;
    };
    _sensor gy89;
    struct _Acc_Cali {
      int16_t acc_calitimer = 0;
      int32_t acc_calitmpx;
      int32_t acc_calitmpy;
      int32_t acc_calitmpz;
      int16_t accel_raw_ref[6][3];
      float acc_offset[3];
      float a[3][3];
      float T[3][3];
      float g = 8192; //+-4g
    };
    _Acc_Cali Acc_Cali;

    /*AttitudeEstimator.cpp*/
    struct _LKF
    {
      float phi;
      float theta;
      float psi;
      float phi_rad;
      float theta_rad;
    };
    _LKF LKF;
    
    struct _IMU {
      float phi;
      float theta;
      float psi;
      float phi_rad;
      float theta_rad;
      float psi_rad;
      float phi_sin;
      float theta_sin;
      float psi_sin;
      float phi_cos;
      float theta_cos;
      float psi_cos;
      float p;
      float q;
      float r;
      float p_rad;
      float q_rad;
      float r_rad;
    };
    _IMU IMU;

    float q0 = 1.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;
    float q3old = 0.0f;
    float exInt = 0.0;
    float eyInt = 0.0;
    float ezInt = 0.0;
    float twoKp = twoKpDef;
    float twoKi = twoKiDef;
    float beta = betaDef;
    float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;

    void Madgwick_MARG_Update();
    void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    void AHRS_filter_init();
    void AHRS();
    
    void AHRS_filter();
    void GY89_Dataanl();
    struct _IIR {
      float b0;
      float b1;
      float b2;
      float a1;
      float a2;
      float element0;
      float element1;
      float element2;
    };
    _IIR gyro_IIRx, gyro_IIRy, gyro_IIRz, acc_IIRx, acc_IIRy, acc_IIRz;
    void IIR_set_cutoff_frequency(float sample_freq, float cutoff_freq, struct _IIR *input_IIR);
    float IIR_filter_apply(float cutoff_freq, float sample, struct _IIR *input_IIR);
    double integral_r = 0;

    /*Control part*/
    float U1, U2, U3, U4;
    uint8_t LockYaw = 0;
    struct _Ztransform {
      float Input[5] = {0, 0, 0, 0, 0};
      float Output[5] = {0, 0, 0, 0, 0};
      float Integral;
    };
    _Ztransform Pcon, Qcon, Rcon, Phicon, Thetacon, Psicon, Vzcon, Zcon, Vxpre, Xpre, Vxcon, Xcon, Vypre, Ypre, Vycon, Ycon;
    struct _Target
    {
      float phi;
      float theta;
      float psi;
      float phi_rad;
      float theta_rad;
      float psi_rad;
      float p;
      float q;
      float r;
      float p_rad;
      float q_rad;
      float r_rad;
      int16_t throttle;
      float x;
      float y;
      float z;
      float ve_x; //East-North
      float ve_y;
      float vb_x; //Body
      float vb_y;
      float v_z;
      float vtest_x;
      float vtest_y;
    };
    struct _PIDpram
    {
      float Kp;
      float Ki;
      float Kd;
    };
    _PIDpram pvel, qvel, rvel, phiang, thetaang, psiang, Vzalt, Zalt, Xpos, Ypos, Vxpos, Vypos;
    _Target Target;
    uint8_t Ctrl_timer1 = 0;
    float EstimatedG;
    void AttitudeControl();
    void AltitudeControl();
    void AngularRateControl();

    /*System.cpp*/
    unsigned short time_out = 0;
    unsigned short battery_warning = 0;
    void InitControl();
    void Loop_Check();
    void Timer_Check();
    void Battery_Check();
    float Rad(float angle);
    float Degree(float rad);
    float data_limitation(float a, float b, float c);
    float invSqrt(float number);
    float voltage;
    float voltageavg;
    /*----------------------------------------*/
  private:
    /*----------------------------------------*/
    /*Copter.cpp*/

    /*Motor.cpp*/
    float PWM1, PWM2, PWM3, PWM4;
    float omega12, omega22, omega32, omega42, omega1, omega2, omega3, omega4;
    float pwm_factor = 65535.0 / 2500.0;
    float PWM;
    float InputK1;
    float InputK2;
    float InputK3;
    float InputK4;
    float Jxx = 0.002973, Jyy = 0.003291, Jzz = 0.005501, armlength = 0.142;
    float ThrustCo = 0.000002039, TorqueCo = 0.00000002527;

    /*System.cpp*/
    unsigned long whole_timer;

    /*----------------------------------------*/
};

