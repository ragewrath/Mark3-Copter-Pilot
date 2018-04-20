#include "Copter.h"
/*----------------------------------------------------old serial communication protocol------------------------------------------------------*/
void _Copter::command_Comm()
{
  //Counttimer1 = micros();
  if (RCsignal.THROTTLE > 950 && RCsignal.THROTTLE < 1050 && RCsignal.YAW < 1100 && RCsignal.YAW > 1000) {
    flag.ARMED = 1;
    //ControlClear();
  }
  if (flag.ARMED == 1 && RCsignal.THROTTLE < 1050 && RCsignal.YAW > 1450)  {
    flag.ARMED = 2;
    //ControlClear();
  }
  //Stopping the motors: throttle low and yaw right.
  if (RCsignal.THROTTLE < 1050 && RCsignal.YAW > 1890)  {
    flag.ARMED = 0;
    //ControlClear();
  }
  if (RCsignal.MODE < 1150 && RCsignal.MODE > 1000) flag.mode = Attitude_mode;
  if (RCsignal.MODE < 1550 && RCsignal.MODE > 1450) flag.mode = Altitude_mode ;
  if (RCsignal.MODE < 2000 && RCsignal.MODE > 1850) flag.mode = loiter_mode;
  if (RCsignal.SWITCH < 1200) flag.turnoff = 0;
  if (RCsignal.SWITCH > 1700) flag.turnoff = 1;
  Xbee_comm();
}
void _Copter::Xbee_comm()
{
  Xbee_receive();
  upporder = comorder;
  switch (upporder)
  {
    case 1006:
      Serial.print(gy89.gyro.origin.x); Serial.print('\t');
      Serial.println();
      break;
  }
  if (Xbee_timer == 10)
  {
    switch (comorder)
    {
      /*-------------------System Test---------------------*/
      case 1000:
        Serial1.print(RCsignal.ROLL); Serial1.print('\t');
        Serial1.print(RCsignal.PITCH); Serial1.print('\t');
        Serial1.print(RCsignal.THROTTLE); Serial1.print('\t');
        Serial1.print(RCsignal.YAW); Serial1.print('\t');
        Serial1.print(RCsignal.MODE); Serial1.print('\t');
        Serial1.print(RCsignal.SWITCH); Serial1.print('\t');
        Serial1.print("Arm: "); Serial1.print(flag.ARMED); Serial1.print('\t');
        Serial1.print("Turn Off: "); Serial1.print(flag.turnoff); Serial1.print('\t');
        if (flag.mode == 1)
        {
          Serial1.print("Attitude mode");
          Serial1.print('\t');
        }
        else if (flag.mode == 2)
        {
          Serial1.print("Altitude mode");
          Serial1.print('\t');
        }
        else if (flag.mode == 3)
        {
          Serial1.print("loiter mode");
          Serial1.print('\t');
        }
        Serial1.println();
        break;
      case 1010:
        Serial1.print(U1); Serial1.print('\t');
        Serial1.println();
        break;
      /*-------------------Gyro Test---------------------*/
      case 1020:
        flag.calibratedG = 0;
        comorder = 0;
        break;
      case 1021:
        Serial1.print(gy89.gyro.quietf.x); Serial1.print('\t');
        Serial1.print(gy89.gyro.quietf.y); Serial1.print('\t');
        Serial1.print(gy89.gyro.quietf.z); Serial1.print('\t');
        Serial1.print(gyro_temp); Serial1.print('\t');
        Serial1.println();
        break;
      case 1022:
        Serial1.print(gy89.gyro.origin.x); Serial1.print('\t');
        Serial1.print(gy89.gyro.aftcal.x); Serial1.print('\t');
        Serial1.println();
        break;
      case 1023:
        Serial1.print(gy89.gyro.origin.x); Serial1.print('\t');
        Serial1.print(gy89.gyro.origin.y); Serial1.print('\t');
        Serial1.print(gy89.gyro.origin.z); Serial1.print('\t');
        Serial1.print(gyro_temp); Serial1.print('\t');
        Serial1.println();
        break;
      case 1024:
        Serial1.print(gy89.gyro.aftcal.x); Serial1.print('\t');
        Serial1.print(gy89.gyro.aftcal.y); Serial1.print('\t');
        Serial1.print(gy89.gyro.aftcal.z); Serial1.print('\t');
        Serial1.println();
        break;
      case 1025:
        Serial1.print(gy89.gyro.aftcal.x); Serial1.print('\t');
        Serial1.print(gy89.gyro.filter.x); Serial1.print('\t');
        Serial1.println();
        break;
      case 1026:
        Serial1.print(gy89.gyro.origin.z); Serial1.print('\t');
        Serial1.print(gy89.gyro.aftcal.z); Serial1.print('\t');
        Serial1.println();
        break;
      case 1027:
        Serial1.print(gy89.gyro.radian.x); Serial1.print('\t');
        Serial1.print(gy89.gyro.radian.y); Serial1.print('\t');
        Serial1.print(gy89.gyro.radian.z); Serial1.print('\t');
        Serial1.println();
        break;
      /*-------------------Accelerometer Test---------------------*/
      case 1031:
        Serial1.print(gy89.acc.origin.x); Serial1.print('\t');
        Serial1.print(gy89.acc.origin.y); Serial1.print('\t');
        Serial1.print(gy89.acc.origin.z); Serial1.print('\t');
        Serial1.println();
        break;
      case 1032:
        Serial1.print(gy89.acc.aftcal.x); Serial1.print('\t');
        Serial1.print(gy89.acc.aftcal.y); Serial1.print('\t');
        Serial1.print(gy89.acc.aftcal.z); Serial1.print('\t');
        Serial1.println();
        break;
      case 1033:
        Serial1.print(gy89.acc.filter.x); Serial1.print('\t');
        Serial1.print(gy89.acc.filter.y); Serial1.print('\t');
        Serial1.print(gy89.acc.filter.z); Serial1.print('\t');
        Serial1.println();
        break;
      /*-------------------Accelerometer Calibration---------------------*/
      case 1041:    //level - z + g
        LSM303DCali(4);
        break;
      case 1042:    //back - z -g
        LSM303DCali(5);
        break;
      case 1043:    //right wing up - x + g
        LSM303DCali(0);
        break;
      case 1044:    //left wing up - x -g
        LSM303DCali(1);
        break;
      case 1045:    //nose up - y +g
        LSM303DCali(2);
        break;
      case 1046:    //nose down - y -g
        LSM303DCali(3);
        break;
      case 1047:    //reset
        LSM303DPointRead();
        break;
      case 1048:
        for (uint8_t point = 0; point < 6; point++)
        {
          Serial1.print(Acc_Cali.accel_raw_ref[point][0]); Serial1.print('\t');
          Serial1.print(Acc_Cali.accel_raw_ref[point][1]); Serial1.print('\t');
          Serial1.print(Acc_Cali.accel_raw_ref[point][2]); Serial1.print('\t');
          Serial1.println();
        }
        comorder = 0;
        break;
      case 1049:
        Serial1.println("-----------------------");
        for (uint8_t i = 0; i < 3; i++)
        {
          for (uint8_t j = 0; j < 3; j++)
          {
            Serial1.print(Acc_Cali.a[i][j]);
            Serial1.print('\t');
          }
          Serial1.println();
        }
        Serial1.println("-----------------------");
        for (uint8_t i = 0; i < 3; i++)
        {
          for (uint8_t j = 0; j < 3; j++)
          {
            Serial1.print(Acc_Cali.T[i][j]);
            Serial1.print('\t');
          }
          Serial1.println();
        }
        Serial1.println("-----------------------");
        comorder = 0;
        break;
      case 1051:
        Serial1.print(IMU.phi); Serial1.print('\t');
        Serial1.print(IMU.theta); Serial1.print('\t');
        Serial1.print(IMU.psi); Serial1.print('\t');
        Serial1.println("-----------------------");
        break;
      case 1052:
        Serial1.print(IMU.p); Serial1.print('\t');
        Serial1.print(IMU.q); Serial1.print('\t');
        Serial1.print(IMU.r); Serial1.print('\t');
        Serial1.println("-----------------------");
        break;
      case 1101:
        Serial1.print(U1); Serial1.print('\t');
        Serial1.print(U2); Serial1.print('\t');
        Serial1.print(U3); Serial1.print('\t');
        Serial1.print(U4); Serial1.print('\t');
        Serial1.println("-----------------------");
        break;
      case 1301:
        Serial1.println();
        Serial1.println("////////////////PID PRAM//////////////");
        Serial1.println();
        Serial1.print("P_Kp =  "); Serial1.println(pvel.Kp);
        Serial1.print("P_Ki =  "); Serial1.println(pvel.Ki);
        Serial1.print("P_Kd =  "); Serial1.println(pvel.Kd);
        Serial1.print("Q_Kp =  "); Serial1.println(qvel.Kp);
        Serial1.print("Q_Ki =  "); Serial1.println(qvel.Ki);
        Serial1.print("Q_Kd =  "); Serial1.println(qvel.Kd);
        Serial1.print("R_Kp =  "); Serial1.println(rvel.Kp);
        Serial1.print("R_Ki =  "); Serial1.println(rvel.Ki);
        Serial1.print("R_Kd =  "); Serial1.println(rvel.Kd);
        Serial1.print("Phi_Kp =  "); Serial1.println(phiang.Kp);
        Serial1.print("Theta_Kp =  "); Serial1.println(thetaang.Kp);
        Serial1.print("Psi_Kp =  "); Serial1.println(psiang.Kp);
        Serial1.println();
        Serial1.print("Z_Kp =  "); Serial1.println(Zalt.Kp);
        Serial1.print("Vz_Kp =  "); Serial1.println(Vzalt.Kp);
        Serial1.print("Vz_Ki =  "); Serial1.println(Vzalt.Ki);
        Serial1.print("Vz_Kd =  "); Serial1.println(Vzalt.Kd);
        Serial1.println();
        Serial1.print("Xpos_Kp =  "); Serial1.println(Xpos.Kp);
        Serial1.print("Vxpos_Kp =  "); Serial1.println(Vxpos.Kp);
        Serial1.print("Vxpos_Ki =  "); Serial1.println(Vxpos.Ki);
        Serial1.print("Vxpos_Kd =  "); Serial1.println(Vxpos.Kd);
        Serial1.print("Ypos_Kp =  "); Serial1.println(Ypos.Kp);
        Serial1.print("Vypos_Kp =  "); Serial1.println(Vypos.Kp);
        Serial1.print("Vypos_Ki =  "); Serial1.println(Vypos.Ki);
        Serial1.print("Vypos_Kd =  "); Serial1.println(Vypos.Kd);
        Serial1.println();
        Serial1.print("Gravity =  "); Serial1.println(EstimatedG);
        Serial1.println("//////////////////////////////////////");
        Serial1.println();
        comorder = 0;
        break;
      case 2001:
        Serial1.print(voltageavg); Serial1.println('\t');
        break;
    }
    Xbee_timer = 0;
  }
  Xbee_timer++;
}
void _Copter::Xbee_receive()
{
  String comdata = "";
  int16_t num = 0, stmp = 0;
  int32_t order = 0;
  float pram;
  while (Serial1.available() > 0)
    comdata += char(Serial1.read());
  datalength = comdata.length();
  if (datalength > 1 && datalength < 5)
  {
    comorder = 0;
    for (uint16_t i = 0; i < comdata.length() ; i++)
      comorder = comorder * 10 + (comdata[i] - '0');
  }
  else if (datalength > 5 && (comdata[0] == '&' && comdata[5] == '_' && comdata[datalength - 1] == '*'))
  {
    for (num = 1; num <= 4; num++)
      order = order * 10 + (comdata[num] - '0');
    for (num = 6; num < (datalength - 1); num++)
      stmp = stmp * 10 + (comdata[num] - '0');
    Serial1.print("Type:  "); Serial1.print(order);
    Serial1.print("  Data:  "); Serial1.print(stmp);
    Serial1.println();
    switch (order)
    {
      case 1011:
        pram = stmp;
        pvel.Kp = pram / 10;
        EEPROM.write(10, stmp & 0b11111111);
        EEPROM.write(11, stmp >> 8);
        Serial1.print("P_Kp =  "); Serial1.print(pvel.Kp);
        Serial1.println();
        break;
      case 1012:
        pram = stmp;
        pvel.Ki = pram / 10;
        EEPROM.write(12, stmp & 0b11111111);
        EEPROM.write(13, stmp >> 8);
        Serial1.print("P_Ki =  "); Serial1.print(pvel.Ki);
        Serial1.println();
        break;
      case 1013:
        pram = stmp;
        pvel.Kd = pram / 10;
        EEPROM.write(14, stmp & 0b11111111);
        EEPROM.write(15, stmp >> 8);
        Serial1.print("P_Kd =  "); Serial1.print(pvel.Kd);
        Serial1.println();
        break;
      case 1021:
        pram = stmp;
        qvel.Kp = pram / 10;
        EEPROM.write(18, stmp & 0b11111111);
        EEPROM.write(19, stmp >> 8);
        Serial1.print("Q_Kp =  "); Serial1.print(qvel.Kp);
        Serial1.println();
        break;
      case 1022:
        pram = stmp;
        qvel.Ki = pram / 10;
        EEPROM.write(20, stmp & 0b11111111);
        EEPROM.write(21, stmp >> 8);
        Serial1.print("Q_Ki =  "); Serial1.print(qvel.Ki);
        Serial1.println();
        break;
      case 1023:
        pram = stmp;
        qvel.Kd = pram / 10;
        EEPROM.write(22, stmp & 0b11111111);
        EEPROM.write(23, stmp >> 8);
        Serial1.print("Q_Kd =  "); Serial1.print(qvel.Kd);
        Serial1.println();
        break;
      case 1031:
        pram = stmp;
        rvel.Kp = pram / 10;
        EEPROM.write(26, stmp & 0b11111111);
        EEPROM.write(27, stmp >> 8);
        Serial1.print("R_Kp =  "); Serial1.print(rvel.Kp);
        Serial1.println();
        break;
      case 1032:
        pram = stmp;
        rvel.Ki = pram / 10;
        EEPROM.write(28, stmp & 0b11111111);
        EEPROM.write(29, stmp >> 8);
        Serial1.print("R_Ki =  "); Serial1.print(rvel.Ki);
        Serial1.println();
        break;
      case 1033:
        pram = stmp;
        rvel.Kd = pram / 10;
        EEPROM.write(30, stmp & 0b11111111);
        EEPROM.write(31, stmp >> 8);
        Serial1.print("R_Kd =  "); Serial1.print(rvel.Kd);
        Serial1.println();
        break;
      case 1051:
        pram = stmp;
        phiang.Kp = pram / 100;
        EEPROM.write(16, stmp & 0b11111111);
        EEPROM.write(17, stmp >> 8);
        Serial1.print("Phi_Kp =  "); Serial1.print(phiang.Kp);
        Serial1.println();
        break;
      case 1052:
        pram = stmp;
        thetaang.Kp = pram / 100;
        EEPROM.write(24, stmp & 0b11111111);
        EEPROM.write(25, stmp >> 8);
        Serial1.print("Theta_Kp =  "); Serial1.print(thetaang.Kp);
        Serial1.println();
        break;
      case 1053:
        pram = stmp;
        psiang.Kp = pram / 100;
        EEPROM.write(32, stmp & 0b11111111);
        EEPROM.write(33, stmp >> 8);
        Serial1.print("Psi_Kp =  "); Serial1.print(psiang.Kp);
        Serial1.println();
        break;
      case 1071:
        pram = stmp;
        Zalt.Kp = pram / 100;
        EEPROM.write(40, stmp & 0b11111111);
        EEPROM.write(41, stmp >> 8);
        Serial1.print("Z_Kp =  "); Serial1.print(Zalt.Kp);
        Serial1.println();
        break;
      case 1081:
        pram = stmp;
        Vzalt.Kp = pram / 100;
        EEPROM.write(42, stmp & 0b11111111);
        EEPROM.write(43, stmp >> 8);
        Serial1.print("Vz_Kp =  "); Serial1.print(Vzalt.Kp);
        Serial1.println();
        break;
      case 1082:
        pram = stmp;
        Vzalt.Ki = pram / 100;
        EEPROM.write(44, stmp & 0b11111111);
        EEPROM.write(45, stmp >> 8);
        Serial1.print("Vz_Ki =  "); Serial1.print(Vzalt.Ki);
        Serial1.println();
        break;
      case 1083:
        pram = stmp;
        Vzalt.Kd = pram / 100;
        EEPROM.write(46, stmp & 0b11111111);
        EEPROM.write(47, stmp >> 8);
        Serial1.print("Vz_Kd =  "); Serial1.print(Vzalt.Kd);
        Serial1.println();
        break;
      case 1091:
        pram = stmp;
        Xpos.Kp = pram / 1000;
        EEPROM.write(50, stmp & 0b11111111);
        EEPROM.write(51, stmp >> 8);
        Serial1.print("Xpos_Kp =  "); Serial1.print(Xpos.Kp);
        Serial1.println();
        break;
      case 1092:
        pram = stmp;
        Vxpos.Kp = pram / 1000;
        EEPROM.write(52, stmp & 0b11111111);
        EEPROM.write(53, stmp >> 8);
        Serial1.print("Vxpos_Kp =  "); Serial1.print(Vxpos.Kp);
        Serial1.println();
        break;
      case 1093:
        pram = stmp;
        Vxpos.Ki = pram / 1000;
        EEPROM.write(54, stmp & 0b11111111);
        EEPROM.write(55, stmp >> 8);
        Serial1.print("Vxpos_Ki =  "); Serial1.print(Vxpos.Ki);
        Serial1.println();
        break;
      case 1094:
        pram = stmp;
        Vxpos.Kd = pram / 1000;
        EEPROM.write(56, stmp & 0b11111111);
        EEPROM.write(57, stmp >> 8);
        Serial1.print("Vxpos_Kd =  "); Serial1.print(Vxpos.Kd);
        Serial1.println();
        break;
      case 1095:
        pram = stmp;
        Ypos.Kp = pram / 1000;
        EEPROM.write(60, stmp & 0b11111111);
        EEPROM.write(61, stmp >> 8);
        Serial1.print("Ypos_Kp =  "); Serial1.print(Ypos.Kp);
        Serial1.println();
        break;
      case 1096:
        pram = stmp;
        Vypos.Kp = pram / 1000;
        EEPROM.write(62, stmp & 0b11111111);
        EEPROM.write(63, stmp >> 8);
        Serial1.print("Vypos_Kp =  "); Serial1.print(Vypos.Kp);
        Serial1.println();
        break;
      case 1097:
        pram = stmp;
        Vypos.Ki = pram / 1000;
        EEPROM.write(64, stmp & 0b11111111);
        EEPROM.write(65, stmp >> 8);
        Serial1.print("Vypos_Ki =  "); Serial1.print(Vypos.Ki);
        Serial1.println();
        break;
      case 1098:
        pram = stmp;
        Vypos.Kd = pram / 1000;
        EEPROM.write(66, stmp & 0b11111111);
        EEPROM.write(67, stmp >> 8);
        Serial1.print("Vypos_Kd =  "); Serial1.print(Vypos.Kd);
        Serial1.println();
        break;
      case 1100:
        pram = stmp;
        EstimatedG = pram / 1000;
        EEPROM.write(70, stmp & 0b11111111);
        EEPROM.write(71, stmp >> 8);
        Serial1.print("Gravity =  "); Serial1.print(EstimatedG);
        Serial1.println();
        break;
    }
  }
}
