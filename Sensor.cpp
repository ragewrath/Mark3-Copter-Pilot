#include "Copter.h"
void _Copter::InitSensor()
{
  Wire.begin();
  Wire.setRate(I2C_RATE_2000);
  Wire1.begin();
  Wire1.setRate(I2C_RATE_2000);
  delay(200);
  //GY89
  I2Cwrite(L3GD20_ADDRESS, L3G_CTRL_REG1, 0xAF, 0);
  I2Cwrite(L3GD20_ADDRESS, L3G_CTRL_REG2, 0, 0);
  I2Cwrite(L3GD20_ADDRESS, L3G_CTRL_REG3, 0, 0);
  I2Cwrite(L3GD20_ADDRESS, L3G_CTRL_REG4, 0x10, 0);
  I2Cwrite(L3GD20_ADDRESS, L3G_FIFO_CTRL_REG, 0x00, 0);
  I2Cwrite(LSM303D_ADDRESS, LSM303D_CTRL_REG0, 0, 0);
  I2Cwrite(LSM303D_ADDRESS, LSM303D_CTRL_REG1, 0x87, 0);
  I2Cwrite(LSM303D_ADDRESS, LSM303D_CTRL_REG2, 0xC8, 0);
  LSM303DPointRead();
}
void _Copter::L3GD20read()
{
  I2CRead(L3GD20_ADDRESS, L3G_OUT_TEMP | (1 << 7), 8);
  gyro_temp = i2cData[0] * -1.0 + 40;
  if (gyro_temp < - 180 && gyro_temp + 256 >= 40)
    gyro_temp = gyro_temp + 256;
  if (gyro_temp < 15)
    gyro_temp = 15;
  if (gyro_temp > 65)
    gyro_temp = 65;

  float gyro_temp_4, gyro_temp_3, gyro_temp_2;
  gyro_temp_4 = gyro_temp * gyro_temp * gyro_temp * gyro_temp;
  gyro_temp_3 = gyro_temp * gyro_temp * gyro_temp;
  gyro_temp_2 = gyro_temp * gyro_temp;

  gy89.gyro.origin.x = ((i2cData[3] << 8) | i2cData[2]);
  gy89.gyro.origin.y = ((i2cData[5] << 8) | i2cData[4]);
  gy89.gyro.origin.z = ((i2cData[7] << 8) | i2cData[6]);
  //Temperature Compensation
  gy89.gyro.quietf.x = 0.00005023 * gyro_temp_4  - 0.008857 * gyro_temp_3 + 0.4812 * gyro_temp_2 - 6.678 * gyro_temp - 120.4;
  gy89.gyro.quietf.y = -0.00009043 * gyro_temp_4  + 0.0134 * gyro_temp_3 - 0.6597 * gyro_temp_2 + 6.861 * gyro_temp + 47.37;
  gy89.gyro.quietf.z = 0.00002922 * gyro_temp_4  - 0.004101 * gyro_temp_3 + 0.1754 * gyro_temp_2 - 4.791 * gyro_temp + 120.7;
  //Offset Calculate

  gy89.gyro.aftcal.x = (gy89.gyro.origin.x - gy89.gyro.quietf.x) * 1.0 / 65.536;
  gy89.gyro.aftcal.y = (gy89.gyro.origin.y - gy89.gyro.quietf.y) * 1.0 / 65.536;
  gy89.gyro.aftcal.z = (gy89.gyro.origin.z - gy89.gyro.quietf.z) * 1.0 / 65.536;
  
  if (flag.calibratedG == 0 && GyroCaliFlag <= 1500)
  {
    GyroCaliFlag++;
    L3GD20Cali();
  }
  if (flag.calibratedG == 1)
  {
    gy89.gyro.aftcal.x = gy89.gyro.aftcal.x - gy89.gyro.radian.x;
    gy89.gyro.aftcal.y = gy89.gyro.aftcal.y - gy89.gyro.radian.y;
    gy89.gyro.aftcal.z = gy89.gyro.aftcal.z - gy89.gyro.radian.z;
  }
}
void _Copter::LSM303DPointRead()
{
  uint8_t point;
  for (point = 0; point < 6; point++)
  {
    Acc_Cali.accel_raw_ref[point][0] = (EEPROM.read(100 + 6 * point + 2) << 8) | EEPROM.read(100 + 6 * point + 1);
    Acc_Cali.accel_raw_ref[point][1] = (EEPROM.read(100 + 6 * point + 4) << 8) | EEPROM.read(100 + 6 * point + 3);
    Acc_Cali.accel_raw_ref[point][2] = (EEPROM.read(100 + 6 * point + 6) << 8) | EEPROM.read(100 + 6 * point + 5);
  }

  Acc_Cali.acc_offset[0] = (float)(Acc_Cali.accel_raw_ref[0][0] + Acc_Cali.accel_raw_ref[1][0]) / 2.0;
  Acc_Cali.acc_offset[1] = (float)(Acc_Cali.accel_raw_ref[2][1] + Acc_Cali.accel_raw_ref[3][1]) / 2.0;
  Acc_Cali.acc_offset[2] = (float)(Acc_Cali.accel_raw_ref[4][2] + Acc_Cali.accel_raw_ref[5][2]) / 2.0;

  for (point = 0; point < 3; point++)
    Acc_Cali.a[0][point] = (float)Acc_Cali.accel_raw_ref[0][point] - Acc_Cali.acc_offset[point];
  for (point = 0; point < 3; point++)
    Acc_Cali.a[1][point] = (float)Acc_Cali.accel_raw_ref[2][point] - Acc_Cali.acc_offset[point];
  for (point = 0; point < 3; point++)
    Acc_Cali.a[2][point] = (float)Acc_Cali.accel_raw_ref[4][point] - Acc_Cali.acc_offset[point];

  Acc_Cali.T[0][0] = (Acc_Cali.g * (Acc_Cali.a[1][1] * Acc_Cali.a[2][2] - Acc_Cali.a[1][2] * Acc_Cali.a[2][1])) / (Acc_Cali.a[0][0] * Acc_Cali.a[1][1] * Acc_Cali.a[2][2] - Acc_Cali.a[0][0] * Acc_Cali.a[1][2] * Acc_Cali.a[2][1] - Acc_Cali.a[0][1] * Acc_Cali.a[1][0] * Acc_Cali.a[2][2] + Acc_Cali.a[0][1] * Acc_Cali.a[1][2] * Acc_Cali.a[2][0] + Acc_Cali.a[0][2] * Acc_Cali.a[1][0] * Acc_Cali.a[2][1] - Acc_Cali.a[0][2] * Acc_Cali.a[1][1] * Acc_Cali.a[2][0]);
  Acc_Cali.T[0][1] = -(Acc_Cali.g * (Acc_Cali.a[0][1] * Acc_Cali.a[2][2] - Acc_Cali.a[0][2] * Acc_Cali.a[2][1])) / (Acc_Cali.a[0][0] * Acc_Cali.a[1][1] * Acc_Cali.a[2][2] - Acc_Cali.a[0][0] * Acc_Cali.a[1][2] * Acc_Cali.a[2][1] - Acc_Cali.a[0][1] * Acc_Cali.a[1][0] * Acc_Cali.a[2][2] + Acc_Cali.a[0][1] * Acc_Cali.a[1][2] * Acc_Cali.a[2][0] + Acc_Cali.a[0][2] * Acc_Cali.a[1][0] * Acc_Cali.a[2][1] - Acc_Cali.a[0][2] * Acc_Cali.a[1][1] * Acc_Cali.a[2][0]);
  Acc_Cali.T[0][2] = (Acc_Cali.g * (Acc_Cali.a[0][1] * Acc_Cali.a[1][2] - Acc_Cali.a[0][2] * Acc_Cali.a[1][1])) / (Acc_Cali.a[0][0] * Acc_Cali.a[1][1] * Acc_Cali.a[2][2] - Acc_Cali.a[0][0] * Acc_Cali.a[1][2] * Acc_Cali.a[2][1] - Acc_Cali.a[0][1] * Acc_Cali.a[1][0] * Acc_Cali.a[2][2] + Acc_Cali.a[0][1] * Acc_Cali.a[1][2] * Acc_Cali.a[2][0] + Acc_Cali.a[0][2] * Acc_Cali.a[1][0] * Acc_Cali.a[2][1] - Acc_Cali.a[0][2] * Acc_Cali.a[1][1] * Acc_Cali.a[2][0]);

  Acc_Cali.T[1][0] = -(Acc_Cali.g * (Acc_Cali.a[1][0] * Acc_Cali.a[2][2] - Acc_Cali.a[1][2] * Acc_Cali.a[2][0])) / (Acc_Cali.a[0][0] * Acc_Cali.a[1][1] * Acc_Cali.a[2][2] - Acc_Cali.a[0][0] * Acc_Cali.a[1][2] * Acc_Cali.a[2][1] - Acc_Cali.a[0][1] * Acc_Cali.a[1][0] * Acc_Cali.a[2][2] + Acc_Cali.a[0][1] * Acc_Cali.a[1][2] * Acc_Cali.a[2][0] + Acc_Cali.a[0][2] * Acc_Cali.a[1][0] * Acc_Cali.a[2][1] - Acc_Cali.a[0][2] * Acc_Cali.a[1][1] * Acc_Cali.a[2][0]);
  Acc_Cali.T[1][1] = (Acc_Cali.g * (Acc_Cali.a[0][0] * Acc_Cali.a[2][2] - Acc_Cali.a[0][2] * Acc_Cali.a[2][0])) / (Acc_Cali.a[0][0] * Acc_Cali.a[1][1] * Acc_Cali.a[2][2] - Acc_Cali.a[0][0] * Acc_Cali.a[1][2] * Acc_Cali.a[2][1] - Acc_Cali.a[0][1] * Acc_Cali.a[1][0] * Acc_Cali.a[2][2] + Acc_Cali.a[0][1] * Acc_Cali.a[1][2] * Acc_Cali.a[2][0] + Acc_Cali.a[0][2] * Acc_Cali.a[1][0] * Acc_Cali.a[2][1] - Acc_Cali.a[0][2] * Acc_Cali.a[1][1] * Acc_Cali.a[2][0]);
  Acc_Cali.T[1][2] = -(Acc_Cali.g * (Acc_Cali.a[0][0] * Acc_Cali.a[1][2] - Acc_Cali.a[0][2] * Acc_Cali.a[1][0])) / (Acc_Cali.a[0][0] * Acc_Cali.a[1][1] * Acc_Cali.a[2][2] - Acc_Cali.a[0][0] * Acc_Cali.a[1][2] * Acc_Cali.a[2][1] - Acc_Cali.a[0][1] * Acc_Cali.a[1][0] * Acc_Cali.a[2][2] + Acc_Cali.a[0][1] * Acc_Cali.a[1][2] * Acc_Cali.a[2][0] + Acc_Cali.a[0][2] * Acc_Cali.a[1][0] * Acc_Cali.a[2][1] - Acc_Cali.a[0][2] * Acc_Cali.a[1][1] * Acc_Cali.a[2][0]);

  Acc_Cali.T[2][0] = (Acc_Cali.g * (Acc_Cali.a[1][0] * Acc_Cali.a[2][1] - Acc_Cali.a[1][1] * Acc_Cali.a[2][0])) / (Acc_Cali.a[0][0] * Acc_Cali.a[1][1] * Acc_Cali.a[2][2] - Acc_Cali.a[0][0] * Acc_Cali.a[1][2] * Acc_Cali.a[2][1] - Acc_Cali.a[0][1] * Acc_Cali.a[1][0] * Acc_Cali.a[2][2] + Acc_Cali.a[0][1] * Acc_Cali.a[1][2] * Acc_Cali.a[2][0] + Acc_Cali.a[0][2] * Acc_Cali.a[1][0] * Acc_Cali.a[2][1] - Acc_Cali.a[0][2] * Acc_Cali.a[1][1] * Acc_Cali.a[2][0]);
  Acc_Cali.T[2][1] = -(Acc_Cali.g * (Acc_Cali.a[0][0] * Acc_Cali.a[2][1] - Acc_Cali.a[0][1] * Acc_Cali.a[2][0])) / (Acc_Cali.a[0][0] * Acc_Cali.a[1][1] * Acc_Cali.a[2][2] - Acc_Cali.a[0][0] * Acc_Cali.a[1][2] * Acc_Cali.a[2][1] - Acc_Cali.a[0][1] * Acc_Cali.a[1][0] * Acc_Cali.a[2][2] + Acc_Cali.a[0][1] * Acc_Cali.a[1][2] * Acc_Cali.a[2][0] + Acc_Cali.a[0][2] * Acc_Cali.a[1][0] * Acc_Cali.a[2][1] - Acc_Cali.a[0][2] * Acc_Cali.a[1][1] * Acc_Cali.a[2][0]);
  Acc_Cali.T[2][2] = (Acc_Cali.g * (Acc_Cali.a[0][0] * Acc_Cali.a[1][1] - Acc_Cali.a[0][1] * Acc_Cali.a[1][0])) / (Acc_Cali.a[0][0] * Acc_Cali.a[1][1] * Acc_Cali.a[2][2] - Acc_Cali.a[0][0] * Acc_Cali.a[1][2] * Acc_Cali.a[2][1] - Acc_Cali.a[0][1] * Acc_Cali.a[1][0] * Acc_Cali.a[2][2] + Acc_Cali.a[0][1] * Acc_Cali.a[1][2] * Acc_Cali.a[2][0] + Acc_Cali.a[0][2] * Acc_Cali.a[1][0] * Acc_Cali.a[2][1] - Acc_Cali.a[0][2] * Acc_Cali.a[1][1] * Acc_Cali.a[2][0]);
}
void _Copter::LSM303DCali(uint8_t point)
{
  /*Point = 0 1 2 3 4 5*/
  if (Acc_Cali.acc_calitimer == 100)
  {
    Acc_Cali.accel_raw_ref[point][0] = Acc_Cali.acc_calitmpx / 100;
    Acc_Cali.accel_raw_ref[point][1] = Acc_Cali.acc_calitmpy / 100;
    Acc_Cali.accel_raw_ref[point][2] = Acc_Cali.acc_calitmpz / 100;
    Acc_Cali.acc_calitmpx = 0;
    Acc_Cali.acc_calitmpy = 0;
    Acc_Cali.acc_calitmpz = 0;
    Acc_Cali.acc_calitimer = 0;
    comorder = 0;
    Serial1.print(Acc_Cali.accel_raw_ref[point][0]); Serial1.print('\t');
    Serial1.print(Acc_Cali.accel_raw_ref[point][1]); Serial1.print('\t');
    Serial1.print(Acc_Cali.accel_raw_ref[point][2]); Serial1.print('\t');
    Serial1.println();
    EEPROM.write(100 + 6 * point + 1, Acc_Cali.accel_raw_ref[point][0] & 0b11111111);
    EEPROM.write(100 + 6 * point + 2, Acc_Cali.accel_raw_ref[point][0] >> 8);
    EEPROM.write(100 + 6 * point + 3, Acc_Cali.accel_raw_ref[point][1] & 0b11111111);
    EEPROM.write(100 + 6 * point + 4, Acc_Cali.accel_raw_ref[point][1] >> 8);
    EEPROM.write(100 + 6 * point + 5, Acc_Cali.accel_raw_ref[point][2] & 0b11111111);
    EEPROM.write(100 + 6 * point + 6, Acc_Cali.accel_raw_ref[point][2] >> 8);
  }
  else
  {
    Acc_Cali.acc_calitmpx += gy89.acc.origin.x;
    Acc_Cali.acc_calitmpy += gy89.acc.origin.y;
    Acc_Cali.acc_calitmpz += gy89.acc.origin.z;
    Acc_Cali.acc_calitimer++;
  }
}
void _Copter::L3GD20Cali()
{
  if (GyroCaliFlag > 400 && GyroCaliFlag < 1001)
  {
    GyroCollection[0] += gy89.gyro.aftcal.x;
    GyroCollection[1] += gy89.gyro.aftcal.y;
    GyroCollection[2] += gy89.gyro.aftcal.z;
  }
  if (GyroCaliFlag == 1001)
  {
    gy89.gyro.radian.x = GyroCollection[0] / 600;
    gy89.gyro.radian.y = GyroCollection[1] / 600;
    gy89.gyro.radian.z = GyroCollection[2] / 600;
    Serial1.print(GyroCollection[0]); Serial1.print("\t");
    Serial1.print(GyroCollection[1]); Serial1.print("\t");
    Serial1.print(GyroCollection[2]); Serial1.print("\t");
    GyroCaliFlag = 0;
    GyroCollection[0] = 0;
    GyroCollection[1] = 0;
    GyroCollection[2] = 0;
    flag.calibratedG = 1;
    Serial1.println("Gyro Offset Calculated");
    Serial1.print(gy89.gyro.radian.x); Serial1.print("\t");
    Serial1.print(gy89.gyro.radian.y); Serial1.print("\t");
    Serial1.print(gy89.gyro.radian.z); Serial1.print("\t");
  }
}
void _Copter::LSM303Dread()
{
  I2CRead(LSM303D_ADDRESS, LSM303D_OUT_X_L_A | (1 << 7), 6);
  gy89.acc.origin.x = ((i2cData[1] << 8) | i2cData[0]);
  gy89.acc.origin.y = ((i2cData[3] << 8) | i2cData[2]);
  gy89.acc.origin.z = ((i2cData[5] << 8) | i2cData[4]);

  gy89.acc.quietf.x = (float)gy89.acc.origin.x - Acc_Cali.acc_offset[0];
  gy89.acc.quietf.y = (float)gy89.acc.origin.y - Acc_Cali.acc_offset[1];
  gy89.acc.quietf.z = (float)gy89.acc.origin.z - Acc_Cali.acc_offset[2];

  gy89.acc.aftcal.x = gy89.acc.quietf.x *  Acc_Cali.T[0][0] + gy89.acc.quietf.y *  Acc_Cali.T[1][0] + gy89.acc.quietf.z *  Acc_Cali.T[2][0];
  gy89.acc.aftcal.y = gy89.acc.quietf.x *  Acc_Cali.T[0][1] + gy89.acc.quietf.y *  Acc_Cali.T[1][1] + gy89.acc.quietf.z *  Acc_Cali.T[2][1];
  gy89.acc.aftcal.z = gy89.acc.quietf.x *  Acc_Cali.T[0][2] + gy89.acc.quietf.y *  Acc_Cali.T[1][2] + gy89.acc.quietf.z *  Acc_Cali.T[2][2];
}
/*----------------------------------------I2C----------------------------------------*/
uint8_t _Copter::I2Cwrite(uint8_t SENSOR_ADDRESS, uint8_t SENSOR_REGISTER, uint8_t SENSOR_VALUE, bool sendStop)
{
  Wire.beginTransmission(SENSOR_ADDRESS);
  Wire.write(SENSOR_REGISTER);    // Chip reset DEVICE_RESET 1
  Wire.write(SENSOR_VALUE);//DEVICE_RESET
  //Wire.endTransmission();
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial1.print(F("i2cWrite failed: "));
    Serial1.println(rcode);
  }
  return rcode;
}
uint8_t _Copter::I2CRead(uint8_t SENSOR_ADDRESS, uint8_t SENSOR_REGISTER, uint8_t nbytes)
{
  uint32_t timeOutTimer;
  Wire.beginTransmission(SENSOR_ADDRESS);
  Wire.write(SENSOR_REGISTER);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial1.print(F("i2cRead failed: "));
    Serial1.println(rcode);
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }
  Wire.requestFrom(SENSOR_ADDRESS, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      i2cData[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        i2cData[i] = Wire.read();
      else {
        Serial1.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}
