//GY89
#define L3GD20_ADDRESS    (0xD6 >> 1)
#define L3G_WHO_AM_I      0x0F

#define L3G_CTRL_REG1     0x20
#define L3G_CTRL_REG2     0x21
#define L3G_CTRL_REG3     0x22
#define L3G_CTRL_REG4     0x23
#define L3G_CTRL_REG5     0x24
#define L3G_REFERENCE     0x25
#define L3G_OUT_TEMP      0x26
#define L3G_STATUS_REG    0x27

#define L3G_OUT_X_L       0x28
#define L3G_OUT_X_H       0x29
#define L3G_OUT_Y_L       0x2A
#define L3G_OUT_Y_H       0x2B
#define L3G_OUT_Z_L       0x2C
#define L3G_OUT_Z_H       0x2D

#define L3G_FIFO_CTRL_REG 0x2E
#define L3G_FIFO_SRC_REG  0x2F

#define L3G_INT1_CFG      0x30
#define L3G_INT1_SRC      0x31
#define L3G_INT1_THS_XH   0x32
#define L3G_INT1_THS_XL   0x33
#define L3G_INT1_THS_YH   0x34
#define L3G_INT1_THS_YL   0x35
#define L3G_INT1_THS_ZH   0x36
#define L3G_INT1_THS_ZL   0x37
#define L3G_INT1_DURATION 0x38

#define LSM303D_ADDRESS   0b0011101
#define LSM303D_CTRL_REG0      0x1F
#define LSM303D_CTRL_REG1      0x20
#define LSM303D_CTRL_REG2      0x21
#define LSM303D_CTRL_REG3      0x22
#define LSM303D_CTRL_REG4      0x23
#define LSM303D_CTRL_REG5      0x24
#define LSM303D_CTRL_REG6      0x25
#define LSM303D_CTRL_REG7      0x26
#define LSM303D_OUT_X_L_A      0x28

#define L3GD20_DEFAULT_FILTER_FREQ    30
#define L3GD20_DEFAULT_RATE 400

#define LSM303D_ACCEL_DEFAULT_RANGE_G      16
#define LSM303D_ACCEL_DEFAULT_RATE      400
#define LSM303D_ACCEL_DEFAULT_ONCHIP_FILTER_FREQ  50
#define LSM303D_ACCEL_DEFAULT_DRIVER_FILTER_FREQ  30
