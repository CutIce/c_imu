#ifndef IMU_BMI088_H
#define IMU_BMI088_H

#include <stdint.h>
#include "config.h"
#include <math.h>

#define BMI088_USE_SPI
// #define BMI088_USE_IIC


#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_WRITE_ACCEL_REG_NUM 6
#define BMI088_WRITE_GYRO_REG_NUM 6

#define BMI088_GYRO_DATA_READY_BIT 0
#define BMI088_ACCEL_DATA_READY_BIT 1
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2

#define BMI088_LONG_DELAY_TIME 80
#define BMI088_COM_WAIT_SENSOR_TIME 150


#define BMI088_ACCEL_IIC_ADDRESSE (0x18 << 1)
#define BMI088_GYRO_IIC_ADDRESSE (0x68 << 1)

// #define BMI088_ACCEL_RANGE_3G
//#define BMI088_ACCEL_RANGE_6G
#define BMI088_ACCEL_RANGE_12G
//#define BMI088_ACCEL_RANGE_24G

#define BMI088_GYRO_RANGE_2000
//#define BMI088_GYRO_RANGE_1000
//#define BMI088_GYRO_RANGE_500
//#define BMI088_GYRO_RANGE_250
//#define BMI088_GYRO_RANGE_125

#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f

#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f


typedef struct {
    uint8_t statue;
    float accel[3];
    float gryo3[3];
    float temp;
    float stamp;
} BMI088_Data_t;

enum {
  BMI088_NO_ERROR = 0x00,
  BMI088_ACC_PWR_CTRL_ERROR = 0x01,
  BMI088_ACC_PWR_CONF_ERROR = 0x02,
  BMI088_ACC_CONF_ERROR = 0x03,
  BMI088_ACC_SELF_TEST_ERROR = 0x04,
  BMI088_ACC_RANGE_ERROR = 0x05,
  BMI088_INT1_IO_CTRL_ERROR = 0x06,
  BMI088_INT_MAP_DATA_ERROR = 0x07,
  BMI088_GYRO_RANGE_ERROR = 0x08,
  BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
  BMI088_GYRO_LPM1_ERROR = 0x0A,
  BMI088_GYRO_CTRL_ERROR = 0x0B,
  BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
  BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,

  BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
  BMI088_SELF_TEST_GYRO_ERROR = 0x40,
  BMI088_NO_SENSOR = 0xFF,
};

typedef struct acc_raw_data_t {
  float x;
  float y;
  float z;
} acc_raw_data_t;

typedef struct acc_data_t {
  acc_raw_data_t acc_raw_data;
  float sensor_time;
  float temp;
bool enale_self_test;
} acc_data_t;

typedef struct gyto_raw_data_t {
  float x;
  float y;
  float z;
} gyro_raw_data_t;

typedef struct gyro_data_t {
  gyro_raw_data_t gyro_raw_data;
  bool enable_self_test;
} gyro_data_t;


// uint8_t BMI088_init(void);
// void BMI088_read(float gry0[3], float accel[3], float* temp);
// void BMI088_read_gryo_who_am_i(void);
// void BMI088_read_accel_who_am_i(void);

#endif // !IMU_BMI088_H