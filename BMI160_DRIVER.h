#ifndef BMI160_DRIVER
#define BMI160_DRIVER

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"




#define DEFAULT_BAUD 400000

//Validated with test sweep
#define BMI160_ADDR 0x69


#define CHIP_ID_REG 0x00

#define ERR_REG 0x02
#define TIMESTAMP_REG 0x18
#define STATUS_REG 0x1B
#define SELF_TEST_REG 0x6D


#define ACC_CFG 0x40
#define ACC_DEF_CFG 0x28
#define ACC_RNG 0x41
#define ACC_DEF_RNG 0x03

#define GYR_CFG 0x42
#define GYR_DEF_CFG 0x28
#define GYR_RNG 0x43
#define GYR_DEF_RNG 0x00

#define GYR_X_REG 0x0C
#define GYR_Y_REG 0x0F
#define GYR_Z_REG 0x10

#define ACC_X_REG 0x12
#define ACC_Y_REG 0x13
#define ACC_Z_REG 0x16

#define CMD_REG 0x7E

#define SOFT_RESET 0xB6
#define SOFT_RESET_WAIT_MS 1

//Accelerometer commands
#define ACCEL_ON 0x11
#define ACCEL_OFF 0x10
//Gyroscope commands
#define GYR_ON 0x15
#define GYR_OFF 0x14


//Device struct
typedef struct{
    //I2C hardware
    i2c_inst_t *I2C_HW;

    //I2C comms pins
    int SDA_PIN;
    int SCL_PIN;

    int EN_PIN;

}bmi160;

//data struct - can represent gyro or acc data
typedef struct{
    uint16_t x;
    uint16_t y;
    uint16_t z;
}bmi160_data;


//Setup I2C for the device
bmi160 init_bmi160(int I2C_HW, int SDA_pin, int SCL_pin, int EN_pin);

//Run a device self test
int self_test(bmi160 *dev);

//Set the config of the accelerometer to the default then turn it on
int boot_def_accel(bmi160 *dev, bool load_cfg);

//Set the config of the gyroscope to the default then turn it on
int boot_def_gyr(bmi160 *dev, bool load_cfg);

//Return the error reg val
uint8_t get_err(bmi160 *dev);

/*
Write a value to a register
*/
int write_register(bmi160 *dev, uint8_t reg, uint8_t data);

/*
Read a single byte from the BMI160
*/
int read_register(bmi160 *dev, uint8_t reg, uint8_t *buf);

/*
Read 2 bytes from the BMI160
*/
int read_2_byte_register(bmi160 *dev, uint8_t reg, uint8_t *buf);

/*
read 3 bytes of data (mainly for timestamp)
*/
int read_3_byte_register(bmi160 *dev, uint8_t reg, uint8_t *buf);

//Returns whether the acceleration data is ready
bool is_acc_ready(bmi160 *dev);

//Returns whether the gyroscope data is ready
bool is_gyr_ready(bmi160 *dev);


//Get the time of the sensor i nraw byte value
uint8_t get_timestamp_raw(bmi160 *dev, uint8_t *tstamp_buf);

//Get the full XYZ gyro data
int get_gyr_data(bmi160 *dev, bmi160_data *gyr_data);
//Get the full XYZ accel data
int get_acc_data(bmi160 *dev, bmi160_data *acc_data);



#endif