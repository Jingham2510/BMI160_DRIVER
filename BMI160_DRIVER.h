#ifndef BMI160_DRIVER
#define BMI160_DRIVER

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"




#define DEFAULT_BAUD 100000


#define BMI160_ADDR 0x68


#define CHIP_ID_REG 0x00

#define STATUS_REG 0x1B

//Device struct
typedef struct{

    //I2C hardware
    i2c_inst_t *I2C_HW;

    //I2C comms pins
    int SDA_PIN;
    int SCL_PIN;

    int EN_PIN;

}bmi160;


//Setup I2C for the device
bmi160 init_bmi160(int I2C_HW, int SDA_pin, int SCL_pin, int EN_pin);


/*

*/
int write_register(bmi160 *dev, uint8_t reg, uint8_t data);

/*
Read a single byte from the BMI160
*/
int read_register(bmi160 *dev, uint8_t reg, uint8_t *buf);

/*Data to read:

1)All 19 registers contianing the magnetomer, gyro, accel data
2) Sensortime data (24bit counter incremembeter every 39us)
      2a) worht noting after 10 mins and 54 seconds the timer wraps
      2b) Dont need total time though just time diff
*/

//Returns whether the acceleration data is ready or not
bool is_acc_ready(bmi160 *dev);

//Returns whether the gyroscope data is ready or not
bool is_gyr_ready(bmi160 *dev);


#endif