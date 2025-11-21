#include "BMI160_DRIVER.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "tusb.h"

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_0 i2c0
#define I2C_1 i2c1
#define I2C_SDA 16
#define I2C_SCL 17
#define EN_P 10


void wait_for_go();
bool is_cmd_flushed(bmi160 *dev);

int main()
{
    stdio_init_all();

    wait_for_go();

    bmi160 IMU = init_bmi160(0, I2C_SDA, I2C_SCL, EN_P);
   

   while(!is_acc_ready(&IMU)){
        printf("Not ready!\n");
        sleep_us(5000);
        //DEBUG - we know is broken atm
        return -1;
   }


   
}


/*
Helper function to apply a wait until the USB bus recieves something (anything)
*/
void wait_for_go(){
    

    while(!tud_cdc_connected()){
        sleep_ms(100);
        printf("Waiting...!\n");

    }

    //Wait for a serial bit/byte
    getchar();

    //Tell the device its going
    printf("Going!\n");

}


bmi160 init_bmi160(int I2C_HW, int SDA_pin, int SCL_pin, int EN_pin){
     
    
    //Assign and turn on the hardware block
    i2c_inst_t *HW_block;

    //Default to the first I2C block (incase of erroneous input)
    if (I2C_HW != 1){
        HW_block = I2C_0;
    }else if(I2C_HW == 1){
        HW_block = I2C_1;
    }


    //Turn on the I2C hw block
    if(i2c_init(HW_block, DEFAULT_BAUD) > DEFAULT_BAUD){
        printf("DEVICE MAY BE OVERDRIVEN");
    }    

    //Setup the pins
    gpio_set_function(SDA_pin, GPIO_FUNC_I2C);
    gpio_set_function(SCL_pin, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_pin);
    gpio_pull_up(SCL_pin);

    bmi160 IMU_dev;
    IMU_dev.I2C_HW = HW_block;
    IMU_dev.SDA_PIN = SDA_pin;
    IMU_dev.SCL_PIN = SCL_pin;
    IMU_dev.EN_PIN = EN_pin;


    
    
    
    //Verify connection to the device
    uint8_t dev_ID;
    read_register(&IMU_dev, CHIP_ID_REG, &dev_ID);
    sleep_ms(1);

    //Soft reset device
    write_register(&IMU_dev, CMD_REG, SOFT_RESET);
    //Sleep for approx time it takes device to reset
    sleep_ms(SOFT_RESET_WAIT_MS);




    //BY DEFAULT ACCEL/GYRO ARE OFF ON BOOT
    boot_def_accel(&IMU_dev, true);


    boot_def_gyr(&IMU_dev, true);
   



    printf("Device initialised\n");

    return IMU_dev;
}

int boot_def_accel(bmi160 *dev, bool load_cfg){

    if(load_cfg){
    //Set the accelerometers default parameters (100 hz sampling)
    write_register(dev, ACC_CFG, ACC_DEF_CFG);
    }

    printf("%04x", get_err(dev));

     //Turn on the accelerometer 
    write_register(dev, CMD_REG, ACCEL_ON);

    sleep_ms(1);

    //Wait until the register has been flushed
    int timeout = 0;
    while(!is_cmd_flushed(dev)){
        sleep_us(5000);
        timeout++;
        if (timeout > 50){
            printf("CMD TIMEOUT\n");
            break;
        }
    }

    sleep_ms(1);

}

int boot_def_gyr(bmi160 *dev, bool load_cfg){
    

    if (load_cfg){
        //Write the default config to the gyr reg
        write_register(dev, GYR_CFG, GYR_DEF_CFG);
    }


    //Turn on the gyroscope
    write_register(dev, CMD_REG, GYR_ON);
    sleep_ms(1);


    //Wait until the register has been flushed
    int timeout = 0;
    while(!is_cmd_flushed(dev)){
        sleep_us(5000);
        timeout++;
        if (timeout > 50){
            printf("CMD TIMEOUT\n");
            break;
        }
    }
}

//Return the error reg value
uint8_t get_err(bmi160 *dev){

    uint8_t err_buf;
    read_register(dev, ERR_REG, &err_buf);

    return err_buf;


}

//Updates a value in a register
int write_register(bmi160 *dev, uint8_t reg, uint8_t data){


    uint8_t data_to_write[2];
    data_to_write[0] = reg;
    data_to_write[1] = data;

    //Write addr then index then data to the VL53l0 device
    int succ = i2c_write_blocking(dev->I2C_HW, BMI160_ADDR, data_to_write, 2, false);

    //Check whether the byte was written
    if (succ == 2){
        return 1;
    }else{
        printf("FAILED TO WRITE VAL (%04x) TO REG  (%04x) - %d\n", data, reg, succ);
        return succ;
    }
}

//Read a single byte from the VL53l0 device
int read_register(bmi160 *dev, uint8_t reg, uint8_t *buf){

    //Indicates a succesful write
    int succ = i2c_write_blocking(dev->I2C_HW, BMI160_ADDR, &reg, 1, true);

    //Check that the write was succesful
    if (succ == 1){
        //Read from the i2c device
        succ = i2c_read_blocking(dev->I2C_HW, BMI160_ADDR, buf, 1, false);

    }else{
        return succ;
    }    

     //Check whether the byte was read
    if (succ == 1){
        return 1;
    }else{
        printf("%04x -READ ERR - %d\n", reg, succ);
        return succ;
    }
}


//Returns whether the acceleration data flag is set
bool is_acc_ready(bmi160 *dev){

    const uint8_t ACC_MASK = 0x80;

    uint8_t rdy_buf;

    read_register(dev, STATUS_REG, &rdy_buf);

    printf("%x - ", rdy_buf);


    //Mask with 7th bit, shift to end
    return (rdy_buf & ACC_MASK) >> 7;


}

//Returns whether the gyroscope data flag is set
bool is_gyr_ready(bmi160 *dev){

    const uint8_t GYR_MASK = 0x40;

    uint8_t rdy_buf;

    read_register(dev, STATUS_REG, &rdy_buf);

    return (rdy_buf & GYR_MASK) >> 6;

}

//Returns whether the cmd register has been flushed
bool is_cmd_flushed(bmi160 *dev){

    //Get the command register value
    uint8_t cmd_val;
    read_register(dev, CMD_REG, &cmd_val);
    
    //If the value is higher that 0, cmd not flushed
    return (cmd_val == 0);
}