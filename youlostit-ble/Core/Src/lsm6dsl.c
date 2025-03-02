/*
 * lsm6dsl.c
 *
 *  Created on: Feb 9, 2025
 *      Author: bryce
 */

#include "lsm6dsl.h"
#include "i2c.h"

#define LSM6DSL_ADDR   0x6A   // 7-bit I2C address
#define CTRL1_XL       0x10   // Accelerometer control register
#define OUTX_L_XL      0x28   // Starting register address for accelerometer output

void lsm6dsl_init() {
	printf("Init accelerometer\n");
    uint8_t data[2];

    // Write 416 Hz mode to the accelerometer control register
    data[0] = CTRL1_XL;  
    data[1] = 0x60;      
    i2c_transaction(LSM6DSL_ADDR, 0, data, 2);
}

void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z) {
    // Write the starting register address
    uint8_t reg = OUTX_L_XL;
    i2c_transaction(LSM6DSL_ADDR, 0, &reg, 1);

    // Read 6 bytes
    uint8_t buffer[6];
    i2c_transaction(LSM6DSL_ADDR, 1, buffer, 6);

    // Combine back (Little Indians)
    *x = (int16_t)((buffer[1] << 8) | buffer[0]);
    *y = (int16_t)((buffer[3] << 8) | buffer[2]);
    *z = (int16_t)((buffer[5] << 8) | buffer[4]);
}
