/*
 * i2c.h
 *
 *  Created on: Feb 2, 2025
 *      Author: bryce
 */

#ifndef I2C_H_
#define I2C_H_


#include <stm32l475xx.h>
#include <stdint.h>
#include <stdio.h>

// Initialize I2C2
void i2c_init();

// Perform an I2C transaction. This function should be able to both read and write data.
// The parameter 'dir' should be used to specify the transaction direction.
// The parameter 'data' should be a pointer to the data to be written or the buffer where the read data should be placed.
// The parameter 'len' should be the number of bytes to read or write.
uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len);



#endif /* I2C_H_ */
