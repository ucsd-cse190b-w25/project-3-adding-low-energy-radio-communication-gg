/*
 * lsm6dsl.h
 *
 *  Created on: Feb 9, 2025
 *      Author: bryce
 */

#ifndef LSM6DSL_H_
#define LSM6DSL_H_

#include <stdint.h>
#include <stdio.h>

// Configure and enable the LSM6DSL. Refer to section 4.1 of the LSM6DSL application note.
void lsm6dsl_init();

// Read the current X,Y, and Z acceleration data from the accelerometer. This data will 
// reflect the orientation and movement of the hardware. Refer to the data sheet for the 
// units that these values are reported in.
void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z);



#endif /* LSM6DSL_H_ */
