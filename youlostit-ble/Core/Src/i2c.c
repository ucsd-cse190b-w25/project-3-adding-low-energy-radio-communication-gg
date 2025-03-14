/*
* i2c.c
*
*  Created on: Feb 2, 2025
*      Author: bryce
*/

#include "i2c.h"


void i2c_init(void) {
    // 1) Enable GPIO clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

    // 2) Enable I2C2 clock so we can configure its registers
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
	
	
	// Source: https://cseweb.ucsd.edu/classes/fa23/cse190-e/docs/stm32l4X-reference-manual.pdf
	//         6.4.19 APB1 peripheral clock enable register 1 (RCC_APB1ENR1)
	//         I2C2 EN is in position 22
	// Source: https://raw.githubusercontent.com/stm32duino/Arduino_Core_STM32/refs/heads/main/system/Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l475xx.h
	//         Macro defined as RCC_APB1ENR1_I2C2EN
	
	
	// Setting up GPIO
	// Chapter 8.4 Table 39.
	GPIOB->MODER &= ~(GPIO_MODER_MODE10 | GPIO_MODER_MODE11);  // Clear mode bits
	GPIOB->MODER |= GPIO_MODER_MODE10_1; // Pin B10 to AF mode
	GPIOB->MODER |= GPIO_MODER_MODE11_1; // Pin B11 to AF mode
	
	// Explicitly set to pull up mode to increase speed
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD10 | GPIO_PUPDR_PUPD11);
	GPIOB->PUPDR |= (GPIO_PUPDR_PUPD10_0 | GPIO_PUPDR_PUPD11_0);
	
	// Set the AF mode to mode 4
	// Source: https://cseweb.ucsd.edu/classes/fa23/cse190-e/docs/stm32l475-datasheet.pdf
	//         Table 17, column I2C2
	// Source: https://cseweb.ucsd.edu/classes/fa23/cse190-e/docs/stm32l4X-reference-manual.pdf
	//         8.5.10 GPIO alternate function high register
	//         Pin 11 is bits 15 to 12
	//         Pin 10 is bits 11 to 8
	// For pin 10 (AFR index 1, bits [11:8])
	GPIOB->AFR[1] &= ~(0xF << 8); // clear
	GPIOB->AFR[1] |=  (0x4 << 8); // set to AF4
	
	// For pin 11 (AFR index 1, bits [15:12])
	GPIOB->AFR[1] &= ~(0xF << 12); // clear
	GPIOB->AFR[1] |=  (0x4 << 12); // set to AF4
	
	I2C2->CR1 &= ~I2C_CR1_PE; // Disable I2C2 before configuration
	
	// Source: https://cseweb.ucsd.edu/classes/fa23/cse190-e/docs/stm32l4X-reference-manual.pdf
	//         39.7.5 I2C timing register
	//         39.4.3 I2C clock requirements
	//         Bunch o Formulas
	//         6.2.8 System clock (SYSCLK) selection
	//         Default is 4MHz
	
	// Assume Prescaler is set to /1 division (4MHz from APB1)
	
	// Given that we have prescaler set to 1, the input I2C clock is 4MHz.
	// This means the period is 250ns
	// For 100 kHz I2C, the period is: 1 / (100KHz) = 10uS
	
	// t_PRESC = (1 + 1)*250ns = 500ns (0.5uS)
	I2C2->TIMINGR &= ~I2C_TIMINGR_PRESC;  // Clear bits 31:28 (PRESC field)
	I2C2->TIMINGR |=  (1 << I2C_TIMINGR_PRESC_Pos);  // Set PRESC = ???
	
	// t_SCLDEL = (SCLDEL+1) * t_PRESC
	I2C2->TIMINGR &= ~I2C_TIMINGR_SCLDEL;  // Clear SCLDEL
	I2C2->TIMINGR |=  (2 << I2C_TIMINGR_SCLDEL_Pos);  // Set SCLDEL = 2
	
	// t_SDADEL = SDADEL * t_PRESC
	I2C2->TIMINGR &= ~I2C_TIMINGR_SDADEL;  // Clear SDADEL
	I2C2->TIMINGR |=  (2 << I2C_TIMINGR_SDADEL_Pos);  // Set SDADEL = 2
	
	// t_SCLL = (SCLL+1) * t_PRESC
	I2C2->TIMINGR &= ~I2C_TIMINGR_SCLL;  // Clear SCLL
	I2C2->TIMINGR |=  (9 << I2C_TIMINGR_SCLL_Pos);  // Set SCLL = ???
	
	// t_SCLH = (SCLH+1) * t_PRESC
	I2C2->TIMINGR &= ~I2C_TIMINGR_SCLH;  // Clear SCLH
	I2C2->TIMINGR |=  (9 << I2C_TIMINGR_SCLH_Pos);  // Set SCLH = ???
	
	I2C2->CR1 |= I2C_CR1_PE;  // Enable I2C2

	// 6) Disable the clock after configuration if you want it off by default
    //    (The config registers generally retain their values unless the MCU resets or the peripheral is forced reset)
    RCC->APB1ENR1 &= ~RCC_APB1ENR1_I2C2EN;  
}


uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len) {
    // 🔧 Enable the I2C2 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;

    // If you disabled I2C in i2c_init(), re-enable it now
    // (i.e., set PE bit if it’s not already set)
    if ((I2C2->CR1 & I2C_CR1_PE) == 0) {
        I2C2->CR1 |= I2C_CR1_PE;
    }

    // ---- Proceed with your transaction logic ----
    // Clear previous config
    I2C2->ICR = I2C_ICR_STOPCF;
    I2C2->CR2 = 0;

    // 7-bit address
    I2C2->CR2 |= ((uint32_t)address << 1) & I2C_CR2_SADD;
    
    // Direction
    if (dir) {
        I2C2->CR2 |= I2C_CR2_RD_WRN;  // Read
    }

    // Number of bytes
    I2C2->CR2 |= ((uint32_t)len << I2C_CR2_NBYTES_Pos);
    // Auto-end
    I2C2->CR2 |= I2C_CR2_AUTOEND;
    // Start condition
    I2C2->CR2 |= I2C_CR2_START;

    // Write or read data
    if (dir == 0) {
        // Writing
        for (uint8_t i = 0; i < len; i++) {
            while (!(I2C2->ISR & I2C_ISR_TXIS));
            I2C2->TXDR = data[i];
        }
    } else {
        // Reading
        for (uint8_t i = 0; i < len; i++) {
            while (!(I2C2->ISR & I2C_ISR_RXNE));
            data[i] = I2C2->RXDR;
        }
    }

    // Wait for STOP
    while (!(I2C2->ISR & I2C_ISR_STOPF));
    I2C2->ICR = I2C_ICR_STOPCF;

    // 🔧 Disable the I2C peripheral (optional)
    I2C2->CR1 &= ~I2C_CR1_PE;

    // 🔧 Gate off I2C2 clock to save power
    RCC->APB1ENR1 &= ~RCC_APB1ENR1_I2C2EN;

    return 0;
}


