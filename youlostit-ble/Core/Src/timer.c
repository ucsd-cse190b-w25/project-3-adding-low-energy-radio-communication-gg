/*
* timer.c
*
*  Created on: Oct 5, 2023
*      Author: schulman
*/

#include "timer.h"


void lptim_init(LPTIM_TypeDef *lptim)
{
    // 💡 Enable clock for LPTIM1 (assume we're using LPTIM1)
    RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;
    
    // 🔧 Disable LPTIM before configuration
    lptim->CR &= ~LPTIM_CR_ENABLE;
    
    // 💯 Enable the LSI oscillator
    RCC->CSR |= RCC_CSR_LSION;
    while (!(RCC->CSR & RCC_CSR_LSIRDY)) {
        // Wait until LSI is ready
    }
    
    // 💯 Select LSI as the clock source for LPTIM1
    // Clear the LPTIM1 clock selection bits and set them for LSI.
    RCC->CCIPR &= ~RCC_CCIPR_LPTIM1SEL;
    RCC->CCIPR |= RCC_CCIPR_LPTIM1SEL_0;  // Assuming '01' selects LSI (check datasheet!)



    // For example, no prescaler (division factor = 1)
    // Clear the prescaler bits
    lptim->CFGR &= ~LPTIM_CFGR_PRESC;
    // Set prescaler to /128 (0b111 for PRESC bits)
    lptim->CFGR &= ~LPTIM_CFGR_PRESC;                   // Clear prescaler bits
    lptim->CFGR |= (0x7 << LPTIM_CFGR_PRESC_Pos);         // Set PRESC to 0b111 for /128


    

    // 6) Enable ARRM interrupt
    lptim->IER |= LPTIM_IER_ARRMIE;

    
    // Configure NVIC for LPTIM1 interrupt
    NVIC_SetPriority(LPTIM1_IRQn, 2);
    NVIC_EnableIRQ(LPTIM1_IRQn);
    
    // Enable LPTIM and start the counter in continuous mode
    lptim->CR |= LPTIM_CR_ENABLE;

    // The LPTIM_ARR register must only be modified when the LPTIM is enabled (ENABLE bit set to ‘1’)
    // Set auto-reload value for a 5-second period
    lptim->ARR = 1249;  // (250 Hz * 5 sec) - 1 = 1249


    lptim->CR |= LPTIM_CR_CNTSTRT;

    
}


void timer_reset(TIM_TypeDef* timer) {}

void timer_set_ms(TIM_TypeDef* timer, uint16_t period_ms) {}
