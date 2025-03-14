/*
* timer.c
*
*  Created on: Oct 5, 2023
*      Author: schulman
*/

#include "timer.h"


void timer_init(TIM_TypeDef* timer)
{
    
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;  // Enable clock for TIM2
    timer->CR1 = 0x0;
    
    timer->CNT = 0;
    timer->SR = 0;
    timer->DIER = 0;
    timer->PSC = 0;
    timer->ARR = 0xFFFFFFFF;
    
    timer->DIER |= 0x1; // enable "Update Interrupt"
    
    // Configure interrupt controller (NVIC) for TIM2
    NVIC_SetPriority(TIM2_IRQn, 2);  // Set priority (lower value = higher priority)
    NVIC_EnableIRQ(TIM2_IRQn);       // Enable TIM2 interrupt in NVIC
    
    
    TIM2->PSC = 3999;  // Set prescaler to divide clock by 4000 (because 4mhz clock -> 1ms tick rate
    
    TIM2->ARR = 999;  // Set auto-reload value for 1-second period
    
    TIM2->CR1 |= TIM_CR1_CEN;  // Enable the timer
    
}

void timer_reset(TIM_TypeDef* timer)
{
    timer->CNT = 0;
    
}

void timer_set_ms(TIM_TypeDef* timer, uint16_t period_ms)
{
    timer->CR1 = 0;
    
    // Change ARR
    timer->ARR = period_ms - 1;  // ARR counts from 0 to period_ms - 1
    
    timer->EGR = TIM_EGR_UG;  // Force an update event to load PSC and ARR
    
    timer->CR1 |= TIM_CR1_CEN;  // Enable the timer
    
}
