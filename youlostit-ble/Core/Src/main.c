/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under BSD 3-Clause license,
* the "License"; You may not use this file except in compliance with the
* License. You may obtain a copy of the License at:
*                        opensource.org/licenses/BSD-3-Clause
*
******************************************************************************
*/
#include "lsm6dsl.h"
#include "i2c.h"
#include "timer.h"
#include <stdbool.h>
#include <string.h>

volatile bool timer_event = false;

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
//#include "ble_commands.h"
#include "ble.h"

#include <stdlib.h>

int dataAvailable = 0;

SPI_HandleTypeDef hspi3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);

/**
* @brief  The application entry point.
* @retval int
*/
int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
    
    /* Configure the system clock */
    SystemClock_Config();
    
    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_SPI3_Init();
    
    //RESET BLE MODULE
    HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_SET);
    
    ble_init();
    
    HAL_Delay(10);

    // --- set deep sleep --- 
    // Clear existing LPMS bits
    PWR->CR1 &= ~PWR_CR1_LPMS;
    // Set LPMS = 010 for Stop 2
    PWR->CR1 |= (0x2 << PWR_CR1_LPMS_Pos);
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    // Clear the LPR bit in PWR->CR1 (if set)
    // PWR->CR1 &= ~PWR_CR1_LPR;
    // --- end set deep sleep ---

    
    uint8_t nonDiscoverable = 0;
    
    // Variables for accelerometer readings
    int16_t ax, ay, az;
    int16_t prev_ax, prev_ay, prev_az;
    // Perform an initial reading of the accelerometer
    lsm6dsl_read_xyz(&ax, &ay, &az);
    prev_ax = ax;
    prev_ay = ay;
    prev_az = az;
    // Variables for movement detection.
    uint32_t no_movement_count = 0;
    const uint16_t MOVEMENT_THRESHOLD = 1500; // adjustable
    const uint32_t NO_MOVEMENT_REQUIRED = 3; // interrupt every 5 seconds. 12 ticks for 1 minute
    
    // lost mode flag
    bool lost_mode = false;
    
    setDiscoverability(0);
    
    while (1)
    {
        
        // Timer interrupt event
        if (timer_event) {
            // --------- Movement Tracking stuff
            timer_event = false;
            // get change in acceleration
            lsm6dsl_read_xyz(&ax, &ay, &az);
            int16_t diff_x = (int16_t)abs(ax - prev_ax);
            int16_t diff_y = (int16_t)abs(ay - prev_ay);
            int16_t diff_z = (int16_t)abs(az - prev_az);
            
            // check for no movement
            if ((diff_x < MOVEMENT_THRESHOLD) && (diff_y < MOVEMENT_THRESHOLD) && (diff_z < MOVEMENT_THRESHOLD)) {
                no_movement_count++;
                printf("Not moving for %ld / %d interrupts\n", no_movement_count, (int)NO_MOVEMENT_REQUIRED);
            } else {
                if (no_movement_count >= NO_MOVEMENT_REQUIRED) {
                    // disconnect ble and set to non discoverable
                    printf("Moving, disconnecting BLE\n");
                    disconnectBLE();
                    printf("Setting to non discoverable\n");
                    setDiscoverability(0);
                }
                no_movement_count = 0;
                lost_mode = false;
            }
            prev_ax = ax;
            prev_ay = ay;
            prev_az = az;
            // check if havent been moving for a minute
            if (no_movement_count >= NO_MOVEMENT_REQUIRED) {
                lost_mode = true;
            }
            
            
            // ------- BLE stuff
            if (lost_mode) {
                // first time in lost mode
                if (no_movement_count == NO_MOVEMENT_REQUIRED) {
                    printf("Entering lost mode\n");
                    setDiscoverability(1);
                }
                // ble given if statement
                if(!nonDiscoverable && HAL_GPIO_ReadPin(BLE_INT_GPIO_Port,BLE_INT_Pin)){
                    catchBLE();
                }else{
                    if (no_movement_count % 2 == 0) { // 10 seconds
                        // Send a string to the NORDIC UART service, remember to not include the newline character
                        unsigned char test_str[60];
                        strcpy((char *)test_str, "PrivTagObama");
                        printf("%s\n", test_str);
                        updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, strlen((char *)test_str), test_str);
                        
                        sprintf((char *)test_str, "Missing for %lus", (no_movement_count - NO_MOVEMENT_REQUIRED) * 5);
                        printf("%s\n", test_str);
                        updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, strlen((char *)test_str), test_str);
                    }
                }
            }
        }
        
        // Wait for interrupt, only uncomment if low power is needed
        if (!lost_mode) __WFI();
        // exited wfi
        
    }
}

/**
* @brief System Clock Configuration
* @attention This changes the System clock frequency, make sure you reflect that change in your timer
* @retval None
*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    
    /** Configure the main internal regulator output voltage
    */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler();
    }
    
    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    // This lines changes system clock frequency
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
* @brief SPI3 Initialization Function
* @param None
* @retval None
*/
static void MX_SPI3_Init(void)
{
    
    /* USER CODE BEGIN SPI3_Init 0 */
    
    /* USER CODE END SPI3_Init 0 */
    
    /* USER CODE BEGIN SPI3_Init 1 */
    
    /* USER CODE END SPI3_Init 1 */
    /* SPI3 parameter configuration*/
    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 7;
    hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&hspi3) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI3_Init 2 */
    
    /* USER CODE END SPI3_Init 2 */
    
}

/**
* @brief GPIO Initialization Function
* @param None
* @retval None
*/
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    
    i2c_init();
    lsm6dsl_init();
    lptim_init(LPTIM1);
    
    /* USER CODE END MX_GPIO_Init_1 */
    
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIO_LED1_GPIO_Port, GPIO_LED1_Pin, GPIO_PIN_RESET);
    
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(BLE_CS_GPIO_Port, BLE_CS_Pin, GPIO_PIN_SET);
    
    
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);
    
    /*Configure GPIO pin : BLE_INT_Pin */
    GPIO_InitStruct.Pin = BLE_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BLE_INT_GPIO_Port, &GPIO_InitStruct);
    
    /*Configure GPIO pins : GPIO_LED1_Pin BLE_RESET_Pin */
    GPIO_InitStruct.Pin = GPIO_LED1_Pin|BLE_RESET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /*Configure GPIO pin : BLE_CS_Pin */
    GPIO_InitStruct.Pin = BLE_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(BLE_CS_GPIO_Port, &GPIO_InitStruct);
    
    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    
    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// LPTIM1 interrupt handler
void LPTIM1_IRQHandler(void)
{
    // check for armie match interrupt flag
    if (LPTIM1->ISR & LPTIM_ISR_ARRM) {
        LPTIM1->ICR = LPTIM_ICR_ARRMCF;  // Clear the ARR flag
        timer_event = true;
    }
}

/* USER CODE END 4 */

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


// Redefine the libc _write() function so you can use printf in your code
int _write(int file, char *ptr, int len) {
    int i = 0;
    for (i = 0; i < len; i++) {
        ITM_SendChar(*ptr++);
    }
    return len;
}

