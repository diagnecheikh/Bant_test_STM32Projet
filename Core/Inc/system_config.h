/*
 * system_config.h
 *
 *  Created on: Jun 18, 2025
 *      Author: cheik
 */

#ifndef INC_SYSTEM_CONFIG_H_
#define INC_SYSTEM_CONFIG_H_

#include "main.h"

void SystemClock_Config(void);
void MX_ADC1_Init(void);
void MX_I2C1_Init(void);
void MX_SPI2_Init(void);
void MX_TIM2_Init(void);
void MX_USART2_UART_Init(void);
void MX_GPIO_Init(void);

#endif /* INC_SYSTEM_CONFIG_H_ */
