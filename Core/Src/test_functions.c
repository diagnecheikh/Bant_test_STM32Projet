/*
 * test_functions.c
 *
 *  Created on: Jun 18, 2025
 *      Author: cheik
 */

#include "main.h"
#include "test_functions.h"
#include <stdio.h> // Pour snprintf

void test_system(void) {
    char msg[50];
    if (RCC->CR & RCC_CR_HSIRDY) {
        snprintf(msg, sizeof(msg), "SYSTEM OK: HSI Ready\n\n");
    } else {
        snprintf(msg, sizeof(msg), "SYSTEM ERROR: HSI Not Ready\n");
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    HAL_Delay(300);
}

void test_led(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_Delay(1000);

    char msg[] = "LED OK\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void test_button(void)
{
    GPIO_PinState state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

    if (state == GPIO_PIN_RESET) {
        char msg[] = "BUTTON PRESSED\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    } else {
        char msg[] = "BUTTON RELEASED\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
}

void test_uart(void) {
    char msg[50];
    snprintf(msg, sizeof(msg), "UART OK: Transmission sent\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void test_adc(void) {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        uint32_t adc_val = HAL_ADC_GetValue(&hadc1);
        float voltage = adc_val * 3.3 / 4096;
        char msg[50];
        int voltage_int = (int)voltage;
        int voltage_dec = (int)((voltage - voltage_int) * 100);
        snprintf(msg, sizeof(msg), "ADC OK: PA0 = %d.%02dV\n", voltage_int, voltage_dec);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    } else {
        HAL_UART_Transmit(&huart2, (uint8_t*)"ADC FAIL\n", 9, HAL_MAX_DELAY);
    }
    HAL_ADC_Stop(&hadc1);
}

void test_spi(void) {
    uint8_t tx = 0x5A;
    uint8_t rx = 0x00;
    char msg[50];

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 1000);

    if (status == HAL_OK && rx == tx) {
        snprintf(msg, sizeof(msg), "SPI OK: Data 0x%02X received\n", rx);
    } else {
        snprintf(msg, sizeof(msg), "SPI FAIL: Error or incorrect data\n");
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void test_i2c(void) {
    uint8_t who_am_i_reg = 0x75;
    uint8_t response = 0;
    char msg[50];
    uint8_t mpu_addr = 0x68 << 1;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, mpu_addr, who_am_i_reg, I2C_MEMADD_SIZE_8BIT, &response, 1, 1000);

    if (status == HAL_OK) {
        if (response == 0x71 || response == 0x70 || response == 0x73) {
            snprintf(msg, sizeof(msg), "I2C OK: MPU WHO_AM_I = 0x%02X\n", response);
        } else {
            snprintf(msg, sizeof(msg), "I2C FAIL: Unexpected WHO_AM_I = 0x%02X\n", response);
        }
    } else {
        snprintf(msg, sizeof(msg), "I2C FAIL: No response\n");
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void test_timer(void) {
    char msg[50];
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    uint32_t duty_cycles[] = {250, 500, 750, 1000};
    const int num_cycles = sizeof(duty_cycles) / sizeof(duty_cycles[0]);

    for (int i = 0; i < num_cycles; ++i) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty_cycles[i]);
        HAL_Delay(500);
    }

    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    snprintf(msg, sizeof(msg), "PWM OK: Duty cycles tested\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void run_all_tests(void) {
    char start_msg[] = "START: Running all tests\n\n";
    char end_msg[] = "END: All tests completed\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)start_msg, strlen(start_msg), HAL_MAX_DELAY);
    HAL_Delay(10);

    test_system();
    test_led();
    test_button();
    test_uart();
    test_adc();
    test_spi();
    test_i2c();
    test_timer();

    HAL_UART_Transmit(&huart2, (uint8_t*)end_msg, strlen(end_msg), HAL_MAX_DELAY);
    HAL_Delay(5000);
}
