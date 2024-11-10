/**
 * @file alarm.h
 * @brief alarm header file
 * 
 */
#pragma once

#include <libopencm3/stm32/adc.h> /**< Include the ADC library */
#include "system_response.h" /**< Include the system response header file */

#define CONVERT_VALUE (100/4095) /**< The maximum value of the ADC */

/**
 * @brief Get the battery value and convert it to a percentage
 * 
 * @return uint16_t 
 */
uint16_t get_battery_value(void);
