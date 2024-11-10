/**
 * @file alarm.h
 * @brief alarm header file
 * 
 */
#pragma once

#include <libopencm3/stm32/adc.h> /**< Include the ADC library */
#include "system_response.h" /**< Include the system response header file */

#define CONVERT_VALUE (100/4095) /**< The maximum value of the ADC */

uint16_t get_channel_value(uint8_t channel);
