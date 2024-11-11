/**
 * @file alarm.h
 * @brief alarm header file
 * 
 */
#pragma once

#include <libopencm3/stm32/adc.h> /**< Include the ADC library */
#include <libopencm3/stm32/i2c.h> /**< For I2C peripheral */
#include "system_response.h" /**< Include the system response header file */
#include "configuration.h" /**< Include the configuration header file */

#define CONVERT_VALUE (100/4095) /**< The maximum value of the ADC */

#define WRITE 0 /**< Write mode */
#define READ 1  /**< Read mode */

#define DISPLAY_UNIT_ADDRESS 0x3F /**< Display unit address */
#define DISPLAY_TENS_ADDRESS 0x3E /**< Display tens address */

#define DISPLAY_0 0x3F /**< Display "0" */
#define DISPLAY_1 0x06 /**< Display "1" */
#define DISPLAY_2 0x5B /**< Display "2" */
#define DISPLAY_3 0x4F /**< Display "3" */
#define DISPLAY_4 0x66 /**< Display "4" */
#define DISPLAY_5 0x6D /**< Display "5" */
#define DISPLAY_6 0x7D /**< Display "6" */
#define DISPLAY_7 0x07 /**< Display "7" */
#define DISPLAY_8 0x7F /**< Display "8" */
#define DISPLAY_9 0x6F /**< Display "9" */

#define LENGTH 1 /**< Length of the display buffer */

#define MAX_TEMP 99 /**< Maximum temperature value */
#define DISPLAY_E 0x79 /**< Display "E" for error */
#define DISPLAY_R 0x31 /**< Display "r" for error */

#define TEN 10 /**< Ten value */

/**
 * @brief Get the battery value and convert it to a percentage
 * 
 * @return uint16_t 
 */
uint16_t get_battery_value(void);

/**
 * @brief Convert a value to the display format
 * @param value The value to convert
 * @return The value in the display format
 */
uint8_t convert_to_display(uint8_t value);

/**
 * @brief Obtain the temperature value
 * @return The unit value in the display format
 */
uint8_t obtain_unit_value(void);

/**
 * @brief Obtain the temperature value
 * @return The tens value in the display format
 */
uint8_t obtain_tens_value(void);

/**
 * @brief Display the temperature value
 * @param value The value to display
 * @param address The address of the device
 * @param i2c The I2C peripheral
 */
void display_temperature(uint8_t value, uint8_t address, uint32_t i2c);

/**
 * @brief Update the I2C value
 * @param i2c The I2C peripheral
 * @param data The data to update
 * @param address The address of the device
 */
void update_i2c_value(uint32_t i2c, uint8_t data, uint8_t address);

/**
 * @brief Show the display
 */
void show_display(void);

/**
 * @brief Activates the alarm
 * 
 * This function checks if the alarm is already active and sets the alarm pin high if it is not.
 * 
 */
void activate_alarm(void);

/**
 * @brief Deactivates the alarm
 * 
 * This function checks if the alarm is already inactive and sets the alarm pin low if it is not.
 * 
 */
void deactivate_alarm(void);
