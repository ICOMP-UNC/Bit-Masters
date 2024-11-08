#pragma once

/**
 * @brief I2C1 rise time in standard mode (100 kHz).
 * 
 * This define sets the maximum rise time for the I2C1 peripheral in 
 * standard mode (100 kHz) according to the I2C specifications. Adjust 
 * if the system clock changes.
 */
#define I2C1_TRISE_100KHZ 36

/**
 * @brief I2C1 CCR value for 100 kHz.
 * 
 * This define configures the Clock Control Register (CCR) of the 
 * I2C1 peripheral for standard mode at a frequency of 100 kHz. 
 * Adjust if a different speed is required.
 */
#define I2C1_CCR_100KHZ 180

/**
 * @brief Configures the I2C1 peripheral on the STM32.
 *
 * This function sets up the I2C1 peripheral on the STM32 microcontroller to operate
 * in standard mode (100 kHz) with GPIO pins configured for open-drain alternate function.
 * It enables the clocks for GPIOB and I2C1, configures the necessary pins for I2C communication,
 * sets the I2C clock frequency, rise time, and CCR value for a 100 kHz bus speed.
 *
 * @note Assumes a system clock frequency of 36 MHz for proper timing configuration.
 *       Adjustments may be necessary for different clock speeds.
 *
 * GPIOB is used for I2C1_SCL and I2C1_SDA lines with open-drain configuration.
 * Before enabling the I2C peripheral, it is disabled to ensure a clean configuration.
 */
void config_i2c(void);