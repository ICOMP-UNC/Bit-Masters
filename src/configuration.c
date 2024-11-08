#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>

#include "configuration.h"

void config_i2c(void)
{
    // Enable clock for GPIOB and I2C1
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_I2C1);

    // GPIO pins configuration for SDA and SCL 
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
              GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
              GPIO_I2C1_SCL | GPIO_I2C1_SDA);

    // Disable I2C1 before configurating it
    i2c_peripheral_disable(I2C1);

    // I2C basic configuration
    i2c_set_clock_frequency(I2C1, I2C_CR2_FREQ_36MHZ);
    i2c_set_standard_mode(I2C1);
    i2c_set_trise(I2C1, I2C1_TRISE_100KHZ); // Rising time in standard mode
    i2c_set_ccr(I2C1, I2C1_CCR_100KHZ);  // Set the CCR to 100 kHz

    // Enable I2C to start communication
    i2c_peripheral_enable(I2C1);
}