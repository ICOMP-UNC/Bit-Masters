#include "alarm.h"

uint16_t get_battery_value(void) {
    /* Configure the ADC */
    adc_set_regular_sequence(ADC1, 1, ADC_CHANNEL_BATTERY_LEVEL); /* Select only one channel */

    /* Start the conversion */
    adc_start_conversion_regular(ADC1);

    /* Wait for the conversion to complete */
    while (!(ADC1_SR & ADC_SR_EOC));

    /* Read the conversion result */
    return adc_read_regular(ADC1) * CONVERT_VALUE;
}



