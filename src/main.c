/*
 * @file main.c
 * @brief Toggles the LED on PC13 using a SysTick timer and EXTI0 interrupt.
 * Enable GPIO interrupt on PA0 to toggle the LED state on both rising and falling edges.
 * The onboard LED (connected to PC13 on STM32F103C8T6 Blue Pill) will blink using SysTick and toggle using EXTI0.
 * This file is based on examples from the libopencm3 project.
 */

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <stdio.h>
#include <stdarg.h> 

// Free RTOS headers
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Constants */
#define TRUE    1
#define FALLING 0
#define RISING  1

/* Pin Definitions */
#define MOTOR_NEG_PORT    GPIOA
#define MOTOR_NEG_PIN     GPIO7 /* PC13 connected to onboard LED */
#define MOTOR_POS_PORT    GPIOA
#define MOTOR_POS_PIN     GPIO6 /* PC13 connected to onboard LED */
#define SWITCH_PORT GPIOA
#define SWITCH_PIN  GPIO0 /* PA0 connected to button (switch) */
#define OVERRIDE_PORT GPIOA
#define OVERRIDE_PIN  GPIO1 /* PA1 connected to button (switch) */
#define MOTION_SENSOR_PORT GPIOA
#define MOTION_SENSOR_PIN  GPIO2 /* PA1 connected to button (switch) */
#define FIRE_SENSOR_PORT GPIOA
#define FIRE_SENSOR_PIN  GPIO3 /* PA1 connected to button (switch) */
#define ALARM_PORT GPIOA
#define ALARM_PIN  GPIO8 /* PA1 connected to button (switch) */
#define LED_PORT GPIOB
#define LED_PIN  GPIO8 /* PB6 connected to LED */
#define FAN_PORT GPIOB
#define FAN_PIN  GPIO9 /* PB6 connected to LED */
#define PRESCALER_VALUE 71
#define TIMER_PERIOD 999
#define ADC_MAX_VALUE 4095
#define PERCENTAGE 100
#define TRUE 1
#define FALSE 0
#define ADC_BUFFER_SIZE 
#define TX_BUFFER_SIZE 128  // Tamaño del buffer de transmisión DMA
#define ONE_MINUTE_DELAY 60000 /**< 60000 ms */

/* Task names */
#define TEMPERATURE_CONTROL_TASK_NAME       "temperature_control"
#define BATTERY_CONTROL_TASK_NAME           "battery_control"

#define CLOSE_DOOR_TASK_NAME                "close_door"
#define OPEN_DOOR_TASK_NAME                 "open_door"

/* Task priorities */
#define TEMPERATURE_CONTROL_PRIORITY        tskIDLE_PRIORITY + 2
#define BATTERY_CONTROL_PRIORITY            tskIDLE_PRIORITY + 1

#define CLOSE_DOOR_PRIORITY                 tskIDLE_PRIORITY + 2
#define OPEN_DOOR_PRIORITY                  tskIDLE_PRIORITY + 2

#define CLOSED 1
#define OPEN 0


static char tx_buffer[TX_BUFFER_SIZE];  // Buffer de transmisión
static volatile uint8_t dma_tx_busy = 0;  // Flag para indicar si el DMA está ocupado

static uint32_t duty_cycle = 0;
static uint32_t temp_value = 0;

/* EXTI state direction (falling or rising edge detection) */
static uint16_t exti_direction_0 = FALLING;
static uint16_t exti_direction_1 = FALLING;
static uint16_t exti_direction_2 = FALLING;
static uint16_t exti_direction_3 = FALLING;

static uint32_t manual_controls = 0; /**< Variable to store the manual control state */

static int get_batt = 0;
static int get_temp = 0;

static uint16_t adc_data = 0; /**< Buffer to store ADC data */

static door_state = OPEN;

SemaphoreHandle_t xADC = NULL;
SemaphoreHandle_t xDoor = NULL;

TaskHandle_t closeDoorTaskHandle = NULL;
TaskHandle_t openDoorTaskHandle = NULL;

/**
 * @brief Initializes the system clock to 72 MHz using an 8 MHz external crystal.
 */
void system_clock_setup(void)
{
    /* Configure the system clock to 72 MHz using PLL and 8 MHz HSE */
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
}

/**
 * @brief Configures GPIO pin PC13 as an output for the onboard LED and PA0 as input for the button (switch).
 */
void gpio_setup(void)
{
    /* Enable GPIO clocks */
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);

    
    gpio_set_mode(MOTOR_NEG_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, MOTOR_NEG_PIN);

    gpio_set_mode(MOTOR_POS_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, MOTOR_POS_PIN);

    gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, LED_PIN);

    gpio_set_mode(FAN_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, FAN_PIN);

    gpio_set_mode(ALARM_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, ALARM_PIN);

    gpio_set_mode(MOTION_SENSOR_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, MOTION_SENSOR_PIN);

    gpio_set_mode(FIRE_SENSOR_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, FIRE_SENSOR_PIN);

    /* Configure PA0 as input (button) with floating input configuration */
    gpio_set_mode(SWITCH_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, SWITCH_PIN);
    /* Configure PA1 as input (button) with floating input configuration */
    gpio_set_mode(OVERRIDE_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, OVERRIDE_PIN);
}

/**
 * @brief Configures EXTI for PA0 to generate interrupts on both falling and rising edges.
 */
void exti_setup(void)
{
    /* Enable AFIO clock (for EXTI configuration) */
    rcc_periph_clock_enable(RCC_AFIO);

    /* Enable EXTI0 interrupt in the NVIC */
    nvic_enable_irq(NVIC_EXTI0_IRQ);
    nvic_enable_irq(NVIC_EXTI1_IRQ);
    nvic_enable_irq(NVIC_EXTI2_IRQ);
    nvic_enable_irq(NVIC_EXTI3_IRQ);

    /* Configure EXTI0 (PA0) for falling edge initially */
    exti_select_source(EXTI0, SWITCH_PORT);        /* Set PA0 as the EXTI0 source */
    exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING); /* Trigger interrupt on falling edge */
    exti_enable_request(EXTI0);                    /* Enable EXTI0 interrupt */

    /* Configure EXTI1 (PA1) for falling edge initially */
    exti_select_source(EXTI1, SWITCH_PORT);        /* Set PA1 as the EXTI1 source */
    exti_set_trigger(EXTI1, EXTI_TRIGGER_FALLING); /* Trigger interrupt on falling edge */
    exti_enable_request(EXTI1);                    /* Enable EXTI1 interrupt */

    /* Configure EXTI2 (PA2) for falling edge initially */
    exti_select_source(EXTI2, SWITCH_PORT);        /* Set PA2 as the EXTI2 source */
    exti_set_trigger(EXTI2, EXTI_TRIGGER_FALLING); /* Trigger interrupt on falling edge */
    exti_enable_request(EXTI2);                    /* Enable EXTI2 interrupt */

    /* Configure EXTI3 (PA3) for falling edge initially */
    exti_select_source(EXTI3, SWITCH_PORT);        /* Set PA3 as the EXTI3 source */
    exti_set_trigger(EXTI3, EXTI_TRIGGER_FALLING); /* Trigger interrupt on falling edge */
    exti_enable_request(EXTI3);                    /* Enable EXTI3 interrupt */
}

// Configuración del ADC para el canal 9 (PB1)
void configure_adc(void)
{
     // Habilitar reloj para el ADC
    rcc_periph_clock_enable(RCC_ADC1);

    // Apagar el ADC antes de configurarlo
    ADC1_CR2 &= ~ADC_CR2_ADON;

    // Configurar la longitud de la secuencia regular a 1 (un canal)
    ADC1_SQR1 &= ~(0xF << 20); // Limpia los bits [23:20] para longitud
    ADC1_SQR1 |= (0 << 20);    // Longitud de la secuencia = 1

    // Configurar tiempo de muestreo para canal 4
    ADC1_SMPR2 &= ~(0x7 << 12); // Limpia los bits [14:12] para canal 4
    ADC1_SMPR2 |= (ADC_SMPR_SMP_1DOT5CYC << 12); // Tiempo de muestreo 1.5 ciclos

    // Configurar tiempo de muestreo para canal 5
    ADC1_SMPR2 &= ~(0x7 << 15); // Limpia los bits [17:15] para canal 5
    ADC1_SMPR2 |= (ADC_SMPR_SMP_1DOT5CYC << 15); // Tiempo de muestreo 1.5 ciclos

    ADC1_CR2 |= ADC_CR2_SWSTART; // Inicia la conversión regular

    ADC1_CR2 |= ADC_CR2_DMA; // Habilita el modo DMA

}


void change_adc_sequence(void){

        ADC1_CR2 &= ~ADC_CR2_ADON; // apaga el adc

        dma_disable_channel(DMA1, DMA_CHANNEL1);

        if(get_batt){
            // Configurar canal 4 en la primera posición de la secuencia regular
            ADC1_SQR3 &= ~0x1F;        // Limpia los bits [4:0]
            ADC1_SQR3 |= (4 << 0);     // Configura canal 4
        }

        if(get_temp){
            // Configurar canal 5 en la primera posición de la secuencia regular
            ADC1_SQR3 &= ~0x1F;        // Limpia los bits [4:0]
            ADC1_SQR3 |= (5 << 0);     // Configura canal 5
        }

        // Encender el ADC
        ADC1_CR2 |= ADC_CR2_ADON;

        dma_enable_channel(DMA1, DMA_CHANNEL1);

}

// Lee el valor actual del ADC en el canal 9
uint16_t get_battery_value(void)
{
    if(xSemaphoreTake(xADC, portMAX_DELAY) == pdTRUE){
    get_batt = 1;
    get_temp = 0;
    change_adc_sequence();
    adc_start_conversion_direct(ADC1);       // Inicia la conversión
    //dma_enable_channel(DMA1, DMA_CHANNEL1);
    xSemaphoreGive(xADC);
    }
    return((adc_data * PERCENTAGE) / ADC_MAX_VALUE);           // Retorna el valor convertido
    
}

uint16_t get_temp_value(void)
{
    if(xSemaphoreTake(xADC, portMAX_DELAY) == pdTRUE){
    get_batt = 0;
    get_temp = 1;
    change_adc_sequence();
    adc_start_conversion_direct(ADC1);       // Inicia la conversión
    //dma_enable_channel(DMA1, DMA_CHANNEL1);
    xSemaphoreGive(xADC);
    }
    return ((adc_data * 30) / ADC_MAX_VALUE);           // Retorna el valor convertido 
}

void configure_usart(void) {
    rcc_periph_clock_enable(RCC_USART1);        // Habilita USART1
    rcc_periph_clock_enable(RCC_GPIOA);         // Habilita GPIOA para USART
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX); // TX
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX); // RX como entrada flotante
    usart_set_baudrate(USART1, 9600);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable_tx_dma(USART1);
    usart_enable(USART1);
}

void dma_setup(void)
{
    
    // Habilita el reloj para el DMA1 y GPIOC
    rcc_periph_clock_enable(RCC_DMA1);
    
    // Configura el dma para transferir datos 
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
    // Origen de los datos a transferir:
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, &ADC_DR(ADC1));
    // Dirección de datos destino. El ODR (Output Data Register) del GPIOA:
    dma_set_memory_address(DMA1, DMA_CHANNEL1, &adc_data);
    // Tamaño del dato a leer
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);
    // Tamaño del dato a escribir
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);


    // Cantidad de datos a transferir:
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, 1);

    // Se incrementa automaticamente la posición en memoria:
    dma_disable_memory_increment_mode(DMA1, DMA_CHANNEL1);
    // La dirección destino se mantiene fija:
    dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL1);
    // Se establece la prioridad del canal 7 del DMA1 como alta:
    dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_VERY_HIGH);
    //Se habilita el modo circular para que la transferencia se repita indefinidamente
    dma_enable_circular_mode(DMA1, DMA_CHANNEL1);

    dma_enable_channel(DMA1, DMA_CHANNEL1);

    dma_disable_channel(DMA1, DMA_CHANNEL4); // Canal asociado al USART1 TX

    dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (uint32_t)&USART1_DR);
    dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t)tx_buffer);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL4, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL4, DMA_CCR_MSIZE_8BIT);
    dma_set_read_from_memory(DMA1, DMA_CHANNEL4);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL4);
    dma_set_priority(DMA1, DMA_CHANNEL4, DMA_CCR_PL_MEDIUM);
    dma_set_number_of_data(DMA1, DMA_CHANNEL4, TX_BUFFER_SIZE);

    dma_enable_channel(DMA1, DMA_CHANNEL4);

    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);
    nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);
}

void dma1_channel4_isr(void) {
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL4, DMA_TCIF)) {
        // Limpia la bandera de interrupción
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL4, DMA_TCIF);
        dma_disable_channel(DMA1, DMA_CHANNEL4);
        dma_tx_busy = 0;
    }
}

void config_pwm(void) 
{
    /* Enable the peripheral clock of GPIOA */
    rcc_periph_clock_enable(RCC_TIM4);

    /* Configure the reset and enable the timer peripheral */
    rcc_periph_reset_pulse(RST_TIM4);

    timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1, TIM_CR1_DIR_UP);

    /* Configure the temporizer to obtain a frequency of 20 kHz */
    timer_set_prescaler(TIM4, PRESCALER_VALUE); 

    /* Configure the temporizer to count up to 999 */
    timer_set_period(TIM4, TIMER_PERIOD);

    /* Enable the output channel 1 as PWM */
    timer_set_oc_mode(TIM4, TIM_OC3, TIM_OCM_PWM2);

    timer_set_oc_mode(TIM4, TIM_OC4, TIM_OCM_PWM2);

    /* Set the output polarity as high */
    timer_set_oc_polarity_high(TIM4, TIM_OC1);

    /* Enable the output for OC2 */
    timer_enable_oc_output(TIM4, TIM_OC3);

    timer_enable_oc_output(TIM4, TIM_OC4);

    /* Enable the break function */
    timer_enable_break_main_output(TIM4);

    /* Enable the temporizer */
    timer_enable_counter(TIM4);
}

void usart_send_string(const char *str, ...) {
    if (dma_tx_busy) {
        // El DMA está ocupado, no podemos enviar
        return;
    }

    char formatted_buffer[TX_BUFFER_SIZE]; // Buffer temporal
    va_list args;
    va_start(args, str);
    int len = vsnprintf(formatted_buffer, sizeof(formatted_buffer), str, args);
    va_end(args);

    if (len < 0 || len >= TX_BUFFER_SIZE) {
        // Error o cadena demasiado grande
        return;
    }

    // Copiar al buffer DMA
    memcpy(tx_buffer, formatted_buffer, len);

    dma_tx_busy = 1; // Marca el DMA como ocupado

    // Configurar DMA
    dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t)tx_buffer);
    dma_set_number_of_data(DMA1, DMA_CHANNEL4, len);

    // Iniciar la transferencia
    dma_enable_channel(DMA1, DMA_CHANNEL4);
}

// Función de interrupción o callback al terminar la transmisión por DMA
void dma_tx_complete_callback(void) {
    dma_tx_busy = 0; // Liberar la bandera de ocupación del DMA
}

void close_door(void){
    gpio_set(MOTOR_NEG_PORT, MOTOR_NEG_PIN); /* Turn on LED on falling edge */
    gpio_clear(MOTOR_POS_PORT, MOTOR_POS_PIN); /* Turn on LED on falling edge */
    usart_send_string("Cerrando puerta\r\n");
}

void open_door(void){
    gpio_clear(MOTOR_NEG_PORT, MOTOR_NEG_PIN); /* Turn off LED on rising edge */
    gpio_set(MOTOR_POS_PORT, MOTOR_POS_PIN); /* Turn off LED on rising edge */
    usart_send_string("Abriendo puerta\r\n");
}

void stop_motor(void){
    gpio_clear(MOTOR_NEG_PORT, MOTOR_NEG_PIN); /* Turn off LED on rising edge */
    gpio_clear(MOTOR_POS_PORT, MOTOR_POS_PIN); /* Turn off LED on rising edge */
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
void temperature_control_task(void *pvParameters __attribute__((unused)))
{
    while(TRUE)    
    {
        temp_value = get_temp_value();

        if(temp_value < 24)
        {
            temp_value = 0; // Turn off fan
            // Notify close_door_task
            xTaskNotifyGive(closeDoorTaskHandle);
        }


        usart_send_string("Temperatura actual: %d°C\r\n", temp_value); // Envía el valor del ADC por el puerto serial
        timer_set_oc_value(TIM4, TIM_OC4, TIMER_PERIOD - (TIMER_PERIOD * temp_value) / 30); // Configura el duty cycle del PWM

        vTaskDelay(4000 / portTICK_PERIOD_MS); // Wait one minute between temperature readings
    }
}

void battery_level_indicator_task(void *pvParameters __attribute__((unused)))
{
    while(TRUE)
    {
        duty_cycle = get_battery_value(); // Lee el valor del ADC
        usart_send_string("Porcentaje de bateria: %d%%\r\n", duty_cycle); // Envía el valor del ADC por el puerto serial
        uint32_t pwm_value = TIMER_PERIOD - (TIMER_PERIOD * duty_cycle) / PERCENTAGE;
        timer_set_oc_value(TIM4, TIM_OC3, pwm_value); // Configura el duty cycle del PWM
        
        vTaskDelay(10000 / portTICK_PERIOD_MS); // Wait one minute between temperature readings
    }    
}


void close_door_task(void *pvParameters)
{
    while (TRUE)
    {
        // Wait for any notification
        if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY)>0){
            if(xSemaphoreTake(xDoor, portMAX_DELAY) == pdTRUE){
                if(door_state == OPEN){
                    door_state = CLOSED;
                    close_door();
                    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for the door to close
                    stop_motor();
                }
                xSemaphoreGive(xDoor);
            }
        }
    }
}

void open_door_task(void *pvParameters)
{
    while (TRUE)
    {
        // Wait for any notification
        if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY)>0){
            if(xSemaphoreTake(xDoor, portMAX_DELAY) == pdTRUE){
                if(door_state == CLOSED){
                    door_state = OPEN;
                    open_door();
                    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for the door to open
                    stop_motor();
                }
                xSemaphoreGive(xDoor);
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief EXTI0 interrupt handler for button press on PA0.
 * Toggles the LED state when the button is pressed (detects both falling and rising edges).
 */
void exti0_isr(void)
{
    /* Clear the EXTI0 interrupt flag */
    exti_reset_request(EXTI0);

    /* Toggle the LED and switch EXTI edge detection */
    if (exti_direction_0 == FALLING)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE; 
        vTaskNotifyGiveFromISR(closeDoorTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        exti_direction_0 = RISING;     /* Switch to rising edge detection */
        exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
    }
    else
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE; 
        vTaskNotifyGiveFromISR(openDoorTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        exti_direction_0 = FALLING;      /* Switch to falling edge detection */
        exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING);
    }
}

/**
 * @brief EXTI0 interrupt handler for button press on PA0.
 * Toggles the LED state when the button is pressed (detects both falling and rising edges).
 */
void exti1_isr(void)
{
    /* Clear the EXTI1 interrupt flag */
    exti_reset_request(EXTI1);

    /* Toggle the LED and switch EXTI edge detection */
    if (exti_direction_1 == FALLING)
    {

        nvic_disable_irq(NVIC_EXTI0_IRQ);
        nvic_enable_irq(NVIC_EXTI2_IRQ);
        nvic_enable_irq(NVIC_EXTI3_IRQ);
        usart_send_string("Controles manuales desactivados\r\n");   
        exti_direction_1 = RISING;     /* Switch to rising edge detection */
        exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
    }
    else
    {
        nvic_enable_irq(NVIC_EXTI0_IRQ);
        nvic_disable_irq(NVIC_EXTI2_IRQ);
        nvic_disable_irq(NVIC_EXTI3_IRQ);
        usart_send_string("Controles manuales activados\r\n");
        exti_direction_1 = FALLING;      /* Switch to falling edge detection */
        exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING);
    }
}

/**
 * @brief EXTI0 interrupt handler for button press on PA0.
 * Toggles the LED state when the button is pressed (detects both falling and rising edges).
 */
void exti2_isr(void)
{
    /* Clear the EXTI2 interrupt flag */
    exti_reset_request(EXTI2);

    if (exti_direction_2 == FALLING)
    {
        usart_send_string("Zona asegurada\r\n");
        BaseType_t xHigherPriorityTaskWoken = pdFALSE; 
        vTaskNotifyGiveFromISR(openDoorTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        
        exti_direction_2 = RISING;     /* Switch to rising edge detection */
        exti_set_trigger(EXTI2, EXTI_TRIGGER_RISING);
        }
    else
    {
        usart_send_string("Zombies cerca\r\n");
        BaseType_t xHigherPriorityTaskWoken = pdFALSE; 
        vTaskNotifyGiveFromISR(closeDoorTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        exti_direction_2 = FALLING;      /* Switch to falling edge detection */
        exti_set_trigger(EXTI2, EXTI_TRIGGER_FALLING);
    }
}

void exti3_isr(void)
{
    /* Clear the EXTI3 interrupt flag */
    exti_reset_request(EXTI3);

    if (exti_direction_3 == FALLING)
    {

        gpio_clear(ALARM_PORT, ALARM_PIN); /* Turn on LED on falling edge */
        BaseType_t xHigherPriorityTaskWoken = pdFALSE; 
        vTaskNotifyGiveFromISR(openDoorTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        exti_direction_3 = RISING;     /* Switch to rising edge detection */
        exti_set_trigger(EXTI3, EXTI_TRIGGER_RISING);
        usart_send_string("Fuego detectado!!!!\r\n");
        usart_send_string("Activando alarma...\r\n");
    }
    else
    {
        gpio_set(ALARM_PORT, ALARM_PIN); /* Turn off LED on rising edge */
        BaseType_t xHigherPriorityTaskWoken = pdFALSE; 
        vTaskNotifyGiveFromISR(closeDoorTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        exti_direction_3 = FALLING;      /* Switch to falling edge detection */
        exti_set_trigger(EXTI3, EXTI_TRIGGER_FALLING);
        usart_send_string("Fuego controlado\r\n");
        usart_send_string("Desactivando alarma\r\n");
    }
}



void vApplicationStackOverflowHook(TaskHandle_t pxTask __attribute__((unused)), char *pcTaskName __attribute__((unused))) {
    while(1);
}

void config_semaphore(void){
    xADC = xSemaphoreCreateBinary();
    if(xADC == NULL){
        while(1){
            usart_send_string("ERROR");
        }
    }
    xSemaphoreGive(xADC);

    xDoor = xSemaphoreCreateBinary();
    if(xDoor == NULL){
        while(1){
            usart_send_string("ERROR");
        }
    }
    xSemaphoreGive(xDoor);
}

/**
 * @brief Main function.
 * Initializes the system clock, GPIO, SysTick, and EXTI, then enters an infinite loop.
 */
int main(void)
{
    system_clock_setup(); /* Configure system clock */
    gpio_setup();         /* Configure GPIO pins */
    exti_setup();         /* Configure EXTI for button press detection */
    configure_adc();      // Configura el ADC
    configure_usart();    // Configura el puerto serial
    config_pwm();         // Configura el PWM
    dma_setup();          // Configura el DMA

    config_semaphore();


    nvic_set_priority(NVIC_EXTI0_IRQ, 2); /* Set EXTI0 interrupt priority to 0 (highest) */
    nvic_set_priority(NVIC_EXTI1_IRQ, 0); /* Set EXTI1 interrupt priority to 0 (highest) */

    // Task creation
    xTaskCreate(temperature_control_task, TEMPERATURE_CONTROL_TASK_NAME, configMINIMAL_STACK_SIZE + 100, NULL, TEMPERATURE_CONTROL_PRIORITY, NULL);
    xTaskCreate(battery_level_indicator_task, BATTERY_CONTROL_TASK_NAME, configMINIMAL_STACK_SIZE + 100, NULL, BATTERY_CONTROL_PRIORITY, NULL);
    xTaskCreate(close_door_task, CLOSE_DOOR_TASK_NAME, configMINIMAL_STACK_SIZE + 100, NULL, CLOSE_DOOR_PRIORITY, &closeDoorTaskHandle);
    xTaskCreate(open_door_task, OPEN_DOOR_TASK_NAME, configMINIMAL_STACK_SIZE + 100, NULL, OPEN_DOOR_PRIORITY, &openDoorTaskHandle);

    vTaskStartScheduler();

    /* Main loop (the program relies on interrupts for operation) */
    while (TRUE)
    {  
    
    }

    return 0;
}
