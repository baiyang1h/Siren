/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef SMART_SOCK_H__
#define SMART_SOCK_H__
#include "nrf_gpio.h"
#include "app_trace.h"

#define LEDS_NUMBER                    1
#define LED_PIN                        24
#define LEDS_MASK                      (1<<LED_PIN)
#define LEDS_INV_MASK                  LEDS_MASK

#define LSM6_INT_PIN                   21
#define TWI_INT_READ()                 ((NRF_GPIO->IN >> LSM6_INT_PIN) & 0x1UL)

//#define GSENSOR_INTERRUPT_WAY
#if defined(GSENSOR_INTERRUPT_WAY)
    #define BUTTONS_NUMBER                 1

    #define BUTTON_START                   21
    #define BUTTON_1                       LSM6_INT_PIN
    #define BUTTON_STOP                    21
    #define BUTTON_PULL                    NRF_GPIO_PIN_PULLUP
    #define BUTTONS_LIST                   { BUTTON_1 }

    #define BSP_BUTTON_0                   BUTTON_1

    #define BSP_BUTTON_0_MASK              (1<<BSP_BUTTON_0)

    #define BUTTONS_MASK                   (BSP_BUTTON_0_MASK)
#endif

#define OP_CONTROL_PIN_NUMBER          7      // OP CONTROL pin number.
#define Temperature_calculate_pwroff() nrf_gpio_pin_clear(OP_CONTROL_PIN_NUMBER)
#define Temperature_calculate_pwron()  nrf_gpio_pin_set(OP_CONTROL_PIN_NUMBER)
#define FACTORY_TEST_PIN               8      // factory test pin number.

#define RX_PIN_NUMBER  12    // UART RX pin number.
#define TX_PIN_NUMBER  9     // UART TX pin number.
#define CTS_PIN_NUMBER 18    // UART Clear To Send pin number. Not used if HWFC is set to false. 
#define RTS_PIN_NUMBER 19    // UART Request To Send pin number. Not used if HWFC is set to false. 
#define HWFC           false // UART hardware flow control.

#define SPIS_MISO_PIN  20    // SPI MISO signal. 
#define SPIS_CSN_PIN   21    // SPI CSN signal. 
#define SPIS_MOSI_PIN  22    // SPI MOSI signal. 
#define SPIS_SCK_PIN   23    // SPI SCK signal. 

#define SPIM0_SCK_PIN       23u     /**< SPI clock GPIO pin number. */
#define SPIM0_MOSI_PIN      20u     /**< SPI Master Out Slave In GPIO pin number. */
#define SPIM0_MISO_PIN      22u     /**< SPI Master In Slave Out GPIO pin number. */
#define SPIM0_SS_PIN        21u     /**< SPI Slave Select GPIO pin number. */

#define SPIM1_SCK_PIN       19u     /**< SPI clock GPIO pin number. */
#define SPIM1_MOSI_PIN      18u     /**< SPI Master Out Slave In GPIO pin number. */
#define SPIM1_MISO_PIN      17u     /**< SPI Master In Slave Out GPIO pin number. */
#define SPIM1_SS_PIN        20u     /**< SPI Slave Select GPIO pin number. */



#define I2C_SCL_PIN         23      // SCL signal pin
#define I2C_SDA_PIN         22      // SDA signal pin   

#define LEDS_OFF(leds_mask) do {  NRF_GPIO->OUTSET = (leds_mask) & (LEDS_MASK & LEDS_INV_MASK); \
                                  NRF_GPIO->OUTCLR = (leds_mask) & (LEDS_MASK & ~LEDS_INV_MASK); } while (0)

#define LEDS_ON(leds_mask)  do {  NRF_GPIO->OUTCLR = (leds_mask) & (LEDS_MASK & LEDS_INV_MASK); \
                                  NRF_GPIO->OUTSET = (leds_mask) & (LEDS_MASK & ~LEDS_INV_MASK); } while (0)

#define LED_IS_ON(leds_mask) ((leds_mask) & (NRF_GPIO->OUT ^ LEDS_INV_MASK) )

#define LEDS_INVERT(leds_mask) do { uint32_t gpio_state = NRF_GPIO->OUT;      \
                                    NRF_GPIO->OUTSET = ((leds_mask) & ~gpio_state); \
                                    NRF_GPIO->OUTCLR = ((leds_mask) & gpio_state); } while (0)

#define LEDS_CONFIGURE(leds_mask) do { uint32_t pin;                  \
                                  for (pin = 0; pin < 32; pin++) \
                                      if ( (leds_mask) & (1 << pin) )   \
                                          nrf_gpio_cfg_output(pin); } while (0)
#define DBG_PRINT    app_trace_log
#endif  // SMART_SOCK_H__
