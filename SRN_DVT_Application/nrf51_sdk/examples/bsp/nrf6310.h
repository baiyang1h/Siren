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
#ifndef NRF6310_H__
#define NRF6310_H__

#define LEDS_NUMBER                    1
#define LED_PIN                        24
#define LEDS_MASK                      (1<<LED_PIN)
#define LEDS_INV_MASK                  LEDS_MASK

#define OP_CONTROL_PIN_NUMBER          7      // OP CONTROL pin number.
#define Temperature_calculate_pwroff() nrf_gpio_pin_clear(OP_CONTROL_PIN_NUMBER)
#define Temperature_calculate_pwron()  nrf_gpio_pin_set(OP_CONTROL_PIN_NUMBER)
#define FACTORY_TEST_PIN               8      // factory test pin number.

#define RX_PIN_NUMBER  12 /*16*/    // UART RX pin number.
#define TX_PIN_NUMBER  9  /*17*/    // UART TX pin number.
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

#define SPIM1_SCK_PIN       19u /*16u*/     /**< SPI clock GPIO pin number. */
#define SPIM1_MOSI_PIN      18u     /**< SPI Master Out Slave In GPIO pin number. */
#define SPIM1_MISO_PIN      17u     /**< SPI Master In Slave Out GPIO pin number. */
#define SPIM1_SS_PIN        20u /*19u*/     /**< SPI Slave Select GPIO pin number. */

// serialization APPLICATION board
#define SER_APP_RX_PIN              12 /*16*/    // UART RX pin number.
#define SER_APP_TX_PIN              9  /*17*/    // UART TX pin number.
#define SER_APP_CTS_PIN             18    // UART Clear To Send pin number.
#define SER_APP_RTS_PIN             19    // UART Request To Send pin number.

#if 0
#define SER_APP_SPIM0_SCK_PIN       20     // SPI clock GPIO pin number.
#define SER_APP_SPIM0_MOSI_PIN      17     // SPI Master Out Slave In GPIO pin number
#define SER_APP_SPIM0_MISO_PIN      16     // SPI Master In Slave Out GPIO pin number
#define SER_APP_SPIM0_SS_PIN        21     // SPI Slave Select GPIO pin number
#define SER_APP_SPIM0_RDY_PIN       19     // SPI READY GPIO pin number
#define SER_APP_SPIM0_REQ_PIN       18     // SPI REQUEST GPIO pin number
#else
#define SER_APP_SPIM0_SCK_PIN       23     // SPI clock GPIO pin number.
#define SER_APP_SPIM0_MOSI_PIN      20     // SPI Master Out Slave In GPIO pin number
#define SER_APP_SPIM0_MISO_PIN      22     // SPI Master In Slave Out GPIO pin number
#define SER_APP_SPIM0_SS_PIN        21     // SPI Slave Select GPIO pin number
#define SER_APP_SPIM0_RDY_PIN       29     // SPI READY GPIO pin number
#define SER_APP_SPIM0_REQ_PIN       28     // SPI REQUEST GPIO pin number

#endif

// serialization CONNECTIVITY board
#if 0
#define SER_CON_RX_PIN              17    // UART RX pin number.
#define SER_CON_TX_PIN              16    // UART TX pin number.
#define SER_CON_CTS_PIN             19    // UART Clear To Send pin number. Not used if HWFC is set to false.
#define SER_CON_RTS_PIN             18    // UART Request To Send pin number. Not used if HWFC is set to false.
#else
#define SER_CON_RX_PIN              12 /*16*/    // UART RX pin number.
#define SER_CON_TX_PIN               9  /*17*/    // UART TX pin number.
#define SER_CON_CTS_PIN             18    // UART Clear To Send pin number. Not used if HWFC is set to false.
#define SER_CON_RTS_PIN             19    // UART Request To Send pin number. Not used if HWFC is set to false.
#endif

#if 0
#define SER_CON_SPIS_SCK_PIN        20    // SPI SCK signal.
#define SER_CON_SPIS_MISO_PIN       16    // SPI MISO signal.
#define SER_CON_SPIS_MOSI_PIN       17    // SPI MOSI signal.
#define SER_CON_SPIS_CSN_PIN        21    // SPI CSN signal.
#define SER_CON_SPIS_RDY_PIN        19     // SPI READY GPIO pin number.
#define SER_CON_SPIS_REQ_PIN        18     // SPI REQUEST GPIO pin number.
#else
#define SER_CON_SPIS_SCK_PIN        23    // SPI SCK signal.
#define SER_CON_SPIS_MOSI_PIN       22    // SPI MOSI signal.
#define SER_CON_SPIS_MISO_PIN       20    // SPI MISO signal.
#define SER_CON_SPIS_CSN_PIN        21    // SPI CSN signal.
#define SER_CON_SPIS_RDY_PIN        29     // SPI READY GPIO pin number.
#define SER_CON_SPIS_REQ_PIN        28     // SPI REQUEST GPIO pin number.
#endif


#define I2C_SCL_PIN                 23     // SCL signal pin
#define I2C_SDA_PIN                 22    // SDA signal pin
#define LSM6_INT_PIN                21

#define TWI_INT_READ()              ((NRF_GPIO->IN >> LSM6_INT_PIN) & 0x1UL)                     /*!< Reads current state of SDA */



// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      NRF_CLOCK_LFCLKSRC_XTAL_20_PPM

#endif  // NRF6310_H__
