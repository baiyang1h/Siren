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

/** @file
 *
 * @defgroup ble_sdk_srv_lbs LEDButton Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief LEDButton Service module.
 *
 * @details This module implements the LEDButton Service with the Battery Level characteristic.
 *          During initialization it adds the LEDButton Service and Battery Level characteristic
 *          to the BLE stack datalbse. Optionally it can also add a Report Reference descriptor
 *          to the Battery Level characteristic (used when including the LEDButton Service in
 *          the HID service).
 *
 *          If specified, the module will support notification of the Battery Level characteristic
 *          through the ble_lbs_battery_level_update() function.
 *          If an event handler is supplied by the application, the LEDButton Service will
 *          generate LEDButton Service events to the application.
 *
 * @note The application must propagate BLE stack events to the LEDButton Service module by calling
 *       ble_lbs_on_ble_evt() from the from the @ref ble_stack_handler callback.
 */

#ifndef BLE_LBS_H__
#define BLE_LBS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define POWER_RESETREAS           0x0007000F

#define LBS_UUID_BASE {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}
#define LBS_UUID_SERVICE          0x1523

#define LBS_UUID_BUTTON_CHAR      0x1524
#define LBS_UUID_OTA_CHAR         0x1525 //jiefu
#define LBS_UUID_DATA_CHAR        0x1526 //jiefu
#define LBS_UUID_TEST_MODE_CHAR   0x1527 //jiefu
#define LBS_UUID_LED_SETTING_CHAR 0x1528 //jiefu
#define LBS_UUID_CALIBRATION_CHAR 0x1529 //jiefu
#define LBS_UUID_VERSION_CHAR     0x1530 //jiefu

#define CHAR_SEND_MAX_LEN         20
#define CHAR_REC_MAX_LEN          10
#define FIRMWARE_VERSION          110                /*   Ver 1.10   */
//#define HARDWARE_VERSION        110                /*   Ver 1.10   */
#define TX_POWER_LEVEL            (0)


#define MCU_SYNC_BYTE                       0x55                                      /**< sync byte between the data communication from MCU to APP */
#define APP_SYNC_BYTE                       0x77                                      /**< sync byte between the data communication from APP to MCU */
#define AD_MAGIC_CODE                       0x99
#define STORED_TEMP_DATA_LEN                16

#define TIMEOUT_PSTORAGE_STORE              (2000)                          /*2 seconds*/
#define TIMEOUT_DISCONNECT_PEER             (8000)
#define TIMEOUT_WATCHDOG                    (10000)


#define LITTLE_ENDIAN_2BYTES_TO_16BIT(BYTE1,BYTE2)  ((BYTE1 & 0xff)| (BYTE2 << 8))

#define LITTLE_ENDIAN_16BIT_BYTE1(VALUE)  (VALUE & 0xff)
#define LITTLE_ENDIAN_16BIT_BYTE2(VALUE)  ((VALUE >> 8) & 0xff)

#define LITTLE_ENDIAN_32BIT_BYTE1(VALUE) (VALUE & 0xff)
#define LITTLE_ENDIAN_32BIT_BYTE2(VALUE) ((VALUE >> 8) & 0xff)
#define LITTLE_ENDIAN_32BIT_BYTE3(VALUE) ((VALUE >> 16) & 0xff)
#define LITTLE_ENDIAN_32BIT_BYTE4(VALUE) ((VALUE >> 24) & 0xff)
/**@breif Macro to unpack 16bit unsigned UUID from octet stream. */
#define UUID16_EXTRACT(DST, SRC) \
    do                           \
    {                            \
        (*(DST))   = (SRC)[1];   \
        (*(DST)) <<= 8;          \
        (*(DST))  |= (SRC)[0];   \
    } while (0)


/*Manufactory data definition*/
#define MFG_SERIAL_NUMBER_H                 (NRF_UICR->CUSTOMER[1])
#define MFG_SERIAL_NUMBER_M                 (NRF_UICR->CUSTOMER[2])
#define MFG_SERIAL_NUMBER_L                 (NRF_UICR->CUSTOMER[3])
#define MFG_BLE_MAC_H                       (NRF_UICR->CUSTOMER[4])
#define MFG_BLE_MAC_L                       (NRF_UICR->CUSTOMER[5])
#define MFG_AUTHENTICATION_CODE             (NRF_UICR->CUSTOMER[6])
#define MFG_HARDWARE_VERSION                (NRF_UICR->CUSTOMER[7])
#define MFG_BOOTLOADER_VERSION              (NRF_UICR->CUSTOMER[8])

typedef struct
{
    uint8_t pwron_reset_cnt;
    uint8_t sysresetreq_reset_cnt;
    uint8_t watchdog_reset_cnt;
    uint8_t pin_reset_cnt;
}device_resetreason_log_t;

typedef struct
{
    uint8_t reserved0                      : 1;
    uint8_t reserved1                      : 1;
    uint8_t reserved2                      : 1;
    uint8_t reserved3                      : 1;
    uint8_t reserved4                      : 1;
    uint8_t reserved5                      : 1;
    uint8_t reserved6                      : 1;
    uint8_t is_enter_ota                   : 1;

    uint8_t is_internal_flash_update       : 1;
    uint8_t is_clearing_log                : 1;
    uint8_t is_inquirying_log              : 1;
    uint8_t is_ble_conn_params_update_finish  : 1;
    uint8_t is_main_process_exe            : 1;
    uint8_t is_flash_operation_finish      : 1;
    uint8_t is_notify_tx_completed         : 1;
    uint8_t is_ble_connected               : 1;

    uint8_t is_device_registered           : 1;
    uint8_t is_requesting_extended_flash_data  : 1;
    uint8_t is_inquirying_leg_flag         : 1;
    uint8_t is_inquirying_motion_status    : 1;
    uint8_t is_inquirying_local_time       : 1;
    uint8_t is_inquirying_mac_addr         : 1;
    uint8_t is_inquirying_ficr_deviceid    : 1;
    uint8_t is_inquirying_device_version   : 1;
}Flag_t;
/**
 * command type between MCU and Mobile APP.
 */
typedef enum
{

    COMMNAD_TYPE_UNKOWN                 = 0,
    COMMAND_TYPE_QUERY_VERSION          = 1,
    COMMAND_TYPE_QUERY_DEVICE_ID        = 2,
    COMMAND_TYPE_QUERY_MAC_ADDR         = 3,
    COMMAND_TYPE_QUERY_DATE_TIME        = 4,
    COMMAND_TYPE_QUERY_BATTERY_LEVEL    = 5,

    COMMAND_TYPE_QUERY_MOTION_STATUS    = 6,
    COMMAND_TYPE_QUERY_LEG_FLAG         = 7,
    COMMAND_TYPE_QUERY_TEMPERATURE_DATA = 8,
    COMMAND_TYPE_DUMP_FLASH_DATA        = 0x0a,

    COMMAND_TYPE_QUERY_LOG              = 0x7F,
    /***/
    COMMAND_TYPE_SET_DATE_TIME          = 0x81,
    COMMAND_TYPE_TRIGGER_OTA            = 0x82,
    COMMAND_TYPE_SET_PATIENT_ID         = 0x83,                  /*!<  */
    COMMAND_TYPE_CLR_LOG                = 0xFF,
    COMMAND_TYPE_MAX

}communication_command_type;

typedef enum
{

    COMMUNICATION_DATA_ONLY                    = 0,
    COMMUNICATION_DATA_WITH_TIMESTAMP          = 1,
}communication_data_type;

/**
 * characteristic list between MCU and Mobile APP.
 */

typedef enum
{

    BLE_CHAR_OTA,
    BLE_CHAR_TEMP_DATA,
    BLE_CHAR_SYS_SETTING,
    BLE_CHAR_QUERY_DATA
}ble_char_list;


typedef enum
{
    LEFT_LEG_FLAG                       = 0,
    RIGHT_LEG_FLAG                      = 1,
    UNKNOW_LEG_FLAG                     = 2,
}leg_flag_t;

typedef enum
{
    MOTION_ACTIVITY                      = 0,
    MOTION_INACTIVITY                    = 1,
}motion_status_t;



typedef enum
{
    COM_DATA_SYNC_NONE,
    COM_DATA_SYNC_TRIG,
    COM_DATA_SYNC_COMPLETE,
}com_data_sync_t;


typedef enum
{
    DEVICE_UNREGISTERED_STATUS,
    DEVICE_REGISTERED_IDLE_STATUS,
    DEVICE_REGISTERED_NORMAL_STATUS,
    DEVICE_REGISTERED_SPECIAL_STATUS,
}device_status_t;


typedef enum
{
    BLE_NO_SCAN,                                                     /**< No advertising running. */
    BLE_WHITELIST_SCAN,                                              /**< Advertising with whitelist. */
    BLE_FAST_SCAN,                                                   /**< Fast advertising running. */
} ble_scan_mode_t;


typedef struct
{
    bool enable;
    uint8_t sequence_id;
    uint8_t data_format;
    com_data_sync_t sync_data_status;
    uint32_t report_per_interval_ticks;
    unsigned long current_ms_ticks;
    communication_command_type cmd;
    ble_char_list char_index;
    bool forced_to_store;        /*Forced to store SPI flash*/
} ble_communication_t;

typedef struct
{
    char device_name[8];
    uint32_t serial_num;
    uint32_t leg_flag;
    uint32_t hardware_version;
    uint32_t bootloader_version;
    uint32_t manufacture_date;
} manufactory_data_t;

typedef struct
{
    uint16_t tm_year;
    uint8_t tm_month;
    uint8_t tm_day;
    uint8_t tm_hour;
    uint8_t tm_minute;
    uint8_t tm_second;
    uint8_t reserved;
}local_time_t;

/**@brief Variable length data encapsulation in terms of length and pointer to data */
typedef struct
{
    uint8_t     * p_data;                                             /**< Pointer to data. */
    uint16_t      data_len;                                           /**< Length of data. */
}data_t;





// Forward declaration of the ble_lbs_t type.
typedef struct ble_lbs_s ble_lbs_t;

/**@brief LEDButton Service event handler type. */
typedef void (*ble_lbs_led_write_handler_t) (ble_lbs_t * p_lbs, uint8_t* new_state);

/**@brief LEDButton Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    //ble_lbs_led_write_handler_t   led_write_handler;                      //jiefu
    //ble_lbs_led_write_handler_t   led_mp_write_handler;                   //jiefu
    /* jiefu 20150221*/
    ble_lbs_led_write_handler_t version_write_handler;
    ble_lbs_led_write_handler_t ota_write_handler;
    ble_lbs_led_write_handler_t data_write_handler;
    ble_lbs_led_write_handler_t led_setting_write_handler;
    #ifndef VIGO_CLEAN_UP
    ble_lbs_led_write_handler_t calibration_write_handler;
    ble_lbs_led_write_handler_t button_write_handler;
    ble_lbs_led_write_handler_t test_mode_write_handler;
    #endif
} ble_lbs_init_t;


/**@brief LEDButton Service structure. This contains various status information for the service. */
typedef struct ble_lbs_s
{
    uint16_t                     service_handle;

    /* jiefu 20150221*/
    ble_gatts_char_handles_t ota_char_handles;
    ble_gatts_char_handles_t data_char_handles;
      ble_gatts_char_handles_t led_setting_char_handles;
      ble_gatts_char_handles_t version_char_handles;
#ifndef VIGO_CLEAN_UP
    ble_gatts_char_handles_t test_mode_char_handles;
    ble_gatts_char_handles_t calibration_char_handles;
    ble_gatts_char_handles_t button_char_handles;
#endif
    uint8_t                      uuid_type;
    uint8_t                      current_led_state;
    uint16_t                     conn_handle;
    bool                         is_notifying;

    /* jiefu 20150221*/
    ble_lbs_led_write_handler_t ota_write_handler;
    ble_lbs_led_write_handler_t data_write_handler;
    ble_lbs_led_write_handler_t led_setting_write_handler;
    ble_lbs_led_write_handler_t version_write_handler;
#ifndef VIGO_CLEAN_UP
    ble_lbs_led_write_handler_t button_write_handler;
    ble_lbs_led_write_handler_t test_mode_write_handler;
    ble_lbs_led_write_handler_t calibration_write_handler;
#endif
} ble_lbs_t;




/**@brief Initialize the LEDButton Service.
 *
 * @param[out]  p_lbs
 * @param[in]   p_lbs_init
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_lbs_init(ble_lbs_t * p_lbs, const ble_lbs_init_t * p_lbs_init);

/**@brief LEDButton Service BLE stack event handler.
 *
 * @details Handles all events from the BLE stack of interest to the LEDButton Service.
 *
 * @param[in]   p_lbs      LEDButton Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_lbs_on_ble_evt(ble_lbs_t * p_lbs, ble_evt_t * p_ble_evt);

/**@brief Handler to notify the LEDButton service on button presses.
 *
 * @param[in]   p_lbs      LEDButton Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
uint32_t ble_lbs_on_button_change(ble_lbs_t * p_lbs, uint8_t button_state);

uint32_t ble_lbs_sensor_data_update(ble_lbs_t * p_lbs, uint8_t *p_buf);

uint32_t ble_lbs_version_data_update(ble_lbs_t * p_lbs, uint8_t *p_buf);

uint32_t ble_lbs_led_setting_update(ble_lbs_t * p_lbs, uint8_t *p_buf);

uint8_t get_temperature_degree(uint16_t adc_val, uint8_t *relative_temp_val);

void temp_sensor_data_encode(communication_data_type type,uint8_t *send_buf, uint16_t len);

void version_info_encode(uint8_t *send_buf, uint16_t len);

void device_id_encode(uint8_t *send_buf, uint16_t len);

void MAC_address_encode(uint8_t *send_buf, uint16_t len);

void current_date_time_encode(communication_command_type type,uint8_t *send_buf, uint16_t len);

void patient_motion_status_encode(uint8_t *send_buf, uint16_t len);

void patient_leg_flag_encode(uint8_t *send_buf, uint16_t len);

void log_inquirying_encode(uint8_t *send_buf, uint16_t len);

void log_clearing_encode(uint8_t *send_buf, uint16_t len);

void battery_voltage_level_encode(uint8_t *send_buf, uint16_t len);

void patient_id_encode(uint8_t *send_buf, uint16_t len);


#endif // BLE_LBS_H__



/** @} */
