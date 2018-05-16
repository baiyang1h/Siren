/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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



#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "Smart_sock.h"
#include "ble_hci.h"
#include "ble_db_discovery.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "device_manager.h"
#include "ble_lbs.h"
#include "pstorage.h"
#include "app_trace.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_drv_timer.h"
#include "app_uart.h"
#include "nrf_adc.h"
#include "spi_master_app.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_wdt.h"
#include "nrf_delay.h"
#include "twi_master.h"
#include "lsm6dsl_reg.h"


#ifdef BLE_DFU_APP_SUPPORT
#include "ble_dfu.h"
#include "dfu_app_handler.h"
#endif // BLE_DFU_APP_SUPPORT


#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define ADVERTISE_DEVICE_NAME            "SIREN"                                     /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "NordicSemiconductor"                      /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                 MSEC_TO_UNITS(4000, UNIT_0_625_MS) /*300*/                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_SMALL_INTERVAL           MSEC_TO_UNITS(500, UNIT_0_625_MS) /*300*/  /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */

#define APP_ADV_TIMEOUT_IN_SECONDS       180                                        /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#if !defined(GSENSOR_INTERRUPT_WAY)
//if Button is not used,then APP_TIMER_OP_QUEUE_SIZE need to increase 2;
#define APP_TIMER_OP_QUEUE_SIZE          3                                          /**< Size of timer operation queues. */
#else
//if Button is not used,then APP_TIMER_OP_QUEUE_SIZE need to decrease 2;
#define APP_TIMER_OP_QUEUE_SIZE          5                                          /**< Size of timer operation queues. */
#endif
//Connection Parameters
#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(500, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(1000, UNIT_1_25_MS)          /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                    0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)           /**< Connection supervisory timeout (4 seconds). */

//Connection Parameters during temperature data syncing.
#define SYNC_MIN_CONN_INTERVAL           MSEC_TO_UNITS(7.5, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (5ms). */
#define SYNC_MAX_CONN_INTERVAL           MSEC_TO_UNITS(15, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (10ms). */
#define SYNC_SLAVE_LATENCY               0                                          /**< Slave latency. */
#define SYNC_CONN_SUP_TIMEOUT            MSEC_TO_UNITS(4000, UNIT_10_MS)           /**< Connection supervisory timeout (4 seconds). */


#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

//#define TX_POWER_LEVEL                   RADIO_TXPOWER_TXPOWER_Neg8dBm//(-8)                                         /**< TX Power Level value. This will be set both in the TX Power service, in the advertising data, and also used to set the radio transmit power. */


#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                         /**< Maximum encryption key size. */

#define MAX_PERIOD_TIME                  1000                                       /**< 1 milisecond: specified value in a capture compare regiser CC[n] for TIMER 2.*/
#define DEAD_BEEF                        0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                 256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                 1                                           /**< UART RX buffer size. */

#define CLOCK_MEAS_INTERVAL              APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)  /**< Clock 1000ms measurement interval (ticks). */

#define SCAN_INTERVAL              MSEC_TO_UNITS(4000, UNIT_0_625_MS) //0x00A0                             /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                MSEC_TO_UNITS(200, UNIT_0_625_MS)  //0x0050                             /**< Determines scan window in units of 0.625 millisecond. */
#define TARGET_UUID                0x180D                             /**< Target device name that application is looking for. */
#define MAX_PEER_COUNT             DEVICE_MANAGER_MAX_CONNECTIONS     /**< Maximum number of peer's application intends to manage. */
#define UUID16_SIZE                2                                  /**< Size of 16 bit UUID */


#ifdef BLE_DFU_APP_SUPPORT
#define DFU_REV_MAJOR                    0x00                                       /** DFU Major revision number to be exposed. */
#define DFU_REV_MINOR                    0x01                                       /** DFU Minor revision number to be exposed. */
#define DFU_REVISION                     ((DFU_REV_MAJOR << 8) | DFU_REV_MINOR)     /** DFU Revision number to be exposed. Combined of major and minor versions. */
#define APP_SERVICE_HANDLE_START         0x000C                                     /**< Handle of first application specific service when when service changed characteristic is present. */
#define BLE_HANDLE_MAX                   0xFFFF                                     /**< Max handle value in BLE. */

STATIC_ASSERT(IS_SRVC_CHANGED_CHARACT_PRESENT);                               /** When having DFU Service support in application the Service Changed Characteristic should always be present. */
#endif // BLE_DFU_APP_SUPPORT

#define ADC_REF_VBG_VOLTAGE_IN_MILLIVOLTS   1200                                         /**< Value in millivolts for voltage used as reference in ADC conversion on NRF51. */
#define ADC_INPUT_PRESCALER                 3                                            /**< Input prescaler for ADC convestion on NRF51. */
#define ADC_RES_10BIT                       1024                                         /**< Maximum digital value for 10-bit ADC conversion. */

#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VBG_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_INPUT_PRESCALER)

#define expire(x,y)         (abs(msticks - (x)) >= (y))//(msticks - (x) >= (y))//(labs(msticks - (x)) >= (y))

#define SMART_SOCK_USING_TEMPERATURE_LOW        (210)                                    /**< lowest temperature value with 10 times when used, because float is not supported for this chip*/
#define SMART_SOCK_USING_TEMPERATURE_HIGH       (420)                                    /**< highest temperature value with 10 times when used*/

#define DEFAULT_TIME_OFFSET_THRESHOLD           (5*60)                                 /**<second, time offset between local time and real time, if larger than this threshold, local time will be forcely updated*/
#define DEFAULT_INQUIRE_TEMP_INTERVAL           (60)                                    /**< second, */

const char daytab[2][13]= 
{
    {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
    {0, 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}
};
static dm_application_instance_t          m_app_handle;                               /**< Application identifier allocated by device manager */

static uint16_t                           m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */


// YOUR_JOB: Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] = {{LBS_UUID_SERVICE, BLE_UUID_TYPE_BLE}}; /**< Universally unique service identifiers. */


static ble_lbs_t                        m_lbs;
#ifdef BLE_DFU_APP_SUPPORT
static ble_dfu_t                         m_dfus;                                    /**< Structure used to identify the DFU service. */
#endif // BLE_DFU_APP_SUPPORT


/*Support BLE centre*/
//static ble_db_discovery_t           m_ble_db_discovery;                  /**< Structure used to identify the DB Discovery module. */
//static ble_hrs_c_t                  m_ble_hrs_c;                         /**< Structure used to identify the heart rate client module. */
//static ble_bas_c_t                  m_ble_bas_c;                         /**< Structure used to identify the Battery Service client module. */
//static ble_gap_scan_params_t        m_scan_param;                        /**< Scan parameters requested for scanning and connection. */
//static dm_application_instance_t    m_dm_app_id;                         /**< Application identifier. */
//static dm_handle_t                  m_dm_device_handle;                  /**< Device Identifier identifier. */
//static uint8_t                      m_peer_count = 0;                    /**< Number of peer's connected. */
//static ble_scan_mode_t              m_scan_mode = BLE_FAST_SCAN;         /**< Scan mode used by application. */
//static uint16_t                     m_conn_handle;                       /**< Current connection handle. */
static volatile bool                m_whitelist_temporarily_disabled = false; /**< True if whitelist has been temporarily disabled. */

bool                         m_memory_access_in_progress = false; /**< Flag to keep track of ongoing operations on persistent memory. */


uint16_t ButtonOneState;
uint8_t FLASH_DATA_DUMP_REQUIRED = 0;

uint32_t report_battery_level_interval_tick = 0;     /* how much time to report battery voltage value, which is based on system tick ( 1ms)*/
uint32_t report_temp_value_cnt_tick = 0;         /* how much time to report temperature value, which is based on system tick ( 1ms)*/
unsigned long ms_battery_level_tick = 0;
unsigned long ms_temp_value_interval_tick = 0;


uint8_t DATA_ONLY_REQUIRED = 0;
uint8_t DATA_TIMESTAMP_REQUIRED = 0;
uint8_t DATA_SEND_FINISHED = 0;
uint8_t SAMPLING_REQUIRED = 1;
uint8_t SIREN_SAMPLE_RATE = 1;

volatile unsigned long msticks = 0;
volatile unsigned long record_motion_msticks = 0;

uint16_t ms5_flag,ms10_flag;
uint32_t ms_Led_500MS_ONE_TIME_ENABLE;
uint32_t ms_BLE_ADC_20MS_ONE_TIME_ENABLE;
uint32_t ms_ADC_SAMPLING_1S_ONE_TIME_ENABLE;
uint32_t ms_ADC_SAMPLING_80MS_VERSION_ONE_TIME_ENABLE;
//uint32_t ms_ADC_SAMPLING_2S_ONE_TIME_ENABLE;

APP_TIMER_DEF(m_clock_timer_id);

static volatile bool               m_adc_completed = true;
static volatile uint8_t            adc_channel;
static nrf_drv_wdt_channel_id      m_channel_id;
static int32_t                     local_time_offset = 0;   /*the offset seconds between local time and the time in mobile APP*/
static uint16_t                    battery_last_voltage = 3000;
uint16_t                           adc_value[7];
uint8_t                            adc_value_flash[12];
uint8_t                            percentage_batt_lvl;
uint32_t                           dump_flash_addr = 0;
Flag_t                             Flag;
volatile bool ble_peripheral_manual_disconnect  = false;

extern uint32_t                    *timestamp_ptr;
extern uint16_t                    *total_step_counter;
extern extended_flash_param_t      *extended_flash_rw_param_ptr;
extern uint8_t                     *time_zone;
extern uint8_t                     sequence_id;
extern uint16_t                    patient_id;
extern ble_communication_t         temp_req;
extern ble_communication_t         battery_level;
extern ble_communication_t         date_time_req;
extern ble_communication_t         patient_req;
extern ble_communication_t         acc_inquiry;
extern ble_communication_t         device_status_inquiry;
extern motion_status_t             motion_status;
extern internal_flash_param_t      internal_flash_param;
/************************************************************************/
void device_pstorage_init(void);
extern void spi_master_init(void);
extern void lsm6dsl_universal_timer_timeout_handler(void);
extern void LSM6DSL_Init(void);
extern void lsm6dsl_inquiry_all_src(lsm6dsl_all_sources_t *p_all_sources);
extern void lsm6dsl_GetAccSTEPCounter(uint16_t* val);

/**@brief Function for calculating the offset seconds between local time and new time.
 *
 * @param[out]  time offset seconds between local time and new time
 * @param[in]   new_time
 * @param[in]   local_time
 */
int32_t time_get_offset_seconds(uint32_t new_time, uint32_t local_time)
{
    return (int32_t)(new_time-local_time);
}  
/**@brief Function for compensating the offset seconds between local time and new time.
 *
 * @param[out]  Synchronous date
 * @param[in]   local_time
 * @param[in]   offset_seconds
 */
void time_get_synced_time(uint32_t local_time, int32_t offset_seconds, uint32_t *psync_date)
{
    *psync_date = local_time+offset_seconds;
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for dumping out all data in SPI flash and send them to BLE central for analysis.
 */
uint32_t flash_data_dump_encode_notify(uint8_t *send_buf, uint16_t len)
{
    uint32_t err_code   = NRF_SUCCESS;
    uint8_t  cnt        = 0;
    unsigned long cur_msticks = 0;

    memset(send_buf,0,len);
    if(dump_flash_addr == 0)
    {
        send_buf[0] = MCU_SYNC_BYTE;
        send_buf[1] = sequence_id;
        send_buf[2] = COMMAND_TYPE_DUMP_FLASH_DATA;                    /*command type*/
        send_buf[3] = LITTLE_ENDIAN_16BIT_BYTE1(patient_id);           /*patient id low 8 bits: 0~7*/
        send_buf[4] = LITTLE_ENDIAN_16BIT_BYTE2(patient_id);           /*patient id high 8 bits: 8~15*/
        send_buf[5] = LITTLE_ENDIAN_32BIT_BYTE1((extended_flash_rw_param_ptr->end_addr)) ;          /**/
        send_buf[6] = LITTLE_ENDIAN_32BIT_BYTE2((extended_flash_rw_param_ptr->end_addr)) ;          /**/
        send_buf[7] = LITTLE_ENDIAN_32BIT_BYTE3((extended_flash_rw_param_ptr->end_addr)) ;          /*24bit length*/
        cnt = len - 8;
    }
    else
    {
        cnt = ((dump_flash_addr+len) > (extended_flash_rw_param_ptr->end_addr)) ? ((extended_flash_rw_param_ptr->end_addr)-dump_flash_addr) : len;  /*less than or equal to 20 bytes*/
    }
    while(dump_flash_addr < (extended_flash_rw_param_ptr->end_addr))
    {
        if(dump_flash_addr == 0)
        {
            SpiFlash_Read_Data(&send_buf[8],dump_flash_addr,cnt);
        }
        else
        {
            memset(send_buf,0,len);
            SpiFlash_Read_Data(&send_buf[0],dump_flash_addr,cnt);
        }
        Flag.is_notify_tx_completed = false;
        cur_msticks = msticks;
        err_code = ble_lbs_version_data_update(&m_lbs,send_buf);
        if(err_code != NRF_SUCCESS)
        {
            SpiFlash_Deep_Sleep_Enter();
            APP_ERROR_HANDLER(err_code);
        }
        while(!Flag.is_notify_tx_completed)
        {
            if(expire(cur_msticks,2*MAX_PERIOD_TIME))
            {
                SpiFlash_Deep_Sleep_Enter();
                APP_ERROR_HANDLER(NRF_ERROR_TIMEOUT);
            }
        }
        dump_flash_addr += cnt;
        cnt = ((dump_flash_addr+len) > (extended_flash_rw_param_ptr->end_addr)) ? ((extended_flash_rw_param_ptr->end_addr)-dump_flash_addr) : len;  /*less than or equal to 20 bytes*/
    }
    return err_code;
}

/**@brief Function for Get all unsynced data in SPI flash and send them to BLE central.
 */
uint32_t temp_data_sync_encode_notify(uint8_t *send_buf, uint16_t len)
{
    uint32_t                 err_code   = NRF_SUCCESS;
    ble_communication_t      *p_com     = &temp_req;
    extended_flash_param_t   *p_flash   = extended_flash_rw_param_ptr;
    unsigned long            cur_msticks = 0;
    uint32_t                 default_time_stamp,sync_time_stamp;

    while((p_flash->read_addr < p_flash->write_addr) ||
          ((p_flash->read_addr > p_flash->write_addr)&&
           (p_flash->roll_back == true)
          )
         )
    {
        int32_t offset = 0;
        memset(send_buf,0,len);
        default_time_stamp = 0;
        sync_time_stamp = 0;
        SpiFlash_Read_Data(send_buf,p_flash->read_addr,STORED_TEMP_DATA_LEN);

        if(abs(local_time_offset) > DEFAULT_TIME_OFFSET_THRESHOLD)
        {
            /*Adjust time stamp of temperature data according to time offset value if neccessary*/
            default_time_stamp += ((send_buf[9]&0x000000FF)<<0);
            default_time_stamp += ((send_buf[10]&0x000000FF)<<8);
            default_time_stamp += ((send_buf[11]&0x000000FF)<<16);
            default_time_stamp += ((send_buf[12]&0x000000FF)<<24);
            

            offset = time_get_offset_seconds(*timestamp_ptr,default_time_stamp);
            if(abs(offset) > abs(local_time_offset))
            {
                time_get_synced_time(default_time_stamp,local_time_offset,&sync_time_stamp);
                send_buf[9]  = ((sync_time_stamp&0x000000FF)>>0);
                send_buf[10] = ((sync_time_stamp&0x0000FF00)>>8);
                send_buf[11] = ((sync_time_stamp&0x00FF0000)>>16);
                send_buf[12] = ((sync_time_stamp&0xFF000000)>>24);
            }
            else
            {
                local_time_offset = 0; //the time stamp in flash is already synced with APP, clear the time offset value.
            }
        }
        memmove(&send_buf[5],send_buf,(STORED_TEMP_DATA_LEN-1));  /*the last byte of every stored temperate data will be ignored*/
        send_buf[0] = MCU_SYNC_BYTE;
        send_buf[1] = p_com->sequence_id;
        send_buf[2] = COMMAND_TYPE_QUERY_TEMPERATURE_DATA;            /*command type*/
        send_buf[3] = LITTLE_ENDIAN_16BIT_BYTE1(patient_id);          /*patient id low 8 bits: 0~7*/
        send_buf[4] = LITTLE_ENDIAN_16BIT_BYTE2(patient_id);          /*patient id high 8 bits: 8~15*/
        Flag.is_notify_tx_completed = false;
        cur_msticks = msticks;
        err_code = ble_lbs_sensor_data_update(&m_lbs,send_buf);
        if(err_code != NRF_SUCCESS)
        {
            SpiFlash_Deep_Sleep_Enter();
            APP_ERROR_HANDLER(err_code);
        }
        while(!Flag.is_notify_tx_completed)
        {
            if(expire(cur_msticks,2*MAX_PERIOD_TIME))
            {
                SpiFlash_Deep_Sleep_Enter();
                APP_ERROR_HANDLER(NRF_ERROR_TIMEOUT);
            }
        }
        p_flash->read_addr += STORED_TEMP_DATA_LEN;
        if(p_flash->read_addr >= p_flash->end_addr)
        {
            p_flash->read_addr = p_flash->start_addr;
            p_flash->roll_back = false;
        }
    }
    return err_code;
}

/**@brief Function for the rtc timer timeout handler.
 *
 * @details Real-time clock operation.
 */
void clock_meas_timeout_handler(void *p_context)
{
    msticks = msticks + MAX_PERIOD_TIME;//Care about the power consumption, Not use TIMER2, but app_timer to check the time interval,

    (*timestamp_ptr)++;

    if((((*timestamp_ptr)+(*time_zone)*3600)%(86400))<60)
    {
        *total_step_counter = 0;
    }

    /*Feed the watch dog, the watch dog timeout is 10 seconds*/
    nrf_drv_wdt_channel_feed(m_channel_id);
    Flag.is_main_process_exe = true;
    lsm6dsl_universal_timer_timeout_handler();
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.*/
    err_code = app_timer_create(&m_clock_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                clock_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    uint32_t                leg_flag = 0 ;
    char dev_name[32] = { 0 };

    snprintf(dev_name,sizeof(dev_name),ADVERTISE_DEVICE_NAME);
    leg_flag = (MFG_BLE_MAC_L & 0x00010000)>>16;
    if (leg_flag == LEFT_LEG_FLAG )
    {
        strcat(dev_name,"_L");
    }
    else if (leg_flag == RIGHT_LEG_FLAG )
    {
        strcat(dev_name,"_R");
    }
    else
    {
        strcat(dev_name,"_L");
    }

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)dev_name,
                                          strlen(dev_name));
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Use an appearance value matching the application's use case.
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
    APP_ERROR_CHECK(err_code); */

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_tx_power_set(TX_POWER_LEVEL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the YYY Service events.
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
static void on_yys_evt(ble_yy_service_t     * p_yy_service,
                       ble_yy_service_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            DBG_PRINT("[APPL]: charact written with value %s. \r\n", p_evt->params.char_xx.value.p_str);
            break;

        default:
            // No implementation needed.
            break;
    }
}*/

#ifndef VIGO_CLEAN_UP
/**@brief Function for handling a write to the LED characteristic of the LED Button service.
 * @detail A pointer to this function is passed to the service in its init structure.
 */

static void button_write_handler(ble_lbs_t * p_lbs, uint8_t* data)
{
    switch(data[0])
    {
        case 0x00:
          break;

        case 0x01:
            break;

        case 0x02:
          break;

        default:
          break;
    }
}

static void test_mode_write_handler(ble_lbs_t * p_lbs, uint8_t* led_state)
{
    switch(led_state[0])
    {
        case 0x00:
            break;

        default:
            break;

    }
}

static void calibration_write_handler(ble_lbs_t * p_lbs, uint8_t* data)
{

}


#endif

/**@brief   Function for handling events from the OTA characteristic of the LED Button service. 
 *
 * @details A pointer to this function is passed to the service in its init structure.
 * 
 * @param[in] p_lbs  Pointer to the LED Button Service structure to which the included data relates.
 * @param[in] data   Pointer to the included data.
 */
static void ota_write_handler(ble_lbs_t * p_lbs, uint8_t* data)
{
    if((NULL == p_lbs)           ||
       (NULL == data)            ||
       (APP_SYNC_BYTE != data[0])
      )
    {
        return;
    }

    if(COMMAND_TYPE_TRIGGER_OTA == data[2])
    {
        switch(data[3])
        {
            case 0x00:
                Flag.is_enter_ota  = false;
                break;

            case 0x01:
                Flag.is_enter_ota  = true;
                break;

            default:
                Flag.is_enter_ota  = false;
                break;
        }
    }
}
/**@brief   Function for handling events from the data characteristic of the LED Button service. 
 *
 * @details A pointer to this function is passed to the service in its init structure.
 * 
 * @param[in] p_lbs  Pointer to the LED Button Service structure to which the included data relates.
 * @param[in] data   Pointer to the included data.
 */
static void data_write_handler(ble_lbs_t * p_lbs, uint8_t* data)
{
    ble_communication_t *p_com = NULL;
    if((NULL == p_lbs)           ||
       (NULL == data)            ||
       (APP_SYNC_BYTE != data[0])
      )
    {
        return;
    }
    p_com = &temp_req;
//    memset(p_com, 0, sizeof(ble_communication_t));
    p_com->sequence_id = data[1];
    if(COMMAND_TYPE_QUERY_TEMPERATURE_DATA == data[2])
    {
        switch(data[3])
        {
            case 0x00:
                p_com->enable = false;
                p_com->report_per_interval_ticks = 0;
                p_com->current_ms_ticks = 0;
                break;

            case 0x01:
                p_com->enable = true;
                p_com->report_per_interval_ticks = 0;
                p_com->current_ms_ticks = 0;
                break;

            case 0x02:      /*2: report every 1 second*/
                p_com->enable = true;
                p_com->report_per_interval_ticks = 1*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;

            case 0x03:     /*3: report every 5 seconds*/
                p_com->enable = true;
                p_com->report_per_interval_ticks = 5*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;

            case 0x04:      /* 4: report every 10 seconds*/
                p_com->enable = true;
                p_com->report_per_interval_ticks = 10*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;

            case 0x05:      /*5: report every 30 seconds */
                p_com->enable = true;
                p_com->report_per_interval_ticks = 30*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;

            case 0x06:      /*6: report every 1 minutes */
                p_com->enable = true;
                p_com->report_per_interval_ticks = 60*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;

            case 0x07:      /*7: report every 5 minutes */
                p_com->enable = true;
                p_com->report_per_interval_ticks = 60*5*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;

            case 0x08:      /*8: report every 10 minutes */
                p_com->enable = true;
                p_com->report_per_interval_ticks = 60*10*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;

            case 0x09:      /* 9: report every 30 minutes*/
                p_com->enable = true;
                p_com->report_per_interval_ticks = 60*30*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;

            case 0x0a:      /* 10: report every 1 hour*/
                p_com->enable = true;
                p_com->report_per_interval_ticks = 60*60*1*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;

            case 0x0b:      /* 11: report every 2 hours*/
                p_com->enable = true;
                p_com->report_per_interval_ticks = 60*60*2*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;

            case 0x0c:      /* 12: report every 6 hours*/
                p_com->enable = true;
                p_com->report_per_interval_ticks = 60*60*6*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;

            case 0x0d:      /* 13: report every 12 hours*/
                p_com->enable = true;
                p_com->report_per_interval_ticks = 60*60*12*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;

            case 0x0e:      /* 14: report every 1 day*/
                p_com->enable = true;
                p_com->report_per_interval_ticks = 60*60*24*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;

            default:
                break;
        }

        switch(data[4])
        {
            case 0x00:
                p_com->data_format = COMMUNICATION_DATA_ONLY;
                break;

            case 0x01:
                p_com->data_format = COMMUNICATION_DATA_WITH_TIMESTAMP;
                break;

            default:
                p_com->data_format = COMMUNICATION_DATA_ONLY;
                break;
        }

        switch(data[5])
        {
            case 0x01:
                p_com->forced_to_store = true;
                break;

            default:
                p_com->forced_to_store = false;
                break;
        }

        if(p_com->enable == true)
        {
            p_com->sync_data_status = COM_DATA_SYNC_TRIG;
        }
    }
}

/**@brief   Function for handling events from the LED characteristic of the LED Button service. 
 *
 * @details A pointer to this function is passed to the service in its init structure.
 * 
 * @param[in] p_lbs  Pointer to the LED Button Service structure to which the included data relates.
 * @param[in] data   Pointer to the included data.
 */
static void led_setting_write_handler(ble_lbs_t * p_lbs, uint8_t* data)
{
    ble_communication_t *p_com = NULL;
    int32_t             offset = 0;

    if((NULL == p_lbs)           ||
       (NULL == data)            ||
       (APP_SYNC_BYTE != data[0])
      )
    {
        return;
    }

    if(COMMAND_TYPE_SET_DATE_TIME == data[2])
    {
        uint32_t latest_time;
        p_com = &date_time_req;
        memset(p_com, 0, sizeof(ble_communication_t));
        p_com->sequence_id = data[1];
        p_com->enable = true;
        p_com->report_per_interval_ticks = 0;
        p_com->current_ms_ticks = 0;

        app_timer_stop(m_clock_timer_id); //stop timer here
        latest_time  = 0;
        latest_time += ((data[3]&0x000000FF)<<0);
        latest_time += ((data[4]&0x000000FF)<<8);
        latest_time += ((data[5]&0x000000FF)<<16);
        latest_time += ((data[6]&0x000000FF)<<24);
        *time_zone   = data[7];
        offset = time_get_offset_seconds(latest_time,*timestamp_ptr);
        /*FIXME. If offset time is bigger than 5 minutes, then this offset will be valid. which is to avoid the bad network scenario,
        for this scenario, BLE connection will be lost and reconnect again in a short time, so mobile APP will set latest time several
        times shortly. which may lead to time offset is invalid. because the important function of offset time is get the difference between
        default local time and time in mobile APP, which is used to adjust the time stamp of previous time stamp of temperature data in
        SPI flash,specially during the period with default local time after factory.
        */
        if(abs(offset) > DEFAULT_TIME_OFFSET_THRESHOLD)
        {
            local_time_offset = offset;
        }
        DBG_PRINT("time is updated, offset is %d seconds \r\n",local_time_offset);
        *timestamp_ptr = latest_time;                                //update local time.
        app_timer_start(m_clock_timer_id, CLOCK_MEAS_INTERVAL, NULL); //stop timer here
    }
    else if(COMMAND_TYPE_SET_PATIENT_ID == data[2])
    {
        p_com = &patient_req;
        p_com->sequence_id = data[1];
        p_com->enable = true;
        p_com->report_per_interval_ticks = 0;
        p_com->current_ms_ticks = 0;
        patient_id = LITTLE_ENDIAN_2BYTES_TO_16BIT(data[3],data[4]);
    }
}
/**@brief   Function for handling events from the version characteristic of the LED Button service. 
 *
 * @details A pointer to this function is passed to the service in its init structure.
 * 
 * @param[in] p_lbs  Pointer to the LED Button Service structure to which the included data relates.
 * @param[in] data   Pointer to the included data.
 */
static void version_write_handler(ble_lbs_t * p_lbs, uint8_t* data)
{
    ble_communication_t *p_com = NULL;

    if((NULL == p_lbs)           ||
       (NULL == data)            ||
       (APP_SYNC_BYTE != data[0])
      )
    {
        return;
    }
    sequence_id = data[1];
    if(COMMAND_TYPE_QUERY_VERSION == data[2])
    {
        /**query Firmware/Hardware version */
        Flag.is_inquirying_device_version = true;
    }
    else if(COMMAND_TYPE_QUERY_DEVICE_ID == data[2])
    {
        /**query Device ID */
        Flag.is_inquirying_ficr_deviceid = true;
    }
    else if(COMMAND_TYPE_QUERY_MAC_ADDR == data[2])
    {
        /**query MAC */
        Flag.is_inquirying_mac_addr = true;
    }
    else if(COMMAND_TYPE_QUERY_DATE_TIME == data[2])
    {
        /**query current date and time */
        Flag.is_inquirying_local_time = true;
    }
    else if(COMMAND_TYPE_QUERY_BATTERY_LEVEL == data[2])
    {
        p_com = &battery_level;
       // memset(p_com, 0, sizeof(ble_communication_t));
        p_com->sequence_id = data[1];
        /**query current battery voltage level */
        switch(data[3])
        {
            case 0x00:
                p_com->enable = false;
                p_com->report_per_interval_ticks = 0;
                p_com->current_ms_ticks = 0;
                break;

            case 0x01:
                p_com->enable = true;
                p_com->report_per_interval_ticks = 0;
                p_com->current_ms_ticks = 0;
                break;

            case 0x02:      /*2: report every 1 second*/
                p_com->enable = true;
                p_com->report_per_interval_ticks = 1*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;

            case 0x03:     /*3: report every 5 seconds*/
                p_com->enable = true;
                p_com->report_per_interval_ticks = 5*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;

            case 0x04:      /* 4: report every 10 seconds*/
                p_com->enable = true;
                p_com->report_per_interval_ticks = 10*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;

            case 0x05:      /*5: report every 30 seconds */
                p_com->enable = true;
                p_com->report_per_interval_ticks = 30*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;

            case 0x06:      /*6: report every 1 minutes */
                p_com->enable = true;
                p_com->report_per_interval_ticks = 60*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;

            case 0x07:      /*7: report every 5 minutes */
                p_com->enable = true;
                p_com->report_per_interval_ticks = 60*5*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;

            case 0x08:      /*8: report every 10 minutes */
                p_com->enable = true;
                p_com->report_per_interval_ticks = 60*10*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;

            case 0x09:      /* 9: report every 30 minutes*/
                p_com->enable = true;
                p_com->report_per_interval_ticks = 60*30*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;

            case 0x0a:      /* 10: report every 1 hour*/
                p_com->enable = true;
                p_com->report_per_interval_ticks = 60*60*1*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;

            case 0x0b:      /* 11: report every 2 hours*/
                p_com->enable = true;
                p_com->report_per_interval_ticks = 60*60*2*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;

            case 0x0c:      /* 12: report every 6 hours*/
                p_com->enable = true;
                p_com->report_per_interval_ticks = 60*60*6*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;

            case 0x0d:      /* 13: report every 12 hours*/
                p_com->enable = true;
                p_com->report_per_interval_ticks = 60*60*12*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;

            case 0x0e:      /* 14: report every 1 day*/
                p_com->enable = true;
                p_com->report_per_interval_ticks = 60*60*24*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                break;


            default:
                break;
        }

    }
    else if(COMMAND_TYPE_QUERY_MOTION_STATUS == data[2])
    {
        /**query patient motion status */
        Flag.is_inquirying_motion_status = true;
    }
    else if(COMMAND_TYPE_QUERY_LEG_FLAG == data[2])
    {
        /**query L/R leg */
        Flag.is_inquirying_leg_flag = true;
    }
    else if(COMMAND_TYPE_DUMP_FLASH_DATA == data[2])
    {
        Flag.is_requesting_extended_flash_data = true;
    }
    else if(COMMAND_TYPE_QUERY_LOG == data[2])
    {
        Flag.is_inquirying_log = true;
    }
    else if(COMMAND_TYPE_CLR_LOG == data[2])
    {
        Flag.is_clearing_log = true;
    }
    else
    {

    }
}

#ifdef BLE_DFU_APP_SUPPORT
/**@brief Function for stopping advertising.
 */
static void advertising_stop(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for loading application-specific context after establishing a secure connection.
 *
 * @details This function will load the application context and check if the ATT table is marked as
 *          changed. If the ATT table is marked as changed, a Service Changed Indication
 *          is sent to the peer if the Service Changed CCCD is set to indicate.
 *
 * @param[in] p_handle The Device Manager handle that identifies the connection for which the context
 *                     should be loaded.
 */
static void app_context_load(dm_handle_t const * p_handle)
{
    uint32_t                 err_code;
    static uint32_t          context_data;
    dm_application_context_t context;

    context.len    = sizeof(context_data);
    context.p_data = (uint8_t *)&context_data;

    err_code = dm_application_context_get(p_handle, &context);
    if (err_code == NRF_SUCCESS)
    {
        // Send Service Changed Indication if ATT table has changed.
        if ((context_data & (DFU_APP_ATT_TABLE_CHANGED << DFU_APP_ATT_TABLE_POS)) != 0)
        {
            err_code = sd_ble_gatts_service_changed(m_conn_handle, APP_SERVICE_HANDLE_START, BLE_HANDLE_MAX);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != BLE_ERROR_INVALID_CONN_HANDLE) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
                (err_code != NRF_ERROR_BUSY) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
            {
                APP_ERROR_HANDLER(err_code);
            }
        }

        err_code = dm_application_context_delete(p_handle);
        APP_ERROR_CHECK(err_code);
    }
    else if (err_code == DM_NO_APP_CONTEXT)
    {
        // No context available. Ignore.
    }
    else
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/** @snippet [DFU BLE Reset prepare] */
/**@brief Function for preparing for system reset.
 *
 * @details This function implements @ref dfu_app_reset_prepare_t. It will be called by
 *          @ref dfu_app_handler.c before entering the bootloader/DFU.
 *          This allows the current running application to shut down gracefully.
 */
static void reset_prepare(void)
{
    uint32_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Disconnect from peer.
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        err_code = bsp_indication_set(BSP_INDICATE_IDLE);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        // If not connected, the device will be advertising. Hence stop the advertising.
        advertising_stop();
    }

    err_code = ble_conn_params_stop();
    APP_ERROR_CHECK(err_code);

    nrf_delay_ms(500);
}
/** @snippet [DFU BLE Reset prepare] */
#endif // BLE_DFU_APP_SUPPORT

/**@brief Function for initializing global variable that will be used by the application.
 */
void app_various_init(void)
{
    memset(&temp_req,0,sizeof(ble_communication_t));
    memset(&battery_level,0,sizeof(ble_communication_t));
    memset(&date_time_req,0,sizeof(ble_communication_t));
    memset(&patient_req,0,sizeof(ble_communication_t));
    memset(&acc_inquiry,0,sizeof(ble_communication_t));
    memset(&device_status_inquiry,0,sizeof(ble_communication_t));
    memset(&Flag,0,sizeof(Flag_t));

    nrf_gpio_cfg_output(OP_CONTROL_PIN_NUMBER);
    Temperature_calculate_pwroff();
    nrf_gpio_cfg_output(FACTORY_TEST_PIN);
    nrf_gpio_pin_set(FACTORY_TEST_PIN);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_lbs_init_t init;

    init.ota_write_handler         = ota_write_handler;
    init.data_write_handler        = data_write_handler;
    init.led_setting_write_handler = led_setting_write_handler;
    init.version_write_handler     = version_write_handler;
#ifndef VIGO_CLEAN_UP
    init.test_mode_write_handler   = test_mode_write_handler;
    init.calibration_write_handler = calibration_write_handler;
    init.button_write_handler      = button_write_handler;
#endif
    err_code = ble_lbs_init(&m_lbs, &init);
    APP_ERROR_CHECK(err_code);

#ifdef BLE_DFU_APP_SUPPORT
    /** @snippet [DFU BLE Service initialization] */
    ble_dfu_init_t   dfus_init;

    // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));

    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.error_handler = NULL;
    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.revision      = DFU_REVISION;

    err_code = ble_dfu_init(&m_dfus, &dfus_init);
    APP_ERROR_CHECK(err_code);

    dfu_app_reset_prepare_set(reset_prepare);
    dfu_app_dm_appl_instance_set(m_app_handle);
    /** @snippet [DFU BLE Service initialization] */
#endif // BLE_DFU_APP_SUPPORT
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_SUCCEEDED)
    {
        Flag.is_ble_conn_params_update_finish = true;
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Database discovery collector initialization.
 */
static void db_discovery_init(void)
{
    uint32_t err_code = ble_db_discovery_init();

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting timers.
*/
static void application_timers_start(void)
{
    /* Start application timers.*/
    uint32_t err_code;
    err_code = app_timer_start(m_clock_timer_id, CLOCK_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);


}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
#if 0
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
#endif
}




/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}



/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t              err_code;
    const ble_gap_evt_t   * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        {
            DBG_PRINT("[APPL]: GAP event connected.\r\n");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            Flag.is_ble_connected = true;
            break;
        }
        case BLE_GAP_EVT_DISCONNECTED:
        {
            DBG_PRINT("[APPL]: GAP event disconnected.\r\n");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            Flag.is_ble_connected = false;
            temp_req.sync_data_status = COM_DATA_SYNC_NONE;
            /*As Jie's requirment
            "当APP和FW各种原因断开后，FW自动恢复到1min一次的采样频率（FW激活后单独工作始终1min采样，idle状态下判断时的设置都不变"*/
            temp_req.report_per_interval_ticks = DEFAULT_INQUIRE_TEMP_INTERVAL*MAX_PERIOD_TIME;
            break;
        }
        case BLE_EVT_TX_COMPLETE:
        {
            DBG_PRINT("[APPL]: BLE event TX complete.\r\n");
            Flag.is_notify_tx_completed = true;
            break;
        }
        case BLE_GAP_EVT_TIMEOUT:
        {
            if(p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                DBG_PRINT("[APPL]: Connection Request timed out.\r\n");
            }
            else if(p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
            {
                err_code = ble_advertising_start(BLE_ADV_MODE_SLOW);
                APP_ERROR_CHECK(err_code);
            }
            break;
        }
        default:
            DBG_PRINT("[APPL]: BLE event %d.\r\n",p_ble_evt->header.evt_id);
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
#ifdef BLE_DFU_APP_SUPPORT
    ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
#endif // BLE_DFU_APP_SUPPORT

    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    /*YOUR_JOB add calls to _on_ble_evt functions from each service your application is using
    ble_xxs_on_ble_evt(&m_xxs, p_ble_evt);
    ble_yys_on_ble_evt(&m_yys, p_ble_evt);
    */
    ble_lbs_on_ble_evt(&m_lbs, p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    ble_gap_addr_t ble_mac;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_4000MS_CALIBRATION, false);

#if defined(S110) || defined(S130) || defined(S132)
    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
#if (defined(S130) || defined(S132))
    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
#endif
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
#endif

    //Set PUBLIC bluetooth MAC from manufactory data.
    ble_mac.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
    ble_mac.addr[0] = LITTLE_ENDIAN_32BIT_BYTE1(MFG_BLE_MAC_L);
    ble_mac.addr[1] = LITTLE_ENDIAN_32BIT_BYTE2(MFG_BLE_MAC_L);
    ble_mac.addr[2] = LITTLE_ENDIAN_32BIT_BYTE3(MFG_BLE_MAC_L);
    ble_mac.addr[3] = LITTLE_ENDIAN_32BIT_BYTE4(MFG_BLE_MAC_L);
    ble_mac.addr[4] = LITTLE_ENDIAN_16BIT_BYTE1((MFG_BLE_MAC_H & 0xffff));
    ble_mac.addr[5] = LITTLE_ENDIAN_16BIT_BYTE2((MFG_BLE_MAC_H & 0xffff));

    err_code = sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &ble_mac);
    APP_ERROR_CHECK(err_code);


    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;
        #if defined(GSENSOR_INTERRUPT_WAY)   
        case BSP_EVENT_LSM6DSL_INT:
        {
            lsm6dsl_all_sources_t all_sources;
            record_motion_msticks = msticks;
            lsm6dsl_inquiry_all_src(&all_sources);
            DBG_PRINT("taxis_int_pin_handler \r\n");
            break;
        }
        #endif

        default:
            break;
    }
}


/**@brief Function for handling the Device Manager events.
 *
 * @param[in] p_evt  Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);

#ifdef BLE_DFU_APP_SUPPORT
    if (p_event->event_id == DM_EVT_LINK_SECURED)
    {
        app_context_load(p_handle);
    }
#endif // BLE_DFU_APP_SUPPORT

    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 *
 */
static void device_manager_init(void)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = false};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(bool is_idle)
{
    uint32_t                 err_code;
    ble_advdata_t            advdata;
    ble_advdata_t            scanrsp;
    ble_adv_modes_config_t options = {0};
    ble_advdata_manuf_data_t manuf_data;
    uint32_t                 DEVICEID_0;
    uint32_t                 DEVICEID_1;
    uint8_t                  sn[8];

    DEVICEID_0 = NRF_FICR->DEVICEID[0];
    DEVICEID_1 = NRF_FICR->DEVICEID[1];

    sn[0] = LITTLE_ENDIAN_32BIT_BYTE1(DEVICEID_0);
    sn[1] = LITTLE_ENDIAN_32BIT_BYTE2(DEVICEID_0);
    sn[2] = LITTLE_ENDIAN_32BIT_BYTE3(DEVICEID_0);
    sn[3] = LITTLE_ENDIAN_32BIT_BYTE4(DEVICEID_0);
    sn[4] = LITTLE_ENDIAN_32BIT_BYTE1(DEVICEID_1);
    sn[5] = LITTLE_ENDIAN_32BIT_BYTE2(DEVICEID_1);
    sn[6] = LITTLE_ENDIAN_32BIT_BYTE3(DEVICEID_1);
    sn[7] = LITTLE_ENDIAN_32BIT_BYTE4(DEVICEID_1);

    manuf_data.company_identifier = LITTLE_ENDIAN_2BYTES_TO_16BIT(sn[0],sn[1]);
    manuf_data.data.size = sizeof(sn) - 2;
    manuf_data.data.p_data = sn + 2;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    if(is_idle == false)
    {
        advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
        advdata.uuids_complete.p_uuids  = m_adv_uuids;
    }

    memset(&scanrsp, 0, sizeof(scanrsp));
    if(is_idle == false)
    {
        scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
        scanrsp.uuids_complete.p_uuids  = m_adv_uuids;
    }
    scanrsp.p_manuf_specific_data = &manuf_data;

    options.ble_adv_slow_enabled  = BLE_ADV_SLOW_ENABLED;
    options.ble_adv_slow_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;
    if(is_idle == false)
    {
        options.ble_adv_slow_interval = APP_ADV_SMALL_INTERVAL;
    }
    else
    {
        options.ble_adv_slow_interval = APP_ADV_INTERVAL;
    }

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 */
static void buttons_leds_init(void)
{
    #if !defined(GSENSOR_INTERRUPT_WAY)
    uint32_t err_code = bsp_init(BSP_INIT_LED,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);
    #else
    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, NULL);
    APP_ERROR_CHECK(err_code);
    #endif
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code;

    err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the UART error.
 */
void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

/**
 * @brief ADC initialization.
 */
void adc_config(uint8_t number)
{
    nrf_adc_config_t nrf_adc_config = NRF_ADC_CONFIG_DEFAULT;

    // Initialize and configure ADC
    switch (number)
    {
        case 0:
            nrf_adc_config.scaling = NRF_ADC_CONFIG_SCALING_INPUT_TWO_THIRDS;
            nrf_adc_configure((nrf_adc_config_t *)&nrf_adc_config);
            nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_2);
            break;

        case 1:
            nrf_adc_config.scaling = NRF_ADC_CONFIG_SCALING_INPUT_TWO_THIRDS;
            nrf_adc_configure( (nrf_adc_config_t *)&nrf_adc_config);
            nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_3);
            break;

        case 2:
            nrf_adc_config.scaling = NRF_ADC_CONFIG_SCALING_INPUT_TWO_THIRDS;
            nrf_adc_configure( (nrf_adc_config_t *)&nrf_adc_config);
            nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_4);
            break;

        case 3:
            nrf_adc_config.scaling = NRF_ADC_CONFIG_SCALING_INPUT_TWO_THIRDS;
            nrf_adc_configure( (nrf_adc_config_t *)&nrf_adc_config);
            nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_5);
            break;

        case 4:
            nrf_adc_config.scaling = NRF_ADC_CONFIG_SCALING_INPUT_TWO_THIRDS;
            nrf_adc_configure( (nrf_adc_config_t *)&nrf_adc_config);
            nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_6);
            break;

        case 5:
            nrf_adc_config.scaling = NRF_ADC_CONFIG_SCALING_INPUT_TWO_THIRDS;
            nrf_adc_configure( (nrf_adc_config_t *)&nrf_adc_config);
            nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_7);
            break;

        case 6:
            nrf_adc_configure( (nrf_adc_config_t *)&nrf_adc_config);
            nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_1);
            break;

    }
    adc_channel = number;
    nrf_adc_int_enable(ADC_INTENSET_END_Enabled << ADC_INTENSET_END_Pos);
    NVIC_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_HIGH);
    NVIC_EnableIRQ(ADC_IRQn);

}

/**@brief Function for handling the ADC interrupt.
 *
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
void ADC_IRQHandler(void)
{
    nrf_adc_conversion_event_clean();
    nrf_adc_stop();

    adc_value[adc_channel] = nrf_adc_result_get();
    m_adc_completed = true;

}

/**
*@brief Function to get ADC sample.
*/

void adc_get_sample(uint8_t num)
{
    adc_config(num);
    m_adc_completed = false;
    nrf_adc_start();
    while (!m_adc_completed)
    {
       power_manage();
    }

}
/**@brief Function for storage temperature,stamptime,battery level to extended flash.
 */
void adc_all_samples_store_to_flash(void)
{
    extended_flash_param_t *p_flash = extended_flash_rw_param_ptr;
    uint8_t           buf[STORED_TEMP_DATA_LEN];
    memset(buf, 0, STORED_TEMP_DATA_LEN);
    get_temperature_degree(adc_value[0],&buf[0]);
    get_temperature_degree(adc_value[1],&buf[1]);
    get_temperature_degree(adc_value[2],&buf[2]);
    get_temperature_degree(adc_value[3],&buf[3]);
    get_temperature_degree(adc_value[4],&buf[4]);
    get_temperature_degree(adc_value[5],&buf[5]);
    buf[6]  = percentage_batt_lvl;                                    
    buf[7]  = ((*total_step_counter&0x00FF)>>0);
    buf[8]  = ((*total_step_counter&0xFF00)>>8);
    buf[9]  = ((*timestamp_ptr&0x000000FF)>>0);
    buf[10] = ((*timestamp_ptr&0x0000FF00)>>8);
    buf[11] = ((*timestamp_ptr&0x00FF0000)>>16);
    buf[12] = ((*timestamp_ptr&0xFF000000)>>24);
    buf[13] = 0; /*reserved*/
    buf[14] = 0; /*reserved*/
    buf[15] = 0; /*reserved*/
    if((p_flash->write_addr + STORED_TEMP_DATA_LEN-1) < p_flash->end_addr)
    {
        SpiFlash_Erase_Program_Onerecord(buf,p_flash->write_addr,STORED_TEMP_DATA_LEN);
        p_flash->write_addr += STORED_TEMP_DATA_LEN;
        if(p_flash->write_addr >= p_flash->end_addr)
        {
            p_flash->write_addr = 0;
            p_flash->roll_back = true;
        }
        if(p_flash->roll_back)
        {
            if(p_flash->write_addr > (p_flash->read_addr&Sector_Size_4k_Mask))
            {
                p_flash->read_addr &= Sector_Size_4k_Mask;
                p_flash->read_addr += Sector_Size_4k;
                if(p_flash->read_addr >= p_flash->end_addr)
                {
                    p_flash->read_addr = 0;
                }
            }
        }
    }
}

/**@brief Function for initialize 3-axis accelerometer and 3-axis gyroscope LSM6DSL.
 *
 * @details This function will set LSM6DSL default wakeup threshold value, and enable wake up interrupt and STEP feature.
 *          twi interface is initialized as hardware twi,not software twi.
 *
 * @param[in] none.
 */
int twi_acc_gyro_init (void)
{
    bool ret = false;
    ret = twi_master_init();
    if (ret == false)
    {
        DBG_PRINT("twi master failed to initiate \r\n");
        return -1;
    }

#if !defined(GSENSOR_INTERRUPT_WAY)
    nrf_gpio_cfg_input(LSM6_INT_PIN,NRF_GPIO_PIN_PULLUP);  //GPIO21 is configured as INPUT
#endif

    LSM6DSL_Init();
    //LSM6DSL_SetMode(LSM6DSL_POWER_DOWN);
    record_motion_msticks = msticks;
    acc_inquiry.report_per_interval_ticks = 1*MAX_PERIOD_TIME;
    acc_inquiry.current_ms_ticks = msticks;
    acc_inquiry.enable = true;
    return 0;

}
/**@brief Function for registered idle status init.
 */
static void device_status_registered_idle_init(void)
{
    ble_communication_t      *p_com = &device_status_inquiry;
    uint32_t               err_code = NRF_SUCCESS;
    advertising_init(true);
    p_com->report_per_interval_ticks = 10*MAX_PERIOD_TIME;// 10 seconds
    p_com->current_ms_ticks = msticks;
    err_code = ble_advertising_start(BLE_ADV_MODE_SLOW);
    APP_ERROR_CHECK(err_code);
    /*clear the communication parameters*/
    memset(&temp_req,0,sizeof(ble_communication_t));
    memset(&battery_level,0,sizeof(ble_communication_t));
}
/**@brief Function for registered normal status init.
 */
static void device_status_registered_normal_init(void)
{
    ble_communication_t      *p_com = &device_status_inquiry;
    ble_communication_t *p_temp_req = &temp_req;
    uint32_t               err_code = NRF_SUCCESS;
    advertising_init(false);
    p_com->report_per_interval_ticks = 2*60*MAX_PERIOD_TIME;// 120 seconds
    p_com->current_ms_ticks = msticks;
    err_code = ble_advertising_start(BLE_ADV_MODE_SLOW);
    APP_ERROR_CHECK(err_code);
    /*Add new feature that automatically inquire the temperature data when device enters normal status.*/
    p_temp_req->enable = true;
    p_temp_req->report_per_interval_ticks = DEFAULT_INQUIRE_TEMP_INTERVAL*MAX_PERIOD_TIME;
    p_temp_req->current_ms_ticks = msticks;
    p_temp_req->data_format = COMMUNICATION_DATA_WITH_TIMESTAMP;
    p_temp_req->forced_to_store = true;
}
/**@brief Function for initialize device status.
 *
 * @details Must be called after device_pstorage_init() API
 *
 * @param[in] none.
 */
static void device_status_init(void)
{
    if(internal_flash_param.mode == DEVICE_REGISTERED_IDLE_STATUS)
    {
        device_status_registered_idle_init();
        Flag.is_device_registered = true;
    }
    else if(internal_flash_param.mode == DEVICE_REGISTERED_NORMAL_STATUS)
    {
        device_status_registered_normal_init();
        Flag.is_device_registered = true;
    }
    else if(internal_flash_param.mode == DEVICE_UNREGISTERED_STATUS)
    {
        Flag.is_device_registered = false;
    }
    else
    {
        Flag.is_device_registered = false;
    }
}
/**
 * @brief WDT events handler.
 */
static void wdt_event_handler(void)
{
   // LEDS_OFF(LEDS_MASK);
   DBG_PRINT("Reset \r\n");

   //NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
}

void device_watchdog_launch(void)
{
    uint32_t err_code= NRF_SUCCESS;

    //Configure WDT.
    nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
    err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();
}
/**@brief Function for handling application msg in the main process.
 */
static void handle_app_message(void)
{
    ble_communication_t  *p_com    = NULL;
    uint32_t             err_code = NRF_SUCCESS;
    uint8_t              buf[CHAR_SEND_MAX_LEN];
    if(Flag.is_internal_flash_update)
    {
        spi_flash_param_pstorage_store(&internal_flash_param,UPDATE_SPI_FLASH_RW_STORE);
        Flag.is_internal_flash_update = false;
    }
    if(Flag.is_inquirying_device_version)
    {
        version_info_encode(buf,sizeof(buf));
        ble_lbs_version_data_update(&m_lbs, buf);
        Flag.is_inquirying_device_version = false;
    }
    if(Flag.is_inquirying_ficr_deviceid)
    {
        device_id_encode(buf,sizeof(buf));
        ble_lbs_version_data_update(&m_lbs, buf);
        Flag.is_inquirying_ficr_deviceid = false;
    }
    if(Flag.is_inquirying_mac_addr)
    {
        MAC_address_encode(buf,sizeof(buf));
        ble_lbs_version_data_update(&m_lbs, buf);
        Flag.is_inquirying_mac_addr = false;
    }
    if(Flag.is_inquirying_local_time)
    {
        current_date_time_encode(COMMAND_TYPE_QUERY_DATE_TIME,buf,sizeof(buf));
        ble_lbs_version_data_update(&m_lbs, buf);
        Flag.is_inquirying_local_time = false;
    }
    if(Flag.is_inquirying_motion_status)
    {
        if ((record_motion_msticks+5000) > msticks)
        {
            motion_status = MOTION_ACTIVITY;
        }
        else
        {
            motion_status = MOTION_INACTIVITY;
        }
        patient_motion_status_encode(buf,sizeof(buf));
        ble_lbs_version_data_update(&m_lbs, buf);
        Flag.is_inquirying_motion_status = false;
    }
    if(Flag.is_inquirying_leg_flag)
    {
        patient_leg_flag_encode(buf,sizeof(buf));
        ble_lbs_version_data_update(&m_lbs, buf);
        Flag.is_inquirying_leg_flag = false;
    }
    if(Flag.is_inquirying_log)
    {
        log_inquirying_encode(buf,sizeof(buf));
        ble_lbs_version_data_update(&m_lbs, buf);
        Flag.is_inquirying_log = false;
    }
    if(Flag.is_clearing_log)
    {
        log_clearing_encode(buf,sizeof(buf));
        ble_lbs_version_data_update(&m_lbs, buf);
        Flag.is_clearing_log = false;
        Flag.is_internal_flash_update = true;
    }
    if(Flag.is_requesting_extended_flash_data)
    {
        ble_gap_conn_params_t new_params;
       /*For a temporary period requires shorter connection interval to transfer a higher mount of data.*/
        memset(&new_params,0,sizeof(ble_gap_conn_params_t));
        new_params.min_conn_interval = SYNC_MIN_CONN_INTERVAL;           /**< Minimum acceptable connection interval (0.5 seconds). */
        new_params.max_conn_interval = SYNC_MAX_CONN_INTERVAL;           /**< Maximum acceptable connection interval (1 second). */
        new_params.conn_sup_timeout  = SYNC_CONN_SUP_TIMEOUT;            /**< Connection supervisory timeout (4 seconds). */
        new_params.slave_latency     = SYNC_SLAVE_LATENCY;               /**< Slave latency. */
        err_code = ble_conn_params_change_conn_params(&new_params);
        if(err_code == NRF_SUCCESS)
        {
            SpiFlash_Deep_Sleep_Exit();//Wake up SPI flash.
            flash_data_dump_encode_notify(buf,sizeof(buf));
            SpiFlash_Deep_Sleep_Enter();

            memset(&new_params,0,sizeof(ble_gap_conn_params_t));
            new_params.min_conn_interval = MIN_CONN_INTERVAL;
            new_params.max_conn_interval = MAX_CONN_INTERVAL;
            new_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;
            new_params.slave_latency     = SLAVE_LATENCY;
            err_code = ble_conn_params_change_conn_params(&new_params);
            APP_ERROR_CHECK(err_code);
            dump_flash_addr = 0;
        }
        Flag.is_requesting_extended_flash_data = false;
    }
    p_com = &battery_level;
    if(p_com->enable)
    {
        if((p_com->report_per_interval_ticks == 0)                          ||
            expire(p_com->current_ms_ticks,p_com->report_per_interval_ticks)
          )
        {
            uint16_t batt_lvl_in_milli_volts;
            adc_get_sample(6);
            batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_value[6]);
            if(batt_lvl_in_milli_volts > battery_last_voltage)
            {
                batt_lvl_in_milli_volts = battery_last_voltage;
            }
            else
            {
                batt_lvl_in_milli_volts = battery_last_voltage-((battery_last_voltage-batt_lvl_in_milli_volts)/5);
            }
            battery_last_voltage = batt_lvl_in_milli_volts;
            percentage_batt_lvl  = battery_level_in_percent(batt_lvl_in_milli_volts);
            battery_voltage_level_encode(buf,sizeof(buf));
            ble_lbs_version_data_update(&m_lbs, buf);
        }
        if(p_com->report_per_interval_ticks == 0)
        {
            p_com->enable = false;
        }
        if(expire(p_com->current_ms_ticks,p_com->report_per_interval_ticks))
        {
            p_com->current_ms_ticks = msticks;
        }
    }
    p_com = &date_time_req;
    if(p_com->enable)
    {
        if(p_com->report_per_interval_ticks == 0)
        {
            p_com->enable = false;
            current_date_time_encode(COMMAND_TYPE_SET_DATE_TIME,buf,sizeof(buf));
            ble_lbs_led_setting_update(&m_lbs, buf);
        }
    }
    p_com = &patient_req;
    if(p_com->enable)
    {
        patient_id_encode(buf,sizeof(buf));
        ble_lbs_led_setting_update(&m_lbs, buf);
        if(p_com->report_per_interval_ticks == 0)
        {
            p_com->enable = false;
        }
    }
    p_com = &temp_req;
    if(p_com->enable)
    {
        int i;
        if(p_com->sync_data_status == COM_DATA_SYNC_TRIG)
        {
            if((extended_flash_rw_param_ptr->write_addr > extended_flash_rw_param_ptr->read_addr) ||
               ((extended_flash_rw_param_ptr->write_addr < extended_flash_rw_param_ptr->read_addr)&&
                (extended_flash_rw_param_ptr->roll_back == true)
               )
              )
            {
                ble_gap_conn_params_t new_params;
                DBG_PRINT("[APPL]: Data sync procedure is triggered \r\n");
                /*For a temporary period requires shorter connection interval to transfer a higher mount of data.*/
                memset(&new_params,0,sizeof(ble_gap_conn_params_t));
                new_params.min_conn_interval = SYNC_MIN_CONN_INTERVAL;           /**< Minimum acceptable connection interval (0.5 seconds). */
                new_params.max_conn_interval = SYNC_MAX_CONN_INTERVAL;           /**< Maximum acceptable connection interval (1 second). */
                new_params.conn_sup_timeout  = SYNC_CONN_SUP_TIMEOUT;            /**< Connection supervisory timeout (4 seconds). */
                new_params.slave_latency     = SYNC_SLAVE_LATENCY;               /**< Slave latency. */
                err_code = ble_conn_params_change_conn_params(&new_params);
                if(err_code == NRF_SUCCESS)
                {
                    SpiFlash_Deep_Sleep_Exit();
                    temp_data_sync_encode_notify(buf,sizeof(buf));
                    SpiFlash_Deep_Sleep_Enter();
                    /*Recovery the default connection parameters Once the process of flash data dumping out is completed. */
                    memset(&new_params,0,sizeof(ble_gap_conn_params_t));
                    new_params.min_conn_interval = MIN_CONN_INTERVAL;
                    new_params.max_conn_interval = MAX_CONN_INTERVAL;
                    new_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;
                    new_params.slave_latency     = SLAVE_LATENCY;
                    err_code = ble_conn_params_change_conn_params(&new_params);
                    APP_ERROR_CHECK(err_code);
                    local_time_offset = 0; // clear local time offset, time stamp needn't to adjust again.
                    p_com->sync_data_status = COM_DATA_SYNC_COMPLETE;
                }
            }
            else
            {
                p_com->sync_data_status = COM_DATA_SYNC_COMPLETE;
                local_time_offset = 0; // clear local time offset, time stamp needn't to adjust again.
            }
        }
        else if((p_com->sync_data_status == COM_DATA_SYNC_COMPLETE) ||
                (p_com->sync_data_status == COM_DATA_SYNC_NONE) 
               )
        {
            if((p_com->report_per_interval_ticks == 0 )                         ||
               expire(p_com->current_ms_ticks,p_com->report_per_interval_ticks))
            {
                Temperature_calculate_pwron();
                nrf_delay_ms(10);
                for (i = 0; i < 6; i++)
                {
                    adc_get_sample(i);
                }
                if(p_com->forced_to_store)
                {
                    SpiFlash_Deep_Sleep_Exit();
                    adc_all_samples_store_to_flash();
                    SpiFlash_Deep_Sleep_Enter();
                }
                DBG_PRINT("[APPL]: collecting temperature data\r\n");

                /*if sync none status, temperature data will NOT be sent out to mobile APP.*/
                if(p_com->sync_data_status != COM_DATA_SYNC_NONE)
                {
                    temp_sensor_data_encode((communication_data_type)p_com->data_format,buf,sizeof(buf));
                    ble_lbs_sensor_data_update(&m_lbs,buf);
                }
                Temperature_calculate_pwroff();
            }
            if(p_com->report_per_interval_ticks == 0 )
            {
                p_com->enable = false;
            }
            if(expire(p_com->current_ms_ticks,p_com->report_per_interval_ticks))
            {
                p_com->current_ms_ticks = msticks;
            }
        }
    }
    else
    {
        Temperature_calculate_pwroff();
    }
    p_com = &acc_inquiry;
    if(p_com->enable)
    {
        uint16_t        current_step_counter = 0;
        static uint16_t last_step_counter    = 0;
        if((p_com->report_per_interval_ticks == 0 )                        ||
           expire(p_com->current_ms_ticks,p_com->report_per_interval_ticks)
          )
        {
            #if !defined(GSENSOR_INTERRUPT_WAY)
            //LSM6DSL_Polling();
            if(TWI_INT_READ() == 0)
            {
                DBG_PRINT("Detected movement \r\n");
                lsm6dsl_all_sources_t all_sources;
                 //WAKE_UP_SRC
                lsm6dsl_inquiry_all_src(&all_sources);
                //DBG_PRINT("WAKEUP INT status is 0x%x \r\n",value);
                record_motion_msticks = msticks;
            }
            else
            {
               //DBG_PRINT("INT is cleared \r\n");
            }
            #endif
            lsm6dsl_GetAccSTEPCounter(&current_step_counter);
            *total_step_counter += (current_step_counter-last_step_counter);
            last_step_counter = current_step_counter;
        }
        if(p_com->report_per_interval_ticks == 0)
        {
            p_com->enable = false;
        }
        if(expire(p_com->current_ms_ticks,p_com->report_per_interval_ticks))
        {
            p_com->current_ms_ticks = msticks;
        }
    }
    if(Flag.is_enter_ota)
    {
        Flag.is_enter_ota = false;
        err_code = sd_power_gpregret_set(BOOTLOADER_DFU_START);
        APP_ERROR_CHECK(err_code);
        NVIC_SystemReset();
    }
}
/**@brief Function for switching device status.
 *
 * @param[in] none.
 */
uint32_t device_switch_status(device_status_t status)
{
    uint32_t                 err_code = NRF_SUCCESS;
    int                      i;
    DBG_PRINT("Device is switching to %d status \r\n", status);
    if(m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Disconnect from peer.
        DBG_PRINT("Disconnect from peer \r\n");
        ble_peripheral_manual_disconnect = true;
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        err_code = ble_conn_params_stop();
        APP_ERROR_CHECK(err_code);
        i = 0;
        while((Flag.is_ble_connected == true) && (i++ < (TIMEOUT_DISCONNECT_PEER/10)))
        {
            nrf_delay_ms(10);
        }

        if (i >= (TIMEOUT_DISCONNECT_PEER/10))
        {
            DBG_PRINT("Time out %d ms to disconnect from peer \r\n",(i*10));
        }
        ble_peripheral_manual_disconnect = false;
    }
    else
    {
        // If not connected, the device will be advertising. Hence stop the advertising.
        DBG_PRINT("Stop advertising \r\n");
        advertising_stop();
    }

    nrf_delay_ms(100);

    if(status == DEVICE_REGISTERED_IDLE_STATUS)
    {
        device_status_registered_idle_init();
        internal_flash_param.mode = DEVICE_REGISTERED_IDLE_STATUS;
        Flag.is_internal_flash_update = true;
    }
    else if(status == DEVICE_REGISTERED_NORMAL_STATUS)
    {
        device_status_registered_normal_init();
        internal_flash_param.mode = DEVICE_REGISTERED_NORMAL_STATUS;
        Flag.is_internal_flash_update = true;
    }
    return 0;
}


/**@brief Function for monitor whether the device status needs to be switched.
 *
 * @param[in] none.
 */
void monitor_device_status(void)
{
    ble_communication_t *p_com = &device_status_inquiry;
    int                 i;
    switch(internal_flash_param.mode)
    {
#if 0  /*Not support unregistered status as required.*/
        case DEVICE_UNREGISTERED_STATUS:
        {
            if(Flag.is_device_registered)
            {
                DBG_PRINT("Device is to be ACTIVATED \r\n");

               // advertising_stop();
                nrf_delay_ms(500);
                advertising_init(true);
                err_code = ble_advertising_start(BLE_ADV_MODE_SLOW);
                APP_ERROR_CHECK(err_code);
                p_com->report_per_interval_ticks = 10*MAX_PERIOD_TIME;
                p_com->current_ms_ticks = msticks;
                internal_flash_param.mode = DEVICE_REGISTERED_IDLE_STATUS;
                spi_flash_param_pstorage_store(&internal_flash_param,UPDATE_SPI_FLASH_RW_STORE);
                DBG_PRINT("Device is activated and in IDLE status by default \r\n");
            }
            break;
        }
  #endif
        case DEVICE_REGISTERED_IDLE_STATUS:
        {
            if(expire(p_com->current_ms_ticks,p_com->report_per_interval_ticks))
            {
                const uint8_t index_table[6]  = {1,1,0,1,1,0};
                uint16_t      degree_table[6] = {0};
                uint8_t       cnt             = 0;
                uint8_t       degree          = 0;
                uint16_t      compare_degree_table[6] = {0};

                /*If there is any movement during the period,start to check the temperature*/
                if(record_motion_msticks > p_com->current_ms_ticks)
                {
                    DBG_PRINT("Device is moved during the period \r\n");
                    Temperature_calculate_pwron();
                    nrf_delay_ms(10);

                    for (i = 0; i < 6; i++)
                    {
                        adc_get_sample(i);
                        get_temperature_degree(adc_value[i],(uint8_t*) &degree);
                        degree_table[i] = 200 + degree;
                        DBG_PRINT("Temperature index:%d, value:%d \r\n",i,degree_table[i]);
                        if (index_table[i] == 0)
                            continue;

                        if ((degree_table[i] >= SMART_SOCK_USING_TEMPERATURE_LOW) && (degree_table[i] <= SMART_SOCK_USING_TEMPERATURE_HIGH))
                        {
                            cnt++;
                        }
                    }
                    Temperature_calculate_pwroff();

                    /*If at least 5 sensors indicated that temperature value are in the range, then start to switch to NORMAL status.*/
                    /*MTS3,MTS5,MTS1，ARCH（前脚掌区域）对应Data0，Data1，Data3，Data4*/
                    /*Condition Version 4
                    条件1：D[0],D[1],D[3],D[4]其中至少2(>=2)个满足在21C-42C范围内*/
                    /*条件2：同时满足 MAX（D[0]：D[5]）-MIN(D[0]：D[5]) >=1 degree 且最大值不等于42.5*/
                    {
                        int j,tmp;
                        for (i = 0; i < 6; i++)
                        {
                            compare_degree_table[i] = degree_table[i];
                        }

                        for (i = 6-1; i > 0; i--)
                        {
                            for (j = 0; j < i; j++)
                            {
                                if (compare_degree_table[j] < compare_degree_table[j+1])
                                {
                                    tmp = compare_degree_table[j];
                                    compare_degree_table[j] = compare_degree_table[j+1];
                                    compare_degree_table[j+1] = tmp;
                                }
                            }
                        }
                    }

                    if((cnt >= 2)                                              && 
                       ((compare_degree_table[0]-compare_degree_table[5] >=10) &&
                        (compare_degree_table[0] != 425)
                       )
                      )
                    {
                        device_switch_status(DEVICE_REGISTERED_NORMAL_STATUS);
                    }
                    else
                    {
                        p_com->current_ms_ticks = msticks;
                    }
                }
                else
                {
                    p_com->current_ms_ticks = msticks;
                }
            }
            break;
        }
        case DEVICE_REGISTERED_NORMAL_STATUS:
        {
            if(expire(p_com->current_ms_ticks,p_com->report_per_interval_ticks))
            {
                bool     index_table[6]  = {true,true,false,true,true,false};
                uint16_t degree_table[6] = {0};
                uint8_t  degree          = 0;
                uint8_t  cnt             = 0;
                uint16_t compare_degree_table[4] = { 0 };
                uint16_t tmp             = 0;
                uint8_t  j               = 0;

                ble_communication_t *p_temp_req = &temp_req;

                /*There is no moving action during the period*/
                if(record_motion_msticks < p_com->current_ms_ticks)
                {
                    DBG_PRINT("Device is static during the period \r\n");
                    Temperature_calculate_pwron();
                    nrf_delay_ms(10);
                    for (i = 0; i < 6; i++)
                    {
                        adc_get_sample(i);
                        get_temperature_degree(adc_value[i],&degree);
                        degree_table[i] = 200 + degree;
                        DBG_PRINT("Temperature index:%d, value:%d \r\n",i,degree_table[i]);
                        if (index_table[i] == false)
                            continue;

                        if ((degree_table[i] > SMART_SOCK_USING_TEMPERATURE_HIGH) || (degree_table[i] < SMART_SOCK_USING_TEMPERATURE_LOW))
                        {
                            cnt++;
                        }
                    }
                    Temperature_calculate_pwroff();
                    nrf_delay_ms(10);
                    {
                        /*条件2：或者满足 D[0],D[2],D[3],D[5] 去掉最低，剩余3个温差在1度内*/
                        compare_degree_table[0] = degree_table[0];
                        compare_degree_table[1] = degree_table[2];
                        compare_degree_table[2] = degree_table[3];
                        compare_degree_table[3] = degree_table[5];
                        for (i = 4-1; i > 0; i--)
                        {
                            for (j = 0; j < i; j++)
                            {
                                if (compare_degree_table[j] < compare_degree_table[j+1])
                                {
                                    tmp = compare_degree_table[j];
                                    compare_degree_table[j] = compare_degree_table[j+1];
                                    compare_degree_table[j+1] = tmp;
                                }
                            }
                        }
                    }

                    /* Condition V4
                    条件1：D[0],D[1],D[3],D[4]其中至少3个满足不在21C-42C范围内*/
                    /*条件2：或者满足 D[0],D[2],D[3],D[5] 去掉最低，剩余3个温差在1度内*/
                    if ((cnt >= 3) || ((compare_degree_table[0] - compare_degree_table[2]) < 10) )
                    {
                        /*FIXME: If data is syncing, don't switch to IDLE mode. which is to avoid interrupt the syncing process*/
                        if(p_temp_req->sync_data_status != COM_DATA_SYNC_TRIG)
                        {
                            device_switch_status(DEVICE_REGISTERED_IDLE_STATUS);
                        }
                        else
                        {
                            p_com->current_ms_ticks = msticks;
                        }
                    }
                    else
                    {
                        p_com->current_ms_ticks = msticks;
                    }
                }
                else
                {
                    p_com->current_ms_ticks = msticks;
                }
            }
            break;
        }
        default:
            break;
    }
}
/**@brief Function for inquirying the reset reason.
 */
static void ble_inquiry_reset_reason(void)
{
    uint32_t reset_reason;
    uint32_t err_code;
    err_code = sd_power_reset_reason_get(&reset_reason);
    APP_ERROR_CHECK(err_code);
    if(!(reset_reason&POWER_RESETREAS))               //power-on-reset or a brown out reset
    {
        internal_flash_param.device_resetreason_log.pwron_reset_cnt++;
    }
    else if(reset_reason&POWER_RESETREAS_SREQ_Msk)    //Reset from AIRCR.SYSRESETREQ
    {
        internal_flash_param.device_resetreason_log.sysresetreq_reset_cnt++;
    }
    else if(reset_reason&POWER_RESETREAS_DOG_Msk)     //Reset from watchdog
    {
        internal_flash_param.device_resetreason_log.watchdog_reset_cnt++;
    }
    else if(reset_reason&POWER_RESETREAS_RESETPIN_Msk)//pin-reset
    {
        internal_flash_param.device_resetreason_log.pin_reset_cnt++;
    }
//    else if(reset_reason&POWER_RESETREAS_OFF_Msk)     //wake-up from OFF mode
//    {
//        
//    }
    err_code = sd_power_reset_reason_clr(POWER_RESETREAS);
    APP_ERROR_CHECK(err_code);
    Flag.is_internal_flash_update = true;
}
/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code = NRF_SUCCESS;
 // Initialize.
#ifndef BUILD_PROD
    app_trace_init();
    DBG_PRINT("\n\rStart: \n\r");
#endif
    timers_init();
    buttons_leds_init();
    app_various_init();
    ble_stack_init();
    device_manager_init();
    gap_params_init();
    services_init();
    conn_params_init();
    db_discovery_init();
    // Start execution.
    application_timers_start();

    spi_master_init();
    device_pstorage_init();
    device_status_init();
    twi_acc_gyro_init();
    err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);  //enable DCDC converter at default.
    APP_ERROR_CHECK(err_code);
    device_watchdog_launch();
    SpiFlash_Deep_Sleep_Enter();
    ble_inquiry_reset_reason();
    UNUSED_VARIABLE(bsp_indication_set(BSP_INDICATE_RESET_OCCUR));
    // Enter main loop.
    for (;;)
    {
        power_manage();
        if(Flag.is_main_process_exe)
        {
            monitor_device_status();
            handle_app_message();
            Flag.is_main_process_exe = false;
        }
    }
}

/**
 * @}
 */
