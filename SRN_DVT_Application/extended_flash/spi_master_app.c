#include <string.h>

#include "nrf_delay.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "nrf_drv_spi.h"
#include "bsp.h"
#include "app_timer.h"
#include "nordic_common.h"
#include "nrf51.h"
#include "spi_master_app.h"
#include "ble_lbs.h"


static volatile bool m_transfer_completed = true;
//static spi_master_ex_state_t m_spi_master_ex_state = (spi_master_ex_state_t)0;

#if (SPI0_ENABLED == 1)
static const nrf_drv_spi_t m_spi_master_0 = NRF_DRV_SPI_INSTANCE(0);
#endif
#if (SPI1_ENABLED == 1)
static const nrf_drv_spi_t m_spi_master_1 = NRF_DRV_SPI_INSTANCE(1);
#endif
#if (SPI2_ENABLED == 1)
static const nrf_drv_spi_t m_spi_master_2 = NRF_DRV_SPI_INSTANCE(2);
#endif

static pstorage_handle_t        g_pstorage_handle;
internal_flash_param_t          internal_flash_param;
extern Flag_t                   Flag;
extern internal_flash_param_t   internal_flash_param;
extern extended_flash_param_t   *extended_flash_rw_param_ptr;
extern uint32_t                 *timestamp_ptr;
extern uint16_t                 *total_step_counter;
extern uint8_t                  *time_zone;

static void spi_flash_pstorage_cb_handler(pstorage_handle_t  * p_handle,
                               uint8_t              op_code,
                               uint32_t             result,
                               uint8_t            * p_data,
                               uint32_t             data_len);
#if (SPI1_ENABLED == 1)
/**@brief Handler for SPI0 master events.
 *
 * @param[in] event SPI master event.
 */
void spi_master_1_event_handler(nrf_drv_spi_event_t event)
{

    switch (event)
    {
        case NRF_DRV_SPI_EVENT_DONE:
            // Check if received data is correct.
            //result = check_buf_equal(m_tx_data_spi, m_rx_data_spi, TX_RX_MSG_LENGTH);
            //APP_ERROR_CHECK_BOOL(result);

            //nrf_drv_spi_uninit(&m_spi_master_1);

            m_transfer_completed = true;
            break;

        default:
            // No implementation needed.
            break;
    }
}
#endif // (SPI1_ENABLED == 1)


void spi_cs_low (void)
{
    // Activate Slave Select signal, if it is to be used.
    nrf_gpio_pin_clear(SPIM1_SS_PIN);
}

void spi_cs_high (void)
{
    // Deactivate Slave Select signal, if it is to be used.
    nrf_gpio_pin_set(SPIM1_SS_PIN);
}


/**@brief Function for initializing a SPI master driver.
 */
void spi_master_init(void)
{
    nrf_drv_spi_t const * p_instance;
    uint32_t err_code = NRF_SUCCESS;

    nrf_drv_spi_config_t config =
    {
        .ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,
        .irq_priority = APP_IRQ_PRIORITY_HIGH,
        .orc          = 0xCC,
        .frequency    = NRF_DRV_SPI_FREQ_8M,
        .mode         = NRF_DRV_SPI_MODE_0,
        .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,//Bits order LSB or MSB
    };

#if (SPI1_ENABLED == 1)
    p_instance = &m_spi_master_1;
    config.sck_pin  = SPIM1_SCK_PIN;
    config.mosi_pin = SPIM1_MOSI_PIN;
    config.miso_pin = SPIM1_MISO_PIN;
    //config.ss_pin   = SPIM0_SS_PIN;
    err_code = nrf_drv_spi_init(p_instance, &config,
            NULL);
#endif // (SPI1_ENABLED == 1)

    APP_ERROR_CHECK(err_code);
    nrf_gpio_cfg_output(SPIM1_SS_PIN);

    spi_cs_high();


}
/**@brief Function for initializing extended flash operation.
 */
void device_pstorage_init(void)
{
    pstorage_module_param_t      param;
    uint32_t                     err_code = NRF_SUCCESS;

    param.block_size  = INTERNAL_FLASH_PARAM_LEN;              //Select block size of 32 bytes
    param.block_count = 1;                                     //Select 1 blocks, total of 32 bytes
    param.cb          = spi_flash_pstorage_cb_handler;         //Set the pstorage callback handler

    err_code = pstorage_register(&param, &g_pstorage_handle);
    if(err_code != NRF_SUCCESS)
    {
        DBG_PRINT("Failed to register with storage module, reason 0x%08X.\r\n", err_code);
    }
    APP_ERROR_CHECK(err_code);
    memset(&internal_flash_param, 0, INTERNAL_FLASH_PARAM_LEN);
    spi_flash_param_pstorage_load(&internal_flash_param);
    if(internal_flash_param.mode == EMPTY_FLASH_VALUE)
    {
        //As requested,unregister status/mode is removed,so IDLE status is the default status after system bring up.
        internal_flash_param.mode = DEVICE_REGISTERED_IDLE_STATUS;
        memset(&internal_flash_param.device_resetreason_log,0,sizeof(device_resetreason_log_t));

        extended_flash_rw_param_ptr->read_addr   = 0;
        extended_flash_rw_param_ptr->write_addr  = 0;
        extended_flash_rw_param_ptr->roll_back   = false;

        *timestamp_ptr = 1514736000;//2018/1/1 0:0:0
        *time_zone = 8;//Ä¬ÈÏ¶«°ËÇø
        *total_step_counter = 0;
        spi_flash_param_pstorage_store(&internal_flash_param, FIRST_SPI_FLASH_RW_STORE);
    }
    extended_flash_rw_param_ptr->start_addr  = 0;
    extended_flash_rw_param_ptr->end_addr    = FLASH_SIZE_160K_USED;
    if((extended_flash_rw_param_ptr->read_addr > extended_flash_rw_param_ptr->end_addr)  ||
       (extended_flash_rw_param_ptr->write_addr > extended_flash_rw_param_ptr->end_addr)
      )
    {
        extended_flash_rw_param_ptr->read_addr   = 0;
        extended_flash_rw_param_ptr->write_addr  = 0;
        extended_flash_rw_param_ptr->roll_back   = false;
    }
    #if 0
    uint8_t tmp_buf[10][16];
    extended_flash_rw_param_ptr->read_addr  = extended_flash_rw_param_ptr->end_addr-0x60;
    extended_flash_rw_param_ptr->write_addr = extended_flash_rw_param_ptr->end_addr-0x60;
    extended_flash_rw_param_ptr->roll_back   = false;
    internal_flash_param.mode = DEVICE_REGISTERED_NORMAL_STATUS;

    SpiFlash_Read_Data(tmp_buf[0], 0x00, 16);
    SpiFlash_Read_Data(tmp_buf[1], 0x10, 16);
    SpiFlash_Read_Data(tmp_buf[2], 0x20, 16);
    SpiFlash_Read_Data(tmp_buf[3], 0x30, 16);
    SpiFlash_Read_Data(tmp_buf[4], 0x40, 16);
    SpiFlash_Read_Data(tmp_buf[5], 0x50, 16);
    SpiFlash_Read_Data(tmp_buf[6], 0x60, 16);
    SpiFlash_Read_Data(tmp_buf[7], 0x70, 16);
    SpiFlash_Read_Data(tmp_buf[8], 0x80, 16);
    SpiFlash_Read_Data(tmp_buf[9], 0x90, 16);
    #endif
    spiFlash_Restore_Erase_sector(extended_flash_rw_param_ptr->write_addr);
}


/**@brief Function for sending and receiving data.
 *
 * @param[in]   p_tx_data    A pointer to a buffer TX.
 * @param[out]  p_rx_data    A pointer to a buffer RX.
 * @param[in]   len          A length of the data buffers.
 */
void spi_send_recv(uint8_t * p_tx_data,
                          uint8_t * p_rx_data,
                          uint16_t  len)
{
    nrf_drv_spi_t const * p_instance;

    m_transfer_completed = false;
#if (SPI1_ENABLED == 1)
    p_instance = &m_spi_master_1;
    uint32_t err_code = nrf_drv_spi_transfer(p_instance,
        p_tx_data, len, p_rx_data, len);
    APP_ERROR_CHECK(err_code);
#endif // (SPI1_ENABLED == 1)
}

/**@brief Function for pstorage module callback.
 *
 * @param[in] p_handle Identifies module and block for which callback is received.
 * @param[in] op_code  Identifies the operation for which the event is notified.
 * @param[in] result   Identifies the result of flash access operation.
 *                     NRF_SUCCESS implies, operation succeeded.
 * @param[in] p_data   Identifies the application data pointer. In case of store operation, this
 *                     points to the resident source of application memory that application can now
 *                     free or reuse. In case of clear, this is NULL as no application pointer is
 *                     needed for this operation.
 * @param[in] data_len Length of data provided by the application for the operation.
 */

static void spi_flash_pstorage_cb_handler(pstorage_handle_t  * p_handle,
                               uint8_t              op_code,
                               uint32_t             result,
                               uint8_t            * p_data,
                               uint32_t             data_len)
{
    if(result == NRF_SUCCESS)
    {
        Flag.is_flash_operation_finish = true;
    }
    switch(op_code)
    {
        case PSTORAGE_LOAD_OP_CODE:
             if (result == NRF_SUCCESS)
             {
                
             }
             else
             {
                
             }
             break;
        case PSTORAGE_STORE_OP_CODE:
             if (result == NRF_SUCCESS)
             {
                
             }
             else
             {
                
             }
             break;
        case PSTORAGE_UPDATE_OP_CODE:
             if (result == NRF_SUCCESS)
             {
                
             }
             else
             {
                
             }
             break;
        case PSTORAGE_CLEAR_OP_CODE:
             if (result == NRF_SUCCESS)
             {
                
             }
             else
             {
                
             }
             break;
    }

}
/**@brief Function for storing extended flash params to flash.
 */
void spi_flash_param_pstorage_store(internal_flash_param_t *p_param, spi_flash_store_t type)
{
    pstorage_handle_t  block_0_handle;
    uint32_t           err_code;
    int32_t            timeout = TIMEOUT_PSTORAGE_STORE/5; // 2 seconds timeout.
    storage_operation  store_fn;

    if(p_param == NULL)
    {
        return;
    }
    //Get block identifiers
    err_code = pstorage_block_identifier_get(&g_pstorage_handle, 0, &block_0_handle);
    APP_ERROR_CHECK(err_code);
    if(type == FIRST_SPI_FLASH_RW_STORE)
    {
        store_fn = pstorage_store;
    }
    else
    {
        store_fn = pstorage_update;
    }
    Flag.is_flash_operation_finish = false;
    err_code = store_fn(&block_0_handle, (uint8_t *)p_param, INTERNAL_FLASH_PARAM_LEN, 0);     //Write to flash, only one block is allowed for each pstorage_store command
    APP_ERROR_CHECK(err_code);
    while((!Flag.is_flash_operation_finish) && (timeout > 0))
    {
       nrf_delay_ms(5);
       timeout--;
    } //Sleep until store operation is finished.

    if(timeout <= 0)
    {
        APP_ERROR_CHECK(NRF_ERROR_TIMEOUT);
        DBG_PRINT("Time out %d ms to wait for pstorage store \r\n",((TIMEOUT_PSTORAGE_STORE/5-timeout)*5));
    }
}
/**@brief Function for loading extended flash params from flash.
 */
void spi_flash_param_pstorage_load(internal_flash_param_t *p_param)
{
    uint32_t           err_code;
    pstorage_handle_t  block_0_handle;
    if(p_param == NULL)
    {
        return;
    }
    //Get block identifiers
    err_code = pstorage_block_identifier_get(&g_pstorage_handle, 0, &block_0_handle);
    APP_ERROR_CHECK(err_code);
    err_code = pstorage_load((uint8_t *)p_param, &block_0_handle, INTERNAL_FLASH_PARAM_LEN, 0);              //Read from flash, only one block is allowed for each pstorage_load comman
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for clearing extended flash params from flash.
 */
void spi_flash_param_pstorage_clear(void)
{
    uint32_t           err_code;
    pstorage_handle_t  block_0_handle;
    int32_t            timeout = TIMEOUT_PSTORAGE_STORE/5; // 2 seconds timeout.
    err_code = pstorage_block_identifier_get(&g_pstorage_handle, 0, &block_0_handle); 
    APP_ERROR_CHECK(err_code);
    Flag.is_flash_operation_finish = false;
    err_code = pstorage_clear(&block_0_handle,INTERNAL_FLASH_PARAM_LEN);
    APP_ERROR_CHECK(err_code);
    while((!Flag.is_flash_operation_finish) && (timeout > 0))
    {
       nrf_delay_ms(5);
       timeout--;
    } //Sleep until store operation is finished.
    if(timeout <= 0)
    {
        APP_ERROR_CHECK(NRF_ERROR_TIMEOUT);
        DBG_PRINT("Time out %d ms to wait for pstorage store \r\n",((TIMEOUT_PSTORAGE_STORE/5-timeout)*5));
    }
}

