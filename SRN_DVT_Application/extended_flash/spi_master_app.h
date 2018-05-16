#ifndef SPI_MASTER_APP_H__
#define SPI_MASTER_APP_H__

#include "spi_flash_driver.h"
#include "pstorage.h"
#include "ble_lbs.h"

#define EMPTY_FLASH_VALUE                0xFF              /**< defines an empty value in flash after flash erased. */

#define INTERNAL_FLASH_PARAM_LEN         32


typedef struct
{
    uint32_t start_addr;
    uint32_t end_addr;
    uint32_t read_addr;
    uint32_t write_addr;
    bool     roll_back;
}extended_flash_param_t;

typedef struct
{
    uint8_t                            mode;          /*sync or async*/
    device_resetreason_log_t           device_resetreason_log;
    uint8_t  reserve[27];
}internal_flash_param_t;

typedef enum
{
    FIRST_SPI_FLASH_RW_STORE,
    UPDATE_SPI_FLASH_RW_STORE
}spi_flash_store_t;

/**@brief Function for performing necessary functions of storing or updating.
 *
 * @param[in] p_dest Destination address where data is stored persistently.
 * @param[in] p_src  Source address containing data to be stored.
 * @param[in] size   Size of data to be stored expressed in bytes. Must be word aligned.
 * @param[in] offset Offset in bytes to be applied when writing to the block.
 *
 * @retval Operation result code.
 */
typedef uint32_t (* storage_operation)(pstorage_handle_t * p_dest,
                                       uint8_t           * p_src,
                                       pstorage_size_t     size,
                                       pstorage_size_t     offset);


void spi_flash_param_pstorage_store(internal_flash_param_t *p_param, spi_flash_store_t type);
void spi_flash_param_pstorage_load(internal_flash_param_t *p_param);
void spi_flash_param_pstorage_clear(void);
#endif

