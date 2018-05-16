/**************************************************************************//**
 * @file     SPI_Flash.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date:  $
 * @brief
 *
 * @note
 * Copyright (C)
*****************************************************************************/
#include "nrf.h"
#include "spi_flash_driver.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "ble_lbs.h"
#include "spi_master_app.h"

extern void spi_cs_low (void);
extern void spi_cs_high (void);
extern void spi_send_recv(uint8_t * p_tx_data,uint8_t * p_rx_data,uint16_t  len);
/*---------------------------------------------------------------------------------------------------------*/
/*  Definitons                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define CS_Low()  spi_cs_low()
#define CS_High() spi_cs_high()
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define Error_inc(x)  x = x + 1;


static bool                     gDPMode = false;
static uint32_t                 next_erase_sector_index = 0;
extern extended_flash_param_t   *extended_flash_rw_param_ptr;
/************************************************************************
Driver level api
************************************************************************/
#if 1
/*
 --Common functions
 */

/*
 * Function:       Wait_Flash_WarmUp
 * Arguments:      None.
 * Description:    Wait some time until flash read / write enable.
 * Return Message: None.
 */
void Wait_Flash_WarmUp(void)
{
    uint32_t time_cnt = FlashFullAccessTime;
    while( time_cnt > 0 )
    {
        time_cnt--;
    }
}
/*
 * Function:       SendByte
 * Arguments:      byte_value, data transfer to flash
 *                 transfer_type, select different type of I/O mode.
 *                 mode:
 *                 SIO, single IO
 * Description:    Send one byte data to flash
 * Return Message: None.
 */
void SendByte(uint8_t byte_value)
{
    uint8_t data_buf = 0;
    uint8_t Byte     = byte_value;
    spi_send_recv(&Byte,&data_buf,1);
}
/*
 * Function:       GetByte
 * Arguments:      byte_value, data receive from flash
 *                 transfer_type, select different type of I/O mode.
 *                 mode:
 *                 SIO, single IO
 * Description:    Get one byte data to flash
 * Return Message: 8 bit data
 */
uint8_t GetByte(void)
{
    uint8_t data_buf  = 0;
    uint8_t dummyByte = 0xcc;
    spi_send_recv(&dummyByte,&data_buf,1);
    return data_buf;
}
/*
 * Function:       InsertDummyCycle
 * Arguments:      dummy_cycle, number of dummy clock cycle
 * Description:    Insert dummy cycle of SCLK
 * Return Message: None.
 */
void InsertDummyCycle(uint8_t dummy_cycle)
{
    uint8_t i;
    for(i=0; i < dummy_cycle; i=i+1)
    {
        CS_Low();
        CS_High();
    }
}

/*
 * Function:       WaitFlashReady
 * Arguments:      ExpectTime, expected time-out value of flash operations.
 *                 No use at non-synchronous IO mode.
 * Description:    Synchronous IO:
 *                 If flash is ready return true.
 *                 If flash is time-out return false.
 *                 Non-synchronous IO:
 *                 Always return true
 * Return Message: true, false
 */
bool WaitFlashReady(uint32_t  ExpectTime)
{
    uint32_t temp = 0;
    while(IsFlashBusy())
    {
        if(temp > ExpectTime)
        {
            return false;
        }
        temp = temp + 1;
    }
    return true;
}

/*
 * Function:       WaitRYBYReady
 * Arguments:      ExpectTime, expected time-out value of flash operations.
 *                 No use at non-synchronous IO mode.
 * Description:    Synchronous IO:
 *                 If flash is ready return true.
 *                 If flash is time-out return false.
 *                 Non-synchronous IO:
 *                 Always return true
 * Return Message: true, false
 */
bool WaitRYBYReady(uint32_t ExpectTime)
{
    uint32_t temp = 0;
    // Insert your code for waiting RYBY (SO) pin ready
    while(nrf_gpio_pin_read(SPIM1_MISO_PIN) == 0)
    {
        if(temp > ExpectTime)
        {
            return false;
        }
        temp = temp + 1;
    }
    return true;
}

/*
 * Function:       IsFlashBusy
 * Arguments:      None.
 * Description:    Check status register WIP bit.
 *                 If  WIP bit = 1: return true ( Busy )
 *                             = 0: return false ( Ready ).
 * Return Message: true, false
 */
bool IsFlashBusy(void)
{
    uint8_t  gDataBuffer;

    CMD_RDSR(&gDataBuffer);
    if((gDataBuffer & FLASH_WIP_MASK)  == FLASH_WIP_MASK)
        return true;
    else
        return false;
}

/*
 * Function:       IsFlashQIO
 * Arguments:      None.
 * Description:    If flash QE bit = 1: return true
 *                                 = 0: return false.
 * Return Message: true, false
 */
bool IsFlashQIO(void)
{
    uint8_t  gDataBuffer;
#ifdef FLASH_NO_QE_BIT
    return true;
#else
    CMD_RDSR(&gDataBuffer);
    if((gDataBuffer & FLASH_QE_MASK) == FLASH_QE_MASK)
        return true;
    else
        return false;
#endif
}
/*
 * Function:       IsFlash4Byte
 * Arguments:      None
 * Description:    Check flash address is 3-byte or 4-byte.
 *                 If flash 4BYTE bit = 1: return true
 *                                    = 0: return false.
 * Return Message: true, false
 */
bool IsFlash4Byte(void)
{
#ifdef FLASH_CMD_RDSCUR
    #ifdef FLASH_4BYTE_ONLY
        return true;
    #elif FLASH_3BYTE_ONLY
        return false;
    #else
        uint8_t  gDataBuffer;
        CMD_RDSCUR(&gDataBuffer);
        if((gDataBuffer & FLASH_4BYTE_MASK) == FLASH_4BYTE_MASK)
            return true;
        else
            return false;
    #endif
#else
    return false;
#endif
}

/*
 * Function:       SendFlashAddr
 * Arguments:      flash_address, 32 bit flash memory address
 *                 io_mode, I/O mode to transfer address
 *                 addr_4byte_mode,
 * Description:    Send flash address with 3-byte or 4-byte mode.
 * Return Message: None
 */
void SendFlashAddr(uint32_t flash_address ,bool addr_4byte_mode)
{
    /* Check flash is 3-byte or 4-byte mode.
       4-byte mode: Send 4-byte address (A31-A0)
       3-byte mode: Send 3-byte address (A23-A0) */
    if(addr_4byte_mode == true)
    {
        SendByte( flash_address >> 24 ); // A31-A24
    }
    /* A23-A0 */
    SendByte(flash_address >> 16);
    SendByte(flash_address >> 8);
    SendByte(flash_address );
}
/*
 * Function:       GetDummyCycle
 * Arguments:      default_cycle, default dummy cycle
 *                 fsptr, pointer of flash status structure
 * Description:    Get dummy cycle for different condition
 *                 default_cycle: Byte3 | Byte2 | Byte1 | Byte0
 *                      DC 1 bit:   x       1       x       0
 *                      DC 2 bit:   11      10      01      00
 *                 Note: the variable dummy cycle only support
                         in some product.
 * Return Message: Dummy cycle value
 */
uint8_t GetDummyCycle(uint32_t default_cycle)
{
#ifdef FLASH_CMD_RDCR
    uint8_t gDataBuffer;
    uint8_t dummy_cycle = default_cycle;
    CMD_RDCR(&gDataBuffer);
    #ifdef SUPPORT_CR_DC
        // product support 1-bit dummy cycle configuration
        if( (gDataBuffer & FLASH_DC_MASK) == FLASH_DC_MASK )
            dummy_cycle = default_cycle >> 16;
        else
            dummy_cycle = default_cycle;
    #elif SUPPORT_CR_DC_2bit
        // product support 2-bit dummy cycle configuration
        switch( gDataBuffer & FLASH_DC_2BIT_MASK ){
            case 0x00:
                dummy_cycle = default_cycle;
            break;
            case 0x40:
                dummy_cycle = default_cycle >> 8;
            break;
            case 0x80:
                dummy_cycle = default_cycle >> 16;
            break;
            case 0xC0:
                dummy_cycle = default_cycle >> 24;
            break;
        }
    #else
         // configuration register not support dummy configuration
         dummy_cycle = default_cycle;
    #endif
    return dummy_cycle;
#else
    // default case: return default dummy cycle
    return default_cycle;
#endif
}

/*
 * ID Command
 */

/*
 * Function:       Read Identifcation
 * Arguments:      Identification, 32 bit buffer to store id
 * Description:    The RDID instruction is to read the manufacturer ID
 *                 of 1-byte and followed by Device ID of 2-byte.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg SpiFlash_Read_id(uint32_t *Identification)
{
    uint32_t temp;
    uint8_t  gDataBuffer[3];

    // Chip select go low to start a flash command
    CS_Low();

    // Send command
    SendByte(FLASH_CMD_RDID);

    // Get manufacturer identification, device identification
    gDataBuffer[0] = GetByte();
    gDataBuffer[1] = GetByte();
    gDataBuffer[2] = GetByte();

    // Chip select go high to end a command
    CS_High();

    // Store identification
    temp =  gDataBuffer[0];
    temp =  (temp << 8 )  | gDataBuffer[1];
    *Identification =  (temp << 8)  | gDataBuffer[2];

    return FlashOperationSuccess;
}
/*
 * Function:       Read Electronic Signature
 * Arguments:      ElectricIdentification, 8 bit buffer to store electric id
 * Description:    The RES instruction is to read the Device
 *                 electric identification of 1-byte.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg SpiFlash_Read_ES(uint8_t *ElectricIdentification)
{
    // Chip select go low to start a flash command
    CS_Low();

    // Send flash command and insert dummy cycle
    SendByte(FLASH_CMD_RES);
    InsertDummyCycle(24);

    // Get electric identification
    *ElectricIdentification = GetByte();

    // Chip select go high to end a flash command
    CS_High();

    return FlashOperationSuccess;
}
/*
 * Function:       Read Electronic Manufacturer ID & Device ID
 * Arguments:      REMS_Identification, 16 bit buffer to store id
 *                 fsptr, pointer of flash status structure
 * Description:    The REMS instruction is to read the Device
 *                 manufacturer ID and electric ID of 1-byte.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg SpiFlash_Read_EMS(uint16_t *REMS_Identification, FlashStatus *fsptr)
{
    uint8_t  gDataBuffer[2];

    // Chip select go low to start a flash command
    CS_Low();

    // Send flash command and insert dummy cycle ( if need )
    // ArrangeOpt = 0x00 will output the manufacturer's ID first
    //            = 0x01 will output electric ID first
    SendByte(FLASH_CMD_REMS);
    InsertDummyCycle(16);
    SendByte(fsptr->ArrangeOpt);

    // Get ID
    gDataBuffer[0] = GetByte();
    gDataBuffer[1] = GetByte();

    // Store identification informaion
    *REMS_Identification = (gDataBuffer[0] << 8) | gDataBuffer[1];

    // Chip select go high to end a flash command
    CS_High();

    return FlashOperationSuccess;
}
/*
 * Simple flash ID test
 */
bool FlashID_Test(void)
{
    uint32_t  flash_id = 0;
    uint16_t  error_cnt = 0;
    uint16_t  rems_id;
    uint8_t   electric_id = 0;
    FlashStatus  flash_state = {0};
    ReturnMsg  msg;

    /* Read flash device id */
    msg =  SpiFlash_Read_id(&flash_id);
    if(msg != (ReturnMsg)FlashOperationSuccess)
        return false;

    msg = SpiFlash_Read_ES(&electric_id);
    if(msg != (ReturnMsg)FlashOperationSuccess)
        return false;

    /* Decide rems_id order. 0: { manufacturer id, device id }
                             1: { device id,  manufacturer id } */
    flash_state.ArrangeOpt = 0;

    msg = SpiFlash_Read_EMS(&rems_id, &flash_state);
    if(msg != (ReturnMsg)FlashOperationSuccess)
        return false;

    /* Compare to expected value */
    if( flash_id != FlashID)
        Error_inc(error_cnt);

    if(electric_id != ElectronicID)
        Error_inc(error_cnt);

    if(flash_state.ArrangeOpt)
    {
        if(rems_id != RESID1)
           Error_inc(error_cnt);
    }
    else
    {
        if(rems_id != RESID0)
           Error_inc(error_cnt);
    }

    if(error_cnt != 0)
        return false;
    else
        return true;
}

/*
 * Register  Command
 */

/*
 * Function:       CMD_RDSR
 * Arguments:      StatusReg, 8 bit buffer to store status register value
 *                 fsptr, pointer of flash status structure
 * Description:    The RDSR instruction is for reading Status
 *                 Register Bits.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg CMD_RDSR(uint8_t *StatusReg)
{
    uint8_t  gDataBuffer;

    // Chip select go low to start a flash command
    CS_Low();

    // Send command
    SendByte(FLASH_CMD_RDSR);
    gDataBuffer = GetByte();

    // Chip select go high to end a flash command
    CS_High();

    *StatusReg = gDataBuffer;

    return FlashOperationSuccess;
}

/*
 * Function:       CMD_WRSR
 * Arguments:      UpdateValue, 8/16 bit status register value to updata
 * Description:    The WRSR instruction is for changing the values of
 *                 Status Register Bits (and configuration register)
 * Return Message: FlashIsBusy, FlashTimeOut, FlashOperationSuccess
 */
#ifdef SUPPORT_WRSR_CR
ReturnMsg CMD_WRSR(uint16 UpdateValue)
#else
ReturnMsg CMD_WRSR(uint8_t UpdateValue)
#endif
{
    // Check flash is busy or not
    if(IsFlashBusy())
        return FlashIsBusy;

    // Setting Write Enable Latch bit
    CMD_WREN();

    // Chip select go low to start a flash command
    CS_Low();

    // Send command and update value
    SendByte(FLASH_CMD_WRSR);
    SendByte(UpdateValue);
#ifdef SUPPORT_WRSR_CR
    SendByte( UpdateValue >> 8);    // write configuration register
#endif

    // Chip select go high to end a flash command
    CS_High();

    if(WaitFlashReady( WriteStatusRegCycleTime ))
        return FlashOperationSuccess;
    else
        return FlashTimeOut;
}

/*
 * Function:       CMD_RDSCUR
 * Arguments:      SecurityReg, 8 bit buffer to store security register value
 * Description:    The RDSCUR instruction is for reading the value of
 *                 Security Register bits.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg CMD_RDSCUR(uint8_t *SecurityReg)
{
    uint8_t  gDataBuffer;

    // Chip select go low to start a flash command
    CS_Low();

    //Send command
    SendByte(FLASH_CMD_RDSCUR);
    gDataBuffer = GetByte();

    // Chip select go high to end a flash command
    CS_High();

    *SecurityReg = gDataBuffer;

    return FlashOperationSuccess;
}

/*
 * Function:       CMD_RDCR
 * Arguments:      ConfigReg, 8 bit buffer to store Configuration register value
 * Description:    The RDCR instruction is for reading Configuration Register Bits.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg CMD_RDCR(uint8_t *ConfigReg)
{
    uint8_t  gDataBuffer;

    // Chip select go low to start a flash command
    CS_Low();

    // Send command
    SendByte(FLASH_CMD_RDCR);
    gDataBuffer = GetByte();

    // Chip select go high to end a flash command
    CS_High();

    *ConfigReg = gDataBuffer;

    return FlashOperationSuccess;
}
/*
 * Function:       CMD_RDFSR
 * Arguments:      FactoryStatusReg, 8 bit buffer to store factory status register value
 * Description:    The RDFSR instruction is for reading Factory Status Register Bits.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg CMD_RDFSR(uint8_t *FactoryStatusReg)
{
    uint8_t  gDataBuffer;

    // Chip select go low to start a flash command
    CS_Low();

    // Send command
    SendByte( FLASH_CMD_RDFSR);
    gDataBuffer = GetByte();

    // Chip select go high to end a flash command
    CS_High();

    *FactoryStatusReg = gDataBuffer;

    return FlashOperationSuccess;
}
/*
 * Read Command
 */

/*
 * Function:       CMD_READ
 * Arguments:      flash_address, 32 bit flash memory address
 *                 target_address, buffer address to store returned data
 *                 byte_length, length of returned data in byte unit
 * Description:    The READ instruction is for reading data out.
 * Return Message: FlashAddressInvalid, FlashOperationSuccess
 */
ReturnMsg CMD_READ(uint32_t flash_address, uint8_t *target_address, uint32_t byte_length)
{
    uint32_t index;
    uint8_t  addr_4byte_mode;

    // Check flash address
    if( flash_address > FlashSize ) return FlashAddressInvalid;

    // Check 3-byte or 4-byte mode
    if(IsFlash4Byte())
        addr_4byte_mode = true;  // 4-byte mode
    else
        addr_4byte_mode = false; // 3-byte mode

    // Chip select go low to start a flash command
    CS_Low();

    // Write READ command and address
    SendByte(FLASH_CMD_READ);
    SendFlashAddr(flash_address, addr_4byte_mode);

    // Set a loop to read data into buffer
    for(index=0; index < byte_length; index++)
    {
        // Read data one byte at a time
        *(target_address + index) = GetByte();
    }

    // Chip select go high to end a flash command
    CS_High();

    return FlashOperationSuccess;
}

/*
 * Program Command
 */

/*
 * Function:       CMD_WREN
 * Arguments:      None
 * Description:    The WREN instruction is for setting rite Enable Latch
 *                 (WEL) bit.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg CMD_WREN(void)
{
    // Chip select go low to start a flash command
    CS_Low();

    // Write Enable command = 0x06, Setting Write Enable Latch Bit
    SendByte(FLASH_CMD_WREN);

    // Chip select go high to end a flash command
    CS_High();

    return FlashOperationSuccess;
}

/*
 * Function:       CMD_WRDI
 * Arguments:      None.
 * Description:    The WRDI instruction is to reset
 *                 Write Enable Latch (WEL) bit.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg CMD_WRDI(void)
{
    // Chip select go low to start a flash command
    CS_Low();

    // Write Disable command = 0x04, resets Write Enable Latch Bit
    SendByte(FLASH_CMD_WRDI);

    CS_High();

    return FlashOperationSuccess;
}

/*
 * Function:       CMD_PP
 * Arguments:      flash_address, 32 bit flash memory address
 *                 source_address, buffer address of source data to program
 *                 byte_length, byte length of data to programm
 * Description:    The PP instruction is for programming
 *                 the memory to be "0".
 *                 The device only accept the last 256 byte ( or 32 byte ) to program.
 *                 If the page address ( flash_address[7:0] ) reach 0xFF, it will
 *                 program next at 0x00 of the same page.
 *                 Some products have smaller page size ( 32 byte )
 * Return Message: FlashAddressInvalid, FlashIsBusy, FlashOperationSuccess,
 *                 FlashTimeOut
 */
ReturnMsg CMD_PP(uint32_t flash_address, uint8_t *source_address, uint32_t byte_length)
{
    uint32_t index;
    uint8_t  addr_4byte_mode;

    // Check flash address
    if(flash_address > FlashSize)
        return FlashAddressInvalid;

    // Check flash is busy or not
    if(IsFlashBusy())
        return FlashIsBusy;

    // Check 3-byte or 4-byte mode
    if(IsFlash4Byte())
        addr_4byte_mode = true;  // 4-byte mode
    else
        addr_4byte_mode = false; // 3-byte mode

    // Setting Write Enable Latch bit
    CMD_WREN();

    // Chip select go low to start a flash command
    CS_Low();

    // Write Page Program command
    SendByte(FLASH_CMD_PP);
    SendFlashAddr(flash_address, addr_4byte_mode);

    // Set a loop to down load whole page data into flash's buffer
    // Note: only last 256 byte ( or 32 byte ) will be programmed
    for(index=0; index < byte_length; index++)
    {
        SendByte( *(source_address + index));
    }

    // Chip select go high to end a flash command
    CS_High();

    if(WaitFlashReady(PageProgramCycleTime))
        return FlashOperationSuccess;
    else
        return FlashTimeOut;
}

/*
 * Erase Command
 */

/*
 * Function:       CMD_SE
 * Arguments:      flash_address, 32 bit flash memory address
 * Description:    The SE instruction is for erasing the data
 *                 of the chosen sector (4KB) to be "1".
 * Return Message: FlashAddressInvalid, FlashIsBusy, FlashOperationSuccess,
 *                 FlashTimeOut
 */
ReturnMsg CMD_SE(uint32_t flash_address)
{
    uint8_t  addr_4byte_mode;

    // Check flash address
    if(flash_address > FlashSize)
        return FlashAddressInvalid;

    // Check flash is busy or not
    if(IsFlashBusy())
        return FlashIsBusy;

    // Check 3-byte or 4-byte mode
    if(IsFlash4Byte())
        addr_4byte_mode = true;  // 4-byte mode
    else
        addr_4byte_mode = false; // 3-byte mode

    // Setting Write Enable Latch bit
    CMD_WREN();

    // Chip select go low to start a flash command
    CS_Low();

    //Write Sector Erase command = 0x20;
    SendByte(FLASH_CMD_SE);
    SendFlashAddr(flash_address, addr_4byte_mode);

    // Chip select go high to end a flash command
    CS_High();

    if(WaitFlashReady(SectorEraseCycleTime))
        return FlashOperationSuccess;
    else
        return FlashTimeOut;
}

/*
 * Function:       CMD_BE32K
 * Arguments:      flash_address, 32 bit flash memory address
 * Description:    The BE32K instruction is for erasing the data
 *                 of the chosen sector (32KB) to be "1".
 * Return Message: FlashAddressInvalid, FlashIsBusy, FlashOperationSuccess,
 *                 FlashTimeOut
 */
ReturnMsg CMD_BE32K(uint32_t flash_address)
{
    uint8_t  addr_4byte_mode;

    // Check flash address
    if(flash_address > FlashSize)
        return FlashAddressInvalid;

    // Check flash is busy or not
    if(IsFlashBusy())
        return FlashIsBusy;

    // Check 3-byte or 4-byte mode
    if(IsFlash4Byte())
        addr_4byte_mode = true;  // 4-byte mode
    else
        addr_4byte_mode = false; // 3-byte mode

    // Setting Write Enable Latch bit
    CMD_WREN();

    // Chip select go low to start a flash command
    CS_Low();

    //Write Block Erase32KB command;
    SendByte(FLASH_CMD_BE32K);
    SendFlashAddr(flash_address, addr_4byte_mode);

    // Chip select go high to end a flash command
    CS_High();

    if(WaitFlashReady(BlockErase32KCycleTime))
        return FlashOperationSuccess;
    else
        return FlashTimeOut;
}

/*
 * Function:       CMD_BE
 * Arguments:      flash_address, 32 bit flash memory address
 * Description:    The BE instruction is for erasing the data
 *                 of the chosen sector (64KB) to be "1".
 * Return Message: FlashAddressInvalid, FlashIsBusy, FlashOperationSuccess,
 *                 FlashTimeOut
 */
ReturnMsg CMD_BE(uint32_t flash_address)
{
    uint8_t  addr_4byte_mode;

    // Check flash address
    if(flash_address > FlashSize)
        return FlashAddressInvalid;

    // Check flash is busy or not
    if(IsFlashBusy())
        return FlashIsBusy;

    // Check 3-byte or 4-byte mode
    if(IsFlash4Byte())
        addr_4byte_mode = true;  // 4-byte mode
    else
        addr_4byte_mode = false; // 3-byte mode

    // Setting Write Enable Latch bit
    CMD_WREN();

    // Chip select go low to start a flash command
    CS_Low();

    //Write Block Erase command = 0xD8;
    SendByte(FLASH_CMD_BE);
    SendFlashAddr(flash_address, addr_4byte_mode);

    // Chip select go high to end a flash command
    CS_High();

    if(WaitFlashReady(BlockEraseCycleTime))
        return FlashOperationSuccess;
    else
        return FlashTimeOut;
}

/*
 * Function:       CMD_CE
 * Arguments:      None.
 * Description:    The CE instruction is for erasing the data
 *                 of the whole chip to be "1".
 * Return Message: FlashIsBusy, FlashOperationSuccess, FlashTimeOut
 */
ReturnMsg CMD_CE(void)
{
    // Check flash is busy or not
    if(IsFlashBusy())
        return FlashIsBusy;

    // Setting Write Enable Latch bit
    CMD_WREN();

    // Chip select go low to start a flash command
    CS_Low();

    //Write Chip Erase command = 0x60;
    SendByte(FLASH_CMD_CE);

    // Chip select go high to end a flash command
    CS_High();

    if(WaitFlashReady(ChipEraseCycleTime))
        return FlashOperationSuccess;
    else
        return FlashTimeOut;
}


/*
 * Mode setting Command
 */

/*
 * Function:       CMD_DP
 * Arguments:      None.
 * Description:    The DP instruction is for setting the
 *                 device on the minimizing the power consumption.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg CMD_DP(void)
{
    // Chip select go low to start a flash command
    CS_Low();

    // Deep Power Down Mode command
    SendByte(FLASH_CMD_DP);

    // Chip select go high to end a flash command
    CS_High();

    return FlashOperationSuccess;
}


/*
 * Function:       CMD_ENSO
 * Arguments:      None.
 * Description:    The ENSO instruction is for entering the secured OTP mode.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg CMD_ENSO(void)
{
    // Chip select go low to start a flash command
    CS_Low();

    // Write ENSO command
    SendByte( FLASH_CMD_ENSO);

    // Chip select go high to end a flash command
    CS_High();

    return FlashOperationSuccess;
}

/*
 * Function:       CMD_EXSO
 * Arguments:      None.
 * Description:    The EXSO instruction is for exiting the secured OTP mode.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg CMD_EXSO(void)
{
    // Chip select go low to start a flash command
    CS_Low();

    // Write EXSO command = 0xC1
    SendByte( FLASH_CMD_EXSO);

    // Chip select go high to end a flash command
    CS_High();

    return FlashOperationSuccess;
}

/*
 * Function:       CMD_SBL
 * Arguments:      burstconfig, burst length configuration
 * Description:    To set the Burst length
 * Return Message: FlashOperationSuccess
 */
ReturnMsg CMD_SBL(uint8_t burstconfig)
{
    // Chip select go low to start a flash command
    CS_Low();

    // Send SBL command and config data
    SendByte(FLASH_CMD_SBL);
    SendByte(burstconfig);

    // Chip select go high to end a flash command
    CS_High();

    return FlashOperationSuccess;
}

/*
 * Function:       CMD_FMEN
 * Arguments:      None
 * Description:    The FMEN instruction is for entering factory mode.
 * Return Message: FlashOperationSuccess, FlashWriteRegFailed
 */
ReturnMsg CMD_FMEN(void)
{
    // Chip select go low to start a flash command
    CS_Low();

    // Write command
    SendByte(FLASH_CMD_FMEN);

    // Chip select go high to end a flash command
    CS_High();

    if(IsFlash4Byte())
        return FlashOperationSuccess;
    else
        return FlashWriteRegFailed;
}
/*
 * Reset setting Command
 */

/*
 * Function:       CMD_RSTEN
 * Arguments:      None.
 * Description:    Enable RST command
 * Return Message: FlashOperationSuccess
 */
ReturnMsg CMD_RSTEN(void)
{
    // Chip select go low to start a flash command
    CS_Low();

    // Write RSTEN command
    SendByte(FLASH_CMD_RSTEN);

    // Chip select go high to end a flash command
    CS_High();

    return FlashOperationSuccess;
}

/*
 * Function:       CMD_RST
 * Arguments:      fsptr, pointer of flash status structure
 * Description:    The RST instruction is used as a system (software) reset that
 *                 puts the device in normal operating Ready mode.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg CMD_RST(FlashStatus *fsptr)
{
    // Chip select go low to start a flash command
    CS_Low();

    // Write RST command = 0x99
    SendByte(FLASH_CMD_RST);

    // Chip select go high to end a flash command
    CS_High();

    // Reset current state
    fsptr->ArrangeOpt = 0;
    fsptr->ModeReg = 0x00;

    return FlashOperationSuccess;
}

/*
 * Security Command
 */


/*
 * Suspend/Resume Command
 */

/*
 * Function:       CMD_PGM_ERS_S
 * Arguments:      None
 * Description:    The PGM_ERS_S suspend Sector-Erase, Block-Erase or
 *                 Page-Program operations and conduct other operations.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg CMD_PGM_ERS_S(void)
{
    // Chip select go low to start a flash command
    CS_Low();

    // Send program/erase suspend command
    SendByte(FLASH_CMD_PGM_ERS_S);

    // Chip select go high to end a flash command
    CS_High();

    return FlashOperationSuccess;
}

/*
 * Function:       CMD_PGM_ERS_R
 * Arguments:      None
 * Description:    The PGM_ERS_R resume Sector-Erase, Block-Erase or
 *                 Page-Program operations.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg CMD_PGM_ERS_R(void)
{
    // Chip select go low to start a flash command
    CS_Low();

    // Send resume command
    SendByte( FLASH_CMD_PGM_ERS_R);

    // Chip select go high to end a flash command
    CS_High();

    return FlashOperationSuccess;
}


/*
 * Function:       CMD_NOP
 * Arguments:      None.
 * Description:    The NOP instruction is null operation of flash.
 * Return Message: FlashOperationSuccess
 */
ReturnMsg CMD_NOP(void)
{
    // Chip select go low to start a flash command
    CS_Low();

    // Write NOP command = 0x00
    SendByte( FLASH_CMD_NOP);

    // Chip select go high to end a flash command
    CS_High();

    return FlashOperationSuccess;
}
#endif //end Driver level api
/************************************************************************
application level api
************************************************************************/
#if 1
/**
 * @brief       This function do the chip erasing to SPI Flash device.
 *
 * @return      None
 */
ReturnMsg SpiFlash_ChipErase(void)
{
    return CMD_CE();
}
ReturnMsg SpiFlash_Sector_Erase(uint32_t Addr)
{
    return CMD_SE(Addr);
}
ReturnMsg SpiFlash_32kBlk_Erase(uint32_t flash_address)
{
    return CMD_BE32K(flash_address);
}
ReturnMsg SpiFlash_64kBlk_Erase(uint32_t flash_address)
{
    return CMD_BE(flash_address);
}
ReturnMsg SpiFlash_Program_Onerecord(uint8_t *Buf, uint32_t Addr, uint32_t Count)
{
    if(Count > STORED_TEMP_DATA_LEN)
        return FlashDataLengthOverrange;

    return CMD_PP(Addr, Buf, Count);
}
ReturnMsg SpiFlash_Erase_Program_Onerecord(uint8_t *Buf, uint32_t Addr, uint32_t Count)
{
    uint32_t tail_sector_index = ((Addr+Count)&Sector_Size_4k_Mask)>>12;
    uint32_t erase_addr;

    /*Erase the sector at first if possible*/
    while(next_erase_sector_index <= tail_sector_index)
    {
        DBG_PRINT("SpiFlash_Erase_Program_Onerecord: 0x%x,Count:%d,next sector: %d \r\n",Addr,Count,next_erase_sector_index);
        erase_addr = next_erase_sector_index*Sector_Size_4k;
        /*If erase address is equal or bigger than flash size, then skip it and
        jump to the first address to erase in future*/

        if (erase_addr >= extended_flash_rw_param_ptr->end_addr)
        {
            next_erase_sector_index = 0;
            break;
        }
        SpiFlash_Sector_Erase(erase_addr);
        next_erase_sector_index++;
    }
    return SpiFlash_Program_Onerecord(Buf,Addr,Count);
}
ReturnMsg SpiFlash_Read_Data(uint8_t *Buf, uint32_t Addr, uint32_t Count)
{
    return CMD_READ(Addr, Buf, Count);
}
void SpiFlash_Deep_Sleep_Enter(void)
{
    if(gDPMode == true)
        return;
    CMD_DP();
    nrf_delay_us(20);//max 10us in tDP
    nrf_delay_us(30);//min 30us in tDPDD
    gDPMode = true;
}
void SpiFlash_Deep_Sleep_Exit(void)
{
    if(gDPMode == false)
        return;
    CS_Low();
    nrf_delay_us(5); //min 20ns in tCRDP
    CS_High();
    nrf_delay_us(50);//min 35us in tRDP
    gDPMode = false;
}
void spiFlash_Restore_Erase_sector(uint32_t Addr)
{
    uint32_t tail_sector_index = (Addr&Sector_Size_4k_Mask)>>12;
    if(Addr == 0)
    {
        next_erase_sector_index = 0;
    }
    else
    {
        next_erase_sector_index = tail_sector_index + 1;
    }
    DBG_PRINT("spiFlash_Restore_Erase_sector addr: 0x%x,next sector: %d \r\n",Addr,next_erase_sector_index);
}
#if 0 //for testing MX25R8035F
uint8_t tmp_buf[16];
adc_all_samples_store_to_flash();
SpiFlash_Read_Data(uint8_t *Buf, uint32_t Addr, uint32_t Count);
#endif
#endif //end application level api


