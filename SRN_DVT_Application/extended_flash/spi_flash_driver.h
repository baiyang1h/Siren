/**************************************************************************//**
 * @file     SPI_Flash.h
 * @version  V1.00
 * $Revision: 4 $
 * $Date:  $
 * @brief    
 *
 * @note
 * Copyright (C) 
*****************************************************************************/
#ifndef __SPI_FLASH_H__
#define __SPI_FLASH_H__

//#include "bcomdef.h"
#include <stdbool.h>
#include <stdint.h>
#include "MX25_DEF.h"
#define Page_Size_256          0x00100
#define Sector_Size_4k         0x01000
#define Sector_Size_4k_Mask    0xFFFFF000
#define Block_Size_32k         0x08000
#define Block_Size_64k         0x10000
#define FLASH_SIZE_160K_USED   0x28000      //160k,40 sector
#define FLASH_SIZE_W25Q80      0x100000     /*1MB for W25Q80 SPI flash*/
/*** MX25 series command hex code definition ***/
//ID comands
#define    FLASH_CMD_RDID      0x9F    //RDID (Read Identification)
#define    FLASH_CMD_RES       0xAB    //RES (Read Electronic ID)
#define    FLASH_CMD_REMS      0x90    //REMS (Read Electronic & Device ID)

//Register comands
#define    FLASH_CMD_WRSR      0x01    //WRSR (Write Status Register)
#define    FLASH_CMD_RDSR      0x05    //RDSR (Read Status Register)
#define    FLASH_CMD_WRSCUR    0x2F    //WRSCUR (Write Security Register)
#define    FLASH_CMD_RDSCUR    0x2B    //RDSCUR (Read Security Register)
#define    FLASH_CMD_RDCR      0x15    //RDCR (Read Configuration Register)
#define    FLASH_CMD_RDFSR     0x44    //RDFSR (Read Factory Status Register)

//READ comands
#define    FLASH_CMD_READ        0x03    //READ (1 x I/O)
#define    FLASH_CMD_2READ       0xBB    //2READ (2 x I/O)
#define    FLASH_CMD_4READ       0xEB    //4READ (4 x I/O)
#define    FLASH_CMD_FASTREAD    0x0B    //FAST READ (Fast read data)
#define    FLASH_CMD_DREAD       0x3B    //DREAD (1In/2 Out fast read)
#define    FLASH_CMD_QREAD       0x6B    //QREAD (1In/4 Out fast read)
#define    FLASH_CMD_RDSFDP      0x5A    //RDSFDP (Read SFDP)

//Program comands
#define    FLASH_CMD_WREN     0x06    //WREN (Write Enable)
#define    FLASH_CMD_WRDI     0x04    //WRDI (Write Disable)
#define    FLASH_CMD_PP       0x02    //PP (page program)
#define    FLASH_CMD_4PP      0x38    //4PP (Quad page program)
#define    FLASH_CMD_QPP      0x32    //QPP (1/4 page program)

//Erase comands
#define    FLASH_CMD_SE       0x20    //SE (Sector Erase)
#define    FLASH_CMD_BE32K    0x52    //BE32K (Block Erase 32kb)
#define    FLASH_CMD_BE       0xD8    //BE (Block Erase)
#define    FLASH_CMD_CE       0x60    //CE (Chip Erase) hex code: 60 or C7

//Mode setting comands
#define    FLASH_CMD_DP       0xB9    //DP (Deep Power Down)
#define    FLASH_CMD_ENSO     0xB1    //ENSO (Enter Secured OTP)
#define    FLASH_CMD_EXSO     0xC1    //EXSO  (Exit Secured OTP)
#ifdef SBL_CMD_0x77
#define    FLASH_CMD_SBL      0x77    //SBL (Set Burst Length) new: 0x77
#else
#define    FLASH_CMD_SBL      0xC0    //SBL (Set Burst Length) Old: 0xC0
#endif
#define    FLASH_CMD_FMEN     0x41    //FMEN( Enter Factory Mode )

//Reset comands
#define    FLASH_CMD_RSTEN     0x66    //RSTEN (Reset Enable)
#define    FLASH_CMD_RST       0x99    //RST (Reset Memory)

//Security comands
#ifdef LCR_CMD_0xDD_0xD5
#else
#endif

//Suspend/Resume comands
#define    FLASH_CMD_PGM_ERS_S    0xB0    //PGM/ERS Suspend (Suspends Program/Erase)
#define    FLASH_CMD_PGM_ERS_R    0x30    //PGM/ERS Erase (Resumes Program/Erase)
#define    FLASH_CMD_NOP          0x00    //NOP (No Operation)

// Return Message
typedef enum
{
    FlashOperationSuccess,
    FlashWriteRegFailed,
    FlashTimeOut,
    FlashIsBusy,
    FlashQuadNotEnable,
    FlashAddressInvalid,
    FlashDataLengthOverrange
}ReturnMsg;
// Flash status structure define
struct sFlashStatus
{
    /* Mode Register:
     * Bit  Description
     * -------------------------
     *  7   RYBY enable
     *  6   Reserved
     *  5   Reserved
     *  4   Reserved
     *  3   Reserved
     *  2   Reserved
     *  1   Parallel mode enable
     *  0   QPI mode enable
    */
    uint8_t    ModeReg;
    uint8_t    ArrangeOpt;
};
typedef struct sFlashStatus FlashStatus;

typedef enum {
    Success,
    InvalidParam,
    WriteRegFailed,
    TimeOut,
    DevBusy,
    CmdSpiOnly,
    CmdQpiOnly,
    QuadNotEnable,
    Cmd3ByteOnly
}SFC_STATUS;

typedef enum
{
    FLASH_NA = 0,
    MX25R3235 = 1,
    MX25U6435 = 2,
    W25Q80    = 3
}Flash_Part_No;


/************************************************************************
Driver level api
************************************************************************/
#if 1
/* Basic functions */
void InsertDummyCycle(uint8_t dummy_cycle);
void SendByte(uint8_t byte_value);
uint8_t GetByte(void);

/* Utility functions */
void Wait_Flash_WarmUp(void);
bool WaitFlashReady(uint32_t  ExpectTime);
bool WaitRYBYReady(uint32_t ExpectTime);
bool IsFlashBusy(void);
bool IsFlashQIO(void);
bool IsFlash4Byte(void);
void SendFlashAddr(uint32_t flash_address ,bool addr_4byte_mode);
uint8_t GetDummyCycle( uint32_t default_cycle);
/*
 * ID Command
 */
ReturnMsg SpiFlash_Read_id(uint32_t *Identification);
ReturnMsg SpiFlash_Read_ES(uint8_t *ElectricIdentification);
ReturnMsg SpiFlash_Read_EMS(uint16_t *REMS_Identification, FlashStatus *fsptr);
/*
 * Simple flash ID test
 */
bool FlashID_Test(void);
/*
 * Register  Command
 */
ReturnMsg CMD_RDSR(uint8_t *StatusReg);
#ifdef SUPPORT_WRSR_CR
ReturnMsg CMD_WRSR(uint16 UpdateValue);
#else
ReturnMsg CMD_WRSR(uint8_t UpdateValue);
#endif
ReturnMsg CMD_RDSCUR(uint8_t *SecurityReg);
ReturnMsg CMD_RDCR(uint8_t *ConfigReg);
ReturnMsg CMD_RDFSR(uint8_t *FactoryStatusReg);
ReturnMsg CMD_READ(uint32_t flash_address, uint8_t *target_address, uint32_t byte_length);
/*
 * Program Command
 */
ReturnMsg CMD_WREN(void);
ReturnMsg CMD_WRDI(void);
ReturnMsg CMD_PP(uint32_t flash_address, uint8_t *source_address, uint32_t byte_length);
/*
 * Erase Command
 */
ReturnMsg CMD_SE(uint32_t flash_address);
ReturnMsg CMD_BE32K(uint32_t flash_address);
ReturnMsg CMD_BE(uint32_t flash_address);
ReturnMsg CMD_CE(void);
/*
 * Mode setting Command
 */
ReturnMsg CMD_DP(void);
ReturnMsg CMD_ENSO(void);
ReturnMsg CMD_EXSO(void);
ReturnMsg CMD_SBL(uint8_t burstconfig);
ReturnMsg CMD_FMEN(void);
/*
 * Reset setting Command
 */
ReturnMsg CMD_RSTEN(void);
ReturnMsg CMD_RST(FlashStatus *fsptr);
/*
 * Suspend/Resume Command
 */
ReturnMsg CMD_PGM_ERS_S(void);
ReturnMsg CMD_PGM_ERS_R(void);
ReturnMsg CMD_NOP(void);
#endif //end driver level api
/************************************************************************
application level api
************************************************************************/
#if 1
ReturnMsg SpiFlash_ChipErase(void);
ReturnMsg SpiFlash_Sector_Erase(uint32_t Addr);
ReturnMsg SpiFlash_32kBlk_Erase(uint32_t Addr);
ReturnMsg SpiFlash_64kBlk_Erase(uint32_t Addr);
extern SFC_STATUS SpiFlash_init(void);
ReturnMsg SpiFlash_Program_Onerecord(uint8_t *Buf, uint32_t Addr, uint32_t Count);
ReturnMsg SpiFlash_Erase_Program_Onerecord(uint8_t *Buf, uint32_t Addr, uint32_t Count);
ReturnMsg SpiFlash_Read_Data(uint8_t *Buf, uint32_t Addr, uint32_t Count);
void SpiFlash_Deep_Sleep_Enter(void);
void SpiFlash_Deep_Sleep_Exit(void);
void spiFlash_Restore_Erase_sector(uint32_t Addr);
#endif //end application level api
#endif
