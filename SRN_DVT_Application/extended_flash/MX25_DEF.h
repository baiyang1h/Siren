/*
 * COPYRIGHT (c) 2010-2017 MACRONIX INTERNATIONAL CO., LTD
 * SPI Flash Low Level Driver (LLD) Sample Code
 *
 * Flash device information define
 *
 * $Id: MX25_DEF.h,v 1.610 2017/09/20 09:37:08 mxclldb1 Exp $
 */

#ifndef    __MX25_DEF_H__
#define    __MX25_DEF_H__
/*
  Compiler Option
*/

/* Select your flash device type */
#define MX25R8035F

/*
  Flash Related Parameter Define
*/

#define    Block_Offset       0x10000     // 64K Block size
#define    Block32K_Offset    0x8000      // 32K Block size
#define    Sector_Offset      0x1000      // 4K Sector size
#define    Page_Offset        0x0100      // 256 Byte Page size
#define    Page32_Offset      0x0020      // 32 Byte Page size (some products have smaller page size)
#define    Block_Num          (FlashSize / Block_Offset)

// Flash control register mask define
// status register
#define    FLASH_WIP_MASK         0x01
#define    FLASH_LDSO_MASK        0x02
#define    FLASH_QE_MASK          0x40
// security register
#define    FLASH_OTPLOCK_MASK     0x03
#define    FLASH_4BYTE_MASK       0x04
#define    FLASH_WPSEL_MASK       0x80
// configuration reigster
#define    FLASH_DC_MASK          0x80
#define    FLASH_DC_2BIT_MASK     0xC0
#define    FLASH_DC_3BIT_MASK     0x07
// other
#define    BLOCK_PROTECT_MASK     0xff
#define    BLOCK_LOCK_MASK        0x01



/* Timer Parameter */
//#define  TIMEOUT    1
//#define  TIMENOTOUT 0
//#define  TPERIOD    240             // ns, The time period of timer count one
//#define  COUNT_ONE_MICROSECOND  16  // us, The loop count value within one micro-second


/*
  System Information Define
*/
#define    CLK_PERIOD             20     // unit: ns
#define    Min_Cycle_Per_Inst     1      // use 12T 8051@@@@需要修改单指令周期8051
#define    One_Loop_Inst          8      // instruction count of one loop (estimate)@@@@需要修改

/*
  Flash ID, Timing Information Define
  (The following information could get from device specification)
*/

#ifdef MX25R8035F
#define    FlashID          0xc22814
#define    ElectronicID     0x14
#define    RESID0           0xc214
#define    RESID1           0x14c2
#define    FlashSize        0x100000       // 1 MB
#define    CE_period        15625000       // tCE /  ( CLK_PERIOD * Min_Cycle_Per_Inst *One_Loop_Inst)
#define    tW               30000000       // 30ms
#define    tDP              10000          // 10us
#define    tBP              100000         // 100us
#define    tPP              10000000       // 10ms
#define    tSE              240000000      // 240ms
#define    tBE32            1750000000     // 1.75s
#define    tBE              3500000000     // 3.5s
#define    tPUW             10000000       // 10ms
#define    tWSR             tBP
// Support I/O mode
#define    SIO              0
#define    DIO              1
#define    QIO              2
#endif

// Flash information define
#define    WriteStatusRegCycleTime     tW / (CLK_PERIOD * Min_Cycle_Per_Inst * One_Loop_Inst)
#define    PageProgramCycleTime        tPP / (CLK_PERIOD * Min_Cycle_Per_Inst * One_Loop_Inst)
#define    SectorEraseCycleTime        tSE / (CLK_PERIOD * Min_Cycle_Per_Inst * One_Loop_Inst)
#define    BlockEraseCycleTime         tBE / (CLK_PERIOD * Min_Cycle_Per_Inst * One_Loop_Inst)
#define    ChipEraseCycleTime          CE_period
#define    FlashFullAccessTime         tPUW / (CLK_PERIOD * Min_Cycle_Per_Inst * One_Loop_Inst)

#ifdef tBP
#define    ByteProgramCycleTime        tBP / (CLK_PERIOD * Min_Cycle_Per_Inst * One_Loop_Inst)
#endif
#ifdef tWSR
#define    WriteSecuRegCycleTime       tWSR / (CLK_PERIOD * Min_Cycle_Per_Inst * One_Loop_Inst)
#endif
#ifdef tBE32
#define    BlockErase32KCycleTime      tBE32 / (CLK_PERIOD * Min_Cycle_Per_Inst * One_Loop_Inst)
#endif
#ifdef tWREAW
#define    WriteExtRegCycleTime        tWREAW / (CLK_PERIOD * Min_Cycle_Per_Inst * One_Loop_Inst)
#endif
#endif    /* end of __MX25_DEF_H__  */

