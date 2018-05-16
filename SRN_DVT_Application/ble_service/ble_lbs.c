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

#include "ble_lbs.h"
#include "nrf51.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "app_error.h"
#include "spi_master_app.h"

#ifdef SIREN_DEBUG
extern uint8_t siren_flash_id;
#endif

#ifdef MP6050_ENABLE
extern uint8_t accel_x_index;
extern uint16_t accel_x;

extern uint8_t accel_y_index;
extern uint16_t accel_y;

extern uint8_t accel_z_index;
extern uint16_t accel_z;

extern uint8_t gyro_x_index;
extern uint16_t gyro_x;

extern uint8_t gyro_y_index;
extern uint16_t gyro_y;

extern uint8_t gyro_z_index;
extern uint16_t gyro_z;
#endif

#ifdef VIGO_CLEAN_UP
extern uint8_t ButtonOneState_index;
extern uint16_t ButtonOneState;

extern uint8_t ButtonTwoState_index;
extern uint16_t ButtonTwoState;

extern uint8_t Batterylevel_index;
extern uint16_t Batterylevel;

extern uint8_t ambient_index;
extern uint16_t ambient;

extern uint8_t Current_user_state_index;
extern uint16_t Current_user_state;
extern uint8_t stamp;

extern uint16_t adc_value[7];
extern uint8_t adc_value_flash[12];

extern uint8_t  percentage_batt_lvl;
#endif
extern internal_flash_param_t      internal_flash_param;

uint8_t sequence_id = 0;
uint16_t patient_id = 0xFFFF;           /*Default value*/

ble_communication_t temp_req;
ble_communication_t battery_level;
ble_communication_t date_time_req;
ble_communication_t patient_req;
ble_communication_t acc_inquiry;
ble_communication_t device_status_inquiry;

motion_status_t motion_status = MOTION_INACTIVITY;
#define                  RAM_NO_INIT                  0x20003FE0
uint32_t                 *timestamp_ptr               = (uint32_t *)(RAM_NO_INIT);//__attribute__((section(".ARM.__at_0x20002800")));
extended_flash_param_t   *extended_flash_rw_param_ptr = (extended_flash_param_t *)(RAM_NO_INIT+sizeof(uint32_t));//__attribute__((section(".ARM.__at_0x2000280C")));
uint8_t                  *time_zone                   = (uint8_t *)(RAM_NO_INIT+sizeof(uint32_t)+sizeof(extended_flash_param_t));
uint16_t                 *total_step_counter          = (uint16_t *)((RAM_NO_INIT+sizeof(uint32_t)+sizeof(extended_flash_param_t)+sizeof(uint8_t)+1)/2*2);
#if 0
/*R1 = 6.98K VDD=1.8V for  NCP15XH103F03RC sensor*/
/*temperature 20 ~ 45 degree celsius, step is 0.1 degree*/
static const uint16_t adc_data_table[] =
{
    0x292,0x291,0x290,0x28f,0x28e,0x28d,0x28b,0x28a,0x289,0x288,0x287,0x286,0x285,0x284,0x283,0x282, /*20 - 21.5*/
    0x281,0x27f,0x27e,0x27d,0x27c,0x27b,0x27a,0x279,0x278,0x277,0x276,0x274,0x273,0x272,0x271,0x270, /*21.6- 23*/
    0x26f,0x26e,0x26d,0x26c,0x26b,0x269,0x268,0x267,0x266,0x265,0x264,0x263,0x262,0x261,0x25f,0x25e,
    0x25d,0x25c,0x25b,0x25a,0x259,0x258,0x257,0x256,0x254,0x253,0x252,0x251,0x250,0x24f,0x24e,0x24d,
    0x24c,0x24a,0x249,0x248,0x247,0x246,0x245,0x244,0x243,0x242,0x240,0x23f,0x23e,0x23d,0x23c,0x23b,
    0x23a,0x239,0x238,0x237,0x235,0x234,0x233,0x232,0x231,0x230,0x22f,0x22e,0x22d,0x22b,0x22a,0x229,
    0x228,0x227,0x226,0x225,0x224,0x223,0x222,0x220,0x21f,0x21e,0x21d,0x21c,0x21b,0x21a,0x219,0x218,
    0x217,0x216,0x214,0x213,0x212,0x211,0x210,0x20f,0x20e,0x20d,0x20c,0x20b,0x20a,0x208,0x207,0x206,
    0x205,0x204,0x203,0x202,0x201,0x200,0x1ff,0x1fe,0x1fc,0x1fb,0x1fa,0x1f9,0x1f8,0x1f7,0x1f6,0x1f5,
    0x1f4,0x1f3,0x1f2,0x1f1,0x1f0,0x1ef,0x1ed,0x1ec,0x1eb,0x1ea,0x1e9,0x1e8,0x1e7,0x1e6,0x1e5,0x1e4,
    0x1e3,0x1e2,0x1e1,0x1e0,0x1df,0x1de,0x1dc,0x1db,0x1da,0x1d9,0x1d8,0x1d7,0x1d6,0x1d5,0x1d4,0x1d3,
    0x1d2,0x1d1,0x1d0,0x1cf,0x1ce,0x1cd,0x1cc,0x1cb,0x1ca,0x1c9,0x1c8,0x1c7,0x1c6,0x1c5,0x1c3,0x1c2,
    0x1c1,0x1c0,0x1bf,0x1be,0x1bd,0x1bc,0x1bb,0x1ba,0x1b9,0x1b8,0x1b7,0x1b6,0x1b5,0x1b4,0x1b3,0x1b2,
    0x1b1,0x1b0,0x1af,0x1ae,0x1ad,0x1ac,0x1ab,0x1aa,0x1a9,0x1a8,0x1a7,0x1a6,0x1a5,0x1a4,0x1a3,0x1a2,
    0x1a1,0x1a0,0x19f,0x19e,0x19d,0x19c,0x19b,0x19a,0x199,0x198,0x197,0x196,0x196,0x195,0x194,0x193,
    0x192,0x191,0x190,0x18f,0x18e,0x18d,0x18c,0x18b,0x18a,0x189,0x188
};

/*R1 = 10K VDD=1.8V for trial board v1.10 NCP15XH103F03RC sensor*/
/*temperature 20 ~ 45 degree celsius, step is 0.1 degree*/
static const uint16_t adc_data_table[] =
{
    0x23a,0x239,0x238,0x236,0x235,0x234,0x233,0x232,0x231,0x22f,0x22e,0x22d,0x22c,0x22b,0x22a,0x228,
    0x227,0x226,0x225,0x224,0x223,0x221,0x220,0x21f,0x21e,0x21d,0x21c,0x21a,0x219,0x218,0x217,0x216,
    0x215,0x214,0x212,0x211,0x210,0x20f,0x20e,0x20d,0x20b,0x20a,0x209,0x208,0x207,0x206,0x205,0x203,
    0x202,0x201,0x200,0x1ff,0x1fe,0x1fd,0x1fb,0x1fa,0x1f9,0x1f8,0x1f7,0x1f6,0x1f5,0x1f3,0x1f2,0x1f1,
    0x1f0,0x1ef,0x1ee,0x1ed,0x1ec,0x1ea,0x1e9,0x1e8,0x1e7,0x1e6,0x1e5,0x1e4,0x1e3,0x1e1,0x1e0,0x1df,
    0x1de,0x1dd,0x1dc,0x1db,0x1da,0x1d9,0x1d7,0x1d6,0x1d5,0x1d4,0x1d3,0x1d2,0x1d1,0x1d0,0x1cf,0x1cd,
    0x1cc,0x1cb,0x1ca,0x1c9,0x1c8,0x1c7,0x1c6,0x1c5,0x1c4,0x1c3,0x1c1,0x1c0,0x1bf,0x1be,0x1bd,0x1bc,
    0x1bb,0x1ba,0x1b9,0x1b8,0x1b7,0x1b6,0x1b5,0x1b3,0x1b2,0x1b1,0x1b0,0x1af,0x1ae,0x1ad,0x1ac,0x1ab,
    0x1aa,0x1a9,0x1a8,0x1a7,0x1a6,0x1a5,0x1a4,0x1a3,0x1a2,0x1a1,0x19f,0x19e,0x19d,0x19c,0x19b,0x19a,
    0x199,0x198,0x197,0x196,0x195,0x194,0x193,0x192,0x191,0x190,0x18f,0x18e,0x18d,0x18c,0x18b,0x18a,
    0x189,0x188,0x187,0x186,0x185,0x184,0x183,0x182,0x181,0x180,0x17f,0x17e,0x17d,0x17c,0x17b,0x17a,
    0x179,0x178,0x177,0x176,0x175,0x174,0x173,0x172,0x171,0x170,0x16f,0x16e,0x16d,0x16c,0x16c,0x16b,
    0x16a,0x169,0x168,0x167,0x166,0x165,0x164,0x163,0x162,0x161,0x160,0x15f,0x15e,0x15d,0x15c,0x15c,
    0x15b,0x15a,0x159,0x158,0x157,0x156,0x155,0x154,0x153,0x152,0x151,0x151,0x150,0x14f,0x14e,0x14d,
    0x14c,0x14b,0x14a,0x149,0x149,0x148,0x147,0x146,0x145,0x144,0x143,0x142,0x141,0x141,0x140,0x13f,
    0x13e,0x13d,0x13c,0x13b,0x13b,0x13a,0x139,0x138,0x137,0x136,0x135
};

/*R1 = 10K VDD=1.8V for trial board v1.10 JINGPU temperature sensor*/
/*temperature 20 ~ 45 degree celsius, step is 0.1 degree*/
static const uint16_t adc_data_table[] =
{
    0x239,0x238,0x236,0x235,0x234,0x233,0x232,0x231,0x230,0x22f,0x22d,0x22c,0x22b,0x22a,0x229,0x228,
    0x227,0x225,0x224,0x223,0x222,0x221,0x220,0x21f,0x21d,0x21c,0x21b,0x21a,0x219,0x218,0x217,0x216,
    0x214,0x213,0x212,0x211,0x210,0x20f,0x20e,0x20c,0x20b,0x20a,0x209,0x208,0x207,0x206,0x205,0x203,
    0x202,0x201,0x200,0x1ff,0x1fe,0x1fd,0x1fc,0x1fa,0x1f9,0x1f8,0x1f7,0x1f6,0x1f5,0x1f4,0x1f3,0x1f1,
    0x1f0,0x1ef,0x1ee,0x1ed,0x1ec,0x1eb,0x1ea,0x1e9,0x1e7,0x1e6,0x1e5,0x1e4,0x1e3,0x1e2,0x1e1,0x1e0,
    0x1df,0x1de,0x1dd,0x1db,0x1da,0x1d9,0x1d8,0x1d7,0x1d6,0x1d5,0x1d4,0x1d3,0x1d2,0x1d1,0x1cf,0x1ce,
    0x1cd,0x1cc,0x1cb,0x1ca,0x1c9,0x1c8,0x1c7,0x1c6,0x1c5,0x1c4,0x1c2,0x1c1,0x1c0,0x1bf,0x1be,0x1bd,
    0x1bc,0x1bb,0x1ba,0x1b9,0x1b8,0x1b7,0x1b6,0x1b5,0x1b4,0x1b3,0x1b1,0x1b0,0x1af,0x1ae,0x1ad,0x1ac,
    0x1ab,0x1aa,0x1a9,0x1a8,0x1a7,0x1a6,0x1a5,0x1a4,0x1a3,0x1a2,0x1a1,0x1a0,0x19f,0x19e,0x19d,0x19c,
    0x19b,0x19a,0x199,0x198,0x197,0x196,0x195,0x194,0x193,0x192,0x191,0x190,0x18f,0x18e,0x18d,0x18c,
    0x18b,0x18a,0x189,0x188,0x187,0x186,0x185,0x184,0x183,0x182,0x181,0x180,0x17f,0x17e,0x17d,0x17c,
    0x17b,0x17a,0x179,0x178,0x177,0x176,0x175,0x174,0x173,0x172,0x171,0x170,0x16f,0x16e,0x16d,0x16c,
    0x16b,0x16b,0x16a,0x169,0x168,0x167,0x166,0x165,0x164,0x163,0x162,0x161,0x160,0x15f,0x15e,0x15d,
    0x15c,0x15b,0x15b,0x15a,0x159,0x158,0x157,0x156,0x155,0x154,0x153,0x152,0x151,0x151,0x150,0x14f,
    0x14e,0x14d,0x14c,0x14b,0x14a,0x149,0x149,0x148,0x147,0x146,0x145,0x144,0x143,0x142,0x142,0x141,
    0x140,0x13f,0x13e,0x13d,0x13c,0x13c,0x13b,0x13a,0x139,0x138,0x137
};

/*R1 = 30K VDD=1.8V for DVT board v1.20 JINGPU temperature sensor*/
/*temperature 20 ~ 45 degree celsius, step is 0.1 degree*/
static const uint16_t adc_data_table[] =
{
    0x12d,0x12c,0x12b,0x12a,0x129,0x128,0x127,0x126,0x125,0x125,0x124,0x123,0x122,0x121,0x120,0x11f,
    0x11e,0x11d,0x11c,0x11b,0x11a,0x119,0x119,0x118,0x117,0x116,0x115,0x114,0x113,0x112,0x111,0x111,
    0x110,0x10f,0x10e,0x10d,0x10c,0x10b,0x10a,0x109,0x109,0x108,0x107,0x106,0x105,0x104,0x103,0x102,
    0x102,0x101,0x100,0xff,0xfe,0xfd,0xfd,0xfc,0xfb,0xfa,0xf9,0xf9,0xf8,0xf7,0xf6,0xf5,
    0xf4,0xf4,0xf3,0xf2,0xf1,0xf0,0xf0,0xef,0xee,0xed,0xec,0xec,0xeb,0xea,0xe9,0xe9,
    0xe8,0xe7,0xe6,0xe6,0xe5,0xe4,0xe3,0xe2,0xe2,0xe1,0xe0,0xdf,0xdf,0xde,0xdd,0xdd,
    0xdc,0xdb,0xda,0xda,0xd9,0xd8,0xd7,0xd7,0xd6,0xd5,0xd4,0xd4,0xd3,0xd2,0xd2,0xd1,
    0xd0,0xcf,0xcf,0xce,0xcd,0xcd,0xcc,0xcb,0xcb,0xca,0xc9,0xc9,0xc8,0xc7,0xc7,0xc6,
    0xc5,0xc5,0xc4,0xc3,0xc3,0xc2,0xc1,0xc1,0xc0,0xbf,0xbf,0xbe,0xbd,0xbd,0xbc,0xbb,
    0xbb,0xba,0xba,0xb9,0xb8,0xb8,0xb7,0xb6,0xb6,0xb5,0xb5,0xb4,0xb3,0xb3,0xb2,0xb2,
    0xb1,0xb0,0xb0,0xaf,0xaf,0xae,0xad,0xad,0xac,0xac,0xab,0xaa,0xaa,0xa9,0xa9,0xa8,
    0xa8,0xa7,0xa6,0xa6,0xa5,0xa5,0xa4,0xa4,0xa3,0xa3,0xa2,0xa1,0xa1,0xa0,0xa0,0x9f,
    0x9f,0x9e,0x9e,0x9d,0x9d,0x9c,0x9b,0x9b,0x9a,0x9a,0x99,0x99,0x98,0x98,0x97,0x97,
    0x96,0x96,0x95,0x95,0x94,0x94,0x93,0x93,0x92,0x92,0x91,0x91,0x90,0x90,0x8f,0x8f,
    0x8e,0x8e,0x8d,0x8d,0x8c,0x8c,0x8b,0x8b,0x8a,0x8a,0x89,0x89,0x88,0x88,0x88,0x87,
    0x87,0x86,0x86,0x85,0x85,0x84,0x84,0x83,0x83,0x83,0x82,
};
#else
/*R1 = 30K,VDD=2.2V for DVT board v1.20 JINGPU temperature sensor*/
/*temperature 20 ~ 45 degree celsius, step is 0.1 degree*/
static const uint16_t adc_data_table[] =
{
 0x170,0x16f,0x16e,0x16d,0x16b,0x16a,0x169,0x168,0x167,0x166,0x164,0x163,0x162,0x161,0x160,0x15f,
 0x15e,0x15d,0x15b,0x15a,0x159,0x158,0x157,0x156,0x155,0x154,0x152,0x151,0x150,0x14f,0x14e,0x14d,
 0x14c,0x14b,0x14a,0x149,0x148,0x147,0x146,0x144,0x143,0x142,0x141,0x140,0x13f,0x13e,0x13d,0x13c,
 0x13b,0x13a,0x139,0x138,0x137,0x136,0x135,0x134,0x133,0x132,0x131,0x130,0x12f,0x12e,0x12d,0x12c,
 0x12b,0x12a,0x129,0x128,0x127,0x126,0x125,0x124,0x123,0x122,0x121,0x120,0x11f,0x11e,0x11d,0x11c,
 0x11b,0x11a,0x119,0x119,0x118,0x117,0x116,0x115,0x114,0x113,0x112,0x111,0x110,0x10f,0x10e,0x10e,
 0x10d,0x10c,0x10b,0x10a,0x109,0x108,0x107,0x106,0x105,0x105,0x104,0x103,0x102,0x101,0x100,0xff,
 0xfe,0xfe,0xfd,0xfc,0xfb,0xfa,0xf9,0xf9,0xf8,0xf7,0xf6,0xf5,0xf4,0xf4,0xf3,0xf2,
 0xf1,0xf0,0xf0,0xef,0xee,0xed,0xec,0xeb,0xeb,0xea,0xe9,0xe8,0xe7,0xe7,0xe6,0xe5,
 0xe4,0xe4,0xe3,0xe2,0xe1,0xe1,0xe0,0xdf,0xde,0xde,0xdd,0xdc,0xdb,0xdb,0xda,0xd9,
 0xd8,0xd8,0xd7,0xd6,0xd5,0xd5,0xd4,0xd3,0xd3,0xd2,0xd1,0xd0,0xd0,0xcf,0xce,0xce,
 0xcd,0xcc,0xcb,0xcb,0xca,0xc9,0xc9,0xc8,0xc7,0xc7,0xc6,0xc5,0xc5,0xc4,0xc3,0xc3,
 0xc2,0xc1,0xc1,0xc0,0xbf,0xbf,0xbe,0xbd,0xbd,0xbc,0xbb,0xbb,0xba,0xba,0xb9,0xb8,
 0xb8,0xb7,0xb6,0xb6,0xb5,0xb4,0xb4,0xb3,0xb3,0xb2,0xb1,0xb1,0xb0,0xb0,0xaf,0xae,
 0xae,0xad,0xad,0xac,0xab,0xab,0xaa,0xaa,0xa9,0xa9,0xa8,0xa7,0xa7,0xa6,0xa6,0xa5,
 0xa5,0xa4,0xa3,0xa3,0xa2,0xa2,0xa1,0xa1,0xa0,0xa0,0x9f,
};
#endif

/**@brief Connect event handler.
 *
 * @param[in]   p_lbs       LEDButton Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_lbs_t * p_lbs, ble_evt_t * p_ble_evt)
{
    p_lbs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Disconnect event handler.
 *
 * @param[in]   p_lbs       LEDButton Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_lbs_t * p_lbs, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_lbs->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Write event handler.
 *
 * @param[in]   p_lbs       LEDButton Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_lbs_t * p_lbs, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

   if ((p_evt_write->handle == p_lbs->led_setting_char_handles.value_handle)  &&
       (p_evt_write->len    == CHAR_REC_MAX_LEN)                              &&
       (p_lbs->led_setting_write_handler != NULL)
      )
   {
       p_lbs->led_setting_write_handler(p_lbs, &(p_evt_write->data[0]));
   }
   else if ((p_evt_write->handle == p_lbs->ota_char_handles.value_handle)      &&      // jiefu
            (p_evt_write->len    == CHAR_REC_MAX_LEN)                          &&
            (p_lbs->ota_write_handler != NULL))
   {
       p_lbs->ota_write_handler(p_lbs, &(p_evt_write->data[0]));
   }
   else if ((p_evt_write->handle == p_lbs->data_char_handles.value_handle)     &&      // jiefu
            (p_evt_write->len    == CHAR_REC_MAX_LEN)                          &&
            (p_lbs->data_write_handler != NULL))
   {
       p_lbs->data_write_handler(p_lbs, &(p_evt_write->data[0]));
   }
   else if ((p_evt_write->handle == p_lbs->version_char_handles.value_handle)  &&      // jiefu
            (p_evt_write->len == CHAR_REC_MAX_LEN)                             &&
            (p_lbs->version_write_handler != NULL))
   {
       p_lbs->version_write_handler(p_lbs, &(p_evt_write->data[0]));
   }
#ifndef VIGO_CLEAN_UP
   else if ((p_evt_write->handle == p_lbs->button_char_handles.value_handle)   &&      // jiefu
            (p_evt_write->len    == CHAR_REC_MAX_LEN)                          &&
            (p_lbs->button_write_handler != NULL))
   {
       p_lbs->button_write_handler(p_lbs, &(p_evt_write->data[0]));
   }

   else if ((p_evt_write->handle == p_lbs->test_mode_char_handles.value_handle)&&      // jiefu
            (p_evt_write->len == CHAR_REC_MAX_LEN)                             &&
            (p_lbs->test_mode_write_handler != NULL))
   {
       p_lbs->test_mode_write_handler(p_lbs, &(p_evt_write->data[0]));
   }

   else if ((p_evt_write->handle == p_lbs->calibration_char_handles.value_handle) &&   // jiefu
            (p_evt_write->len    == CHAR_REC_MAX_LEN)                             &&
            (p_lbs->calibration_write_handler != NULL))
   {
       p_lbs->calibration_write_handler(p_lbs, &(p_evt_write->data[0]));
   }
#endif
}


void ble_lbs_on_ble_evt(ble_lbs_t * p_lbs, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_lbs, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_lbs, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_lbs, p_ble_evt);
            break;

        default:
            break;
    }
}




/**@brief Add ota characteristic. UUID: 1525
 *
 * @param[in]   p_lbs        LEDButton Service structure.
 * @param[in]   p_lbs_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t ota_char_add(ble_lbs_t * p_lbs, const ble_lbs_init_t * p_lbs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;


    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_lbs->uuid_type;
    ble_uuid.uuid = LBS_UUID_OTA_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = CHAR_REC_MAX_LEN;//sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = CHAR_REC_MAX_LEN;//sizeof(uint8_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_lbs->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_lbs->ota_char_handles);
}

/**@brief Add data characteristic. UUID: 1526
 *
 * @param[in]   p_lbs        LEDButton Service structure.
 * @param[in]   p_lbs_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t data_char_add(ble_lbs_t * p_lbs, const ble_lbs_init_t * p_lbs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_lbs->uuid_type;
    ble_uuid.uuid = LBS_UUID_DATA_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = CHAR_SEND_MAX_LEN;//sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = CHAR_SEND_MAX_LEN;//sizeof(uint8_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_lbs->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_lbs->data_char_handles);
}




/**@brief Add LED motor characteristic. UUID: 1528
 *
 * @param[in]   p_lbs        LEDButton Service structure.
 * @param[in]   p_lbs_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t led_setting_char_add(ble_lbs_t * p_lbs, const ble_lbs_init_t * p_lbs_init)  // jiefu
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_lbs->uuid_type;
    ble_uuid.uuid = LBS_UUID_LED_SETTING_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = CHAR_SEND_MAX_LEN;//sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = CHAR_SEND_MAX_LEN;//sizeof(uint8_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_lbs->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_lbs->led_setting_char_handles);   // jiefu
}




/**@brief Add version characteristic. UUID: 1530
 *
 * @param[in]   p_lbs        LEDButton Service structure.
 * @param[in]   p_lbs_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t version_char_add(ble_lbs_t * p_lbs, const ble_lbs_init_t * p_lbs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_lbs->uuid_type;
    ble_uuid.uuid = LBS_UUID_VERSION_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = CHAR_SEND_MAX_LEN;//sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = CHAR_SEND_MAX_LEN;//sizeof(uint8_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_lbs->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_lbs->version_char_handles);
}


uint32_t ble_lbs_init(ble_lbs_t * p_lbs, const ble_lbs_init_t * p_lbs_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_lbs->conn_handle       = BLE_CONN_HANDLE_INVALID;

    p_lbs->ota_write_handler = p_lbs_init->ota_write_handler;
    p_lbs->data_write_handler = p_lbs_init->data_write_handler;
    p_lbs->led_setting_write_handler = p_lbs_init->led_setting_write_handler;
    p_lbs->version_write_handler = p_lbs_init->version_write_handler;

#ifndef VIGO_CLEAN_UP
    p_lbs->button_write_handler = p_lbs_init->button_write_handler;
    p_lbs->test_mode_write_handler = p_lbs_init->test_mode_write_handler;
    p_lbs->calibration_write_handler = p_lbs_init->calibration_write_handler;
#endif

    // Add base UUID to softdevice's internal list.
    ble_uuid128_t base_uuid = LBS_UUID_BASE;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_lbs->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    ble_uuid.type = p_lbs->uuid_type;
    ble_uuid.uuid = LBS_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_lbs->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

#ifndef VIGO_CLEAN_UP
    err_code = button_char_add(p_lbs, p_lbs_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
#endif

    err_code  = ota_char_add(p_lbs, p_lbs_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code  = data_char_add(p_lbs, p_lbs_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

#ifndef VIGO_CLEAN_UP
    err_code  = test_mode_char_add(p_lbs, p_lbs_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
#endif

    err_code  = led_setting_char_add(p_lbs, p_lbs_init); // jiefu
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

#ifndef VIGO_CLEAN_UP
    err_code  = calibration_char_add(p_lbs, p_lbs_init); // jiefu
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
#endif
    err_code  = version_char_add(p_lbs, p_lbs_init); // jiefu
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

uint8_t get_temperature_degree(uint16_t adc_val, uint8_t *relative_temp_val)
{
    int low, mid,high;
    low = 0;
    high = sizeof(adc_data_table)/sizeof(uint16_t) - 1;

    if (adc_val > adc_data_table[0])
    {
        *relative_temp_val = 0;
        return 1;
    }
    else if (adc_val < adc_data_table[high])
    {
        *relative_temp_val = high;
        return 1;
    }

    while (low <= high )
    {
        mid = (low + high)/2;
        if (adc_data_table[mid] == adc_val)
        {
            break;
        }
        else
        {
            if (adc_data_table[mid] < adc_val)
            {
                high = mid - 1;
            }
            else
            {
                low = mid + 1;
            }
        }
    }

    *relative_temp_val = mid;
    return 0;
}
void temp_sensor_data_encode(communication_data_type type,uint8_t *send_buf, uint16_t len)
{
    ble_communication_t      *p_com   = &temp_req;
    memset(send_buf,0,len);
    if(p_com->forced_to_store == true)
    {
        SpiFlash_Read_Data(send_buf,extended_flash_rw_param_ptr->read_addr,STORED_TEMP_DATA_LEN);
        memmove(&send_buf[5],send_buf,(STORED_TEMP_DATA_LEN-1));  /*the last byte of every stored temperate data will be ignored*/
        send_buf[0] = MCU_SYNC_BYTE;
        send_buf[1] = p_com->sequence_id;
        send_buf[2] = COMMAND_TYPE_QUERY_TEMPERATURE_DATA;            /*command type*/
        send_buf[3] = LITTLE_ENDIAN_16BIT_BYTE1(patient_id);          /*patient id low 8 bits: 0~7*/
        send_buf[4] = LITTLE_ENDIAN_16BIT_BYTE2(patient_id);          /*patient id high 8 bits: 8~15*/

        extended_flash_rw_param_ptr->read_addr += STORED_TEMP_DATA_LEN;
        if(extended_flash_rw_param_ptr->read_addr >= extended_flash_rw_param_ptr->end_addr)
        {
            extended_flash_rw_param_ptr->read_addr = 0;
            extended_flash_rw_param_ptr->roll_back = false;
        }
    }
    else
    {
        send_buf[0] = MCU_SYNC_BYTE;
        send_buf[1] = p_com->sequence_id;
        send_buf[2] = COMMAND_TYPE_QUERY_TEMPERATURE_DATA;            /*command type*/
        send_buf[3] = LITTLE_ENDIAN_16BIT_BYTE1(patient_id);          /*patient id low 8 bits: 0~7*/
        send_buf[4] = LITTLE_ENDIAN_16BIT_BYTE2(patient_id);          /*patient id high 8 bits: 8~15*/
        get_temperature_degree(adc_value[0],&send_buf[5]);
        get_temperature_degree(adc_value[1],&send_buf[6]);
        get_temperature_degree(adc_value[2],&send_buf[7]);
        get_temperature_degree(adc_value[3],&send_buf[8]);
        get_temperature_degree(adc_value[4],&send_buf[9]);
        get_temperature_degree(adc_value[5],&send_buf[10]);
        send_buf[11] = percentage_batt_lvl;
        send_buf[12] = ((*total_step_counter&0x00FF)>>0);
        send_buf[13] = ((*total_step_counter&0xFF00)>>8);
        if(COMMUNICATION_DATA_WITH_TIMESTAMP == type)
        {
            send_buf[14] = ((*timestamp_ptr&0x000000FF)>>0);
            send_buf[15] = ((*timestamp_ptr&0x0000FF00)>>8);
            send_buf[16] = ((*timestamp_ptr&0x00FF0000)>>16);
            send_buf[17] = ((*timestamp_ptr&0xFF000000)>>24);
        }
    }
}


uint32_t ble_lbs_sensor_data_update(ble_lbs_t * p_lbs, uint8_t *p_buf)
{
    ble_gatts_hvx_params_t params;
    uint16_t len = CHAR_SEND_MAX_LEN;//= sizeof(button_state);
    uint32_t err_code;

    if (p_lbs->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        memset(&params, 0, sizeof(params));
        params.type = BLE_GATT_HVX_NOTIFICATION;
        params.handle = p_lbs->data_char_handles.value_handle;
        params.p_data = p_buf;
        params.p_len = &len;
        err_code = sd_ble_gatts_hvx(p_lbs->conn_handle, &params);
        if ((err_code == NRF_SUCCESS) && (len != CHAR_SEND_MAX_LEN))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }


    return err_code;
}

void version_info_encode(uint8_t *send_buf, uint16_t len)
{
    uint32_t hardware_ver = MFG_HARDWARE_VERSION;
    uint8_t ver_low,ver_high;
    memset(send_buf,0,len);
    send_buf[0] = MCU_SYNC_BYTE;
    send_buf[1] = sequence_id;
    send_buf[2] = COMMAND_TYPE_QUERY_VERSION;                     /*command type*/
    send_buf[3] = LITTLE_ENDIAN_16BIT_BYTE1(patient_id);          /*patient id low 8 bits: 0~7*/
    send_buf[4] = LITTLE_ENDIAN_16BIT_BYTE2(patient_id);          /*patient id high 8 bits: 8~15*/

    send_buf[5] = FIRMWARE_VERSION;
    ver_low = (uint8_t)((hardware_ver & 0x00FFFFFF)>>8);
    ver_high = (uint8_t)((hardware_ver & 0x00FFFFFF)>>16);
    send_buf[6] =  ver_low + ver_high*100; // FIXME: this format of version SHOULD BE modified in future.
    return;
}

void device_id_encode(uint8_t *send_buf, uint16_t len)
{
    uint32_t DEVICEID_0;
    uint32_t DEVICEID_1;

    DEVICEID_0 = NRF_FICR->DEVICEID[0];
    DEVICEID_1 = NRF_FICR->DEVICEID[1];

    memset(send_buf,0,len);
    send_buf[0] = MCU_SYNC_BYTE;
    send_buf[1] = sequence_id;
    send_buf[2] = COMMAND_TYPE_QUERY_DEVICE_ID;                   /*command type*/
    send_buf[3] = LITTLE_ENDIAN_16BIT_BYTE1(patient_id);          /*patient id low 8 bits: 0~7*/
    send_buf[4] = LITTLE_ENDIAN_16BIT_BYTE2(patient_id);          /*patient id high 8 bits: 8~15*/

    send_buf[5] = LITTLE_ENDIAN_32BIT_BYTE1(DEVICEID_0);
    send_buf[6] = LITTLE_ENDIAN_32BIT_BYTE2(DEVICEID_0);
    send_buf[7] = LITTLE_ENDIAN_32BIT_BYTE3(DEVICEID_0);
    send_buf[8] = LITTLE_ENDIAN_32BIT_BYTE4(DEVICEID_0);
    send_buf[9] = LITTLE_ENDIAN_32BIT_BYTE1(DEVICEID_1);
    send_buf[10] = LITTLE_ENDIAN_32BIT_BYTE2(DEVICEID_1);
    send_buf[11] = LITTLE_ENDIAN_32BIT_BYTE3(DEVICEID_1);
    send_buf[12] = LITTLE_ENDIAN_32BIT_BYTE4(DEVICEID_1);
    return;
}

void MAC_address_encode(uint8_t *send_buf, uint16_t len)
{
    uint32_t      err_code;
    ble_gap_addr_t m_ble_addr;/**< Variable for getting and setting of BLE device address. */

    err_code = sd_ble_gap_address_get(&m_ble_addr);
    APP_ERROR_CHECK(err_code);

    memset(send_buf,0,len);
    send_buf[0] = MCU_SYNC_BYTE;
    send_buf[1] = sequence_id;
    send_buf[2] = COMMAND_TYPE_QUERY_MAC_ADDR;                    /*command type*/
    send_buf[3] = LITTLE_ENDIAN_16BIT_BYTE1(patient_id);          /*patient id low 8 bits: 0~7*/
    send_buf[4] = LITTLE_ENDIAN_16BIT_BYTE2(patient_id);          /*patient id high 8 bits: 8~15*/
    send_buf[5] = m_ble_addr.addr[0];
    send_buf[6] = m_ble_addr.addr[1];
    send_buf[7] = m_ble_addr.addr[2];
    send_buf[8] = m_ble_addr.addr[3];
    send_buf[9] = m_ble_addr.addr[4];
    send_buf[10] = m_ble_addr.addr[5];

    return;
}
void current_date_time_encode(communication_command_type type, uint8_t *send_buf, uint16_t len)
{
    memset(send_buf,0,len);
    send_buf[0] = MCU_SYNC_BYTE;
    send_buf[1] = sequence_id;
    send_buf[2] = type;                                           /*command type*/
    send_buf[3] = LITTLE_ENDIAN_16BIT_BYTE1(patient_id);          /*patient id low 8 bits: 0~7*/
    send_buf[4] = LITTLE_ENDIAN_16BIT_BYTE2(patient_id);          /*patient id high 8 bits: 8~15*/
    send_buf[5] = ((*timestamp_ptr&0x000000FF)>>0);
    send_buf[6] = ((*timestamp_ptr&0x0000FF00)>>8);
    send_buf[7] = ((*timestamp_ptr&0x00FF0000)>>16);
    send_buf[8] = ((*timestamp_ptr&0xFF000000)>>24);
    send_buf[9] = *time_zone;
    return;
}

void patient_id_encode(uint8_t *send_buf, uint16_t len)
{
    memset(send_buf,0,len);
    send_buf[0] = MCU_SYNC_BYTE;
    send_buf[1] = sequence_id;
    send_buf[2] = COMMAND_TYPE_SET_PATIENT_ID;                    /*command type*/
    send_buf[3] = LITTLE_ENDIAN_16BIT_BYTE1(patient_id);          /*patient id low 8 bits: 0~7*/
    send_buf[4] = LITTLE_ENDIAN_16BIT_BYTE2(patient_id);          /*patient id high 8 bits: 8~15*/
    return;
}

void patient_motion_status_encode(uint8_t *send_buf, uint16_t len)
{
    memset(send_buf,0,len);
    send_buf[0] = MCU_SYNC_BYTE;
    send_buf[1] = sequence_id;
    send_buf[2] = COMMAND_TYPE_QUERY_MOTION_STATUS;               /*command type*/
    send_buf[3] = LITTLE_ENDIAN_16BIT_BYTE1(patient_id);          /*patient id low 8 bits: 0~7*/
    send_buf[4] = LITTLE_ENDIAN_16BIT_BYTE2(patient_id);          /*patient id high 8 bits: 8~15*/
    send_buf[5] = motion_status;
    return;
}

void patient_leg_flag_encode(uint8_t *send_buf, uint16_t len)
{
    leg_flag_t leg_flag = (leg_flag_t)((MFG_BLE_MAC_L & 0x00010000)>>16);
    if (leg_flag >= UNKNOW_LEG_FLAG )
    {
        leg_flag = LEFT_LEG_FLAG;
    }
    memset(send_buf,0,len);
    send_buf[0] = MCU_SYNC_BYTE;
    send_buf[1] = sequence_id;
    send_buf[2] = COMMAND_TYPE_QUERY_LEG_FLAG;                    /*command type*/
    send_buf[3] = LITTLE_ENDIAN_16BIT_BYTE1(patient_id);          /*patient id low 8 bits: 0~7*/
    send_buf[4] = LITTLE_ENDIAN_16BIT_BYTE2(patient_id);          /*patient id high 8 bits: 8~15*/
    send_buf[5] = leg_flag ;                                      /*1 - Right leg - fake data since there is no real data stored in the flash*/
    return;
}

void log_inquirying_encode(uint8_t *send_buf, uint16_t len)
{
    memset(send_buf,0,len);
    send_buf[0] = MCU_SYNC_BYTE;
    send_buf[1] = sequence_id;
    send_buf[2] = COMMAND_TYPE_QUERY_LOG;                         /*command type*/
    send_buf[3] = internal_flash_param.device_resetreason_log.pwron_reset_cnt;
    send_buf[4] = internal_flash_param.device_resetreason_log.sysresetreq_reset_cnt;
    send_buf[5] = internal_flash_param.device_resetreason_log.watchdog_reset_cnt;
    send_buf[6] = internal_flash_param.device_resetreason_log.pin_reset_cnt;
    return;
}

void log_clearing_encode(uint8_t *send_buf, uint16_t len)
{
    memset(send_buf,0,len);
    send_buf[0] = MCU_SYNC_BYTE;
    send_buf[1] = sequence_id;
    send_buf[2] = COMMAND_TYPE_CLR_LOG;                         /*command type*/
    memset(&internal_flash_param.device_resetreason_log,0,sizeof(device_resetreason_log_t));
    send_buf[3] = internal_flash_param.device_resetreason_log.pwron_reset_cnt;
    send_buf[4] = internal_flash_param.device_resetreason_log.sysresetreq_reset_cnt;
    send_buf[5] = internal_flash_param.device_resetreason_log.watchdog_reset_cnt;
    send_buf[6] = internal_flash_param.device_resetreason_log.pin_reset_cnt;
    return;
}

void battery_voltage_level_encode(uint8_t *send_buf, uint16_t len)
{
    ble_communication_t *p_com = &battery_level;


    memset(send_buf,0,len);
    send_buf[0] = MCU_SYNC_BYTE;
    send_buf[1] = p_com->sequence_id;
    send_buf[2] = COMMAND_TYPE_QUERY_BATTERY_LEVEL;               /*command type*/
    send_buf[3] = LITTLE_ENDIAN_16BIT_BYTE1(patient_id);          /*patient id low 8 bits: 0~7*/
    send_buf[4] = LITTLE_ENDIAN_16BIT_BYTE2(patient_id);          /*patient id high 8 bits: 8~15*/


    send_buf[5] =  LITTLE_ENDIAN_16BIT_BYTE1(percentage_batt_lvl);
    send_buf[6] =  0;

    return;
}


uint32_t ble_lbs_version_data_update(ble_lbs_t * p_lbs, uint8_t *p_buf)
{

    ble_gatts_hvx_params_t params;
    uint16_t len = CHAR_SEND_MAX_LEN;
    uint32_t err_code;

    if (p_lbs->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        memset(&params, 0, sizeof(params));
        params.type = BLE_GATT_HVX_NOTIFICATION;
        params.handle = p_lbs->version_char_handles.value_handle;
        params.p_data = p_buf;
        params.p_len = &len;
        err_code = sd_ble_gatts_hvx(p_lbs->conn_handle, &params);
        if ((err_code == NRF_SUCCESS) && (len != CHAR_SEND_MAX_LEN))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}



uint32_t ble_lbs_led_setting_update(ble_lbs_t * p_lbs, uint8_t *p_buf)
{

    ble_gatts_hvx_params_t params;
    uint16_t len =CHAR_SEND_MAX_LEN;

    uint32_t err_code;

    if (p_lbs->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        memset(&params, 0, sizeof(params));
        params.type = BLE_GATT_HVX_NOTIFICATION;
        params.handle = p_lbs->led_setting_char_handles.value_handle;
        params.p_data = p_buf;
        params.p_len = &len;
        err_code = sd_ble_gatts_hvx(p_lbs->conn_handle, &params);
        if ((err_code == NRF_SUCCESS) && (len != CHAR_SEND_MAX_LEN))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;

}



#ifndef VIGO_CLEAN_UP
uint32_t ble_lbs_on_button_change(ble_lbs_t * p_lbs, uint8_t button_state)
{

     ble_gatts_hvx_params_t params;
    uint16_t len =CHAR_SEND_MAX_LEN;//= sizeof(button_state);
    uint8_t  buf[CHAR_SEND_MAX_LEN];
//  uint8_t i = 0;


    buf[0] = 0x55;
    buf[1] = button_state;
    buf[2] = 0xAA;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
    buf[8] = 0;
    buf[9] = 0;
    buf[10] = 0;
    buf[11] = 0;
    buf[12] = 0;
    buf[13] = 0;
    buf[14] = 0;
    buf[15] = 0;
    buf[16] = 0;
    buf[17] = 0;
    buf[18] = 0;
    buf[19] = 0;

    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_lbs->button_char_handles.value_handle;
    params.p_data = buf;
    params.p_len = &len;

    return sd_ble_gatts_hvx(p_lbs->conn_handle, &params);
}

uint32_t ble_lbs_calibration_data_update(ble_lbs_t * p_lbs, uint8_t button_state)
{

     ble_gatts_hvx_params_t params;
     uint16_t len =CHAR_SEND_MAX_LEN;//= sizeof(button_state);
     uint8_t  buf[CHAR_SEND_MAX_LEN];
//     uint8_t i = 0;

    buf[0] = 0x55;
    //buf[1] = prox_1;
    buf[2] = 0xAA;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
    buf[8] = 0;
    buf[9] = 0;
    buf[10] = 0;
    buf[11] = 0;
    buf[12] = 0;
    buf[13] = 0;
    buf[14] = 0;
    buf[15] = 0;
    buf[16] = 0;
    buf[17] = 0;
    buf[18] = 0;
    buf[19] = 0;

    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_lbs->calibration_char_handles.value_handle;
    params.p_data = buf;
    params.p_len = &len;

    return sd_ble_gatts_hvx(p_lbs->conn_handle, &params);
}

/**@brief Add Button state characteristic. UUID: 1524
 *
 * @param[in]   p_lbs        LEDButton Service structure.
 * @param[in]   p_lbs_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t button_char_add(ble_lbs_t * p_lbs, const ble_lbs_init_t * p_lbs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;


    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_lbs->uuid_type;
    ble_uuid.uuid = LBS_UUID_BUTTON_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = CHAR_SEND_MAX_LEN;///sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = CHAR_SEND_MAX_LEN;//sizeof(uint8_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_lbs->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_lbs->button_char_handles);
}


/**@brief Add test mode characteristic. UUID: 1527
 *
 * @param[in]   p_lbs        LEDButton Service structure.
 * @param[in]   p_lbs_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t test_mode_char_add(ble_lbs_t * p_lbs, const ble_lbs_init_t * p_lbs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;


    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_lbs->uuid_type;
    ble_uuid.uuid = LBS_UUID_TEST_MODE_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = CHAR_REC_MAX_LEN;//sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = CHAR_REC_MAX_LEN;//sizeof(uint8_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_lbs->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_lbs->test_mode_char_handles);
}



/**@brief Add calibration characteristic. UUID: 1529
 *
 * @param[in]   p_lbs        LEDButton Service structure.
 * @param[in]   p_lbs_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t calibration_char_add(ble_lbs_t * p_lbs, const ble_lbs_init_t * p_lbs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_lbs->uuid_type;
    ble_uuid.uuid = LBS_UUID_CALIBRATION_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = CHAR_SEND_MAX_LEN;//sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = CHAR_SEND_MAX_LEN;//sizeof(uint8_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_lbs->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_lbs->calibration_char_handles);
}

#endif



