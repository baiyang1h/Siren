#include "app_error.h"
#include "twi_master.h"
#include "nrf_delay.h"
#include "lsm6dsl_reg.h"
#include <string.h>
#define  WAKEUP_THRESHOLD(x) (x*64/2000)  //mg,full scale:LSM6DSL_2g
static lsm6dsl_ctx_t dev_ctx;
static bool          is_lsm6dsl_init_success = false;

void LSM6DSL_Init(void);
void lsm6dsl_inquiry_all_src(lsm6dsl_all_sources_t *p_all_sources);
/*******************************************************************************
* Function Name    : lsm6dsl_universal_timer_timeout_handler
* Description      : Executed once a second.
*******************************************************************************/
void lsm6dsl_universal_timer_timeout_handler(void)
{
    static uint8_t delay_cnt = 0;
    if(!is_lsm6dsl_init_success)
    {
        delay_cnt++;
        if(delay_cnt>=10)
        {
            delay_cnt = 0;
            LSM6DSL_Init();
        }
    }
}
/*******************************************************************************
* Function Name    : platform_write
* Description      : Write data to lsm6dsl register.
* Input            : None
* Output           : None
* Return           : None
*******************************************************************************/
static int32_t platform_write(interface_type_t handle, uint8_t Reg, uint8_t *Bufp,
                              uint16_t len)
{
    uint8_t buf[LSM6DSL_MAX_COMMUNICATE_LEN+1];
    buf[0] = Reg;
    memcpy(&buf[1],Bufp,len);
    if(handle == interface_i2c)
    {
        twi_master_transfer(LSM6DSL_I2C_ADD_L, buf, len+1, TWI_ISSUE_STOP);
    }
    else if(handle == interface_spi)
    {

    }

    return 0;
}
/*******************************************************************************
* Function Name    : platform_read
* Description      : Read out data from lsm6dsl register.
* Input            : None
* Output           : None
* Return           : None
*******************************************************************************/
static int32_t platform_read(interface_type_t handle, uint8_t Reg, uint8_t *Bufp,
                             uint16_t len)
{
    if(handle == interface_i2c)
    {
        twi_master_transfer(LSM6DSL_I2C_ADD_L, &Reg, 1, TWI_DONT_ISSUE_STOP);
        twi_master_transfer(LSM6DSL_I2C_ADD_L|TWI_READ_BIT, Bufp, len, TWI_ISSUE_STOP);
    }
    else if(handle == interface_spi)
    {

    }

    return 0;
}
/*******************************************************************************
* Function Name    : LSM6DSL_Init
* Description      : lsm6dsl init function. It must be fullfilled with either
*                  : I2C or SPI functions
* Input            : None
* Output           : None
* Return           : None
*******************************************************************************/
void LSM6DSL_Init(void)
{
    uint8_t              whoamI;
    uint8_t              rst;
    lsm6dsl_int1_route_t lsm6dsl_int1_route;
    lsm6dsl_xl_hm_mode_t lsm6dsl_xl_hm_mode;
  
    memset(&lsm6dsl_int1_route,0,sizeof(lsm6dsl_int1_route_t));

    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg  = platform_read;
    dev_ctx.handle    = interface_i2c;
    /* Check device ID */
    whoamI = 0;
    lsm6dsl_device_id_get(&dev_ctx, &whoamI);
    if(whoamI != LSM6DSL_ID)
        return;
    /* Restore default configuration */
    lsm6dsl_reset_set(&dev_ctx, PROPERTY_ENABLE);
    do
    {
        lsm6dsl_reset_get(&dev_ctx, &rst);
    }while(rst);
    /* Enable Block Data Update */
    lsm6dsl_block_data_update_set(&dev_ctx, PROPERTY_DISABLE);
    /* Set Output Data Rate */
    lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_26Hz);
    lsm6dsl_gy_data_rate_set(&dev_ctx, LSM6DSL_GY_ODR_OFF);
    /* Set full scale */
    lsm6dsl_xl_full_scale_set(&dev_ctx, LSM6DSL_2g);
    lsm6dsl_gy_full_scale_set(&dev_ctx, LSM6DSL_2000dps);
    /* Configure filtering chain(No aux interface) */  
    /* Accelerometer - analog filter */
    lsm6dsl_xl_filter_analog_set(&dev_ctx, LSM6DSL_XL_ANA_BW_400Hz);
    /* Accelerometer - LPF1 path ( LPF2 not used )*/
    //lsm6dsl_xl_lp1_bandwidth_set(&dev_ctx, LSM6DSL_XL_LP1_ODR_DIV_4);
    /* Accelerometer - LPF1 + LPF2 path */
    lsm6dsl_xl_lp2_bandwidth_set(&dev_ctx, LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_100);
    /* Accelerometer - High Pass / Slope path */
    //lsm6dsl_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
    //lsm6dsl_xl_hp_bandwidth_set(&dev_ctx, LSM6DSL_XL_HP_ODR_DIV_100);

    /* Gyroscope - filtering chain */
    lsm6dsl_gy_band_pass_set(&dev_ctx, LSM6DSL_HP_260mHz_LP1_STRONG);

    lsm6dsl_pin_polarity_set(&dev_ctx, LSM6DSL_ACTIVE_LOW);
    lsm6dsl_wkup_threshold_set(&dev_ctx, WAKEUP_THRESHOLD(200));//unit mg
    lsm6dsl_wkup_dur_set(&dev_ctx, 3);//unit ODR_time option:10,1,2,3
    lsm6dsl_int_notification_set(&dev_ctx, LSM6DSL_INT_LATCHED);

    lsm6dsl_int1_route.int1_wu = PROPERTY_ENABLE;
    lsm6dsl_pin_int1_route_set(&dev_ctx,lsm6dsl_int1_route);
    lsm6dsl_xl_hm_mode = LSM6DSL_XL_NORMAL;
    lsm6dsl_xl_power_mode_set(&dev_ctx,lsm6dsl_xl_hm_mode);

    lsm6dsl_pedo_sens_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dsl_pedo_step_reset_set(&dev_ctx, PROPERTY_ENABLE);
    is_lsm6dsl_init_success = true;
}
/*******************************************************************************
* Function Name    : lsm6dsl_inquiry_all_src
* Description      : lsm6dsl inquiry all sources.
* Input            : None
* Output           : None
* Return           : None
*******************************************************************************/
void lsm6dsl_inquiry_all_src(lsm6dsl_all_sources_t *p_all_sources)
{
    lsm6dsl_all_sources_get(&dev_ctx, p_all_sources);
}
/*******************************************************************************
* Function Name  : lsm6dsl_GetAccSTEPCounter
* Description    : Read the STEP Values Output Registers
* Input          : 16-bits pointer
* Output         : None
* Return         : None
*******************************************************************************/
void lsm6dsl_GetAccSTEPCounter(uint16_t* val)
{
    uint16_t value;
    uint8_t *valueL = (uint8_t *)(&value);
    uint8_t *valueH = ((uint8_t *)(&value)+1);

    lsm6dsl_read_reg(&dev_ctx, LSM6DSL_STEP_COUNTER_L, valueL, 1);
    lsm6dsl_read_reg(&dev_ctx, LSM6DSL_STEP_COUNTER_L, valueH, 1);
    *val = value;
}

