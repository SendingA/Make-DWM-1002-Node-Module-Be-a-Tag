// Inspired by Kris Winer (https://github.com/kriswiner/LSM6DSM_LIS2MDL_LPS22HB)
// Adapted for NRF52 by Niklas Casaril <niklas@loligoelectronics.com>

#define NRF_LOG_MODULE_NAME LPS22HB

#include "boards.h"

#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "nrf_log.h"

NRF_LOG_MODULE_REGISTER();

#include "nrf_drv_twi.h"
#include "nrf_drv_spi.h"
#include "lps22hb.h"
#include "lps22hb_reg.h"
#include "dw_pdoa_tag_common.h"
#include <math.h>

struct lps22hb_cfg _cfg = LPS22HB_CFG_DEFAULTS;

static uint8_t lps22hb_read(uint8_t reg)
{
    uint8_t out[1] = {reg};
    uint8_t in[1];

    if (_cfg.spi) {
        /* Mark as read */
        out[0] |= 0x80;
        nrf_gpio_pin_clear(_cfg.spi_cs_pin);
        nrf_drv_spi_transfer(_cfg.spi, out, 1, NULL, 0);
        nrf_drv_spi_transfer(_cfg.spi, out, 1, in, 1);

        nrf_gpio_pin_set(_cfg.spi_cs_pin);
    }

    if (_cfg.twi) {
        nrf_drv_twi_enable(_cfg.twi);
        nrf_drv_twi_tx(_cfg.twi, _cfg.twi_addr, out, sizeof(out), false);
        nrf_drv_twi_rx(_cfg.twi, _cfg.twi_addr, in, sizeof(in));
        nrf_drv_twi_disable(_cfg.twi);
    }

    return in[0];
}

static void lps22hb_write(uint8_t reg, uint8_t value)
{
    uint8_t out[2] = {reg, value};

    if (_cfg.spi) {
        nrf_gpio_pin_clear(_cfg.spi_cs_pin);
        nrf_drv_spi_transfer(_cfg.spi, out, 2, NULL, 0);
        nrf_gpio_pin_set(_cfg.spi_cs_pin);
        return;
    }
        
    if (_cfg.twi) {
        nrf_drv_twi_enable(_cfg.twi);
        nrf_drv_twi_tx(_cfg.twi, _cfg.twi_addr, out, sizeof(out), false);
        nrf_drv_twi_disable(_cfg.twi);
    }

}

int32_t lps22hb_hwtest(void)
{
    uint8_t device_id = lps22hb_read(LPS22HB_WHO_AM_I);
    uint8_t result = (device_id == LPS22HB_WHO_AM_I_RES);
    if (!result)
    {
        NRF_LOG_ERROR("HWTEST: %u, got=%X, expected=%X", result, device_id,
                     LPS22HB_WHO_AM_I_RES);
    }
    return (result) ? NRF_SUCCESS : 1;
}

void lps22hb_init(struct lps22hb_cfg *cfg)
{
    memcpy(&_cfg, cfg, sizeof(struct lps22hb_cfg));

    lps22hb_write(LPS22HB_CTRL_REG2, LPS22HB_REG2_RESET);
    lps22hb_write(LPS22HB_CTRL_REG3, LPS22HB_REG3_DRDY);
}

void lps22hb_oneshot()
{
    lps22hb_write(LPS22HB_CTRL_REG2, LPS22HB_REG2_ONESHOT);
}

lps22hb_result_t lps22hb_poll(void)
{
    lps22hb_result_t result;

    uint8_t temp_h = lps22hb_read(LPS22HB_TEMP_OUT_H);
    uint8_t temp_l = lps22hb_read(LPS22HB_TEMP_OUT_L);
    result.temperature = ((int32_t)temp_h << 8) | (int32_t)temp_l;

    uint8_t press_l = lps22hb_read(LPS22HB_PRESS_OUT_L);
    uint8_t press_h = lps22hb_read(LPS22HB_PRESS_OUT_H);
    uint8_t press_xl = lps22hb_read(LPS22HB_PRESS_OUT_XL);
    result.pressure = (((uint32_t)press_h << 16) | ((uint32_t)press_l << 8) |
                       ((uint32_t)press_xl)) *
                      100 / 4096;

    return result;
}

void lps22hb_cfg_init(struct lps22hb_cfg *cfg)
{
    memcpy(&_cfg, cfg, sizeof(struct lps22hb_cfg));
}

void lps22hb_lowpower_cfg(void)
{
    uint8_t temp = lps22hb_read(LPS22HB_RES_CONF);
    temp =  0x1;
    lps22hb_write(LPS22HB_RES_CONF, temp);
}

void lps22hb_power_dowm()
{
    uint8_t temp = lps22hb_read(LPS22HB_RES_CONF);
    temp =  0x0;
    lps22hb_write(LPS22HB_CTRL_REG1, temp);
}

void sensor_10dof_lps22hb_read(uint8_t address, uint8_t *buffer)
{
    *buffer = lps22hb_read(address);
}

void sensor_10dof_lps22hb_write(uint8_t address, uint8_t buffer)
{
    lps22hb_write(address, buffer);
}

void lps22hb_offsetBias(uint32_t * pressure_bias)
{
    uint32_t temp[20] = {0};
    
    lps22hb_result_t result;
    uint32_t sum = 0, mean_val;
    
    for (int ii = 0; ii < 20; ii++)
    {
        result = lps22hb_poll();
        sum += result.pressure;

        lps22hb_oneshot();
        temp[ii] = result.pressure;
    }
    
    mean_val = sum/20;
    
    sum = 0;
    for (int ii = 0; ii < 20; ii++)
    {
        sum += powf((temp[ii] - mean_val), 2);
    }
    
    pressure_bias[0] = sqrt(sum/20);
}