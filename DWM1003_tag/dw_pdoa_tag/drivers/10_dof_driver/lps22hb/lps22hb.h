// Inspired by Kris Winer (https://github.com/kriswiner/LSM6DSM_LIS2MDL_LPS22HB)
// Adapted for NRF52 by Niklas Casaril <niklas@loligoelectronics.com>

#ifndef LPS22HB_H
#define LPS22HB_H

#include "nrf_drv_twi.h"

typedef struct {
    int32_t temperature;
    uint32_t pressure;
} lps22hb_result_t;

struct nrf_drv_twi_t;
struct nrf_drv_spi_t;

struct lps22hb_cfg {
    const nrf_drv_twi_t *twi;
    uint8_t twi_addr;
    const nrf_drv_spi_t *spi;
    void (*spi_rd_cb)(int en);
    int spi_cs_pin;
    int int_pin;
};
#define LPS22HB_CFG_DEFAULTS {.twi=0,.spi=0,.spi_rd_cb=0,.spi_cs_pin=0xff,.int_pin=0xff}

#ifdef __cplusplus
extern "C" {
#endif

void lps22hb_init(struct lps22hb_cfg *cfg);
int32_t lps22hb_hwtest(void);
void lps22hb_oneshot(void);
lps22hb_result_t lps22hb_poll(void);
void lps22hb_lowpower_cfg(void);
void lps22hb_cfg_init(struct lps22hb_cfg *cfg);
void sensor_10dof_lps22hb_read(uint8_t address, uint8_t *buffer);
void sensor_10dof_lps22hb_write(uint8_t address, uint8_t buffer);
void lps22hb_offsetBias(uint32_t * pressure_bias);
void lps22hb_power_dowm();

#ifdef __cplusplus
}
#endif
#endif

