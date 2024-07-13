#ifndef LSM6DSR_H
#define LSM6DSR_H

// Inspired by Kris Winer (https://github.com/kriswiner/LSM6DSR_LIS2MDL_LPS22HB)
// Adapted for NRF52 by Niklas Casaril <niklas@loligoelectronics.com>

/* LSM6DSR registers
  http://www.st.com/content/ccc/resource/technical/document/datasheet/76/27/cf/88/c5/03/42/6b/DM00218116.pdf/files/DM00218116.pdf/jcr:content/translations/en.DM00218116.pdf
*/
#define LSM6DSR_FUNC_CFG_ACCESS       0x01
#define LSM6DSR_PIN_CTRL              0x02
#define LSM6DSR_FIFO_CTRL1            0x07
#define LSM6DSR_FIFO_CTRL2            0x08
#define LSM6DSR_FIFO_CTRL3            0x09
#define LSM6DSR_FIFO_CTRL4            0x0A
#define LSM6DSR_COUNTER_BDR_REG1      0x0B
#define LSM6DSR_COUNTER_BDR_REG2      0x0C
#define LSM6DSR_INT1_CTRL             0x0D
#define LSM6DSR_INT2_CTRL             0x0E
#define LSM6DSR_WHO_AM_I              0x0F   // should be 0x6B
#define LSM6DSR_WHO_AM_I_VALUE        0x6B
#define LSM6DSR_CTRL1_XL              0x10
#define LSM6DSR_CTRL2_G               0x11
#define LSM6DSR_CTRL3_C               0x12
#define LSM6DSR_CTRL4_C               0x13
#define LSM6DSR_CTRL5_C               0x14
#define LSM6DSR_CTRL6_C               0x15
#define LSM6DSR_CTRL7_G               0x16
#define LSM6DSR_CTRL8_XL              0x17
#define LSM6DSR_CTRL9_XL              0x18
#define LSM6DSR_CTRL10_C              0x19
#define LSM6DSR_ALL_INT_SRC           0x1A
#define LSM6DSR_WAKE_UP_SRC           0x1B
#define LSM6DSR_TAP_SRC               0x1C
#define LSM6DSR_D6D_SRC               0x1D
#define LSM6DSR_STATUS_REG            0x1E
#define LSM6DSR_OUT_TEMP_L            0x20
#define LSM6DSR_OUT_TEMP_H            0x21
#define LSM6DSR_OUTX_L_G              0x22
#define LSM6DSR_OUTX_H_G              0x23
#define LSM6DSR_OUTY_L_G              0x24
#define LSM6DSR_OUTY_H_G              0x25
#define LSM6DSR_OUTZ_L_G              0x26
#define LSM6DSR_OUTZ_H_G              0x27
#define LSM6DSR_OUTX_L_A              0x28
#define LSM6DSR_OUTX_H_A              0x29
#define LSM6DSR_OUTY_L_A              0x2A
#define LSM6DSR_OUTY_H_A              0x2B
#define LSM6DSR_OUTZ_L_A              0x2C
#define LSM6DSR_OUTZ_H_A              0x2D
#define LSM6DSR_FIFO_STATUS1          0x3A
#define LSM6DSR_FIFO_STATUS2          0x3B
#define LSM6DSR_TIMESTAMP0            0x40
#define LSM6DSR_TIMESTAMP1            0x41
#define LSM6DSR_TIMESTAMP2            0x42
#define LSM6DSR_TIMESTAMP3            0x43
#define LSM6DSR_TAP_CFG0              0x56
#define LSM6DSR_TAP_CFG1              0x57
#define LSM6DSR_TAP_CFG2              0x58
#define LSM6DSR_TAP_THS_6D            0x59
#define LSM6DSR_INT_DUR2              0x5A
#define LSM6DSR_WAKE_UP_THS           0x5B
#define LSM6DSR_WAKE_UP_DUR           0x5C
#define LSM6DSR_FREE_FALL             0x5D
#define LSM6DSR_MD1_CFG               0x5E
#define LSM6DSR_MD2_CFG               0x5F
#define LSM6DSR_INT_OIS               0x6F
#define LSM6DSR_CTRL1_OIS             0x70
#define LSM6DSR_CTRL2_OIS             0x71
#define LSM6DSR_CTRL3_OIS             0x72
#define LSM6DSR_X_OFS_USR             0x73
#define LSM6DSR_Y_OFS_USR             0x74
#define LSM6DSR_Z_OFS_USR             0x75
#define LSM6DSR_FIFO_DATA_OUT_TAG     0x78
#define LSM6DSR_FIFO_DATA_OUT_X_L     0x79
#define LSM6DSR_FIFO_DATA_OUT_X_H     0x7A
#define LSM6DSR_FIFO_DATA_OUT_Y_L     0x7B
#define LSM6DSR_FIFO_DATA_OUT_Y_H     0x7C
#define LSM6DSR_FIFO_DATA_OUT_Z_L     0x7D
#define LSM6DSR_FIFO_DATA_OUT_Z_H     0x7E

#define LSM6DSR_AFS_2G  0x00
#define LSM6DSR_AFS_4G  0x02
#define LSM6DSR_AFS_8G  0x03
#define LSM6DSR_AFS_16G 0x01

#define LSM6DSR_GFS_245DPS  0x00
#define LSM6DSR_GFS_500DPS  0x01
#define LSM6DSR_GFS_1000DPS 0x02
#define LSM6DSR_GFS_2000DPS 0x03

#define LSM6DSR_AODR_12_5Hz  0x01  // same for accel and gyro in normal mode
#define LSM6DSR_AODR_26Hz    0x02
#define LSM6DSR_AODR_52Hz    0x03
#define LSM6DSR_AODR_104Hz   0x04
#define LSM6DSR_AODR_208Hz   0x05
#define LSM6DSR_AODR_416Hz   0x06
#define LSM6DSR_AODR_833Hz   0x07
#define LSM6DSR_AODR_1660Hz  0x08
#define LSM6DSR_AODR_3330Hz  0x09
#define LSM6DSR_AODR_6660Hz  0x0A

#define LSM6DSR_GODR_12_5Hz  0x01   
#define LSM6DSR_GODR_26Hz    0x02
#define LSM6DSR_GODR_52Hz    0x03
#define LSM6DSR_GODR_104Hz   0x04
#define LSM6DSR_GODR_208Hz   0x05
#define LSM6DSR_GODR_416Hz   0x06
#define LSM6DSR_GODR_833Hz   0x07
#define LSM6DSR_GODR_1660Hz  0x08
#define LSM6DSR_GODR_3330Hz  0x09
#define LSM6DSR_GODR_6660Hz  0x0A


struct nrf_drv_twi_t;
struct nrf_drv_spi_t;

struct lsm6dsr_cfg {
    const nrf_drv_twi_t *twi;
    uint8_t twi_addr;
    const nrf_drv_spi_t *spi;
    void (*spi_rd_cb)(int en);
    int spi_cs_pin;
    int int_pin;
    int32_t acc_sensitivity;
    int32_t gyro_sensitivity;
};
#define LSM6DSR_CFG_DEFAULTS {.twi=0,.spi=0,.spi_rd_cb=0,.spi_cs_pin=0xff,.int_pin=0xff}


#ifdef __cplusplus
extern "C" {
#endif

    void lsm6dsr_init(struct lsm6dsr_cfg *cfg);
    int32_t lsm6dsr_hwtest();

    void lsm6dsr_offsetBias(float * dest1, float * dest2);
    uint8_t lsm6dsr_readData_i(int16_t *gyro, int16_t *acc);
    uint8_t lsm6dsr_readData_f(float *dest1, float *dest2);

#ifdef __cplusplus
}
#endif

#endif
