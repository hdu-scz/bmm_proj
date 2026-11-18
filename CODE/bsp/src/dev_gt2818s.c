/**
 * @file dev_gt2818s.c
 * @brief GT2818S AFE 设备对象实现 (BSP 层)
 * @author 
 * @date 2026-07-16
 * @note
 */

#include <string.h>
#include "dev_gt2818s.h"
#include "periph_spi.h"
#include "periph_io.h"
#include "periph_dwt.h"

/* ==================== CS 电平宏定义 ==================== */
#define CS_HIGH  (1U)
#define CS_LOW   (0U)

/* CRC-15 PEC table, poly = 0x4599 */
static const uint16_t crc15Table[256] = {
    0x0000, 0xC599, 0xCEAB, 0x0B32, 0xD8CF, 0x1D56, 0x1664, 0xD3FD,
    0xF407, 0x319E, 0x3AAC, 0xFF35, 0x2CC8, 0xE951, 0xE263, 0x27FA,
    0xAD97, 0x680E, 0x633C, 0xA6A5, 0x7558, 0xB0C1, 0xBBF3, 0x7E6A,
    0x5990, 0x9C09, 0x973B, 0x52A2, 0x815F, 0x44C6, 0x4FF4, 0x8A6D,
    0x5B2E, 0x9EB7, 0x9585, 0x501C, 0x83E1, 0x4678, 0x4D4A, 0x88D3,
    0xAF29, 0x6AB0, 0x6182, 0xA41B, 0x77E6, 0xB27F, 0xB94D, 0x7CD4,
    0xF6B9, 0x3320, 0x3812, 0xFD8B, 0x2E76, 0xEBEF, 0xE0DD, 0x2544,
    0x02BE, 0xC727, 0xCC15, 0x098C, 0xDA71, 0x1FE8, 0x14DA, 0xD143,
    0xF3C5, 0x365C, 0x3D6E, 0xF8F7, 0x2B0A, 0xEE93, 0xE5A1, 0x2038,
    0x07C2, 0xC25B, 0xC969, 0x0CF0, 0xDF0D, 0x1A94, 0x11A6, 0xD43F,
    0x5E52, 0x9BCB, 0x90F9, 0x5560, 0x869D, 0x4304, 0x4836, 0x8DAF,
    0xAA55, 0x6FCC, 0x64FE, 0xA167, 0x729A, 0xB703, 0xBC31, 0x79A8,
    0xA8EB, 0x6D72, 0x6640, 0xA3D9, 0x7024, 0xB5BD, 0xBE8F, 0x7B16,
    0x5CEC, 0x9975, 0x9247, 0x57DE, 0x8423, 0x41BA, 0x4A88, 0x8F11,
    0x057C, 0xC0E5, 0xCBD7, 0x0E4E, 0xDDB3, 0x182A, 0x1318, 0xD681,
    0xF17B, 0x34E2, 0x3FD0, 0xFA49, 0x29B4, 0xEC2D, 0xE71F, 0x2286,
    0xA213, 0x678A, 0x6CB8, 0xA921, 0x7ADC, 0xBF45, 0xB477, 0x71EE,
    0x5614, 0x938D, 0x98BF, 0x5D26, 0x8EDB, 0x4B42, 0x4070, 0x85E9,
    0x0F84, 0xCA1D, 0xC12F, 0x04B6, 0xD74B, 0x12D2, 0x19E0, 0xDC79,
    0xFB83, 0x3E1A, 0x3528, 0xF0B1, 0x234C, 0xE6D5, 0xEDE7, 0x287E,
    0xF93D, 0x3CA4, 0x3796, 0xF20F, 0x21F2, 0xE46B, 0xEF59, 0x2AC0,
    0x0D3A, 0xC8A3, 0xC391, 0x0608, 0xD5F5, 0x106C, 0x1B5E, 0xDEC7,
    0x54AA, 0x9133, 0x9A01, 0x5F98, 0x8C65, 0x49FC, 0x42CE, 0x8757,
    0xA0AD, 0x6534, 0x6E06, 0xAB9F, 0x7862, 0xBDFB, 0xB6C9, 0x7350,
    0x51D6, 0x944F, 0x9F7D, 0x5AE4, 0x8919, 0x4C80, 0x47B2, 0x822B,
    0xA5D1, 0x6048, 0x6B7A, 0xAEE3, 0x7D1E, 0xB887, 0xB3B5, 0x762C,
    0xFC41, 0x39D8, 0x32EA, 0xF773, 0x248E, 0xE117, 0xEA25, 0x2FBC,
    0x0846, 0xCDDF, 0xC6ED, 0x0374, 0xD089, 0x1510, 0x1E22, 0xDBBB,
    0x0AF8, 0xCF61, 0xC453, 0x01CA, 0xD237, 0x17AE, 0x1C9C, 0xD905,
    0xFEFF, 0x3B66, 0x3054, 0xF5CD, 0x2630, 0xE3A9, 0xE89B, 0x2D02,
    0xA76F, 0x62F6, 0x69C4, 0xAC5D, 0x7FA0, 0xBA39, 0xB10B, 0x7492,
    0x5368, 0x96F1, 0x9DC3, 0x585A, 0x8BA7, 0x4E3E, 0x450C, 0x8095
};

/* ==================== 私有函数声明 ==================== */
static void     init(Dev_Gt2818s_t *self);
static void     deinit(Dev_Gt2818s_t *self);
static void     wakeup(Dev_Gt2818s_t *self);
static void     cs_set(Dev_Gt2818s_t *self, uint8_t level);
static uint8_t  write_cfg(Dev_Gt2818s_t *self);
static uint8_t  read_cfg(Dev_Gt2818s_t *self);
static uint8_t  adcv(Dev_Gt2818s_t *self, uint8_t adc_mode, uint8_t ch);
static uint8_t  rdcv(Dev_Gt2818s_t *self);
static uint8_t  adgp(Dev_Gt2818s_t *self, uint8_t adc_mode, uint8_t ch);
static uint8_t  rdgp(Dev_Gt2818s_t *self);
static uint8_t  clrcell(Dev_Gt2818s_t *self);
static uint8_t  clraux(Dev_Gt2818s_t *self);
static uint16_t pec15_cal(uint16_t len, uint8_t *pData);

/* ==================== 全局 GT2818S 设备对象实例 ==================== */
Dev_Gt2818s_t dev_gt2818s_01 = {
    .periph = {
        .spi   = &periph_spi0,
        .io_cs = &periph_io[IO_PB12], /* CS0 */
    },
    .reg = {0},
    .func = {
        .isInitialized = 0,
        .init          = init,
        .deinit        = deinit,
        .wakeup        = wakeup,
        .cs_set        = cs_set,
        .write_cfg     = write_cfg,
        .read_cfg      = read_cfg,
        .adcv          = adcv,
        .rdcv          = rdcv,
        .adgp          = adgp,
        .rdgp          = rdgp,
        .clrcell       = clrcell,
        .clraux        = clraux,
    },
};

Dev_Gt2818s_t dev_gt2818s_02 = {
    .periph = {
        .spi   = &periph_spi0,
        .io_cs = &periph_io[IO_PC4], /* CS1 */
    },
    .reg = {0},
    .func = {
        .isInitialized = 0,
        .init          = init,
        .deinit        = deinit,
        .wakeup        = wakeup,
        .cs_set        = cs_set,
        .write_cfg     = write_cfg,
        .read_cfg      = read_cfg,
        .adcv          = adcv,
        .rdcv          = rdcv,
        .adgp          = adgp,
        .rdgp          = rdgp,
        .clrcell       = clrcell,
        .clraux        = clraux,
    },
};

Dev_Gt2818s_t dev_gt2818s_03 = {
    .periph = {
        .spi   = &periph_spi0,
        .io_cs = &periph_io[IO_PB1], /* CS2 */
    },
    .reg = {0},
    .func = {
        .isInitialized = 0,
        .init          = init,
        .deinit        = deinit,
        .wakeup        = wakeup,
        .cs_set        = cs_set,
        .write_cfg     = write_cfg,
        .read_cfg      = read_cfg,
        .adcv          = adcv,
        .rdcv          = rdcv,
        .adgp          = adgp,
        .rdgp          = rdgp,
        .clrcell       = clrcell,
        .clraux        = clraux,
    },
};

Dev_Gt2818s_t dev_gt2818s_04 = {
    .periph = {
        .spi   = &periph_spi0,
        .io_cs = &periph_io[IO_PA2], /* CS3 */
    },
    .reg = {0},
    .func = {
        .isInitialized = 0,
        .init          = init,
        .deinit        = deinit,
        .wakeup        = wakeup,
        .cs_set        = cs_set,
        .write_cfg     = write_cfg,
        .read_cfg      = read_cfg,
        .adcv          = adcv,
        .rdcv          = rdcv,
        .adgp          = adgp,
        .rdgp          = rdgp,
        .clrcell       = clrcell,
        .clraux        = clraux,
    },
};

/* ==================== 方法实现 ==================== */

static void init(Dev_Gt2818s_t *self)
{
    if (self == 0) {
        return;
    }

    if (self->periph.spi != 0 && self->periph.spi->init != 0) {
        self->periph.spi->init();
    }

    if (self->periph.io_cs != 0) {
        self->periph.io_cs->init(IO_MODE_OUTPUT_PP);
        self->periph.io_cs->write(CS_HIGH);
    }

    self->func.isInitialized = 1;
}

static void deinit(Dev_Gt2818s_t *self)
{
    if (self == 0) {
        return;
    }

    if (self->periph.io_cs != 0) {
        self->periph.io_cs->deinit();
    }

    if (self->periph.spi != 0) {
        self->periph.spi->deinit();
    }

    self->func.isInitialized = 0;
}

static void wakeup(Dev_Gt2818s_t *self)
{
    if (self == 0 || self->periph.io_cs == 0) {
        return;
    }
    self->func.cs_set(self, CS_LOW);
    periph_dwt.delay_us(150);
    self->func.cs_set(self, CS_HIGH);
    periph_dwt.delay_us(150);
    self->func.cs_set(self, CS_LOW);
    periph_dwt.delay_us(150);
    self->func.cs_set(self, CS_HIGH);
    periph_dwt.delay_us(150);
}

static void cs_set(Dev_Gt2818s_t *self, uint8_t level)
{
    if (self == 0 || self->periph.io_cs == 0) {
        return;
    }
    self->periph.io_cs->write(level);
}

static uint8_t write_cfg(Dev_Gt2818s_t *self)
{
    uint8_t frameHead[4];
    uint8_t frameInfo[12];
    uint16_t pecCal;
    Cfg_A_Group_Def *cfg_a;
    Cfg_B_Group_Def *cfg_b;

    if ((self == 0) || (self->func.isInitialized == 0) ||
        (self->periph.spi == 0) || (self->periph.io_cs == 0)) {
        return 1;
    }

    cfg_a = &self->reg.cfg_a_set;
    cfg_b = &self->reg.cfg_b_set;

    /* ========== WRCFGA: CMD=0x0001 ========== */
    memset(frameHead, 0x00, sizeof(frameHead));
    frameHead[0] = 0x00;
    frameHead[1] = 0x01;
    pecCal = pec15_cal(2, frameHead);
    frameHead[2] = (uint8_t)((pecCal >> 8) & 0xFFU);
    frameHead[3] = (uint8_t)(pecCal & 0xFFU);

    pecCal = pec15_cal(6, (uint8_t *)cfg_a);
    cfg_a->pec_H = (uint8_t)((pecCal >> 8) & 0xFFU);
    cfg_a->pec_L = (uint8_t)(pecCal & 0xFFU);

    memset(frameInfo, 0x00, sizeof(frameInfo));
    memcpy(frameInfo, frameHead, sizeof(frameHead));
    memcpy(frameInfo + 4, (uint8_t *)cfg_a, 8);

    self->func.wakeup(self);
    self->func.cs_set(self, CS_LOW);
    self->periph.spi->send_array(frameInfo, (uint16_t)sizeof(frameInfo), 1000U);
    self->func.cs_set(self, CS_HIGH);

    /* ========== WRCFGB: CMD=0x0024 ========== */
    memset(frameHead, 0x00, sizeof(frameHead));
    frameHead[0] = 0x00;
    frameHead[1] = 0x24;
    pecCal = pec15_cal(2, frameHead);
    frameHead[2] = (uint8_t)((pecCal >> 8) & 0xFFU);
    frameHead[3] = (uint8_t)(pecCal & 0xFFU);

    pecCal = pec15_cal(6, (uint8_t *)cfg_b);
    cfg_b->pec_H = (uint8_t)((pecCal >> 8) & 0xFFU);
    cfg_b->pec_L = (uint8_t)(pecCal & 0xFFU);

    memset(frameInfo, 0x00, sizeof(frameInfo));
    memcpy(frameInfo, frameHead, sizeof(frameHead));
    memcpy(frameInfo + 4, (uint8_t *)cfg_b, 8);

    self->func.cs_set(self, CS_LOW);
    self->periph.spi->send_array(frameInfo, (uint16_t)sizeof(frameInfo), 1000U);
    self->func.cs_set(self, CS_HIGH);

    return 0;
}

static uint8_t read_cfg(Dev_Gt2818s_t *self)
{
    uint8_t frameHead[4];
    uint8_t rxBuf[8];
    uint16_t pecCal;
    uint16_t pecRx;
    uint8_t ret = 0;
    Cfg_A_Group_Def *cfg_a;
    Cfg_B_Group_Def *cfg_b;

    if ((self == 0) || (self->func.isInitialized == 0) ||
        (self->periph.spi == 0) || (self->periph.io_cs == 0)) {
        return 1;
    }

    cfg_a = &self->reg.cfg_a_read;
    cfg_b = &self->reg.cfg_b_read;

    /* ========== RDCFGA: CMD=0x0002 ========== */
    memset(frameHead, 0x00, sizeof(frameHead));
    frameHead[0] = 0x00;
    frameHead[1] = 0x02;
    pecCal = pec15_cal(2, frameHead);
    frameHead[2] = (uint8_t)((pecCal >> 8) & 0xFFU);
    frameHead[3] = (uint8_t)(pecCal & 0xFFU);

    memset(rxBuf, 0x00, sizeof(rxBuf));
    self->func.wakeup(self);
    self->func.cs_set(self, CS_LOW);
    self->periph.spi->send_array(frameHead, (uint16_t)sizeof(frameHead), 1000U);
    self->periph.spi->read_array(rxBuf, (uint16_t)sizeof(rxBuf), 1000U);
    self->func.cs_set(self, CS_HIGH);

    pecCal = pec15_cal(6, rxBuf);
    pecRx = ((uint16_t)rxBuf[6] << 8) | (uint16_t)rxBuf[7];
    if (pecCal == pecRx) {
        memcpy((uint8_t *)cfg_a, rxBuf, 8);
    } else {
        memset((uint8_t *)cfg_a, 0xFF, 8);
        ret = 2;
    }

    /* ========== RDCFGB: CMD=0x0026 ========== */
    memset(frameHead, 0x00, sizeof(frameHead));
    frameHead[0] = 0x00;
    frameHead[1] = 0x26;
    pecCal = pec15_cal(2, frameHead);
    frameHead[2] = (uint8_t)((pecCal >> 8) & 0xFFU);
    frameHead[3] = (uint8_t)(pecCal & 0xFFU);

    memset(rxBuf, 0x00, sizeof(rxBuf));
    self->func.cs_set(self, CS_LOW);
    self->periph.spi->send_array(frameHead, (uint16_t)sizeof(frameHead), 1000U);
    self->periph.spi->read_array(rxBuf, (uint16_t)sizeof(rxBuf), 1000U);
    self->func.cs_set(self, CS_HIGH);

    pecCal = pec15_cal(6, rxBuf);
    pecRx = ((uint16_t)rxBuf[6] << 8) | (uint16_t)rxBuf[7];
    if (pecCal == pecRx) {
        memcpy((uint8_t *)cfg_b, rxBuf, 8);
    } else {
        memset((uint8_t *)cfg_b, 0xFF, 8);
        if (ret == 0) {
            ret = 3;
        }
    }

    return ret;
}

static uint8_t adcv(Dev_Gt2818s_t *self, uint8_t adc_mode, uint8_t ch)
{
    uint8_t frameHead[4];
    uint16_t pecCal;
    uint16_t adcv_config;

    adcv_config |= (1<<9);
    adcv_config |= (3<<5);
    adcv_config |= (adc_mode<<7);
    adcv_config |= (0<<4);
    adcv_config |= ch;

    memset(frameHead, 0x00, sizeof(frameHead));
    frameHead[0] = (uint8_t)((adcv_config >> 8) & 0xFFU);
    frameHead[1] = (uint8_t)(adcv_config & 0xFFU);
    pecCal = pec15_cal(2, frameHead);
    frameHead[2] = (uint8_t)((pecCal >> 8) & 0xFFU);
    frameHead[3] = (uint8_t)(pecCal & 0xFFU);

    self->func.wakeup(self);
    self->func.cs_set(self, CS_LOW);
    self->periph.spi->send_array(frameHead, (uint16_t)sizeof(frameHead), 1000U);
    self->func.cs_set(self, CS_HIGH);       
    return 0;
}

static const uint16_t vol_regGroup_cmd[NUM_VOL_REGS] = {
    0x0004, /* RDCVA */
    0x0006, /* RDCVB */
    0x0008, /* RDCVC */
    0x000A, /* RDCVD */
    0x0009, /* RDCVE */
    0x000B  /* RDCVF */
};

static uint8_t rdcv(Dev_Gt2818s_t *self)
{
    uint8_t frameHead[4];
    uint8_t rxBuf[8];
    uint16_t pecCal;
    uint16_t pecRx;
    uint8_t i;
    uint8_t ret = 0;
    Cell_Vol_Group_Def *cell_reg;

    if ((self == 0) || (self->func.isInitialized == 0) ||
        (self->periph.spi == 0) || (self->periph.io_cs == 0)) {
        return 1;
    }

    self->func.wakeup(self);

    for (i = 0; i < NUM_VOL_REGS; i++) {
        cell_reg = &self->reg.cell_vol_reg[i];

        memset(frameHead, 0x00, sizeof(frameHead));
        frameHead[0] = (uint8_t)((vol_regGroup_cmd[i] >> 8) & 0xFFU);
        frameHead[1] = (uint8_t)(vol_regGroup_cmd[i] & 0xFFU);
        pecCal = pec15_cal(2, frameHead);
        frameHead[2] = (uint8_t)((pecCal >> 8) & 0xFFU);
        frameHead[3] = (uint8_t)(pecCal & 0xFFU);

        memset(rxBuf, 0x00, sizeof(rxBuf));
        self->func.cs_set(self, CS_LOW);
        self->periph.spi->send_array(frameHead, (uint16_t)sizeof(frameHead), 1000U);
        self->periph.spi->read_array(rxBuf, (uint16_t)sizeof(rxBuf), 1000U);
        self->func.cs_set(self, CS_HIGH);

        pecCal = pec15_cal(6, rxBuf);
        pecRx = ((uint16_t)rxBuf[6] << 8) | (uint16_t)rxBuf[7];
        if (pecCal == pecRx) {
            /* 小端: L + H 组成 16bit 电压 */
            cell_reg->cell_vol[0] = (uint16_t)rxBuf[0] | ((uint16_t)rxBuf[1] << 8);
            cell_reg->cell_vol[1] = (uint16_t)rxBuf[2] | ((uint16_t)rxBuf[3] << 8);
            cell_reg->cell_vol[2] = (uint16_t)rxBuf[4] | ((uint16_t)rxBuf[5] << 8);
            cell_reg->pec_H = rxBuf[6];
            cell_reg->pec_L = rxBuf[7];
        } else {
            memset((uint8_t *)cell_reg, 0xFF, sizeof(Cell_Vol_Group_Def));
            ret = (uint8_t)(10U + i);
        }
    }

    return ret;
}

static uint8_t adgp(Dev_Gt2818s_t *self, uint8_t adc_mode, uint8_t ch)
{
    uint8_t frameHead[4];
    uint16_t pecCal;
    uint16_t adgp_config;

    adgp_config |= (1<<10);
    adgp_config |= (3<<5);
    adgp_config |= (adc_mode<<7);
    adgp_config |= ch;
    return 0;
}

static const uint16_t aux_regGroup_cmd[NUM_AUX_REGS] = {
    0x000C, /* RDAUXA */
    0x000E, /* RDAUXB */
    0x000D, /* RDAUXC */
    0x000F  /* RDAUXD */
};

static uint8_t rdgp(Dev_Gt2818s_t *self)
{
    uint8_t frameHead[4];
    uint8_t rxBuf[8];
    uint16_t pecCal;
    uint16_t pecRx;
    uint8_t i;
    uint8_t ret = 0;
    Aux_Group_Def *aux_reg;

    if ((self == 0) || (self->func.isInitialized == 0) ||
        (self->periph.spi == 0) || (self->periph.io_cs == 0)) {
        return 1;
    }

    self->func.wakeup(self);

    for (i = 0; i < NUM_AUX_REGS; i++) {
        aux_reg = &self->reg.cell_aux_reg[i];

        memset(frameHead, 0x00, sizeof(frameHead));
        frameHead[0] = (uint8_t)((aux_regGroup_cmd[i] >> 8) & 0xFFU);
        frameHead[1] = (uint8_t)(aux_regGroup_cmd[i] & 0xFFU);
        pecCal = pec15_cal(2, frameHead);
        frameHead[2] = (uint8_t)((pecCal >> 8) & 0xFFU);
        frameHead[3] = (uint8_t)(pecCal & 0xFFU);

        memset(rxBuf, 0x00, sizeof(rxBuf));
        self->func.cs_set(self, CS_LOW);
        self->periph.spi->send_array(frameHead, (uint16_t)sizeof(frameHead), 1000U);
        self->periph.spi->read_array(rxBuf, (uint16_t)sizeof(rxBuf), 1000U);
        self->func.cs_set(self, CS_HIGH);

        pecCal = pec15_cal(6, rxBuf);
        pecRx = ((uint16_t)rxBuf[6] << 8) | (uint16_t)rxBuf[7];
        if (pecCal == pecRx) {
            aux_reg->aux[0] = (uint16_t)rxBuf[0] | ((uint16_t)rxBuf[1] << 8);
            aux_reg->aux[1] = (uint16_t)rxBuf[2] | ((uint16_t)rxBuf[3] << 8);
            aux_reg->aux[2] = (uint16_t)rxBuf[4] | ((uint16_t)rxBuf[5] << 8);
            aux_reg->pec_H = rxBuf[6];
            aux_reg->pec_L = rxBuf[7];
        } else {
            memset((uint8_t *)aux_reg, 0xFF, sizeof(Aux_Group_Def));
            ret = (uint8_t)(20U + i);
        }
    }

    return ret;
}

static uint8_t clrcell(Dev_Gt2818s_t *self)
{
    uint8_t frameHead[4];
    uint16_t pecCal;
    uint16_t clrcell_cmd = 0x0711;
    
    memset(frameHead, 0x00, sizeof(frameHead));
    frameHead[0] = (uint8_t)((clrcell_cmd >> 8) & 0xFFU);
    frameHead[1] = (uint8_t)(clrcell_cmd & 0xFFU);
    pecCal = pec15_cal(2, frameHead);
    frameHead[2] = (uint8_t)((pecCal >> 8) & 0xFFU);
    frameHead[3] = (uint8_t)(pecCal & 0xFFU);   

    self->func.wakeup(self);
    self->func.cs_set(self, CS_LOW);
    self->periph.spi->send_array(frameHead, (uint16_t)sizeof(frameHead), 1000U);
    self->func.cs_set(self, CS_HIGH);       
    return 0;
}

static uint8_t clraux(Dev_Gt2818s_t *self)
{
    uint8_t frameHead[4];
    uint16_t pecCal;
    uint16_t clraux_cmd = 0x0712;
    
    memset(frameHead, 0x00, sizeof(frameHead));
    frameHead[0] = (uint8_t)((clraux_cmd >> 8) & 0xFFU);
    frameHead[1] = (uint8_t)(clraux_cmd & 0xFFU);
    pecCal = pec15_cal(2, frameHead);
    frameHead[2] = (uint8_t)((pecCal >> 8) & 0xFFU);
    frameHead[3] = (uint8_t)(pecCal & 0xFFU);   

    self->func.wakeup(self);
    self->func.cs_set(self, CS_LOW);
    self->periph.spi->send_array(frameHead, (uint16_t)sizeof(frameHead), 1000U);
    self->func.cs_set(self, CS_HIGH);       
    return 0;
}

static uint16_t pec15_cal(uint16_t len, uint8_t *pData)
{
    uint16_t remainder;
    uint16_t addr;
    uint16_t i;

    if ((pData == 0) || (len == 0)) {
        return 0;
    }

    remainder = 16U; /* PEC seed */

    for (i = 0; i < len; i++) {
        addr = ((remainder >> 7) ^ pData[i]) & 0xFFU;
        remainder = (uint16_t)((remainder << 8) ^ crc15Table[addr]);
        remainder &= 0x7FFFU;
    }

    return (uint16_t)(remainder << 1); /* 15-bit PEC left-aligned to 16-bit */
}

