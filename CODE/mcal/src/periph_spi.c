/**
 * @file periph_spi.c
 * @brief SPI 外设对象实现 (MCAL 层)
 * @author
 * @date 2026-07-02
 */

#include "periph_spi.h"
#include "gd32f30x_gpio.h"
#include "gd32f30x_spi.h"
#include "gd32f30x_rcu.h"

/* ==================== SPI0 GPIO 引脚映射（项目仅使用 SPI0）==================== */
#define SPI0_SCK_PORT     GPIOA
#define SPI0_SCK_PIN      GPIO_PIN_5
#define SPI0_MISO_PORT    GPIOA
#define SPI0_MISO_PIN     GPIO_PIN_6
#define SPI0_MOSI_PORT    GPIOA
#define SPI0_MOSI_PIN     GPIO_PIN_7

static void periph_spi_init(Periph_Spi_t *self);
static void periph_spi_deinit(Periph_Spi_t *self);
static int8_t periph_spi_send_array(Periph_Spi_t *self, const uint8_t *data, uint16_t size, uint32_t timeout);
static int8_t periph_spi_read_array(Periph_Spi_t *self, uint8_t *data, uint16_t size, uint32_t timeout);

static void spi_clock_enable(uint32_t spi_periph);
static void spi_clock_disable(uint32_t spi_periph);
static void spi_gpio_clock_enable(uint32_t port);
static int8_t spi_wait_flag(uint32_t spi_periph, uint32_t flag, uint32_t *timeout);

/* ==================== 全局 SPI0 对象实例（静态初始化） ==================== */
Periph_Spi_t periph_spi0 = {
    .spi_periph          = SPI0,
    .baudrate_prescaler  = SPI_PSC_8,
    .clock_polarity      = SPI_CK_PL_HIGH_PH_2EDGE,
    .frame_size          = SPI_FRAMESIZE_8BIT,
    .direction           = SPI_TRANSMODE_FULLDUPLEX,
    .physical_pins = {
        .sck  = {SPI0_SCK_PORT,  SPI0_SCK_PIN},
        .miso = {SPI0_MISO_PORT, SPI0_MISO_PIN},
        .mosi = {SPI0_MOSI_PORT, SPI0_MOSI_PIN},
    },
    .state               = 0,
    .error_code          = 0,
    .init                = periph_spi_init,
    .deinit              = periph_spi_deinit,
    .send_array          = periph_spi_send_array,
    .read_array          = periph_spi_read_array
};

static void spi_clock_enable(uint32_t spi_periph)
{
    if (spi_periph == SPI0) {
        rcu_periph_clock_enable(RCU_SPI0);
    } else if (spi_periph == SPI1) {
        rcu_periph_clock_enable(RCU_SPI1);
    } else if (spi_periph == SPI2) {
        rcu_periph_clock_enable(RCU_SPI2);
    }
}

static void spi_clock_disable(uint32_t spi_periph)
{
    if (spi_periph == SPI0) {
        rcu_periph_clock_disable(RCU_SPI0);
    } else if (spi_periph == SPI1) {
        rcu_periph_clock_disable(RCU_SPI1);
    } else if (spi_periph == SPI2) {
        rcu_periph_clock_disable(RCU_SPI2);
    }
}

static void spi_gpio_clock_enable(uint32_t port)
{
    if (port == GPIOA) {
        rcu_periph_clock_enable(RCU_GPIOA);
    } else if (port == GPIOB) {
        rcu_periph_clock_enable(RCU_GPIOB);
    } else if (port == GPIOC) {
        rcu_periph_clock_enable(RCU_GPIOC);
    }
}

static int8_t spi_wait_flag(uint32_t spi_periph, uint32_t flag, uint32_t *timeout)
{
    while (RESET == spi_i2s_flag_get(spi_periph, flag)) {
        if (*timeout == 0U) {
            return -1;
        }
        (*timeout)--;
    }
    return 1;
}

static void periph_spi_init(Periph_Spi_t *self)
{
    spi_parameter_struct spi_parameter;

    spi_clock_enable(self->spi_periph);
    spi_gpio_clock_enable(self->physical_pins.sck.port);
    spi_gpio_clock_enable(self->physical_pins.miso.port);
    spi_gpio_clock_enable(self->physical_pins.mosi.port);

    gpio_init(self->physical_pins.sck.port, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, self->physical_pins.sck.pin);
    gpio_init(self->physical_pins.mosi.port, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, self->physical_pins.mosi.pin);
    gpio_init(self->physical_pins.miso.port, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, self->physical_pins.miso.pin);

    spi_struct_para_init(&spi_parameter);
    spi_parameter.device_mode          = SPI_MASTER;
    spi_parameter.trans_mode           = self->direction;
    spi_parameter.frame_size           = self->frame_size;
    spi_parameter.clock_polarity_phase = self->clock_polarity;
    spi_parameter.nss                  = SPI_NSS_SOFT;
    spi_parameter.prescale             = self->baudrate_prescaler;
    spi_parameter.endian               = SPI_ENDIAN_MSB;

    spi_init(self->spi_periph, &spi_parameter);
    spi_enable(self->spi_periph);
}

static void periph_spi_deinit(Periph_Spi_t *self)
{
    spi_disable(self->spi_periph);
    spi_clock_disable(self->spi_periph);
}

static int8_t periph_spi_send_array(Periph_Spi_t *self, const uint8_t *data, uint16_t size, uint32_t timeout)
{
    uint16_t i;

    if ((self == 0) || (data == 0) || (size == 0U) || (timeout == 0U)) {
        return 0;
    }

    for (i = 0; i < size; i++) {
        if (spi_wait_flag(self->spi_periph, SPI_FLAG_TBE, &timeout) != 1) {
            return -1;
        }
        spi_i2s_data_transmit(self->spi_periph, data[i]);
        if (spi_wait_flag(self->spi_periph, SPI_FLAG_RBNE, &timeout) != 1) {
            return -1;
        }
        (void)spi_i2s_data_receive(self->spi_periph);
    }
    return 1;
}

static int8_t periph_spi_read_array(Periph_Spi_t *self, uint8_t *data, uint16_t size, uint32_t timeout)
{
    uint16_t i;

    if ((self == 0) || (data == 0) || (size == 0U) || (timeout == 0U)) {
        return 0;
    }

    for (i = 0; i < size; i++) {
        if (spi_wait_flag(self->spi_periph, SPI_FLAG_TBE, &timeout) != 1) {
            return -1;
        }
        spi_i2s_data_transmit(self->spi_periph, 0xFF);
        if (spi_wait_flag(self->spi_periph, SPI_FLAG_RBNE, &timeout) != 1) {
            return -1;
        }
        data[i] = (uint8_t)spi_i2s_data_receive(self->spi_periph);
    }
    return 1;
}
