/*
 * Author: Javier Arteaga <javier@emutex.com>
 * Based on work from: Dan O'Donovan <dan@emutex.com>
 *                     Nicola Lunghi <nicola.lunghi@emutex.com>
 * Copyright (c) 2017 Emutex Ltd.
 * Copyright (c) 2014 Intel Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <sys/file.h>
#include <sys/mman.h>

#include "common.h"
#include "gpio.h"
#include "x86/up2.h"

#define PLATFORM_NAME "UP2"

#define MRAA_UP2_GPIOCOUNT   28
#define MRAA_UP2_REGSIZE     72
#define MRAA_UP2_I2C0_VLS_OE 60
#define MRAA_UP2_I2C1_VLS_OE 61

#define MRAA_UP2_NORTH_BASE     434
#define MRAA_UP2_NORTHWEST_BASE 357
#define MRAA_UP2_WEST_BASE      310
#define MRAA_UP2_SOUTHWEST_BASE 267

#define MRAA_UP2_FPGA_ENABLE    (MRAA_UP2_NORTH_BASE + 71)
#define MRAA_UP2_FPGA_RESET     (MRAA_UP2_NORTH_BASE + 72)
#define MRAA_UP2_FPGA_DATAOUT   (MRAA_UP2_SOUTHWEST_BASE + 16)
#define MRAA_UP2_FPGA_DATAIN    (MRAA_UP2_SOUTHWEST_BASE + 17)
#define MRAA_UP2_FPGA_STROBE    (MRAA_UP2_SOUTHWEST_BASE + 18)
#define MRAA_UP2_FPGA_CLEAR     (MRAA_UP2_SOUTHWEST_BASE + 19)

typedef enum {
    MRAA_UP2_FPGA_NONE = -1,
    MRAA_UP2_FPGA_SOC_TO_HAT = 0,
    MRAA_UP2_FPGA_HAT_TO_SOC = 1
} mraa_up2_fpga_dir_t;

typedef struct {
    int offset;
    int initdir;
} mraa_up2_fpga_pininfo_t;

static const mraa_up2_fpga_pininfo_t mraa_up2_fpga_pininfo[] = {
    [0]  = {-1, MRAA_UP2_FPGA_NONE},
    [1]  = {-1, MRAA_UP2_FPGA_NONE},
    [2]  = {-1, MRAA_UP2_FPGA_NONE},
    [3]  = {23, MRAA_UP2_FPGA_HAT_TO_SOC}, /*I2C1_SDA*/
    [4]  = {-1, MRAA_UP2_FPGA_NONE},
    [5]  = {22, MRAA_UP2_FPGA_HAT_TO_SOC}, /*I2C1_SCL*/
    [6]  = {-1, MRAA_UP2_FPGA_NONE},
    [7]  = { 4, MRAA_UP2_FPGA_HAT_TO_SOC}, /*ADC0*/
    [8]  = { 0, MRAA_UP2_FPGA_HAT_TO_SOC}, /*UART1_TX*/
    [9]  = {-1, MRAA_UP2_FPGA_NONE},
    [10] = { 1, MRAA_UP2_FPGA_HAT_TO_SOC}, /*UART1_RX*/
    [11] = { 2, MRAA_UP2_FPGA_HAT_TO_SOC}, /*UART1_RTS / ADC1*/
    [12] = {41, MRAA_UP2_FPGA_HAT_TO_SOC}, /*I2S_CLK*/
    [13] = { 5, MRAA_UP2_FPGA_HAT_TO_SOC}, /*ADC2*/
    [14] = {-1, MRAA_UP2_FPGA_NONE},
    [15] = { 6, MRAA_UP2_FPGA_HAT_TO_SOC}, /*ADC3*/
    [16] = {12, MRAA_UP2_FPGA_HAT_TO_SOC}, /*SPI1_CS1*/
    [17] = {-1, MRAA_UP2_FPGA_NONE},
    [18] = {11, MRAA_UP2_FPGA_HAT_TO_SOC}, /*SPI1_MISO*/
    [19] = {15, MRAA_UP2_FPGA_HAT_TO_SOC}, /*SPI0_MOSI*/
    [21] = {16, MRAA_UP2_FPGA_HAT_TO_SOC}, /*SPI0_MISO*/
    [22] = {10, MRAA_UP2_FPGA_HAT_TO_SOC}, /*SPI1_MOSI*/
    [23] = {19, MRAA_UP2_FPGA_HAT_TO_SOC}, /*SPI0_CLK*/
    [24] = {18, MRAA_UP2_FPGA_HAT_TO_SOC}, /*SPI0_CS0*/
    [25] = {-1, MRAA_UP2_FPGA_NONE},
    [26] = {17, MRAA_UP2_FPGA_HAT_TO_SOC}, /*SPI0_CS1*/
    [27] = {21, MRAA_UP2_FPGA_HAT_TO_SOC}, /*I2C0_SDA*/
    [28] = {20, MRAA_UP2_FPGA_HAT_TO_SOC}, /*I2C0_SCL*/
    [29] = {7,  MRAA_UP2_FPGA_HAT_TO_SOC}, /*GPIO5*/
    [30] = {-1, MRAA_UP2_FPGA_NONE},
    [31] = {14, MRAA_UP2_FPGA_HAT_TO_SOC}, /*SPI1_CLK*/
    [32] = {25, MRAA_UP2_FPGA_HAT_TO_SOC}, /*PWM0*/
    [33] = {24, MRAA_UP2_FPGA_HAT_TO_SOC}, /*PWM1*/
    [34] = {-1, MRAA_UP2_FPGA_NONE},
    [35] = {40, MRAA_UP2_FPGA_HAT_TO_SOC}, /*I2S_FRM*/
    [36] = {3,  MRAA_UP2_FPGA_HAT_TO_SOC}, /*UART1_CTS*/
    [37] = {13, MRAA_UP2_FPGA_HAT_TO_SOC}, /*GPIO26*/
    [38] = {39, MRAA_UP2_FPGA_HAT_TO_SOC}, /*I2S_DIN*/
    [39] = {-1, MRAA_UP2_FPGA_NONE},
    [40] = {38, MRAA_UP2_FPGA_HAT_TO_SOC}, /*I2S_DOUT*/
};

// shared cache of FPGA output configuration register
static unsigned __int128* shmreg = NULL;
static int lockfd;

static mraa_gpio_context clear;
static mraa_gpio_context strobe;
static mraa_gpio_context datain;
static mraa_gpio_context dataout;

// utility function to setup pin mapping of boards
static mraa_result_t
mraa_up2_set_pininfo(mraa_board_t* board, int mraa_index, char* name,
        mraa_pincapabilities_t caps, int sysfs_pin)
{
    if (mraa_index < board->phy_pin_count) {
        mraa_pininfo_t* pin_info = &board->pins[mraa_index];
        strncpy(pin_info->name, name, MRAA_PIN_NAME_SIZE);
        pin_info->capabilities = caps;
        if (caps.gpio) {
            pin_info->gpio.pinmap = sysfs_pin;
            pin_info->gpio.mux_total = 0;
        }
        if (caps.pwm) {
            int controller = 0;
            pin_info->pwm.parent_id = (unsigned int) controller;
            pin_info->pwm.pinmap = 0;
            pin_info->pwm.mux_total = 0;
        }
        if (caps.aio) {
            pin_info->aio.mux_total = 0;
            pin_info->aio.pinmap = 0;
        }
        if (caps.i2c) {
            pin_info->i2c.pinmap = 1;
            pin_info->i2c.mux_total = 0;
        }
        if (caps.spi) {
            pin_info->spi.mux_total = 0;
        }
        if (caps.uart) {
            pin_info->uart.mux_total = 0;
        }
        return MRAA_SUCCESS;
    }
    return MRAA_ERROR_INVALID_RESOURCE;
}

static mraa_result_t
mraa_up2_get_pin_index(mraa_board_t* board, char* name, int* pin_index)
{
    int i;
    for (i = 0; i < board->phy_pin_count; ++i) {
        if (strncmp(name, board->pins[i].name, MRAA_PIN_NAME_SIZE) == 0) {
            *pin_index = i;
            return MRAA_SUCCESS;
        }
    }
    return MRAA_ERROR_INVALID_RESOURCE;
}

static mraa_result_t
mraa_up2_get_confreg_offset(int pin, int* offset)
{
    if (pin < 1 || pin > MRAA_UP2_PINCOUNT)
        return MRAA_ERROR_INVALID_PARAMETER;

    *offset = mraa_up2_fpga_pininfo[pin].offset;
    return MRAA_SUCCESS;
}

static mraa_result_t
mraa_up2_confreg_get(unsigned __int128 **reg, int* already_configured)
{
    *already_configured = 0;

    int fd = shm_open("/upfpga_confreg", O_RDWR | O_CREAT | O_EXCL, 0600);
    if (fd == -1 && errno == EEXIST) {
        *already_configured = 1;
        fd = shm_open("/upfpga_confreg", O_RDWR, 0600);
    }
    if (fd == -1) return MRAA_ERROR_NO_RESOURCES;

    size_t len = sizeof(**reg);
    int ret = ftruncate(fd, len);
    if (ret == -1) return MRAA_ERROR_NO_RESOURCES;

    void *mem = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (mem == MAP_FAILED) return MRAA_ERROR_NO_RESOURCES;

    *reg = mem;
    lockfd = fd;

    return MRAA_SUCCESS;
}

static mraa_result_t
mraa_up2_confreg_write(unsigned __int128 newreg)
{
    mraa_result_t ret;

    if (!shmreg) {
        return MRAA_ERROR_PLATFORM_NOT_INITIALISED;
    }

    if (newreg >= (unsigned __int128) 1 << MRAA_UP2_REGSIZE) {
        return MRAA_ERROR_INVALID_PARAMETER;
    }

    if (newreg == *shmreg) {
        return MRAA_SUCCESS;
    }

    // clear FPGA internal counters
    ret = mraa_gpio_write(clear, 0);
    if (ret != MRAA_SUCCESS) return ret;

    ret = mraa_gpio_write(clear, 1);
    if (ret != MRAA_SUCCESS) return ret;

    // bit-bang the new configuration register
    for (int i = MRAA_UP2_REGSIZE - 1; i >= 0; i--) {
        int bit = (newreg >> i) & 0x1;

        ret = mraa_gpio_write(strobe, 0);
        if (ret != MRAA_SUCCESS) return ret;

        ret = mraa_gpio_write(datain, bit);
        if (ret != MRAA_SUCCESS) return ret;

        ret = mraa_gpio_write(strobe, 1);
        if (ret != MRAA_SUCCESS) return ret;
    }

    // read back the register from the FPGA
    unsigned __int128 verify = 0;
    for (int i = MRAA_UP2_REGSIZE - 1; i >= 0; i--) {
        ret = mraa_gpio_write(strobe, 0);
        if (ret != MRAA_SUCCESS) return ret;

        ret = mraa_gpio_write(strobe, 1);
        if (ret != MRAA_SUCCESS) return ret;

        int bit = mraa_gpio_read(dataout);
        if (bit == -1) return MRAA_ERROR_UNSPECIFIED;

        verify |= (unsigned __int128) bit << i;
    }

    if (newreg != verify) {
        return MRAA_ERROR_UNSPECIFIED;
    }

    *shmreg = newreg;

    // send one last strobe cycle to acknowledge read-back
    ret = mraa_gpio_write(strobe, 0);
    if (ret != MRAA_SUCCESS) return ret;

    ret = mraa_gpio_write(strobe, 1);
    if (ret != MRAA_SUCCESS) return ret;

    return MRAA_SUCCESS;
}

static mraa_result_t
mraa_up2_init_pin(int sysfspin, mraa_gpio_dir_t dir, mraa_gpio_context* dev)
{
    mraa_gpio_context d = mraa_gpio_init_raw(sysfspin);
    if (!d) return MRAA_ERROR_NO_RESOURCES;

    *dev = d;

    return mraa_gpio_dir(d, dir);
}

static mraa_result_t
mraa_up2_fpga_init(void)
{
    mraa_result_t ret;
    int already_configured;
    int iret;

    // initialise the FPGA comms lines
    ret = mraa_up2_init_pin(MRAA_UP2_FPGA_CLEAR, MRAA_GPIO_OUT_HIGH, &clear);
    if (ret != MRAA_SUCCESS) goto unlock;

    ret = mraa_up2_init_pin(MRAA_UP2_FPGA_DATAOUT, MRAA_GPIO_IN, &dataout);
    if (ret != MRAA_SUCCESS) goto unlock;

    ret = mraa_up2_init_pin(MRAA_UP2_FPGA_DATAIN, MRAA_GPIO_OUT_LOW, &datain);
    if (ret != MRAA_SUCCESS) goto unlock;

    ret = mraa_up2_init_pin(MRAA_UP2_FPGA_STROBE, MRAA_GPIO_OUT_LOW, &strobe);
    if (ret != MRAA_SUCCESS) goto unlock;

    iret = flock(lockfd, LOCK_EX);
    if (iret == -1) return MRAA_ERROR_INVALID_RESOURCE;

    // we can stop here if the FPGA has been configured before
    ret = mraa_up2_confreg_get(&shmreg, &already_configured);
    if (ret != MRAA_SUCCESS || already_configured) goto unlock;

    unsigned __int128 initreg = 0;
    for (size_t i = 1; i < MRAA_UP2_PINCOUNT; i++) {
        const mraa_up2_fpga_pininfo_t pininfo = mraa_up2_fpga_pininfo[i];
        if (pininfo.offset < 0) continue;
        if (pininfo.initdir == MRAA_UP2_FPGA_HAT_TO_SOC) {
            initreg |= (unsigned __int128) 1 << pininfo.offset;
        }
    }

    // disable FPGA outputs
    mraa_gpio_context enable;
    ret = mraa_up2_init_pin(MRAA_UP2_FPGA_ENABLE, MRAA_GPIO_OUT_LOW, &enable);
    if (ret != MRAA_SUCCESS) goto unlock;

    // ensure FPGA is brought out of reset
    mraa_gpio_context reset;
    ret = mraa_up2_init_pin(MRAA_UP2_FPGA_RESET, MRAA_GPIO_OUT_HIGH, &reset);
    if (ret != MRAA_SUCCESS) goto unlock;

    // apply configuration
    ret = mraa_up2_confreg_write(initreg);
    if (ret != MRAA_SUCCESS) goto unlock;

    // re-enable outputs
    ret = mraa_gpio_write(enable, 1);
    if (ret != MRAA_SUCCESS) goto unlock;

unlock:
    iret = flock(lockfd, LOCK_UN);
    if (iret == -1) return MRAA_ERROR_INVALID_RESOURCE;

    return ret;
}

static mraa_result_t
mraa_up2_pins_to_confreg_bits(const int* pins, int* bits, size_t npins)
{
    for (size_t i = 0; i < npins; i++) {
        mraa_result_t ret = mraa_up2_get_confreg_offset(pins[i], &bits[i]);
        if (ret != MRAA_SUCCESS) return ret;
    }

    return MRAA_SUCCESS;
}

static mraa_result_t
mraa_up2_confreg_set_bits(const int* bits, const int* values, size_t nbits)
{
    mraa_result_t ret;
    int iret;

    unsigned __int128 newreg = *shmreg;

    iret = flock(lockfd, LOCK_EX);
    if (iret == -1) return MRAA_ERROR_INVALID_RESOURCE;

    for (size_t i = 0; i < nbits; i++) {
        if (values[i]) {
            newreg |= (unsigned __int128) 1 << bits[i];
        } else {
            newreg &= ~((unsigned __int128) 1 << bits[i]);
        }
    }

    ret = mraa_up2_confreg_write(newreg);
    if (ret != MRAA_SUCCESS) goto unlock;

unlock:
    iret = flock(lockfd, LOCK_UN);
    if (iret == -1) return MRAA_ERROR_INVALID_RESOURCE;

    return ret;
}

static mraa_result_t
mraa_up2_confreg_set_pins(const int* pins, const int* values, size_t npins)
{
    int bits[npins];
    mraa_result_t ret = mraa_up2_pins_to_confreg_bits(pins, bits, npins);
    if (ret != MRAA_SUCCESS) return ret;

    return mraa_up2_confreg_set_bits(bits, values, npins);
}

static mraa_result_t
mraa_up2_gpio_dir_pre(mraa_gpio_context dev, mraa_gpio_dir_t dir)
{
    mraa_result_t ret;
    const int pin = mraa_gpio_get_pin(dev);
    size_t nbits = 1;

    // switching direction on a GPIO could involve up to 2 confreg bits
    // regular pins:    flip pin direction
    // i2c-shared pins: flip pin direction + disable vls_oe
    int bits[2];
    int values[2] = {
        (dir == MRAA_GPIO_IN) ?
            MRAA_UP2_FPGA_HAT_TO_SOC : MRAA_UP2_FPGA_SOC_TO_HAT
    };

    // ignore pins without a FPGA mapping
    ret = mraa_up2_pins_to_confreg_bits(&pin, bits, 1);
    if (ret != MRAA_SUCCESS) return MRAA_SUCCESS;

    int vls = -1;
    if (pin == plat->i2c_bus[0].sda || pin == plat->i2c_bus[0].scl)
        vls = MRAA_UP2_I2C0_VLS_OE;
    if (pin == plat->i2c_bus[1].sda || pin == plat->i2c_bus[1].scl)
        vls = MRAA_UP2_I2C1_VLS_OE;
    if (vls >= 0) {
        // 0 = disable I2C level shifter
        values[1] = 0;
        bits[1] = vls;
        nbits = 2;
    }

    return mraa_up2_confreg_set_bits(bits, values, nbits);
}

static mraa_result_t
mraa_up2_i2c_init_post(mraa_i2c_context dev)
{
    // ignore I2Cs without a FPGA mapping
    if (dev->busnum != plat->i2c_bus[0].bus_id &&
        dev->busnum != plat->i2c_bus[1].bus_id)
            return MRAA_SUCCESS;

    // 1 = enable I2C level shifter
    const int value = 1;

    int bit;
    if (dev->busnum == plat->i2c_bus[0].bus_id) {
        bit = MRAA_UP2_I2C1_VLS_OE;
    } else {
        bit = MRAA_UP2_I2C0_VLS_OE;
    }

    return mraa_up2_confreg_set_bits(&bit, &value, 1);
}

static mraa_result_t
mraa_up2_aio_init_post(mraa_aio_context dev)
{
    int pin;
    const int fpgadir = MRAA_UP2_FPGA_HAT_TO_SOC;

    if (dev->channel == 0)
        mraa_up2_get_pin_index(plat, "ADC0", &pin);
    else if (dev->channel == 1)
        mraa_up2_get_pin_index(plat, "UART1_RTS", &pin);
    else if (dev->channel == 2)
        mraa_up2_get_pin_index(plat, "ADC2", &pin);
    else if (dev->channel == 3)
        mraa_up2_get_pin_index(plat, "ADC3", &pin);
    else
        return MRAA_SUCCESS;

    return mraa_up2_confreg_set_pins(&pin, &fpgadir, 1);
}

static mraa_result_t
mraa_up2_pwm_init_pre(int pin)
{
    const int fpgadir = MRAA_UP2_FPGA_SOC_TO_HAT;

    return mraa_up2_confreg_set_pins(&pin, &fpgadir, 1);
}

static mraa_result_t
mraa_up2_spi_init_pre(int bus)
{
    if (bus < 0 || bus >= plat->spi_bus_count)
        return MRAA_SUCCESS;

    const int fpgadirs[] = {
        MRAA_UP2_FPGA_SOC_TO_HAT,
        MRAA_UP2_FPGA_SOC_TO_HAT,
        MRAA_UP2_FPGA_HAT_TO_SOC,
        MRAA_UP2_FPGA_SOC_TO_HAT,
    };

    const int pins[] = {
        plat->spi_bus[bus].cs,
        plat->spi_bus[bus].mosi,
        plat->spi_bus[bus].miso,
        plat->spi_bus[bus].sclk,
    };

    return mraa_up2_confreg_set_pins(pins, fpgadirs, 4);
}

static mraa_result_t
mraa_up2_uart_init_post(mraa_uart_context dev)
{
    if (dev->index < 0 || dev->index >= plat->uart_dev_count)
        return MRAA_SUCCESS;

    const int fpgadirs[] = {
        MRAA_UP2_FPGA_HAT_TO_SOC,
        MRAA_UP2_FPGA_SOC_TO_HAT,
    };

    const int pins[] = {
        plat->uart_dev[dev->index].rx,
        plat->uart_dev[dev->index].tx,
    };

    return mraa_up2_confreg_set_pins(pins, fpgadirs, 2);
}

static mraa_result_t
mraa_up2_uart_set_flowcontrol_replace(mraa_uart_context dev,
        mraa_boolean_t xonxoff, mraa_boolean_t rtscts)
{
    if (dev->index < 0 || dev->index >= plat->uart_dev_count)
        return MRAA_SUCCESS;

    // temporarily uninstall this hook to call original uart_set_flowcontrol
    mraa_result_t (*func) (mraa_uart_context, mraa_boolean_t, mraa_boolean_t) =
        dev->advance_func->uart_set_flowcontrol_replace;

    dev->advance_func->uart_set_flowcontrol_replace = NULL;
    mraa_result_t ret = mraa_uart_set_flowcontrol(dev, xonxoff, rtscts);
    dev->advance_func->uart_set_flowcontrol_replace = func;
    if (ret != MRAA_SUCCESS) return ret;

    if (!rtscts) return MRAA_SUCCESS;

    const int fpgadirs[] = {
        MRAA_UP2_FPGA_HAT_TO_SOC,
        MRAA_UP2_FPGA_SOC_TO_HAT,
    };

    const int pins[] = {
        plat->uart_dev[dev->index].cts,
        plat->uart_dev[dev->index].rts,
    };

    return mraa_up2_confreg_set_pins(pins, fpgadirs, 2);
}

mraa_board_t*
mraa_up2_board()
{
    mraa_board_t* b = (mraa_board_t*) calloc(1, sizeof (mraa_board_t));

    if (b == NULL) {
        return NULL;
    }

    b->platform_name = PLATFORM_NAME;
    b->phy_pin_count = MRAA_UP2_PINCOUNT;
    b->gpio_count = MRAA_UP2_GPIOCOUNT;

    b->pins = (mraa_pininfo_t*) malloc(sizeof(mraa_pininfo_t) * MRAA_UP2_PINCOUNT);
    if (b->pins == NULL) {
        goto error;
    }

    b->adv_func = (mraa_adv_func_t *) calloc(1, sizeof (mraa_adv_func_t));
    if (b->adv_func == NULL) {
        free(b->pins);
        goto error;
    }

    // switch direction in the FPGA when switching dirs/modes for the SOC
    b->adv_func->gpio_dir_pre = &mraa_up2_gpio_dir_pre;
    b->adv_func->i2c_init_post = &mraa_up2_i2c_init_post;
    b->adv_func->aio_init_post = &mraa_up2_aio_init_post;
    b->adv_func->pwm_init_pre = &mraa_up2_pwm_init_pre;
    b->adv_func->spi_init_pre = &mraa_up2_spi_init_pre;
    b->adv_func->uart_init_post = &mraa_up2_uart_init_post;
    b->adv_func->uart_set_flowcontrol_replace = &mraa_up2_uart_set_flowcontrol_replace;

    mraa_up2_set_pininfo(b, 0, "INVALID",    (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, -1);
    mraa_up2_set_pininfo(b, 1, "3.3v",       (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, -1);
    mraa_up2_set_pininfo(b, 2, "5v",         (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, -1);
    mraa_up2_set_pininfo(b, 3, "I2C1_SDA",   (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 1, 0, 0}, MRAA_UP2_WEST_BASE + 2);
    mraa_up2_set_pininfo(b, 4, "5v",         (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, -1);
    mraa_up2_set_pininfo(b, 5, "I2C1_SCL",   (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 1, 0, 0}, MRAA_UP2_WEST_BASE + 3);
    mraa_up2_set_pininfo(b, 6, "GND",        (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, -1);
    mraa_up2_set_pininfo(b, 7, "ADC0",       (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 1, 0}, MRAA_UP2_NORTHWEST_BASE + 76);
    mraa_up2_set_pininfo(b, 8, "UART1_TX",   (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 1}, MRAA_UP2_NORTH_BASE + 43);
    mraa_up2_set_pininfo(b, 9, "GND",        (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, -1);
    mraa_up2_set_pininfo(b, 10, "UART1_RX",  (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 1}, MRAA_UP2_NORTH_BASE + 42);
    mraa_up2_set_pininfo(b, 11, "UART1_RTS", (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 1, 1}, MRAA_UP2_NORTH_BASE + 44); // also ADC1
    mraa_up2_set_pininfo(b, 12, "I2S_CLK",   (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, MRAA_UP2_WEST_BASE + 16);
    mraa_up2_set_pininfo(b, 13, "ADC2",      (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 1, 0}, MRAA_UP2_NORTHWEST_BASE + 75);
    mraa_up2_set_pininfo(b, 14, "GND",       (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, -1);
    mraa_up2_set_pininfo(b, 15, "ADC3",      (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 1, 0}, MRAA_UP2_NORTHWEST_BASE + 74);
    mraa_up2_set_pininfo(b, 16, "GPIO23",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, MRAA_UP2_NORTH_BASE + 37);
    mraa_up2_set_pininfo(b, 17, "3.3v",      (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, -1);
    mraa_up2_set_pininfo(b, 18, "GPIO24",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, MRAA_UP2_NORTHWEST_BASE + 48);
    mraa_up2_set_pininfo(b, 19, "SPI0_MOSI", (mraa_pincapabilities_t) {1, 1, 0, 0, 1, 0, 0, 0}, MRAA_UP2_NORTHWEST_BASE + 65);
    mraa_up2_set_pininfo(b, 20, "GND",       (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, -1);
    mraa_up2_set_pininfo(b, 21, "SPI0_MISO", (mraa_pincapabilities_t) {1, 1, 0, 0, 1, 0, 0, 0}, MRAA_UP2_NORTHWEST_BASE + 64);
    mraa_up2_set_pininfo(b, 22, "GPIO25",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, MRAA_UP2_NORTHWEST_BASE + 45);
    mraa_up2_set_pininfo(b, 23, "SPI0_CLK",  (mraa_pincapabilities_t) {1, 1, 0, 0, 1, 0, 0, 0}, MRAA_UP2_NORTHWEST_BASE + 61);
    mraa_up2_set_pininfo(b, 24, "SPI0_CS0",  (mraa_pincapabilities_t) {1, 1, 0, 0, 1, 0, 0, 0}, MRAA_UP2_NORTHWEST_BASE + 62);
    mraa_up2_set_pininfo(b, 25, "GND",       (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, -1);
    mraa_up2_set_pininfo(b, 26, "SPI0_CS1",  (mraa_pincapabilities_t) {1, 1, 0, 0, 1, 0, 0, 0}, MRAA_UP2_NORTHWEST_BASE + 63);
    mraa_up2_set_pininfo(b, 27, "I2C0_SDA",  (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 1, 0, 0}, MRAA_UP2_WEST_BASE + 0);
    mraa_up2_set_pininfo(b, 28, "I2C0_SCL",  (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 1, 0, 0}, MRAA_UP2_WEST_BASE + 1);
    mraa_up2_set_pininfo(b, 29, "GPIO5",     (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, MRAA_UP2_NORTHWEST_BASE + 73);
    mraa_up2_set_pininfo(b, 30, "GND",       (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, -1);
    mraa_up2_set_pininfo(b, 31, "GPIO6",     (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, MRAA_UP2_NORTHWEST_BASE + 47);
    mraa_up2_set_pininfo(b, 32, "PWM0",      (mraa_pincapabilities_t) {1, 1, 1, 0, 0, 0, 0, 0}, MRAA_UP2_NORTH_BASE + 34);
    mraa_up2_set_pininfo(b, 33, "PWM1",      (mraa_pincapabilities_t) {1, 1, 1, 0, 0, 0, 0, 0}, MRAA_UP2_NORTH_BASE + 35);
    mraa_up2_set_pininfo(b, 34, "GND",       (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, -1);
    mraa_up2_set_pininfo(b, 35, "I2S_FRM",   (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, MRAA_UP2_WEST_BASE + 17);
    mraa_up2_set_pininfo(b, 36, "UART1_CTS", (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 1}, MRAA_UP2_NORTH_BASE + 45);
    mraa_up2_set_pininfo(b, 37, "GPIO26",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, MRAA_UP2_NORTHWEST_BASE + 46);
    mraa_up2_set_pininfo(b, 38, "I2S_DIN",   (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, MRAA_UP2_WEST_BASE + 18);
    mraa_up2_set_pininfo(b, 39, "GND",       (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, -1);
    mraa_up2_set_pininfo(b, 40, "I2S_DOUT",  (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, MRAA_UP2_WEST_BASE + 19);

    b->i2c_bus_count = 0;
    b->def_i2c_bus = 0;
    int i2c_bus_num;

    // Configure I2C adaptor #0 (default)
    // (For consistency with Raspberry Pi 2, use I2C1 as our primary I2C bus)
    i2c_bus_num = mraa_find_i2c_bus_pci("0000:00", "0000:00:16.1", "i2c_designware.1");
    if (i2c_bus_num != -1) {
        int i = b->i2c_bus_count;
        b->i2c_bus[i].bus_id = i2c_bus_num;
        mraa_up2_get_pin_index(b, "I2C1_SDA", &(b->i2c_bus[i].sda));
        mraa_up2_get_pin_index(b, "I2C1_SCL", &(b->i2c_bus[i].scl));
        b->i2c_bus_count++;
    }

    // Configure I2C adaptor #1
    // (normally reserved for accessing HAT EEPROM)
    i2c_bus_num = mraa_find_i2c_bus_pci("0000:00", "0000:00:16.0", "i2c_designware.0");
    if (i2c_bus_num != -1) {
        int i = b->i2c_bus_count;
        b->i2c_bus[i].bus_id = i2c_bus_num;
        mraa_up2_get_pin_index(b, "I2C0_SDA", &(b->i2c_bus[i].sda));
        mraa_up2_get_pin_index(b, "I2C0_SCL", &(b->i2c_bus[i].scl));
        b->i2c_bus_count++;
    }

    // Configure PWM
    b->pwm_default_period = 5000;
    b->pwm_max_period = 218453;
    b->pwm_min_period = 1;

    b->spi_bus_count = 0;
    b->def_spi_bus = 0;

    // Configure SPI #0 CS0 (default)
    b->spi_bus[0].bus_id = 1;
    b->spi_bus[0].slave_s = 0;
    mraa_up2_get_pin_index(b, "SPI0_CS0",  &(b->spi_bus[0].cs));
    mraa_up2_get_pin_index(b, "SPI0_MOSI", &(b->spi_bus[0].mosi));
    mraa_up2_get_pin_index(b, "SPI0_MISO", &(b->spi_bus[0].miso));
    mraa_up2_get_pin_index(b, "SPI0_CLK",  &(b->spi_bus[0].sclk));
    b->spi_bus_count++;

    // Configure SPI #0 CS1
    b->spi_bus[1].bus_id = 1;
    b->spi_bus[1].slave_s = 1;
    mraa_up2_get_pin_index(b, "SPI0_CS1",  &(b->spi_bus[1].cs));
    mraa_up2_get_pin_index(b, "SPI0_MOSI", &(b->spi_bus[1].mosi));
    mraa_up2_get_pin_index(b, "SPI0_MISO", &(b->spi_bus[1].miso));
    mraa_up2_get_pin_index(b, "SPI0_CLK",  &(b->spi_bus[1].sclk));
    b->spi_bus_count++;

    b->uart_dev_count = 0;
    b->def_uart_dev = 0;
    b->uart_dev[0].device_path = "/dev/ttyS4";

    // Configure UART #1 (default)
    mraa_up2_get_pin_index(b, "UART1_RX", &(b->uart_dev[0].rx));
    mraa_up2_get_pin_index(b, "UART1_TX", &(b->uart_dev[0].tx));
    mraa_up2_get_pin_index(b, "UART1_CTS", &(b->uart_dev[0].cts));
    mraa_up2_get_pin_index(b, "UART1_RTS", &(b->uart_dev[0].rts));
    b->uart_dev_count++;

    // Configure ADCs
    b->aio_count = 4;
    b->adc_raw = 12;
    b->adc_supported = 12;

    mraa_result_t ret = mraa_up2_fpga_init();
    if (ret != MRAA_SUCCESS) goto error;

    return b;

error:
    syslog(LOG_CRIT, "up2: Platform failed to initialise");
    free(b);
    return NULL;
}
