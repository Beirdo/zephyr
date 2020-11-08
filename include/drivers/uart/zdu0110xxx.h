/*
 * Copyright (c) 2020 Gavin Hurlbut
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public APIs and command definitions for the ZDU0110xxx driver
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_UART_ZDU0110XXX_H_
#define ZEPHYR_INCLUDE_DRIVERS_UART_ZDU0110XXX_H_

#include <errno.h>

#include <device.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/* UART0 -> 0x20, UART1 -> 0x40 */
#define ZDU_UART_BITS(x)                    (((x & 1) + 1) << 5)


/* Commands (as defined in https://www.zilog.com/docs/PS0389.pdf) */

#define ZDU_CMD_EEPROM_WRITE                0x00
#define ZDU_CMD_EEPROM_READ                 0x01
#define ZDU_CMD_EEPROM_SET_CURR_LOC         0x02
#define ZDU_CMD_EEPROM_GET_CURR_LOC         0x03
#define ZDU_CMD_EEPROM_ERASE_PAGE           0x04   

#define ZDU_CMD_GPIO_SET_OUTPUT             0x06
#define ZDU_CMD_GPIO_GET_INPUT              0x07
#define ZDU_CMD_GPIO_SET_CONFIG             0x08
#define ZDU_CMD_GPIO_GET_CONFIG             0x09
#define ZDU_CMD_GPIO_GET_INT_STATUS         0x0F

#define ZDU_CMD_UART_GET_STATUS(x)          (0x01 | ZDU_UART_BITS(x))
#define ZDU_CMD_UART_SET_INT_ENABLE(x)      (0x02 | ZDU_UART_BITS(x))
#define ZDU_CMD_UART_GET_INT_STATUS(x)      (0x03 | ZDU_UART_BITS(x))
#define ZDU_CMD_UART_TX_DATA(x)             (0x04 | ZDU_UART_BITS(x))
#define ZDU_CMD_UART_RX_DATA(x)             (0x05 | ZDU_UART_BITS(x))
#define ZDU_CMD_UART_SET_BAUD(x)            (0x06 | ZDU_UART_BITS(x))
#define ZDU_CMD_UART_GET_BAUD(x)            (0x07 | ZDU_UART_BITS(x))
#define ZDU_CMD_UART_SET_CONFIG(x)          (0x08 | ZDU_UART_BITS(x))
#define ZDU_CMD_UART_GET_CONFIG(x)          (0x09 | ZDU_UART_BITS(x))
#define ZDU_CMD_UART_SET_TX_WATERMARK(x)    (0x0A | ZDU_UART_BITS(x))
#define ZDU_CMD_UART_GET_TX_WATERMARK(x)    (0x0B | ZDU_UART_BITS(x))
#define ZDU_CMD_UART_SET_RX_WATERMARK(x)    (0x0C | ZDU_UART_BITS(x))
#define ZDU_CMD_UART_GET_RX_WATERMARK(x)    (0x0D | ZDU_UART_BITS(x))
#define ZDU_CMD_UART_SET_ENABLE(x)          (0x0E | ZDU_UART_BITS(x))
#define ZDU_CMD_UART_GET_FIFO_LEVELS(x)     (0x11 | ZDU_UART_BITS(x))

#define ZDU_CMD_SYS_GET_STATUS              0xE1
#define ZDU_CMD_SYS_GET_LAST_OP_RESULT      0xE3
#define ZDU_CMD_SYS_GET_VERSION             0xE5
#define ZDU_CMD_SYS_GET_INT_SOURCE          0xEF

/* Subcommands for GPIO Config */
#define ZDU_SUBCMD_GPIO_DIRECTION           0x01
#define ZDU_SUBCMD_GPIO_PULLUPS             0x02
#define ZDU_SUBCMD_GPIO_OPEN_DRAIN          0x03
#define ZDU_SUBCMD_GPIO_DEBOUNCE            0x04
#define ZDU_SUBCMD_GPIO_INTERRUPT           0x05
#define ZDU_SUBCMD_GPIO_SOFT_RESET          0x0A

/* Subcommands for UART Config */
#define ZDU_SUBCMD_UART_DATA_BITS           0x01
#define ZDU_SUBCMD_UART_PARITY              0x02
#define ZDU_SUBCMD_UART_STOP_BITS           0x03
#define ZDU_SUBCMD_UART_FLOW_CONTROL        0x04
#define ZDU_SUBCMD_UART_RESET_FIFOS         0x06
#define ZDU_SUBCMD_UART_LOOPBACK            0x08
#define ZDU_SUBCMD_UART_SOFT_RESET          0x0A


int zdu0110xxx_get_buffers(const struct device *dev, uint8_t **cmd_buf, uint8_t **rsp_buf);
int zdu0110xxx_release_buffers(const struct device *dev);


uint8_t zdu0110xxx_get_slave_addr(const struct device *dev);


/**
 * @brief Send command to the ZDU0110xxx, and get result.
 *
 * @param dev	        ZDU0110xxx device structure.
 * @param cmd_buf       Data buffer for command data (including command)
 * @param cmd_buf_len   Length of data in cmd_data (not validated yet)
 * @param rsp_buf       Data buffer for command response (NULL if none required)
 * @param rsp_buf_len   Length of data to read into rsp_buf
 *
 * @return	    0 on success.
 */
int zdu0110xxx_send_command(const struct device *dev,
                uint8_t *cmd_buf, size_t cmd_buf_len,
                uint8_t *rsp_buf, size_t rsp_buf_len);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ZEPHYR_INCLUDE_DRIVERS_UART_ZDU0110XXX_H_ */
