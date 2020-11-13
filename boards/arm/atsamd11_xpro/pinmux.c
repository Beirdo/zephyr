/*
 * Copyright (c) 2018 Bryan O'Donoghue
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <drivers/pinmux.h>
#include <soc.h>

static int board_pinmux_init(const struct device *dev)
{
	const struct device *muxa = device_get_binding(DT_LABEL(DT_NODELABEL(pinmux_a)));

	ARG_UNUSED(dev);

#if (ATMEL_SAM0_DT_SERCOM_CHECK(0, atmel_sam0_uart) && CONFIG_UART_SAM0)
#warning Pin mapping may not be configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(1, atmel_sam0_uart) && CONFIG_UART_SAM0)
#warning Pin mapping may not be configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(2, atmel_sam0_uart) && CONFIG_UART_SAM0)
	/* SERCOM2 on RX=PA11, TX=PA10 */
	pinmux_pin_set(muxa, 11, PINMUX_FUNC_D);
	pinmux_pin_set(muxa, 10, PINMUX_FUNC_D);
#endif

#if (ATMEL_SAM0_DT_SERCOM_CHECK(0, atmel_sam0_spi) && CONFIG_SPI_SAM0)
	/* SPI SERCOM0 on MISO=PA09/pad 3, MOSI=PA06/pad 0, SCK=PA07/pad 1 */
	/* SS_A=PA08, SS_B=PA15, SS_C=PA27, in GPIO mode. */
	pinmux_pin_set(muxa, 6, PINMUX_FUNC_C);
	pinmux_pin_set(muxa, 7, PINMUX_FUNC_C);
	pinmux_pin_set(muxa, 9, PINMUX_FUNC_D);
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(1, atmel_sam0_spi) && CONFIG_SPI_SAM0)
#warning Pin mapping may not be configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(2, atmel_sam0_spi) && CONFIG_SPI_SAM0)
#warning Pin mapping may not be configured
#endif

#if (ATMEL_SAM0_DT_SERCOM_CHECK(0, atmel_sam0_i2c) && CONFIG_I2C_SAM0)
#warning Pin mapping may not be configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(1, atmel_sam0_i2c) && CONFIG_I2C_SAM0)
	/* SERCOM2 on SDA=PA22, SCL=PA234 */
	pinmux_pin_set(muxa, 22, PINMUX_FUNC_C);
	pinmux_pin_set(muxa, 23, PINMUX_FUNC_C);
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(2, atmel_sam0_i2c) && CONFIG_I2C_SAM0)
#warning Pin mapping may not be configured
#endif

#if (ATMEL_SAM0_DT_TCC_CHECK(0, atmel_sam0_tcc_pwm) && CONFIG_PWM_SAM0_TCC)
	/* TCC0 on WO6=PA16, WO7=PA17 */
	pinmux_pin_set(muxa, 16, PINMUX_FUNC_E);
	pinmux_pin_set(muxa, 17, PINMUX_FUNC_E);
#endif

#ifdef CONFIG_USB_DC_SAM0
	/* USB DP on PA25, USB DM on PA24 */
	pinmux_pin_set(muxa, 25, PINMUX_FUNC_G);
	pinmux_pin_set(muxa, 24, PINMUX_FUNC_G);
#endif

#ifdef CONFIG_ADC_SAM0
	/* AIN0 on PA02, AIN1 on PA03 */
	pinmux_pin_set(muxa, 2, PINMUX_FUNC_B);
	pinmux_pin_set(muxa, 3, PINMUX_FUNC_B);
#endif

	/* SWCLK on PA30, SWDIO on PA31 */
	pinmux_pin_set(muxa, 30, PINMUX_FUNC_G);
	pinmux_pin_set(muxa, 31, PINMUX_FUNC_G);


	/* Available GPIOs: PA02 (QT button), PA03 (QT button), PA04, PA05, 
	 *                  PA08 (SPI_SS_A), PA14 (button), 
	 *                  PA15 (SPI_SS_B), PA16 (TCC0/W06, LED0),
	 *                  PA17 (TCC0/WO7), PA27 (SPI_SS_DBG, VBUS_SENSE), 
	 */

	return 0;
}

SYS_INIT(board_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
