/*
 * Copyright (c) 2017 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _ATMEL_SAMD11_SOC_H_
#define _ATMEL_SAMD11_SOC_H_

#ifndef _ASMLANGUAGE

#define DONT_USE_CMSIS_INIT

#include <zephyr/types.h>

/* Add include for DTS generated information */
#include <devicetree.h>

#if defined(CONFIG_SOC_PART_NUMBER_SAMD11C14A)
#include <samd11c14a.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAMD11D14AM)
#include <samd11d14am.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAMD11D14AS)
#include <samd11d14as.h>
#elif defined(CONFIG_SOC_PART_NUMBER_SAMD11D14AU)
#include <samd11d14au.h>
#else
#error Library does not support the specified device.
#endif

#endif /* _ASMLANGUAGE */

#include "../common/atmel_sam0_dt.h"

/** Processor Clock (HCLK) Frequency */
#define SOC_ATMEL_SAM0_HCLK_FREQ_HZ ATMEL_SAM0_DT_CPU_CLK_FREQ_HZ

/** Master Clock (MCK) Frequency */
#define SOC_ATMEL_SAM0_MCK_FREQ_HZ SOC_ATMEL_SAM0_HCLK_FREQ_HZ
#define SOC_ATMEL_SAM0_XOSC32K_FREQ_HZ 32768
#define SOC_ATMEL_SAM0_OSC8M_FREQ_HZ 8000000
#define SOC_ATMEL_SAM0_GCLK0_FREQ_HZ SOC_ATMEL_SAM0_MCK_FREQ_HZ

#if defined(CONFIG_SOC_ATMEL_SAMD_XOSC32K_AS_MAIN)
#define SOC_ATMEL_SAM0_GCLK1_FREQ_HZ SOC_ATMEL_SAM0_XOSC32K_FREQ_HZ
#elif defined(CONFIG_SOC_ATMEL_SAMD_OSC8M_AS_MAIN)
#define SOC_ATMEL_SAM0_GCLK1_FREQ_HZ SOC_ATMEL_SAM0_OSC8M_FREQ_HZ
#else
#error Unsupported GCLK1 clock source.
#endif

#define SOC_ATMEL_SAM0_GCLK3_FREQ_HZ SOC_ATMEL_SAM0_OSC8M_FREQ_HZ
#define SOC_ATMEL_SAM0_APBA_FREQ_HZ SOC_ATMEL_SAM0_MCK_FREQ_HZ
#define SOC_ATMEL_SAM0_APBB_FREQ_HZ SOC_ATMEL_SAM0_MCK_FREQ_HZ
#define SOC_ATMEL_SAM0_APBC_FREQ_HZ SOC_ATMEL_SAM0_MCK_FREQ_HZ

#endif /* _ATMEL_SAMD_SOC_H_ */
