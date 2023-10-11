/*
 * Copyright (c) 2023, Quincy.W <wangqyfm@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _APM32F_HAL_H_
#define _APM32F_HAL_H_

#ifndef _ASMLANGUAGE

#ifdef CONFIG_SOC_SERIES_APM32F4X
#include <apm32f4xx.h>

#include "apm32f4xx_adc.h"
#include "apm32f4xx_can.h"
#include "apm32f4xx_comp.h"
#include "apm32f4xx_crc.h"
#include "apm32f4xx_cryp.h"
#include "apm32f4xx_dac.h"
#include "apm32f4xx_dbgmcu.h"
#include "apm32f4xx_dci.h"
#include "apm32f4xx_dma.h"
#include "apm32f4xx_dmc.h"
#include "apm32f4xx_eint.h"
#include "apm32f4xx_fmc.h"
#include "apm32f4xx_gpio.h"
#include "apm32f4xx_hash.h"
#include "apm32f4xx_i2c.h"
#include "apm32f4xx_iwdt.h"
#include "apm32f4xx_misc.h"
#include "apm32f4xx_pmu.h"
#include "apm32f4xx_qspi.h"
#include "apm32f4xx_rcm.h"
#include "apm32f4xx_rng.h"
#include "apm32f4xx_rtc.h"
#include "apm32f4xx_sdio.h"
#include "apm32f4xx_smc.h"
#include "apm32f4xx_spi.h"
#include "apm32f4xx_syscfg.h"
#include "apm32f4xx_tmr.h"
#include "apm32f4xx_usart.h"
#include "apm32f4xx_usb.h"
#include "apm32f4xx_usb_device.h"
#include "apm32f4xx_usb_host.h"
#include "apm32f4xx_wwdt.h"

#endif /* #ifdef CONFIG_SOC_SERIES_APM32F4X */


#ifdef CONFIG_SOC_SERIES_APM32F1X
#include <apm32f10x.h>

#include "apm32f10x_adc.h"
#include "apm32f10x_bakpr.h"
#include "apm32f10x_can.h"
#include "apm32f10x_crc.h"
#include "apm32f10x_dac.h"
#include "apm32f10x_dbgmcu.h"
#include "apm32f10x_dma.h"
#include "apm32f10x_dmc.h"
#include "apm32f10x_eint.h"
#include "apm32f10x_fmc.h"
#include "apm32f10x_gpio.h"
#include "apm32f10x_i2c.h"
#include "apm32f10x_iwdt.h"
#include "apm32f10x_misc.h"
#include "apm32f10x_pmu.h"
#include "apm32f10x_qspi.h"
#include "apm32f10x_rcm.h"
#include "apm32f10x_rtc.h"
#include "apm32f10x_sci2c.h"
#include "apm32f10x_sdio.h"
#include "apm32f10x_smc.h"
#include "apm32f10x_spi.h"
#include "apm32f10x_tmr.h"
#include "apm32f10x_usart.h"
#include "apm32f10x_wwdt.h"

#endif /* #ifdef CONFIG_SOC_SERIES_APM32F1X */

#endif /* !_ASMLANGUAGE */

#endif /* _APM32F_HAL_H_ */