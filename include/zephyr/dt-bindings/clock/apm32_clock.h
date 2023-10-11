/*
 * Copyright (c) 2023, Quincy.W <wangqyfm@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_APM32_CLOCK_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_APM32_CLOCK_H_

#define APM32_CLOCK_REG_OFFSET 5

#define APM32_RCM_REG_OFFSET(clk_cfg) (clk_cfg >> APM32_CLOCK_REG_OFFSET)
#define APM32_RCM_BIT_OFFSET(clk_cfg) (clk_cfg & 0x1f)

#define APM32_RCM_FIELD_DEF_OFFSET(address, offset) (((address) << APM32_CLOCK_REG_OFFSET) | (offset))

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_APM32_CLOCK_H_ */
