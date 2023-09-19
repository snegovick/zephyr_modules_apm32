/*
 * Copyright (c) 2023, Quincy.W <wangqyfm@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */
#include <soc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/apm32_clock.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT geehy_apm32_rcm

static inline int apm32_clock_control_on(const struct device *dev, clock_control_subsys_t sub_system)
{
	uint32_t clk_ctrl = *(uint32_t *)(sub_system);

	uint32_t reg_offset = APM32_RCM_REG_OFFSET(clk_ctrl);
	uint32_t bit_offset = APM32_RCM_BIT_OFFSET(clk_ctrl);

	*((uint32_t *)(RCM_BASE + reg_offset)) |= (1 << bit_offset);

	return 0;
}

static inline int apm32_clock_control_off(const struct device *dev, clock_control_subsys_t sub_system)
{
	uint32_t clk_ctrl = *(uint32_t *)(sub_system);

	uint32_t reg_offset = APM32_RCM_REG_OFFSET(clk_ctrl);
	uint32_t bit_offset = APM32_RCM_BIT_OFFSET(clk_ctrl);

	*((uint32_t *)(RCM_BASE + reg_offset)) ^= ~(1 << bit_offset);

	return 0;
}

static int apm32_clock_control_get_subsys_rate(const struct device *clock, clock_control_subsys_t sub_system,
											   uint32_t *rate)
{
	;

	return 0;
}

static const struct clock_control_driver_api apm32_clock_control_api = {
	.on = apm32_clock_control_on,
	.off = apm32_clock_control_off,
	.get_rate = apm32_clock_control_get_subsys_rate,
};

static int apm32_clock_control_init(const struct device *dev)
{
	return 0;
}

DEVICE_DT_INST_DEFINE(0, &apm32_clock_control_init, NULL, NULL, NULL, PRE_KERNEL_1,
					  CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &apm32_clock_control_api);
