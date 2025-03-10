/*
 * Copyright (c) 2023, Quincy.W <wangqyfm@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_SOC_ARM_GEEHY_APM32_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ARM_GEEHY_APM32_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>

#include <zephyr/dt-bindings/pinctrl/apm32-pinctrl.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @cond INTERNAL_HIDDEN */

/** Type for APM32 pin. */
typedef struct pinctrl_soc_pin {
	/** Pinmux settings (port, pin and function). */
	uint32_t pinmux;
	/** Pin configuration (bias, drive and slew rate). */
	uint32_t pincfg;
} pinctrl_soc_pin_t;

/**
 * @brief Utility macro to initialize pinmux field in #pinctrl_pin_t.
 *
 * @param node_id Node identifier.
 */
#define Z_PINCTRL_APM32_PINMUX_INIT(node_id) DT_PROP(node_id, pinmux)

/**
 * @brief Definitions used to initialize fields in #pinctrl_pin_t
 */
#define APM32_NO_PULL     0x0
#define APM32_PULL_UP     0x1
#define APM32_PULL_DOWN   0x2
#define APM32_PUSH_PULL   0x0
#define APM32_OPEN_DRAIN  0x1
#define APM32_OUTPUT_LOW  0x0
#define APM32_OUTPUT_HIGH 0x1
#define APM32_GPIO_OUTPUT 0x1

#ifdef CONFIG_SOC_SERIES_APM32F1X
/**
 * @brief Utility macro to initialize pincfg field in #pinctrl_pin_t (F1).
 *
 * @param node_id Node identifier.
 */
#define Z_PINCTRL_APM32_PINCFG_INIT(node_id)				       \
	(((APM32_NO_PULL * DT_PROP(node_id, bias_disable)) << APM32_PUPD_SHIFT) | \
	 ((APM32_PULL_UP * DT_PROP(node_id, bias_pull_up)) << APM32_PUPD_SHIFT) | \
	 ((APM32_PULL_DOWN * DT_PROP(node_id, bias_pull_down)) << APM32_PUPD_SHIFT) | \
	 ((APM32_PUSH_PULL * DT_PROP(node_id, drive_push_pull)) << APM32_CNF_OUT_0_SHIFT) | \
	 ((APM32_OPEN_DRAIN * DT_PROP(node_id, drive_open_drain)) << APM32_CNF_OUT_0_SHIFT) | \
	 ((APM32_OUTPUT_LOW * DT_PROP(node_id, output_low)) << APM32_ODR_SHIFT) | \
	 ((APM32_OUTPUT_HIGH * DT_PROP(node_id, output_high)) << APM32_ODR_SHIFT) | \
	 (DT_ENUM_IDX(node_id, slew_rate) << APM32_MODE_OSPEED_SHIFT))
#else
/**
 * @brief Utility macro to initialize pincfg field in #pinctrl_pin_t (non-F1).
 *
 * @param node_id Node identifier.
 */
#define Z_PINCTRL_APM32_PINCFG_INIT(node_id)				       \
	(((APM32_NO_PULL * DT_PROP(node_id, bias_disable)) << APM32_PUPDR_SHIFT) | \
	 ((APM32_PULL_UP * DT_PROP(node_id, bias_pull_up)) << APM32_PUPDR_SHIFT) | \
	 ((APM32_PULL_DOWN * DT_PROP(node_id, bias_pull_down)) << APM32_PUPDR_SHIFT) | \
	 ((APM32_PUSH_PULL * DT_PROP(node_id, drive_push_pull)) << APM32_OTYPER_SHIFT) | \
	 ((APM32_OPEN_DRAIN * DT_PROP(node_id, drive_open_drain)) << APM32_OTYPER_SHIFT) | \
	 ((APM32_OUTPUT_LOW * DT_PROP(node_id, output_low)) << APM32_ODR_SHIFT) | \
	 ((APM32_OUTPUT_HIGH * DT_PROP(node_id, output_high)) << APM32_ODR_SHIFT) | \
	 ((APM32_GPIO_OUTPUT * DT_PROP(node_id, output_low)) << APM32_MODER_SHIFT) | \
	 ((APM32_GPIO_OUTPUT * DT_PROP(node_id, output_high)) << APM32_MODER_SHIFT) | \
	 (DT_ENUM_IDX(node_id, slew_rate) << APM32_OSPEEDR_SHIFT))
#endif /* CONFIG_SOC_SERIES_APM32F1X */

/**
 * @brief Utility macro to initialize each pin.
 *
 * @param node_id Node identifier.
 * @param state_prop State property name.
 * @param idx State property entry index.
 */
#define Z_PINCTRL_STATE_PIN_INIT(node_id, state_prop, idx)		       \
	{ .pinmux = 0,		       \
	  .pincfg = 0},

/**
 * @brief Utility macro to initialize state pins contained in a given property.
 *
 * @param node_id Node identifier.
 * @param prop Property name describing state pins.
 */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)			       \
	{DT_FOREACH_PROP_ELEM(node_id, prop, Z_PINCTRL_STATE_PIN_INIT)}

/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_ARM_GEEHY_APM32_PINCTRL_SOC_H_ */
