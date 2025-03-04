/*
 * Copyright (c) 2023, Quincy.W <wangqyfm@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_APM32_CLOCK_CONTROL_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_APM32_CLOCK_CONTROL_H_

#include <zephyr/drivers/clock_control.h>

/** Common clock control device node for all APM32 chips */
#define APM32_CLOCK_CONTROL_NODE DT_NODELABEL(rcm)

/** RCC node related symbols */

#define APM32_AHB_PRESCALER	    DT_PROP(DT_NODELABEL(rcm), ahb_prescaler)
#define APM32_APB1_PRESCALER	DT_PROP(DT_NODELABEL(rcm), apb1_prescaler)
#define APM32_APB2_PRESCALER	DT_PROP(DT_NODELABEL(rcm), apb2_prescaler)
#define APM32_APB3_PRESCALER	DT_PROP(DT_NODELABEL(rcm), apb3_prescaler)
#define APM32_AHB3_PRESCALER	DT_PROP(DT_NODELABEL(rcm), ahb3_prescaler)
#define APM32_AHB4_PRESCALER	DT_PROP(DT_NODELABEL(rcm), ahb4_prescaler)
#define APM32_CPU1_PRESCALER	DT_PROP(DT_NODELABEL(rcm), cpu1_prescaler)
#define APM32_CPU2_PRESCALER	DT_PROP(DT_NODELABEL(rcm), cpu2_prescaler)

#if DT_NODE_HAS_PROP(DT_NODELABEL(rcm), ahb_prescaler)
#define APM32_CORE_PRESCALER	APM32_AHB_PRESCALER
#elif DT_NODE_HAS_PROP(DT_NODELABEL(rcm), cpu1_prescaler)
#define APM32_CORE_PRESCALER	APM32_CPU1_PRESCALER
#endif

#if DT_NODE_HAS_PROP(DT_NODELABEL(rcm), ahb3_prescaler)
#define APM32_FLASH_PRESCALER	APM32_AHB3_PRESCALER
#elif DT_NODE_HAS_PROP(DT_NODELABEL(rcm), ahb4_prescaler)
#define APM32_FLASH_PRESCALER	APM32_AHB4_PRESCALER
#else
#define APM32_FLASH_PRESCALER	APM32_CORE_PRESCALER
#endif

#define APM32_D1CPRE	DT_PROP(DT_NODELABEL(rcm), d1cpre)
#define APM32_HPRE	DT_PROP(DT_NODELABEL(rcm), hpre)
#define APM32_D2PPRE1	DT_PROP(DT_NODELABEL(rcm), d2ppre1)
#define APM32_D2PPRE2	DT_PROP(DT_NODELABEL(rcm), d2ppre2)
#define APM32_D1PPRE	DT_PROP(DT_NODELABEL(rcm), d1ppre)
#define APM32_D3PPRE	DT_PROP(DT_NODELABEL(rcm), d3ppre)

#define DT_RCC_CLOCKS_CTRL	DT_CLOCKS_CTLR(DT_NODELABEL(rcm))

/* To enable use of IS_ENABLED utility macro, these symbols
 * should not be defined directly using DT_SAME_NODE.
 */
#if DT_SAME_NODE(DT_RCC_CLOCKS_CTRL, DT_NODELABEL(pll))
#define APM32_SYSCLK_SRC_PLL	1
#endif
#if DT_SAME_NODE(DT_RCC_CLOCKS_CTRL, DT_NODELABEL(clk_hsi))
#define APM32_SYSCLK_SRC_HSI	1
#endif
#if DT_SAME_NODE(DT_RCC_CLOCKS_CTRL, DT_NODELABEL(clk_hse))
#define APM32_SYSCLK_SRC_HSE	1
#endif
#if DT_SAME_NODE(DT_RCC_CLOCKS_CTRL, DT_NODELABEL(clk_msi))
#define APM32_SYSCLK_SRC_MSI	1
#endif
#if DT_SAME_NODE(DT_RCC_CLOCKS_CTRL, DT_NODELABEL(clk_msis))
#define APM32_SYSCLK_SRC_MSIS	1
#endif
#if DT_SAME_NODE(DT_RCC_CLOCKS_CTRL, DT_NODELABEL(clk_csi))
#define APM32_SYSCLK_SRC_CSI	1
#endif


/** PLL node related symbols */

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(pll), st_stm32f2_pll_clock, okay) || \
	DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(pll), st_stm32f4_pll_clock, okay) || \
	DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(pll), st_stm32f7_pll_clock, okay) || \
	DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(pll), st_stm32g0_pll_clock, okay) || \
	DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(pll), st_stm32g4_pll_clock, okay) || \
	DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(pll), st_stm32l4_pll_clock, okay) || \
	DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(pll), st_stm32u5_pll_clock, okay) || \
	DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(pll), st_stm32wb_pll_clock, okay) || \
	DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(pll), st_stm32h7_pll_clock, okay)
#define APM32_PLL_ENABLED	1
#define APM32_PLL_M_DIVISOR	DT_PROP(DT_NODELABEL(pll), div_m)
#define APM32_PLL_N_MULTIPLIER	DT_PROP(DT_NODELABEL(pll), mul_n)
#define APM32_PLL_P_ENABLED	DT_NODE_HAS_PROP(DT_NODELABEL(pll), div_p)
#define APM32_PLL_P_DIVISOR	DT_PROP_OR(DT_NODELABEL(pll), div_p, 1)
#define APM32_PLL_Q_ENABLED	DT_NODE_HAS_PROP(DT_NODELABEL(pll), div_q)
#define APM32_PLL_Q_DIVISOR	DT_PROP_OR(DT_NODELABEL(pll), div_q, 1)
#define APM32_PLL_R_ENABLED	DT_NODE_HAS_PROP(DT_NODELABEL(pll), div_r)
#define APM32_PLL_R_DIVISOR	DT_PROP_OR(DT_NODELABEL(pll), div_r, 1)
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(pll2), st_stm32u5_pll_clock, okay)
#define APM32_PLL2_ENABLED	1
#define APM32_PLL2_M_DIVISOR	DT_PROP(DT_NODELABEL(pll2), div_m)
#define APM32_PLL2_N_MULTIPLIER	DT_PROP(DT_NODELABEL(pll2), mul_n)
#define APM32_PLL2_P_ENABLED	DT_NODE_HAS_PROP(DT_NODELABEL(pll2), div_p)
#define APM32_PLL2_P_DIVISOR	DT_PROP_OR(DT_NODELABEL(pll2), div_p, 1)
#define APM32_PLL2_Q_ENABLED	DT_NODE_HAS_PROP(DT_NODELABEL(pll2), div_q)
#define APM32_PLL2_Q_DIVISOR	DT_PROP_OR(DT_NODELABEL(pll2), div_q, 1)
#define APM32_PLL2_R_ENABLED	DT_NODE_HAS_PROP(DT_NODELABEL(pll2), div_r)
#define APM32_PLL2_R_DIVISOR	DT_PROP_OR(DT_NODELABEL(pll2), div_r, 1)
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(pll3), st_stm32h7_pll_clock, okay) || \
	DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(pll3), st_stm32u5_pll_clock, okay)
#define APM32_PLL3_ENABLED	1
#define APM32_PLL3_M_DIVISOR	DT_PROP(DT_NODELABEL(pll3), div_m)
#define APM32_PLL3_N_MULTIPLIER	DT_PROP(DT_NODELABEL(pll3), mul_n)
#define APM32_PLL3_P_ENABLED	DT_NODE_HAS_PROP(DT_NODELABEL(pll3), div_p)
#define APM32_PLL3_P_DIVISOR	DT_PROP_OR(DT_NODELABEL(pll3), div_p, 1)
#define APM32_PLL3_Q_ENABLED	DT_NODE_HAS_PROP(DT_NODELABEL(pll3), div_q)
#define APM32_PLL3_Q_DIVISOR	DT_PROP_OR(DT_NODELABEL(pll3), div_q, 1)
#define APM32_PLL3_R_ENABLED	DT_NODE_HAS_PROP(DT_NODELABEL(pll3), div_r)
#define APM32_PLL3_R_DIVISOR	DT_PROP_OR(DT_NODELABEL(pll3), div_r, 1)
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(pll), st_stm32f1_pll_clock, okay)
#define APM32_PLL_ENABLED	1
#define APM32_PLL_XTPRE		DT_PROP(DT_NODELABEL(pll), xtpre)
#define APM32_PLL_MULTIPLIER	DT_PROP(DT_NODELABEL(pll), mul)
#elif DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(pll), st_stm32f0_pll_clock, okay) || \
	DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(pll), st_stm32f100_pll_clock, okay) || \
	DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(pll), st_stm32f105_pll_clock, okay)
#define APM32_PLL_ENABLED	1
#define APM32_PLL_MULTIPLIER	DT_PROP(DT_NODELABEL(pll), mul)
#define APM32_PLL_PREDIV	DT_PROP(DT_NODELABEL(pll), prediv)
#elif DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(pll), st_stm32l0_pll_clock, okay)
#define APM32_PLL_ENABLED	1
#define APM32_PLL_DIVISOR	DT_PROP(DT_NODELABEL(pll), div)
#define APM32_PLL_MULTIPLIER	DT_PROP(DT_NODELABEL(pll), mul)
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(pll2), st_stm32f105_pll2_clock, okay)
#define APM32_PLL2_ENABLED	1
#define APM32_PLL2_MULTIPLIER	DT_PROP(DT_NODELABEL(pll2), mul)
#define APM32_PLL2_PREDIV	DT_PROP(DT_NODELABEL(pll2), prediv)
#endif

/** PLL/PLL1 clock source */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pll), okay) && \
	DT_NODE_HAS_PROP(DT_NODELABEL(pll), clocks)
#define DT_PLL_CLOCKS_CTRL	DT_CLOCKS_CTLR(DT_NODELABEL(pll))
#if DT_SAME_NODE(DT_PLL_CLOCKS_CTRL, DT_NODELABEL(clk_msi))
#define APM32_PLL_SRC_MSI	1
#endif
#if DT_SAME_NODE(DT_PLL_CLOCKS_CTRL, DT_NODELABEL(clk_msis))
#define APM32_PLL_SRC_MSIS	1
#endif
#if DT_SAME_NODE(DT_PLL_CLOCKS_CTRL, DT_NODELABEL(clk_hsi))
#define APM32_PLL_SRC_HSI	1
#endif
#if DT_SAME_NODE(DT_PLL_CLOCKS_CTRL, DT_NODELABEL(clk_csi))
#define APM32_PLL_SRC_CSI	1
#endif
#if DT_SAME_NODE(DT_PLL_CLOCKS_CTRL, DT_NODELABEL(clk_hse))
#define APM32_PLL_SRC_HSE	1
#endif
#if DT_SAME_NODE(DT_PLL_CLOCKS_CTRL, DT_NODELABEL(pll2))
#define APM32_PLL_SRC_PLL2	1
#endif

#endif

/** PLL2 clock source */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pll2), okay) && \
	DT_NODE_HAS_PROP(DT_NODELABEL(pll2), clocks)
#define DT_PLL2_CLOCKS_CTRL	DT_CLOCKS_CTLR(DT_NODELABEL(pll2))
#if DT_SAME_NODE(DT_PLL2_CLOCKS_CTRL, DT_NODELABEL(clk_msis))
#define APM32_PLL2_SRC_MSIS	1
#endif
#if DT_SAME_NODE(DT_PLL2_CLOCKS_CTRL, DT_NODELABEL(clk_hsi))
#define APM32_PLL2_SRC_HSI	1
#endif
#if DT_SAME_NODE(DT_PLL2_CLOCKS_CTRL, DT_NODELABEL(clk_hse))
#define APM32_PLL2_SRC_HSE	1
#endif

#endif

/** PLL3 clock source */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pll3), okay) && \
	DT_NODE_HAS_PROP(DT_NODELABEL(pll3), clocks)
#define DT_PLL3_CLOCKS_CTRL	DT_CLOCKS_CTLR(DT_NODELABEL(pll3))
#if DT_SAME_NODE(DT_PLL3_CLOCKS_CTRL, DT_NODELABEL(clk_msis))
#define APM32_PLL3_SRC_MSIS	1
#endif
#if DT_SAME_NODE(DT_PLL3_CLOCKS_CTRL, DT_NODELABEL(clk_hsi))
#define APM32_PLL3_SRC_HSI	1
#endif
#if DT_SAME_NODE(DT_PLL3_CLOCKS_CTRL, DT_NODELABEL(clk_hse))
#define APM32_PLL3_SRC_HSE	1
#endif

#endif


/** Fixed clocks related symbols */

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_lse), fixed_clock, okay)
#define APM32_LSE_ENABLED	1
#define APM32_LSE_FREQ		DT_PROP(DT_NODELABEL(clk_lse), clock_frequency)
#define APM32_LSE_DRIVING	0
#elif DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_lse), st_stm32_lse_clock, okay)
#define APM32_LSE_ENABLED	1
#define APM32_LSE_FREQ		DT_PROP(DT_NODELABEL(clk_lse), clock_frequency)
#define APM32_LSE_DRIVING	DT_PROP(DT_NODELABEL(clk_lse), driving_capability)
#else
#define APM32_LSE_ENABLED	0
#define APM32_LSE_FREQ		0
#define APM32_LSE_DRIVING	0
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_msi), st_stm32_msi_clock, okay) || \
	DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_msi), st_stm32l0_msi_clock, okay)
#define APM32_MSI_ENABLED	1
#define APM32_MSI_RANGE		DT_PROP(DT_NODELABEL(clk_msi), msi_range)
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_msi), st_stm32_msi_clock, okay)
#define APM32_MSI_ENABLED	1
#define APM32_MSI_PLL_MODE	DT_PROP(DT_NODELABEL(clk_msi), msi_pll_mode)
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_msis), st_stm32u5_msi_clock, okay)
#define APM32_MSIS_ENABLED	1
#define APM32_MSIS_RANGE	DT_PROP(DT_NODELABEL(clk_msis), msi_range)
#define APM32_MSIS_PLL_MODE	DT_PROP(DT_NODELABEL(clk_msis), msi_pll_mode)
#else
#define APM32_MSIS_ENABLED	0
#define APM32_MSIS_RANGE	0
#define APM32_MSIS_PLL_MODE	0
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_msik), st_stm32u5_msi_clock, okay)
#define APM32_MSIK_ENABLED	1
#define APM32_MSIK_RANGE	DT_PROP(DT_NODELABEL(clk_msik), msi_range)
#define APM32_MSIK_PLL_MODE	DT_PROP(DT_NODELABEL(clk_msik), msi_pll_mode)
#else
#define APM32_MSIK_ENABLED	0
#define APM32_MSIK_RANGE	0
#define APM32_MSIK_PLL_MODE	0
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_csi), fixed_clock, okay)
#define APM32_CSI_ENABLED	1
#define APM32_CSI_FREQ		DT_PROP(DT_NODELABEL(clk_csi), clock_frequency)
#else
#define APM32_CSI_FREQ		0
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_lsi), fixed_clock, okay)
#define APM32_LSI_ENABLED	1
#define APM32_LSI_FREQ		DT_PROP(DT_NODELABEL(clk_lsi), clock_frequency)
#else
#define APM32_LSI_FREQ		0
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_hsi), fixed_clock, okay)
#define APM32_HSI_DIV_ENABLED	0
#define APM32_HSI_ENABLED	1
#define APM32_HSI_FREQ		DT_PROP(DT_NODELABEL(clk_hsi), clock_frequency)
#elif DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_hsi), st_stm32h7_hsi_clock, okay) \
	|| DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_hsi), st_stm32g0_hsi_clock, okay)
#define APM32_HSI_DIV_ENABLED	1
#define APM32_HSI_ENABLED	1
#define APM32_HSI_DIVISOR	DT_PROP(DT_NODELABEL(clk_hsi), hsi_div)
#define APM32_HSI_FREQ		DT_PROP(DT_NODELABEL(clk_hsi), clock_frequency)
#else
#define APM32_HSI_DIV_ENABLED	0
#define APM32_HSI_DIVISOR	1
#define APM32_HSI_FREQ		0
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_hse), fixed_clock, okay)
#define APM32_HSE_ENABLED	1
#define APM32_HSE_FREQ		DT_PROP(DT_NODELABEL(clk_hse), clock_frequency)
#elif DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_hse), st_stm32_hse_clock, okay)
#define APM32_HSE_ENABLED	1
#define APM32_HSE_BYPASS	DT_PROP(DT_NODELABEL(clk_hse), hse_bypass)
#define APM32_HSE_FREQ		DT_PROP(DT_NODELABEL(clk_hse), clock_frequency)
#elif DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(clk_hse), st_stm32wl_hse_clock, okay)
#define APM32_HSE_ENABLED	1
#define APM32_HSE_TCXO		DT_PROP(DT_NODELABEL(clk_hse), hse_tcxo)
#define APM32_HSE_DIV2		DT_PROP(DT_NODELABEL(clk_hse), hse_div2)
#define APM32_HSE_FREQ		DT_PROP(DT_NODELABEL(clk_hse), clock_frequency)
#else
#define APM32_HSE_FREQ		0
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(perck), st_stm32_clock_mux, okay)
#define APM32_CKPER_ENABLED	1
#endif

/** Driver structure definition */

struct apm32_pclken {
	uint32_t bus;
	uint32_t enr;
};

/** Device tree clocks helpers  */

#define APM32_CLOCK_INFO(clk_index, node_id)				\
	{								\
	.enr = DT_CLOCKS_CELL_BY_IDX(node_id, clk_index, bits),		\
	.bus = DT_CLOCKS_CELL_BY_IDX(node_id, clk_index, bus)		\
	}
#define APM32_DT_CLOCKS(node_id)					\
	{								\
		LISTIFY(DT_NUM_CLOCKS(node_id),				\
			APM32_CLOCK_INFO, (,), node_id)			\
	}

#define APM32_DT_INST_CLOCKS(inst)					\
	APM32_DT_CLOCKS(DT_DRV_INST(inst))

#define APM32_DOMAIN_CLOCK_INST_SUPPORT(inst) DT_INST_CLOCKS_HAS_IDX(inst, 1) ||
#define APM32_DT_INST_DEV_DOMAIN_CLOCK_SUPPORT				\
		(DT_INST_FOREACH_STATUS_OKAY(APM32_DOMAIN_CLOCK_INST_SUPPORT) 0)

#define APM32_DOMAIN_CLOCK_SUPPORT(id) DT_CLOCKS_HAS_IDX(DT_NODELABEL(id), 1) ||
#define APM32_DT_DEV_DOMAIN_CLOCK_SUPPORT					\
		(DT_FOREACH_STATUS_OKAY(APM32_DOMAIN_CLOCK_SUPPORT) 0)

/** Clock source binding accessors */

/**
 * @brief Obtain register field from clock configuration.
 *
 * @param clock clock bit field value.
 */
#define APM32_CLOCK_REG_GET(clock) \
	(((clock) >> APM32_CLOCK_REG_SHIFT) & APM32_CLOCK_REG_MASK)

/**
 * @brief Obtain position field from clock configuration.
 *
 * @param clock Clock bit field value.
 */
#define APM32_CLOCK_SHIFT_GET(clock) \
	(((clock) >> APM32_CLOCK_SHIFT_SHIFT) & APM32_CLOCK_SHIFT_MASK)

/**
 * @brief Obtain mask field from clock configuration.
 *
 * @param clock Clock bit field value.
 */
#define APM32_CLOCK_MASK_GET(clock) \
	(((clock) >> APM32_CLOCK_MASK_SHIFT) & APM32_CLOCK_MASK_MASK)

/**
 * @brief Obtain value field from clock configuration.
 *
 * @param clock Clock bit field value.
 */
#define APM32_CLOCK_VAL_GET(clock) \
	(((clock) >> APM32_CLOCK_VAL_SHIFT) & APM32_CLOCK_VAL_MASK)

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_APM32_CLOCK_CONTROL_H_ */
