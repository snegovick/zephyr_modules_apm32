/*
 * Copyright (c) 2023, Quincy.W <wangqyfm@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/arch/cpu.h>
#include <soc.h>

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 * So the init priority has to be 0 (zero).
 *
 * @return 0
 */
static int apm32f4_init(void)
{
	// uint32_t key = irq_lock();
	
	SystemInit();
	SystemCoreClockUpdate();
	
	// NMI_INIT();
	// irq_unlock(key);

	return 0;
}

SYS_INIT(apm32f4_init, PRE_KERNEL_1, 0);
