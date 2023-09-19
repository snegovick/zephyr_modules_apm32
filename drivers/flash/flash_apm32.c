/*
 * Copyright (c) 2023, Quincy.W <wangqyfm@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#define DT_DRV_COMPAT geehy_apm32_flash_controller

#include <string.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/init.h>
#include <soc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(flash_apm32, CONFIG_FLASH_LOG_LEVEL);

struct flash_apm32_priv {
	uint32_t regs;
// #if DT_NODE_HAS_PROP(DT_INST(0, st_stm32_flash_controller), clocks) || \
// 	DT_NODE_HAS_PROP(DT_INST(0, st_stm32h7_flash_controller), clocks)
// 	/* clock subsystem driving this peripheral */
// 	struct stm32_pclken pclken;
// #endif
	struct k_sem sem;
};

/* Let's wait for double the max erase time to be sure that the operation is
 * completed.
 */
#define APM32_FLASH_TIMEOUT	\
	(2 * DT_PROP(DT_INST(0, geehy_apm32_nv_flash), max_erase_time))

static const struct flash_parameters flash_apm32_parameters = {
	.write_block_size = FLASH_APM32_WRITE_BLOCK_SIZE,
	/* Some SoCs (L0/L1) use an EEPROM under the hood. Distinguish
	 * between them based on the presence of the PECR register. */
#if defined(FLASH_PECR_ERASE)
	.erase_value = 0,
#else
	.erase_value = 0xff,
#endif
};

static int flash_apm32_write_protection(const struct device *dev, bool enable);

int __weak flash_apm32_check_configuration(void)
{
	return 0;
}

#define flash_apm32_sem_init(dev)
#define flash_apm32_sem_take(dev)
#define flash_apm32_sem_give(dev)


int flash_apm32_wait_flash_idle(const struct device *dev)
{
	int64_t timeout_time = k_uptime_get() + APM32_FLASH_TIMEOUT;
	int rc;
	uint32_t busy_flags;

	rc = flash_apm32_check_status(dev);
	if (rc < 0) {
		return -EIO;
	}

	busy_flags = FLASH_APM32_SR_BUSY;

/* Some Series can't modify FLASH_CR reg while CFGBSY is set. Wait as well */
#if defined(FLASH_APM32_SR_CFGBSY)
	busy_flags |= FLASH_APM32_SR_CFGBSY;
#endif

	while ((FLASH_APM32_REGS(dev)->FLASH_APM32_SR & busy_flags)) {
		if (k_uptime_get() > timeout_time) {
			LOG_ERR("Timeout! val: %d", APM32_FLASH_TIMEOUT);
			return -EIO;
		}
	}

	return 0;
}

static void flash_apm32_flush_caches(const struct device *dev,
				     off_t offset, size_t len)
{
#if defined(CONFIG_SOC_SERIES_STM32F0X) || defined(CONFIG_SOC_SERIES_STM32F3X) || \
	defined(CONFIG_SOC_SERIES_STM32G0X) || defined(CONFIG_SOC_SERIES_STM32L5X) || \
	defined(CONFIG_SOC_SERIES_STM32U5X)
	ARG_UNUSED(dev);
	ARG_UNUSED(offset);
	ARG_UNUSED(len);
#elif defined(CONFIG_SOC_SERIES_STM32F4X) || \
	defined(CONFIG_SOC_SERIES_STM32L4X) || \
	defined(CONFIG_SOC_SERIES_STM32WBX) || \
	defined(CONFIG_SOC_SERIES_STM32G4X)
	ARG_UNUSED(offset);
	ARG_UNUSED(len);

	FLASH_TypeDef *regs = FLASH_APM32_REGS(dev);

	if (regs->ACR & FLASH_ACR_DCEN) {
		regs->ACR &= ~FLASH_ACR_DCEN;
		regs->ACR |= FLASH_ACR_DCRST;
		regs->ACR &= ~FLASH_ACR_DCRST;
		regs->ACR |= FLASH_ACR_DCEN;
	}
#elif defined(CONFIG_SOC_SERIES_STM32F7X)
	SCB_InvalidateDCache_by_Addr((uint32_t *)(CONFIG_FLASH_BASE_ADDRESS
						  + offset), len);
#endif
}

static int flash_apm32_read(const struct device *dev, off_t offset,
			    void *data,
			    size_t len)
{
	if (!flash_apm32_valid_range(dev, offset, len, false)) {
		LOG_ERR("Read range invalid. Offset: %ld, len: %zu",
			(long int) offset, len);
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}

	LOG_DBG("Read offset: %ld, len: %zu", (long int) offset, len);

	memcpy(data, (uint8_t *) CONFIG_FLASH_BASE_ADDRESS + offset, len);

	return 0;
}

static int flash_apm32_erase(const struct device *dev, off_t offset,
			     size_t len)
{
	int rc;

	if (!flash_apm32_valid_range(dev, offset, len, true)) {
		LOG_ERR("Erase range invalid. Offset: %ld, len: %zu",
			(long int) offset, len);
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}

	flash_apm32_sem_take(dev);

	LOG_DBG("Erase offset: %ld, len: %zu", (long int) offset, len);

	rc = flash_apm32_write_protection(dev, false);
	if (rc == 0) {
		rc = flash_apm32_block_erase_loop(dev, offset, len);
	}

	flash_apm32_flush_caches(dev, offset, len);

	int rc2 = flash_apm32_write_protection(dev, true);

	if (!rc) {
		rc = rc2;
	}

	flash_apm32_sem_give(dev);

	return rc;
}

static int flash_apm32_write(const struct device *dev, off_t offset,
			     const void *data, size_t len)
{
	int rc;

	if (!flash_apm32_valid_range(dev, offset, len, true)) {
		LOG_ERR("Write range invalid. Offset: %ld, len: %zu",
			(long int) offset, len);
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}

	flash_apm32_sem_take(dev);

	LOG_DBG("Write offset: %ld, len: %zu", (long int) offset, len);

	rc = flash_apm32_write_protection(dev, false);
	if (rc == 0) {
		rc = flash_apm32_write_range(dev, offset, data, len);
	}

	int rc2 = flash_apm32_write_protection(dev, true);

	if (!rc) {
		rc = rc2;
	}

	flash_apm32_sem_give(dev);

	return rc;
}

static int flash_apm32_write_protection(const struct device *dev, bool enable)
{
#if 0
	FLASH_TypeDef *regs = FLASH_APM32_REGS(dev);

	int rc = 0;

	if (enable) {
		rc = flash_apm32_wait_flash_idle(dev);
		if (rc) {
			flash_apm32_sem_give(dev);
			return rc;
		}
	}

#if defined(FLASH_SECURITY_NS)
	if (enable) {
		regs->NSCR |= FLASH_APM32_NSLOCK;
	} else {
		if (regs->NSCR & FLASH_APM32_NSLOCK) {
			regs->NSKEYR = FLASH_KEY1;
			regs->NSKEYR = FLASH_KEY2;
		}
	}
#else	/* FLASH_SECURITY_SEC | FLASH_SECURITY_NA */
#if defined(FLASH_CR_LOCK)
	if (enable) {
		regs->CR |= FLASH_CR_LOCK;
	} else {
		if (regs->CR & FLASH_CR_LOCK) {
			regs->KEYR = FLASH_KEY1;
			regs->KEYR = FLASH_KEY2;
		}
	}
#else
	if (enable) {
		regs->PECR |= FLASH_PECR_PRGLOCK;
		regs->PECR |= FLASH_PECR_PELOCK;
	} else {
		if (regs->PECR & FLASH_PECR_PRGLOCK) {
			LOG_DBG("Disabling write protection");
			regs->PEKEYR = FLASH_PEKEY1;
			regs->PEKEYR = FLASH_PEKEY2;
			regs->PRGKEYR = FLASH_PRGKEY1;
			regs->PRGKEYR = FLASH_PRGKEY2;
		}
		if (FLASH->PECR & FLASH_PECR_PRGLOCK) {
			LOG_ERR("Unlock failed");
			rc = -EIO;
		}
	}
#endif
#endif /* FLASH_SECURITY_NS */

	if (enable) {
		LOG_DBG("Enable write protection");
	} else {
		LOG_DBG("Disable write protection");
	}

	return rc;
#endif
	return 0;
}

static const struct flash_parameters *
flash_apm32_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_apm32_parameters;
}

static struct flash_apm32_priv flash_data = {
	.regs = DT_INST_REG_ADDR(0),
	/* Getting clocks information from device tree description depending
	 * on the presence of 'clocks' property.
	 */
#if DT_INST_NODE_HAS_PROP(0, clocks)
	.pclken = {
		.enr = DT_INST_CLOCKS_CELL(0, bits),
		.bus = DT_INST_CLOCKS_CELL(0, bus),
	}
#endif
};

static const struct flash_driver_api flash_apm32_api = {
	.erase = flash_apm32_erase,
	.write = flash_apm32_write,
	.read = flash_apm32_read,
	.get_parameters = flash_apm32_get_parameters,
// #ifdef CONFIG_FLASH_PAGE_LAYOUT
// 	.page_layout = flash_apm32_page_layout,
// #endif
};

static int apm32_flash_init(const struct device *dev)
{
	int rc;
	/* Below is applicable to F0, F1, F3, G0, G4, L1, L4, L5, U5 & WB55 series.
	 * For F2, F4, F7 & H7 series, this is not applicable.
	 */
#if DT_INST_NODE_HAS_PROP(0, clocks)
	struct flash_apm32_priv *p = FLASH_APM32_PRIV(dev);
	const struct device *const clk = DEVICE_DT_GET(APM32_CLOCK_CONTROL_NODE);

	/*
	 * On STM32 F0, F1, F3 & L1 series, flash interface clock source is
	 * always HSI, so statically enable HSI here.
	 */
#if defined(CONFIG_SOC_SERIES_STM32F0X) || \
	defined(CONFIG_SOC_SERIES_STM32F1X) || \
	defined(CONFIG_SOC_SERIES_STM32F3X) || \
	defined(CONFIG_SOC_SERIES_STM32L1X)
	LL_RCC_HSI_Enable();

	while (!LL_RCC_HSI_IsReady()) {
	}
#endif

	if (!device_is_ready(clk)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* enable clock */
	if (clock_control_on(clk, (clock_control_subsys_t *)&p->pclken) != 0) {
		LOG_ERR("Failed to enable clock");
		return -EIO;
	}
#endif

	flash_apm32_sem_init(dev);

	LOG_DBG("Flash initialized. BS: %zu",
		flash_apm32_parameters.write_block_size);

	/* Check Flash configuration */
	rc = flash_apm32_check_configuration();
	if (rc < 0) {
		return rc;
	}

#if ((CONFIG_FLASH_LOG_LEVEL >= LOG_LEVEL_DBG) && CONFIG_FLASH_PAGE_LAYOUT)
	const struct flash_pages_layout *layout;
	size_t layout_size;

	flash_apm32_page_layout(dev, &layout, &layout_size);
	for (size_t i = 0; i < layout_size; i++) {
		LOG_DBG("Block %zu: bs: %zu count: %zu", i,
			layout[i].pages_size, layout[i].pages_count);
	}
#endif

	return flash_apm32_write_protection(dev, false);
}

DEVICE_DT_INST_DEFINE(0, apm32_flash_init, NULL,
		    &flash_data, NULL, POST_KERNEL,
		    CONFIG_FLASH_INIT_PRIORITY, &flash_apm32_api);
