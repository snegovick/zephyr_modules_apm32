/*
 * Copyright (c) 2023, Quincy.W <wangqyfm@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */
#include <soc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/clock_control.h>
//#include <zephyr/dt-bindings/clock/clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>

#include <apm32f10x_gpio.h>
#include <apm32f10x_rcm.h>
#include <apm32f10x_usart.h>

#define DT_DRV_COMPAT geehy_apm32_uart

struct apm32_usart_config
{
	uint32_t base;
	uint32_t clk_cfg;
	const struct pinctrl_dev_config *pcfg;
	
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t irq_config_func;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

struct apm32_usart_data
{
	uint32_t baud_rate;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t user_cb;
	void *user_data;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void usart_apm32_isr(const struct device *dev)
{
	struct apm32_usart_data *const data = dev->data;

	if (data->user_cb)
	{
		data->user_cb(dev, data->user_data);
	}
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static int usart_apm32_init(const struct device *dev)
{
	USART_Config_T USART_ConfigStruct;
	const struct device *rcm = DEVICE_DT_GET(DT_NODELABEL(rcm));
	const struct apm32_usart_config *const cfg = dev->config;
	struct apm32_usart_data *const data = dev->data;
	int ret;

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0)
	{
		return ret;
	}

	USART_ConfigStruct.baudRate = data->baud_rate;
	USART_ConfigStruct.hardwareFlow = USART_HARDWARE_FLOW_NONE;
	USART_ConfigStruct.mode = USART_MODE_TX_RX;
	USART_ConfigStruct.parity = USART_PARITY_NONE;

	(void)clock_control_on(rcm, (clock_control_subsys_t *)&cfg->clk_cfg);

	USART_Config((USART_T *)cfg->base, &USART_ConfigStruct);
	USART_Enable((USART_T *)cfg->base);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	cfg->irq_config_func(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

	return 0;
}

static int usart_apm32_poll_in(const struct device *dev, unsigned char *c)
{
	const struct apm32_usart_config *const cfg = dev->config;
	int ch = -1;

	if (USART_ReadStatusFlag((USART_T *)cfg->base, USART_FLAG_RXBNE) != RESET)
	{
		ch = USART_RxData((USART_T *)cfg->base);
	}

	return ch;
}

static void usart_apm32_poll_out(const struct device *dev, unsigned char c)
{
	const struct apm32_usart_config *const cfg = dev->config;

	USART_TxData((USART_T *)cfg->base, (uint8_t)c);

	while (USART_ReadStatusFlag((USART_T *)cfg->base, USART_FLAG_TXC) == RESET)
		;
}

static int usart_apm32_err_check(const struct device *dev)
{
	const struct apm32_usart_config *const cfg = dev->config;
	USART_T *uart = (USART_T *)cfg->base;
	int errors = 0;

	if (USART_ReadStatusFlag(uart, USART_FLAG_CTS) != RESET)
	{
		USART_ClearStatusFlag(uart, USART_FLAG_CTS);
	}

	if (USART_ReadStatusFlag(uart, USART_FLAG_LBD) != RESET)
	{
		USART_ClearStatusFlag(uart, USART_FLAG_LBD);
	}

	if (USART_ReadStatusFlag(uart, USART_FLAG_TXBE) != RESET)
	{
		USART_ClearStatusFlag(uart, USART_FLAG_TXBE);
	}

	if (USART_ReadStatusFlag(uart, USART_FLAG_PE) != RESET)
	{
		USART_ClearStatusFlag(uart, USART_FLAG_PE);
		errors |= UART_ERROR_PARITY;
	}

	if (USART_ReadStatusFlag(uart, USART_FLAG_FE) != RESET)
	{
		USART_ClearStatusFlag(uart, USART_FLAG_FE);
		errors |= UART_ERROR_FRAMING;
	}

	if (USART_ReadStatusFlag(uart, USART_FLAG_NE) != RESET)
	{
		USART_ClearStatusFlag(uart, USART_FLAG_NE);
		errors |= UART_ERROR_NOISE;
	}

	if (USART_ReadStatusFlag(uart, USART_FLAG_OVRE) != RESET)
	{
		USART_ClearStatusFlag(uart, USART_FLAG_OVRE);
		errors |= UART_ERROR_OVERRUN;
	}

	return errors;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
int usart_apm32_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len)
{
	const struct apm32_usart_config *const cfg = dev->config;
	uint8_t num_tx = 0U;

	while ((len - num_tx > 0) && usart_flag_get(cfg->base, USART_FLAG_TBE))
	{
		usart_data_transmit(cfg->base, tx_data[num_tx++]);
	}

	return num_tx;
}

int usart_apm32_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	const struct apm32_usart_config *const cfg = dev->config;
	uint8_t num_rx = 0U;

	while ((size - num_rx > 0) && usart_flag_get(cfg->base, USART_FLAG_RBNE))
	{
		rx_data[num_rx++] = usart_data_receive(cfg->base);
	}

	return num_rx;
}

void usart_apm32_irq_tx_enable(const struct device *dev)
{
	const struct apm32_usart_config *const cfg = dev->config;

	USART_EnableInterrupt((USART_T *)cfg->base, USART_INT_RXBNE);
}

void usart_apm32_irq_tx_disable(const struct device *dev)
{
	const struct apm32_usart_config *const cfg = dev->config;

	USART_DisableInterrupt((USART_T *)cfg->base, USART_INT_RXBNE);
}

int usart_apm32_irq_tx_ready(const struct device *dev)
{
	const struct apm32_usart_config *const cfg = dev->config;

	return usart_flag_get(cfg->base, USART_FLAG_TBE) && usart_interrupt_flag_get(cfg->base, USART_INT_FLAG_TC);
}

int usart_apm32_irq_tx_complete(const struct device *dev)
{
	const struct apm32_usart_config *const cfg = dev->config;

	return usart_flag_get(cfg->base, USART_FLAG_TC);
}

void usart_apm32_irq_rx_enable(const struct device *dev)
{
	const struct apm32_usart_config *const cfg = dev->config;

	usart_interrupt_enable(cfg->base, USART_INT_RBNE);
}

void usart_apm32_irq_rx_disable(const struct device *dev)
{
	const struct apm32_usart_config *const cfg = dev->config;

	usart_interrupt_disable(cfg->base, USART_INT_RBNE);
}

int usart_apm32_irq_rx_ready(const struct device *dev)
{
	const struct apm32_usart_config *const cfg = dev->config;

	return usart_flag_get(cfg->base, USART_FLAG_RBNE);
}

void usart_apm32_irq_err_enable(const struct device *dev)
{
	const struct apm32_usart_config *const cfg = dev->config;

	usart_interrupt_enable(cfg->base, USART_INT_ERR);
	usart_interrupt_enable(cfg->base, USART_INT_PERR);
}

void usart_apm32_irq_err_disable(const struct device *dev)
{
	const struct apm32_usart_config *const cfg = dev->config;

	usart_interrupt_disable(cfg->base, USART_INT_ERR);
	usart_interrupt_disable(cfg->base, USART_INT_PERR);
}

int usart_apm32_irq_is_pending(const struct device *dev)
{
	const struct apm32_usart_config *const cfg = dev->config;

	return ((usart_flag_get(cfg->base, USART_FLAG_RBNE) && usart_interrupt_flag_get(cfg->base, USART_INT_FLAG_RBNE)) ||
			(usart_flag_get(cfg->base, USART_FLAG_TC) && usart_interrupt_flag_get(cfg->base, USART_INT_FLAG_TC)));
}

void usart_apm32_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb, void *user_data)
{
	struct apm32_usart_data *const data = dev->data;

	data->user_cb = cb;
	data->user_data = user_data;
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api usart_apm32_driver_api = {
	.poll_in = usart_apm32_poll_in,
	.poll_out = usart_apm32_poll_out,
	.err_check = usart_apm32_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = usart_apm32_fifo_fill,
	.fifo_read = usart_apm32_fifo_read,
	.irq_tx_enable = usart_apm32_irq_tx_enable,
	.irq_tx_disable = usart_apm32_irq_tx_disable,
	.irq_tx_ready = usart_apm32_irq_tx_ready,
	.irq_tx_complete = usart_apm32_irq_tx_complete,
	.irq_rx_enable = usart_apm32_irq_rx_enable,
	.irq_rx_disable = usart_apm32_irq_rx_disable,
	.irq_rx_ready = usart_apm32_irq_rx_ready,
	.irq_err_enable = usart_apm32_irq_err_enable,
	.irq_err_disable = usart_apm32_irq_err_disable,
	.irq_is_pending = usart_apm32_irq_is_pending,
	.irq_callback_set = usart_apm32_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define APM32_USART_IRQ_HANDLER(n)                                                                                     \
	static void usart_apm32_config_func_##n(const struct device *dev)                                                  \
	{                                                                                                                  \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), usart_apm32_isr, DEVICE_DT_INST_GET(n), 0);             \
		irq_enable(DT_INST_IRQN(n));                                                                                   \
	}
#define APM32_USART_IRQ_HANDLER_FUNC_INIT(n) .irq_config_func = usart_apm32_config_func_##n
#else /* CONFIG_UART_INTERRUPT_DRIVEN */
#define APM32_USART_IRQ_HANDLER(n)
#define APM32_USART_IRQ_HANDLER_FUNC_INIT(n)
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#define APM32_USART_INIT(n)                                                                                            \
	PINCTRL_DT_INST_DEFINE(n);                                                                                         \
	APM32_USART_IRQ_HANDLER(n)                                                                                         \
	static struct apm32_usart_data usart_apm32_data_##n = {                                                            \
		.baud_rate = DT_INST_PROP(n, current_speed),                                                                   \
	};                                                                                                                 \
	static const struct apm32_usart_config usart_apm32_config_##n = {                                                  \
		.base = DT_INST_REG_ADDR(n),                                                                                   \
		.clk_cfg = DT_INST_CLOCKS_CELL(n, bits),                                                                         \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                                                     \
		APM32_USART_IRQ_HANDLER_FUNC_INIT(n)};                                                                         \
	DEVICE_DT_INST_DEFINE(n, &usart_apm32_init, NULL, &usart_apm32_data_##n, &usart_apm32_config_##n, PRE_KERNEL_1,    \
						  CONFIG_SERIAL_INIT_PRIORITY, &usart_apm32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(APM32_USART_INIT)
