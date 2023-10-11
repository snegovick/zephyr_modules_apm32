/*
 * Copyright (c) 2023, Quincy.W <wangqyfm@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT geehy_apm32_gpio

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control/apm32_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>

#include <apm32f_hal.h>

/**
 * @brief Common GPIO driver for APM32 MCUs.
 */

#define APM32_PORTA 0
#define APM32_PORTB 1
#define APM32_PORTC 2
#define APM32_PORTD 3
#define APM32_PORTE 4
#define APM32_PORTF 5
#define APM32_PORTG 6
#define APM32_PORTH 7

/**
 * @brief configuration of GPIO device
 */
struct gpio_apm32_config
{
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	/* port base address */
	uint32_t *base;
	/* IO port */
	int port;
	uint32_t clk_ctrl;
};

/**
 * @brief driver data
 */
struct gpio_apm32_data
{
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	/* device's owner of this data */
	const struct device *dev;
	/* user ISR cb */
	sys_slist_t cb;
};

#if 0

/**
 * @brief EXTI interrupt callback
 */
static void gpio_apm32_isr(int line, void *arg)
{
    struct gpio_apm32_data *data = arg;

    gpio_fire_callbacks(&data->cb, data->dev, BIT(line));
}

/**
 * @brief Common gpio flags to custom flags
 */
static int gpio_apm32_flags_to_conf(gpio_flags_t flags, int *pincfg)
{

    if ((flags & GPIO_OUTPUT) != 0)
    {
        /* Output only or Output/Input */

        *pincfg = APM32_PINCFG_MODE_OUTPUT;

        if ((flags & GPIO_SINGLE_ENDED) != 0)
        {
            if (flags & GPIO_LINE_OPEN_DRAIN)
            {
                *pincfg |= APM32_PINCFG_OPEN_DRAIN;
            }
            else
            {
                /* Output can't be open source */
                return -ENOTSUP;
            }
        }
        else
        {
            *pincfg |= APM32_PINCFG_PUSH_PULL;
        }

        if ((flags & GPIO_PULL_UP) != 0)
        {
            *pincfg |= APM32_PINCFG_PULL_UP;
        }
        else if ((flags & GPIO_PULL_DOWN) != 0)
        {
            *pincfg |= APM32_PINCFG_PULL_DOWN;
        }
    }
    else if ((flags & GPIO_INPUT) != 0)
    {
        /* Input */

        *pincfg = APM32_PINCFG_MODE_INPUT;

        if ((flags & GPIO_PULL_UP) != 0)
        {
            *pincfg |= APM32_PINCFG_PULL_UP;
        }
        else if ((flags & GPIO_PULL_DOWN) != 0)
        {
            *pincfg |= APM32_PINCFG_PULL_DOWN;
        }
        else
        {
            *pincfg |= APM32_PINCFG_FLOATING;
        }
    }
    else
    {
        /* Deactivated: Analog */
        *pincfg = APM32_PINCFG_MODE_ANALOG;
    }

    return 0;
}
#endif

#if defined(CONFIG_GPIO_GET_CONFIG) && !defined(CONFIG_SOC_SERIES_APM32F1X)
/**
 * @brief Custom apm32 flags to zephyr
 */
static int gpio_apm32_pincfg_to_flags(struct gpio_apm32_pin pin_cfg, gpio_flags_t *out_flags)
{
	gpio_flags_t flags = 0;

	if (pin_cfg.mode == LL_GPIO_MODE_OUTPUT)
	{
		flags |= GPIO_OUTPUT;
		if (pin_cfg.type == LL_GPIO_OUTPUT_OPENDRAIN)
		{
			flags |= GPIO_OPEN_DRAIN;
		}
	}
	else if (pin_cfg.mode == LL_GPIO_MODE_INPUT)
	{
		flags |= GPIO_INPUT;
#ifdef CONFIG_SOC_SERIES_APM32F1X
	}
	else if (pin_cfg.mode == LL_GPIO_MODE_FLOATING)
	{
		flags |= GPIO_INPUT;
#endif
	}
	else
	{
		flags |= GPIO_DISCONNECTED;
	}

	if (pin_cfg.pupd == LL_GPIO_PULL_UP)
	{
		flags |= GPIO_PULL_UP;
	}
	else if (pin_cfg.pupd == LL_GPIO_PULL_DOWN)
	{
		flags |= GPIO_PULL_DOWN;
	}

	if (pin_cfg.out_state != 0)
	{
		flags |= GPIO_OUTPUT_HIGH;
	}
	else
	{
		flags |= GPIO_OUTPUT_LOW;
	}

	*out_flags = flags;

	return 0;
}
#endif /* CONFIG_GPIO_GET_CONFIG */

#if 0
/**
 * @brief Translate pin to pinval that the LL library needs
 */
static inline uint32_t apm32_pinval_get(int pin)
{
    uint32_t pinval;

#ifdef CONFIG_SOC_SERIES_APM32F1X
    pinval = (1 << pin) << GPIO_PIN_MASK_POS;
    if (pin < 8)
    {
        pinval |= 1 << pin;
    }
    else
    {
        pinval |= (1 << (pin % 8)) | 0x04000000;
    }
#else
    pinval = 1 << pin;
#endif
    return pinval;
}
#endif

#if 0
/**
 * @brief Configure the hardware.
 */
static void gpio_apm32_configure_raw(const struct device *dev, int pin,
                                     int conf, int func)
{
    const struct gpio_apm32_config *cfg = dev->config;
    GPIO_T *gpio = (GPIO_T *)cfg->base;

    int pin_ll = apm32_pinval_get(pin);

#ifdef CONFIG_SOC_SERIES_APM32F1X
    ARG_UNUSED(func);

    uint32_t temp = conf &
                    (APM32_MODE_INOUT_MASK << APM32_MODE_INOUT_SHIFT);

    if (temp == APM32_MODE_INPUT)
    {
        temp = conf & (APM32_CNF_IN_MASK << APM32_CNF_IN_SHIFT);

        if (temp == APM32_CNF_IN_ANALOG)
        {
            LL_GPIO_SetPinMode(gpio, pin_ll, LL_GPIO_MODE_ANALOG);
        }
        else if (temp == APM32_CNF_IN_FLOAT)
        {
            LL_GPIO_SetPinMode(gpio, pin_ll, LL_GPIO_MODE_FLOATING);
        }
        else
        {
            temp = conf & (APM32_PUPD_MASK << APM32_PUPD_SHIFT);

            if (temp == APM32_PUPD_PULL_UP)
            {
                LL_GPIO_SetPinPull(gpio, pin_ll,
                                   LL_GPIO_PULL_UP);
            }
            else
            {
                LL_GPIO_SetPinPull(gpio, pin_ll,
                                   LL_GPIO_PULL_DOWN);
            }

            LL_GPIO_SetPinMode(gpio, pin_ll, LL_GPIO_MODE_INPUT);
        }
    }
    else
    {
        temp = conf & (APM32_CNF_OUT_1_MASK << APM32_CNF_OUT_1_SHIFT);

        if (temp == APM32_CNF_GP_OUTPUT)
        {
            LL_GPIO_SetPinMode(gpio, pin_ll, LL_GPIO_MODE_OUTPUT);
        }
        else
        {
            LL_GPIO_SetPinMode(gpio, pin_ll,
                               LL_GPIO_MODE_ALTERNATE);
        }

        temp = conf & (APM32_CNF_OUT_0_MASK << APM32_CNF_OUT_0_SHIFT);

        if (temp == APM32_CNF_PUSH_PULL)
        {
            LL_GPIO_SetPinOutputType(gpio, pin_ll,
                                     LL_GPIO_OUTPUT_PUSHPULL);
        }
        else
        {
            LL_GPIO_SetPinOutputType(gpio, pin_ll,
                                     LL_GPIO_OUTPUT_OPENDRAIN);
        }

        temp = conf &
               (APM32_MODE_OSPEED_MASK << APM32_MODE_OSPEED_SHIFT);

        if (temp == APM32_MODE_OUTPUT_MAX_2)
        {
            LL_GPIO_SetPinSpeed(gpio, pin_ll,
                                LL_GPIO_SPEED_FREQ_LOW);
        }
        else if (temp == APM32_MODE_OUTPUT_MAX_10)
        {
            LL_GPIO_SetPinSpeed(gpio, pin_ll,
                                LL_GPIO_SPEED_FREQ_MEDIUM);
        }
        else
        {
            LL_GPIO_SetPinSpeed(gpio, pin_ll,
                                LL_GPIO_SPEED_FREQ_HIGH);
        }
    }
#else
    unsigned int mode, otype, ospeed, pupd;

    mode = conf & (APM32_MODER_MASK << APM32_MODER_SHIFT);
    otype = conf & (APM32_OTYPER_MASK << APM32_OTYPER_SHIFT);
    ospeed = conf & (APM32_OSPEEDR_MASK << APM32_OSPEEDR_SHIFT);
    pupd = conf & (APM32_PUPDR_MASK << APM32_PUPDR_SHIFT);

    z_apm32_hsem_lock(CFG_HW_GPIO_SEMID, HSEM_LOCK_DEFAULT_RETRY);

#if defined(CONFIG_SOC_SERIES_APM32L4X) && defined(GPIO_ASCR_ASC0)
    /*
     * For APM32L47xx/48xx, register ASCR should be configured to connect
     * analog switch of gpio lines to the ADC.
     */
    if (mode == APM32_MODER_ANALOG_MODE)
    {
        LL_GPIO_EnablePinAnalogControl(gpio, pin_ll);
    }
#endif

    LL_GPIO_SetPinOutputType(gpio, pin_ll, otype >> APM32_OTYPER_SHIFT);

    LL_GPIO_SetPinSpeed(gpio, pin_ll, ospeed >> APM32_OSPEEDR_SHIFT);

    LL_GPIO_SetPinPull(gpio, pin_ll, pupd >> APM32_PUPDR_SHIFT);

    if (mode == APM32_MODER_ALT_MODE)
    {
        if (pin < 8)
        {
            LL_GPIO_SetAFPin_0_7(gpio, pin_ll, func);
        }
        else
        {
            LL_GPIO_SetAFPin_8_15(gpio, pin_ll, func);
        }
    }

    LL_GPIO_SetPinMode(gpio, pin_ll, mode >> APM32_MODER_SHIFT);

    z_apm32_hsem_unlock(CFG_HW_GPIO_SEMID);
#endif /* CONFIG_SOC_SERIES_APM32F1X */
}
#endif

#if 0
/**
 * @brief GPIO port clock handling
 */
static int gpio_apm32_clock_request(const struct device *dev, bool on)
{
    const struct gpio_apm32_config *cfg = dev->config;
    int ret = 0;

    __ASSERT_NO_MSG(dev != NULL);

    /* enable clock for subsystem */
    const struct device *const clk = DEVICE_DT_GET(APM32_CLOCK_CONTROL_NODE);

    if (on)
    {
        ret = clock_control_on(clk,
                               (clock_control_subsys_t *)&cfg->pclken);
    }
    else
    {
        ret = clock_control_off(clk,
                                (clock_control_subsys_t *)&cfg->pclken);
    }

    if (ret != 0)
    {
        return ret;
    }

    return ret;
}

static inline uint32_t gpio_apm32_pin_to_exti_line(int pin)
{
#if defined(CONFIG_SOC_SERIES_APM32L0X) || defined(CONFIG_SOC_SERIES_APM32F0X)
    return ((pin % 4 * 4) << 16) | (pin / 4);
#elif defined(CONFIG_SOC_SERIES_APM32MP1X)
    return (((pin * 8) % 32) << 16) | (pin / 4);
#elif defined(CONFIG_SOC_SERIES_APM32G0X) || defined(CONFIG_SOC_SERIES_APM32L5X) || defined(CONFIG_SOC_SERIES_APM32U5X)
    return ((pin & 0x3) << (16 + 3)) | (pin >> 2);
#else
    return (0xF << ((pin % 4 * 4) + 16)) | (pin / 4);
#endif
}

static void gpio_apm32_set_exti_source(int port, int pin)
{
    uint32_t line = gpio_apm32_pin_to_exti_line(pin);

#if defined(CONFIG_SOC_SERIES_APM32L0X) && defined(LL_SYSCFG_EXTI_PORTH)
    /*
     * Ports F and G are not present on some APM32L0 parts, so
     * for these parts port H external interrupt should be enabled
     * by writing value 0x5 instead of 0x7.
     */
    if (port == APM32_PORTH)
    {
        port = LL_SYSCFG_EXTI_PORTH;
    }
#endif

#ifdef CONFIG_SOC_SERIES_APM32F1X
    LL_GPIO_AF_SetEXTISource(port, line);
#elif CONFIG_SOC_SERIES_APM32MP1X
    LL_EXTI_SetEXTISource(port, line);
#elif defined(CONFIG_SOC_SERIES_APM32G0X) || defined(CONFIG_SOC_SERIES_APM32L5X) || defined(CONFIG_SOC_SERIES_APM32U5X)
    LL_EXTI_SetEXTISource(port, line);
#else
    LL_SYSCFG_SetEXTISource(port, line);
#endif
}

static int gpio_apm32_get_exti_source(int pin)
{
    uint32_t line = gpio_apm32_pin_to_exti_line(pin);
    int port;

#ifdef CONFIG_SOC_SERIES_APM32F1X
    port = LL_GPIO_AF_GetEXTISource(line);
#elif CONFIG_SOC_SERIES_APM32MP1X
    port = LL_EXTI_GetEXTISource(line);
#elif defined(CONFIG_SOC_SERIES_APM32G0X) || defined(CONFIG_SOC_SERIES_APM32L5X) || defined(CONFIG_SOC_SERIES_APM32U5X)
    port = LL_EXTI_GetEXTISource(line);
#else
    port = LL_SYSCFG_GetEXTISource(line);
#endif

#if defined(CONFIG_SOC_SERIES_APM32L0X) && defined(LL_SYSCFG_EXTI_PORTH)
    /*
     * Ports F and G are not present on some APM32L0 parts, so
     * for these parts port H external interrupt is enabled
     * by writing value 0x5 instead of 0x7.
     */
    if (port == LL_SYSCFG_EXTI_PORTH)
    {
        port = APM32_PORTH;
    }
#endif

    return port;
}

/**
 * @brief Enable EXTI of the specific line
 */
static int gpio_apm32_enable_int(int port, int pin)
{
#if defined(CONFIG_SOC_SERIES_APM32F2X) || defined(CONFIG_SOC_SERIES_APM32F3X) ||                                      \
	defined(CONFIG_SOC_SERIES_APM32F4X) || defined(CONFIG_SOC_SERIES_APM32F7X) ||                                      \
	defined(CONFIG_SOC_SERIES_APM32H7X) || defined(CONFIG_SOC_SERIES_APM32L1X) ||                                      \
	defined(CONFIG_SOC_SERIES_APM32L4X) || defined(CONFIG_SOC_SERIES_APM32G4X)
    const struct device *const clk = DEVICE_DT_GET(APM32_CLOCK_CONTROL_NODE);
    struct apm32_pclken pclken = {
#ifdef CONFIG_SOC_SERIES_APM32H7X
        .bus = APM32_CLOCK_BUS_APB4,
        .enr = LL_APB4_GRP1_PERIPH_SYSCFG
#else
        .bus = APM32_CLOCK_BUS_APB2,
        .enr = LL_APB2_GRP1_PERIPH_SYSCFG
#endif /* CONFIG_SOC_SERIES_APM32H7X */
    };
    int ret;

    /* Enable SYSCFG clock */
    ret = clock_control_on(clk, (clock_control_subsys_t *)&pclken);
    if (ret != 0)
    {
        return ret;
    }
#endif

    gpio_apm32_set_exti_source(port, pin);

    return 0;
}

#endif

static int gpio_apm32_port_get_raw(const struct device *dev, uint32_t *value)
{
	const struct gpio_apm32_config *cfg = dev->config;
	GPIO_T *gpio = (GPIO_T *)cfg->base;

	*value = GPIO_ReadInputPort(gpio);

	return 0;
}

static int gpio_apm32_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask, gpio_port_value_t value)
{
	const struct gpio_apm32_config *cfg = dev->config;
	GPIO_T *gpio = (GPIO_T *)cfg->base;
	uint32_t port_value;

	port_value = GPIO_ReadOutputPort(gpio);
	GPIO_WriteOutputPort(gpio, (port_value & ~mask) | (mask & value));

	return 0;
}

static int gpio_apm32_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_apm32_config *cfg = dev->config;
	GPIO_T *gpio = (GPIO_T *)cfg->base;

#ifdef CONFIG_SOC_SERIES_APM32F1X
	WRITE_REG(gpio->BSC, pins);
#else
	WRITE_REG(gpio->BSCL, pins);
#endif /* #ifdef CONFIG_SOC_SERIES_APM32F1X */

	return 0;
}

static int gpio_apm32_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_apm32_config *cfg = dev->config;
	GPIO_T *gpio = (GPIO_T *)cfg->base;

#ifdef CONFIG_SOC_SERIES_APM32F1X
	WRITE_REG(gpio->BC, pins);
#else
	WRITE_REG(gpio->BSCH, pins);
#endif

	return 0;
}

static int gpio_apm32_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_apm32_config *cfg = dev->config;
	GPIO_T *gpio = (GPIO_T *)cfg->base;

	WRITE_REG(gpio->ODATA, READ_REG(gpio->ODATA) ^ pins);

	return 0;
}

#ifdef CONFIG_SOC_SERIES_APM32F1X
#define IS_GPIO_OUT GPIO_OUT
#else
#define IS_GPIO_OUT APM32_GPIO
#endif

#if 0
int gpio_apm32_configure(const struct device *dev, int pin, int conf, int func)
{
    int ret;

    ret = pm_device_runtime_get(dev);
    if (ret < 0)
    {
        return ret;
    }

    gpio_apm32_configure_raw(dev, pin, conf, func);

    if (func == IS_GPIO_OUT)
    {
        uint32_t gpio_out = conf & (APM32_ODR_MASK << APM32_ODR_SHIFT);

        if (gpio_out == APM32_ODR_1)
        {
            gpio_apm32_port_set_bits_raw(dev, BIT(pin));
        }
        else if (gpio_out == APM32_ODR_0)
        {
            gpio_apm32_port_clear_bits_raw(dev, BIT(pin));
        }
    }

    return pm_device_runtime_put(dev);
}
#endif

/**
 * @brief Configure pin
 */
static int gpio_apm32_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_apm32_config *cfg = dev->config;
	GPIO_Config_T gpio_config = {0};
	GPIO_T *gpio = (GPIO_T *)cfg->base;

	if ((flags & GPIO_OUTPUT) != 0)
	{
		if ((flags & GPIO_SINGLE_ENDED) != 0)
		{
			if (flags & GPIO_LINE_OPEN_DRAIN)
			{
#ifdef CONFIG_SOC_SERIES_APM32F1X
				gpio_config.mode = GPIO_MODE_OUT_OD;
#else
				gpio_config.mode = GPIO_MODE_OUT;
				gpio_config.otype = GPIO_OTYPE_OD;
#endif /* #ifdef CONFIG_SOC_SERIES_APM32F1X */
			}
			else
			{
				/* Output can't be open source */
				return -ENOTSUP;
			}
		}
		else
		{
#ifdef CONFIG_SOC_SERIES_APM32F1X
			gpio_config.mode = GPIO_MODE_OUT_PP;
#else
				gpio_config.mode = GPIO_MODE_OUT;
				gpio_config.otype = GPIO_OTYPE_PP;
#endif /* #ifdef CONFIG_SOC_SERIES_APM32F1X */
		}

		if ((flags & GPIO_PULL_UP) != 0)
		{
			;
		}
		else if ((flags & GPIO_PULL_DOWN) != 0)
		{
			;
		}
	}
	else if ((flags & GPIO_INPUT) != 0)
	{
		if ((flags & GPIO_PULL_UP) != 0)
		{
#ifdef CONFIG_SOC_SERIES_APM32F1X
			gpio_config.mode = GPIO_MODE_IN_PU;
#else
            gpio_config.mode = GPIO_MODE_IN;
            gpio_config.pupd = GPIO_PUPD_UP;
#endif /* #ifdef CONFIG_SOC_SERIES_APM32F1X */
		}
		else if ((flags & GPIO_PULL_DOWN) != 0)
		{
#ifdef CONFIG_SOC_SERIES_APM32F1X
			gpio_config.mode = GPIO_MODE_IN_PU;
#else
            gpio_config.mode = GPIO_MODE_IN;
            gpio_config.pupd = GPIO_PUPD_DOWN;
#endif /* #ifdef CONFIG_SOC_SERIES_APM32F1X */
		}
		else
		{
#ifdef CONFIG_SOC_SERIES_APM32F1X
			gpio_config.mode = GPIO_MODE_IN_PU;
#else
            gpio_config.mode = GPIO_MODE_IN;
            gpio_config.pupd = GPIO_PUPD_NOPULL;
#endif /* #ifdef CONFIG_SOC_SERIES_APM32F1X */
		}
	}
	else
	{
#ifdef CONFIG_SOC_SERIES_APM32F1X
		gpio_config.mode = GPIO_MODE_ANALOG;
#else
        gpio_config.mode = GPIO_MODE_AN;
#endif /* #ifdef CONFIG_SOC_SERIES_APM32F1X */
	}

	if ((flags & GPIO_OUTPUT) != 0)
	{
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0)
		{
			gpio_apm32_port_set_bits_raw(dev, BIT(pin));
		}
		else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0)
		{
			gpio_apm32_port_clear_bits_raw(dev, BIT(pin));
		}
	}

	gpio_config.pin = 1 << pin;
#ifdef CONFIG_SOC_SERIES_APM32F1X
	gpio_config.speed = GPIO_SPEED_10MHz;
#else
    gpio_config.speed = GPIO_SPEED_25MHz;
#endif /* #ifdef CONFIG_SOC_SERIES_APM32F1X */

	GPIO_Config(gpio, &gpio_config);

	return 0;
}

#if defined(CONFIG_GPIO_GET_CONFIG) && !defined(CONFIG_SOC_SERIES_APM32F1X)
/**
 * @brief Get configuration of pin
 */
static int gpio_apm32_get_config(const struct device *dev, gpio_pin_t pin, gpio_flags_t *flags)
{
	const struct gpio_apm32_config *cfg = dev->config;
	GPIO_T *gpio = (GPIO_T *)cfg->base;
	struct gpio_apm32_pin pin_config;
	int pin_ll;
	int err;

	err = pm_device_runtime_get(dev);
	if (err < 0)
	{
		return err;
	}

	pin_ll = apm32_pinval_get(pin);
	pin_config.type = LL_GPIO_GetPinOutputType(gpio, pin_ll);
	pin_config.pupd = LL_GPIO_GetPinPull(gpio, pin_ll);
	pin_config.mode = LL_GPIO_GetPinMode(gpio, pin_ll);
	pin_config.out_state = LL_GPIO_IsOutputPinSet(gpio, pin_ll);

	gpio_apm32_pincfg_to_flags(pin_config, flags);

	return pm_device_runtime_put(dev);
}
#endif /* CONFIG_GPIO_GET_CONFIG */

static int gpio_apm32_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin, enum gpio_int_mode mode,
											  enum gpio_int_trig trig)
{
	return -1;
#if 0
    const struct gpio_apm32_config *cfg = dev->config;
    struct gpio_apm32_data *data = dev->data;
    int edge = 0;
    int err = 0;

    if (mode == GPIO_INT_MODE_DISABLED)
    {
        if (gpio_apm32_get_exti_source(pin) == cfg->port)
        {
            apm32_exti_disable(pin);
            apm32_exti_unset_callback(pin);
            apm32_exti_trigger(pin, APM32_EXTI_TRIG_NONE);
        }
        /* else: No irq source configured for pin. Nothing to disable */
        goto exit;
    }

    /* Level trigger interrupts not supported */
    if (mode == GPIO_INT_MODE_LEVEL)
    {
        err = -ENOTSUP;
        goto exit;
    }

    if (apm32_exti_set_callback(pin, gpio_apm32_isr, data) != 0)
    {
        err = -EBUSY;
        goto exit;
    }

    gpio_apm32_enable_int(cfg->port, pin);

    switch (trig)
    {
    case GPIO_INT_TRIG_LOW:
        edge = APM32_EXTI_TRIG_FALLING;
        break;
    case GPIO_INT_TRIG_HIGH:
        edge = APM32_EXTI_TRIG_RISING;
        break;
    case GPIO_INT_TRIG_BOTH:
        edge = APM32_EXTI_TRIG_BOTH;
        break;
    }

    apm32_exti_trigger(pin, edge);

    apm32_exti_enable(pin);

exit:
    return err;
#endif
}

static int gpio_apm32_manage_callback(const struct device *dev, struct gpio_callback *callback, bool set)
{
	struct gpio_apm32_data *data = dev->data;

	return gpio_manage_callback(&data->cb, callback, set);
}

static const struct gpio_driver_api gpio_apm32_driver = {
	.pin_configure = gpio_apm32_pin_configure,
#if defined(CONFIG_GPIO_GET_CONFIG) && !defined(CONFIG_SOC_SERIES_APM32F1X)
	.pin_get_config = gpio_apm32_get_config,
#endif /* CONFIG_GPIO_GET_CONFIG */
	.port_get_raw = gpio_apm32_port_get_raw,
	.port_set_masked_raw = gpio_apm32_port_set_masked_raw,
	.port_set_bits_raw = gpio_apm32_port_set_bits_raw,
	.port_clear_bits_raw = gpio_apm32_port_clear_bits_raw,
	.port_toggle_bits = gpio_apm32_port_toggle_bits,
	.pin_interrupt_configure = gpio_apm32_pin_interrupt_configure,
	.manage_callback = gpio_apm32_manage_callback,
};

/**
 * @brief Initialize GPIO port
 */
static int gpio_apm32_init(const struct device *dev)
{
	const struct device *rcm = DEVICE_DT_GET(DT_NODELABEL(rcm));
	const struct gpio_apm32_config *cfg = (const struct gpio_apm32_config *)dev->config;
	struct gpio_apm32_data *data = dev->data;
	int ret;

	data->dev = dev;

	if (!device_is_ready(rcm))
	{
		return -ENODEV;
	}

	ret = clock_control_on(rcm, (clock_control_subsys_t *)&cfg->clk_ctrl);

	return ret;
}

#define GPIO_DEVICE_INIT(__node, __suffix, __base_addr, __port, __clk)                                                 \
	static const struct gpio_apm32_config gpio_apm32_cfg_##__suffix = {                                                \
		.common =                                                                                                      \
			{                                                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_NGPIOS(16U),                                                  \
			},                                                                                                         \
		.base = (uint32_t *)__base_addr,                                                                               \
		.port = __port,                                                                                                \
		.clk_ctrl = __clk};                                                                                            \
	static struct gpio_apm32_data gpio_apm32_data_##__suffix;                                                          \
	DEVICE_DT_DEFINE(__node, gpio_apm32_init, PM_DEVICE_DT_GET(__node), &gpio_apm32_data_##__suffix,                   \
					 &gpio_apm32_cfg_##__suffix, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY, &gpio_apm32_driver)

#define GPIO_DEVICE_INIT_APM32(__suffix, __SUFFIX)                                                                     \
	GPIO_DEVICE_INIT(DT_NODELABEL(gpio##__suffix), __suffix, DT_REG_ADDR(DT_NODELABEL(gpio##__suffix)),                \
					 APM32_PORT##__SUFFIX, DT_CLOCKS_CELL(DT_NODELABEL(gpio##__suffix), bits))

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpioa), okay)
GPIO_DEVICE_INIT_APM32(a, A);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpioa), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpiob), okay)
GPIO_DEVICE_INIT_APM32(b, B);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpiob), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpioc), okay)
GPIO_DEVICE_INIT_APM32(c, C);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpioc), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpiod), okay)
GPIO_DEVICE_INIT_APM32(d, D);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpiod), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpioe), okay)
GPIO_DEVICE_INIT_APM32(e, E);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpioe), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpiof), okay)
GPIO_DEVICE_INIT_APM32(f, F);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpiof), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpiog), okay)
GPIO_DEVICE_INIT_APM32(g, G);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpiog), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpioh), okay)
GPIO_DEVICE_INIT_APM32(h, H);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpioh), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpioi), okay)
GPIO_DEVICE_INIT_APM32(i, I);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpioi), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpioj), okay)
GPIO_DEVICE_INIT_APM32(j, J);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpioj), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpiok), okay)
GPIO_DEVICE_INIT_APM32(k, K);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpiok), okay) */
