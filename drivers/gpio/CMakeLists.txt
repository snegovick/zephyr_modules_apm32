# Copyright (c) 2023, Quincy.W <wangqyfm@foxmail.com>
# Copyright (c) 2025, Excave.ru
# SPDX-License-Identifier: Apache-2.0

zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_GPIO_APM32 gpio_apm32.c)
