# Copyright (c) 2023, Quincy.W <wangqyfm@foxmail.com>
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_UART_APM32 uart_apm32.c)