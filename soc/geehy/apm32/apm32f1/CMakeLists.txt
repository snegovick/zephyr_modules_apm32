# Copyright (c) 2023, Quincy.W <wangqyfm@foxmail.com>
# Copyright (c) 2025, Excave.ru
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(${ZEPHYR_BASE}/drivers)
zephyr_include_directories(.)
zephyr_sources(
  soc.c
  )

set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld CACHE INTERNAL "")
