# apm32f103vc_mini
# Copyright (c) 2023, Quincy.W <wangqyfm@foxmail.com>
# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=STM32F103VC" "--speed=4000")
board_runner_args(pyocd "--target=STM32F103VC" "--frequency=10000000")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
