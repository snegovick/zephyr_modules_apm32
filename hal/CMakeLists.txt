# Copyright (c) 2023, Quincy.W <wangqyfm@foxmail.com>
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

if(CONFIG_CPU_CORTEX_M4)
  zephyr_compile_definitions( -DCORE_CM4 )
elseif(CONFIG_CPU_CORTEX_M7)
  zephyr_compile_definitions( -DCORE_CM7 )
endif()

if(CONFIG_APM32F10X_LD)
  zephyr_compile_definitions(APM32F10X_LD)
elseif(CONFIG_APM32F10X_MD)
  zephyr_compile_definitions(APM32F10X_MD)
elseif(CONFIG_APM32F10X_HD)
  zephyr_compile_definitions(APM32F10X_HD)
elseif(CONFIG_APM32F10X_CL)
  zephyr_compile_definitions(APM32F10X_CL)
endif()

if(CONFIG_SOC_SERIES_APM32F1X)

set(SERIES_NAME_L apm32f10x)
set(STD_PERIPH_DRV_DIR APM32F10x_StdPeriphDriver)

elseif(CONFIG_SOC_SERIES_APM32F4X)

set(SERIES_NAME_L apm32f4xx)
set(STD_PERIPH_DRV_DIR APM32F4xx_StdPeriphDriver)

else()
message(ERROR "Invalid SERIES")
endif()

string(REPLACE "_StdPeriphDriver" "" SERIES_NAME "${STD_PERIPH_DRV_DIR}")

zephyr_include_directories(common)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../include)

zephyr_include_directories(
  ${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/inc
  ${SERIES_NAME_L}/Libraries/Device/Geehy/${SERIES_NAME}/Include
)

zephyr_library_sources(${SERIES_NAME_L}/Libraries/Device/Geehy/${SERIES_NAME}/Source/system_${SERIES_NAME_L}.c)
zephyr_library_sources(${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/src/${SERIES_NAME_L}_rcm.c)
zephyr_library_sources(${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/src/${SERIES_NAME_L}_misc.c)

zephyr_library_sources_ifdef(CONFIG_APM32_HAL_ADC     ${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/src/${SERIES_NAME_L}_adc.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_BAKPR   ${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/src/${SERIES_NAME_L}_bakpr.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_CAN     ${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/src/${SERIES_NAME_L}_can.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_CRC     ${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/src/${SERIES_NAME_L}_crc.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_DAC     ${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/src/${SERIES_NAME_L}_dac.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_DBGMCU  ${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/src/${SERIES_NAME_L}_dbgmcu.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_DMA     ${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/src/${SERIES_NAME_L}_dma.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_DMC     ${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/src/${SERIES_NAME_L}_dmc.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_EINT    ${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/src/${SERIES_NAME_L}_eint.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_FMC     ${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/src/${SERIES_NAME_L}_fmc.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_GPIO    ${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/src/${SERIES_NAME_L}_gpio.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_I2C     ${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/src/${SERIES_NAME_L}_i2c.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_IWDT    ${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/src/${SERIES_NAME_L}_iwdt.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_PMU     ${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/src/${SERIES_NAME_L}_pmu.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_QSPI    ${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/src/${SERIES_NAME_L}_qspi.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_RTC     ${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/src/${SERIES_NAME_L}_rtc.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_SCI2C   ${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/src/${SERIES_NAME_L}_sci2c.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_SDIO    ${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/src/${SERIES_NAME_L}_sdio.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_SMC     ${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/src/${SERIES_NAME_L}_smc.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_SPI     ${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/src/${SERIES_NAME_L}_spi.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_TMR     ${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/src/${SERIES_NAME_L}_tmr.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_USART   ${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/src/${SERIES_NAME_L}_usart.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_WWDT    ${SERIES_NAME_L}/Libraries/${STD_PERIPH_DRV_DIR}/src/${SERIES_NAME_L}_wwdt.c)


