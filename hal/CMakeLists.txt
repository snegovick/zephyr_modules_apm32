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

set(PARENT_DIR apm32f10x)
set(SERIES_DIR APM32F10x_StdPeriphDriver)

elseif(CONFIG_SOC_SERIES_APM32F4X)

set(PARENT_DIR apm32f4xx)
set(SERIES_DIR APM32F4xx_StdPeriphDriver)

else()
message(ERROR "Invalid SERIES")
endif()


zephyr_include_directories(common)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../include)

zephyr_include_directories(
  ${PARENT_DIR}/Libraries/${SERIES_DIR}/inc
  ${PARENT_DIR}/Libraries/Device/Geehy/${PARENT_DIR}/Include
)

zephyr_library_sources(${PARENT_DIR}/Libraries/Device/Geehy/${PARENT_DIR}/Source/system_${PARENT_DIR}.c)
zephyr_library_sources(${PARENT_DIR}/Libraries/${SERIES_DIR}/src/${PARENT_DIR}_rcm.c)
zephyr_library_sources(${PARENT_DIR}/Libraries/${SERIES_DIR}/src/${PARENT_DIR}_misc.c)

zephyr_library_sources_ifdef(CONFIG_APM32_HAL_ADC     ${PARENT_DIR}/Libraries/${SERIES_DIR}/src/${PARENT_DIR}_adc.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_BAKPR   ${PARENT_DIR}/Libraries/${SERIES_DIR}/src/${PARENT_DIR}_bakpr.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_CAN     ${PARENT_DIR}/Libraries/${SERIES_DIR}/src/${PARENT_DIR}_can.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_CRC     ${PARENT_DIR}/Libraries/${SERIES_DIR}/src/${PARENT_DIR}_crc.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_DAC     ${PARENT_DIR}/Libraries/${SERIES_DIR}/src/${PARENT_DIR}_dac.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_DBGMCU  ${PARENT_DIR}/Libraries/${SERIES_DIR}/src/${PARENT_DIR}_dbgmcu.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_DMA     ${PARENT_DIR}/Libraries/${SERIES_DIR}/src/${PARENT_DIR}_dma.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_DMC     ${PARENT_DIR}/Libraries/${SERIES_DIR}/src/${PARENT_DIR}_dmc.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_EINT    ${PARENT_DIR}/Libraries/${SERIES_DIR}/src/${PARENT_DIR}_eint.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_FMC     ${PARENT_DIR}/Libraries/${SERIES_DIR}/src/${PARENT_DIR}_fmc.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_GPIO    ${PARENT_DIR}/Libraries/${SERIES_DIR}/src/${PARENT_DIR}_gpio.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_I2C     ${PARENT_DIR}/Libraries/${SERIES_DIR}/src/${PARENT_DIR}_i2c.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_IWDT    ${PARENT_DIR}/Libraries/${SERIES_DIR}/src/${PARENT_DIR}_iwdt.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_PMU     ${PARENT_DIR}/Libraries/${SERIES_DIR}/src/${PARENT_DIR}_pmu.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_QSPI    ${PARENT_DIR}/Libraries/${SERIES_DIR}/src/${PARENT_DIR}_qspi.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_RTC     ${PARENT_DIR}/Libraries/${SERIES_DIR}/src/${PARENT_DIR}_rtc.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_SCI2C   ${PARENT_DIR}/Libraries/${SERIES_DIR}/src/${PARENT_DIR}_sci2c.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_SDIO    ${PARENT_DIR}/Libraries/${SERIES_DIR}/src/${PARENT_DIR}_sdio.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_SMC     ${PARENT_DIR}/Libraries/${SERIES_DIR}/src/${PARENT_DIR}_smc.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_SPI     ${PARENT_DIR}/Libraries/${SERIES_DIR}/src/${PARENT_DIR}_spi.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_TMR     ${PARENT_DIR}/Libraries/${SERIES_DIR}/src/${PARENT_DIR}_tmr.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_USART   ${PARENT_DIR}/Libraries/${SERIES_DIR}/src/${PARENT_DIR}_usart.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_WWDT    ${PARENT_DIR}/Libraries/${SERIES_DIR}/src/${PARENT_DIR}_wwdt.c)


