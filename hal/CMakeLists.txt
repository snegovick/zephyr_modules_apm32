# Copyright (c) 2023, Quincy.W <wangqyfm@foxmail.com>
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

if(CONFIG_CPU_CORTEX_M4)
  zephyr_compile_definitions( -DCORE_CM4 )
elseif(CONFIG_CPU_CORTEX_M7)
  zephyr_compile_definitions( -DCORE_CM7 )
endif()

zephyr_include_directories(common)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../include)

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
zephyr_include_directories(
  apm32f10x/Libraries/APM32F10x_StdPeriphDriver/inc
  apm32f10x/Libraries/Device/Geehy/APM32F10x/Include
)

zephyr_library_sources(apm32f10x/Libraries/Device/Geehy/APM32F10x/Source/system_apm32f10x.c)
zephyr_library_sources(apm32f10x/Libraries/APM32F10x_StdPeriphDriver/src/apm32f10x_rcm.c)
zephyr_library_sources(apm32f10x/Libraries/APM32F10x_StdPeriphDriver/src/apm32f10x_misc.c)

zephyr_library_sources_ifdef(CONFIG_APM32_HAL_ADC apm32f10x/Libraries/APM32F10x_StdPeriphDriver/src/apm32f10x_adc.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_BAKPR apm32f10x/Libraries/APM32F10x_StdPeriphDriver/src/apm32f10x_bakpr.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_CAN apm32f10x/Libraries/APM32F10x_StdPeriphDriver/src/apm32f10x_can.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_CRC apm32f10x/Libraries/APM32F10x_StdPeriphDriver/src/apm32f10x_crc.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_DAC apm32f10x/Libraries/APM32F10x_StdPeriphDriver/src/apm32f10x_dac.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_DBGMCU apm32f10x/Libraries/APM32F10x_StdPeriphDriver/src/apm32f10x_dbgmcu.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_DMA apm32f10x/Libraries/APM32F10x_StdPeriphDriver/src/apm32f10x_dma.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_DMC apm32f10x/Libraries/APM32F10x_StdPeriphDriver/src/apm32f10x_dmc.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_EINT apm32f10x/Libraries/APM32F10x_StdPeriphDriver/src/apm32f10x_eint.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_FMC apm32f10x/Libraries/APM32F10x_StdPeriphDriver/src/apm32f10x_fmc.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_GPIO apm32f10x/Libraries/APM32F10x_StdPeriphDriver/src/apm32f10x_gpio.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_I2C apm32f10x/Libraries/APM32F10x_StdPeriphDriver/src/apm32f10x_i2c.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_IWDT apm32f10x/Libraries/APM32F10x_StdPeriphDriver/src/apm32f10x_iwdt.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_PMU apm32f10x/Libraries/APM32F10x_StdPeriphDriver/src/apm32f10x_pmu.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_QSPI apm32f10x/Libraries/APM32F10x_StdPeriphDriver/src/apm32f10x_qspi.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_RTC apm32f10x/Libraries/APM32F10x_StdPeriphDriver/src/apm32f10x_rtc.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_SCI2C apm32f10x/Libraries/APM32F10x_StdPeriphDriver/src/apm32f10x_sci2c.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_SDIO apm32f10x/Libraries/APM32F10x_StdPeriphDriver/src/apm32f10x_sdio.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_SMC apm32f10x/Libraries/APM32F10x_StdPeriphDriver/src/apm32f10x_smc.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_SPI apm32f10x/Libraries/APM32F10x_StdPeriphDriver/src/apm32f10x_spi.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_TMR apm32f10x/Libraries/APM32F10x_StdPeriphDriver/src/apm32f10x_tmr.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_USART apm32f10x/Libraries/APM32F10x_StdPeriphDriver/src/apm32f10x_usart.c)
zephyr_library_sources_ifdef(CONFIG_APM32_HAL_WWDT apm32f10x/Libraries/APM32F10x_StdPeriphDriver/src/apm32f10x_wwdt.c)

endif()

