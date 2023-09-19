Zephyr-RTOS 在极海 APM32 系列 MCU 的移植

# 仓库概况

这个仓库的内容是整理本人之前工作中的积累，使之尽可能符合 Zephyr 官方文档中对 [Module](https://docs.zephyrproject.org/3.4.0/develop/modules.html) 的要求。

请注意：
1. 仓库只是一个包含了基本功能的框架，MCU 的许多功能没有实现。 
2. 仓库的内容暂时只能保证编译通过，实际功能可能存在 Bug，使用时请自行确认功能完整。

因为使用本仓库导致的一切问题，本人概不负责。

# 使用

## 兼容性

目前是基于 3.4.0 版本，在 Windows 下进行开发测试，其他版本（环境）可能存在兼容性问题。

## 提供的内容
- Board
  - apm32f103vc_mini
- HAL
  - apm32f10x
- Driver
  - clock_control
  - flash
  - gpio
  - ...

## 使用仓库

克隆或下载本仓库代码，构建项目时在 west 编译命令后追加 **ZEPHYR_EXTRA_MODULES** 参数就可以启用仓库的内容。ZEPHYR_EXTRA_MODULES 参数是本仓库在编译机器上的目录路径（绝对路径）。例如：

```
west build -p -b apm32f103vc_mini C:\Users\user\zephyrproject\zephyr\samples\basic\blinky -- -DZEPHYR_EXTRA_MODULES="C:\Users\user\zephyrproject\zephyr_modules_apm32"
```

## 一些问题

本仓库里提供的驱动存在于 Zephyr 源码之外，构建/编译时会出现如下类似的告警：
```
CMake Warning at C:/Users/user/zephyrproject/zephyr/CMakeLists.txt:870 (message):
  No SOURCES given to Zephyr library: drivers__clock_control

  Excluding target from build.
```

# 其他

如果你对这个项目感兴趣，可以联系本人加入本项目，贡献自己的力量。

如果觉得项目对你有帮助，也可以联系本人，给予赞赏或捐赠~

