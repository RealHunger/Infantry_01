# Infantry_01

`Infantry_01` 是一个基于 STM32F407 的步兵机器人控制固件工程。

这个仓库不是桌面程序，不能像普通应用一样双击运行。
它的基本使用流程是：

1. 安装交叉编译工具链
2. 使用 CMake + Ninja 编译固件
3. 通过调试器把生成的固件下载到 STM32F407 开发板/控制板
4. 上电后由 FreeRTOS 启动各任务，进入机器人控制逻辑

## 目录说明

- `Application/`：全局控制状态、机器人业务逻辑、自瞄接口
- `ALL_Task/`：FreeRTOS 任务入口，如云台、底盘、电机、传感器任务
- `Bsp/`：板级支持代码，包含 CAN、UART、USB CDC、LED 等
- `Components/`：电机、遥控器、BMI088、IST8310 等组件驱动
- `Algorithm/`：算法模块，例如 Mahony AHRS
- `Core/`：STM32CubeMX 生成的启动和外设初始化代码
- `USB_DEVICE/`：USB 设备相关代码
- `cmake/`：CMake 工具链和 STM32CubeMX 集成脚本

## 环境要求

在开始前，请确保本机安装了以下工具：

- `cmake` 3.22 或更高
- `ninja`
- ARM 交叉编译工具链：
  - `arm-none-eabi-gcc`
  - `arm-none-eabi-g++`
  - `arm-none-eabi-objcopy`
  - `arm-none-eabi-size`

这些命令需要已经加入系统 `PATH`。

## 如何编译

仓库已经配置好了 CMake Presets，推荐直接使用 Debug 配置。

### 1. 查看可用配置

```sh
cmake --list-presets
```

### 2. 生成 Debug 构建目录

```sh
cmake --preset Debug
```

### 3. 编译固件

```sh
cmake --build --preset Debug
```

编译成功后，主要输出位于：

- `build/Debug/`

常见产物包括：

- `Infantry_01.elf`
- map 文件
- `compile_commands.json`

如果你需要其他构建类型，也可以使用：

```sh
cmake --preset Release
cmake --build --preset Release
```

或者：

```sh
cmake --preset RelWithDebInfo
cmake --build --preset RelWithDebInfo
```

## 如何“运行”这个工程

对于这个项目，“运行”通常指把编译好的固件烧录到 STM32F407 板子上，然后让板子上电执行。

也就是说：

- 电脑上负责的是编译和下载
- 真正执行代码的是单片机本体

程序入口在 `Core/Src/main.c`：

- 调用 `HAL_Init()`
- 初始化时钟和外设
- 调用 `Robot_Global_Init()`
- 调用 `MX_FREERTOS_Init()`
- 启动 `osKernelStart()`

之后各个 FreeRTOS 任务开始运行，例如云台、底盘、电机和传感器任务。

## 烧录到开发板

仓库中已经提供了一个简单的 `openocd.cfg`，目标是 STM32F4，并使用 CMSIS-DAP 接口。

如果你本机安装了 OpenOCD，可以尝试：

```sh
openocd -f openocd.cfg
```

如果你想直接下载编译结果，常见做法如下：

```sh
openocd -f openocd.cfg -c "program build/Debug/Infantry_01.elf verify reset exit"
```

说明：

- 这条命令依赖你已经先完成 `cmake --build --preset Debug`
- 调试器接口配置来自仓库中的 `openocd.cfg`
- 如果你使用的不是 CMSIS-DAP，而是 ST-Link/J-Link，可能需要改 OpenOCD 配置

## 常见开发流程

推荐使用下面这套流程：

```sh
cmake --preset Debug
cmake --build --preset Debug
openocd -f openocd.cfg -c "program build/Debug/Infantry_01.elf verify reset exit"
```

如果只是改了少量代码，通常只需要重新执行：

```sh
cmake --build --preset Debug
```

## 调试建议

- 优先使用 `Debug` 预设进行开发
- 构建失败时先确认 `arm-none-eabi-*` 工具链是否在 `PATH`
- 如果下载失败，先检查调试器连接、供电和驱动
- 如果代码改动涉及 `Core/` 或 `USB_DEVICE/`，注意这些目录里有 CubeMX 生成内容
- 如果修改任务逻辑，建议重点检查 `ALL_Task/`、`Application/` 和 `Components/`

## 测试说明

当前仓库没有配置自动化测试框架。

也就是说，目前没有：

- `ctest`
- Unity/Ceedling
- 单元测试目标
- 单个测试用例运行命令

因此，当前最接近“测试”的验证方式是重新编译：

```sh
cmake --build --preset Debug
```

如果后续引入自动化测试，可以再补充：

- 如何运行全部测试
- 如何运行单个测试
- 如何查看失败输出

## 代码修改建议

- 优先修改 `Application/`、`ALL_Task/`、`Bsp/`、`Components/`、`Algorithm/`
- 尽量不要随意改 `Drivers/` 和 `Middlewares/` 中的第三方代码
- 修改 `Core/` 中的生成文件时，尽量放在 `USER CODE BEGIN/END` 标记内
- 任何影响外设初始化、CAN/UART 协议、任务调度的改动，都建议重新完整编译验证

## 一句话总结

如果你只是想把这个项目跑起来，最短路径就是：

```sh
cmake --preset Debug
cmake --build --preset Debug
openocd -f openocd.cfg -c "program build/Debug/Infantry_01.elf verify reset exit"
```

然后给板子上电，让 STM32 真正执行固件。
