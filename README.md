# STM32_ctrlmodule — 射频前端控制模块

8~18GHz 射频前端的配套控制模块，由 **STM32F103 控制板** 和 **Qt6 PC 上位机** 组成，通过 USB CDC 通信。

## 目录结构

```
├── firmware/          STM32F103 固件 (USB CDC + SPI → 74HC595)
├── software/          Qt6/C++ 上位机控制软件
├── hardware/          电路设计 (立创EDA)、BOM、Pick & Place
├── enclosure/         外壳 3D 模型 (SolidWorks)
└── docs/              项目文档
```

## 快速开始

### 上位机

```bash
cd software/pc_control
cmake -B build -DCMAKE_PREFIX_PATH=/path/to/Qt6
cmake --build build --config Release
./build/Release/rf_frontend_control.exe
```

依赖：Qt 6 (Core, Gui, Widgets, SerialPort)

### 固件

用 STM32CubeIDE 打开 `firmware/` 目录，或使用 CMake：

```bash
cd firmware
cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build build
```

## 通信协议

| 项目 | 说明 |
|------|------|
| 接口 | USB CDC (虚拟串口) |
| 数据帧 | 12 字节固定帧，96 位控制字 |
| 方向 | 双向：PC → 固件（控制字下发），固件 → PC（调试回显） |

96 位控制字经全位序反转后通过 SPI 驱动 12 片 74HC595，产生并行控制信号经三路 D-sub 37 芯连接器输出至各射频模块。

详见 `docs/96位数据位映射定义.md`。

## 许可证

MIT