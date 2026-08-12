# AlwaysbeA 水下康复机械下肢控制系统

本仓库包含一套基于 ESP32-S3 的水下康复机械下肢控制系统：ESP32-S3 读取 JY61P 姿态传感器，控制三路水下电调，并通过 Wi-Fi 网页、Android App、UART 或 BLE UART 提供控制入口。

> 安全提示：这是会驱动电机和机械结构的实验性控制系统。首次测试、校准和调参时必须拆下螺旋桨或其他负载，并确保急停可随时操作。它不是经过认证的医疗器械控制器。

## 功能概览

- ESP32-S3 + ESP-IDF 5.1.2 固件
- 三路 50 Hz ESC PWM，脉宽范围 800–2200 μs，中位 1500 μs
- JY61P 姿态采集与三段式 PD 稳定控制
- 助力、主动、阻抗三种控制模式
- 闩锁式急停、IMU 超时保护、超角度保护
- ESP32 内嵌控制网页及实时 3D 姿态显示
- Android 7+ App：连接设备热点并打开 WebView 控制页
- UART、HTTP 和 BLE UART 共用一套文本指令协议

## 仓库结构

```text
.
├── firmware/       ESP32-S3 固件、内嵌网页和串口调试工具
├── android/        AlwaysbeA Android App
└── docs/           工程交接、接线、构建和安全说明
```

## 硬件接线

| 设备/信号 | ESP32-S3 引脚 | 备注 |
|---|---:|---|
| 左方向舵 ESC 信号 | GPIO15 | 50 Hz PWM |
| 推进 ESC 信号 | GPIO16 | 50 Hz PWM |
| 右方向舵 ESC 信号 | GPIO14 | 50 Hz PWM |
| JY61P TX | GPIO6 / UART2 RX | 9600 8N1 |
| UART 控制台 | UART0 / CH340 | 115200 8N1 |

三个电调必须与 ESP32 共地。电调红线悬空，不得连接 ESP32 的 3.3 V。电机和电调使用满足电流要求的独立电源。

## 快速构建固件

要求：Windows、ESP-IDF v5.1.2，以及 ESP32-S3 工具链。

ESP-IDF 5.1 在 Windows 上可能无法处理包含非 ASCII 字符的工程绝对路径。如果 CMake 显示乱码路径或找不到 `xtensa-esp32s3-elf-gcc`，请把仓库克隆到纯 ASCII 路径（例如 `C:\src\alwaysbea`）后重试。

```powershell
cd firmware
.\build_idf.bat build
```

烧录前在设备管理器确认当前 USB-Serial-JTAG COM 号：

```powershell
.\build_idf.bat -p COM39 flash
```

也可以先进入已配置好的 ESP-IDF 环境，然后在 `firmware` 目录直接执行：

```powershell
idf.py build
idf.py -p COM39 flash
```

## 快速构建 Android App

要求：JDK 17、Android SDK 34、Gradle 8.7。

```powershell
cd android
gradle assembleRelease
```

未签名产物位于 `android/app/build/outputs/apk/release/`。正式发布需使用自行保管的签名密钥；签名文件和密码不得提交到仓库。

## 使用

固件启动后默认创建以下热点：

- SSID：`AlwaysbeA`
- 默认密码：`12345678`
- 控制页：`http://192.168.4.1`

实际部署前建议在 `idf.py menuconfig` 的通信配置中修改默认热点密码。

常用控制命令：

```text
HELP
STATUS
START
STOP
MODE assist|active|impedance
T1|T2|T3 -1000..1000
TALL -1000..1000
JOY x y
STAB ON|OFF|ZERO
PULSE n us
CAL
```

详细说明见 [docs/HANDOVER.md](docs/HANDOVER.md) 和 [firmware/README.md](firmware/README.md)。

## 安全规则

- `STOP` 是闩锁式急停；急停后必须执行 `START` 才会重新允许输出。
- `CAL` 会短时输出满量程校准脉宽，只能在电调无负载时执行。
- IMU 数据超时超过 200 ms 或姿态超过配置角度时，稳定控制停止输出。
- 阻抗模式的物理反转要求支持双向运行的电调；不能把发送了反向脉宽等同于电机一定安全反转。
- 在真实人体附近运行前，应增加独立硬件急停、电源隔离、机械限位和故障验证。

## 版本状态

本次入库基线来自 2026-08-12 已完成并实测的三模式、急停修复、内嵌网页和 Android 连接版本。当前主要后续工作是实机轴向确认、稳定参数调试和不同负载下的模式输出上限标定。

本仓库暂未声明开源许可证。除非仓库所有者另行添加许可证，否则代码的使用、复制和分发权利不作授予。
