# AlwaysbeA 单腿康复设备引脚与接线说明

> 文档日期：2026-08-13  
> 适用对象：当前 ESP32-S3 QFN56 / 8 MB PSRAM 单腿原型，以及已烧录的 `single-leg-mvp-0.3` 固件
> 当前固件镜像 SHA-256：`6D7B36327372B98803DFFBB1F744C4A4CC4C2F5CDF085E0BA8AC83CB30699643`

本文以当前仓库的 `firmware/sdkconfig`、BSP 驱动和 ESP32-S3 芯片固定接口为准。表中的 GPIO 是 ESP32-S3 芯片 GPIO 编号，不是某块开发板丝印的排针序号；实际排针位置必须再对照所用开发板的正反面丝印或原理图。

## 1. 一页接线总表

| 外部设备/信号 | 外设方向 | ESP32-S3 端 | 当前参数 | 必需性 |
|---|---|---|---|---|
| 左侧纠偏电机 ESC，T1 信号 | ESP → ESC | `GPIO15` | LEDC CH0，50 Hz，800–2200 μs，中位 1500 μs | 必需 |
| 中间推进器 ESC，T2 信号 | ESP → ESC | `GPIO16` | LEDC CH1，50 Hz，800–2200 μs，中位 1500 μs | 必需 |
| 右侧纠偏电机 ESC，T3 信号 | ESP → ESC | `GPIO14` | LEDC CH2，50 Hz，800–2200 μs，中位 1500 μs | 必需 |
| JY61P TX 数据输出 | JY61P → ESP | `GPIO6` / UART2 RX | 9600 baud，8N1 | 必需 |
| JY61P RX 配置输入 | ESP → JY61P | `GPIO5` / UART2 TX | 9600 baud，8N1 | 可选，当前业务代码未发送配置 |
| CH340 RX | ESP → CH340 | `GPIO43` / UART0 TX | 115200 baud，8N1 | 调试用 |
| CH340 TX | CH340 → ESP | `GPIO44` / UART0 RX | 115200 baud，8N1 | 调试用 |
| 原生 USB D− | 双向 USB | `GPIO19` | ESP32-S3 固定 USB 信号 | 烧录/USB 调试 |
| 原生 USB D+ | 双向 USB | `GPIO20` | ESP32-S3 固定 USB 信号 | 烧录/USB 调试 |
| BOOT 下载选择 | 按键/下载电路 → ESP | `GPIO0` | 复位时拉低进入下载模式 | 板级功能 |
| RESET/EN | 按键/下载电路 → ESP | `EN/CHIP_PU`，不是普通 GPIO | 拉低复位，释放后启动 | 板级功能 |
| 所有设备逻辑地 | 公共参考 | `GND` | ESP、JY61P、三路 ESC 信号地必须共地 | 必需 |

当前应用没有定义外接 I²C、SPI、ADC、压力传感器、力传感器、电流传感器、蜂鸣器、外部急停输入、状态 LED 或普通按键 GPIO。不能因为代码中没有使用某个 GPIO，就直接认定开发板排针一定空闲：Flash/PSRAM、启动绑带电阻、USB 和板载器件仍可能占用或限制部分引脚。

## 2. 三路 ESC / 水下动力单元

### 2.1 信号映射

| 固件名称 | 机械含义 | GPIO | LEDC 通道 | 正常中位 |
|---|---|---:|---:|---:|
| `T1` / ESC 1 | 左侧横向纠偏电机 | 15 | 0 | 1500 μs |
| `T2` / ESC 2 | 中间抬腿推进器 | 16 | 1 | 1500 μs |
| `T3` / ESC 3 | 右侧横向纠偏电机 | 14 | 2 | 1500 μs |

三路 PWM 共用 LEDC 低速定时器 0：

```text
频率           50 Hz
周期           20,000 μs
允许脉宽       800–2,200 μs
停止/中位      1,500 μs
逻辑输出电平   ESP32-S3 的 3.3 V GPIO 电平
```

模式与电机用途：

- 助力训练：T2 根据抬腿主轴输出正向助力；T1/T3 根据横向角度做差动纠偏。
- 主动训练：T2 强制保持 1500 μs，不提供抬腿助推；T1/T3 只负责扶正腿位。
- 阻抗训练：T2 使用反向控制逻辑；真实反转必须由双向电调和水下台架验证。当前代码支持反向命令不等于现有电调已经能够物理反转。

### 2.2 每个 ESC 的三类线

典型舵机式 ESC 控制插头包含信号、地和 BEC 电源三根线，颜色不能作为唯一依据，应以电调标签和说明书为准：

| ESC 线 | 接法 |
|---|---|
| 信号线 | 分别接 `GPIO15`、`GPIO16`、`GPIO14` |
| GND/信号地 | 接 ESP32 `GND`，三路必须共地 |
| 红线/BEC 电源 | 当前工程说明要求悬空，不接 ESP32 `3V3`；若未来使用 BEC 给控制板供电，必须先完成电压、并联和反灌电评估 |

严禁把电机主电源或电池直接接到 ESP32 的 `3V3`/GPIO。不要在没有确认电调 BEC 类型的情况下把三路红线并联；多个 BEC 并联可能互相反灌。电机三相线或电机正负动力线只连接电调与电机，不连接 ESP32。

## 3. JY61P 小腿姿态传感器

### 3.1 UART 接线

```text
JY61P TX  ─────────>  ESP32 GPIO6  / UART2 RX
JY61P RX  <─────────  ESP32 GPIO5  / UART2 TX（可选）
JY61P GND ──────────  ESP32 GND
JY61P VCC ──────────  按模块标签/说明书提供的电源，不接 GPIO
```

| 项目 | 当前值 |
|---|---|
| UART 控制器 | UART2 |
| ESP RX | GPIO6 |
| ESP TX | GPIO5 |
| 波特率 | 9600 |
| 数据格式 | 8 数据位、无校验、1 停止位（8N1） |
| 流控 | 无 |
| 接收缓冲 | 1024 字节 |

当前固件持续使用 GPIO6 接收 JY61P 数据。GPIO5 已配置为 UART2 TX，预留给读取或修改 JY61P 配置，但当前业务流程没有向 JY61P 发送配置命令，因此只做姿态采集时可以不接 JY61P RX。

JY61P 的具体供电电压应以手中模块丝印和厂家资料为准。ESP32-S3 GPIO 不是 5 V 容忍输入；如果 JY61P TX 的逻辑高电平可能达到 5 V，必须先测量并增加合适的电平转换或分压，不能直接接 GPIO6。

### 3.2 安装方向与软件轴

- 安装位置：单条腿的小腿中部。
- 当前抬腿主轴：JY61P `pitch` / Y 轴角度与 Y 轴角速度。
- 当前横向偏移轴：JY61P `roll` / X 轴角度与 X 轴角速度。
- 当前 `pitch`、`roll` 符号配置均为 `+1`。
- 实物安装后仍要验证“向前抬腿”和“向左/向右偏移”对应的正负方向；如相反，应修改轴交换/符号配置，不要通过对调 ESC 电源线来修正传感器方向。

## 4. CH340 UART0 调试接口

ESP32-S3 UART0 的芯片默认引脚为：

```text
ESP32 GPIO43 / U0TXD ─────> CH340 RXD
ESP32 GPIO44 / U0RXD <───── CH340 TXD
ESP32 GND            ────── CH340 GND
```

| 项目 | 当前值 |
|---|---|
| UART 控制器 | UART0 |
| ESP TX | GPIO43 |
| ESP RX | GPIO44 |
| 波特率 | 115200 |
| 数据格式 | 8N1，无流控 |
| Windows 当前枚举 | `COM37`，VID/PID `1A86:7523` |

COM 号是 Windows 动态分配结果，不属于硬件引脚，重新插拔或换 USB 口后可能变化。本机最近一次烧录后，CH340 已枚举但没有回读到应用命令文本；因此 UART0 的芯片引脚和固件配置是确定的，CH340 到实体板的 TX/RX/自动下载电路仍应结合原理图或万用表继续核对。

调试时优先使用：

```powershell
python firmware/tools/serial_ctl.py COM37 --send "STATUS" -t 5
```

该工具会在打开端口前关闭 DTR/RTS，减少自动下载电路把芯片复位进下载模式的风险。

## 5. 原生 USB-Serial/JTAG

ESP32-S3 的原生 USB 引脚是芯片固定功能：

| USB 信号 | ESP32-S3 GPIO | 当前用途 |
|---|---:|---|
| USB D− | GPIO19 | USB-Serial/JTAG、烧录 |
| USB D+ | GPIO20 | USB-Serial/JTAG、烧录 |
| USB VBUS | 开发板 USB 供电/检测电路 | 不是普通 GPIO |
| USB GND | GND | 公共地 |

本机最近一次枚举为 `COM39`，VID/PID `303A:1001`，芯片 MAC `E0:72:A1:F7:0A:B4`。该 COM 号同样可能变化。

当前固件启用了 USB-Serial/JTAG 作为辅助控制台，但普通串口软件打开原生 USB 端口时可能切换 DTR/RTS 并触发 USB 复位/下载模式。烧录使用 `COM39`；普通命令调试优先使用 CH340。GPIO19/20 不得同时分配给电机、传感器或其他普通 GPIO 功能，否则会破坏原生 USB。

## 6. BOOT、RESET 与启动状态

| 名称 | 芯片信号 | 作用 |
|---|---|---|
| BOOT | GPIO0 | 芯片复位瞬间拉低时进入 ROM 下载模式；正常启动时应释放为高 |
| RESET / EN | `CHIP_PU` | 拉低复位芯片；它不是可由应用随意使用的 GPIO |

如果串口显示 `DOWNLOAD(USB/UART0)` 和 `waiting for download`，通常表示 GPIO0 在复位时被拉低，并不等于应用固件损坏。释放 BOOT 后短按 RESET/EN，或者用 esptool 的 `--after hard_reset`，即可重新从 Flash 启动。

## 7. 电源、地线与水下安全

1. ESP32、JY61P 和三路 ESC 的信号地必须共地，否则 PWM 和 UART 没有可靠电平参考。
2. ESP32 GPIO 只传输逻辑信号，不能给电机、推进器或大功率负载供电。
3. 当前文档无法从固件判断开发板具体 `5V/VIN/3V3` 排针位置，也无法确认 JY61P 模块电源版本、ESC BEC 输出电压和三路动力配电；这些项目必须以实体丝印、原理图和万用表测量为准。
4. ESP32-S3 GPIO 不耐 5 V。连接到 GPIO5、GPIO6、GPIO14、GPIO15、GPIO16、GPIO43、GPIO44 的外部电路都必须满足 3.3 V GPIO 的电气安全范围。
5. 水下使用时，动力电源、逻辑电源、USB、串口和传感器接头需要独立的防水、应力释放和漏电保护设计。USB 接线调试和带电水下测试不能同时进行。
6. 当前急停是由网页、UART、BLE 等命令触发的固件闩锁，没有独立硬件急停输入 GPIO。正式载人或水下测试前，应增加能直接切断动力电源的硬件急停，不能只依赖 ESP32、无线链路或软件按钮。

## 8. 未定义或仍需现场确认的连接

| 项目 | 当前状态 |
|---|---|
| 独立硬件急停输入 | 未定义 GPIO，也没有驱动代码 |
| 电池电压/母线电流检测 | 未定义 ADC 引脚，没有传感器 |
| 电机转速/推力/力矩检测 | 未定义引脚，没有传感器 |
| 足底压力或负重传感器 | 未定义引脚，没有传感器 |
| 第二个 IMU/大腿 IMU | 未定义 UART/I²C 引脚 |
| 蜂鸣器、振动器、状态灯 | 未定义 GPIO |
| 外接显示屏或触摸屏 | 未定义 SPI/I²C/并口引脚，当前 UI 在手机 WebView 中 |
| CH340 的 DTR/RTS 到 EN/GPIO0 电路 | 固件无法确认，需要开发板原理图或实测 |
| 开发板 `VIN/5V/3V3` 物理排针位置 | 取决于具体开发板，需看丝印/原理图 |
| Flash/PSRAM 物理信号 | 属于芯片/模组内部与板级资源，不得根据“应用未用”自行复用 |

## 9. 修改引脚时的唯一入口

三路 ESC 和 JY61P 引脚均由 ESP-IDF Kconfig 控制。修改时应使用：

```powershell
cd firmware
.\build_idf.bat menuconfig
```

然后进入相应的 BSP/ESC Control 配置，不要只改文档：

```text
CONFIG_ESC_MOTOR_GPIO_1=15
CONFIG_ESC_MOTOR_GPIO_2=16
CONFIG_ESC_MOTOR_GPIO_3=14
CONFIG_ESC_IMU_UART_RX_GPIO=6
CONFIG_ESC_IMU_UART_TX_GPIO=5
CONFIG_ESC_IMU_UART_BAUD=9600
CONFIG_ESC_CONSOLE_BAUD=115200
```

UART0 的 GPIO43/44 和原生 USB 的 GPIO19/20 是当前芯片默认/固定接口，不由上述项目 Kconfig 重新映射。每次改针脚后至少要重新执行完整构建、检查 GPIO 冲突，并在不连接电机动力的条件下先验证 PWM/UART。
