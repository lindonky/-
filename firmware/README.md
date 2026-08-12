# ESP32-S3 三路电调控制固件

该目录是 AlwaysbeA 水下康复机械下肢控制系统的 ESP32-S3 固件，基于 ESP-IDF v5.1.2。

完整系统说明及安全注意事项见仓库根目录的 `README.md` 和 `docs/HANDOVER.md`。

## 主要功能

- 三路 ESC：50 Hz PWM，800–2200 μs，中位 1500 μs
- JY61P 姿态传感器：UART2 / GPIO6，9600 8N1
- 助力、主动、阻抗三种控制模式
- 三段式 PD 姿态稳定
- 闩锁式急停、IMU 超时和超角度保护
- Wi-Fi SoftAP、内嵌控制网页及 HTTP API
- UART0 控制台和 BLE Nordic UART Service

## 构建

安装 ESP-IDF v5.1.2 后，在 Windows PowerShell 或 cmd 中运行：

```powershell
.\build_idf.bat build
```

脚本优先使用环境中的 `IDF_PATH`；若未设置，会尝试开发机原始默认位置 `C:\Espressif\frameworks\esp-idf-v5.1.2`。

也可以先进入 ESP-IDF 命令环境，再直接运行：

```powershell
idf.py build
```

## 烧录

烧录前确认当前 USB-Serial-JTAG 的 COM 号。设备复位后端口号可能改变。

```powershell
.\build_idf.bat -p COM39 flash
```

CH340 端口主要用于 UART0 控制台；USB-Serial-JTAG 端口适合自动复位烧录。

## 接线

| 信号 | 引脚 |
|---|---:|
| ESC 1 / 左方向舵 | GPIO15 |
| ESC 2 / 推进 | GPIO16 |
| ESC 3 / 右方向舵 | GPIO14 |
| JY61P TX → ESP RX | GPIO6 |
| UART0 控制台 | 115200 8N1 |

三个电调与 ESP32 必须共地。电调红线悬空，不得接 ESP32 3.3 V。

## 网页控制

默认配置：

- 热点：`AlwaysbeA`
- 密码：`12345678`
- 地址：`http://192.168.4.1`

可在 `idf.py menuconfig` 中修改热点配置。部署到非实验环境前应更改默认密码。

HTTP 接口：

| 端点 | 方法 | 说明 |
|---|---|---|
| `/` | GET | 控制网页 |
| `/leg.js` | GET | 3D 腿部模型 |
| `/api/status` | GET | JSON 状态 |
| `/api/training/status` | GET | 单腿训练会话 JSON 摘要 |
| `/api/cmd` | POST | body 为文本命令 |

## 控制命令

所有传输共用相同的行文本协议：

```text
HELP
STATUS
START
STOP
MODE [assist|active|impedance]
T1|T2|T3 <speed>
TALL <speed>
JOY <x> <y>
STAB [ON|OFF|ZERO]
TRAIN
TRAIN START|PAUSE|RESUME|STOP|RESET
TRAIN HEIGHT <100..230 cm>
TRAIN SHANK <20..70 cm>   # 0 恢复身高比例估算
PULSE <n> <us>
CAL
```

速度范围为 `-1000..1000`。负值能否造成真实反转取决于电调是否支持双向运行。
主动模式强制中间推进器保持 1500 μs；`T2`、`TALL`、`JOY` 和 `PULSE`
均不能绕过该限制。计步在 ESP32 高频 IMU 路径中完成，一步定义为单腿一次
完整抬起—回位。身高/小腿长仅用于等效步长估算。

## 串口调试

```powershell
pip install pyserial
python tools/serial_ctl.py COM37
python tools/serial_ctl.py COM37 --send "STATUS"
```

JY61P 若有接收字节但有效帧始终为零，优先检查模块波特率是否为 9600。

## 关键默认配置

- 上电自动校准：关闭
- 双向脉宽：开启；当前测试电调的物理反转尚未验证成功
- 助力模式上限：800
- 主动模式主推进：关闭；左右纠偏上限：1000
- 阻抗模式上限：600
- HTTP 请求头缓冲：2048 字节
- Flash：16 MB，自定义 4 MB app 分区

## 安全

- `STOP` 会立即回中位并锁定，只有 `START` 才能重新使能。
- `CAL` 会短时输出满量程脉宽，执行前必须拆除负载并清空机械活动范围。
- 首次实机调参时必须限制机械行程，并随时准备切断硬件电源。
