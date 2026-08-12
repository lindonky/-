#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ESP32-S3 电调串口控制/调试助手（运行在电脑端）。

用法示例：
  python serial_ctl.py COM5                 # 交互模式：输入命令，实时显示设备输出
  python serial_ctl.py COM5 --send "fwd 500"   # 发一条命令，读 3 秒输出后退出
  python serial_ctl.py COM5 --log out.txt   # 交互模式同时把输出存到文件

设备端命令：fwd/rev/set/stop/pulse/status/cal/help
"""
import argparse
import threading
import time

import serial


def read_loop(ser, log_file, stop_event):
    while not stop_event.is_set():
        try:
            data = ser.read(256)
        except serial.SerialException:
            break
        if data:
            text = data.decode("utf-8", errors="replace")
            print(text, end="", flush=True)
            if log_file:
                log_file.write(text)
                log_file.flush()


def main():
    ap = argparse.ArgumentParser(description="ESP32-S3 ESC serial console helper")
    ap.add_argument("port", help="串口号，例如 COM5")
    ap.add_argument("-b", "--baud", type=int, default=115200, help="波特率，默认 115200")
    ap.add_argument("--send", help="发送单条命令后读取输出一段时间并退出")
    ap.add_argument("-t", "--time", type=float, default=3.0, help="--send 后读取输出的秒数")
    ap.add_argument("--log", help="同时把输出追加保存到该文件")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    # 不要驱动 DTR/RTS：很多板子的 CH340 复位电路接到 EN/GPIO0，
    # 打开串口时默认拉高 DTR/RTS 会把芯片带进下载模式。
    ser.dtr = False
    ser.rts = False
    log_file = open(args.log, "a", encoding="utf-8") if args.log else None
    stop_event = threading.Event()

    if args.send:
        ser.write((args.send + "\r\n").encode())
        threading.Thread(target=read_loop, args=(ser, log_file, stop_event),
                         daemon=True).start()
        time.sleep(args.time)
        stop_event.set()
    else:
        print(f"已连接 {args.port} @ {args.baud}。输入命令回车发送（fwd 500 / rev 300 / "
              f"stop / status ...），Ctrl+C 退出。")
        threading.Thread(target=read_loop, args=(ser, log_file, stop_event),
                         daemon=True).start()
        try:
            while True:
                line = input()
                if not line:
                    continue
                ser.write((line.rstrip("\r\n") + "\r\n").encode())
        except (KeyboardInterrupt, EOFError):
            pass
        finally:
            stop_event.set()

    ser.close()
    if log_file:
        log_file.close()


if __name__ == "__main__":
    main()
