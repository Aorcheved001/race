import subprocess
import serial
import time
import argparse
import sys
import re

# =======================================================================
# 这是一个为 Trae (AI) 准备的自动化编译、烧录和回调测试脚本。
# AI 可以通过运行这个脚本，获取代码编译和运行后在串口输出的数据，从而实现闭环自动调参。
# =======================================================================

def build_project():
    """调用 ADS 命令行进行无头编译"""
    print("[AI_TUNER] 开始编译项目...")
    # ADS路径和工作空间配置
    cmd = [
        r"E:\Aorcheved_01\AURIX-Studio-1.10.2\eclipse\eclipsec.exe",
        "-nosplash",
        "-application", "org.eclipse.cdt.managedbuilder.core.headlessbuild",
        "-data", r"E:\Aorcheved_01",  # ADS Workspace 目录
        "-build", "new_ins"
    ]
    try:
        # 为了不刷屏，只在出错时显示详细信息
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if result.returncode != 0:
            print("[AI_TUNER] 编译失败！")
            print(result.stdout)
            print(result.stderr)
            sys.exit(1)
        print("[AI_TUNER] 编译成功！")
    except FileNotFoundError:
        print("[AI_TUNER] 未找到 ADS 编译器，请检查路径。")
        sys.exit(1)

def flash_project():
    """调用 DAS / Memtool 命令行进行烧录"""
    print("[AI_TUNER] 开始烧录...")
    # TODO: 替换为实际的烧录命令
    # 例如：Memtool.exe -c target.cfg -w Debug\new_ins.elf -s
    # 这里用 echo 模拟成功
    print("[AI_TUNER] 烧录成功！（模拟）")

def read_serial_feedback(port, baudrate, timeout_sec=5):
    """读取串口数据作为回调（Callback）提供给 AI 分析"""
    print(f"[AI_TUNER] 正在监听串口 {port} ({baudrate} bps)，持续 {timeout_sec} 秒...")
    
    collected_data = []
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            start_time = time.time()
            while time.time() - start_time < timeout_sec:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"[MCU_CALLBACK] {line}")
                    collected_data.append(line)
                    
                    # 如果单片机输出 "TEST_FINISH"，说明测试结束，提前退出
                    if "TEST_FINISH" in line:
                        break
                        
    except serial.SerialException as e:
        print(f"[AI_TUNER] 串口打开失败: {e}")
        sys.exit(1)
        
    return collected_data

def evaluate_performance(data):
    """(可选) 简单的 Python 端数据处理，AI 可以直接读取原始日志，或者我们在这里算出一个 MSE"""
    # 这里只是提取包含特定关键字的数据供 AI 阅读
    print("\n--- [AI 分析报告摘要] ---")
    for line in data:
        if "ERROR" in line or "RMS" in line or "OVERSHOOT" in line:
            print(line)
    print("-------------------------\n")
    print("[AI_TUNER] 流程结束，请 AI 根据上方 [MCU_CALLBACK] 进行下一步调参。")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Trae AI 自动编译、烧录与串口监听工具")
    parser.add_argument("--port", type=str, default="COM3", help="单片机连接的串口号")
    parser.add_argument("--baud", type=int, default=115200, help="波特率")
    parser.add_argument("--time", type=int, default=5, help="监听时长（秒）")
    parser.add_argument("--skip-build", action="store_true", help="跳过编译和烧录步骤（仅监听）")
    
    args = parser.parse_args()
    
    if not args.skip_build:
        build_project()
        flash_project()
        
    data = read_serial_feedback(args.port, args.baud, args.time)
    evaluate_performance(data)
