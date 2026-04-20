# AURIX TC377 new_ins 项目配置

> 项目: new_ins (AURIX TC377 惯性导航系统)
> ADS 版本: v1.10.2
> 芯片: TC377TP

---

## 项目概述

本项目是基于 Infineon AURIX TC377 的惯性导航系统 (INS)，包含：
- EKF 融合姿态估计
- 电机/编码器控制
- GNSS/GPS 接收
- 无线数据传输

---

## 重要约束

### 编码约束
**C/C++ 源文件使用 GB2312 编码！**
- `.c` / `.h` 文件使用 GB2312
- `.md` / `.json` / `.xml` 等保持 UTF-8
- Hook 已配置自动转换 UTF-8 → GB2312（排除 .md/.json 等）

### 构建约束
**必须使用 ADS IDE 构建！**
- 修改代码后，在 ADS IDE 中按 Ctrl+B 构建
- 不能使用命令行编译 (Tasking 非商业版限制)
- 构建成功后，可用 `tools\headless_build.bat --flash-only` 烧录

---

## 必读文档

在开始工作前，请先阅读以下文档：

1. **[工具链约束指南](docs/TOOLCHAIN_GUIDE.md)** - 了解何时使用什么工具
2. **[项目索引](docs/PROJECT_INDEX.md)** - 了解项目结构和模块
3. **[实现摘要](docs/INS_IMPLEMENTATION_SUMMARY.md)** - INS 实现细节

---

## 关键路径

```
code/
├── system/init_all.c    # 所有外设初始化
├── system/interrupt.c   # 中断任务调度
├── ins/Ins.c           # INS 核心引擎
└── control/            # 电机/PID/编码器

tools/headless_build.bat  # 构建+烧录脚本
scripts/serial_reader.py   # 串口数据采集
Debug/new_ins.elf         # 固件文件
```

---

## 常用操作

| 操作 | 命令 |
|------|------|
| 烧录固件 | `tools\headless_build.bat --flash-only` |
| 采集串口数据 | `python scripts\serial_reader.py -t 60 --save` |
| 分析漂移 | `python scripts\analyze_drift.py data\file.txt` |

---

## 调试端口

- **COM33**: Debug Port (Infineon DAS JDS) - 用于烧录和调试
- **COM24**: Wireless Data Port (USB-SERIAL, 115200bps) - 数据输出
