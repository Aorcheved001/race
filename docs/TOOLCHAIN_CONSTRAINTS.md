# AURIX TC377 工具链任务约束书

> 版本: v2.0 | 日期: 2026-04-11 | 芯片: TC377TP

---

## 1. 环境概览

| 项目 | 值 |
|------|-----|
| ADS 安装目录 | `E:\Aorcheved_01\AURIX-Studio-1.10.2` |
| Trae 工作区 | `D:\race\save\4.10\new_ins` |
| ADS 工作区 | `E:\Aorcheved_01\new_ins` |
| 构建输出目录 | `E:\Aorcheved_01\new_ins\Debug\` |
| 目标芯片 | TC377 (AURIX TriCore) |
| 调试器 | DAS JDS TriBoard TC2XX V2.0 |
| Debug 串口 | COM33 (Infineon DAS JDS) |
| 无线数据串口 | COM24 (USB-SERIAL, 115200bps) |

---

## 2. 目录结构

```
new_ins/
├── code/                          # 嵌入式源码
├── libraries/                     # 库文件
├── Debug/                         # 编译输出
├── .vscode/ .settings/            # IDE 配置 (VS Code)
├── .cproject / .project           # 工程文件 (ADS/Eclipse)
├── Lcf_Tasking_Tricore_Tc.lsl     # 链接脚本
│
├── tools/                         # 编译/烧录/同步工具
│   ├── headless_build.bat         # 编译+烧录主脚本
│   ├── flash_ads.bat              # 纯烧录脚本
│   └── AURIX修改工程名称.bat      # ADS 自带工具
│
├── scripts/                       # Python 工具
│   ├── serial_reader.py           # 串口数据采集
│   ├── analyze_drift.py           # 静态漂移分析
│   ├── analyze_dynamic.py         # 动态收敛分析
│   ├── ai_tuner.py                # 自动编译烧录测试
│   └── mag_calibration/
│       └── plotter.py             # 磁力计校准可视化
│
├── data/                          # 串口采集数据
│   └── *.txt                      # 历史测试数据
│
└── docs/                          # 文档
    ├── TOOLCHAIN_CONSTRAINTS.md   # 本文件
    ├── PROJECT_INDEX.md           # 项目索引
    └── INS_Summary.md             # INS 设计文档
```

---

## 3. 工具清单与调用方式

### 3.1 编译工具

| 工具 | 路径 | 用途 | 命令行可用性 |
|------|------|------|:---:|
| make (GNU) | `%ADS_DIR%\tools\make\make.exe` | 构建调度 | 允许 |
| cctc (Tasking 编译器) | `%ADS_DIR%\tools\Compilers\Tasking_1.1r8\ctc\bin\cctc.exe` | .c→.o 编译 | 允许 |
| ltc (Tasking 链接器) | 同上目录 `\ltc.exe` | .o→.elf 链接 | **禁止** |
| elfsize (Tasking) | 同上目录 `\elfsize.exe` | ELF 大小显示 | **禁止** |

### 3.2 烧录工具

| 工具 | 路径 | 用途 | 命令行可用性 |
|------|------|------|:---:|
| AurixFlasher | `%ADS_DIR%\tools\AurixFlasherSoftwareTool_v1.0.8\AurixFlasher.exe` | 固件烧录 | 允许 |

### 3.3 辅助工具

| 工具 | 路径 | 用途 |
|------|------|------|
| `tools\headless_build.bat` | `d:\race\save\4.10\new_ins\tools\headless_build.bat` | 构建/烧录脚本 |
| `tools\flash_ads.bat` | `d:\race\save\4.10\new_ins\tools\flash_ads.bat` | 纯烧录脚本 |
| `scripts\serial_reader.py` | `d:\race\save\4.10\new_ins\scripts\serial_reader.py` | 串口数据读取 |
| `scripts\analyze_drift.py` | `d:\race\save\4.10\new_ins\scripts\analyze_drift.py` | 静态漂移分析 |
| `scripts\analyze_dynamic.py` | `d:\race\save\4.10\new_ins\scripts\analyze_dynamic.py` | 动态收敛分析 |
| `scripts\ai_tuner.py` | `d:\race\save\4.10\new_ins\scripts\ai_tuner.py` | 自动编译烧录测试 |
| `scripts\mag_calibration\plotter.py` | `d:\race\save\4.10\new_ins\scripts\mag_calibration\plotter.py` | 磁力计校准可视化 |

---

## 4. 许可证限制 (Tasking Non-Commercial)

Tasking VX-toolset **非商业版**有以下硬性限制：

```
禁止 standalone 运行的工具:
├── ltc.exe      (链接器)   → 报错: F001/F101 protection error
└── elfsize.exe  (大小显示) → 报错: F104 License does not support running as standalone

允许 standalone 运行的工具:
├── cctc.exe     (C/C++ 编译器)  → 正常工作
├── astc.exe    (汇编器)        → 正常工作
├── cptc.exe    (C++ 编译器)    → 正常工作
└── amk.exe     (归档器/ar)     → 正常工作
```

**核心结论**: `.o` 目标文件的编译可以在命令行完成，但最终链接生成 `.elf` 必须通过 ADS IDE 完成。

---

## 5. 允许的操作

### 5.1 编译流程

| 步骤 | 方式 | 命令/操作 |
|------|------|-----------|
| 全量编译 | **必须通过 IDE** | ADS IDE → Project → Build Project (Ctrl+B) |
| 增量编译 (仅改.c) | 可尝试命令行 | `tools\headless_build.bat --no-clean` (前提: ELF 已存在且只改源码未改 LCF) |
| 清理产物 | 命令行允许 | 删除 `Debug\*.elf`, `Debug\*.hex`, `Debug\*.map` |

### 5.2 烧录流程

| 步骤 | 方式 | 命令 |
|------|------|------|
| 仅烧录 | **推荐，命令行** | `tools\headless_build.bat --flash-only` |
| 编译+烧录 | 部分可行 | 先 IDE 编译，再 `--flash-only` |
| 全片擦除烧录 | 命令行 | `tools\headless_build.bat --flash-only -erase-all` |
| 快速烧录 (无校验) | 命令行 | `tools\headless_build.bat --flash-only -no-verify` |

### 5.3 串口监控

| 操作 | 命令 |
|------|------|
| 实时监控 COM24 | `python scripts\serial_reader.py` |
| 定时采集 (5秒) | `python scripts\serial_reader.py -t 5` |
| 保存日志 | `python scripts\serial_reader.py --save -t 30` |
| 指定端口 | `python scripts\serial_reader.py -p COM24` |

### 5.4 AurixFlasher 参数参考

```
AurixFlasher.exe [选项]

必需参数 (二选一):
  -elf <file>    ELF 格式固件
  -hex <file>    Intel HEX 格式固件

可选参数:
  -erase <arg>   on=擦除使用区域(默认) | all=全片擦除 | off=不擦除
  -ver <arg>     on=烧录后校验(默认) | off=跳过校验
  -connect <arg> 0=热连接(需先halt) | 6=reset&halt(默认)
  -start <arg>   on=烧录后复位运行(默认) | off=保持halt
  -ucb <arg>     on=编程BMHD(危险!) | off=跳过(默认)
  -log <file.xml>  输出详细XML日志
```

---

## 6. 禁止的操作

### 6.1 绝对禁止

| 操作 | 原因 | 后果 |
|------|------|------|
| 命令行调用 ltc/cctc 链接模式 | 许可证限制 | F001/F101 报错，构建失败 |
| 命令行调用 elfsize | 许可证限制 | F104 报错 |
| 直接修改 `Debug\makefile` | ADS 自动生成 | 下次 IDE 构建会被覆盖 |
| 删除 `.metadata` 目录中 IDE 正在使用的锁文件 | 导致 IDE 异常 | workspace 损坏 |
| 使用 `-ucb on` 烧录 BMHD | 极高风险 | **可能变砖**，无法恢复 |
| Eclipse headless build (Java 模式) | Booster 需要 UI Workbench | `Workbench has not been created yet` 错误 |
| 在 IDE 打开时执行 `--flash-only` | 文件冲突 | 可能写入失败 |

### 6.2 不推荐

| 操作 | 原因 | 建议 |
|------|------|------|
| 修改 ADS 安装目录下的配置文件 | 升级时丢失 | 在工程目录内做自定义 |
| 手动编辑 `.cproject` | 格式复杂易出错 | 通过 IDE 界面修改 |
| 使用中文路径作为 workspace | Java IO 兼容性问题 | 已通过 Make 模式绕过 |

---

## 7. 标准工作流

### 7.1 日常开发 (最常用)

```
1. 修改代码 (.c/.h 文件)
       ↓
2. ADS IDE 中 Build (Ctrl+B)
   → 生成 Debug\new_ins.elf
       ↓
3. 命令行烧录
   $ tools\headless_build.bat --flash-only
   → 3秒内完成擦除/编程/校验/启动
       ↓
4. (可选) 监控串口数据
   $ python scripts\serial_reader.py -t 10 --save
```

### 7.2 只换固件 (不改代码)

```
$ tools\headless_build.bat --flash-only
```

### 7.3 调试问题

```
# 查看烧录日志
type Debug\flash_log.xml

# 检查串口连通性
python scripts\serial_reader.py --list

# 采集数据用于分析
python scripts\serial_reader.py --save -t 60 -o data\debug_log.txt
```

---

## 8. 文件约定

### 8.1 关键输出文件

| 文件 | 生成者 | 说明 |
|------|--------|------|
| `Debug\new_ins.elf` | ADS IDE (链接器) | 主固件，~10MB |
| `Debug\new_ins.hex` | ADS IDE (链接器) | HEX格式，~218KB |
| `Debug\new_ins.map` | ADS IDE (链接器) | 内存映射表 |
| `Debug\flash_log.xml` | AurixFlasher | 烧录详细日志 |
| `data\*.txt` | serial_reader.py | 串口数据日志 |

### 8.2 不要手动管理的文件

```
Debug\*.o              ← 由 make + cctc 管理
Debug\*.d              ← 依赖关系文件
Debug\*.src            ← 编译器中间文件
Debug\subdir.mk       ← ADS 自动生成
Debug\makefile         ← ADS 自动生成
Debug\sources.mk      ← ADS 自动生成
Debug\*.opt            ← 编译器选项缓存
.metadata\*            ← IDE workspace 元数据 (勿动!)
```

---

## 9. 故障排查速查

| 症状 | 原因 | 解决方案 |
|------|------|----------|
| `ltc F101: cannot create` | 命令行链接被禁 | 改用 IDE 编译 |
| `elfsize F104: License` | elfsize 被禁 | 脚本已用伪 elfsize 绕过 |
| `拒绝访问 .metadata` | workspace 锁定 | 关闭 IDE 或删除 `.lock` |
| `Workbench has not been created` | Booster 需要 UI | 使用 Make 模式 (v8脚本已处理) |
| `License does not support standalone` | Tasking 许可证限制 | 见第4节 |
| 烧录 `Pass` 但设备不跑 | 检查 reset 连接 | 尝试 `-connect 6 -start on` |
| COM24 读不到数据 | 无线模块未上电/波特率不匹配 | 检查硬件，确认 115200 |
| `Build failed - ELF not generated` | 链接失败 | 必须用 IDE 编译一次 |

---

## 10. 脚本参数速查

### headless_build.bat

```bat
tools\headless_build.bat [选项]

模式:
  (无参数)          仅编译 (需要 IDE 链接支持)
  -f, -flash        编译 + 烧录
  --flash-only      仅烧录 (推荐)
  --no-flash        强制仅编译

烧录选项:
  -erase-all        全片擦除
  -no-verify        跳过校验 (加速)
  -no-start         烧录后保持 HALT
  --no-clean        增量编译 (不清理旧文件)
```

### serial_reader.py

```bash
python scripts\serial_reader.py [选项]

端口:
  -p, --port COM   串口号 (默认 COM24)
  -b, --baud N     波特率 (默认 115200)
  -l, --list       列出可用端口

控制:
  -t, --time N     运行N秒后停止 (0=无限)
  -n, --lines N    读取N行后停止 (0=无限)
  -s, --save       保存到日志文件
  -o, --output F   指定日志文件路径
  -q, --quiet      不输出统计信息
```
