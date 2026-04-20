# Claude Code 工具链约束指南

> 版本: v1.0 | 创建日期: 2026-04-16 | 项目: new_ins (AURIX TC377)

---

## 1. 核心工具选择原则

### 1.1 文件读取类工具

| 场景 | 使用工具 | 原因 |
|------|----------|------|
| 已知文件路径，需要读取完整内容 | **Read** | 专用工具，效率最高 |
| 已知文件路径，只需要部分内容 | **Read** + offset/limit | 避免加载大文件全部内容 |
| 查找某个类/函数/变量的定义位置 | **Grep** | 支持正则，快速定位 |
| 查找某个符号的所有引用 | **Grep** | 全局搜索能力强 |
| 不知道文件名，只知道模式 | **Glob** | 文件名模式匹配 |

### 1.2 文件修改类工具

| 场景 | 使用工具 | 原因 |
|------|----------|------|
| 修改现有文件的部分内容 | **Edit** | 精确替换，保留其他内容 |
| 创建新文件或完全重写 | **Write** | 完整写入，适合新文件 |
| 批量替换某个字符串 | **Edit** + replace_all=true | 一次性替换所有实例 |

### 1.3 命令执行类工具

| 场景 | 使用工具 | 原因 |
|------|----------|------|
| Git 操作 (status, diff, log) | **Bash** | Git 是标准命令行工具 |
| 运行构建脚本 (.bat/.py) | **Bash** | 脚本执行需要 shell |
| 安装依赖 (pip, npm) | **Bash** | 包管理需要命令行 |
| 文件搜索 (find/grep 命令) | **禁止**，用 Glob/Grep | 专用工具更高效 |
| 文件内容查看 (cat/head/tail) | **禁止**，用 Read | Read 更规范 |

---

## 2. 项目特定约束

### 2.1 编码约束 (重要!)

```
本项目所有文件必须使用 GB2312 编码！
- 中文注释和字符串必须使用 GB2312
- Hook 已配置自动转换 UTF-8 → GB2312
- 不要手动更改文件编码
```

### 2.2 文件类型与目录规范

| 文件类型 | 必须放置目录 | 示例 |
|----------|--------------|------|
| `.c` / `.h` C 源代码 | `code/*/` 或 `libraries/*/` | `code/ins/Ins.c` |
| `.bat` / `.cmd` 脚本 | `tools/` | `tools/headless_build.bat` |
| `.py` Python 脚本 | `scripts/` | `scripts/serial_reader.py` |
| `.txt` 数据文件 | `data/` | `data/serial_log_*.txt` |
| `.md` / `.txt` 文档 | `docs/` | `docs/PROJECT_INDEX.md` |
| `.png` / 图片 | 项目根目录或子目录 | `mag_calibration/calib_plot.png` |

### 2.3 构建与烧录约束

```
本项目使用 Tasking 非商业版编译器，有以下限制：

编译流程:
1. 修改代码后 → 必须使用 ADS IDE 构建 (Ctrl+B)
2. 不能使用命令行编译 (ltc 许可证限制)
3. 生成 ELF 后 → 可用 headless_build.bat --flash-only 烧录

烧录流程:
- 首选: tools\headless_build.bat --flash-only
- 备选: ADS IDE 烧录功能
- 禁止: 直接调用 AurixFlasher 带特殊参数 (除非用户明确要求)
```

---

## 3. 工具使用决策树

### 3.1 需要查看代码时

```
用户问: "XXX 函数在哪里定义的？"
    ↓
使用 Grep 搜索 "XXX\s*\(" 或 "void XXX"
    ↓
找到文件路径和行号
    ↓
使用 Read 读取相关代码片段
    ↓
回答用户问题 (包含 file:line 引用)
```

### 3.2 需要修改代码时

```
用户问: "把 XXX 改成 YYY"
    ↓
先用 Read 读取目标文件
    ↓
理解代码上下文
    ↓
使用 Edit 精确替换
    ↓
验证修改是否正确
```

### 3.3 需要查找文件时

```
用户问: "项目里有哪些 .py 文件？"
    ↓
使用 Glob 搜索 "**/*.py"
    ↓
列出所有匹配文件
```

### 3.4 需要执行项目操作时

```
用户问: "帮我烧录程序" 或 "读取串口数据"
    ↓
确认 Debug\new_ins.elf 存在 (用 Glob)
    ↓
使用 Bash 执行对应脚本:
  - 烧录: tools\headless_build.bat --flash-only
  - 串口: python scripts\serial_reader.py -t 10
```

---

## 4. 禁止事项

### 4.1 禁止使用的 Bash 命令

| 禁止命令 | 正确替代 | 原因 |
|----------|----------|------|
| `cat file.txt` | `Read file.txt` | Read 是专用工具 |
| `head -n 20 file.txt` | `Read file.txt` + limit=20 | Read 支持分页 |
| `tail -n 20 file.txt` | `Read file.txt` + offset | Read 支持偏移 |
| `grep -r "pattern" .` | `Grep "pattern"` | Grep 工具更高效 |
| `find . -name "*.c"` | `Glob "**/*.c"` | Glob 专用于文件搜索 |
| `ls -la` | `Glob "*"` 或 `Bash ls` | 看情况选择 |

### 4.2 其他禁止事项

```
- 禁止在未读取文件的情况下使用 Edit
- 禁止使用 Write 覆盖已存在的文件 (应先 Read 再 Edit)
- 禁止删除用户未明确要求删除的文件
- 禁止修改 .cproject / .project 等 IDE 配置文件
- 禁止执行危险的 git 命令 (reset --hard, force push 等) 未经确认
```

---

## 5. 项目关键路径速查

### 5.1 源代码

```
code/
├── system/          # 系统初始化、中断调度
│   ├── init_all.c   # system_init_all() 所有外设初始化
│   └── interrupt.c  # InterruptTasks_Poll() 中断任务调度
├── ins/             # 惯性导航系统
│   ├── Ins.c/h      # INS 核心引擎，EKF 融合
│   └── imu660.c/h   # IMU 驱动，磁力计校准
└── control/         # 控制模块
    ├── Motor.c/h    # 电机驱动
    ├── PID.c/h      # 速度 PID
    └── encoder.c/h  # 编码器
```

### 5.2 工具脚本

```
tools/
├── headless_build.bat  # 构建 + 烧录自动化
└── flash_ads.bat       # 仅烧录

scripts/
├── serial_reader.py    # 串口数据采集
├── analyze_drift.py    # 漂移分析
└── trajectory_viewer.py # 轨迹可视化
```

### 5.3 构建输出

```
Debug/
├── new_ins.elf    # 固件文件 (ADS 生成)
├── new_ins.hex    # HEX 格式
└── new_ins.map    # 内存映射
```

---

## 6. 常见任务工作流

### 6.1 修改代码并烧录

```bash
# 1. 读取并修改代码
Read code/ins/Ins.c
Edit code/ins/Ins.c "old" "new"

# 2. 提醒用户在 ADS IDE 中构建
"请在 ADS IDE 中按 Ctrl+B 构建项目"

# 3. 烧录
Bash "tools\headless_build.bat --flash-only"
```

### 6.2 分析串口数据

```bash
# 1. 采集数据
Bash "python scripts\serial_reader.py -t 60 --save"

# 2. 分析漂移
Bash "python scripts\analyze_drift.py data\serial_log_*.txt"
```

### 6.3 查找功能实现

```bash
# 1. 搜索函数定义
Grep "Ins_init\s*\(" --type c

# 2. 读取相关代码
Read code/ins/Ins.c:offset,limit
```

---

## 7. 输出格式规范

### 7.1 代码引用格式

```
使用 file_path:line_number 格式引用代码位置:
- code/ins/Ins.c:45 表示 Ins.c 文件的第 45 行
- code/system/init_all.c:8 表示 init_all.c 文件的第 8 行
```

### 7.2 中文回复规范

```
- 使用简洁的中文回复
- 代码和技术术语保持英文
- 文件路径使用反斜杠 (\) 或正斜杠 (/) 均可
- 列表使用 markdown 格式
```

---

## 8. 快速参考卡

```
┌─────────────────────────────────────────────────────────────┐
│                    工具选择速查表                            │
├─────────────────────────────────────────────────────────────┤
│ 读取文件 → Read          │ 修改文件 → Edit                  │
│ 搜索代码 → Grep          │ 创建文件 → Write                 │
│ 查找文件 → Glob          │ 执行命令 → Bash                  │
├─────────────────────────────────────────────────────────────┤
│ 已知路径？                                                │
│   是 → 直接用 Read/Grep                                   │
│   否 → 先用 Glob 找文件                                   │
├─────────────────────────────────────────────────────────────┤
│ 修改现有文件？                                            │
│   是 → 先 Read，再 Edit                                   │
│   否 → 用 Write 创建新文件                                │
├─────────────────────────────────────────────────────────────┤
│ 需要 shell？                                              │
│   是 → 用 Bash (git, scripts, build tools)                │
│   否 → 用专用工具 (Read/Grep/Glob)                        │
└─────────────────────────────────────────────────────────────┘
```

---

*此文档应在每次启动 Claude Code 时作为上下文加载*
