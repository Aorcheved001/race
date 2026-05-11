# AURIX TC377 new_ins Project Index

> Version: v2.0 | Chip: TC377TP | ADS: v1.10.2

---

## 1. Architecture Overview

```
new_ins/
├── code/                    Application source code (3 modules)
│   ├── system/              System init, interrupt scheduler, menu
│   ├── ins/                 INS (Inertial Navigation System) module
│   └── control/             Motor, PID, encoder, servo, track, remote
├── libraries/               Third-party libraries (ZF driver + Infineon iLLD)
│   ├── zf_common/           ZF common utilities (FIFO, debug, font, etc.)
│   ├── zf_driver/           ZF peripheral drivers (GPIO, UART, PWM, etc.)
│   ├── zf_device/           ZF device drivers (sensors, displays, wireless)
│   ├── zf_components/       ZF components (printf redirect, assistant)
│   └── infineon_libraries/  Infineon iLLD low-level drivers for TC37A
├── Debug/                   Build output (.elf, .hex, .map, .o files)
├── tools/                   Build/flash/sync scripts
├── scripts/                 Python tools (serial reader, analyzers)
├── data/                    Serial port collected data
├── docs/                    Documentation
├── .cproject / .project     ADS/Eclipse project configuration
└── Lcf_Tasking_Tricore_Tc.lsl  Linker script (memory layout definition)
```

---

## 2. Entry Points & Execution Flow

### 2.1 Core Entry Files

| File | Role | Key Function |
|------|------|-------------|
| `user/cpu0_main.c` | **CPU0 main entry** | `core0_main()` - init + main loop |
| `user/cpu1_main.c` | CPU1 entry | Secondary core |
| `user/cpu2_main.c` | CPU2 entry | Secondary core |
| `user/isr.c` | ISR dispatcher | Interrupt service routing |
| `user/isr_config.h` | ISR configuration | Vector table config |

### 2.2 Execution Flow

```
core0_main()
  → clock_init()              // Get clock frequency (MUST keep)
  → debug_init()              // Init debug UART
  → system_init_all()         // [init_all.c] All peripherals init
  → cpu_wait_event_ready()    // Wait all cores ready
  → while(1) {
        InterruptTasks_Poll()   // [interrupt.c] Poll pending tasks
      }
```

### 2.3 Init Chain (system_init_all)

```
system_init_all()  →  code/system/init_all.c:8
  → imu963ra_init()          // IMU sensor init
  → imu_bias_init()           // IMU bias calibration
  → imu_gyro_z_autocalib()    // Gyro Z-axis auto-calibration
  → imu_mag_bias_load()       // Magnetometer calibration params
  → ips200_init()             // LCD display init (SPI mode)
  → gnss_init(TAU1201)        // GNSS/GPS receiver init
  → key_init(8)               // Button/key scanner init (8 keys)
  → encoder_init()            // Wheel encoder init
  → encoder_layer_set_model() // Encoder tick-to-meter model
  → wireless_uart_init()      // COM24 wireless UART (115200bps)
  → motor_init()              // Motor PWM/direction pins
  → servo_init()              // Servo control init
  → yaokong_init(0.8f)        // Remote control receiver init
  → Ins_init()                // INS navigation system init
  → INS_init()                // INS state machine init
  → pit_ms_init(CCU60_CH0, 1) // 1ms periodic timer (CCU6)
  → wheel_pid_init()          // Speed PID controller init
  → track_init()              // Trajectory tracking init
```

---

## 3. Module Index - code/

### 3.1 system/ - System Infrastructure

| File | Purpose | Key APIs |
|------|---------|----------|
| `init_all.c/h` | **System init orchestrator** | `system_init_all()` |
| `interrupt.c/h` | **Interrupt scheduler** with time-sliced tasks | `InterruptTasks_Poll()`, `Interrupt_1ms/2ms/4ms/8ms/16ms/40ms()` |
| `menu.c/h` | LCD menu UI (IPS200 display) | Menu rendering & input |

**Interrupt task schedule** (from `interrupt.c`):
- **1ms**: Key scanning (`key_scanner()`)
- **4ms**: INS display (`INS_Display()`), magnetometer raw data send (`imu_mag_send_raw_data_to_pc()`)
- **40ms**: Position send to host via wireless UART (`send_pos_to_host()`)

**Data output format** (COM24 wireless UART, from `interrupt.c:114`):
```
imu_yaw:{gyro_deg:.2f},{mag_rel_deg:.2f},{ekf_deg:.2f}\r\n
```
Three yaw layers: gyro-integrated, magnetometer-relative, EKF-fused.

### 3.2 ins/ - Inertial Navigation System

| File | Purpose | Key APIs |
|------|---------|----------|
| `Ins.c/h` | **Core INS engine** - EKF-based 3-layer yaw fusion | `Ins_init()`, `Ins_update()`, `Ins_get_state()`, `Ins_get_yaw_layers()` |
| `gnss.c/h` | GNSS/GPS receiver interface | `gnss_init()`, GNSS data parsing |
| `imu660.c/h` | IMU660RA 6-axis sensor driver + magnetometer calibration | IMU raw data read, bias handling, ellipsoid fitting calibration |

**Key structures** (from `Ins.h`):
```c
INS_State        { x, y, yaw }                    // Position + heading
INS_Input        { v_mps, omega_rad_s, gyro_z_rad_s, mag_yaw_rad, mag_valid }
INS_Config      { kalman params, wheelbase, encoder coeffs, ZUPT thresholds }
YawEKF2State    { 2-state EKF: [yaw, bias], P matrix, noise params }
```

**INS state dimensions**: 3-state (x, y, yaw), 2-input (speed, angular rate)

### 3.3 control/ - Vehicle Control

| File | Purpose | Key APIs |
|------|---------|----------|
| `Motor.c/h` | **Motor driver** - PWM + direction for LB/RB motors | `motor_init()`, `motor_control(motor_type, duty)` |
| `PID.c/h` | **Speed PID controller** for wheel speed | `wheel_pid_init()`, `wheel_pid_update()`, `wheel_pid_set_target_speed()` |
| `encoder.c/h` | **Wheel encoder** - tick counting to speed | `encoder_init()`, encoder layer state |
| `servo.c/h` | Servo motor control (steering?) | `servo_init()`, servo control |
| `track.c/h` | **Trajectory tracking** logic | `track_init()`, path following |
| `yaokong.c/h` | **Remote control** receiver (RC) | `yaokong_init()`, RC channel read |
| `ins_new_264.c/h` | INS variant/extension (264-point algorithm?) | Extended INS calculations |
| `zf_device_lora3a22.c/h` | LoRa wireless module driver | LoRa communication |

**Motor pin mapping** (from `Motor.h`):
```
Left-Back:  PWM=ATOM0_CH2/P21_4, DIR=P21_3
Right-Back: PWM=ATOM0_CH3/P21_5, DIR=P21_2
```

**PID structure** (from `PID.h`):
```
WHEEL_PID_LAYER:
  - pid_spd: PID_INFO { iError, LastError, SumError }
  - speed_param[4]: [Kp, Ki, Kd, integral_limit]
  - target_speed_mps: target speed setpoint
  - out_pwm: final PWM output value
```

---

## 4. Library Index - libraries/

### 4.1 zf_common/ - Common Utilities

| Directory/File | Purpose |
|----------------|---------|
| `zf_common_clock.c/h` | Clock/timing utilities |
| `zf_common_debug.c/h` | Debug output helpers |
| `zf_common_fifo.c/h` | FIFO ring buffer implementation |
| `zf_common_font.c/h` | Font data for LCD/OLED |
| `zf_common_function.c/h` | General math/utility functions |
| `zf_common_interrupt.c/h` | Interrupt abstraction layer |

### 4.2 zf_driver/ - Peripheral Drivers

| File | Peripheral |
|------|-----------|
| `zf_driver_adc.c/h` | ADC (Analog-to-Digital Converter) |
| `zf_driver_delay.c/h` | Delay functions |
| `zf_driver_dma.c/h` | DMA (Direct Memory Access) |
| `zf_driver_encoder.c/h` | Quadrature encoder interface |
| `zf_driver_exti.c/h` | External interrupt |
| `zf_driver_flash.c/h` | Flash memory operations |
| `zf_driver_gpio.c/h` | GPIO (General Purpose I/O) |
| `zf_driver_pit.c/h` | PIT (Periodic Interrupt Timer) |
| `zf_driver_pwm.c/h` | PWM output generation |
| `zf_driver_soft_iic.c/h` | Software I2C |
| `zf_driver_soft_spi.c/h` | Software SPI |
| `zf_driver_spi.c/h` | Hardware SPI |
| `zf_driver_timer.c/h` | Timer/counter |
| `zf_driver_uart.c/h` | UART/serial communication |

### 4.3 zf_device/ - Device Drivers

| File | Device |
|------|--------|
| `zf_device_imu660ra.c/h` | **IMU660RA** 6-axis IMU (accel+gyro) |
| `zf_device_imu963ra.c/h` | **IMU963RA** 9-axis IMU (+magnetometer) |
| `zf_device_gnss.c/h` | **GNSS/GPS** receiver (TAU1201) |
| `zf_device_encoder.c/h` | Absolute/relative encoder |
| `zf_device_ips114.c/h` | IPS114 TFT LCD (1.44") |
| `zf_device_ips200.c/h` | IPS200 TFT LCD (2.0") |
| `zf_device_key.c/h` | Button/key input |
| `zf_device_mt9v03x.c/h` | MT9V03X camera sensor |
| `zf_device_oled.c/h` | OLED display |
| `zf_device_wifi_uart.c/h` | WiFi-UART bridge |
| `zf_device_wireless_uart.c/h` | **Wireless UART** (Lora3a22, COM24) |

### 4.4 zf_components/ - High-Level Components

| File | Purpose |
|------|---------|
| `printf_redirect.c/h` | Redirect printf to UART |
| `seekfree_assistant.c/h` | SeekFree assistant library |
| `seekfree_assistant_interface.c/h` | Assistant interface definitions |

### 4.5 infineon_libraries/ - Infineon iLLD (TC37A)

```
infineon_libraries/iLLD/TC37A/Tricore/
├── Cpu/Std/           IfxCpu (CPU core control)
├── Gtm/Std/           IfxGtm (Generic Timer Module)
├── Gtm/Atom/Pwm/      IfxGtm_Atom_Pwm (PWM generation)
├── Gpt12/Std/         IfxGpt12 (General Purpose Timer 12)
├── Port/Std/          IfxPort (Port I/O pins)
├── Scu/Std/           IfxScu (System Control Unit)
├── Src/Std/           IfxSrc (Service Request Generator)
├── Stm/Std/           IfxStm (System Timer Module)
├── Asclin/Std/        IfxAsclin (ASC LIN Serial Interface = UART)
├── Asclin/Asc/        IfxAsclin_Asc (ASC sub-module)
├── Qspi/Std/          IfxQspi (Quad SPI)
├── Evadc/Std/         IfxEvadc (Enhanced ADC)
├── Dma/Std/           IfxDma (DMA)
├── Flash/Std/         IfxFlash (Flash memory)
├── _Impl/             Configuration implementations (_cfg.c files)
├── _PinMap/           Pin mapping definitions
├── Configurations/    Ifx_Cfg_Ssw (Software Switch)
└── Service/           Bsp (Board Support Package), Math, StdIf, SpiIf
```

---

## 5. Configuration Files

| File | Purpose | Edit Frequency |
|------|---------|---------------|
| `.cproject` | ADS build config (compiler flags, toolchain, include paths) | Rare (via IDE) |
| `.project` | Eclipse project identity | Never |
| `Lcf_Tasking_Tricore_Tc.lsl` | **Linker script** - memory map, section placement | When adding large RAM/Flash usage |
| `.settings/*.prefs` | IDE preferences | Auto-generated |
| `libraries/zf_device/zf_device_config.a` | Precompiled device config library | Recompile if device changes |
| `code/control/zf_device_lora3a22.c/h` | LoRa/wireless module config | When changing wireless settings |

**Wireless UART config** (from `zf_device_wireless_uart.h`):
```
WIRELESS_UART_INDEX     = UART_2
WIRELESS_UART_BUAD_RATE = 115200
WIRELESS_UART_TX_PIN    = UART2_RX_P10_6
WIRELESS_UART_RX_PIN    = UART2_TX_P10_5
WIRELESS_UART_RTS_PIN   = P10_2
WIRELESS_UART_AUTO_BAUD_RATE = 1  (auto baud rate enabled)
```

---

## 6. Build Output - Debug/

| Pattern | Description |
|---------|-------------|
| `new_ins.elf` | **Final firmware** (ELF format, ~10MB) - generated by ADS IDE only |
| `new_ins.hex` | Firmware in Intel HEX format (~218KB) |
| `new_ins.map` | Memory map / symbol table |
| `*.o` (~150 files) | Compiled object files (one per .c source) |
| `*.d` | Dependency tracking files |
| `*.src` | Compiler intermediate files |
| `subdir.mk` | Makefile fragments per directory |
| `makefile` | Top-level makefile (auto-generated by ADS) |
| `flash_log.xml` | AurixFlasher flash log |
| `Seekfree_TC377_Opensource_Library.*` | Legacy build artifacts (can ignore) |

---

## 7. Communication Interfaces

### 7.1 COM33 - Debug Port (Infineon DAS JDS)
- **Purpose**: Debugging, flashing, console I/O
- **Protocol**: DAS proprietary
- **Usage**: Used by ADS IDE and AurixFlasher

### 7.2 COM24 - Wireless Data Port (USB-SERIAL)
- **Purpose**: Real-time telemetry data output
- **Baud rate**: 115200 bps
- **Data format**: Text lines, ~25Hz (40ms period)
- **Content**: `imu_yaw:{gyro:.2f},{mag_rel:.2f},{ekf:.2f}\r\n`
- **Source**: `code/system/interrupt.c` → `send_pos_to_host()` → `wireless_uart_send_string()`

---

## 8. Tools & Scripts

### 8.1 tools/ - Build & Flash

| Script | Purpose | Usage |
|--------|---------|-------|
| `headless_build.bat` | Build + flash automation | `tools\headless_build.bat --flash-only` |
| `flash_ads.bat` | Flash only | `tools\flash_ads.bat` |

### 8.2 scripts/ - Python Tools

| Script | Purpose | Usage |
|--------|---------|-------|
| `serial_reader.py` | Serial port data collection | `python scripts\serial_reader.py -t 60 --save` |
| `analyze_drift.py` | Static yaw drift analysis | `python scripts\analyze_drift.py data\file.txt` |
| `analyze_dynamic.py` | Dynamic convergence analysis | `python scripts\analyze_dynamic.py data\file.txt` |
| `ai_tuner.py` | Auto build/flash/test loop | `python scripts\ai_tuner.py --port COM24` |
| `mag_calibration/plotter.py` | Magnetometer calibration GUI | `python scripts\mag_calibration\plotter.py` |

### 8.3 data/ - Collected Data

All serial port collected data is stored here:
- `data/baseline_*.txt` - Static drift tests
- `data/dynamic_*.txt` - Dynamic rotation tests
- `data/r2_*.txt` - Round 2 tuning tests

---

## 9. Quick Reference: Find Code By Feature

| You want to... | Go to file | Key function/struct |
|---------------|-----------|-------------------|
| Change motor PWM pins | `code/control/Motor.h` | `MOTORC_PWM_PIN`, `MOTORD_PWM_PIN` |
| Tune PID parameters | `code/control/PID.h` | `WHEEL_PID_LAYER.speed_param[4]` |
| Adjust INS EKF noise | `code/ins/Ins.h` | `INS_Config.{Q_yaw, R_mag}` |
| Change wheelbase | `code/ins/Ins.h` | `INS_WHEELBASE_M` or `INS_Config.wheelbase` |
| Add new peripheral init | `code/system/init_all.c` | `system_init_all()` function body |
| Add periodic task | `code/system/interrupt.c` | `Interrupt_Xms()` functions |
| Change data output format | `code/system/interrupt.c` | `send_pos_to_host()` sprintf |
| Wireless UART config | `libraries/zf_device/zf_device_wireless_uart.h` | Baud rate, pins |
| IMU sensor config | `libraries/zf_device/zf_device_imu963ra.c/h` | I2C/SPI address, range |
| Magnetometer calibration | `code/ins/imu660.c` | `solve_ellipsoid()`, `imu_mag_calib_start()` |
| GNSS receiver config | `code/ins/gnss.c/h` | Protocol, baud rate |
| Memory layout | `Lcf_Tasking_Tricore_Tc.lsl` | Section addresses |
| Compiler flags | `.cproject` (via IDE) | Optimization, defines |

---

## 10. Competition Application Scenarios

For detailed analysis of the 21st Smart Car Competition - Kart Racing Group, see:

**[应用场景.md](../应用场景.md)** - Competition task requirements and project capability analysis

### Quick Links

- **科目1: 自动驾驶** - Trajectory recording/playback, garage positioning
- **科目2: 战场救护/语音交互** - Precise positioning, voice control
- **科目3: 如影随形/穿越迷宫** - Path planning, obstacle avoidance

### Key Capabilities

| Feature | Status | Module |
|---------|--------|--------|
| INS Navigation | ? Implemented | `code/ins/Ins.c` |
| Trajectory Recording | ? Implemented | `code/control/track.c` |
| Pure Pursuit Control | ? Implemented | `code/control/track.c` |
| Reverse Driving | ? Planned | `code/control/track.c` |
| Garage Positioning | ? Planned | `code/control/track.c` |

---

## 11. Documentation Index

| Document | Purpose |
|----------|---------|
| [CLAUDE.md](../CLAUDE.md) | Project overview and constraints |
| [应用场景.md](../应用场景.md) | Competition task analysis |
| [TOOLCHAIN_GUIDE.md](TOOLCHAIN_GUIDE.md) | Toolchain constraints and workflow |
| [PROJECT_INDEX.md](PROJECT_INDEX.md) | This file - module and API reference |
| [INS_IMPLEMENTATION_SUMMARY.md](INS_IMPLEMENTATION_SUMMARY.md) | INS algorithm details |

---

**Last Updated**: 2025-01-XX  
**Maintainer**: Project Development Team
