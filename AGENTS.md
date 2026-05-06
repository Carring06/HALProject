# AGENTS.md

## Repository Structure

This is a **multi-project repository** — each subdirectory is an independent STM32 project with its own build config. There is no unified build system. Work inside the specific project directory.

Active projects:
- `Rear_kick_leg_chassis_code_framework/` — STM32H723VBTx, 4x M3508 + 4x DM6220, FreeRTOS
- `have_newRemote and AGV/` — STM32H723VBTx, 4x M3508 + 4x GM6020, FreeRTOS
- Other directories contain standalone experiments or older iterations

## Build & Flash

- **IDE**: Projects use **EIDE** (`.eide/eide.yml`) or **Keil MDK-ARM** (`.uvprojx`)
- **CubeMX**: `.ioc` files configure peripherals. Regenerate code via STM32CubeMX
- **Flash**: OpenOCD with CMSIS-DAP (`interface: cmsis-dap, target: stm32h7x`) or JLink (`speed: 8000`)
- No CLI build/test commands — use the IDE or EIDE toolchain

## Code Generation Constraints

When modifying CubeMX-generated files (`Core/Src/`, `Core/Inc/`), **only edit between `USER CODE BEGIN/END` markers**. Code outside these blocks is overwritten on regeneration.

## Key Architecture Facts

- **MCU**: STM32H723VBTx/VGTx (Cortex-M7, 480MHz, FDCAN)
- **RTOS**: FreeRTOS with CMSIS-RTOS wrapper (`cmsis_os.h`)
- **Task priority**: `INS_TASK` (Realtime, 1ms) > `CHASSIS_TASK` (AboveNormal, 5ms) > `defaultTask` (Normal)
- **Directory layout** (per project):
  - `User/APP/` — FreeRTOS tasks (`INS_task`, `AGV_chassis_task`)
  - `User/Bsp/` — Board support (CAN, UART, SPI, PWM)
  - `User/Devices/` — Device drivers (DJI_Motor, DM_Motor, BMI088)
  - `User/Algorithm/` — Control algorithms (PID, Mahony, EKF, Kalman)
  - `User/Controller/` — Advanced controllers (fuzzy PID, LDOB, feedforward)
  - `User/Lib/` — Utility functions (constrain, deadband, ramp)

## Motor Control Quirks

- **DJI motors** (M3508, GM6020): CAN standard frames, encoder range 0–8191, current-mode control
- **DM motors** (DM6220, DM4310): Different protocol, position range ±12.5 (float), supports position/speed/MIT modes
- **CAN bus split**: FDCAN1 handles steering/DM motors, FDCAN2 handles hub motors
- **PID init trap**: `PID_init()` checks `Initlized` flag — calling it twice **will not update parameters**. Modify struct fields directly for runtime tuning

## Critical Gotchas

- **6020/DM620 zeroing**: First boot requires mechanical zero calibration (`AGV_chassis_Init` or `Save_Motor_Zero`)
- **Dead zone**: SBUS remote needs software deadband (0.03) — hardware neutral is imprecise
- **INS startup**: IMU attitude integration delayed 3s after boot to avoid transient errors
- **Encoder overflow**: Handle 0/8192 boundary crossing for DJI motors, ±12.5 wrap for DM motors
- **AXI_SRAM**: DMA buffers for CAN/UART use AXI_SRAM for performance

## Existing Detailed Notes

Per-project deep-dive notes exist at:
- `have_newRemote and AGV/User/AGENTS.md`
- `Rear_kick_leg_chassis_code_framework/User/AGENTS.md`

These contain full register maps, PID tuning guides, and kinematic formulas — consult them when working deep in those projects.
