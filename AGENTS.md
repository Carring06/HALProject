# AGENTS.md

## Repository Structure

Multi-project STM32 repo — each subdirectory is an independent project with its own build config. Work inside the specific project directory.

Active projects:
- `R1Streering wheel_AGV_New/` — DM6220 (MIT mode) + M3508, mixed FDCAN1/2, FreeRTOS
- `Rear_kick_leg_chassis_code_framework_bad/` — DM6220 dual-loop PID plan, 4x M3508 + 4x DM6220, FreeRTOS (reference for DM6220 work)
- `have_newRemote and AGV/` — GM6020 (DJI) steering + M3508, FreeRTOS (reference for 6020 dual-loop PID)

Other directories are standalone experiments or older iterations.

## Build & Flash

- **IDE**: EIDE (`.eide/eide.yml`) or Keil MDK-ARM (`.uvprojx`)
- **MCU**: STM32H723VGTx (Cortex-M7, 480MHz, double-precision FPU, 3x FDCAN)
- **CubeMX**: `.ioc` files configure peripherals. Regenerate via STM32CubeMX
- **Flash**: OpenOCD with CMSIS-DAP (`interface: cmsis-dap, target: stm32h7x`) or JLink (`speed: 8000`)
- **Uploader config**: set in `.eide/eide.yml` per project
- No CLI build/test commands — use IDE or EIDE toolchain

## Code Generation Constraints

When modifying CubeMX-generated files (`Core/Src/`, `Core/Inc/`), **only edit between `USER CODE BEGIN/END` markers**.

## Shared Directory Layout (per project)

| Path | Contents |
|------|----------|
| `User/APP/` | FreeRTOS task entrypoints (`AGV_chassis_task`, `INS_task`, `Manipulator_Task`) |
| `User/Bsp/` | Board support (CAN, UART, PWM, DWT, USB) |
| `User/Devices/` | Device drivers (DJI_Motor, DM_Motor, BMI088, Remote_Control) |
| `User/Algorithm/` | Control algorithms (PID, Mahony, EKF, Kalman, FIFO) |
| `User/Controller/` | Advanced controllers (fuzzy PID, LDOB, feedforward) |
| `User/Lib/` | Utilities (constrain, deadband, ramp, OLS) |

## CAN Allocation — NOT Uniform Across Projects

Each project splits FDCAN1/2/3 differently. **Do not assume a single convention**. Check `User/Bsp/bsp_can.c` callbacks and `User/APP/AGV_chassis_task.c` `Enable_Motor_Mode` calls.

Examples:
- `R1Streering wheel_AGV_New`: FDCAN1 carries DM6220[0,3] + M3508[0,3]; FDCAN2 carries DM6220[1,2] + M3508[1,2]
- `Rear_kick_leg_chassis_code_framework_bad` (planned): FDCAN1 for wheel indices 1,2; FDCAN2 for indices 0,3
- `have_newRemote and AGV`: FDCAN1 for 6020+gyro, FDCAN2 for M3508

## Motor Protocol Differences

- **DJI motors** (M3508, GM6020): CAN standard frames, encoder 0–8191, current-mode control
- **DM motors** (DM6220, DM4310): MIT mode (pos/vel/kp/kd/torque packed in 8 bytes), position range ±π (float)
- **PID init trap**: `PID_init()` checks `Initlized` flag — calling it twice **does not update params**. Modify struct fields directly for runtime tuning

## FreeRTOS Task Priorities

Common pattern (exact names vary):
1. `INS_TASK` (Realtime, 1ms) — IMU read + attitude
2. `CHASSIS_TASK` (AboveNormal, 5ms) — chassis kinematics + motor PID
3. `defaultTask` (Normal) — USB init

## Per-Project Deep-Dive Notes

Detailed register maps, PID tuning guides, and kinematics exist at:
- `have_newRemote and AGV/User/AGENTS.md`
- `Rear_kick_leg_chassis_code_framework_bad/User/AGENTS.md`
- `Streering wheel_AGV/Streering wheel_AGV/User/AGENTS.md`

Consult these when working deep in those projects.
