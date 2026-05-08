# AGENTS.md — Steering Wheel AGV Chassis Task

## 项目概述

本项目为**四轮独立转向+驱动AGV底盘**控制代码，基于 **STM32H723** 系列MCU，运行 FreeRTOS 实时操作系统。

### 硬件配置
| 组件 | 型号 | 数量 | 通信方式 |
|------|------|------|----------|
| 驱动电机 | DJI M3508 | 4 | FDCAN (标准帧) |
| 转向电机 | DM J6220 | 4 | FDCAN (MIT模式) |
| 遥控器 | 富斯SBUS遥控器 | 1 | SBUS (UART) |
| IMU | BMI088 | 1 | SPI |

### CAN总线分配
- **FDCAN1**: 左前6220 (ID 0x01) + 右后6220 (ID 0x04) + 部分3508电机
- **FDCAN2**: 右前6220 (ID 0x02) + 左后6220 (ID 0x03) + 部分3508电机

---

## AGV_Chassis_Task 任务详解

### 任务基本信息
- **执行周期**: 5ms (osDelay(5))
- **优先级**: AboveNormal (CHASSIS_TASK)
- **入口函数**: `AGV_Chassis_Task()`

### 主循环执行流程

```
┌─────────────────────────────────────────────────┐
│  AGV_Chassis_Task() - 5ms周期循环                │
├─────────────────────────────────────────────────┤
│  1. systemvalue == Initing ?                    │
│     ├─ Yes: AGV_Chassis_Init() (转向归零)       │
│     │        Wheel_Angle_Last_Init()            │
│     │        systemvalue = Running              │
│     └─ No: 继续                                 │
│                                                  │
│  2. Motor_pid_init() - 重置PID参数              │
│                                                  │
│  3. RemoteControlChassis() - 解析遥控器输入     │
│     └─ FS_Remote_Ctrl.ch[0~3] → vx, vy, vw     │
│                                                  │
│  4. CHASSIS_Single_Loop_Out() - 核心控制输出    │
│     ├─ Absolute_Cal() → 运动学解算              │
│     │   ├─ AGV_angle_calc() → 4轮目标角度       │
│     │   └─ AGV_speed_calc() → 4轮目标速度       │
│     ├─ AGV_Set_Motor_Speed() → 设置3508目标     │
│     ├─ AGV_Set_Motor_angle() → 设置6220目标     │
│     ├─ 3508速度环PID × 4 → 输出电流              │
│     ├─ 6220角度环PID × 4 → 角度环输出            │
│     ├─ 6220速度环PID × 4 → 扭矩输出              │
│     └─ DM_Motor_Four_Ctrl() → CAN发送MIT指令     │
│                                                  │
│  5. osDelay(5) - 等待下一周期                    │
└─────────────────────────────────────────────────┘
```

---

## 核心数据结构

### Chassis_Speed (底盘速度)
```c
typedef struct {
    float vx;   // 车体系X轴速度 (前后方向)
    float vy;   // 车体系Y轴速度 (左右方向)
    float vw;   // 旋转角速度 (逆时针为正)
} Chassis_Speed;
```

### SystemValue (系统状态)
```c
typedef enum {
    Initing = 0,    // 初始化中 (转向电机归零)
    Running = 1,    // 正常运行
} SystemValue;
```

### 电机PID控制结构
- **DM_6220_pid_set[4]**: 转向电机控制结构，每个包含：
  - `Angle_pid`: 位置环PID (Kp=20, 电机1 Kp=20)
  - `Speed_pid`: 速度环PID (Kp=0.07, 电机1 Kp=0.053)
- **DJI_3508_pid_set[4]**: 驱动电机控制结构，每个包含：
  - `Speed_pid`: 速度环PID (Kp=13.0, Ki=0.03, Kd=0)

---

## 运动学解算逻辑

### 坐标系定义
- **车体坐标系**: 前进为X正，左行为Y正，逆时针旋转为正
- **遥控器映射** (富斯遥控器):
  - `ch[3]` / 149 → vx (最大约5m/s)
  - `ch[2]` / 150 → vy (最大约5m/s)
  - `ch[0]` / 150 → vw (最大约5rad/s)

### Absolute_Cal() — 坐标变换
当前传入 angle=0，无云台跟随功能，直接使用遥控器车体系速度。

### AGV_angle_calc() — 转向角度计算
1. **atan2计算各轮应转角度**:
   ```
   atan_angle[i] = atan2(vx ± vw*OFFSET_Y, vy ± vw*OFFSET_X)
   ```
   - 左前(LQ): vx - vw*Y, vy - vw*X
   - 右前(RQ): vx - vw*Y, vy + vw*X
   - 右后(RH): vx + vw*Y, vy + vw*X
   - 左后(LH): vx + vw*Y, vy - vw*X

2. **角度回环处理**: `AngleLoop_f()` 将角度限制在 [-2π, 2π]

3. **翻转检测**: 如果目标角度与当前角度差值 > π/2，则所有轮子翻转180°，同时反转驱动方向 (`sign_group[] = -1`)

4. **静止保持**: 当 vx=vy=vw=0 时，保持上一时刻的目标角度

### AGV_speed_calc() — 驱动速度计算
1. **速度转换系数**: `wheel_rpm_ratio = 786.43 / WHEEL_PERIMETER` (786.43 ≈ 60 * 2π * 减速比相关的常数)

2. **各轮速度**:
   ```
   wheel_speed[i] = sqrt((vy ± vw*OFFSET_X)² + (vx ± vw*OFFSET_Y)²) * ratio
   ```

3. **方向修正**: `out_speed[i] = (-1) * sign_group[i] * wheel_speed[i]`

---

## PID控制链

### 转向电机 (DM6220) — 串级PID
```
目标角度 → [角度环PID (位置式, Kp=20)] → 目标速度 → [速度环PID (位置式, Kp=0.07)] → 扭矩 → mit_ctrl2()
```

### 驱动电机 (DJI M3508) — 单环PID
```
目标速度(RPM) → [速度环PID (位置式, Kp=13, Ki=0.03)] → 设定电流 → CAN发送(0x200帧)
```

**注意**: 代码中 3508 的 CAN 发送被注释掉了（第526-527行），当前仅转向电机在工作。

---

## 初始化流程 (AGV_Chassis_Init)

1. 进入最多1500次循环的初始化
2. 4个转向电机目标角度设为 0.0f
3. 通过角度环+速度环串级PID控制，将转向轮归零
4. 使用 `DM_Motor_Four_Ctrl()` 发送扭矩指令
5. 完成后设置 `systemvalue = Running`

---

## 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| CHASSIS_OFFSET_X | 0.315f | 底盘中心到轮X方向距离(m) |
| CHASSIS_OFFSET_Y | 0.295f | 底盘中心到轮Y方向距离(m) |
| WHEEL_RADIUS | 0.065f | 轮半径(m) |
| WHEEL_PERIMETER | 0.408f | 轮周长(m) |
| 任务周期 | 5ms | 控制频率200Hz |
| 角度回环周期 | 2π (6.28f) | 完整圆周 |

---

## 重要注意事项

1. **PID重复初始化问题**: `Motor_pid_init()` 在每个周期都调用，利用 `Initlized = false` 强制重新初始化PID参数（根据AGENTS.md全局说明，`PID_init()` 检查 `Initlized` 标志，二次调用不会更新参数，这里通过手动设为 false 来规避）

2. **3508 CAN发送被注释**: `CHASSIS_Single_Loop_Out()` 末尾的 `DJI_Motor_ctrl()` 调用被注释，驱动电机实际没有收到指令

3. **遥控器映射**: 当前使用富斯遥控器 (`FS_Remote_Ctrl`)，原DJI遥控器映射被注释

4. **无云台跟随**: `Absolute_Cal()` 传入 angle=0，云台跟随功能未启用

5. **翻转策略**: 当转向角度差 > 90° 时，4个轮同时翻转180°并反转驱动方向，避免单轮大角度旋转

---

## 与 Rear_kick_leg_chassis_code_framework 项目对比

### 对比工程信息
- **对比源**: `D:\DeskTop\HALProject\Rear_kick_leg_chassis_code_framework\User\APP\AGV_chassis_task.c`
- **对比文档**: `D:\DeskTop\HALProject\Rear_kick_leg_chassis_code_framework\User\AGENTS.md`

### 核心差异对比表

| 对比项 | 当前工程 (Streering wheel_AGV) | Rear_kick_leg 工程 | 评价 |
|--------|-------------------------------|-------------------|------|
| **遥控器** | 富斯遥控器 `FS_Remote_Ctrl` | 大疆SBUS遥控器 `remote_ctrl.rc.ch` | 仅硬件差异 |
| **遥控器死区** | **无死区处理** | 有死区处理 (0.03f阈值) | ⚠️ 当前工程缺失 |
| **3508 CAN发送** | **被注释**，驱动电机不工作 | 按FDCAN分组发送 (FDCAN1: 电机1,2; FDCAN2: 电机0,3) | 🔴 当前工程严重缺陷 |
| **PID初始化** | 每个5ms循环调用 `Motor_pid_init()` | 任务启动时调用一次 `AGV_classis_Pid_data_Init()` | ⚠️ 当前工程效率低 |
| **翻转标志** | 全局 `sign_group[4]`，所有轮统一翻转 | 独立 `reverse_flag[4]`，每轮独立判断 | 🔴 当前工程逻辑错误 |
| **初始化目标角度** | 全部设为 `0.0f` | 使用机械中位 `L_Q_DM6220_Middle_Pos` 等 | ⚠️ 当前工程不精确 |
| **atan2参数顺序** | `atan2(vx成分, vy成分)` | `atan2(vy成分, vx成分)` | ⚠️ 当前工程x/y可能颠倒 |
| **角度回环周期** | `2π (6.28f)` | `25.0f` (DM6220 ±12.5范围) | 不同单位体系，无优劣 |
| **初始化延时** | 无延时，纯while循环 | 每1ms延时 + `cnt%5==0` 控制节奏 | ⚠️ 当前工程CPU占用高 |
| **FDCAN选择** | 硬编码 ID 映射 | 按索引动态选择 `(i==1\|\|i==2) ? &hfdcan1 : &hfdcan2` | ⚠️ 当前工程扩展性差 |
| **代码结构** | 展开式，每轮单独写 | 循环式，`for(uint8_t i=0; i<4; i++)` | ⚠️ 当前工程冗余度高 |

### 详细差异分析

#### 1. 🔴 3508驱动电机未工作 (严重)
当前工程第526-527行:
```c
//DJI_Motor_ctrl(chassis_3508_motor, &hfdcan1,0);
//DJI_Motor_ctrl(chassis_3508_motor, &hfdcan2,0);
```
被注释掉，轮毂电机没有收到任何CAN指令。Rear_kick_leg 工程使用 `canx_send_data()` 手动构建0x200标准帧并正确发送到对应FDCAN总线。

#### 2. 🔴 翻转策略逻辑错误
当前工程 `AGV_angle_calc()` 第369-386行:
```c
if(fabs(Find_min_Angle(angle_temp,wheel_angle[0])) > (Pi/2))
{
    for(int i=0;i<4;i++) wheel_angle[i] += Pi;  // 全部翻转
    for(int i=0;i<4;i++) sign_group[i] = -1;     // 全部反转
}
```
**问题**: 仅检测轮0的角度差，却让4个轮同时翻转。四轮全向移动时各轮目标角度不同，此逻辑会导致不需要翻转的轮也翻转180°，造成运动异常。

Rear_kick_leg 工程对每个轮独立判断:
```c
for(int i=0;i<4;i++) {
    if(Find_min_Angle(&wheel_angle[i], chassis_dm_info[i].Data.pos) == -1)
        reverse_flag[i] = -1;
    else
        reverse_flag[i] = 1;
}
```

#### 3. ⚠️ PID每周期重复初始化
当前工程主循环每5ms调用 `Motor_pid_init()`，手动将 `Initlized = false` 后重新初始化。这是**不必要的CPU开销**，且违背了PID只初始化一次的最佳实践。

Rear_kick_leg 工程在任务入口处初始化一次，主循环不再调用。

#### 4. ⚠️ 遥控器缺少死区处理
当前工程 `RemoteControlChassis()` 直接线性映射:
```c
absolute_chassis_speed->vx = (fp32)FS_Remote_Ctrl.ch[3]/149;
```
摇杆中立位不精准会导致底盘微小漂移。

Rear_kick_leg 工程:
```c
if (fabs(chassis_speed.vx) < 0.03f) chassis_speed.vx = 0;
if (fabs(chassis_speed.vy) < 0.03f) chassis_speed.vy = 0;
```

#### 5. ⚠️ atan2 参数顺序可疑
当前工程:
```c
atan_angle[0] = atan2((speed->vx - speed->vw*CHASSIS_OFFSET_Y),
                      (speed->vy - speed->vw*CHASSIS_OFFSET_X));
```
标准atan2应为 `atan2(y, x)`。如果vx是前进方向(y分量)，vy是横向(x分量)，则当前写法可能是正确的（取决于坐标系定义），但与Rear_kick_leg工程的参数顺序**完全相反**，需要实测验证。

#### 6. ⚠️ 初始化目标角度不精确
当前工程所有轮目标角度设为 `0.0f`，但DM6220的机械零点可能不是0。Rear_kick_leg 使用 `L_Q_DM6220_Middle_Pos` 等宏定义各轮实际机械中位。

#### 7. ⚠️ 初始化循环无延时
当前工程 `AGV_Chassis_Init()` 是纯while循环，无 `vTaskDelay()`，会持续占用CPU。Rear_kick_leg 使用 `vTaskDelay(pdMS_TO_TICKS(1))` 每1ms执行一次，且每5ms才计算PID。

### 综合评判

| 维度 | 当前工程 | Rear_kick_leg 工程 |
|------|---------|-------------------|
| **功能完整性** | ⭐⭐ (3508未工作) | ⭐⭐⭐⭐ (完整双环+驱动) |
| **代码质量** | ⭐⭐ (展开式冗余) | ⭐⭐⭐⭐ (循环模块化) |
| **逻辑正确性** | ⭐⭐ (翻转策略有误) | ⭐⭐⭐⭐ (独立翻转标志) |
| **性能** | ⭐⭐ (PID重复初始化) | ⭐⭐⭐⭐ (一次初始化) |
| **可维护性** | ⭐⭐ (硬编码) | ⭐⭐⭐⭐ (宏定义+循环) |

**结论**: **Rear_kick_leg 工程整体质量明显更优**。当前工程存在3个关键问题需要修复:
1. 取消注释3508 CAN发送代码
2. 修正翻转策略为独立判断
3. PID初始化移至任务入口，遥控器增加死区处理
