# AGV项目代码深度学习记录

> **项目来源**: `D:\DeskTop\HALProject\have_newRemote and AGV`
> **目标平台**: STM32H723VBT6 (CtrlBoard-H7_IMU)
> **控制对象**: 四轮舵轮底盘 (4x M3508 + 4x GM6020) + 云台
> **RTOS**: FreeRTOS

---

## 1. 项目架构总览

### 1.1 硬件映射
| 模块 | 型号/规格 | 通信接口 | 备注 |
|:---|:---|:---|:---|
| **MCU** | STM32H723VBT6 | - | 主频480MHz, 带FDCAN |
| **IMU** | BMI088 | SPI | 加速度±6g, 陀螺仪±2000°/s |
| **轮毂电机** | DJI M3508 x4 | FDCAN2 | 编码器4096 CPR, 减速比19:1 |
| **转向电机** | DJI GM6020 x4 | FDCAN1 | 编码器8192 CPR, 直驱1:1 |
| **遥控器** | SBUS协议 | UART5 + DMA | 双缓冲接收, 25字节帧 |
| **云台电机** | DJI M3508 x2 | FDCAN1 | Pitch + Yaw |

### 1.2 FreeRTOS 任务调度
```text
优先级从高到低:
1. INS_TASK      (Realtime)    - 512栈  - IMU读取 + Mahony姿态解算 (1ms)
2. CHASSIS_TASK  (AboveNormal) - 1024栈 - 底盘运动学 + PID控制 (5ms)
3. REMOTE_TASK   (AboveNormal) - 256栈  - 预留遥控处理 (当前空转)
4. defaultTask   (Normal)      - 128栈  - USB设备初始化
```

### 1.3 目录结构
| 路径 | 功能说明 |
|:---|:---|
| `User/APP/` | 应用层任务逻辑 (AGV_chassis_task, INS_task) |
| `User/Algorithm/` | 核心算法 (PID, Mahony, EKF, Kalman, FIFO) |
| `User/Bsp/` | 板级驱动 (CAN, UART, PWM, DWT, USB) |
| `User/Controller/` | 高级控制器 (模糊PID, 前馈, LDOB, 跟踪微分器) |
| `User/Devices/` | 设备驱动 (DJI_Motor, BMI088, NewRemote_Control) |
| `User/Lib/` | 通用工具 (限幅, 死区, 斜坡, OLS最小二乘) |

---

## 2. AGV_chassis_task 核心详解 (重点)

> **文件位置**: `User/APP/AGV_chassis_task.c/.h`
> **执行周期**: 5ms (`vTaskDelayUntil`)
> **核心功能**: 舵轮底盘运动学解算 + 8电机PID双环/单环控制

### 2.1 全局变量与结构体
```c
// 电机数组索引定义
// 0-3: M3508轮毂电机 (左前, 左后, 右后, 右前)
// 4-7: DM6220转向电机 (对应上述顺序)
DJI_Motor_Info_Typedef chassis_motor_info[8];
DJI_Motor_Ctrl_Typedef chassis_motor_ctrl[8];

// 云台电机
// 0: Pitch, 1: Yaw
DJI_Motor_Info_Typedef gimbal_motor_info[2];
DJI_Motor_Ctrl_Typedef gimbal_motor_ctrl[2];

// 控制目标
AGV_chassis_speed_Typedef chassis_speed; // vx, vy, vw
AGV_gimbal_ctrl_Typedef gimbal_ctrl;     // vyaw, vpitch
```

### 2.2 任务执行流程
```c
void AGV_chassis_task(void) {
    // 1. 初始化电机TxID和PID参数
    AGV_classis_Pid_data_Init();
    AGV_Gimbal_Init();
    vTaskDelay(1000); // 等待系统稳定

    // 2. 主循环 (5ms周期)
    while(1) {
        if(systemvalue == Initing) {
            AGV_chassis_Init(); // 6020归零 (耗时约1.5s)
            systemvalue = Running;
        }
        
        RemoteControl();      // 解析遥控器 → 目标速度
        gimbal_control();     // 云台Yaw轴控制
        chassis_control();    // 底盘运动学 + 电机PID控制
        
        vTaskDelayUntil(&xLastWakeTime, 5);
    }
}
```

### 2.3 6020归零初始化 (`AGV_chassis_Init`)
- **目的**: 将转向电机归位到机械中位
- **方法**: 位置环PID控制，目标角度为预定义的机械中位 (`L_Q_6020_Middle_ECD` 等)
- **过程**: 
  1. 循环1500次 (约1.5s)
  2. 每5ms读取编码器，执行过零处理 `AGV_chassis_Zero_Check`
  3. 角度环PID输出 → 速度环PID → 电流指令
  4. 通过FDCAN1发送控制报文
- **关键参数**: 机械中位定义在头文件中，范围 0~8192

### 2.4 遥控器映射 (`RemoteControl`)
```c
// 通道映射 (归一化处理)
chassis_speed.vx = remote_ctrl.rc.ch[3] * 4 / 760.0f; // 最大 4m/s
chassis_speed.vy = remote_ctrl.rc.ch[2] * 4 / 760.0f;
chassis_speed.vw = remote_ctrl.rc.ch[1] * 4 / 760.0f; // 最大 4rad/s
gimbal_ctrl.vyaw = remote_ctrl.rc.ch[0] * 360 / 760.0f;

// 死区处理
if(fabs(chassis_speed.vx) < 0.03) chassis_speed.vx = 0;
if(fabs(chassis_speed.vy) < 0.03) chassis_speed.vy = 0;
```

### 2.5 运动学解算 (核心算法)

#### 角度解算 (`AGV_angle_calc`)
**输入**: 底盘目标速度 `vx, vy, vw`
**输出**: 4个6020电机的目标角度 `out_angle[4]`

1. **计算理论朝向角**: 使用 `atan2` 计算合成速度方向
   ```c
   // 考虑旋转半径的偏移量 (CHASSIS_OFFSET_X/Y)
   atan_angle[0] = atan2(vx + vw*OFFSET_Y, vy + vw*OFFSET_X); // 左前
   // ... 其他轮类似，符号根据位置变化
   ```
2. **转换为编码器值**: 
   ```c
   wheel_angle[i] = Middle_ECD + (atan_angle * 4096.0f / 180.0f);
   ```
3. **角度循环约束**: `loop_f()` 将角度限制在 `0~8192` 范围内
4. **最短路径判断**: `Find_min_Angle()` 
   - 比较目标角度与当前编码器角度的差值
   - 如果差值 > 2048 (半圈)，说明走远路了，设置 `reverse_flag[i] = -1`
   - 用于后续速度环的方向修正

#### 速度解算 (`AGV_speed_calc`)
**输入**: 底盘目标速度 + `reverse_flag`
**输出**: 4个3508电机的目标转速 `out_speed[4]` (单位: RPM)

1. **计算轮子线速度**: 
   ```c
   wheel_speed[0] = sqrt((vx + vw*OFFSET_Y)^2 + (vy + vw*OFFSET_X)^2) * wheel_rpm_ratio;
   ```
2. **转换为RPM**: `wheel_rpm_ratio = 786.43 / WHEEL_PERIMETER`
3. **方向修正**: `out_speed[i] = reverse_flag[i] * wheel_speed[i] * -1`
   - `reverse_flag`: 处理6020反转时的轮子方向
   - `*-1`: 3508物理安装方向导致需要反向

### 2.6 PID控制策略 (`chassis_control`)

**执行顺序**:
1. 调用 `AGV_angle_calc()` 和 `AGV_speed_calc()` 获取目标值
2. **6220转向电机控制 (双环PID)**:
   ```c
   // 角度环 (位置式PID)
   Out_temp = PID_Calc(&Angle_pid, Act_Encoder, Target_Angle);
   // 速度环 (角度环输出作为速度环目标)
   SET_Current = PID_Calc(&Speed_pid, Act_Velocity, Out_temp);
   ```
3. **3508轮毂电机控制 (单环PID)**:
   ```c
   // 速度环 (直接输出电流)
   SET_Current = PID_Calc(&Speed_pid, Act_Velocity, Target_RPM);
   ```
4. **CAN发送**:
   - `DJI_Motor_ctrl_6020()` → FDCAN1
   - `DJI_Motor_ctrl()` → FDCAN2

### 2.7 云台控制 (`gimbal_control`)
- **控制对象**: Yaw轴电机 (3508)
- **控制模式**: 速度环PID (单环)
- **目标值**: 遥控器右摇杆水平通道映射的角度速度 `gimbal_ctrl.vyaw`
- **实现**: `PID_Calc(&gimbal_motor_ctrl[yaw].Speed_pid, Velocity, vyaw)`

---

## 3. INS_task 姿态解算

> **文件位置**: `User/APP/INS_task.c/.h`
> **执行周期**: 1ms (`osDelay(1)`)

### 3.1 核心流程
1. **读取IMU**: `BMI088_Read()` 获取加速度和角速度
2. **Mahony滤波**: 
   - `mahony_input()` → `mahony_update()` → `mahony_output()`
   - 输出四元数 `q[4]` 和欧拉角 `roll, pitch, yaw`
3. **运动加速度提取**:
   - 重力向量从大地系转机体系: `EarthFrameToBodyFrame()`
   - 从加速度计数据减去重力分量
   - 一阶低通滤波: `INS.MotionAccel_b[i] = ...`
   - 转回大地系: `BodyFrameToEarthFrame()`
4. **零漂处理**: 当加速度 < 阈值时清零
5. **速度/位移积分** (3s后启动):
   - `v_n = v_n + MotionAccel_n * dt`
   - `x_n = x_n + v_n * dt`
6. **偏航角累计**: 处理 `yaw` 跨越 ±π 的圈数计数

---

## 4. 关键算法库

### 4.1 PID控制器 (`User/Algorithm/PID/`)
- **结构体**: `PidTypedef` (位置式/增量式)
- **核心函数**: `PID_Calc(fdb, ref)`
- **特性**:
  - 积分限幅 (`max_iout`)
  - 输出限幅 (`max_out`)
  - 误差缓冲 `error[3]` 用于微分项计算

### 4.2 高级控制器 (`User/Controller/`)
- **模糊PID**: 7x7规则表，根据误差E和误差变化率EC自适应调整Kp/Ki/Kd
- **前馈控制 (Feedforward)**: 基于参考信号的一阶/二阶导数计算前馈量
- **线性扰动观测器 (LDOB)**: 估计系统总扰动并补偿
- **跟踪微分器 (TD)**: 安排过渡过程，提取高质量微分信号

### 4.3 最小二乘法 (OLS) (`User/Lib/`)
- **用途**: 信号平滑和微分提取 (比直接差分噪声小)
- **核心**: `OLS_Derivative()` 返回拟合直线的斜率k
- **应用**: 用于PID的微分项和Feedforward/LDOB的导数计算

### 4.4 工具函数 (`User/Lib/`)
- `float_constrain()`: 限幅
- `float_deadband()`: 死区处理
- `loop_float_constrain()`: 循环限幅 (用于角度)
- `ramp_calc()`: 斜坡函数 (平滑目标值变化)
- `sign()`: 符号函数

---

## 5. BSP与设备驱动

### 5.1 FDCAN (`User/Bsp/bsp_can.c`)
- **配置**: 3个FDCAN实例，过滤器设为全接收 (FilterID1/2 = 0)
- **发送**: `canx_send_data()` 封装标准帧发送
- **中断接收**: 
  - `HAL_FDCAN_RxFifo0Callback`: FDCAN1/3 → 更新6020/云台电机数据
  - `HAL_FDCAN_RxFifo1Callback`: FDCAN2 → 更新3508电机数据
- **数据解析**: `DJI_Motor_Info_Update()` 解析8字节CAN数据，处理过零和累计角度

### 5.2 SBUS遥控器 (`User/Bsp/bsp_uart.c`)
- **接收方式**: UART5 + DMA双缓冲 (`DMA_SxCR_DBM`)
- **中断**: `HAL_UARTEx_RxEventCallback` (IDLE线空闲中断)
- **解析**: `SBUS_TO_RC()` 将25字节SBUS帧解包为6通道+4拨杆数据
- **特点**: 使用AXI_SRAM存放缓冲区，提高DMA效率

### 5.3 BMI088 (`User/Devices/BMI088/`)
- **接口**: SPI
- **初始化**: 
  - 软件复位 → 配置量程/带宽/中断
  - 可选校准模式: `Calibrate_MPU_Offset()` 采集20000次数据计算零偏
- **数据读取**: 批量读取加速度(6字节)和陀螺仪(8字节)
- **数据处理**: 原始值 * 灵敏度 - 零偏 → 物理量 (m/s², rad/s)

---

## 6. 关键参数速查表

| 参数 | 值 | 说明 |
|:---|:---|:---|
| `L_Q_6020_Middle_ECD` | 7509 | 左前6020机械中位 |
| `L_H_6020_Middle_ECD` | 6144 | 左后6020机械中位 |
| `R_H_6020_Middle_ECD` | 4779 | 右后6020机械中位 |
| `R_Q_6020_Middle_ECD` | 2048 | 右前6020机械中位 |
| `Speed_3508_KP` | 13.0 | 3508速度环P |
| `Speed_3508_KI` | 0.03 | 3508速度环I |
| `Speed_6020_KP` | 80.0 | 6020速度环P |
| `Angle_6020_KP` | 1.0 | 6020角度环P |
| `CHASSIS_OFFSET_X` | 0.18m | 底盘半宽 |
| `CHASSIS_OFFSET_Y` | 0.20m | 底盘半长 |
| `WHEEL_RADIUS` | 0.065m | 轮子半径 |
| `WHEEL_PERIMETER` | 0.408m | 轮子周长 |

---

## 7. 调试与注意事项

1. **CAN ID 冲突**: FDCAN1同时接收6020和云台电机，需在回调中正确区分ID范围
2. **6020过零处理**: 角度跨越0/8192边界时必须正确处理，否则PID会突变
3. **电机方向**: 3508物理安装方向可能需要 `*-1` 修正，代码中已体现
4. **姿态解算延迟**: INS积分在启动3s后才生效，避免初始瞬态影响
5. **遥控器死区**: 硬件摇杆中立点不精准，软件死区 0.03 和 5 是必要的
6. **机械中位标定**: 首次使用必须准确测量并修改 `*_Middle_ECD` 宏定义

---

---

## 8. PID 算法核心学习笔记 (重点补充)

> **文件位置**: `User/Algorithm/PID/pid.c` 与 `pid.h`
> **适用场景**: 舵轮底盘速度/角度控制、云台控制等所有闭环控制场景。
> **说明**: 本项目使用的是**经典数字位置式/增量式 PID**，针对嵌入式环境进行了优化（如限幅、防积分饱和）。

### 8.1 PID 函数接口与使用指南

本节讲解 `pid.c` 提供的三个核心 API 怎么用，以及新手最容易踩的坑。

#### 1. `PID_init` - 初始化函数
- **原型**: `void PID_init(PidTypedef *pid, PID_mode_e mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)`
- **参数说明**:
  - `pid`: 指向你要初始化的 PID 结构体实例。
  - `mode`: 选择模式，`PID_POSITION`（位置式，输出绝对值）或 `PID_DELTA`（增量式，输出变化量）。本项目主要用位置式。
  - `PID[3]`: 浮点数组，依次存放 `{Kp, Ki, Kd}`。
  - `max_out`: **总输出限幅**（例如电机电流上限 16000）。
  - `max_iout`: **积分项限幅**（防止积分饱和，通常设为总限幅的一半或更小）。
- **使用场景**: 在系统启动或任务创建后的初始化阶段调用，通常**只调用一次**。
- **⚠️ 注意事项**:
  - **防重复初始化机制**: 代码内部检查 `if(pid->Initlized != true)`。这意味着如果你在程序运行中想修改 PID 参数，再次调用此函数是**无效**的，因为它会跳过赋值。
  - **如何动态改参**: 如果想在线调整参数，请直接修改结构体成员（如 `pid->Kp = new_val;`），不要依赖此函数。

#### 2. `PID_Calc` - 核心计算函数
- **原型**: `fp32 PID_Calc(PidTypedef *pid, fp32 fdb, fp32 ref)`
- **参数说明**:
  - `pid`: 正在使用的 PID 结构体。
  - `fdb` (Feedback): **反馈值**，来自传感器（编码器角度、转速等）。
  - `ref` (Reference): **目标值**，你希望电机达到的状态。
- **返回值**: 计算出的控制输出量（通常是电流值）。
- **使用场景**: 放在主控制循环（如 5ms 周期任务）中，**每次循环调用一次**。
- **⚠️ 注意事项**:
  - **参数顺序**: `fdb` 在前，`ref` 在后。代码内部计算误差公式为 `Error = ref - fdb`（目标 - 反馈）。
  - **方向反了怎么办**: 如果电机转动方向与预期相反，**不要**改接线！尝试将 Kp 改为负值，或者交换传入的 fdb 和 ref 顺序。
  - **空指针保护**: 代码开头检查了 `if (pid == NULL)`，但为了性能，建议确保传入的地址一定有效。

#### 3. `PID_clear` - 清除/复位函数
- **原型**: `void PID_clear(PidTypedef *pid)`
- **使用场景**: 当系统发生错误复位、急停、或从手动切回自动模式时调用，用于消除累积的历史误差（特别是积分项）。
- **⚠️ 注意事项**:
  - **副作用**: 该函数会将 `pid->Initlized` 重置为 `false`。
  - **严重后果**: 如果你在控制循环中调用了 `PID_clear`，下一次调用 `PID_Calc` 时，由于检测到未初始化，计算结果会全为 0，电机会失去控制力。
  - **最佳实践**: 除非你想彻底禁用该 PID 环，否则尽量不要在运行中调用 `PID_clear`。如果只是想消除积分累积，建议手动清零 `pid->Iout = 0;` 和误差数组，保留 `Initlized` 标志。

### 8.2 PID 结构体详解 (`PidTypedef`)
结构体是 PID 算法的“灵魂”，它不仅仅存储参数，还记录了历史状态。

```c
typedef struct {
    PID_mode_e pid_mode;    // 控制模式：0=位置式 (输出绝对值), 1=增量式 (输出变化量)
    bool Initlized;         // 初始化标志：未初始化时计算直接归零，防止野指针或垃圾数据
    fp32 Kp, Ki, Kd;        // 核心三参数：比例、积分、微分系数
    
    fp32 max_out;           // 最终输出限幅值 (如电机电流最大 16000)
    fp32 max_iout;          // 积分项限幅值 (防止积分饱和/积分风暴)
    
    fp32 set, fdb;          // set: 目标值 (Target), fdb: 反馈值 (Actual)
    fp32 error[3];          // 误差历史: [0]当前, [1]上一次, [2]上上次 (用于计算微分)
    
    fp32 Pout, Iout, Dout;  // 分解输出：方便单独查看 P/I/D 三项的贡献值
    fp32 out;               // 最终输出结果 (Pout + Iout + Dout)
    
    fp32 Dbuf[3];           // 微分专用缓冲: 0=当前误差差值, 1=上次差值...
} PidTypedef;
```

### 8.3 两种 PID 模式对比

| 模式 | 公式逻辑 | 特点 | 本项目应用场景 |
|:---|:---|:---|:---|
| **位置式 (PID_POSITION)** | `Out = Kp*Err + Ki*ΣErr + Kd*(Err-Last_Err)` | 输出是**控制量的绝对值**。<br>积分项会不断累积，容易产生超调。 | **舵轮 6020 角度环**<br>**3508 速度环**<br>(需要直接控制电流大小) |
| **增量式 (PID_DELTA)** | `Out += Kp*(Err-Last) + Ki*Err + Kd*(Err-2*Last+Last2)` | 输出是**控制量的变化量**。<br>累加到上一次输出上。误动作影响小，切换无冲击。 | 暂未使用<br>(通常用于步进电机控制或需要平滑切换的场合) |

### 8.4 核心计算流程 (`PID_Calc`)

代码执行步骤非常严密，建议按照以下顺序理解：

1. **更新误差 (Error)**:
   ```c
   pid->error[2] = pid->error[1]; // 数据搬家：上上次 -> 上上次
   pid->error[1] = pid->error[0]; // 数据搬家：上次 -> 上上次
   pid->error[0] = ref - fdb;     // 计算当前误差：目标 - 反馈
   ```

2. **计算三项输出 (以位置式为例)**:
   - **P (比例)**: `Pout = Kp * error[0]` -> 反应速度最快，误差大时出力大。
   - **I (积分)**: `Iout += Ki * error[0]` -> 消除稳态误差，但**必须限幅**！
   - **D (微分)**: `Dout = Kd * (error[0] - error[1])` -> 预测未来趋势，抑制超调。
     - *注意*：代码里用了 `Dbuf` 数组来存储误差的差值，这是一种为了平滑计算的写法。

3. **限幅保护 (至关重要！)**:
   ```c
   LimitMax(pid->Iout, pid->max_iout); // 先限制积分项
   pid->out = pid->Pout + pid->Iout + pid->Dout; // 相加
   LimitMax(pid->out, pid->max_out);   // 再限制总输出
   ```
   - **为什么先限制积分？** 如果积分项无限累积（积分饱和），一旦误差反向，积分项需要很久才能减下来，导致电机“刹不住车”或“反应迟钝”。这就是**抗积分饱和 (Anti-windup)** 技术。

### 8.5 初学者避坑指南 (重要细节)

1. **初始化状态 (`Initlized` 标志)**:
   - 在 `PID_init` 中，只有 `Initlized != true` 时才会赋值参数，并置 `true`。
   - **坑**: 如果你在运行中想修改 PID 参数（比如动态调参），直接调 `PID_init` 是没用的，必须**手动修改**结构体里的 `Kp/Ki/Kd`，或者先调用 `PID_clear()` 把标志位复位。
   - 计算函数里有一个检查：`if(pid->Initlized == true)`，如果没初始化，计算结果全是 0，电机不会动！

2. **误差方向 (Ref - Fdb)**:
   - 代码逻辑是 `error = ref - fdb` (目标 - 实际)。
   - 如果发现电机**反转**（本该正转结果反向猛转），不要急着改接线，先试着把 `Kp` 改成负数，或者交换 `ref` 和 `fdb` 的顺序。

3. **清零操作 (`PID_clear`)**:
   - **什么时候需要清零？**
     - 切换控制模式时（比如从自动切回手动）。
     - 报错复位时。
     - 电机急停重新启动时。
   - **注意**: `PID_clear` 会把 `Initlized` 设为 `false`！这意味着清零后，下一次调用 `PID_Calc` 会直接返回 0。**正确做法**：清零后，如果需要继续控制，必须确保参数已经配置好，或者不要调用 `PID_clear`，而是手动将 `out`, `Iout`, `error` 清零。
     *(注：原代码 `PID_clear` 把 Initlized 设为 false 其实是一个设计上的小隐患，导致清零后无法立即复用，除非重新 init 或手动改回 true)*

4. **调试建议**:
   - **先调 P**: I 和 D 设为 0。加大 P 直到电机开始抖动，然后回调到不抖动的 60%-70%。
   - **再加 D**: 如果电机响应慢、有超调，加一点 D。
   - **最后加 I**: 如果电机停不下来（有稳态误差），慢慢加 I。舵轮速度环的 I 通常很小 (如 0.03)。

### 8.6 `pid.c` 与 `AGV_chassis_task.c` 的实战联系

> **核心逻辑**: `AGV_chassis_task.c` 负责**业务逻辑**（我要怎么动），`pid.c` 负责**底层算法**（怎么算出力矩）。两者通过 **PID 结构体**紧密连接。

#### 1. 纽带：电机控制结构体数组
在 `AGV_chassis_task.c` 中定义了全局数组，它是 PID 算法的“家”：
```c
// 0-3: 3508 (轮毂), 4-7: 6020 (转向)
DJI_Motor_Ctrl_Typedef chassis_motor_ctrl[8];
```
`DJI_Motor_Ctrl_Typedef` 内部包含了 `Angle_pid` 和 `Speed_pid` 两个 `PidTypedef` 结构体实例。这意味着每个电机都自带了专属的 PID 计算器。

#### 2. 初始化阶段：分发参数 (`AGV_classis_Pid_data_Init`)
底盘启动时，会调用此函数，将头文件 (`AGV_chassis_task.h`) 里定义好的宏参数灌入 PID 结构体：
```c
// 遍历 8 个电机
for(uint8_t i = 0; i < 8; i++) {
    if(i < 4) { // 3508 只需要速度环
        PID_init(&chassis_motor_ctrl[i].Speed_pid, PID_POSITION, Spe_3508_PID, ...);
    } else {    // 6020 需要双环 (角度 + 速度)
        PID_init(&chassis_motor_ctrl[i].Speed_pid, PID_POSITION, Spe_6020_PID, ...);
        PID_init(&chassis_motor_ctrl[i].Angle_pid, PID_POSITION, Ang_6020_PID, ...);
    }
}
```
**关键点**: 这里通过循环和索引区分了电机类型，分别赋予不同的 Kp/Ki/Kd。

#### 3. 控制阶段：串联双环 (`chassis_control`)
这是最核心的逻辑。在 5ms 的任务循环中，PID 计算将**目标解算值**与**传感器反馈值**结合：

- **3508 轮毂电机 (单环速度控制)**:
  ```c
  // 反馈：电机当前转速 (Data.Velocity)
  // 目标：运动学解算出的转速 (out_speed[i])
  // 输出：直接变成设定电流 (SET_Current)
  chassis_motor_info[i].Data.SET_Current = PID_Calc(
      &chassis_motor_ctrl[i].Speed_pid,   // 使用专属计算器
      chassis_motor_info[i].Data.Velocity,
      out_speed[i]
  );
  ```

- **6220 转向电机 (双环级联控制)**:
  - **外环 (角度环)**: 算出“需要转多快”
    ```c
    // 目标：运动学解算的角度 (out_angle[i])
    // 反馈：编码器角度 (Data.Encoder)
    // 输出：作为内环的目标速度！
    Out_temp = PID_Calc(
        &chassis_motor_ctrl[i+4].Angle_pid, 
        chassis_motor_info[i+4].Data.Encoder, 
        out_angle[i]
    );
    ```
  - **内环 (速度环)**: 算出“需要给多大电流”
    ```c
    // 目标：外环输出的 Out_temp
    // 反馈：电机当前转速 (Data.Velocity)
    // 输出：设定电流 (SET_Current)
    chassis_motor_info[i+4].Data.SET_Current = PID_Calc(
        &chassis_motor_ctrl[i+4].Speed_pid, 
        chassis_motor_info[i+4].Data.Velocity, 
        Out_temp
    );
    ```

#### 4. 数据流向总结图
```text
[AGV_chassis_task.c - 业务层]
       |
       +-- 遥控器/运动学解算 --> [out_angle / out_speed] (PID 的 Target)
       |
       +-- CAN 中断接收 ------> [Data.Encoder / Data.Velocity] (PID 的 Feedback)
       |
       v
[调用 PID_Calc] ---> [Algorithm/PID/pid.c - 算法层]
       |                   |
       |                   +-- 读取 Kp, Ki, Kd, max_out
       |                   +-- 计算误差 P+I+D
       |                   +-- 执行限幅 (Anti-Windup)
       |                   v
       +-- 返回 [Output] ---> 写入 [Data.SET_Current] (发送给电机)
```

---

## 9. AGV_chassis_task.c 函数调用与引用全景图

> **说明**: 本节汇总了 `AGV_chassis_task.c` 文件头部引用 (`#include`) 的所有头文件，以及代码中实际调用的函数和访问的全局变量。理解这些调用关系是理清代码逻辑脉络的关键。

### 9.1 操作系统服务 (`freertos.h`, `task.h`)
提供实时操作系统的调度与同步服务。

| 函数/接口 | 代码位置 | 具体作用 |
|:---|:---|:---|
| `vTaskDelay()` | 任务初始化、归零循环 | **绝对延时**。挂起任务指定时间（如 1000ms），用于等待传感器稳定或机械归位完成。 |
| `xTaskGetTickCount()` | 主循环开始 | **获取系统节拍**。记录当前系统时间滴答数，配合延时函数使用。 |
| `vTaskDelayUntil()` | 主循环末尾 | **周期性延时**。确保任务严格按照设定的周期（5ms）运行，消除任务执行耗时带来的时间误差。 |

### 9.2 电机驱动控制 (`DJI_Motor.h`)
封装了 CAN 通信的底层细节，向上层应用提供简洁的电机控制接口。

| 函数 | 代码位置 | 具体作用 |
|:---|:---|:---|
| `DJI_Motor_ctrl()` | `chassis_control` (3508), `gimbal_control` (云台) | **组帧发送 (标准)**。将 4 个电机的电流指令打包成 CAN 报文（ID 通常为 0x200），发送至指定总线。 |
| `DJI_Motor_ctrl_6020()` | `AGV_chassis_Init`, `chassis_control` | **组帧发送 (转向专用)**。专门针对 6020 电机封装，将 4 个转向电机的电流指令打包（ID 0x1FF）发送至 CAN1。 |

### 9.3 核心控制算法 (`pid.h`)
提供经典的数字 PID 控制算法（位置式/增量式）。该头文件通常通过 `DJI_Motor.h` 间接包含。

| 函数 | 代码位置 | 具体作用 |
|:---|:---|:---|
| `PID_init()` | `AGV_classis_Pid_data_Init` | **参数配置**。一次性设置 PID 模式、Kp/Ki/Kd 参数数组、输出限幅值 (`max_out`) 和积分限幅值 (`max_iout`)。 |
| `PID_Calc()` | `chassis_control`, `gimbal_control`, 归零逻辑 | **闭环计算**。输入当前反馈值和目标值，返回计算出的控制量（通常是电流值）。这是实现速度环和角度环的核心。 |

### 9.4 外部状态读取 (数据依赖)
通过包含对应头文件，访问由**中断回调**或**其他后台任务**维护的全局变量，获取外部输入状态。

| 引用头文件 | 访问对象 | 具体作用 |
|:---|:---|:---|
| `New_Remote_Control.h` | 全局变量 `remote_ctrl` | **获取用户意图**。读取 SBUS 遥控器解析后的通道原始值（如 `rc.ch[3]`），将其映射为底盘的目标速度。 |
| `ins_task.h` | 全局变量 `INS` | **获取姿态信息**。读取 IMU 解算后的欧拉角 (`Roll`, `Pitch`, `Yaw`) 或加速度数据（当前代码中仅用于注释掉的调试打印）。 |

### 9.5 调试与辅助 (`bsp_uart.h`, `AGV_chassis_task.h`)
辅助调试打印以及本模块内部的声明。

| 头文件 | 作用说明 |
|:---|:---|
| `bsp_uart.h` | 提供串口通信的底层支持。虽然代码中未显式调用其函数，但通过 `printf` 重定向（`fputc`）实现了将调试信息打印到串口助手的功能。 |
| `AGV_chassis_task.h` | **本模块头文件**。定义了所有的结构体（如 `chassis_speed`）、宏定义（如 PID 参数）以及本 `.c` 文件中实现的全局函数声明。 |

---

> **最后更新**: 2026-05-01
> **维护者**: AGV开发团队