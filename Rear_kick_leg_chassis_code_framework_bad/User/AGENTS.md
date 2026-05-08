# AGV项目代码深度学习记录

> **项目来源**: `D:\DeskTop\HALProject\Rear_kick_leg_chassis_code_framework`
> **目标平台**: STM32H723VBT6 (CtrlBoard-H7_IMU)
> **控制对象**: 四轮舵轮底盘 (4x M3508 + 4x DM6220) + 机械臂
> **RTOS**: FreeRTOS

---

## 1. 项目架构总览

### 1.1 硬件映射
| 模块 | 型号/规格 | 通信接口 | 备注 |
|:---|:---|:---|:---|
| **MCU** | STM32H723VBT6 | - | 主频480MHz, 带3路FDCAN |
| **IMU** | BMI088 | SPI | 加速度±6g, 陀螺仪±2000°/s |
| **轮毂电机** | DJI M3508 x4 | FDCAN2 | 编码器4096 CPR, 减速比19:1 |
| **转向电机** | DM6220 x4 | FDCAN1 | 位置±π范围(弧度制), 软件双环PID控制 |
| **遥控器** | SBUS协议 | UART5 + DMA | 双缓冲接收, 25字节帧, 通道范围±660 |
| **机械臂电机** | DM-J4310/J4340/J8009 | FDCAN1 | 位置模式控制 (与转向DM共用FDCAN1) |

### 1.2 FreeRTOS 任务调度
```text
优先级从高到低:
1. INS_TASK      (Realtime)    - 512栈  - IMU读取 + Mahony姿态解算 (1ms)
2. CHASSIS_TASK  (AboveNormal) - 1024栈 - 底盘运动学 + 3508速度环 + DM6220双环PID + 机械臂DM控制 (5ms)
3. defaultTask   (Normal)      - 128栈  - USB设备初始化
```

### 1.3 目录结构
| 路径 | 功能说明 |
|:---|:---|
| `User/APP/` | 应用层任务逻辑 (AGV_chassis_task, INS_task, Manipulator_Task) |
| `User/Algorithm/` | 核心算法 (PID, Mahony, EKF, Kalman, FIFO) |
| `User/Bsp/` | 板级驱动 (CAN, UART, PWM, DWT, USB) |
| `User/Controller/` | 高级控制器 (模糊PID, 前馈, LDOB, 跟踪微分器) |
| `User/Devices/` | 设备驱动 (DJI_Motor, DM_Motor, BMI088, Remote_Control) |
| `User/Lib/` | 通用工具 (限幅, 死区, 斜坡, OLS最小二乘) |

---

## 2. 大更改计划 (2026-05-03)

### 2.1 FDCAN分配变更（按电机ID分组）
| FDCAN | 用途 | 连接的电机 | 说明 |
|:---|:---|:---|:---|
| FDCAN1 | 电机总线1 | DM6220(0x02索引1,0x03索引2) + 3508(0x202索引1,0x203索引2) | 索引1,2 |
| FDCAN2 | 电机总线2 | DM6220(0x01索引0,0x04索引3) + 3508(0x201索引0,0x204索引3) | 索引0,3 |
| FDCAN3 | 暂时不用 | - | 保留扩展 |

**电机索引分配：**
- 索引0：左前（DM6220 ID 0x01 → FDCAN2，3508 RxID 0x201 → FDCAN2）
- 索引1：左后（DM6220 ID 0x02 → FDCAN1，3508 RxID 0x202 → FDCAN1）
- 索引2：右后（DM6220 ID 0x03 → FDCAN1，3508 RxID 0x203 → FDCAN1）
- 索引3：右前（DM6220 ID 0x04 → FDCAN2，3508 RxID 0x204 → FDCAN2）

**FDCAN选择判断：**
- 索引1,2 → FDCAN1
- 索引0,3 → FDCAN2

**注意：** 发送和接收都需要根据索引选择对应的FDCAN句柄。

### 2.2 DM6220控制方式变更
- **原方案**: 使用DM6220内置PID，位置速度模式 (`Pos_mode`)，发送目标位置和速度，kp/kd传入电机
- **新方案**: **软件双环PID控制**，不再使用电机内置PID
  - 外环(角度环): 目标角度 → PID计算 → 目标速度
  - 内环(速度环): 目标速度 → PID计算 → 输出扭矩/电流
  - 控制模式: **MIT模式** (`Mit_mode`)，直接发送位置、速度、kp、kd、扭矩指令
  - 注意: kp/kd在MIT模式中设为0，因为PID在软件中实现

### 2.3 需要修改的文件清单（更新于2026-05-03）

1. **`User/APP/AGV_chassis_task.c`** ✅ 已完成
   - [x] 启用DM6220双环PID初始化 (`AGV_classis_Pid_data_Init`)
   - [x] 修改3508按FDCAN分组发送（FDCAN1:索引1,2；FDCAN2:索引0,3）
   - [x] 修改DM6220根据索引选择FDCAN（索引1,2→FDCAN1，索引0,3→FDCAN2）
   - [x] 启用`chassis_control`中的DM6220双环PID控制
   - [x] `DM_Motor_Ctrl`调用改为MIT模式，kp/kd=0(软件PID)
   - [x] 更新`AGV_chassis_Init`归零逻辑，使用双环PID
   - [x] 更新`DM6220_Init`根据索引选择FDCAN

2. **`User/Bsp/bsp_can.c`** ✅ 已完成
   - [x] 修改FDCAN1回调: DM6220(0x12,0x13) + 3508(0x202,0x203)
   - [x] 修改FDCAN2回调: DM6220(0x11,0x14) + 3508(0x201,0x204)
   - [x] 修改FDCAN2过滤器配置: 接收所有标准ID，在回调中筛选
   - [x] 添加`#include "DM_Motor.h"`
   - [x] 修正extern声明: `chassis_dm_info` 和 `chassis_dm_ctrl`

3. **`User/Devices/DM_Motor/DM_Motor.c`** ✅ 已完成
   - [x] 转向DM电机模式从`Pos_mode`改为`Mit_mode`
   - [x] 修正`DM_Motor_Ctrl`函数定义，去掉`volatile`

4. **`User/APP/AGV_chassis_task.h`** ⏳ 待完成
   - [ ] 添加DM6220双环PID参数宏定义 (Angle_DM6220_KP/KI/KD, Speed_DM6220_KP/KI/KD) - 当前为0.00f
   - [ ] 添加MIT模式kp/kd/扭矩参数宏定义 (DM6220_KP/DM6220_KD/DM6220_TOR)

5. **`User/Devices/DJI_Motor/DJI_Motor.c`** ✅ 已完成
   - [x] 3508按FDCAN分组发送，接收在对应FDCAN回调中处理

**已知待修复问题：**
- [ ] `bsp_can.c` 第36行 extern 声明可能仍有类型不匹配问题
- [ ] 函数调用参数缺少逗号（如`PID_Calc`, `DM_Motor_Ctrl`等）
- [ ] `AGV_angle_calc`中`Find_min_Angle`调用时参数可能需要调整

---

## 3. AGV_chassis_task 核心详解 (更新后)

> **文件位置**: `User/APP/AGV_chassis_task.c/.h`
> **执行周期**: 5ms (`vTaskDelayUntil`)
> **核心功能**: 舵轮底盘运动学解算 + 8电机PID控制(3508速度环 + DM6220双环PID)

### 3.1 全局变量与结构体
```c
// 电机数组索引定义
// 0-3: M3508轮毂电机 (左前, 左后, 右后, 右前)
// 0-3: DM6220转向电机 (对应上述顺序)
DJI_Motor_Info_Typedef chassis_motor_info[4];  // 3508信息
DJI_Motor_Ctrl_Typedef chassis_motor_ctrl[4];   // 3508控制
DM_Motor_Info_Typedef chassis_dm_info[4];       // DM6220信息
DM_Motor_Ctrl_Typedef chassis_dm_ctrl[4];       // DM6220控制(含双环PID)

// 上一次DM转向目标角度
float wheel_angle_last[4];
// 3508反转标志（最短路径判断用）
volatile int8_t reverse_flag[4] = {1,1,1,1};
// 系统状态（Initing/Running）
SystemValue systemvalue = Initing;
// 控制目标
AGV_chassis_speed_Typedef chassis_speed; // vx, vy, vw
```

### 3.2 任务执行流程
```c
void AGV_chassis_task(void) {
    // 1. 初始化3508电机CAN ID（Tx:0x200, Rx:0x201~0x204对应4个轮毂电机）
    for (uint8_t i = 0; i < 4; i++) {
        chassis_motor_info[i].Motor_Type = DJI_M3508;
        chassis_motor_info[i].ID_Set.TxIdentifier = 0x200;
        chassis_motor_info[i].ID_Set.RxIdentifier = 0x200 + i + 1; // 0x201~0x204
    }

    // 2. 拷贝DM6220转向电机全局配置（ID、模式等）
    for (uint8_t i = 0; i < 4; i++) {
        chassis_dm_info[i] = AGV_Rotate_DM_Motor[i];
        // 注意: 模式需要从Pos_mode改为Mit_mode(在DM_Motor.c中修改)
    }

    // 3. 初始化PID参数（3508速度环 + DM6220双环）
    AGV_classis_Pid_data_Init(); // 将启用DM6220的PID初始化

    // 4. 使能DM6220电机（MIT模式）
    DM6220_Init(); // 需要修改为MIT模式使能

    // 5. 等待系统稳定
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 主循环（5ms周期）
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = 5;
    while (1) {
        if (systemvalue == Initing) {
            AGV_chassis_Init(); // DM6220归零（使用双环PID控制）
            systemvalue = Running;
        }
        
        RemoteControl();      // 解析遥控器 → 底盘速度
        chassis_control();    // 3508速度控制 + DM6220双环PID控制
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
```

### 3.3 DM6220归零初始化 (`AGV_chassis_Init`) - 更新后
- **目的**: 将转向电机归位到机械中位，使用软件双环PID控制
- **方法**: 
  1. 初始化目标角度为预定义的机械中位（弧度制）
  2. 初始化`wheel_angle_last`记录上一次目标角度
  3. **使用双环PID控制归零**: 角度环 → 速度环 → MIT模式指令
- **过程**: 
  1. 循环1500次 (约1.5s，每1ms延时)
  2. 每5ms读取电机位置，执行过零处理（`AGV_chassis_Zero_Check`，周期2π）
  3. 角度环PID输出 → 速度环PID → MIT模式指令
- **关键函数**:
  - `AGV_chassis_Zero_Check`: 过零处理，处理DM6220位置范围±π的循环
  - `Find_min_Angle`: 最短路径判断，返回`reverse_flag`（-1需反转，1正常）
- **关键参数**: 机械中位需要实际测量，位置范围 ±π (约±3.14159)，周期2π

### 3.4 遥控器映射 (`RemoteControl`)
```c
// 大疆SBUS遥控器通道映射（范围-660~660，归一化到±4m/s或±4rad/s）
chassis_speed.vx = (float)remote_ctrl.rc.ch[3] * 4 / 660.0f;  // 前后速度，最大4m/s
chassis_speed.vy = (float)remote_ctrl.rc.ch[2] * 4 / 660.0f;  // 左右速度，最大4m/s
chassis_speed.vw = (float)remote_ctrl.rc.ch[1] * 4 / 660.0f;  // 旋转角速度，最大4rad/s

// 死区处理（硬件摇杆中性位不精准）
if (fabs(chassis_speed.vx) < 0.03f) chassis_speed.vx = 0;
if (fabs(chassis_speed.vy) < 0.03f) chassis_speed.vy = 0;

// 云台yaw控制（通道0映射）
gimbal_ctrl.vyaw = (float)remote_ctrl.rc.ch[0] * 360 / 660.0f;
```

### 3.5 运动学解算

#### 角度解算 (`AGV_angle_calc`，已启用)
**输入**: 底盘目标速度 `vx, vy, vw`
**输出**: 4个DM6220电机的目标角度 `out_angle[4]` (单位: 弧度，范围±π)

1. **计算理论朝向角**: 使用 `atan2(y, x)` 计算合成速度方向（弧度制）
    ```c
    // 左前轮：y分量=vy + vw*CHASSIS_OFFSET_X，x分量=vx + vw*CHASSIS_OFFSET_Y
    atan_angle[0] = atan2((speed->vy + speed->vw * CHASSIS_OFFSET_X),
                          (speed->vx + speed->vw * CHASSIS_OFFSET_Y));
    // 其他轮类似，调整OFFSET符号
    ```
2. **转换为DM6220位置值**: 直接加上机械中位（弧度制）
    ```c
    wheel_angle[i] = Middle_Pos[i] + atan_angle[i];
    ```
3. **循环限幅**: 使用 `loop_f()` 处理DM6220位置的周期性（±π，周期2π）
4. **最短路径判断**: `Find_min_Angle()` 比较目标角度与当前电机位置，设置 `reverse_flag[i]`
5. **零速保持**: 当`vx=vy=vw=0`时，保持上一次的目标角度`wheel_angle_last[i]`

#### 速度解算 (`AGV_speed_calc`，已启用)
**输入**: 底盘目标速度 `vx, vy, vw` + `reverse_flag`
**输出**: 4个3508电机的目标转速 `out_speed[4]` (单位: RPM)

1. **计算轮子线速度**: 合成平移速度+旋转附加速度的矢量模长
2. **转换为RPM**: `wheel_rpm_ratio = 786.43f / WHEEL_PERIMETER`
3. **方向修正**: `out_speed[i] = reverse_flag[i] * (int16_t)wheel_speed[i] * -1`

### 3.6 PID控制策略 (`chassis_control`，更新后)

**执行顺序**:
1. 调用 `AGV_angle_calc()` 获取DM6220目标角度 `out_angle[4]`
2. 调用 `AGV_speed_calc()` 获取3508目标转速 `out_speed[4]`
3. **DM6220转向电机控制 (双环PID + MIT模式)**:
    ```c
    for (uint8_t i = 0; i < 4; i++) {
        // 外环: 角度环PID
        float angle_out = PID_Calc(&chassis_dm_ctrl[i].Angle_pid,
                                   chassis_dm_info[i].Data.pos,  // DM电机反馈位置(弧度)
                                   out_angle[i]);
        // 内环: 速度环PID
        float speed_out = PID_Calc(&chassis_dm_ctrl[i].Speed_pid,
                                   chassis_dm_info[i].Data.vel,  // DM电机反馈速度
                                   angle_out);
        // 发送MIT模式指令 (kp/kd=0，因为PID在软件实现)
        DM_Motor_Ctrl(&hfdcan1, &chassis_dm_info[i],
                      out_angle[i], speed_out, 0, 0, DM6220_TOR, 0);
    }
    ```
4. **3508轮毂电机控制 (单环PID)**:
    ```c
    for (uint8_t i = 0; i < 4; i++) {
        chassis_motor_info[i].Data.SET_Current = PID_Calc(&chassis_motor_ctrl[i].Speed_pid,
                                                          chassis_motor_info[i].Data.Velocity,
                                                          out_speed[i]);
    }
    // 发送3508控制电流到FDCAN2
    DJI_Motor_ctrl(chassis_motor_info, &hfdcan2, 0);
    ```

---

## 4. DM6220电机驱动详解 (更新)

> **文件位置**: `User/Devices/DM_Motor/DM_Motor.c/.h`
> **控制模式**: **MIT模式** (Mit_mode) - 软件PID，不再使用位置速度模式

### 4.1 DM电机通信协议
- **CAN ID分配**: 
  - TxID: 发送ID (主控→电机)
  - RxID: 接收ID (电机→主控)
  - MIT模式ID: 0x000 (TxID + MIT_MODE)
- **数据格式** (MIT模式，8字节):
  - 位置: 16位，范围 P_MIN2~P_MAX2
  - 速度: 12位，范围 V_MIN2~V_MAX2
  - KP: 12位
  - KD: 12位
  - 扭矩: 12位，范围 T_MIN2~T_MAX2
- **反馈数据** (8字节):
  - ID: 4位
  - 状态: 4位
  - 位置: 16位 (转换为浮点数，范围P_MIN~P_MAX)
  - 速度: 12位 (转换为浮点数，范围V_MIN~V_MAX)
  - 扭矩: 12位 (转换为浮点数，范围T_MIN~T_MAX)
  - 温度: 2字节 (MOS管温度 + 线圈温度)

### 4.2 关键函数
| 函数 | 功能 |
|:---|:---|
| `Enable_Motor_Mode()` | 使能电机，进入MIT模式 |
| `Disable_Motor_Mode()` | 失能电机 |
| `Save_Motor_Zero()` | 保存当前位置为零点 |
| `DM_Motor_Info_Update()` | 更新电机反馈数据 (位置/速度/扭矩/温度) |
| `DM_Motor_Ctrl()` | 发送MIT模式控制指令 |
| `mit_ctrl2()` | MIT模式控制2 (使用P_MIN2等参数) |

### 4.3 电机反馈数据结构
```c
typedef struct {
    uint16_t id;    // 电机ID
    uint16_t state; // 电机状态
    float pos;      // 位置 (弧度，范围P_MIN~P_MAX)
    float vel;      // 速度 (弧度/秒)
    float tor;      // 扭矩 (Nm)
    float Tmos;     // MOS管温度
    float Tcoil;    // 线圈温度
} DM_Motor_Data_Typedef;
```

---

## 5. FDCAN配置与回调函数 (更新)

> **文件位置**: `User/Bsp/bsp_can.c`

### 5.1 FDCAN分配
| FDCAN | 用途 | FIFO | 接收ID范围 |
|:---|:---|:---|:---|
| FDCAN1 | DM6220转向 + 机械臂DM | FIFO0 | DM电机反馈ID |
| FDCAN2 | M3508轮毂电机 | FIFO1 | 0x201~0x204 (3508反馈) |
| FDCAN3 | 暂时不用 | - | - |

### 5.2 回调函数修改计划
1. **`HAL_FDCAN_RxFifo0Callback` (FDCAN1)**:
   - 添加DM6220反馈数据接收:
     ```c
     case DM_RxID1: case DM_RxID2: case DM_RxID3: case DM_RxID4:
         DM_Motor_Info_Update(&chassis_dm_info[i], g_Can1RxData, RxHeader1.DataLength);
         break;
     ```
   - 机械臂DM电机反馈 (如有需要)

2. **`HAL_FDCAN_RxFifo1Callback` (FDCAN2)**:
   - 添加3508反馈数据接收:
     ```c
     case 0x201: case 0x202: case 0x203: case 0x204:
         uint8_t j = RxHeader2.Identifier - 0x201;
         DJI_Motor_Info_Update(&chassis_motor_info[j], g_Can2RxData, RxHeader2.DataLength);
         break;
     ```
   - 需要添加`extern DJI_Motor_Info_Typedef chassis_motor_info[4];`

3. **过滤器配置**:
   - FDCAN2过滤器需要配置为接收0x201~0x204 (标准ID)

---

## 6. 关键参数速查表 (更新)

| 参数 | 值 | 说明 |
|:---|:---|:---|
| `L_Q_DM6220_Middle_Pos` | 0.0 (弧度) | 左前DM6220机械中位 (需实测) |
| `L_H_DM6220_Middle_Pos` | 0.0 (弧度) | 左后DM6220机械中位 |
| `R_H_DM6220_Middle_Pos` | 0.0 (弧度) | 右后DM6220机械中位 |
| `R_Q_DM6220_Middle_Pos` | 0.0 (弧度) | 右前DM6220机械中位 |
| `Speed_3508_KP` | **待调整** | 3508速度环P |
| `Speed_3508_KI` | **待调整** | 3508速度环I |
| `Speed_3508_KD` | **待调整** | 3508速度环D |
| `Speed_3508_PID_Out_Max` | **待调整** | 3508速度环输出限幅 |
| `Speed_3508_Iout_Max` | **待调整** | 3508速度环积分限幅 |
| `Angle_DM6220_KP` | **待调整** | DM6220角度环P (软件PID) |
| `Angle_DM6220_KI` | **待调整** | DM6220角度环I |
| `Angle_DM6220_KD` | **待调整** | DM6220角度环D |
| `Angle_DM6220_PID_Out_Max` | **待调整** | DM6220角度环输出限幅 |
| `Speed_DM6220_KP` | **待调整** | DM6220速度环P (软件PID) |
| `Speed_DM6220_KI` | **待调整** | DM6220速度环I |
| `Speed_DM6220_KD` | **待调整** | DM6220速度环D |
| `Speed_DM6220_PID_Out_Max` | **待调整** | DM6220速度环输出限幅 |
| `DM6220_KP` | 0 | MIT模式KP (软件PID，设为0) |
| `DM6220_KD` | 0 | MIT模式KD (软件PID，设为0) |
| `DM6220_TOR` | **待调整** | MIT模式扭矩限制 |
| `CHASSIS_OFFSET_X` | 0.18m | 底盘半宽 (中心到左右轮距离) |
| `CHASSIS_OFFSET_Y` | 0.20m | 底盘半长 (中心到前后轮距离) |
| `WHEEL_RADIUS` | 0.065m | 轮子半径 |
| `WHEEL_PERIMETER` | 0.408m | 轮子周长 |
| `wheel_rpm_ratio` | 786.43f / WHEEL_PERIMETER | 线速度转RPM系数 |
| `DM_POS_MIN` | -PI (约-3.14159) | DM电机位置最小值 (弧度) |
| `DM_POS_MAX` | PI (约3.14159) | DM电机位置最大值 (弧度) |
| SBUS通道范围 | -660~660 | 大疆遥控器通道量程 |
| 遥控器死区 | 0.03f | 摇杆中性位死区 (vx/vy) |
| 速度最大值 | 4m/s | 底盘前后/左右最大速度 |
| 角速度最大值 | 4rad/s | 底盘旋转最大角速度 |

---

## 7. 调试与注意事项 (更新)

1. **CAN ID 配置**: 
   - 3508: TxID=0x200，RxID=0x201~0x204（对应4个轮毂电机），通过FDCAN2
   - DM6220: TxID从`AGV_Rotate_DM_Motor`数组获取，RxID对应，通过FDCAN1
   - 机械臂DM: 通过FDCAN1 (与转向DM共用)
2. **DM6220控制模式变更**: 
   - 从`Pos_mode`(位置速度模式，内置PID) 改为 `Mit_mode`(MIT模式，软件PID)
   - 需要在`DM_Motor.c`中修改`AGV_Rotate_DM_Motor`数组的`.Mode`字段
   - `DM_Motor_Ctrl`调用时kp/kd设为0，因为PID在软件中实现
3. **电机方向**: 
   - 3508: 正转对应整车后退，需要在速度计算时乘-1
   - 反转标志: `reverse_flag[i]`根据最短路径判断自动设置
4. **姿态解算延迟**: INS积分在启动3s后才生效
5. **遥控器死区**: 硬件摇杆中立点不精准，vx/vy死区0.03f
6. **PID参数调整**: 
   - 3508速度环PID: 需要调整
   - **DM6220双环PID**: 全新调试，参考旧项目`have_newRemote and AGV`中6020的PID参数思路
   - 角度环输入: 目标角度(弧度) - 反馈角度(弧度)
   - 速度环输入: 角度环输出 - 反馈速度
7. **位置溢出处理**: 
   - DM6220位置范围 ±π，周期2π
   - 使用`loop_f()`和`AGV_chassis_Zero_Check()`处理跨周期
   - `Find_min_Angle()`判断最短路径，设置反转标志
8. **FDCAN分配变更**: 
   - FDCAN1: DM6220转向 + 机械臂DM (需添加反馈接收)
   - FDCAN2: M3508轮毂电机 (从FDCAN3迁移)
   - FDCAN3: 暂时不用
9. **回调函数更新**: 
   - FDCAN1回调: 添加DM6220反馈解析 (`DM_Motor_Info_Update`)
   - FDCAN2回调: 添加3508反馈解析 (`DJI_Motor_Info_Update`)
10. **旧代码参考**: `have_newRemote and AGV\User\APP\AGV_chassis_task.c`中的6020双环PID控制逻辑

---

## 8. PID 算法核心学习笔记

> **文件位置**: `User/Algorithm/PID/pid.c` 与 `pid.h`
> **适用场景**: 舵轮底盘速度/角度控制、机械臂控制等所有闭环控制场景。

### 8.1 PID 函数接口与使用指南

#### 1. `PID_init` - 初始化函数
- **原型**: `void PID_init(PidTypedef *pid, PID_mode_e mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)`
- **参数说明**:
  - `pid`: 指向你要初始化的 PID 结构体实例。
  - `mode`: 选择模式，`PID_POSITION`（位置式）或 `PID_DELTA`（增量式）。
  - `PID[3]`: 浮点数组，依次存放 `{Kp, Ki, Kd}`。
  - `max_out`: **总输出限幅**。
  - `max_iout`: **积分项限幅**。
- **使用场景**: 在系统启动或任务创建后的初始化阶段调用，通常**只调用一次**。

#### 2. `PID_Calc` - 核心计算函数
- **原型**: `fp32 PID_Calc(PidTypedef *pid, fp32 fdb, fp32 ref)`
- **参数说明**:
  - `pid`: 正在使用的 PID 结构体。
  - `fdb` (Feedback): **反馈值**，来自传感器（编码器角度、转速等）。
  - `ref` (Reference): **目标值**，你希望电机达到的状态。
- **返回值**: 计算出的控制输出量（电流值或速度值）。
- **使用场景**: 放在主控制循环（如 5ms 周期任务）中，**每次循环调用一次**。

#### 3. `PID_clear` - 清除/复位函数
- **原型**: `void PID_clear(PidTypedef *pid)`
- **使用场景**: 当系统发生错误复位、急停时调用，用于消除累积的历史误差。

### 8.2 PID 结构体详解 (`PidTypedef`)
```c
typedef struct {
    PID_mode_e pid_mode;    // 控制模式：0=位置式, 1=增量式
    bool Initlized;         // 初始化标志
    fp32 Kp, Ki, Kd;        // 核心三参数
    
    fp32 max_out;           // 最终输出限幅值
    fp32 max_iout;          // 积分项限幅值
    
    fp32 set, fdb;          // set: 目标值, fdb: 反馈值
    fp32 error[3];          // 误差历史: [0]当前, [1]上一次, [2]上上次
    
    fp32 Pout, Iout, Dout;  // 分解输出
    fp32 out;               // 最终输出结果
    
    fp32 Dbuf[3];           // 微分专用缓冲
} PidTypedef;
```

### 8.3 两种 PID 模式对比

| 模式 | 公式逻辑 | 特点 | 本项目应用场景 |
|:---|:---|:---|:---|
| **位置式 (PID_POSITION)** | `Out = Kp*Err + Ki*ΣErr + Kd*(Err-Last_Err)` | 输出是**控制量的绝对值**。 | **DM6220 角度环**<br>**DM6220 速度环**<br>**3508 速度环** |
| **增量式 (PID_DELTA)** | `Out += Kp*(Err-Last) + Ki*Err + Kd*(Err-2*Last+Last2)` | 输出是**控制量的变化量**。 | 暂未使用 |

### 8.4 核心计算流程 (`PID_Calc`)

1. **更新误差 (Error)**:
    ```c
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = ref - fdb;     // 计算当前误差：目标 - 反馈
    ```

2. **计算三项输出 (以位置式为例)**:
    - **P (比例)**: `Pout = Kp * error[0]`
    - **I (积分)**: `Iout += Ki * error[0]`
    - **D (微分)**: `Dout = Kd * (error[0] - error[1])`

3. **限幅保护**:
    ```c
    LimitMax(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
    ```

### 8.5 初学者避坑指南 (重要细节)

1. **初始化状态 (`Initlized` 标志)**:
    - 在 `PID_init` 中，只有 `Initlized != true` 时才会赋值参数。
    - **坑**: 如果想在运行中修改 PID 参数，直接调 `PID_init` 是没用的，必须**手动修改**结构体里的 `Kp/Ki/Kd`。

2. **误差方向 (Ref - Fdb)**:
    - 代码逻辑是 `error = ref - fdb` (目标 - 实际)。
    - 如果发现电机**反转**，不要急着改接线，先试着把 `Kp` 改成负数。

3. **清零操作 (`PID_clear`)**:
    - **注意**: `PID_clear` 会把 `Initlized` 设为 `false`！这意味着清零后，下一次调用 `PID_Calc` 会直接返回 0。
    - **正确做法**: 如果只是想消除积分累积，建议手动清零 `pid->Iout = 0;` 和误差数组。

4. **调试建议**:
    - **先调 P**: I 和 D 设为 0。加大 P 直到电机开始抖动，然后回调到不抖动的 60%-70%。
    - **再加 D**: 如果电机响应慢、有超调，加一点 D。
    - **最后加 I**: 如果电机停不下来（有稳态误差），慢慢加 I。
    - **DM6220双环**: 先调内环(速度环)，再调外环(角度环)。或者先调角度环(把速度环当执行器)，再细调速度环。

### 8.6 `pid.c` 与 `AGV_chassis_task.c` 的实战联系 (更新)

> **核心逻辑**: `AGV_chassis_task.c` 负责**业务逻辑**，`pid.c` 负责**底层算法**。两者通过 **PID 结构体**紧密连接。

#### 1. 纽带：电机控制结构体数组
在 `AGV_chassis_task.c` 中定义了全局数组：
```c
// 3508 轮毂电机 (0-3)
DJI_Motor_Ctrl_Typedef chassis_motor_ctrl[4];
// DM6220 转向电机 (0-3)
DM_Motor_Ctrl_Typedef chassis_dm_ctrl[4];
```
`DM_Motor_Ctrl_Typedef` 内部包含了 `Angle_pid` 和 `Speed_pid` 两个 `PidTypedef` 结构体实例。

#### 2. 初始化阶段：分发参数 (`AGV_classis_Pid_data_Init`)
底盘启动时，将头文件里定义好的宏参数灌入 PID 结构体：
```c
// 遍历 4 个3508
for(uint8_t i = 0; i < 4; i++) {
    PID_init(&chassis_motor_ctrl[i].Speed_pid, PID_POSITION, Spe_3508_PID, ...);
}
// 遍历 4 个DM6220 (新启用)
for(uint8_t i = 0; i < 4; i++) {
    PID_init(&chassis_dm_ctrl[i].Speed_pid, PID_POSITION, Spe_DM6220_PID, ...);
    PID_init(&chassis_dm_ctrl[i].Angle_pid, PID_POSITION, Ang_DM6220_PID, ...);
}
```

#### 3. 控制阶段：串联双环 (`chassis_control`)

- **3508 轮毂电机 (单环速度控制)**:
    ```c
    chassis_motor_info[i].Data.SET_Current = PID_Calc(
        &chassis_motor_ctrl[i].Speed_pid,
        chassis_motor_info[i].Data.Velocity,
        out_speed[i]
    );
    ```

- **DM6220 转向电机 (双环级联控制)**:
    - **外环 (角度环)**: 算出"需要转多快"
      ```c
      float angle_out = PID_Calc(
          &chassis_dm_ctrl[i].Angle_pid,
          chassis_dm_info[i].Data.pos,  // DM电机反馈位置(弧度)
          out_angle[i]
      );
      ```
    - **内环 (速度环)**: 算出"需要给多大扭矩"
      ```c
      float speed_out = PID_Calc(
          &chassis_dm_ctrl[i].Speed_pid,
          chassis_dm_info[i].Data.vel,  // DM电机反馈速度
          angle_out
      );
      ```
    - **发送控制指令 (MIT模式)**:
      ```c
      DM_Motor_Ctrl(&hfdcan1, &chassis_dm_info[i], 
                    out_angle[i], speed_out, 0, 0, DM6220_TOR, 0);
      ```

#### 4. 数据流向总结图
```text
[AGV_chassis_task.c - 业务层]
       |
       +-- 遥控器/运动学解算 --> [out_angle / out_speed] (PID 的 Target)
       |
       +-- CAN 中断接收 ------> [Data.pos / Data.vel] (PID 的 Feedback)
       |
       v
[调用 PID_Calc] ---> [Algorithm/PID/pid.c - 算法层]
       |                   |
       |                   +-- 读取 Kp, Ki, Kd, max_out
       |                   +-- 计算误差 P+I+D
       |                   +-- 执行限幅 (Anti-Windup)
       |                   v
       +-- 返回 [Output] ---> 写入 [DM_Motor_Ctrl / DJI_Motor_ctrl] (发送给电机)
```

---

## 9. AGV_chassis_task.c 函数调用与引用全景图

> **说明**: 本节汇总了 `AGV_chassis_task.c` 文件头部引用的头文件，以及代码中实际调用的函数和访问的全局变量。

### 9.1 操作系统服务 (`freertos.h`, `task.h`)

| 函数/接口 | 代码位置 | 具体作用 |
|:---|:---|:---|
| `vTaskDelay()` | 任务初始化、归零循环 | **绝对延时**。挂起任务指定时间。 |
| `xTaskGetTickCount()` | 主循环开始 | **获取系统节拍**。记录当前系统时间滴答数。 |
| `vTaskDelayUntil()` | 主循环末尾 | **周期性延时**。确保任务严格按照设定的周期（5ms）运行。 |

### 9.2 电机驱动控制 (`DJI_Motor.h`, `DM_Motor.h`)
封装了 CAN 通信的底层细节，向上层应用提供简洁的电机控制接口。

| 函数 | 代码位置 | 具体作用 |
|:---|:---|:---|
| `DJI_Motor_ctrl()` | `chassis_control` (3508) | **组帧发送**。将 4 个3508电机的电流指令打包成 CAN 报文，通过FDCAN2发送。 |
| `DM_Motor_Ctrl()` | `chassis_control` (DM6220) | **DM电机控制**。发送MIT模式指令，通过FDCAN1发送。 |
| `Enable_Motor_Mode()` | 初始化 | **使能DM电机**。让电机进入MIT模式。 |
| `Save_Motor_Zero()` | 归零初始化 | **保存零点**。将当前位置设为机械零点。 |

### 9.3 核心控制算法 (`pid.h`)
提供经典的数字 PID 控制算法。

| 函数 | 代码位置 | 具体作用 |
|:---|:---|:---|
| `PID_init()` | `AGV_classis_Pid_data_Init` | **参数配置**。设置 PID 模式、Kp/Ki/Kd 参数、输出限幅值。 |
| `PID_Calc()` | `chassis_control` | **闭环计算**。输入当前反馈值和目标值，返回计算出的控制量。 |

### 9.4 外部状态读取 (数据依赖)

| 引用头文件 | 访问对象 | 具体作用 |
|:---|:---|:---|
| `New_Remote_Control.h` | 全局变量 `remote_ctrl` | **获取用户意图**。读取 SBUS 遥控器解析后的通道值。 |
| `ins_task.h` | 全局变量 `INS` | **获取姿态信息**。读取 IMU 解算后的欧拉角。 |
| `bsp_can.h` | CAN句柄 | **CAN通信**。提供FDCAN1/2/3的发送接口。 |

### 9.5 调试与辅助 (`bsp_uart.h`, `AGV_chassis_task.h`)

| 头文件 | 作用说明 |
|:---|:---|
| `bsp_uart.h` | 提供串口通信的底层支持，用于调试信息打印。 |
| `AGV_chassis_task.h` | **本模块头文件**。定义了所有的结构体、宏定义以及函数声明。 |

---

## 10. DM6220 与 GM6020 差异对比 (更新)

| 特性 | GM6020 (旧项目) | DM6220 (本项目) |
|:---|:---|:---|
| **通信协议** | DJI标准CAN协议 | DM自定义协议 (MIT模式) |
| **编码器分辨率** | 8192 CPR (0~8191) | 位置范围 ±π (弧度，浮点数) |
| **控制方式** | 直接发送电流值 | MIT模式，软件双环PID |
| **反馈数据** | 编码器值、转速、转矩 | 位置(浮点)、速度(浮点)、转矩(浮点)、温度 |
| **归零方式** | 软件PID归零 | `Save_Motor_Zero()` 保存零点 + 软件PID归零 |
| **CAN ID** | 标准ID (0x205~0x208等) | TxID + MIT_MODE |
| **PID参数** | 针对电流环设计 | 针对位置/速度环设计，软件实现 |

---

> **最后更新**: 2026-05-03
> **维护者**: AGV开发团队
> **注意事项**: 
> 1. DM6220的PID参数需要重新调试，参考旧项目6020的双环PID思路，但参数值需要适配DM6220特性
> 2. 机械中位需要实际测量并修改对应的位置参数(弧度制)
> 3. DM6220使用浮点数表示位置和速度，与DJI电机的编码器值不同，需要注意数据类型转换
> 4. 首次使用时必须执行零点保存操作 (`Save_Motor_Zero`)
> 5. FDCAN分配已变更：FDCAN1(DM6220+机械臂)，FDCAN2(3508)，FDCAN3(暂时不用)
> 6. 不再使用DM6220内置PID，改为软件双环PID + MIT模式控制
