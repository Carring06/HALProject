#include "AGV_chassis_task.h"


// 开始标志位,初始化为Initing
SystemValue systemvalue = Initing;

// 3508轮毂电机信息结构体 (0-3: 左前, 左后, 右后, 右前)
DJI_Motor_Info_Typedef chassis_motor_info[4];
// 3508轮毂电机控制结构体
DJI_Motor_Ctrl_Typedef chassis_motor_ctrl[4];

// DM6220转向电机信息结构体 (0-3: 左前, 左后, 右后, 右前)
DM_Motor_Info_Typedef chassis_dm_info[4];
// DM6220转向电机控制结构体
DM_Motor_Ctrl_Typedef chassis_dm_ctrl[4];

extern DM_Motor_Info_Typedef AGV_Rotate_DM_Motor[4];

// 上一次的目标角度
float wheel_angle_last[4];
// 3508反转标志
volatile int8_t reverse_flag[4] = {1, 1, 1, 1};

extern FDCAN_HandleTypeDef hfdcan1; //  DM6220(0x01,0x03) + 3508(0x201,0x203) | 左侧舵轮（索引0,2） |
extern FDCAN_HandleTypeDef hfdcan2; //  DM6220(0x02,0x04) + 3508(0x202,0x204) | 右侧舵轮（索引1,3） |
extern FDCAN_HandleTypeDef hfdcan3; // 暂时不用

AGV_chassis_speed_Typedef chassis_speed;
AGV_gimbal_ctrl_Typedef gimbal_ctrl;

extern INS_t INS;

void AGV_chassis_task(void)
{
    // 初始化3508电机发送标识符ID    初始化有待改进
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_motor_info[i].Motor_Type = DJI_M3508;
        chassis_motor_info[i].ID_Set.TxIdentifier = 0x200;
        chassis_motor_info[i].ID_Set.RxIdentifier = 0x200 + i + 1;
    }

    // 初始化DM6220电机（ID待后续填写，先设为0x00）
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_dm_info[i] = AGV_Rotate_DM_Motor[i];
        /*
            chassis_dm_info[i].Mode = AGV_Rotate_DM_Motor[i].Mode;
            chassis_dm_info[i].Motor_Type = AGV_Rotate_DM_Motor[i].Motor_Type;
            chassis_dm_info[i].ID_Set.TxIdentifier = AGV_Rotate_DM_Motor[i].ID_Set.TxIdentifier;
            chassis_dm_info[i].ID_Set.RxIdentifier = AGV_Rotate_DM_Motor[i].ID_Set.RxIdentifier;
        */
    }

    // 初始化电机pid数据
    AGV_classis_Pid_data_Init();

    // 初始化DM6220电机
    DM6220_Init();

    // 等待1s
    vTaskDelay(pdMS_TO_TICKS(1000));

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 5;
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        if (systemvalue == Initing)
        {
            AGV_chassis_Init();
            systemvalue = Running;
        }

        RemoteControl();
        chassis_control();

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// 给电机初始化pid参数
void AGV_classis_Pid_data_Init(void)
{
    float Spe_3508_PID[3] = {Speed_3508_KP, Speed_3508_KI, Speed_3508_KD};
    float Spe_DM6220_PID[3] = {Speed_DM6220_KP, Speed_DM6220_KI, Speed_DM6220_KD};
    float Ang_DM6220_PID[3] = {Angle_DM6220_KP, Angle_DM6220_KI, Angle_DM6220_KD};

    // 初始化3508轮毂电机（速度环）
    for (uint8_t i = 0; i < 4; i++)
    {
        PID_init(&chassis_motor_ctrl[i].Speed_pid, (PID_mode_e)0, Spe_3508_PID,
                 Speed_3508_PID_Out_Max, Speed_3508_Iout_Max);
    }

    // 初始化DM6220转向电机（双环：角度环+速度环）
    for (uint8_t i = 0; i < 4; i++)
    {
        PID_init(&chassis_dm_ctrl[i].Speed_pid, (PID_mode_e)0, Spe_DM6220_PID,
                 Speed_DM6220_PID_Out_Max, Speed_DM6220_Iout_Max);
        PID_init(&chassis_dm_ctrl[i].Angle_pid, (PID_mode_e)0, Ang_DM6220_PID,
                 Angle_DM6220_PID_Out_Max, Angle_DM6220_Iout_Max);
    }
}

// DM6220电机初始化（使能）
void DM6220_Init(void)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        // 根据索引选择FDCAN：索引1,2→FDCAN1，索引0,3→FDCAN2
        FDCAN_HandleTypeDef *hfdcan_dm = (i == 1 || i == 2) ? &hfdcan1 : &hfdcan2;
        chassis_dm_info[i].Motor_state = Enable_Motor_Mode(hfdcan_dm,
                                                           chassis_dm_info[i].ID_Set.TxIdentifier,
                                                           chassis_dm_info[i].Mode,
                                                           10);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 底盘初始化（DM6220归零到机械中位，双环PID控制）
void AGV_chassis_Init(void)
{
    bool return_flag = true;
    uint16_t cnt = 0;
    float Tar_Temp[4];
    float Act_Temp = 0;
    float Out_temp = 0;

    // 初始化目标角度（机械中位，DM6220范围±12.5）
    Tar_Temp[0] = L_Q_DM6220_Middle_Pos;
    Tar_Temp[1] = L_H_DM6220_Middle_Pos;
    Tar_Temp[2] = R_H_DM6220_Middle_Pos;
    Tar_Temp[3] = R_Q_DM6220_Middle_Pos;

    // 初始化上一次的目标角度
    for (int i = 0; i < 4; i++)
    {
        wheel_angle_last[i] = Tar_Temp[i];
    }

    while (return_flag)
    {
        cnt++;
        if (cnt >= 1500)
        {
            return_flag = false; // 退出初始化
        }

        // 每5ms执行一次双环PID（与chassis_control节奏一致）
        if (cnt % 5 == 0)
        {
            // DM6220电机归位（双环PID控制）
            for (uint8_t i = 0; i < 4; i++)
            {
                // 角度环（外环）：当前位置 → 目标速度
                Act_Temp = chassis_dm_info[i].Data.pos; // DM6220反馈位置（±12.5）
                AGV_chassis_Zero_Check(Tar_Temp[i], &Act_Temp, 25.0f); // 过零处理
                Out_temp = PID_Calc(&chassis_dm_ctrl[i].Angle_pid, Act_Temp, Tar_Temp[i]);

                // 速度环（内环）：角度环输出 → 扭矩
                Act_Temp = chassis_dm_info[i].Data.vel; // DM6220反馈速度
                float tor_output = PID_Calc(&chassis_dm_ctrl[i].Speed_pid, Act_Temp, Out_temp);

								// 根据索引选择FDCAN：索引1,2→FDCAN1，索引0,3→FDCAN2
								FDCAN_HandleTypeDef *hfdcan_dm = (i == 1 || i == 2) ? &hfdcan1 : &hfdcan2;
                DM_Motor_Ctrl(hfdcan_dm, &chassis_dm_info[i], Tar_Temp[i], Out_temp,
                             0, 0, tor_output, 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // 1ms延时
    }
}

// 过零处理函数（用于DM6220，位置范围±3.14）
void AGV_chassis_Zero_Check(float Tar_Angle, float *Acl_Angle, float max)
{
    if (Tar_Angle - *Acl_Angle > max / 2)
    {
        *Acl_Angle += max;
    }
    else if (Tar_Angle - *Acl_Angle < -max / 2)
    {
        *Acl_Angle -= max;
    }
}

// 映射遥控器数据到底盘控制
void RemoteControl(void)
{
    chassis_speed.vx = (float)remote_ctrl.rc.ch[3] * 4 / 660.0f; // 4m/s  //大疆遥控是-660~660，这里取值范围为-660~660，实际上是-4m/s~4m/s
    chassis_speed.vy = (float)remote_ctrl.rc.ch[2] * 4 / 660.0f;
    chassis_speed.vw = (float)remote_ctrl.rc.ch[1] * 4 / 660.0f; //(float)remote_ctrl.rc.ch[0] / 165.0f;//4rad/s 线速度为1.08m/s

    if (fabs(chassis_speed.vx) < 0.03f)
    {
        chassis_speed.vx = 0;
    }
    if (fabs(chassis_speed.vy) < 0.03f)
    {
        chassis_speed.vy = 0;
    }

    gimbal_ctrl.vyaw = (float)remote_ctrl.rc.ch[0] * 360 / 660.0f;//暂时不考虑
}

// 数据解算（角度计算，输出DM6220目标位置，±12.5范围）
void AGV_angle_calc(AGV_chassis_speed_Typedef *speed, float *out_angle)
{
    float atan_angle[4], wheel_angle[4];
    float rad_to_dm = 12.5f / PI; // 弧度转DM6220比例因子

    if (!(speed->vx == 0 && speed->vy == 0 && speed->vw == 0))
    {
        // 左前：atan2返回弧度
        atan_angle[0] = atan2((speed->vy + speed->vw * CHASSIS_OFFSET_X),
                              (speed->vx + speed->vw * CHASSIS_OFFSET_Y));
        // 左后
        atan_angle[1] = atan2((speed->vy + speed->vw * CHASSIS_OFFSET_X),
                              (speed->vx - speed->vw * CHASSIS_OFFSET_Y));
        // 右后
        atan_angle[2] = atan2((speed->vy - speed->vw * CHASSIS_OFFSET_X),
                              (speed->vx - speed->vw * CHASSIS_OFFSET_Y));
        // 右前
        atan_angle[3] = atan2((speed->vy - speed->vw * CHASSIS_OFFSET_X),
                              (speed->vx + speed->vw * CHASSIS_OFFSET_Y));

        // 弧度转DM6220位置（±12.5范围）
        for (int i = 0; i < 4; i++)
        {
            atan_angle[i] *= rad_to_dm;
        }
    }

    // 角度转换为DM6220位置（±12.5范围，机械中位为基准）
    wheel_angle[0] = L_Q_DM6220_Middle_Pos + atan_angle[0];
    wheel_angle[1] = L_H_DM6220_Middle_Pos + atan_angle[1];
    wheel_angle[2] = R_H_DM6220_Middle_Pos + atan_angle[2];
    wheel_angle[3] = R_Q_DM6220_Middle_Pos + atan_angle[3];

    // 循环限幅（DM6220位置范围±12.5，周期25.0f）
    for (int i = 0; i < 4; i++)
    {
        loop_f(&wheel_angle[i], 25.0f);
    }

    // 输出
    if (speed->vx == 0 && speed->vy == 0 && speed->vw == 0)
    {
        for (int i = 0; i < 4; i++)
        {
            out_angle[i] = wheel_angle_last[i];
        }
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            if (Find_min_Angle(&wheel_angle[i], chassis_dm_info[i].Data.pos) == -1)
            {
                reverse_flag[i] = -1;
            }
            else
            {
                reverse_flag[i] = 1;
            }
            loop_f(&wheel_angle[i], 25.0f);
        }

        for (int i = 0; i < 4; i++)
        {
            out_angle[i] = wheel_angle[i];
            wheel_angle_last[i] = wheel_angle[i];
        }
    }
}

void loop_f(float *angle, float max)
{
    if (*angle > max / 2)
    {
        *angle -= max;
    }
    else if (*angle < -max / 2)
    {
        *angle += max;
    }
}

void AGV_speed_calc(AGV_chassis_speed_Typedef *speed, int16_t *out_speed)
{
    float wheel_rpm_ratio;
    wheel_rpm_ratio = 786.43f / WHEEL_PERIMETER;

    float wheel_speed[4];

    wheel_speed[0] = sqrt(pow(speed->vx + speed->vw * CHASSIS_OFFSET_Y, 2) +
                          pow(speed->vy + speed->vw * CHASSIS_OFFSET_X, 2)) *
                     wheel_rpm_ratio;
    wheel_speed[1] = sqrt(pow(speed->vx - speed->vw * CHASSIS_OFFSET_Y, 2) +
                          pow(speed->vy + speed->vw * CHASSIS_OFFSET_X, 2)) *
                     wheel_rpm_ratio;
    wheel_speed[2] = sqrt(pow(speed->vx - speed->vw * CHASSIS_OFFSET_Y, 2) +
                          pow(speed->vy - speed->vw * CHASSIS_OFFSET_X, 2)) *
                     wheel_rpm_ratio;
    wheel_speed[3] = sqrt(pow(speed->vx + speed->vw * CHASSIS_OFFSET_Y, 2) +
                          pow(speed->vy - speed->vw * CHASSIS_OFFSET_X, 2)) *
                     wheel_rpm_ratio;

    for (int i = 0; i < 4; i++)
    {
        // 乘-1是因为3508轮子的正转是使整车向后
        out_speed[i] = reverse_flag[i] * (int16_t)wheel_speed[i] * -1;
    }
}

void chassis_control(void)
{
    float out_angle[4];
    int16_t out_speed[4];
    float Act_Temp = 0;
    float Out_temp = 0;

    AGV_angle_calc(&chassis_speed, out_angle);
    AGV_speed_calc(&chassis_speed, out_speed);

    // DM6220转向电机控制（双环PID + MIT模式）
    for (uint8_t i = 0; i < 4; i++)
    {
        // 角度环PID（外环）：目标角度 → 输出目标速度
        Act_Temp = chassis_dm_info[i].Data.pos; // DM6220反馈位置（±12.5范围）
        AGV_chassis_Zero_Check(out_angle[i], &Act_Temp, 25.0f); // 过零处理
        Out_temp = PID_Calc(&chassis_dm_ctrl[i].Angle_pid, Act_Temp, out_angle[i]);

        // 速度环PID（内环）：角度环输出 → 输出扭矩
        Act_Temp = chassis_dm_info[i].Data.vel; // DM6220反馈速度
        float tor_output = PID_Calc(&chassis_dm_ctrl[i].Speed_pid, Act_Temp, Out_temp);

        // 根据索引选择FDCAN：索引0,2→FDCAN1，索引1,3→FDCAN2
        FDCAN_HandleTypeDef *hfdcan_dm = (i == 0 || i == 2) ? &hfdcan1 : &hfdcan2;
        DM_Motor_Ctrl(hfdcan_dm, &chassis_dm_info[i], out_angle[i], Out_temp,
                     0, 0, tor_output, 0);
    }

    // 3508轮毂电机控制（速度环）
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_motor_info[i].Data.SET_Current = PID_Calc(&chassis_motor_ctrl[i].Speed_pid,
                                                          chassis_motor_info[i].Data.Velocity,
                                                          out_speed[i]);
    }

    // 3508按FDCAN分组发送（根据新分配）
    // FDCAN1：索引1(0x202),2(0x203) -> TxID=0x200
    // FDCAN2：索引0(0x201),3(0x204) -> TxID=0x200
    uint8_t data1[8] = {0}; // FDCAN1: 电机1和2
    uint8_t data2[8] = {0}; // FDCAN2: 电机0和3

    data1[2] = (uint8_t)((chassis_motor_info[1].Data.SET_Current >> 8) & 0xFF);
    data1[3] = (uint8_t)(chassis_motor_info[1].Data.SET_Current & 0xFF);
    data1[4] = (uint8_t)((chassis_motor_info[2].Data.SET_Current >> 8) & 0xFF);
    data1[5] = (uint8_t)(chassis_motor_info[2].Data.SET_Current & 0xFF);
    canx_send_data(&hfdcan1, 0x200, data1, 8);

    data2[0] = (uint8_t)((chassis_motor_info[0].Data.SET_Current >> 8) & 0xFF);
    data2[1] = (uint8_t)(chassis_motor_info[0].Data.SET_Current & 0xFF);
    data2[6] = (uint8_t)((chassis_motor_info[3].Data.SET_Current >> 8) & 0xFF);
    data2[7] = (uint8_t)(chassis_motor_info[3].Data.SET_Current & 0xFF);
    canx_send_data(&hfdcan2, 0x200, data2, 8);
}

int8_t Find_min_Angle(float *tar_angle, float act_angle)
{
    float diff = *tar_angle - act_angle;
    int8_t flag = 1;
    float max = 2*PI; // 弧度制周期2π

    // 调整目标角度到当前位置的短路径（±π范围内）
    if (diff > max / 2)  // 差值超过π，目标减一个周期
    {
        *tar_angle -= max;
        flag = -1;
    }
    else if (diff < -max / 2)  // 差值小于-π，目标加一个周期
    {
        *tar_angle += max;
        flag = -1;
    }
    return flag;
}

/**
  * @brief  舵轮转向反转检查（适配DM6220 0~360°范围）
  * @param  tar_pos: 4个转向DM的目标位置（单位：°，0~360，函数内会调整为短路径）
  * @param  cur_pos: 4个转向DM的当前位置（单位：°，0~360）
  * @note   当目标与当前位置差值超过180°时，调整目标为短路径，并设置reverse_flag反转轮毂方向
  */
// void AGV_Reverse_Check(float *tar_pos, float *cur_pos)
// {
//     for (uint8_t i = 0; i < 4; i++)
//     {
//         float diff = tar_pos[i] - cur_pos[i];
//         // 处理0~360°循环，判断是否需要走短路径
//         if (diff > 180.0f)
//         {
//             // 目标在当前顺时针180°以外，调整为逆时针短路径（减180°）
//             tar_pos[i] -= 180.0f;
//             if (tar_pos[i] < 0) tar_pos[i] += 360.0f;
//             reverse_flag[i] = -1;
//         }
//         else if (diff < -180.0f)
//         {
//             // 目标在当前逆时针180°以外，调整为顺时针短路径（加180°）
//             tar_pos[i] += 180.0f;
//             if (tar_pos[i] >= 360.0f) tar_pos[i] -= 360.0f;
//             reverse_flag[i] = -1;
//         }
//         else
//         {
//             reverse_flag[i] = 1;
//         }
//     }
// }


