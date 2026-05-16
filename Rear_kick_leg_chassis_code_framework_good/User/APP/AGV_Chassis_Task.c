#include "AGV_Chassis_Task.h"

/*====================================================================
 * 系统状态
 *====================================================================*/
SystemValue systemvalue = Initing;

/*====================================================================
 * PID控制结构体
 *====================================================================*/
DM_Motor_Ctrl_Typedef  DM_6220_pid_set[4];      // DM6220转向电机 控制+PID
DJI_Motor_Ctrl_Typedef DJI_3508_pid_set[4];     // M3508轮毂电机 控制+PID

/* 电机反馈信息 (bsp_can.c CAN中断实时更新) */
extern DJI_Motor_Info_Typedef chassis_3508_motor[4];
extern DM_Motor_Info_Typedef  Chassis_6220_DM_Motor[4];

/* 遥控器 */
extern Remote_Info_Typedef remote_ctrl;

/*====================================================================
 * 全局变量
 *====================================================================*/
float    wheel_angle_last[4];           // 上一次目标角度 (摇杆回中保持用)
volatile int8_t sign_group[4] = {1,1,1,1};  // 方向标志组

extern FDCAN_HandleTypeDef hfdcan1;   //0x01  0x04
extern FDCAN_HandleTypeDef hfdcan2; 	//0x02  0x03
extern FDCAN_HandleTypeDef hfdcan3;

Chassis_Speed chassis_speed;
Chassis_Speed *absolute_chassis_speed = &chassis_speed;

float pid[6] = {0,0,0,0,0,0};   // 保留 (未使用)

/*====================================================================
 * PID 参数 (来自 R1Streering 已验证值)
 *====================================================================*/
fp32 DM_6220_pos_pid[3]          = {20.0f, 0.0f, 0.0f};   // 角度环 Kp=20
fp32 DM_6220_speed_pid[3]        = {0.07f, 0.0f, 0.0f};   // 速度环 Kp=0.07
fp32 DM_6220_Motor1_pos_pid[3]   = {20.0f, 0.0f, 0.0f};   // 电机0角度环
fp32 DM_6220_Motor1_speed_pid[3] = {0.03f,0.0f, 0.0f};   // 电机0速度环
fp32 DJI_3508_speed_pid[3]       = {13.0f, 0.03f,0.0f};   // 3508速度环

/*====================================================================
 * 角度限幅 / 过零处理
 *====================================================================*/
void loop_f(float *angle, float max)
{
    if (*angle > max / 2)       *angle -= max;
    else if (*angle < -max / 2) *angle += max;
}

void AGV_chassis_Zero_Check(float Tar_Angle, float *Acl_Angle, float max)
{
    if (Tar_Angle - *Acl_Angle > max / 2)
        *Acl_Angle += max;
    else if (Tar_Angle - *Acl_Angle < -max / 2)
        *Acl_Angle -= max;
}

/*====================================================================
 * PID 初始化 (循环式, 一次调用即生效)
 *====================================================================*/
void Motor_pid_init(void)
{
    PID_mode_e mode = PID_POSITION;

    for (uint8_t i = 0; i < 4; i++)
    {
        fp32 *pos_pid   = (i == 0) ? DM_6220_Motor1_pos_pid   : DM_6220_pos_pid;
        fp32 *speed_pid = (i == 0) ? DM_6220_Motor1_speed_pid : DM_6220_speed_pid;

        PID_init(&DM_6220_pid_set[i].Angle_pid, mode, pos_pid,   1000.0f, 500.0f);
        PID_init(&DM_6220_pid_set[i].Speed_pid, mode, speed_pid, 10.0f,   500.0f);
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        PID_init(&DJI_3508_pid_set[i].Speed_pid, mode, DJI_3508_speed_pid, 10000.0f, 5000.0f);
    }
}

/*====================================================================
 * DM6220 电机使能 (MIT模式)
 *====================================================================*/
void Chassis_DM_Motor_Enable(void)
{
    // 电机索引: 0=左前, 1=右前, 2=右后, 3=左后
    uint8_t dm_id[4]                = {0x01,   0x02,   0x03,   0x04};
    FDCAN_HandleTypeDef *hcan[4]    = {&hfdcan1, &hfdcan2, &hfdcan2, &hfdcan1};

    for (uint8_t i = 0; i < 4; i++)
    {
        Enable_Motor_Mode(hcan[i], dm_id[i], Mit_mode, 1);
    }
}

/*====================================================================
 * 归零初始化 (目标角度=0, 双环PID)
 *====================================================================*/
void AGV_Chassis_Init(void)
{
    bool return_flag = true;
    uint16_t cnt = 0;
    float tar_angle_temp = 0.0f;

    while (return_flag)
    {
        cnt++;
        if (cnt > 1500)
            return_flag = false;

        float tor[4];

        for (uint8_t i = 0; i < 4; i++)
        {
            float angle_out = PID_Calc(&DM_6220_pid_set[i].Angle_pid,
                                       Chassis_6220_DM_Motor[i].Data.pos,
                                       tar_angle_temp, 0);
            tor[i] = PID_Calc(&DM_6220_pid_set[i].Speed_pid,
                              Chassis_6220_DM_Motor[i].Data.vel,
                              angle_out, 1);
        }

        DM_Motor_Four_Ctrl(tor[0], tor[1], tor[2], tor[3]);
    }
}

/*====================================================================
 * DM6220 扭矩批量发送
 * FDCAN1: 电机0(左前), 电机3(左后)
 * FDCAN2: 电机1(右前), 电机2(右后)
 *====================================================================*/
void DM_Motor_Four_Ctrl(fp32 tor1, fp32 tor2, fp32 tor3, fp32 tor4)
{
    mit_ctrl2(&hfdcan1, Chassis_6220_DM_Motor[0].ID_Set.TxIdentifier, 0,0,0,0, tor1, 0);
    mit_ctrl2(&hfdcan2, Chassis_6220_DM_Motor[1].ID_Set.TxIdentifier, 0,0,0,0, tor2, 0);
    mit_ctrl2(&hfdcan2, Chassis_6220_DM_Motor[2].ID_Set.TxIdentifier, 0,0,0,0, tor3, 0);
    mit_ctrl2(&hfdcan1, Chassis_6220_DM_Motor[3].ID_Set.TxIdentifier, 0,0,0,0, tor4, 0);
}

/*====================================================================
 * 遥控器 → 底盘速度映射
 *====================================================================*/
void RemoteControlChassis(void)
{
    // 大疆SBUS: ch[-660~660] → 速度/角速度
    absolute_chassis_speed->vx = (fp32)remote_ctrl.rc.ch[2] / 165.0f;   // ±4m/s
    absolute_chassis_speed->vy = -(fp32)remote_ctrl.rc.ch[3] / 165.0f;   // ±4m/s
    absolute_chassis_speed->vw =-(fp32)remote_ctrl.rc.ch[0] / 165.0f;   // ±4rad/s, 取反
}

/*====================================================================
 * wheel_angle_last 初始化
 *====================================================================*/
void Wheel_Angle_Last_Init(void)
{
    for (uint8_t i = 0; i < 4; i++)
        wheel_angle_last[i] = 0.0f;
}

/*====================================================================
 * 存储数组 (运动学解算输出 → PID输入)
 *====================================================================*/
fp32    chassis_6220_setangle[4];   // DM6220 目标角度
int16_t chassis_3508_setspeed[4];   // M3508  目标转速

/*====================================================================
 * 坐标变换 (云台跟随预留, 当前 angle=0 退化)
 *====================================================================*/
void Absolute_Cal(Chassis_Speed *absolute_speed, fp32 angle)
{
    fp32 angle_hd = -angle;
    Chassis_Speed temp_speed;

    temp_speed.vw = absolute_speed->vw;
    temp_speed.vx = absolute_speed->vx * cos(angle_hd) - absolute_speed->vy * sin(angle_hd);
    temp_speed.vy = absolute_speed->vx * sin(angle_hd) + absolute_speed->vy * cos(angle_hd);

    AGV_angle_calc(&temp_speed, chassis_6220_setangle);
    AGV_speed_calc(&temp_speed, chassis_3508_setspeed);
}

/*====================================================================
 * 最短路径角度差 (保留当前工程逻辑)
 * 返回角度差值, fabs() > Pi/2 表示需要翻转
 *====================================================================*/
fp32 Find_min_Angle(int16_t angle1, fp32 angle2)
{
    fp32 err = (fp32)angle1 - angle2;
    if (fabs(err) > Pi)
        err = 2*Pi - fabs(err);
    return err;
}

/*====================================================================
 * M3508 目标转速解算
 *====================================================================*/
float x;
void AGV_speed_calc(Chassis_Speed *speed, int16_t *out_speed)
{
    fp32 wheel_rpm_ratio = CHASSIS_DECELE_RATIO / WHEEL_PERIMETER * 60;

    // 合速度死区
    x = sqrt(pow(speed->vy,2) + pow(speed->vx,2));
    if (x < 0.1f)
    {
        speed->vy = 0.f;
        speed->vx = 0.f;
    }

    int16_t wheel_speed[4];
    wheel_speed[0] = sqrt(pow(speed->vy - speed->vw * CHASSIS_OFFSET_X, 2)
                        + pow(speed->vx - speed->vw * CHASSIS_OFFSET_Y, 2)) * wheel_rpm_ratio;
    wheel_speed[1] = sqrt(pow(speed->vy - speed->vw * CHASSIS_OFFSET_X, 2)
                        + pow(speed->vx + speed->vw * CHASSIS_OFFSET_Y, 2)) * wheel_rpm_ratio;
    wheel_speed[2] = sqrt(pow(speed->vy + speed->vw * CHASSIS_OFFSET_X, 2)
                        + pow(speed->vx + speed->vw * CHASSIS_OFFSET_Y, 2)) * wheel_rpm_ratio;
    wheel_speed[3] = sqrt(pow(speed->vy + speed->vw * CHASSIS_OFFSET_X, 2)
                        + pow(speed->vx - speed->vw * CHASSIS_OFFSET_Y, 2)) * wheel_rpm_ratio;

    for (int i = 0; i < 4; i++)
        out_speed[i] = (-1) * sign_group[i] * wheel_speed[i];
}

/*====================================================================
 * DM6220 目标角度解算 (保留当前工程完整逻辑)
 *====================================================================*/
fp32 wheel_angle[4];
void AGV_angle_calc(Chassis_Speed *speed, fp32 *out_angle)
{
    float angle_temp;
    fp64 atan_angle[4];

    if (!(speed->vx == 0 && speed->vy == 0 && speed->vw == 0))
    {
        atan_angle[0] = atan2((speed->vx - speed->vw * CHASSIS_OFFSET_Y),
                              (speed->vy - speed->vw * CHASSIS_OFFSET_X));
        atan_angle[1] = atan2((speed->vx - speed->vw * CHASSIS_OFFSET_Y),
                              (speed->vy + speed->vw * CHASSIS_OFFSET_X));
        atan_angle[2] = atan2((speed->vx + speed->vw * CHASSIS_OFFSET_Y),
                              (speed->vy + speed->vw * CHASSIS_OFFSET_X));
        atan_angle[3] = atan2((speed->vx + speed->vw * CHASSIS_OFFSET_Y),
                              (speed->vy - speed->vw * CHASSIS_OFFSET_X));
    }

    for (int i = 0; i < 4; i++)
        wheel_angle[i] = (fp32)atan_angle[i];

    // 首次回环 6.28 ≈ 2π
    for (int i = 0; i < 4; i++)
        loop_f(&wheel_angle[i], 6.28f);

    // 翻转检测: 仅检查电机0, 统一翻转4轮
    angle_temp = Chassis_6220_DM_Motor[0].Data.pos;
    loop_f(&angle_temp, 2*Pi);

    if (fabs(Find_min_Angle((int16_t)angle_temp, wheel_angle[0])) > (Pi/2))
    {
        for (int i = 0; i < 4; i++)
            wheel_angle[i] += Pi;
        for (int i = 0; i < 4; i++)
            sign_group[i] = -1;
    }
    else
    {
        for (int i = 0; i < 4; i++)
            sign_group[i] = 1;
    }

    // 二次回环 2π
    for (int i = 0; i < 4; i++)
        loop_f(&wheel_angle[i], 2*Pi);

    // 摇杆回中保持, 否则更新
    if (speed->vx == 0 && speed->vy == 0 && speed->vw == 0)
    {
        for (int i = 0; i < 4; i++)
            out_angle[i] = wheel_angle_last[i];
    }
    else
    {
        for (int i = 0; i < 4; i++)
            out_angle[i] = wheel_angle[i];
    }
}

/*====================================================================
 * 设定目标值
 *====================================================================*/
void AGV_Set_Motor_Speed(int16_t *out_speed, DJI_Motor_Ctrl_Typedef *Motor)
{
    for (int i = 0; i < 4; i++)
        Motor[i].Speed_set.ref = out_speed[i];
}

void AGV_Set_Motor_Angle(fp32 *out_angle, DM_Motor_Ctrl_Typedef *Motor)
{
    for (int i = 0; i < 4; i++)
        Motor[i].Angle_set.ref = out_angle[i];
}

/*====================================================================
 * 核心控制: 运动学 → PID → CAN  (5ms执行)
 *====================================================================*/
float tor11, tor22, tor33, tor44;
void CHASSIS_Single_Loop_Out(void)
{
    Absolute_Cal(absolute_chassis_speed, 0);

    AGV_Set_Motor_Speed(chassis_3508_setspeed, DJI_3508_pid_set);
    AGV_Set_Motor_Angle(chassis_6220_setangle,  DM_6220_pid_set);

    // ---- M3508 速度环 PID ----
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_3508_motor[i].Data.SET_Current =
            (int16_t)PID_Calc(&DJI_3508_pid_set[i].Speed_pid,
                              chassis_3508_motor[i].Data.Velocity,
                              DJI_3508_pid_set[i].Speed_set.ref, 1);
    }

    // ---- DM6220 双环 PID ----
    float tor[4];
    for (uint8_t i = 0; i < 4; i++)
    {
        float angle_out = PID_Calc(&DM_6220_pid_set[i].Angle_pid,
                                   Chassis_6220_DM_Motor[i].Data.pos,
                                   DM_6220_pid_set[i].Angle_set.ref, 0);
        tor[i] = PID_Calc(&DM_6220_pid_set[i].Speed_pid,
                           Chassis_6220_DM_Motor[i].Data.vel,
                           angle_out, 1);
    }
    tor11 = tor[0]; tor22 = tor[1]; tor33 = tor[2]; tor44 = tor[3];

    // ---- CAN 发送 ----
    DM_Motor_Four_Ctrl(tor11, tor22, tor33, tor44);
    DJI_Motor_ctrl(chassis_3508_motor, &hfdcan1, 1);
    DJI_Motor_ctrl(chassis_3508_motor, &hfdcan2, 1);
}

/*====================================================================
 * FreeRTOS 任务入口 (5ms 周期)
 *====================================================================*/
void AGV_Chassis_Task(void)
{
    Motor_pid_init();
    Chassis_DM_Motor_Enable();
    Wheel_Angle_Last_Init();

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        if (systemvalue == Initing)
        {
            AGV_Chassis_Init();
            Wheel_Angle_Last_Init();
            systemvalue = Running;
        }

        RemoteControlChassis();
        CHASSIS_Single_Loop_Out();

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}
