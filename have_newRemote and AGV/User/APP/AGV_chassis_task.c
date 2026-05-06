#include "AGV_chassis_task.h"
#include "freertos.h"
#include "task.h"
#include "DJI_Motor.h"
#include "New_Remote_Control.h"
#include "bsp_uart.h"
#include "ins_task.h"

//计数器
uint32_t chassis_count = 0;
int16_t debug_Target;

//开始标志位,初始化为Initing 
SystemValue systemvalue = Initing;

//底盘电机信息结构体
DJI_Motor_Info_Typedef chassis_motor_info[8];//0-3为3508，4-7为6020
//底盘电机控制结构体
DJI_Motor_Ctrl_Typedef chassis_motor_ctrl[8];

//云台电机信息结构体(3508)
DJI_Motor_Info_Typedef gimbal_motor_info[2];//pitch轴电机为0，yaw轴电机为1
//云台电机控制结构体
DJI_Motor_Ctrl_Typedef gimbal_motor_ctrl[2];

//上一次的目标角度
float wheel_angle_last[4];
//3508反转标志
volatile int8_t reverse_flag[4] = {1,1,1,1};

extern FDCAN_HandleTypeDef hfdcan1;//6020
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;//3508

AGV_chassis_speed_Typedef chassis_speed;
AGV_gimbal_ctrl_Typedef gimbal_ctrl;

extern INS_t INS;						// 姿态信息结构体变量

void AGV_chassis_task(void)
{
    //初始化底盘电机信息结构体的电机类型和发送标识符ID
    for(uint8_t i = 0;i<4;i++)
    {
        // chassis_motor_info[i].Motor_Type = 1;
        // chassis_motor_info[i + 4].Motor_Type = 0;
        chassis_motor_info[i].ID_Set.TxIdentifier = 0x200;//初始化3508发送标识符ID
        chassis_motor_info[i + 4].ID_Set.TxIdentifier = 0x1FF;//初始化6020发送标识符ID，电压控制
    }
    //初始化电机pid数据
    AGV_classis_Pid_data_Init();

    //初始化云台电机(pid,结构体信息等)
    AGV_Gimbal_Init();

    //等待1s
    vTaskDelay(pdMS_TO_TICKS(1000));


    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 5;
    xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        if(systemvalue==Initing)
        {
            //初始化底盘
            AGV_chassis_Init();//花费1.5s
            systemvalue = Running;
        }

        RemoteControl();
        gimbal_control();
        chassis_control();

        // printf("%.2f, %.2f, %.2f\n",INS.Roll,INS.Pitch,INS.Yaw);
        
        chassis_count++;
        if(chassis_count >= 500)
        {
            chassis_count = 0;
            if(debug_Target == 0)
            debug_Target = 1000;
            else
            debug_Target = 4000;
        }
        //5ms控制一次电机
        vTaskDelayUntil(&xLastWakeTime,xFrequency);
    }  
}

//给8个电机初始化pid参数
void AGV_classis_Pid_data_Init(void)
{
    float Spe_3508_PID[3] = {Speed_3508_KP,Speed_3508_KI,Speed_3508_KD};
    float Spe_6020_PID[3] = {Speed_6020_KP,Speed_6020_KI,Speed_6020_KD};
    float Ang_6020_PID[3] = {Angle_6020_KP,Angle_6020_KI,Angle_6020_KD};

    for(uint8_t i = 0;i<8;i++)
    {
        if(i<4)//数组0-3为3508
        {
            PID_init(&chassis_motor_ctrl[i].Speed_pid,(PID_mode_e)0,Spe_3508_PID,Speed_3508_PID_Out_Max,Speed_3508_Iout_Max);
        }else//6020
        {
            PID_init(&chassis_motor_ctrl[i].Speed_pid,(PID_mode_e)0,Spe_6020_PID,Speed_6020_PID_Out_Max,Speed_6020_Iout_Max);
            PID_init(&chassis_motor_ctrl[i].Angle_pid,(PID_mode_e)0,Ang_6020_PID,Angle_6020_PID_Out_Max,Angle_6020_Iout_Max);
        }
    }  
}

void AGV_chassis_Init(void)
{
    bool return_flag = true;
    uint16_t cnt = 0;
    int16_t Tar_Temp[4];
    int16_t Act_Temp = 0;
    int16_t Out_temp = 0;

    //初始化目标角度
    Tar_Temp[0] = L_Q_6020_Middle_ECD;
    Tar_Temp[1] = L_H_6020_Middle_ECD;
    Tar_Temp[2] = R_H_6020_Middle_ECD;
    Tar_Temp[3] = R_Q_6020_Middle_ECD;

    //初始化上一次的目标角度
    wheel_angle_last[0] = L_Q_6020_Middle_ECD;
    wheel_angle_last[1] = L_H_6020_Middle_ECD;
    wheel_angle_last[2] = R_H_6020_Middle_ECD;
    wheel_angle_last[3] = R_Q_6020_Middle_ECD;

    while(return_flag)
    {
        cnt++;
        if(cnt>=1500)
        {
            return_flag = false;//退出初始化
        }

        if(cnt % 5 == 0)
        {
            //6020电机归位
            //写入电流值即可
            for(uint8_t i = 0;i<4;i++)
            {
                Act_Temp = chassis_motor_info[i + 4].Data.Encoder;
                AGV_chassis_Zero_Check(Tar_Temp[i],&Act_Temp,8192);
                Out_temp = PID_Calc(&chassis_motor_ctrl[i + 4].Angle_pid,Act_Temp,Tar_Temp[i]);

                Act_Temp = chassis_motor_info[i + 4].Data.Velocity;
                chassis_motor_info[i + 4].Data.SET_Current = PID_Calc(&chassis_motor_ctrl[i + 4].Speed_pid,Act_Temp,Out_temp);

                //回到机械零点，使用这一段代码时目标值千万不能为0
                // Act_Temp = chassis_motor_info[i + 4].Data.Encoder;
                // Out_temp = PID_Calc(&chassis_motor_ctrl[i + 4].Angle_pid,Act_Temp,Tar_Temp[i]);

                // Act_Temp = chassis_motor_info[i + 4].Data.Velocity;
                // chassis_motor_info[i + 4].Data.SET_Current = PID_Calc(&chassis_motor_ctrl[i + 4].Speed_pid,Act_Temp,Out_temp);
            }
            DJI_Motor_ctrl_6020(chassis_motor_info,&hfdcan1,0);
        }
        vTaskDelay(pdMS_TO_TICKS(1));//1ms
    }  
}

//过零处理函数
void AGV_chassis_Zero_Check(int16_t Tar_Angle,int16_t *Acl_Angle,int16_t max)
{
    if(Tar_Angle - *Acl_Angle > max / 2)
    {
        *Acl_Angle += max;
    }else if(Tar_Angle - *Acl_Angle < -max / 2)
    {
        *Acl_Angle -= max;
    }else
    {

    }
}

//映射遥控器数据到底盘控制
void RemoteControl(void)
{
    chassis_speed.vx = (float)remote_ctrl.rc.ch[3] * 4 / 760.0f;//4m/s
    chassis_speed.vy = (float)remote_ctrl.rc.ch[2] * 4 / 760.0f;
    chassis_speed.vw = (float)remote_ctrl.rc.ch[1] * 4 / 760.0f;//(float)remote_ctrl.rc.ch[0] / 165.0f;//4rad/s 线速度为1.08m/s


    if(fabs(chassis_speed.vx) < 0.03)
    {
        chassis_speed.vx = 0;
    }
    if(fabs(chassis_speed.vy) < 0.03)
    {
        chassis_speed.vy = 0;
    }

    // //映射成转速
    // // gimbal_ctrl.vpitch = (float)remote_ctrl.rc.ch[1] / 165.0f;//4rad/s 
    gimbal_ctrl.vyaw = (float)remote_ctrl.rc.ch[0] * 360 / 760.0f;//4rad/s 
	// printf("%.2f,%.2f,%.2f,%d\n",chassis_speed.vx,chassis_speed.vy,gimbal_ctrl.vyaw,remote_ctrl.rc.ch[3]);
	// printf("%d,%d,%d,%d\n",remote_ctrl.rc.ch[0],remote_ctrl.rc.ch[1],remote_ctrl.rc.ch[2],remote_ctrl.rc.ch[3]);

}

//数据解算
void AGV_angle_calc(AGV_chassis_speed_Typedef *speed,int16_t *out_angle)
{
    float atan_angle[4],wheel_angle[4];
    int16_t act_temp[4];

    if(!(speed->vx == 0 && speed->vy == 0 && speed->vw == 0))//防止atan2()的两个参数都为0,在原点函数会返回nan
    {
        //左前1
        atan_angle[0]=atan2((speed->vx + speed->vw*CHASSIS_OFFSET_Y),(speed->vy + speed->vw*CHASSIS_OFFSET_X))*180.0f/PI;
        //左后2
        atan_angle[1]=atan2((speed->vx - speed->vw*CHASSIS_OFFSET_Y),(speed->vy + speed->vw*CHASSIS_OFFSET_X))*180.0f/PI;
        //右后3
        atan_angle[2]=atan2((speed->vx - speed->vw*CHASSIS_OFFSET_Y),(speed->vy - speed->vw*CHASSIS_OFFSET_X))*180.0f/PI;	
        //右前4
        atan_angle[3]=atan2((speed->vx + speed->vw*CHASSIS_OFFSET_Y),(speed->vy - speed->vw*CHASSIS_OFFSET_X))*180.0f/PI;

    }
		
	// printf("%.2f,%.2f,%.2f,%.2f\n",atan_angle[0],atan_angle[1],atan_angle[2],atan_angle[3]);

    //角度转换为机械角度
	wheel_angle[0] = L_Q_6020_Middle_ECD + (atan_angle[0] * 4096.0f / 180.0f);
	wheel_angle[1] = L_H_6020_Middle_ECD + (atan_angle[1] * 4096.0f / 180.0f);
	wheel_angle[2] = R_H_6020_Middle_ECD + (atan_angle[2] * 4096.0f / 180.0f);
	wheel_angle[3] = R_Q_6020_Middle_ECD + (atan_angle[3] * 4096.0f / 180.0f);

    //
	for(int i = 0;i < 4;i++)
	{
 	   loop_f(&wheel_angle[i],8192.0f);	
	}

	// printf("%.2f,%.2f,%.2f,%.2f\n",wheel_angle[0],wheel_angle[1],wheel_angle[2],wheel_angle[3]);

	// for(int i = 0;i < 4;i++)
	// {
    //     reverse_flag[i] *= Find_min_Angle(&wheel_angle[i],(float)chassis_motor_info[i + 4].Data.Encoder);
    //     loop_f(&wheel_angle[i],8192.0f);
	// }

    //输出
    if(speed->vx == 0 && speed->vy == 0 && speed->vw == 0) 
    {
        for(int i = 0;i < 4;i++)
        {
            out_angle[i] = (int16_t)wheel_angle_last[i];
        }
    }else
    {
        for(int i = 0;i < 4;i++)
    	{
            if(Find_min_Angle(&wheel_angle[i],(float)chassis_motor_info[i + 4].Data.Encoder) == -1)
            {
                reverse_flag[i] = -1;
            }else
            {
                reverse_flag[i] = 1;
            }
            loop_f(&wheel_angle[i],8192.0f);
        }

        for(int i = 0;i < 4;i++)
        {
            out_angle[i] = (int16_t)wheel_angle[i];
            wheel_angle_last[i] = (int16_t)wheel_angle[i];
        }
    }
    //2
    // printf("%d,%d\n",out_angle[1],chassis_motor_info[1 + 4].Data.Encoder);
}

void loop_f(float *angle,float max)
{
	if(*angle > max)
	{
		*angle -= max;
	}else if(*angle < 0)
	{
		*angle += max;
	}
    else
    {

    }
}

void AGV_speed_calc(AGV_chassis_speed_Typedef *speed,int16_t *out_speed)
{
	float wheel_rpm_ratio;
    wheel_rpm_ratio = 786.43f/WHEEL_PERIMETER;
    // wheel_rpm_ratio = (60.0f/WHEEL_PERIMETER)*3591/187;

    float wheel_speed[4],x;

    wheel_speed[0] = sqrt(	pow(speed->vx + speed->vw*CHASSIS_OFFSET_Y,2)
                       +	pow(speed->vy + speed->vw*CHASSIS_OFFSET_X,2)
                       ) * wheel_rpm_ratio;	
	wheel_speed[1] = sqrt(	pow(speed->vx - speed->vw*CHASSIS_OFFSET_Y,2)
                       +	pow(speed->vy + speed->vw*CHASSIS_OFFSET_X,2)   
                       ) * wheel_rpm_ratio;	
	wheel_speed[2] = sqrt(	pow(speed->vx - speed->vw*CHASSIS_OFFSET_Y,2)
                       +	pow(speed->vy - speed->vw*CHASSIS_OFFSET_X,2)
                       ) * wheel_rpm_ratio;	
	wheel_speed[3] = sqrt(	pow(speed->vx + speed->vw*CHASSIS_OFFSET_Y,2)
                       +	pow(speed->vy - speed->vw*CHASSIS_OFFSET_X,2)
                       ) * wheel_rpm_ratio;	
                                                                                            
	// printf("%.2f,%.2f,%.2f,%.2f\n",wheel_speed[0],wheel_speed[1],wheel_speed[2],wheel_speed[3]);

    // x = sqrt(pow(speed->vy,2) + pow(speed->vx,2));
	
	// if(x < 0.1f )
	// {
	// 	wheel_speed[0] = 0;	
	// 	wheel_speed[1] = 0;	
	// 	wheel_speed[2] = 0;	
	// 	wheel_speed[3] = 0;	
	// }


    for(int i = 0;i < 4;i++)
    {
        //乘-1是因为3508轮子的正转是使整车向后
        out_speed[i] = reverse_flag[i] * (int16_t)wheel_speed[i] * -1;
    }
    // for(int i = 0;i < 4;i++)
    // {
    //     //乘-1是因为3508轮子的正转是使整车向后
    //     out_speed[i] = reverse_flag[i] * 200 * -1;
    // }
    // for(int i = 0;i < 4;i++)
    // {
    //     //乘-1是因为3508轮子的正转是使整车向后
    //     out_speed[i] =  (int16_t)wheel_speed[i] * -1;
    // }
}
void chassis_control(void)
{
    // int16_t Tar_Temp = 0;
    int16_t Act_Temp = 0;
    int16_t Out_temp = 0;

    int16_t out_angle[4],out_speed[4];
    AGV_angle_calc(&chassis_speed,out_angle);
    AGV_speed_calc(&chassis_speed,out_speed);



    for(uint8_t i = 0;i < 4;i++)
    {
        Act_Temp = chassis_motor_info[i + 4].Data.Encoder;
        AGV_chassis_Zero_Check(out_angle[i],&Act_Temp,8192);
        Out_temp = PID_Calc(&chassis_motor_ctrl[i + 4].Angle_pid,Act_Temp,out_angle[i]);

        Act_Temp = chassis_motor_info[i + 4].Data.Velocity;
        chassis_motor_info[i + 4].Data.SET_Current = PID_Calc(&chassis_motor_ctrl[i + 4].Speed_pid,Act_Temp,Out_temp);



        // chassis_motor_info[i + 4].Data.SET_Current = 0;
        // if(i == 2)
        // {

        // Act_Temp = chassis_motor_info[i + 4].Data.Encoder;
        // AGV_chassis_Zero_Check(out_angle[i],&Act_Temp,8192);
        // Out_temp = PID_Calc(&chassis_motor_ctrl[i + 4].Angle_pid,Act_Temp,debug_Target);

        // Act_Temp = chassis_motor_info[i + 4].Data.Velocity;
        // chassis_motor_info[i + 4].Data.SET_Current = PID_Calc(&chassis_motor_ctrl[i + 4].Speed_pid,Act_Temp,Out_temp);
        // printf("%d,%d,%d,%d\n",debug_Target,chassis_motor_info[2 + 4].Data.Encoder,chassis_motor_info[2 + 4].Data.SET_Current);
        // }

    }

    //发送6020控制电流
    DJI_Motor_ctrl_6020(chassis_motor_info,&hfdcan1,0);

    for(uint8_t i = 0;i < 4;i++)
    {
        chassis_motor_info[i].Data.SET_Current = PID_Calc(&chassis_motor_ctrl[i].Speed_pid,
                                                    chassis_motor_info[i].Data.Velocity,
                                                    out_speed[i]);
        // chassis_motor_info[i].Data.SET_Current = PID_Calc(&chassis_motor_ctrl[i].Speed_pid,
        //                                             chassis_motor_info[i].Data.Velocity,
        //                                             200*reverse_flag[i]);
    }

    //发送3508控制电流
     DJI_Motor_ctrl(chassis_motor_info,&hfdcan2,0);
}

//
int8_t Find_min_Angle(float *tar_angle,float act_angle)
{
    float err;
    int8_t flag = 1;
    int16_t max = 8192;

    if(*tar_angle - act_angle > max / 2)
    {
        act_angle += max;
    }else if(*tar_angle - act_angle < -max / 2)
    {
        act_angle -= max;
    }else
    {

    }

    err = *tar_angle - act_angle;
    if(fabs(err) > 2048)
    {
        *tar_angle -= 4096;
        flag = -1;
    }
    return flag;
}

//
void AGV_Gimbal_Init(void)
{
    gimbal_motor_info[pitch].ID_Set.TxIdentifier = 0x200;//初始化3508发送标识符ID
    gimbal_motor_info[yaw].ID_Set.TxIdentifier = 0x200;//初始化3508发送标识符ID

    float Spe_3508_PID[3] = {Speed_3508_KP,Speed_3508_KI,Speed_3508_KD};
    float Angle_3508_PID[3] = {Angle_KP,Angle_KI,Angle_KD};
    PID_init(&gimbal_motor_ctrl[pitch].Speed_pid,(PID_mode_e)0,Spe_3508_PID,Speed_3508_PID_Out_Max,Speed_3508_Iout_Max);
    PID_init(&gimbal_motor_ctrl[yaw].Speed_pid,(PID_mode_e)0,Spe_3508_PID,Speed_3508_PID_Out_Max,Speed_3508_Iout_Max);
    PID_init(&gimbal_motor_ctrl[pitch].Angle_pid,(PID_mode_e)0,Angle_3508_PID,Speed_3508_PID_Out_Max,Angle_Iout_Max);
    PID_init(&gimbal_motor_ctrl[yaw].Angle_pid,(PID_mode_e)0,Angle_3508_PID,Speed_3508_PID_Out_Max,Angle_PID_Out_Max);
}

void gimbal_control(void)
{
    /*测试云台*/
//    printf("%d,%d\n",gimbal_motor_info[pitch].Data.Velocity,gimbal_motor_info[yaw].Data.Velocity);
    // printf("%d,%d\n",gimbal_motor_info[pitch].Data.Velocity,chassis_motor_info[1].Data.Velocity);
    // printf("%d,%d\n",gimbal_motor_info[pitch].Data.Encoder,gimbal_motor_info[yaw].Data.Encoder);

    // gimbal_motor_info[pitch].Data.SET_Current = 1000;
    // gimbal_motor_info[yaw].Data.SET_Current = 300;

    // float angle_pitch = gimbal_motor_info[pitch].Data.Encoder;//弧度建立编码值与弧度的对应关系
    // angle_pitch += gimbal_ctrl.vpitch ;




    // Out_temp = PID_Calc(&gimbal_motor_ctrl[yaw].Angle_pid,Act_Temp,Tar_Temp[i]);

    // Act_Temp = gimbal_motor_info[yaw].Data.Velocity;
    // gimbal_motor_info[yaw].Data.SET_Current = PID_Calc(&gimbal_motor_ctrl[yaw].Speed_pid,Act_Temp,Out_temp);


    fp32 n_target = (fp32)gimbal_ctrl.vyaw;


    

    gimbal_motor_info[yaw].Data.SET_Current = PID_Calc(&gimbal_motor_ctrl[yaw].Speed_pid,
                                                    gimbal_motor_info[yaw].Data.Velocity,
                                                    n_target);//gimbal_ctrl.vyaw
    //发送3508控制电流
    DJI_Motor_ctrl(gimbal_motor_info,&hfdcan1,0);

}
