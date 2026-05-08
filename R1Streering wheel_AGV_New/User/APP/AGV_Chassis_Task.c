#include "AGV_Chassis_Task.h"

//c
#include <stdbool.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

//motor drive
#include "DJI_Motor.h"
#include "DM_Motor.h"

//pid
#include "pid.h"

//REMOTE_CONTROL
#include "Remote_Control.h" 

#include "fdcan.h"

const DJI_Motor_Type_e Motor_3508_Type[4] = {DJI_M3508, DJI_M3508, DJI_M3508, DJI_M3508}; //底盘电机类型数组

//pid结构体

DM_Motor_Ctrl_Typedef DM_6220_pid_set[4];				//底盘6220电机控制结构体     都有内置目标值 
DJI_Motor_Ctrl_Typedef DJI_3508_pid_set[4];          	//底盘3508电机控制结构体

//电机信息结构体
extern DJI_Motor_Info_Typedef chassis_3508_motor[4];           //底盘四个3508电机的信息结构体 
extern DM_Motor_Info_Typedef Chassis_6220_DM_Motor[4];  //底盘四个6220电机的信息结构体

//底盘控制模式
Chassis_Mode chassis_mode = RC_NO_INIT; //初始状态为未初始化

//底盘速度结构体
Chassis_Speed chassis_speed;

Chassis_Speed *absolute_chassis_speed=&chassis_speed;

//遥控器数据结构体
extern Remote_Info_Typedef remote_ctrl;

//这个不能放函数里面,因为每次进这个函数,都会被先初始化一次
fp32 wheel_angle_last[4];  //6220编码器上一次的目标角度

//开始标志位,初始化为Initing
SystemValue systemvalue=Initing;


float pid[6] ={0,0,0,0,0,0};

void AGV_Chassis_Task(void* argument )
{
    Motor_pid_init(); //电机PID初始化
    Chassis_DM_Motor_Enable();
    
    Wheel_Angle_Last_Init(); // 初始化上一次的目标角度为机械零位
   
	//get now time
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
 	
    while(1)
    {
         if(systemvalue == Initing)
        {
		  AGV_Chassis_Init();  //1.5s
		  Wheel_Angle_Last_Init();    //初始化四个最后速度为机械零点
		  systemvalue = Running;      //系统初始化结束							
       }
       
	   //遥控获取运动数据						
		RemoteControlChassis();	   
        CHASSIS_Single_Loop_Out();

		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5)); //每5ms执行一次	 
	}
}


void Chassis_DM_Motor_Enable(void)
{
	//转向电机使能
    Enable_Motor_Mode(&hfdcan1,0x01, Mit_mode, 1);
 	Enable_Motor_Mode(&hfdcan2,0x02, Mit_mode, 1);
	Enable_Motor_Mode(&hfdcan2,0x03, Mit_mode, 1);
	Enable_Motor_Mode(&hfdcan1,0x04, Mit_mode, 1);
}

void AGV_Chassis_Init(void)
{
    bool return_flag = true;

    uint16_t cnt = 0;                   // 初始化尝试计数器
    float tar_angle_temp = 0.0f;        // 临时变量，用于存储目标角度

    while (return_flag)
    {
        if(cnt > 1500)
        {
          return_flag = false; // 超过1500次尝试后，停止尝试 并且回正
        }
           cnt++;
        
          /*************************************************************底盘电机6220归位*******************************************************/
		//DM6220电机调控力矩 
        float tor_1,tor_2,tor_3,tor_4;
		//左前电机归位
		float angle_out_1 = PID_Calc(&DM_6220_pid_set[0].Angle_pid, Chassis_6220_DM_Motor[0].Data.pos, tar_angle_temp, 0);  //角度环pid输出，输入为当前角度和目标角度
		tor_1 = PID_Calc(&DM_6220_pid_set[0].Speed_pid, Chassis_6220_DM_Motor[0].Data.vel, angle_out_1, 1); //速度环pid输出，输入为当前速度和角度环pid输出    
		//右前电机归位
		float angle_out_2 = PID_Calc(&DM_6220_pid_set[1].Angle_pid, Chassis_6220_DM_Motor[1].Data.pos, tar_angle_temp, 0);  //角度环pid输出，输入为当前角度和目标角度
		tor_2  = PID_Calc(&DM_6220_pid_set[1].Speed_pid, Chassis_6220_DM_Motor[1].Data.vel, angle_out_2, 1); //速度环pid输出，输入为当前速度和角度环pid输出
		//右后电机归位
		float angle_out_3 = PID_Calc(&DM_6220_pid_set[2].Angle_pid, Chassis_6220_DM_Motor[2].Data.pos, tar_angle_temp, 0);  //角度环pid输出，输入为当前角度和目标角度
		tor_3= PID_Calc(&DM_6220_pid_set[2].Speed_pid, Chassis_6220_DM_Motor[2].Data.vel, angle_out_3, 1); //速度环pid输出，输入为当前速度和角度环pid输出
		//左后电机归位
		float angle_out_4 = PID_Calc(&DM_6220_pid_set[3].Angle_pid, Chassis_6220_DM_Motor[3].Data.pos, tar_angle_temp, 0);  //角度环pid输出，输入为当前角度和目标角度
		tor_4 = PID_Calc(&DM_6220_pid_set[3].Speed_pid, Chassis_6220_DM_Motor[3].Data.vel, angle_out_4, 1); //速度环pid输出，输入为当前速度和角度环pid输出

		DM_Motor_Four_Ctrl(tor_1,tor_2,tor_3,tor_4);
	}

}

void DM_Motor_Four_Ctrl(fp32 tor1, fp32 tor2, fp32 tor3, fp32 tor4)
{
	 //力矩控制 模式下，位置、速度、kp、kd参数不需要，直接传0，最后一个参数为延时1ms
	mit_ctrl2(&hfdcan1,Chassis_6220_DM_Motor[0].ID_Set.TxIdentifier,0,0,0,0,tor1,0);
	mit_ctrl2(&hfdcan2,Chassis_6220_DM_Motor[1].ID_Set.TxIdentifier,0,0,0,0,tor2,0);
	mit_ctrl2(&hfdcan2,Chassis_6220_DM_Motor[2].ID_Set.TxIdentifier,0,0,0,0,tor3,0);
	mit_ctrl2(&hfdcan1,Chassis_6220_DM_Motor[3].ID_Set.TxIdentifier,0,0,0,0,tor4,0);
    osDelay(1);
}
void RemoteControlChassis(void) 
{
	//底盘朝前模式
    //大疆遥控
	absolute_chassis_speed->vx=(fp32)remote_ctrl.rc.ch[2]/165;	 //4m/s
	absolute_chassis_speed->vy=(fp32)remote_ctrl.rc.ch[3]/165;	 //4m/s
	absolute_chassis_speed->vw=-(fp32)remote_ctrl.rc.ch[0]/165;  //顺时针4rad  i0 
    
    //富斯遥控
//    absolute_chassis_speed->vx=(fp32)FS_Remote_Ctrl.ch[3]/149;  //5m/s
//    absolute_chassis_speed->vy=(fp32)FS_Remote_Ctrl.ch[2]/150;  //5m/s
//    absolute_chassis_speed->vw=(fp32)FS_Remote_Ctrl.ch[0]/150;  //5m/s
}

void Wheel_Angle_Last_Init(void)
{
    wheel_angle_last[0]= 0.0f;
    wheel_angle_last[1]= 0.0f;
    wheel_angle_last[2]= 0.0f;
    wheel_angle_last[3]= 0.0f;		
}
//2.3.4号电机pid参数
fp32 DM_6220_pos_pid[3] = {20.0f, 0.0f, 0.0f};      		//底盘6220电机位置环PID参数 {Kp, Ki, Kd}
fp32 DM_6220_speed_pid[3] = {0.07f , 0.0f , 0.0f};          //底盘电机6220速度环PID参数 {Kp, Ki, Kd}

//这个1号电机有点特殊 单独给一个pid参数 //建议自己去调一遍 我这个参数很随意的 不是很好
fp32 DM_6220_Motor1_pos_pid[3]={20.0f,0.0f,0.0f};
fp32 DM_6220_Motor1_speed_pid[3]={0.053f,0.0f,0.0f};

//底盘3508电机速度环PID参数 {Kp, Ki, Kd}
fp32 DJI_3508_speed_pid[3] = {13.0f, 0.03f, 0.0f};		

void Motor_pid_init(void)
{
	PID_mode_e A =PID_POSITION;
	//注释掉的这部分是pid调参时用到的 平时不用开
/*	
	DM_6220_pid_set[0].Angle_pid.Initlized = false;
	DM_6220_pid_set[0].Speed_pid.Initlized = false;

	DM_6220_pid_set[1].Angle_pid.Initlized = false;
	DM_6220_pid_set[1].Speed_pid.Initlized = false;
	
	DM_6220_pid_set[2].Angle_pid.Initlized = false;
	DM_6220_pid_set[2].Speed_pid.Initlized = false;
	
	DM_6220_pid_set[3].Angle_pid.Initlized = false;
	DM_6220_pid_set[3].Speed_pid.Initlized = false;
*/	
    PID_init(&DM_6220_pid_set[0].Angle_pid, A, DM_6220_Motor1_pos_pid, 1000.0f, 500.0f); //底盘6220电机位置环PID初始化 扭矩输出必须小于10
	PID_init(&DM_6220_pid_set[0].Speed_pid, A, DM_6220_Motor1_speed_pid,10.0f,500.0f); 
	
    PID_init(&DM_6220_pid_set[1].Angle_pid, A, DM_6220_pos_pid, 1000.0f, 500.0f); //底盘6220电机位置环PID初始化
	PID_init(&DM_6220_pid_set[1].Speed_pid, A, DM_6220_speed_pid,10.0f,500.0f);
	
    PID_init(&DM_6220_pid_set[2].Angle_pid, A, DM_6220_pos_pid, 1000.0f, 500.0f); //底盘6220电机位置环PID初始化
	PID_init(&DM_6220_pid_set[2].Speed_pid, A, DM_6220_speed_pid,10.0f,500.0f);  
	
    PID_init(&DM_6220_pid_set[3].Angle_pid, A, DM_6220_pos_pid, 1000.0f, 500.0f); //底盘6220电机位置环PID初始化
	PID_init(&DM_6220_pid_set[3].Speed_pid, A, DM_6220_speed_pid,10.0f,500.0f); 

    PID_init(&DJI_3508_pid_set[0].Speed_pid, A, DJI_3508_speed_pid, 10000.0f, 5000.0f); //底盘3508电机速度环PID初始化
    PID_init(&DJI_3508_pid_set[1].Speed_pid, A, DJI_3508_speed_pid, 10000.0f, 5000.0f); //底盘3508电机速度环PID初始化
    PID_init(&DJI_3508_pid_set[2].Speed_pid, A, DJI_3508_speed_pid, 10000.0f, 5000.0f); //底盘3508电机速度环PID初始化
    PID_init(&DJI_3508_pid_set[3].Speed_pid, A, DJI_3508_speed_pid, 10000.0f, 5000.0f); //底盘3508电机速度环PID初始化  
}

//存放目标速度目标角度的数组
fp32 chassis_6220_setangle[4];
int16_t chassis_3508_setspeed[4];

//如果有云台 将云台坐标转换为底盘坐标
//没有云台 就是简单的根据遥控值进行坐标转换
void Absolute_Cal(Chassis_Speed* absolute_speed, fp32 angle)
{
	//取负因为是逆时针,而4310关节电机传进来的pos是弧度制,云台坐标转化到底盘坐标为θ,θ逆时针为正,yaw轴顺时为正
	//yaw轴顺时针,4310电机逆时针,pos为正,而实际上云台坐标系和底盘坐标系的夹角是yaw角,传入yaw角也不是不行,主要是yaw角有延迟
    fp32 angle_hd= -angle;
    Chassis_Speed temp_speed;
    temp_speed.vw = absolute_speed->vw; 
    temp_speed.vx = absolute_speed->vx * cos(angle_hd) - absolute_speed->vy * sin(angle_hd);//横向速度
    temp_speed.vy = absolute_speed->vx * sin(angle_hd) + absolute_speed->vy * cos(angle_hd);//纵向速度
	//将底盘方向转化为6220目标角度
	AGV_angle_calc(&temp_speed,chassis_6220_setangle);
    //将底盘速度转化为3508目标角度
	AGV_speed_calc(&temp_speed,chassis_3508_setspeed);
}

/**
  * @brief  找出两角的较小差值
  * @param  角1，角2
  * @retval 
  * @attention 
  */

fp32 Find_min_Angle(int16_t angle1,fp32 angle2)
{
	fp32 err;

    err = (fp32)angle1 - angle2;
    if(fabs(err) > Pi )
    {
        err = 2*Pi - fabs(err);
    }
    return err;
}


/**
  * @brief  计算底盘驱动电机的目标速度
  * @param  speed 底盘坐标的速度 
  * @param  out_speed 3508目标速度
  * @retval 
  * @attention
  */
float   x;
int8_t sign_group[4]={1,1,1,1}; //电机正反标志位
//3508目标速度计算
void AGV_speed_calc(Chassis_Speed *speed, int16_t* out_speed) 
{ 
    fp32 wheel_rpm_ratio;   //线速度到转速(rpm)转换
	
	//减速比CHASSIS_DECELE_RATIO
	//周长WHEEL_PERIMETER
    wheel_rpm_ratio = 	CHASSIS_DECELE_RATIO / WHEEL_PERIMETER * 60;
    
	//先转后走,而不是转的过程中也在走
	x = sqrt(pow(speed->vy,2) + pow(speed->vx,2));
	
	if(x < 0.1f )
	{
		speed->vy=0.f;
		speed->vx=0.f;	
	} 
	int16_t wheel_speed[4];
	//这一步是将线速度转化3508的转速rpm
    wheel_speed[0] = sqrt(	pow(speed->vy - speed->vw*CHASSIS_OFFSET_X,2)
                       +	pow(speed->vx - speed->vw*CHASSIS_OFFSET_Y,2)
                       ) * wheel_rpm_ratio;
    wheel_speed[1] = sqrt(	pow(speed->vy - speed->vw*CHASSIS_OFFSET_X,2)
                       +	pow(speed->vx + speed->vw*CHASSIS_OFFSET_Y,2)
                       ) * wheel_rpm_ratio;
    wheel_speed[2] = sqrt(	pow(speed->vy + speed->vw*CHASSIS_OFFSET_X,2)
                       +	pow(speed->vx + speed->vw*CHASSIS_OFFSET_Y,2)
                       ) * wheel_rpm_ratio;
    wheel_speed[3] = sqrt(	pow(speed->vy + speed->vw*CHASSIS_OFFSET_X,2)
                       +	pow(speed->vx - speed->vw*CHASSIS_OFFSET_Y,2) 
                       ) * wheel_rpm_ratio; 
	
    for(int i=0;i<4;i++)
	//-1乘是因为3508轮子的正转是使整车向后
    out_speed[i] = (-1)*sign_group[i] * wheel_speed[i];	 
}

//两个参数分别是,底盘坐标速度 / 6020目标速度
//speed_vx是横向速度,speed_vy是纵向速度,speed_vw是旋转速度

fp32 wheel_angle[4]; //6220目标角度
void AGV_angle_calc(Chassis_Speed *speed, fp32* out_angle) 
{
 	float angle_temp;
	fp64 atan_angle[4];
	//6020目标角度计算
    if(!(speed->vx == 0 && speed->vy == 0 && speed->vw == 0))//防止atan2()的两个参数都为0,在原点函数会返回nan
    {
		//应转角度
		  //左前
		  atan_angle[0]=atan2((speed->vx - speed->vw*CHASSIS_OFFSET_Y),(speed->vy - speed->vw*CHASSIS_OFFSET_X));	
		   //右前
		  atan_angle[1]=atan2((speed->vx - speed->vw*CHASSIS_OFFSET_Y),(speed->vy + speed->vw*CHASSIS_OFFSET_X));
		  //右后
		  atan_angle[2]=atan2((speed->vx + speed->vw*CHASSIS_OFFSET_Y),(speed->vy + speed->vw*CHASSIS_OFFSET_X));
		  //左后
		  atan_angle[3]=atan2((speed->vx + speed->vw*CHASSIS_OFFSET_Y),(speed->vy - speed->vw*CHASSIS_OFFSET_X));		 		
    } 

		wheel_angle[0] =  (fp32)atan_angle[0] ;
		wheel_angle[1] =  (fp32)atan_angle[1] ;
		wheel_angle[2] =  (fp32)atan_angle[2] ;
		wheel_angle[3] =  (fp32)atan_angle[3] ;
	
		//加个回环
		AngleLoop_f(&wheel_angle[0],6.28f);//浮点型角度回环
		AngleLoop_f(&wheel_angle[1],6.28f);
		AngleLoop_f(&wheel_angle[2],6.28f);
		AngleLoop_f(&wheel_angle[3],6.28f);
		
						
		angle_temp = Chassis_6220_DM_Motor[0].Data.pos; //当前角度
			
		AngleLoop_f(&angle_temp,2*Pi);				
			
		if(fabs(Find_min_Angle(angle_temp,wheel_angle[0]))>(Pi/2))
		{
			for(int i=0;i<4;i++)
			{
				wheel_angle[i] += Pi; //加180度	
			}		
				for(int i=0;i<4;i++)	
				{
				sign_group[i] = -1;
				}					
		}
		else
		{	
				for(int i=0;i<4;i++)	
				{
				sign_group[i] = 1;
				}
		}
			
		AngleLoop_f(&wheel_angle[0],2*Pi);
		AngleLoop_f(&wheel_angle[1],2*Pi);
		AngleLoop_f(&wheel_angle[2],2*Pi);
		AngleLoop_f(&wheel_angle[3],2*Pi);
			
	  if(speed->vx == 0 && speed->vy == 0 && speed->vw == 0)//摇杆回中时
	  {
		//摇杆回中时,目标角度等于初始位置的角度
		for(int i=0;i<4;i++)
		 out_angle[i] = wheel_angle_last[i];		  
	  }
	  else
	  {
		for(int i=0;i<4;i++)
		{
		 out_angle[i] = wheel_angle[i];
		}
	  }  
}

//电机目标速度out_speed数组,    
void AGV_Set_Motor_Speed(int16_t*out_speed, DJI_Motor_Ctrl_Typedef* Motor ) 
{
    Motor[0].Speed_set.ref = out_speed[0];
    Motor[1].Speed_set.ref = out_speed[1];
    Motor[2].Speed_set.ref = out_speed[2];
    Motor[3].Speed_set.ref = out_speed[3];
}


//电机目标角度out_angle数组,    
void AGV_Set_Motor_angle(fp32 *out_angle, DM_Motor_Ctrl_Typedef* Motor ) 
{
    Motor[0].Angle_set.ref = out_angle[0];
    Motor[1].Angle_set.ref = out_angle[1];
    Motor[2].Angle_set.ref = out_angle[2];
    Motor[3].Angle_set.ref = out_angle[3];
}

float tor11,tor22,tor33,tor44;
void CHASSIS_Single_Loop_Out(void)
{	
	Absolute_Cal(absolute_chassis_speed,0);
				
    AGV_Set_Motor_Speed(chassis_3508_setspeed,DJI_3508_pid_set);//设置各个电机的目标速度
	AGV_Set_Motor_angle(chassis_6220_setangle,DM_6220_pid_set);//设置各个电机的目标角度

    /************************************底盘3508电机速度环计算*********************************************/
		//左前 
		chassis_3508_motor[0].Data.SET_Current                    
        = (int16_t)PID_Calc(
                            &DJI_3508_pid_set[0].Speed_pid, 
                            chassis_3508_motor[0].Data.Velocity, 
                            DJI_3508_pid_set[0].Speed_set.ref, 1);

		//右前
        chassis_3508_motor[1].Data.SET_Current
        = (int16_t)PID_Calc(
                            &DJI_3508_pid_set[1].Speed_pid, 
                            chassis_3508_motor[1].Data.Velocity, 
                            DJI_3508_pid_set[1].Speed_set.ref, 1);
		//右后
        chassis_3508_motor[2].Data.SET_Current
        = (int16_t)PID_Calc(
                            &DJI_3508_pid_set[2].Speed_pid, 
                            chassis_3508_motor[2].Data.Velocity, 
                            DJI_3508_pid_set[2].Speed_set.ref, 1);
		//左后
        chassis_3508_motor[3].Data.SET_Current
        = (int16_t)PID_Calc(    
                            &DJI_3508_pid_set[3].Speed_pid, 
                            chassis_3508_motor[3].Data.Velocity, 
                            DJI_3508_pid_set[3].Speed_set.ref, 1);
	
	/************************************底盘6020电机位置环速度环计算*********************************************/	

		//左前       
        fp32 angle_out1  
        =  PID_Calc(
                    &DM_6220_pid_set[0].Angle_pid, 
                    Chassis_6220_DM_Motor[0].Data.pos, 
                    DM_6220_pid_set[0].Angle_set.ref, 0);  //角度环pid输出，输入为当前角度和目标角度
        tor11  
        = PID_Calc(
                    &DM_6220_pid_set[0].Speed_pid, 
                    Chassis_6220_DM_Motor[0].Data.vel, 
                    angle_out1, 1); //速度环pid输出，输入为当前速度和角度环pid输出                        
		//右前
		fp32 angle_out2 
        =  PID_Calc(
                    &DM_6220_pid_set[1].Angle_pid, 
                    Chassis_6220_DM_Motor[1].Data.pos, 
                    DM_6220_pid_set[1].Angle_set.ref, 0);  //角度环pid输出，输入为当前角度和目标角度
        tor22 
        = PID_Calc(
                    &DM_6220_pid_set[1].Speed_pid, 
                    Chassis_6220_DM_Motor[1].Data.vel, 
                    angle_out2, 1); //速度环pid输出，输入为当前速度和角度环pid输出
		
		//右后
        fp32 angle_out3 
        =  PID_Calc(
                    &DM_6220_pid_set[2].Angle_pid, 
                    Chassis_6220_DM_Motor[2].Data.pos, 
                    DM_6220_pid_set[2].Angle_set.ref, 0);  //角度环pid输出，输入为当前角度和目标角度
        tor33          
        = PID_Calc(
                    &DM_6220_pid_set[2].Speed_pid, 
                    Chassis_6220_DM_Motor[2].Data.vel, 
                    angle_out3, 1); //速度环pid输出，输入为当前速度和角度环pid输出  
		
		//左后
        fp32 angle_out4 
        =  PID_Calc(
                    &DM_6220_pid_set[3].Angle_pid, 
                    Chassis_6220_DM_Motor[3].Data.pos, 
                    DM_6220_pid_set[3].Angle_set.ref, 0);  //角度环pid输出，输入为当前角度和目标角度
       tor44 
        = PID_Calc(
                    &DM_6220_pid_set[3].Speed_pid, 
                    Chassis_6220_DM_Motor[3].Data.vel, 
                    angle_out4, 1); //速度环pid输出，输入为当前速度和角度环pid输出

    /************************************将控制指令发送给电机*********************************************/
		
        //底盘6220电机调控
		DM_Motor_Four_Ctrl(tor11,tor22,tor33,tor44);
		
		//can发送给3508
		DJI_Motor_ctrl(chassis_3508_motor, &hfdcan1,1); //底盘3508电机调控
		DJI_Motor_ctrl(chassis_3508_motor, &hfdcan2,1);

}
