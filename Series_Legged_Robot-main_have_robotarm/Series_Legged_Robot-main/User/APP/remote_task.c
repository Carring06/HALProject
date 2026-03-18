#include "remote_task.h"
//#include "watch_task.h"
#include "cmsis_os.h"
#include "Remote_Control.h"
#include "chassisR_task.h"
#define REMOTE_OVERTIME	1000

extern chassis_t chassis_move;
extern INS_t INS;
uint32_t REMOTE_TIME=10;//ps2手柄任务周期是10ms
extern vmc_leg_t right;			
extern vmc_leg_t left;

extern uint16_t adc_val[2];

//extern Remote_Info_Typedef remote_ctrl;

/**************************************************************************
Function: Sbus Remote
Input   : none
Output  : none
Auth    : DHY (qq:965849293)
Date		: 2024
**************************************************************************/	
void Remote_task(void)
{
//	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE*2);
	float dt = REMOTE_TIME / 1000.0f;
	static float last_jump_vrb = 0;
	static uint32_t vbat_low_count = 0;
	
	while(1)
	{

		remote_ctrl.rc.last_s[0] = remote_ctrl.rc.s[0];
		remote_ctrl.rc.last_s[1] = remote_ctrl.rc.s[1];

		remote_crtl(&chassis_move,(float)REMOTE_TIME/1000.0f);

		//	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE*2); // 接收完毕后重启
		osDelay(REMOTE_TIME);

	}
}
//1   0
//	1
//	3
//	2

void remote_crtl(chassis_t *chassis,float dt)
{
	
	if(remote_ctrl.rc.s[0] == 2&&remote_ctrl.rc.s[1] == 1) 
	{
		chassis->recover_flag=1;
		chassis->start_flag=1;
		
		chassis->jump_flag =0;
		chassis->jump_flag1=0;
		chassis->jump_flag2=0;
	}
	else if(remote_ctrl.rc.s[0] == 2) 
	{
		chassis->start_flag=0;
		
		chassis->recover_flag=0;
		
		chassis->jump_flag = 0;
	}
	else if(remote_ctrl.rc.s[0] == 3&&remote_ctrl.rc.s[1] == 2) 
	{
		chassis->start_flag=1;
		chassis->recover_flag=0;
	}
	
	/*
	if(remote_ctrl.rc.s[0] == 1&&remote_ctrl.rc.s[1] == 3)
	{
		chassis->recover_flag=1;//需要自起
		chassis->leg_set_l = chassis->leg_set_r=0.08f;//原始腿长
	}*/
	/********************jump*******************/
	if(remote_ctrl.rc.s[0] == 1&&remote_ctrl.rc.s[1] == 3&&chassis->jump_flag == 0){
	//chassis->jump_flag1=1;
	//	chassis->jump_flag2=1;
	//	chassis->jump_flag = 1;
	}	
	else if(remote_ctrl.rc.s[0] == 3&&remote_ctrl.rc.s[1] == 3)
	{
		chassis->jump_flag = 0;
	}
			
	
	if(chassis->start_flag==1)
	{
		//启动
		chassis->v_set=(((float)remote_ctrl.rc.ch[3]/660))*(2.0f);//往前大于0
		
		
		chassis->x_set=chassis->x_set+chassis->v_set*dt;
    			
		chassis->turn_set=chassis->turn_set+((float)remote_ctrl.rc.ch[2]/660)*(-0.02f);//往右大于0
		
		
	  
		//chassis->roll_set=(((float)remote_ctrl.rc.ch[0]/660.0f))*(-0.7f);
		chassis->roll_set=(((float)remote_ctrl.rc.ch[3]/660))*(0.8f);
		
		PID_Calc(&Roll_Pid,chassis->roll,chassis->roll_set);
//		mySaturate(&chassis->roll_set,-0.40f,0.40f);	

		if(chassis_move.start_flag==1&&remote_ctrl.rc.s[1] != 2)	
		{	
			chassis->leg_set_l = chassis->leg_set_r= (((float)remote_ctrl.rc.ch[1]/660))*(0.2f) + 0.25f; 
		}
		else if(chassis_move.start_flag==1)	
		{
			chassis->leg_set_l = chassis->leg_set_r= 0.2f; 
		}
		if(remote_ctrl.rc.s[0] == 3&&remote_ctrl.rc.s[1] == 2)
		{
			chassis->leg_set_l = chassis->leg_set_r= 0.18;
		}
			
			//chassis->leg_set_l -=Roll_Pid.out;
		//	chassis->leg_set_r +=Roll_Pid.out;
			
		mySaturate(&chassis->leg_set_l,0.13f,0.36f);
		mySaturate(&chassis->leg_set_r,0.13f,0.36f);
		
		if(fabsf(chassis->leg_set_r-chassis->last_leg_set)>0.0001f)
		{//遥控器控制腿长在变化
			right.leg_flag=1;	//为1标志着遥控器在控制腿长伸缩，根据这个标志可以不进行离地检测，因为当腿长在主动伸缩时，离地检测会误判为离地了
			left.leg_flag=1;	 			
		}
		chassis->last_leg_set=chassis->leg_set_r;
	 }
	else if(chassis->start_flag==0)
	{
		//关闭
		chassis->v_set=0.0f;//清零
		chassis->x_set=chassis->x_filter=0;//保存
		chassis->turn_set=chassis->total_yaw;//保存
		chassis->leg_set_l = chassis->leg_set_r=0.20f;//原始腿长
		chassis->roll_set=0.0f; 
		
		chassis->jump_flag1=0;
		chassis->jump_flag2=0;
	}
}


