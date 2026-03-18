/**
  *********************************************************************
  * @file      chassisR_task.c/h
  * @brief     该任务控制右半部分的电机，分别是两个DM4310和一个DM6215，这三个电机挂载在can1总线上
	*						 从底盘上往下看，右上角的DM4310发送id为6、接收id为3，
	*						 右下角的DM4310发送id为8、接收id为4，
	*						 右边DM轮毂电机发送id为1、接收id为0。
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "chassisR_task.h"
#include "fdcan.h"
#include "cmsis_os.h"
#include "Remote_Control.h"
#include "DJI_Motor.h"
#include "VOFA.h"
#include "user_lib.h"
float LQR_K_R[12]={       

  -3.3259  , -0.2272,   -1.2959,   -1.2122 ,   3.3133,    0.3249,
    2.5219 ,   0.2142 ,   1.8118 ,   1.6073 ,   8.2857 ,   0.4180

};

//三次多项式拟合系数
float Poly_Coefficient[12][4]={
  {-98.3096, 140.2594, -86.4875, -0.1004},
  {12.4208, -10.6371, -4.1387, 0.0599},
  {-64.6022, 81.0188, -35.6636, 0.7330},
  {-29.9093, 41.7217, -21.7010, 0.1188},
  {30.9832, -11.9421, -9.7815, 6.0739},
  {1.6150, 0.5318, -1.9641, 1.0377},
  {482.8160, -431.3896, 101.6063, 10.2035},
  {33.5754, -37.0143, 13.5757, 0.4588},
  {103.2263, -47.1407, -23.4195, 15.7584},
  {23.8281, 6.1643, -24.5466, 11.3172},
  {235.8469, -281.1150, 118.3817, -4.3864},
  {40.5478, -47.2771, 19.6754, -1.2988}
};

vmc_leg_t right;

extern INS_t INS;
extern vmc_leg_t left;
																
chassis_t chassis_move;
float jump_time;
extern float jump_time2;

																
PidTypedef LegR_Pid;		// 右腿的腿长pd
PidTypedef Tp_Pid;			// 防劈叉补偿pd
PidTypedef Turn_Pid;		// 转向pd
PidTypedef Roll_Pid;		// 横滚角补偿pd

PidTypedef Spin_LegR_Pid;	// 起立右腿腿长pd
PidTypedef Spin_AngleR_Pid;	// 起立旋转角度pd

uint32_t CHASSR_TIME=1;

float spin_LegR_pid[3] = {170, 0, 3000};
float spin_angleR_pid[3] = {30, 0, 3000};
float spin_angle_ref_R;					// 旋转角度目标
float spin_angle_fdb_R;					// 旋转角度反馈
float spin_angle_init_R;				// 初始旋转角度
float spin_angle_end = 5.5; 			// 最终旋转角度
float spin_Leg = 0.3f;					// 旋转长度目标

Ramp_Typedef Ramp_R;

extern uint8_t Spin_legL_finish_flag;	//左腿旋转完成标志位
uint8_t Spin_legR_finish_flag = 0;      //右腿旋转完成标志位

void ChassisR_task(void)
{
	while(INS.ins_flag==0)
	{//等待加速度收敛
	  osDelay(1);	
	}
	
	ChassisR_init(&chassis_move,&right,&LegR_Pid);//初始化右边两个关节电机和右边轮毂电机的id和控制模式、初始化腿部
	Pensation_init(&Roll_Pid,&Tp_Pid,&Turn_Pid);//补偿pid初始化


	while(1)
	{	
		
		PID_init(&Spin_LegR_Pid,PID_POSITION,spin_LegR_pid,20,0);
		PID_init(&Spin_AngleR_Pid,PID_POSITION,spin_angleR_pid,20,0);
		#if SAVE_ZERO 
//		if(remote_ctrl.rc.last_s[1] == 2 && remote_ctrl.rc.s[1] == 3)
//		{
//			save_motor_zero(&hfdcan1,0x02,0x00);
//			save_motor_zero(&hfdcan2,0x03,0x00);
//			osDelay(CHASSR_TIME);
//		}
//		if(remote_ctrl.rc.last_s[0] == 2 && remote_ctrl.rc.s[0] == 3)
//		{
//			save_motor_zero(&hfdcan1,0x01,0x00);
//			save_motor_zero(&hfdcan2,0x04,0x00);
//			osDelay(CHASSR_TIME);
//		}
		disable_motor_mode(&hfdcan1,0x01,0x00);
		osDelay(CHASSR_TIME);
		disable_motor_mode(&hfdcan1,0x02,0x00);
		osDelay(CHASSR_TIME);
		disable_motor_mode(&hfdcan2,0x03,0x00);
		osDelay(CHASSR_TIME);
		disable_motor_mode(&hfdcan2,0x04,0x00);
		osDelay(CHASSR_TIME);
		#else
		chassisR_feedback_update(&chassis_move,&right,&INS);//更新数据
		
		chassisR_control_loop(&chassis_move,&right,&INS,LQR_K_R,&LegR_Pid);//控制计算
		if(chassis_move.start_flag==1&&remote_ctrl.rc.s[1] != 2)	
		{
			mit_ctrl(&hfdcan1,0x02, 0.0f, 0.0f,0.0f, 0.0f,right.torque_set[1]);//right.torque_set[1]
			osDelay(CHASSR_TIME);
			mit_ctrl(&hfdcan1,0x01, 0.0f, 0.0f,0.0f, 0.0f,right.torque_set[0]);//right.torque_set[0]
			osDelay(CHASSR_TIME);
			DJI_Motor_ctrl(DJI_Chassis_Wheel_Motor,&hfdcan1,CHASSR_TIME);
//			osDelay(CHASSR_TIME);
		}
		else if(chassis_move.start_flag==1)	
		{			
			mit_ctrl(&hfdcan1,0x02, 0.0f, 0.0f,0.0f, 0.0f,right.torque_set[1]);//right.torque_set[1]
			osDelay(CHASSR_TIME);
			mit_ctrl(&hfdcan1,0x01, 0.0f, 0.0f,0.0f, 0.0f,right.torque_set[0]);//right.torque_set[0]
			osDelay(CHASSR_TIME);
			DJI_Motor_ctrl(DJI_Chassis_Wheel_Motor,&hfdcan1,CHASSR_TIME);

		}
		else if(chassis_move.start_flag==0)	
		{
			mit_ctrl(&hfdcan1,0x02, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right.torque_set[0]
			osDelay(CHASSR_TIME);
			mit_ctrl(&hfdcan1,0x01, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right.torque_set[1]
			osDelay(CHASSR_TIME);
			
			DJI_Chassis_Wheel_Motor[0].Data.SET_Current = 0;
			DJI_Chassis_Wheel_Motor[1].Data.SET_Current = 0;
			DJI_Motor_ctrl(DJI_Chassis_Wheel_Motor,&hfdcan1,CHASSR_TIME);
//			osDelay(CHASSR_TIME);
		}
		#endif
		vofa_data[0] = right.theta;
		vofa_data[1] = right.d_theta;
		vofa_data[2] = chassis_move.x_set;
		vofa_data[3] = chassis_move.x_filter;
		vofa_data[4] = chassis_move.v_set;
		vofa_data[5] = chassis_move.v_filter;
		vofa_data[6] = chassis_move.myPithR;
		vofa_data[7] = chassis_move.myPithL;
		vofa_data[8] = chassis_move.myPithGyroR;
		vofa_data[9] = chassis_move.myPithGyroL;		
		Vofa_JustFloat_send(vofa_data,10);
	}
}

void ChassisR_init(chassis_t *chassis,vmc_leg_t *vmc,PidTypedef *legr)
{
  const static float legr_pid[3] = {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};

	joint_motor_init(&chassis->joint_motor[0],1,MIT_MODE);//发送id为6
	joint_motor_init(&chassis->joint_motor[1],2,MIT_MODE);//发送id为8
	
	//wheel_motor_init(&chassis->wheel_motor[0],1,MIT_MODE);//发送id为1
	
	VMC_init(vmc);//给杆长赋值
	
	PID_init(legr, PID_POSITION,legr_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);//腿长pid

	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hfdcan1,chassis->joint_motor[1].para.id,chassis->joint_motor[1].mode);
	  osDelay(1);
	}
	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hfdcan1,chassis->joint_motor[0].para.id,chassis->joint_motor[0].mode);
	  osDelay(1);
	}

//	for(int j=0;j<10;j++)
//	{
//    enable_motor_mode(&hfdcan1,chassis->wheel_motor[0].para.id,chassis->wheel_motor[0].mode);//右边轮毂电机
//	  osDelay(1);
//	}
}

void Pensation_init(PidTypedef *roll,PidTypedef *Tp,PidTypedef *turn)
{//补偿pid初始化：横滚角补偿、防劈叉补偿、偏航角补偿
	const static float roll_pid[3] = {ROLL_PID_KP, ROLL_PID_KI,ROLL_PID_KD};
	const static float tp_pid[3] = {TP_PID_KP, TP_PID_KI, TP_PID_KD};
	const static float turn_pid[3] = {TURN_PID_KP, TURN_PID_KI, TURN_PID_KD};

	
	PID_init(roll, PID_POSITION, roll_pid, ROLL_PID_MAX_OUT, ROLL_PID_MAX_IOUT);
	PID_init(Tp, PID_POSITION, tp_pid, TP_PID_MAX_OUT,TP_PID_MAX_IOUT);
	PID_init(turn, PID_POSITION, turn_pid, TURN_PID_MAX_OUT, TURN_PID_MAX_IOUT);

}

void chassisR_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins)
{
	vmc->phi1=pi/2.0f+chassis->joint_motor[0].para.pos;
	vmc->phi4=pi/2.0f+chassis->joint_motor[1].para.pos;	
	chassis->myPithR=ins->Pitch;
	chassis->myPithGyroR=ins->Gyro[0];
	
	chassis->total_yaw=ins->YawTotalAngle;
	chassis->roll=ins->Roll;
	chassis->theta_err=0.0f-(vmc->theta+left.theta);
	/*
	if(ins->Pitch<(3.1415926f/6.0f)&&ins->Pitch>(-3.1415926f/6.0f))
	{//根据pitch角度判断倒地自起是否完成
		chassis->recover_flag=0;
	}*/
}

uint8_t right_flag=0;
uint8_t left_flag;
//float mg=21.0f;
//float mg=102.9f/2; //无配重
float mg = 114.66/2;
float Leg_mg = 20.9f; 		// 全腿重量
float sLeg_mg = 16.71f; 	// 小腿重量
void chassisR_control_loop(chassis_t *chassis,vmc_leg_t *vmcr,INS_t *ins,float *LQR_K,PidTypedef *leg)
{
	VMC_calc_1_right(vmcr,ins,((float)CHASSR_TIME)*3.0f/1000.0f);//计算theta和d_theta给lqr用，同时也计算右腿长L0,该任务控制周期是3*0.001秒
	
	for(int i=0;i<12;i++)
	{
		LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcr->L0 );	
	}
		
	chassis->turn_T=PID_Calc(&Turn_Pid, chassis->total_yaw, chassis->turn_set);//yaw轴pid计算
	chassis->turn_T=Turn_Pid.Kp*(chassis->turn_set-chassis->total_yaw)-Turn_Pid.Kd*ins->Gyro[2];//这样计算更稳一点

//	chassis->roll_f0=Roll_Pid.Kp*(chassis->roll_set-chassis->roll)-Roll_Pid.Kd*ins->Gyro[1];
	
	mySaturate(&chassis->roll_f0,-Roll_Pid.max_out,Roll_Pid.max_out);
	
	chassis->leg_tp=PID_Calc(&Tp_Pid, chassis->theta_err,0.0f);//防劈叉pid计算
	
	if(Spin_legL_finish_flag == 0 || Spin_legR_finish_flag == 0)
	{
		spin_angle_fdb_R = vmcr->alpha;
		if(spin_angle_fdb_R >= 0) spin_angle_fdb_R = spin_angle_fdb_R - 2*pi;
		spin_angle_fdb_R = -spin_angle_fdb_R;
		
		if(spin_angle_fdb_R <= 0.9) Spin_legR_finish_flag = 1;
		static uint8_t updateR_flag = 0;
		
		if(remote_ctrl.rc.last_s[0] == 2 && remote_ctrl.rc.s[0] == 3)
		{
			updateR_flag = 1;
		}
		else updateR_flag = 0;
		
		if(remote_ctrl.rc.s[0] == 3) spin_angle_ref_R = ramp_update(&Ramp_R,spin_angle_init_R,spin_angle_end,4000,updateR_flag);
		else if(remote_ctrl.rc.s[0] == 2)
		{
			spin_angle_ref_R  = spin_angle_fdb_R;
			spin_angle_init_R = spin_angle_fdb_R;
		}

		vmcr->Tp = Leg_mg*vmcr->L0/2*arm_sin_f32(spin_angle_fdb_R) + PID_Calc(&Spin_AngleR_Pid,spin_angle_fdb_R,spin_angle_ref_R);
		vmcr->F0 = -sLeg_mg*arm_cos_f32(spin_angle_fdb_R) + PID_Calc(&Spin_LegR_Pid,vmcr->L0,spin_Leg);//前馈+pd
		chassis->wheel_motor[0].Data.SET_Current = 0;
		
		if(fabs(spin_angle_fdb_R - spin_angle_end) < 0.35) 
		{
			spin_Leg = 0.13;
			if(fabs(spin_Leg - vmcr->L0) < 0.02) Spin_legR_finish_flag = 1;
		}
		else spin_Leg = 0.3;
		
	}
	else if(Spin_legL_finish_flag == 1 && Spin_legR_finish_flag == 1)
	{
		if(remote_ctrl.rc.s[0] == 2) Spin_legR_finish_flag = 0;
		
		chassis->wheel_motor[0].wheel_T=(LQR_K[0]*(vmcr->theta-0.0f)
										+LQR_K[1]*(vmcr->d_theta-0.0f)
										+LQR_K[2]*(chassis->x_filter-chassis->x_set)
										+LQR_K[3]*(chassis->v_filter-chassis->v_set)
										+LQR_K[4]*(chassis->myPithR-0.04f-chassis->phi_set)
										+LQR_K[5]*(chassis->myPithGyroR-0.0f));
	
		//右边髋关节输出力矩				
		vmcr->Tp=	(LQR_K[6]*(vmcr->theta-0.0f)
					+LQR_K[7]*(vmcr->d_theta-0.0f)
					+LQR_K[8]*(chassis->x_filter-chassis->x_set)
					+LQR_K[9]*(chassis->v_filter-chassis->v_set)
					+LQR_K[10]*(chassis->myPithR+0.f-chassis->phi_set)
					+LQR_K[11]*(chassis->myPithGyroR-0.0f));
	
		vmcr->Tp=vmcr->Tp+chassis->leg_tp-0.0f;//髋关节输出力矩
		
		chassis->wheel_motor[0].wheel_T=chassis->wheel_motor[0].wheel_T-chassis->turn_T;	//轮毂电机输出力矩
		mySaturate(&chassis->wheel_motor[0].wheel_T,-4.8f,4.8f);	
		
		chassis->wheel_motor[0].Data.SET_Current = (int16_t)(chassis->wheel_motor[0].wheel_T*3330.0f);
		VAL_LIMIT(chassis->wheel_motor[0].Data.SET_Current,-16384,16384);
		
		DJI_Chassis_Wheel_Motor[0].Data.SET_Current = -chassis->wheel_motor[0].Data.SET_Current;

		if(chassis->jump_flag1==1||chassis->jump_flag1==2||chassis->jump_flag1==3)
		{
			if(chassis->jump_flag1==1)
			{//压缩阶段
				vmcr->F0=mg/arm_cos_f32(vmcr->theta)+PID_Calc(leg,vmcr->L0,0.04f);//前馈+pd

				if(vmcr->L0<0.08f)
				{
					jump_time++;
				}
				if(jump_time>=10&&jump_time2>=10)
				{  
					jump_time=0;
					jump_time2=0;
					chassis->jump_flag1=2;//压缩完毕进入上升加速阶段
					chassis->jump_flag2=2;//压缩完毕进入上升加速阶段
					chassis->jump_f = 0;
				}			 
			}
			else if(chassis->jump_flag1==2)
			{//上升加速阶段			
				vmcr->F0=mg/arm_cos_f32(vmcr->theta)+PID_Calc(leg,vmcr->L0,0.19f);//前馈+pd
		
				if(vmcr->L0>0.16f)
				{
					jump_time++;
				}
				if(jump_time>=2&&jump_time2>=2)
				{  
				 	chassis->jump_f++;
					jump_time=0;
					jump_time2=0;
					chassis->jump_flag1=3;//上升完毕进入缩腿阶段
					chassis->jump_flag2=3;
				}	 
			}
			else if(chassis->jump_flag1==3)
			{//缩腿阶段
				vmcr->F0=PID_Calc(leg,vmcr->L0,0.10f);//pd
				chassis->theta_set=0.0f;
				if(vmcr->L0<0.15f)
				{
					jump_time++;
				}
				if(jump_time>=3&&jump_time2>=3)
				{ 
					jump_time=0;
					jump_time2=0;
					chassis->leg_set_r=0.10f;
					chassis->last_leg_set=0.10f;
					chassis->jump_flag1=0;//缩腿完毕
					chassis->jump_flag2=0;			
				}
			}
		}	
		else
		{
			vmcr->F0=mg/arm_cos_f32(vmcr->theta)+PID_Calc(leg,vmcr->L0,chassis->leg_set_r);//前馈+pd
		// 	vmcr->F0=PID_Calc(leg,vmcr->L0,chassis->leg_set_r);//前馈+pd
		}
		
		right_flag=ground_detectionR(vmcr,ins);//右腿离地检测
		right_flag = 0;
		if(chassis->recover_flag==0)		
		{
			//倒地自起不需要检测是否离地	 
			if((right_flag==1&&left_flag==1&&vmcr->leg_flag==0&&chassis->jump_flag1!=1&&chassis->jump_flag2!=1&&chassis->jump_flag1!=2&&chassis->jump_flag2!=2)
				||chassis->jump_flag1==3)
			{
				//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
				//排除跳跃的压缩阶段、上升阶段、跳跃的缩腿阶段
				chassis->wheel_motor[0].wheel_T=0.0f;
				vmcr->Tp=LQR_K[6]*(vmcr->theta-0.0f)+ LQR_K[7]*(vmcr->d_theta-0.0f);
	
				chassis->x_filter=0.0f;
				chassis->x_set=chassis->x_filter;
				vmcr->Tp=vmcr->Tp+chassis->leg_tp;			 
			}
			else
			{//没有离地
				vmcr->leg_flag=0;//置为0
				/*				
				if(chassis->jump_flag1==0)
				{//不跳跃的时候需要roll轴补偿						
				vmcr->F0=vmcr->F0+chassis->roll_f0;//roll轴补偿取反然后加上去    			
				}
				*/
			}
		}
		else if(chassis->recover_flag==1)
		{
			vmcr->Tp=0.0f;
			chassis->wheel_motor[0].wheel_T = 0.0f;
			vmcr->F0=PID_Calc(leg,vmcr->L0,0.055f);//pd
		}

		mySaturate(&vmcr->F0,-150.0f,150.0f);//限幅 
	}	
	VMC_calc_2(vmcr);//计算期望的关节输出力矩

	if(chassis->jump_flag1==1||chassis->jump_flag1==2||chassis->jump_flag1==3)
	{//跳跃的时候需要更大扭矩
		mySaturate(&vmcr->torque_set[1],-20.0f,20.0f);	
		mySaturate(&vmcr->torque_set[0],-20.0f,20.0f);	
	}	                                
	else                              
	{//不跳跃的时候最大为额定扭矩     
		mySaturate(&vmcr->torque_set[1],-20.0f,20.0f);	
		mySaturate(&vmcr->torque_set[0],-20.0f,20.0f);	
	}
}

void mySaturate(float *in,float min,float max)
{
  if(*in < min)
  {
    *in = min;
  }
  else if(*in > max)
  {
    *in = max;
  }
}




