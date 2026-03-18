/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "pid.h"
#include "user_lib.h"
#define DEBUG 1
void PID_init(PidTypedef *pid, uint8_t mode, const float PID[3], float max_out, float max_iout)
{
	if (pid == NULL || PID == NULL)
	{
		return;
	}
	#if DEBUG
		pid->Kp = PID[0];
		pid->Ki = PID[1];
		pid->Kd = PID[2];
	#endif
	if(pid->Initlized != true)
	{
		pid->pid_mode = mode;
		pid->Kp = PID[0];
		pid->Ki = PID[1];
		pid->Kd = PID[2];
		pid->max_out = max_out;
		pid->max_iout = max_iout;
		pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
		pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
		pid->Initlized = true;
	}
}


float PID_Calc(PidTypedef *pid, float fdb, float ref)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->ref = ref;
    pid->fdb = fdb;
    pid->error[0] = ref - fdb;
	if(pid->Initlized == true)
	{
		switch(pid->pid_mode)
		{
			case PID_POSITION:
			{
				pid->Pout = pid->Kp * pid->error[0];
				pid->Iout += pid->Ki * pid->error[0];
				pid->Dbuf[2] = pid->Dbuf[1];
				pid->Dbuf[1] = pid->Dbuf[0];
				pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
				pid->Dout = pid->Kd * pid->Dbuf[0];
				VAL_LIMIT(pid->Iout, -pid->max_iout, pid->max_iout);
				pid->out = pid->Pout + pid->Iout + pid->Dout;
				VAL_LIMIT(pid->out, -pid->max_out, pid->max_out);
			}break;
			
			case PID_DELTA:
			{
				pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
				pid->Iout = pid->Ki * pid->error[0];
				pid->Dbuf[2] = pid->Dbuf[1];
				pid->Dbuf[1] = pid->Dbuf[0];
				pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
				pid->Dout = pid->Kd * pid->Dbuf[0];
				pid->out += pid->Pout + pid->Iout + pid->Dout;
				VAL_LIMIT(pid->out, -pid->max_out, pid->max_out);
			}break;
		}
	}
	else
	{
	    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
		pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
		pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
		pid->fdb = pid->ref = 0.0f;	
	}
    return pid->out;
}

void PID_clear(PidTypedef *pid)
{
    if (pid == NULL)
    {
        return;
    }
	pid->Initlized = false;
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->ref = 0.0f;
}

