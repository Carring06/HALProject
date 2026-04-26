#include "Manipulator_Task.h"

/*达妙电机内部集成了PID控制算法，因此不需要额外的控制算法，如若需求高精度，自行加入算法控制 */
float joint_pos[5] = {};
float joint_vel[5] = {};

// 机械臂控制任务（仅按键按下才使能电机，无多余操作）
void ManipulatorTask(void)
{
    // 初始化关节目标位置，换句话说，也就是起始位置（仅初始化变量，无使能操作）
    joint_pos[0] = 0.0f;
    joint_pos[1] = -2.0f;
    joint_pos[2] = 0.0f;
    joint_pos[3] = -3.14;
    joint_pos[4] = 0.0f;
    // 初始化关节速度（仅初始化变量，无使能操作）
    joint_vel[0] = 5.0f;
    joint_vel[1] = 5.0f;
    joint_vel[2] = 5.0f;
    joint_vel[3] = 3.0f;
    joint_vel[4] = 3.0f;
    /* 任务主循环 */
    while (1)
    {
        // 仅当遥控器S1=1（按键按下）时，才使能所有机械臂电机
        if (remote_ctrl.rc.s[0] == 1)
        {
            Enable_Motor_Mode(&hfdcan2, Manipulator_J4310_Motor_Base_Rotate_TxID, POS_MODE, 1);

            Enable_Motor_Mode(&hfdcan2, Manipulator_J4310_Motor_Base_TxID, POS_MODE, 1);
            Enable_Motor_Mode(&hfdcan2, Manipulator_J4340_Motor_Jonit_TxID, POS_MODE, 1);
            Enable_Motor_Mode(&hfdcan2, Manipulator_J4310_Motor_Rotate_TxID, POS_MODE, 1);
            Enable_Motor_Mode(&hfdcan2, Manipulator_J4310_Motor_Gripper_TxID, POS_MODE, 1);
        }

        // S1=2时，失能所有机械臂电机
        if (remote_ctrl.rc.s[0] == 2)
        {
            Disable_Motor_Mode(&hfdcan2, Manipulator_J4310_Motor_Base_Rotate_TxID, POS_MODE, 1);

            Disable_Motor_Mode(&hfdcan2, Manipulator_J4310_Motor_Base_TxID, POS_MODE, 1);
            Disable_Motor_Mode(&hfdcan2, Manipulator_J4340_Motor_Jonit_TxID, POS_MODE, 1);
            Disable_Motor_Mode(&hfdcan2, Manipulator_J4310_Motor_Rotate_TxID , POS_MODE, 1);
			Disable_Motor_Mode(&hfdcan2, Manipulator_J4310_Motor_Gripper_TxID, POS_MODE, 1);
        }

        // S1=3时，仅电机已使能（按键先按过）才会响应位置指令
        if (remote_ctrl.rc.s[0] == 3)
        {

            // 摇杆修改目标位置
            joint_pos[0] += ((float)remote_ctrl.rc.ch[0] / 660) * (-0.008f);

            joint_pos[1] += ((float)remote_ctrl.rc.ch[1] / 660) * (-0.008f);
            joint_pos[2] += ((float)remote_ctrl.rc.ch[2] / 660) * (-0.008f);
            joint_pos[3] += ((float)remote_ctrl.rc.ch[3] / 660) * (-0.008f);
            joint_pos[4] += ((float)remote_ctrl.rc.ch[4] / 660) * (-0.008f);

            /*当遥控器要兼容除了机械臂的其他电机时打开注释，互换程序*/
            // joint_pos[0] += ((float)remote_ctrl.rc.ch[4] / 660) * (-0.008f);
            // joint_pos[1] += ((float)remote_ctrl.rc.ch[0] / 660) * (-0.008f);
            // joint_pos[2] += ((float)remote_ctrl.rc.ch[1] / 660) * (-0.008f);
            // joint_pos[3] += ((float)remote_ctrl.rc.ch[2] / 660) * (-0.008f);
            // joint_pos[4] += ((float)remote_ctrl.rc.ch[3] / 660) * (-0.008f);

            // 角度限位
            VAL_LIMIT(joint_pos[0], -2.0f, 2.0f);
            VAL_LIMIT(joint_pos[1], -2.0f, -0.1f);
            VAL_LIMIT(joint_pos[2], 0.0f, 2.5f);
            VAL_LIMIT(joint_pos[3], -3.14f, 3.14f);
            VAL_LIMIT(joint_pos[4], 0.0f, 3.14f);

            // 发送位置指令（仅按键先使能后，电机才会执行）
            DM_Motor_Ctrl(&hfdcan2, &Arm_DM_Motor[0], joint_pos[0], joint_vel[0], 0, 0, 0, 1); 

            DM_Motor_Ctrl(&hfdcan2, &Arm_DM_Motor[1], joint_pos[1], joint_vel[1], 0, 0, 0, 1);
            DM_Motor_Ctrl(&hfdcan2, &Arm_DM_Motor[2], joint_pos[2], joint_vel[2], 0, 0, 0, 1);
            DM_Motor_Ctrl(&hfdcan2, &Arm_DM_Motor[3], joint_pos[3], joint_vel[3], 0, 0, 0, 1);
            DM_Motor_Ctrl(&hfdcan2, &Arm_DM_Motor[4], joint_pos[4], joint_vel[4], 0, 0, 0, 1);
        }

        osDelay(1); // 任务周期1ms
    }
}
