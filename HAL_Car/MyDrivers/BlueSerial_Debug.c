#include "BlueSerial_Debug.h"

void BlueSerial_DebugSpeedPID(void)
{
    if (Serial_RxFlag == 1)
    {
        // OLED_ShowString(0, 0, "                       ", OLED_6X8);
        // OLED_ShowString(0, 0, Serial_RxPacket, OLED_6X8);
        // OLED_Update();

        // strtok strcmp atoi/atof

        char *Tag = strtok(Serial_RxPacket, ",");
        if (strcmp(Tag, "key") == 0)
        {
            char *Name = strtok(NULL, ",");
            char *Action = strtok(NULL, ",");
        }
        else if (strcmp(Tag, "slider") == 0)
        {
            char *Name = strtok(NULL, ",");
            char *Value = strtok(NULL, ",");

            if (strcmp(Name, "M1Kp") == 0) /*Name == "M1Kp"——字符串不可这么比较，要用strcmp*/
            {
                Car_SpeedM1PID.Kp = atof(Value);Car_SpeedM2PID.Kp = atof(Value);
            }
            else if (strcmp(Name, "M1Kd") == 0)
            {
                Car_SpeedM1PID.Kd = atof(Value);Car_SpeedM2PID.Kd = atof(Value);
            }
            else if (strcmp(Name, "M1Ki") == 0)
            {
                Car_SpeedM1PID.Ki = atof(Value);Car_SpeedM2PID.Ki = atof(Value);
            }
            else if (strcmp(Name, "M2Kp") == 0)
            {
                
            }
            else if (strcmp(Name, "M2Kd") == 0)
            {
                
            }
            else if (strcmp(Name, "M2Ki") == 0)
            {
                
            }
            else if (strcmp(Name, "target") == 0)
            {
                Car_SpeedM1PID.Target = atof(Value);
                Car_SpeedM2PID.Target = atof(Value);
            }
        }
        else if (strcmp(Tag, "joystick") == 0)
        {
            int8_t LH = atoi(strtok(NULL, ","));
            int8_t LV = atoi(strtok(NULL, ","));
            int8_t RH = atoi(strtok(NULL, ","));
            int8_t RV = atoi(strtok(NULL, ","));

            BLE_Serial_Printf("joystick,%d,%d,%d,%d\r\n", LH, LV, RH, RV);
        }

        Serial_RxFlag = 0;
    }
}

void BlueSerial_DebugAnglePID(void)
{
    if (Serial_RxFlag == 1)
    {

        // strtok strcmp atoi/atof

        char *Tag = strtok(Serial_RxPacket, ",");

        if (strcmp(Tag, "slider") == 0)
        {
            char *Name = strtok(NULL, ",");
            char *Value = strtok(NULL, ",");

            if (strcmp(Name, "Angle1Kp") == 0) /*Name == "M1Kp"——字符串不可这么比较，要用strcmp*/
            {
                Car_UprightM1PID.Kp = atof(Value);
                Car_UprightM2PID.Kp = atof(Value);
            }
            else if (strcmp(Name, "Angle1Kd") == 0)
            {
                Car_UprightM1PID.Kd = atof(Value);
                Car_UprightM2PID.Kd = atof(Value);
            }
            else if (strcmp(Name, "Angle1Ki") == 0)
            {
                Car_UprightM1PID.Ki = atof(Value);
                Car_UprightM2PID.Ki = atof(Value);
            }
            //             else if (strcmp(Name, "Angle2Kp") == 0) /*Name == "M1Kp"——字符串不可这么比较，要用strcmp*/
            //            {
            // Car_UprightM2PID.Kp = atof(Value);
            //            }
            //            else if (strcmp(Name, "Angle2Kd") == 0)
            //            {
            // Car_UprightM2PID.Kd = atof(Value);
            //            }
            //            else if (strcmp(Name, "Angle2Ki") == 0)
            //            {
            // Car_UprightM2PID.Ki = atof(Value);
            //            }
            else if (strcmp(Name, "target") == 0)
            {
                Car_UprightM1PID.Target = atof(Value);
                Car_UprightM2PID.Target = atof(Value);
            }
        }

        Serial_RxFlag = 0;
    }
}

void BlueSerial_DebugPID(void)
{
    if (Serial_RxFlag == 1)
    {

        // strtok strcmp atoi/atof

        char *Tag = strtok(Serial_RxPacket, ",");

        if (strcmp(Tag, "slider") == 0)
        {
            char *Name = strtok(NULL, ",");
            char *Value = strtok(NULL, ",");

					   if (strcmp(Name, "M1Kp") == 0) /*Name == "M1Kp"——字符串不可这么比较，要用strcmp*/
            {
                Car_SpeedM1PID.Kp = atof(Value);
                Car_SpeedM2PID.Kp = atof(Value);
            }
            else if (strcmp(Name, "M1Kd") == 0)
            {
                Car_SpeedM1PID.Kd = atof(Value);
                Car_SpeedM2PID.Kd = atof(Value);
            }
            else if (strcmp(Name, "M1Ki") == 0)
            {
                Car_SpeedM1PID.Ki = atof(Value);
                Car_SpeedM2PID.Ki = atof(Value);
            }
            else if (strcmp(Name, "Angle1Kp") == 0) /*Name == "M1Kp"——字符串不可这么比较，要用strcmp*/
            {
                Car_UprightM1PID.Kp = atof(Value);
                Car_UprightM2PID.Kp = atof(Value);
            }
            else if (strcmp(Name, "Angle1Kd") == 0)
            {
                Car_UprightM1PID.Kd = atof(Value);
                Car_UprightM2PID.Kd = atof(Value);
            }
            else if (strcmp(Name, "Angle1Ki") == 0)
            {
                Car_UprightM1PID.Ki = atof(Value);
                Car_UprightM2PID.Ki = atof(Value);
            }
            //             else if (strcmp(Name, "Angle2Kp") == 0) /*Name == "M1Kp"——字符串不可这么比较，要用strcmp*/
            //            {
            // Car_UprightM2PID.Kp = atof(Value);
            //            }
            //            else if (strcmp(Name, "Angle2Kd") == 0)
            //            {
            // Car_UprightM2PID.Kd = atof(Value);
            //            }
            //            else if (strcmp(Name, "Angle2Ki") == 0)
            //            {
            // Car_UprightM2PID.Ki = atof(Value);
            //            }
            else if (strcmp(Name, "target") == 0)
            {
                Car_UprightM1PID.Target = atof(Value);
                Car_UprightM2PID.Target = atof(Value);
            }
        }

        Serial_RxFlag = 0;
    }
}
