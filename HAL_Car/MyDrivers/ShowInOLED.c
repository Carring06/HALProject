#include "Header.h"

float angle;
extern int16_t AX, AY, AZ, GX, GY, GZ;

void OLED_ShowPIDOut(void)
{
    angle = -Angle;
    OLED_Printf(0, 0, OLED_8X16, "M1Out:%.3f", Car_UprightM1PID.Out);
    OLED_Printf(0, 16, OLED_8X16, "M2Out:%.3f", Car_UprightM2PID.Out);
    OLED_Printf(0, 32, OLED_8X16, "Angle:%.3f", angle);
    OLED_Update();

    // OLED_Printf(0, 0, OLED_8X16, "M1Out:%.3f", Car_SpeedM1PID.Out);
    // OLED_Printf(0, 16, OLED_8X16, "M2Out:%.3f", Car_SpeedM2PID.Out);
    // OLED_Update();
}

void OLED_ShowAccGyro(void)
{
    // MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);

    OLED_Printf(0, 0, OLED_8X16, "%+06d", AX);
    OLED_Printf(0, 16, OLED_8X16, "%+06d", AY);
    OLED_Printf(0, 32, OLED_8X16, "%+06d", AZ);
    OLED_Printf(64, 0, OLED_8X16, "%+06d", GX);
    OLED_Printf(64, 16, OLED_8X16, "%+06d", GY);
    OLED_Printf(64, 32, OLED_8X16, "%+06d", GZ);

    OLED_Printf(0, 48, OLED_8X16, "Acc");
    OLED_Printf(64, 48, OLED_8X16, "Gyro");

    OLED_Update();
}
