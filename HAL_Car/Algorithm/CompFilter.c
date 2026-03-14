#include "CompFilter.h"

#define Calc_Time 0.01

/*--------------------------------------------------------------各轴的值*/
extern int16_t AX, AY, AZ, GX, GY, GZ;
/*--------------------------------------------------------------各角度值*/
float AngleAcc;
float AngleGyro;
float Angle;
/*----------------------------------------------互补滤波的 “融合权重系数”*/
float Alpha = 0.28;

void Complementary_Filter(void)
{


    GY -= 6;

    AngleAcc = -atan2(AX, AZ) / 3.14159 * 180;

    AngleGyro = Angle + GY / 32768.0 * 2000 * Calc_Time;

    Angle = Alpha * AngleAcc + (1 - Alpha) * AngleGyro;

}

float GetPitchAngle(void)
{
    return Angle;
}
