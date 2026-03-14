#include "Motor_driver.h"

/*C 语言中，全局 / 静态结构体的初始化值必须是编译期常量，不能用一个变量（哪怕是全局变量）来初始化。*/

// float target_speed = 1000.0;

extern float Act_speedM1;
extern float Act_speedM2;

PID_t Car_SpeedM1PID = {

    .Kp = 0.0,
    .Ki = 0.0,
    .Kd = 0.0,
    .Target = 0.0, /*.Target = target_speed(×)——>C 语言中，全局 / 静态结构体的初始化值必须是编译期常量，不能用一个变量（哪怕是全局变量）来初始化。*/

    .OutMax = 10.0,
    .OutMin = -10.0,

    .OutOffset = 0.0,

};
PID_t Car_SpeedM2PID = {

    .Kp = 0.0,
    .Ki = 0.0,
    .Kd = 0.0,
    .Target = 0.0, /*.Target = target_speed(×)——>C 语言中，全局 / 静态结构体的初始化值必须是编译期常量，不能用一个变量（哪怕是全局变量）来初始化。*/

    .OutMax = 10.0,
    .OutMin = -10.0,

    .OutOffset = 0.0,

};

void Car_SpeedM1Driver(void)
{

    /*传实际速度*/

    // 这里读取实际速度可能有误                                
    Car_SpeedM1PID.Actual = Act_speedM1;                                

    /*利用函数进行PID计算*/
    PID_Update(&Car_SpeedM1PID);
    /*输出PID值*/
    Car_UprightM1Driver(Car_SpeedM1PID.Out);
}
void Car_SpeedM2Driver(void)
{
    /*传实际速度*/

    // 这里读取实际速度可能有误
    Car_SpeedM2PID.Actual = Act_speedM2; 

    /*利用函数进行PID计算*/
    PID_Update(&Car_SpeedM2PID);
    /*输出PID值*/
    Car_UprightM2Driver(Car_SpeedM2PID.Out);
}

/*
        🔺🔺🔺左轮右轮都要🔺🔺🔺
        目标角度必定为0°(偏差要在合理范围内——车不倒);
        当前角度为：Angle,直接用(已读取于Complementary_Filter())。
*/
PID_t Car_UprightM1PID = {

    .Kp = 0.00,
    .Ki = 0.00,
    .Kd = 0.00,

    .Target = 0.0,

    .OutMax = 1000.0,
    .OutMin = -1000.0,

    .OutOffset = 50.0,

};
PID_t Car_UprightM2PID = {

    .Kp = 0.00,
    .Ki = 0.00,
    .Kd = 0.00,

    .Target = 0.0,

    .OutMax = 1000.0,
    .OutMin = -1000.0,

    .OutOffset = 50.0

};

/*
    目前存在的问题：
                   (1) 由于PID算法的特性，当目标角度为0°时，PID输出值会一直为0，导致车轮不转动。
                       解决方法：在PID算法中加入一个微小的偏移量，使得PID输出值不为0。
                   (2)电机反应慢，导致车轮转动速度不够快。车子在可以保持直立时（-a ~ 0 ~ +a），轮子跟不上，导致车子自身角度超出可直立的区间。     
                       解决方法：先去确定可直立的角度区间，然后调节电机转速，使得车轮转动速度足够快。
*/
void Car_UprightM1Driver(float ComeInOut)
{
    Car_UprightM1PID.Target = ComeInOut; /*目标角度*/
        /*传实际角度*/
        Car_UprightM1PID.Actual = GetPitchAngle(); /*当前角度*/
    /*利用函数进行PID计算*/
    PID_Update(&Car_UprightM1PID);
    /*输出PID值*/
    // Car_SpeedM1Driver(Car_UprightM1PID.Out);
    MotorM1_Set(Car_UprightM1PID.Out);
}

void Car_UprightM2Driver(float ComeInOut)
{
    Car_UprightM2PID.Target = ComeInOut; /*目标角度*/
    /*传实际角度*/
    Car_UprightM2PID.Actual = GetPitchAngle(); /*当前角度*/
    /*利用函数进行PID计算*/
    PID_Update(&Car_UprightM2PID);
    /*输出PID值*/
    // Car_SpeedM2Driver(Car_UprightM2PID.Out);
    MotorM2_Set(Car_UprightM2PID.Out);
}

// void Car_SpeedM1Driver(void)
// {

//     /*传实际速度*/

//     // 这里读取实际速度可能有误,得追加计算进行转换

//     Car_SpeedM1PID.Actual = Act_speedM1; /*实际速度*/

//     /*利用函数进行PID计算*/
//     PID_Update(&Car_SpeedM1PID);
//     /*输出PID值*/
//     MotorM1_Set(Car_SpeedM1PID.Out);
// }
// void Car_SpeedM2Driver(void)
// {
//     /*传实际速度*/

//     // 这里读取实际速度可能有误

//     Car_SpeedM2PID.Actual = Act_speedM2; /*实际速度*/

//     /*利用函数进行PID计算*/
//     PID_Update(&Car_SpeedM2PID);
//     /*输出PID值*/
//     MotorM2_Set(Car_SpeedM2PID.Out);
// }
