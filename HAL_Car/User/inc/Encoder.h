#ifndef __ENCODER_H
#define __ENCODER_H

void Encoder_Init(void);
int16_t Encoder_GetM1(void);
int16_t Encoder_GetM2(void);
int16_t Encoder_GetM1Pos(void);
int16_t Encoder_GetM2Pos(void);
float Encoder_GetM1Angle(void);
float Encoder_GetM2Angle(void);

#endif
