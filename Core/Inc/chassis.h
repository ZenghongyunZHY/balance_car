#ifndef __CHASSIS_H
#define __CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float left_mps;// 左轮速度，单位 m/s
    float right_mps;// 右轮速度，单位 m/s
} Chassis_Speed_t;

void Chassis_Init(void);
void Chassis_Set_PWM(int left_pwm, int right_pwm);
void Chassis_Stop(void);
Chassis_Speed_t Chassis_Get_Speed(float dt);

#ifdef __cplusplus
}
#endif

#endif