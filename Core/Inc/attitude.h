#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float pitch;// 俯仰角，单位弧度
    float gyro_y;// 陀螺仪 y 轴数据，单位 rad/s
} Attitude_t;

void Attitude_Init(void);
int Attitude_Update(float dt, Attitude_t *attitude);

#ifdef __cplusplus
}
#endif

#endif