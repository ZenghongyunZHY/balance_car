#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float pitch;// 俯仰角，单位弧度
    float gyro_y;// 陀螺仪 y 轴数据，单位 rad/s
} Attitude_t;

extern volatile float dbg_ax;
extern volatile float dbg_ay;
extern volatile float dbg_az;
extern volatile float dbg_acc_norm;
extern volatile uint8_t dbg_acc_trusted;
extern volatile float dbg_dt_in;
extern volatile float dbg_dt_used;
extern volatile float dbg_pitch_acc_raw;
extern volatile float dbg_pitch_acc_zeroed;
extern volatile float dbg_pitch_zero;
extern volatile float dbg_gyro_y;

void Attitude_Init(void);
int Attitude_Update(float dt, Attitude_t *attitude);

#ifdef __cplusplus
}
#endif

#endif
