#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

//引脚说明:
//PA8 PB15 对应 BIN1/BIN2
//PB13 PB14 对应 AIN1/AIN2
//TIM2 CH1/CH2 分别对应 PWMA 和 PWMB
//PB12 对应电机使能引脚(STBY)
//TIM3 TIM4 分别对应电机1和电机2的编码器输入
typedef struct motor_speed {
    float speed_mps1; // 电机1的线速度，单位为米/秒
    float speed_mps2; // 电机2的线速度，单位为米/秒
    float speed_rps1; // 电机1的转速，单位为转/秒
    float speed_rps2; // 电机2的转速，单位为转/秒
    float speed_rad1; // 电机1的角速度，单位为弧度/秒
    float speed_rad2; // 电机2的角速度，单位为弧度/秒
}motor_speed;

void motor_init(void);
void motor_set_pwm(uint8_t channel, uint16_t duty_cycle, uint8_t direction);
void motor_stop(void);
void motor_get_encoder_speed(float dt);
void motor_get_speed_mps_rps(float dt, motor_speed *speed);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
