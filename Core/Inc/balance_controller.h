#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float pitch;          // 当前俯仰角，rad
    float gyro_y;         // 当前俯仰角速度，rad/s

    float speed_left;     // 左轮线速度，m/s
    float speed_right;    // 右轮线速度，m/s
} Balance_Feedback_t;

typedef struct
{
    float target_speed;   // 目标前后速度，m/s
    float target_turn;    // 目标转向差速，m/s
} Balance_Target_t;

typedef struct
{
    int left_pwm;
    int right_pwm;
} Balance_Output_t;

void Balance_Controller_Init(void);

Balance_Output_t Balance_Controller_Update(Balance_Target_t target,Balance_Feedback_t feedback,float dt);


#ifdef __cplusplus
}
#endif

#endif // BALANCE_CONTROLLER_H