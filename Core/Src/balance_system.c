#include "balance_system.h"
#include "attitude.h"
#include "chassis.h"
#include "balance_controller.h"
#include "tim.h"

//姿态解算的dt
static uint16_t ahrs_last_counter = 0;
static uint16_t ahrs_new_counter = 0;
//控制算法的dt
static uint16_t control_last_counter = 0;
static uint16_t control_new_counter = 0;

static float ahrs_dt_s;
static float control_dt_s;

static Attitude_t attitude;
static Chassis_Speed_t chassis_speed;
static Balance_Feedback_t feedback;
static Balance_Target_t target;
static Balance_Output_t output;

void balance_system_init(void)
{
    Attitude_Init();
    Chassis_Init();
    Balance_Controller_Init();

    target.target_speed = 0.0f;
    target.target_turn = 0.0f;
}
static float counter_to_time(uint16_t new_counter, uint16_t last_counter)
{
    uint16_t counter_diff = new_counter - last_counter;

    //一个counter是1us
    return (float)counter_diff / 1000000.0f;
}

void balance_system_run(Balance_Target_t new_target,uint8_t *is_error,uint8_t *is_first_run)
{
    target = new_target;

    ahrs_new_counter = __HAL_TIM_GET_COUNTER(&htim1);
    control_new_counter = ahrs_new_counter; // 使用相同的计数器来计算控制算法的 dt

    if(*is_first_run)
    {
        // 第一次运行时，初始化 last_counter 并设置 is_first_run 标志为 0
        ahrs_last_counter = ahrs_new_counter;
        control_last_counter = control_new_counter;
        *is_first_run = 0;
        return;
    }

    ahrs_dt_s = counter_to_time(ahrs_new_counter, ahrs_last_counter);

    int ret = Attitude_Update(ahrs_dt_s, &attitude);

    if (ret == 0)
    {
        // Attitude_Update 正在读取传感器数据，跳过这次控制
        return;
    }
    else if (ret == -1)
    {
        // Attitude_Update 读取传感器数据失败，可能是通信错误,停止底盘并返回
        Chassis_Stop();
        *is_error = 1;
        return;
    }
    else if (ret == 1)
    {
        ahrs_last_counter = ahrs_new_counter;
    }
    control_dt_s = counter_to_time(control_new_counter, control_last_counter);
    control_last_counter = control_new_counter;
    chassis_speed = Chassis_Get_Speed(control_dt_s);

    feedback.pitch = attitude.pitch;
    feedback.gyro_y = attitude.gyro_y;
    feedback.speed_left = chassis_speed.left_mps;
    feedback.speed_right = chassis_speed.right_mps;

    output = Balance_Controller_Update(target, feedback, control_dt_s);
    Chassis_Set_PWM(output.left_pwm, output.right_pwm);

}
