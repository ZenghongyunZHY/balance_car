#include "balance_system.h"
#include "attitude.h"
#include "chassis.h"
#include "balance_controller.h"
#include "tim.h"

static uint32_t last_counter = 0;
static uint32_t new_counter = 0;

static float dt_s;

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
static float counter_to_time(uint32_t new_counter, uint32_t last_counter)
{
    uint32_t counter_diff = new_counter - last_counter;

    //一个counter是1us
    return (float)counter_diff / 1000000.0f;
}

void balance_system_run(Balance_Target_t new_target,uint8_t *is_error,uint8_t *is_first_run)
{
    target = new_target;

    new_counter = __HAL_TIM_GET_COUNTER(&htim1);

    if(*is_first_run)
    {
        // 第一次运行时，初始化 last_counter 并设置 is_first_run 标志为 0
        last_counter = new_counter;
        *is_first_run = 0;
        return;
    }

    dt_s = counter_to_time(new_counter, last_counter);
    last_counter = new_counter;

    int ret = Attitude_Update(dt_s, &attitude);

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
        // Attitude_Update 成功更新姿态数据，继续执行控制算法
        chassis_speed = Chassis_Get_Speed(dt_s);

        feedback.pitch = attitude.pitch;
        feedback.gyro_y = attitude.gyro_y;
        feedback.speed_left = chassis_speed.left_mps;
        feedback.speed_right = chassis_speed.right_mps;

        output = Balance_Controller_Update(target, feedback, dt_s);
        Chassis_Set_PWM(output.left_pwm, output.right_pwm);
    }
}
