#include "balance_controller.h"
#include "pid.h"

#define PWM_MAX             9000.0f
#define TARGET_PITCH_MAX    0.20f     // 大约 11.5 度，后面可调小
#define DIFF_PWM_MAX        3000.0f

static PID_t speed_pid;
static PID_t angle_pid;
static PID_t turn_pid;

static float limit_float(float x, float min, float max)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

void Balance_Controller_Init(void)
{
    /*
     * 初始参数只是能跑框架，不是最终参数。
     * 后面主要调 angle_pid。
     */
    PID_Init(&speed_pid, 0.8f, 0.1f, 0.0f, -TARGET_PITCH_MAX, TARGET_PITCH_MAX, -0.5f, 0.5f);

    PID_Init(&angle_pid, 5000.0f, 0.0f, 50.0f, -PWM_MAX, PWM_MAX, -0.2f, 0.2f);

    PID_Init(&turn_pid, 1200.0f, 0.0f, 0.0f, -DIFF_PWM_MAX, DIFF_PWM_MAX, -0.5f, 0.5f);
}

Balance_Output_t Balance_Controller_Update(Balance_Target_t target, Balance_Feedback_t feedback, float dt)
{
    Balance_Output_t out;

    float speed_avg = 0.5f * (feedback.speed_left + feedback.speed_right);// 当前平均速度
    float speed_diff = feedback.speed_right - feedback.speed_left;// 当前差速

    /*
     * 速度环：
     * 输入：目标速度、当前平均速度
     * 输出：目标俯仰角
     */
    float target_pitch = PID_Update(&speed_pid, target.target_speed, speed_avg, dt);

    target_pitch = limit_float(target_pitch,  -TARGET_PITCH_MAX, TARGET_PITCH_MAX);

    /*
     * 角度环：
     * 输入：目标俯仰角、当前俯仰角、gyro_y
     * 输出：平均 PWM
     */
    float average_pwm = PID_Update_With_D(&angle_pid, target_pitch, feedback.pitch, feedback.gyro_y, dt);

    /*
     * 转向环：
     * 输入：目标差速、当前差速
     * 输出：差分 PWM
     */
    float diff_pwm = PID_Update(&turn_pid, target.target_turn, speed_diff, dt);

    out.left_pwm  = (int)(average_pwm - diff_pwm);
    out.right_pwm = (int)(average_pwm + diff_pwm);

    if (out.left_pwm > PWM_MAX) out.left_pwm = PWM_MAX;
    if (out.left_pwm < -PWM_MAX) out.left_pwm = -PWM_MAX;

    if (out.right_pwm > PWM_MAX) out.right_pwm = PWM_MAX;
    if (out.right_pwm < -PWM_MAX) out.right_pwm = -PWM_MAX;

    return out;
}