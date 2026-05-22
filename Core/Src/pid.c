#include "pid.h"

static float limit_float(float x, float min, float max)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

void PID_Init(PID_t *pid,float kp,float ki,float kd,float out_min,float out_max,float integral_min,float integral_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->integral = 0.0f;
    pid->last_error = 0.0f;

    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
}


float PID_Update(PID_t *pid, float target, float feedback, float dt)
{
    float error = target - feedback;

    pid->integral += error * dt;
    pid->integral = limit_float(pid->integral,pid->integral_min,pid->integral_max);

    float derivative = (error - pid->last_error) / dt;
    pid->last_error = error;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    return limit_float(output, pid->out_min, pid->out_max);
}

/*
 * 这个函数适合角度环：
 * target - feedback 是角度误差；
 * derivative_feedback 可以直接传 gyro_y；
 * 这样 D 项直接用陀螺仪角速度，不用对角度差分，噪声更小。
 */
float PID_Update_With_D(PID_t *pid,float target,float feedback,float derivative_feedback,float dt)
{
    float error = target - feedback;

    pid->integral += error * dt;
    pid->integral = limit_float(pid->integral, pid->integral_min,pid->integral_max);

    pid->last_error = error;

    float output = pid->kp * error + pid->ki * pid->integral - pid->kd * derivative_feedback;

    return limit_float(output, pid->out_min, pid->out_max);
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}
