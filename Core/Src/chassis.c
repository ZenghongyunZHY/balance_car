#include "chassis.h"
#include "motor.h"

#define CHASSIS_PWM_MAX 9000

#define PWM_DEAD_ZONE 350
#define PWM_NOISE_BAND 80    // 小于这个认为是噪声，不输出

static void set_one_motor_signed(uint8_t channel, int pwm)
{
    uint8_t dir;// 0: 正转，1: 反转
    uint16_t duty;// PWM 占空比，0-9000

    if (pwm >= 0)
    {
        dir = 0;
        duty = pwm;
    }
    else
    {
        dir = 1;
        duty = -pwm;
    }

    if (duty > CHASSIS_PWM_MAX)
    {
        duty = CHASSIS_PWM_MAX;
    }

    motor_set_pwm(channel, duty, dir);
}

void Chassis_Init(void)
{
    motor_init();
}

static int add_deadzone(int pwm)
{
    int sign;
    int mag;
    if (pwm > 0)
    {
        sign = 1;
        mag = pwm;
    }
    else if (pwm < 0)
    {
        sign = -1;
        mag = -pwm;
    }
    else
    {
        return 0;
    }

    //很小的输出认为是噪声，直接输出 0
    if (mag < PWM_NOISE_BAND)
    {
        return 0;
    }

    int mapped = PWM_DEAD_ZONE + (mag - PWM_NOISE_BAND) * (CHASSIS_PWM_MAX - PWM_DEAD_ZONE) / (CHASSIS_PWM_MAX - PWM_NOISE_BAND);

    if (mapped > CHASSIS_PWM_MAX)
    {
        mapped = CHASSIS_PWM_MAX;
    }
    return sign * mapped;

}

void Chassis_Set_PWM(int left_pwm, int right_pwm)
{
    //left_pwm = add_deadzone(left_pwm);
    //right_pwm = add_deadzone(right_pwm);

    set_one_motor_signed(1, left_pwm);
    set_one_motor_signed(2, right_pwm);
}

void Chassis_Stop(void)
{
    motor_stop();
}

Chassis_Speed_t Chassis_Get_Speed(float dt)
{
    Chassis_Speed_t speed;
    motor_speed raw;

    motor_get_speed_mps_rps(dt, &raw);

    speed.left_mps = raw.speed_mps1;
    speed.right_mps = raw.speed_mps2;

    return speed;
}