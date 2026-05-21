#include "motor.h"
#include "main.h"
#include "tim.h"
#include "gpio.h"

static uint16_t last_encoder_count1 = 0;
static uint16_t last_encoder_count2 = 0;
static uint16_t now_encoder_count1 = 0;
static uint16_t now_encoder_count2 = 0;
static float encoder_speed1 = 0;
static float encoder_speed2 = 0;

#define WHEEL_DIAMETER_M      0.06766f
#define WHEEL_CIRCUMFERENCE_M (3.1415926f * WHEEL_DIAMETER_M)

#define ENCODER_PPR           13.0f
#define GEAR_RATIO            30.0f
#define ENCODER_MULTIPLY      4.0f

#define COUNTS_PER_REV        (ENCODER_PPR * GEAR_RATIO * ENCODER_MULTIPLY)


void motor_init(void)
{
    // 启动 TIM3 和 TIM4 的编码器接口
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    // 启动 TIM2 的 PWM 输出
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    // 设置 PWM 占空比为 0，初始状态下电机不转动
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    // 设置电机使能引脚为高电平，启用电机
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    // 设置AIN1/AIN2引脚为低电平，设置电机转动方向
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);  
    // 设置BIN1/BIN2引脚为低电平，设置电机转动方向
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
}

void motor_set_pwm(uint8_t channel, uint16_t duty_cycle, uint8_t direction)
{
    if (channel == 1) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty_cycle);
        if(direction == 0) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);  
        } else {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); 
        }
    } else if (channel == 2) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty_cycle);
        if(direction == 0) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);  
        } else {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); 
        }
    }
}

void motor_stop(void)
{
    // 设置 PWM 占空比为 0，停止电机转动
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    // 设置电机使能引脚为低电平，禁用电机
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
}

void motor_get_encoder_speed(float dt)
{
    // 读取 TIM3 和 TIM4 的计数值，计算电机转速
    now_encoder_count1 = __HAL_TIM_GET_COUNTER(&htim3);
    now_encoder_count2 = __HAL_TIM_GET_COUNTER(&htim4);

    int16_t delta_count1 = (int16_t)(uint16_t)(now_encoder_count1 - last_encoder_count1);
    int16_t delta_count2 = (int16_t)(uint16_t)(now_encoder_count2 - last_encoder_count2);

    // 更新上一次的计数值
    last_encoder_count1 = now_encoder_count1;
    last_encoder_count2 = now_encoder_count2;

    // 计算转速，单位为计数/秒
    encoder_speed1 = (float)delta_count1 / dt;
    encoder_speed2 = (float)delta_count2 / dt;
}

// 将计数/秒转换为线速度，单位为米/秒,以及转速，单位为转/秒
void motor_get_speed_mps_rps(float dt, motor_speed *speed)
{
    motor_get_encoder_speed(dt);
    // 将计数/秒转换为线速度，单位为米/秒
    speed->speed_mps1 = ((float)(encoder_speed1 / COUNTS_PER_REV)) * WHEEL_CIRCUMFERENCE_M;
    speed->speed_mps2 = ((float)(encoder_speed2 / COUNTS_PER_REV)) * WHEEL_CIRCUMFERENCE_M;
    speed->speed_rps1 = (float)encoder_speed1 / COUNTS_PER_REV;
    speed->speed_rps2 = (float)encoder_speed2 / COUNTS_PER_REV;
    speed->speed_rad1 = speed->speed_rps1 * 2.0f * 3.1415926f;
    speed->speed_rad2 = speed->speed_rps2 * 2.0f * 3.1415926f;  
}


