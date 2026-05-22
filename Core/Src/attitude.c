#include "attitude.h"
#include "MPU9250_AHRS.h"
#include "MPU9250_Init.h"
//这里不用#include<math.h>,开销太大了，直接写个近似的asinf函数
#include "math_approx.h"

static uint8_t mpu_buffer[22];

static float accel[3];
static float gyro[3];
static float mag[3];

static float accel_bias[3];
static float accel_scale[3];
static float gyro_bias[3];

static uint8_t credible_of_accel = 0;
static uint8_t is_mag_calib_enabled = 0;

static float quat_to_pitch(float q[4])
{
    float q0 = q[0];
    float q1 = q[1];
    float q2 = q[2];
    float q3 = q[3];

    float s = 2.0f * (q0 * q2 - q1 * q3);

    if (s > 1.0f) s = 1.0f;
    if (s < -1.0f) s = -1.0f;

    return asinf(s);
}

void Attitude_Init(void)
{
    is_mag_calib_enabled = 0;

    if(MPU9250_Init() != HAL_OK)
    {
        Error_Handler();
    }

    Get_Data_From_Flash(accel_bias, accel_scale, is_mag_calib_enabled);

    MPU9250_Calibrate_Gyro(gyro_bias);// 进行陀螺仪标定，获取陀螺仪的零偏值
}

int Attitude_Update(float dt, Attitude_t *attitude)
{
    HAL_StatusTypeDef status;

    status = MPU9250_Get_original_Data(mpu_buffer, accel, gyro, mag);

    if (status == HAL_BUSY)
    {
        return 0;
    }

    if (status != HAL_OK)
    {
        return -1;
    }

    MPU9250_Get_Calibrated_Data(gyro, gyro_bias, accel, accel_bias, accel_scale, &credible_of_accel, mag);

    MPU9250_Mahony_Fusion(dt, gyro,accel,mag, credible_of_accel, is_mag_calib_enabled, 2.0f, 0.0f);

    attitude->pitch = quat_to_pitch(output_quaternion);
    attitude->gyro_y = gyro[1];

    return 1;
}