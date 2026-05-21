#ifndef __MPU9250_AHRS_H__
#define __MPU9250_AHRS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "mpu9250_lowlevel.h"
#include "mpu9250_reg.h"
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <string.h>

// 4g, 500dps, 16bit conversion factors
#define turn_to_m_s2   0.000122f
#define turn_to_rad    (0.000122f * 3.1415926f / 180.0f)
#define turn_to_deg_s  0.015f
#define turn_to_uT     0.15f

// Flash storage
#define ACCEL_CALIB_MAGIC       0xA55A1234U
#define ACCEL_CALIB_FLASH_ADDR  0x0801FC00U

typedef struct
{
    uint32_t magic;
    float accel_bias[3];
    float accel_scale[3];
} AccelCalibData;

typedef struct
{
    // Reserved for accel + magnetometer calibration data.
} AccelMagCalibData;

extern float output_quaternion[4];

HAL_StatusTypeDef MPU9250_Get_original_Data(uint8_t *buffer, float *accel, float *gyro, float *mag);

void MPU9250_Calibrate_Gyro(float *gyro_bias);
void Wait_Turn(void);
void Init_Turn_Pin(void);
HAL_StatusTypeDef Save_Accel_Calib_To_Flash(float *accel_bias, float *accel_scale);
void get_data_suggestion(void);
void MPU9250_Calibrate_Accel(float *accel_bias, float *accel_scale, uint8_t is_mag_calib_enabled);
void MPU9250_Calibrate_Mag(float *mag_bias, float *mag_scale);

void MPU9250_Get_Calibrated_Gyro(float *gyro, float *gyro_bias);
void Get_Data_From_Flash(float *accel_basic, float *accel_scale, uint8_t is_mag_calib_enabled);
void MPU9250_Get_Calibrated_Accel(float *accel, float *accel_bias, float *accel_scale, uint8_t *credible_of_accel);
void MPU9250_Get_Calibrated_Mag(void);
void MPU9250_Get_Calibrated_Data(float *gyro,
                                 float *gyro_bias,
                                 float *accel,
                                 float *accel_bias,
                                 float *accel_scale,
                                 uint8_t *credible_of_accel,
                                 float *mag);

void MPU9250_Only_gyro_Update(float dt, float *gyro);
void MPU9250_Mahony_Fusion(float dt,
                           float *gyro,
                           float *accel,
                           float *mag,
                           int8_t credible_of_accel,
                           uint8_t is_mag_calib_enabled,
                           float kp,
                           float ki);

#ifdef __cplusplus
}
#endif

#endif /* __MPU9250_AHRS_H__ */
