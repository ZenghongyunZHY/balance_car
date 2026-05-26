#include "attitude.h"
#include "MPU9250_Init.h"
#include "mpu9250_lowlevel.h"
#include "mpu9250_reg.h"

#include <math.h>
#include <stdint.h>

#define ATTITUDE_IMU_FRAME_SIZE         14U
#define ATTITUDE_DMA_TIMEOUT_MS         20U

#define ATTITUDE_PI                     3.14159265358979323846f
#define ATTITUDE_GRAVITY_MPS2           9.80665f

/* MPU9250_Init.c configures +/-4g accel and +/-500dps gyro. */
#define ATTITUDE_ACCEL_LSB_TO_MPS2      (ATTITUDE_GRAVITY_MPS2 / 8192.0f)
#define ATTITUDE_GYRO_LSB_TO_RADPS      ((ATTITUDE_PI / 180.0f) / 65.5f)

#define ATTITUDE_COMPLEMENTARY_ALPHA    0.98f
#define ATTITUDE_ACCEL_TRUST_RANGE      2.0f

#define ATTITUDE_DT_DEFAULT_S           0.005f
#define ATTITUDE_DT_MIN_S               0.001f
#define ATTITUDE_DT_MAX_S               0.05f

#define ATTITUDE_GYRO_BIAS_SAMPLES      300U
#define ATTITUDE_GYRO_BIAS_DELAY_MS     2U

static uint8_t mpu_buffer[ATTITUDE_IMU_FRAME_SIZE];
static uint8_t imu_dma_started = 0U;

static float pitch_rad = 0.0f;
static float gyro_y_bias = 0.0f;

static int16_t parse_i16_be(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

static float limit_dt(float dt_s)
{
    if (dt_s < ATTITUDE_DT_MIN_S)
    {
        return ATTITUDE_DT_DEFAULT_S;
    }

    if (dt_s > ATTITUDE_DT_MAX_S)
    {
        return ATTITUDE_DT_MAX_S;
    }

    return dt_s;
}

static void parse_imu_frame(const uint8_t *buffer, float accel[3], float gyro[3])
{
    int16_t accel_raw[3];
    int16_t gyro_raw[3];

    accel_raw[0] = parse_i16_be(&buffer[0]);
    accel_raw[1] = parse_i16_be(&buffer[2]);
    accel_raw[2] = parse_i16_be(&buffer[4]);

    gyro_raw[0] = parse_i16_be(&buffer[8]);
    gyro_raw[1] = parse_i16_be(&buffer[10]);
    gyro_raw[2] = parse_i16_be(&buffer[12]);

    for (uint8_t i = 0U; i < 3U; ++i)
    {
        accel[i] = (float)accel_raw[i] * ATTITUDE_ACCEL_LSB_TO_MPS2;
        gyro[i] = (float)gyro_raw[i] * ATTITUDE_GYRO_LSB_TO_RADPS;
    }
}

static HAL_StatusTypeDef read_imu_frame_dma(uint8_t *buffer)
{
    if (imu_dma_started == 0U)
    {
        HAL_StatusTypeDef status = MPU9250_ReadRegsDMA(MPU9250_REG_ACCEL_XOUT_H,
                                                       buffer,
                                                       ATTITUDE_IMU_FRAME_SIZE,
                                                       ATTITUDE_DMA_TIMEOUT_MS);
        if (status != HAL_OK)
        {
            return status;
        }

        imu_dma_started = 1U;
        return HAL_BUSY;
    }

    HAL_StatusTypeDef status = MPU9250_ReadRegsDMA_Poll(ATTITUDE_DMA_TIMEOUT_MS);
    if (status == HAL_BUSY)
    {
        return HAL_BUSY;
    }

    imu_dma_started = 0U;
    return status;
}

static uint8_t accel_is_trusted(const float accel[3])
{
    float norm = sqrtf((accel[0] * accel[0]) +
                       (accel[1] * accel[1]) +
                       (accel[2] * accel[2]));

    return (fabsf(norm - ATTITUDE_GRAVITY_MPS2) <= ATTITUDE_ACCEL_TRUST_RANGE) ? 1U : 0U;
}

static HAL_StatusTypeDef calibrate_gyro_y(void)
{
    uint8_t buffer[6];
    int32_t gyro_y_sum = 0;

    for (uint16_t i = 0U; i < ATTITUDE_GYRO_BIAS_SAMPLES; ++i)
    {
        HAL_StatusTypeDef status = MPU9250_ReadRegs(MPU9250_REG_GYRO_XOUT_H,
                                                    buffer,
                                                    sizeof(buffer));
        if (status != HAL_OK)
        {
            return status;
        }

        gyro_y_sum += parse_i16_be(&buffer[2]);
        HAL_Delay(ATTITUDE_GYRO_BIAS_DELAY_MS);
    }

    gyro_y_bias = ((float)gyro_y_sum / (float)ATTITUDE_GYRO_BIAS_SAMPLES) *
                  ATTITUDE_GYRO_LSB_TO_RADPS;

    return HAL_OK;
}

static void init_pitch_from_accel(void)
{
    uint8_t buffer[ATTITUDE_IMU_FRAME_SIZE];
    float accel[3];
    float gyro[3];

    if (MPU9250_ReadRegs(MPU9250_REG_ACCEL_XOUT_H, buffer, sizeof(buffer)) != HAL_OK)
    {
        pitch_rad = 0.0f;
        return;
    }

    parse_imu_frame(buffer, accel, gyro);
    if (accel_is_trusted(accel) != 0U)
    {
        pitch_rad = atan2f(-accel[0], accel[2]);
    }
    else
    {
        pitch_rad = 0.0f;
    }
}

void Attitude_Init(void)
{
    imu_dma_started = 0U;
    pitch_rad = 0.0f;
    gyro_y_bias = 0.0f;

    if (MPU9250_Init() != HAL_OK)
    {
        Error_Handler();
    }

    if (calibrate_gyro_y() != HAL_OK)
    {
        Error_Handler();
    }

    init_pitch_from_accel();
}

int Attitude_Update(float dt, Attitude_t *attitude)
{
    float accel[3];
    float gyro[3];

    if (attitude == NULL)
    {
        return -1;
    }

    HAL_StatusTypeDef status = read_imu_frame_dma(mpu_buffer);
    if (status == HAL_BUSY)
    {
        return 0;
    }

    if (status != HAL_OK)
    {
        return -1;
    }

    parse_imu_frame(mpu_buffer, accel, gyro);

    float dt_s = limit_dt(dt);
    float gyro_y = gyro[1] - gyro_y_bias;
    float pitch_gyro = pitch_rad + (gyro_y * dt_s);
    float pitch_next = pitch_gyro;

    if (accel_is_trusted(accel) != 0U)
    {
        float pitch_acc = atan2f(-accel[0], accel[2]);
        pitch_next = (ATTITUDE_COMPLEMENTARY_ALPHA * pitch_gyro) +
                     ((1.0f - ATTITUDE_COMPLEMENTARY_ALPHA) * pitch_acc);
    }

    pitch_rad = pitch_next;

    attitude->pitch = pitch_rad;
    attitude->gyro_y = gyro_y;

    return 1;
}
