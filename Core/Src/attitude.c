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
#define ATTITUDE_DT_MAX_S               0.05f

#define ATTITUDE_GYRO_BIAS_SAMPLES      300U
#define ATTITUDE_GYRO_BIAS_DELAY_MS     2U

static uint8_t mpu_buffer[ATTITUDE_IMU_FRAME_SIZE];
static uint8_t imu_dma_started = 0U;

static float pitch_rad = 0.0f;
static float pitch_zero = 0.0f;
static float gyro_y_bias = 0.0f;

volatile float dbg_ax = 0.0f;
volatile float dbg_ay = 0.0f;
volatile float dbg_az = 0.0f;
volatile float dbg_acc_norm = 0.0f;
volatile uint8_t dbg_acc_trusted = 0U;
volatile float dbg_dt_in = 0.0f;
volatile float dbg_dt_used = 0.0f;
volatile float dbg_pitch_acc_raw = 0.0f;
volatile float dbg_pitch_acc_zeroed = 0.0f;
volatile float dbg_pitch_zero = 0.0f;
volatile float dbg_gyro_y = 0.0f;

static int16_t parse_i16_be(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

static float wrap_pi(float x)
{
    while (x > ATTITUDE_PI)
    {
        x -= 2.0f * ATTITUDE_PI;
    }

    while (x < -ATTITUDE_PI)
    {
        x += 2.0f * ATTITUDE_PI;
    }

    return x;
}

static float limit_dt(float dt_s)
{
    if (dt_s <= 0.0f)
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

static void start_next_imu_frame_dma(void)
{
    if (MPU9250_ReadRegsDMA(MPU9250_REG_ACCEL_XOUT_H,
                            mpu_buffer,
                            ATTITUDE_IMU_FRAME_SIZE,
                            ATTITUDE_DMA_TIMEOUT_MS) == HAL_OK)
    {
        imu_dma_started = 1U;
    }
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
        pitch_zero = 0.0f;
        pitch_rad = 0.0f;
        dbg_pitch_zero = pitch_zero;
        return;
    }

    parse_imu_frame(buffer, accel, gyro);
    if (accel_is_trusted(accel) != 0U)
    {
        pitch_zero = atan2f(-accel[0], accel[2]);
        pitch_rad = 0.0f;
    }
    else
    {
        pitch_zero = 0.0f;
        pitch_rad = 0.0f;
    }

    dbg_pitch_zero = pitch_zero;
}

void Attitude_Init(void)
{
    imu_dma_started = 0U;
    pitch_rad = 0.0f;
    pitch_zero = 0.0f;
    gyro_y_bias = 0.0f;
    dbg_pitch_zero = pitch_zero;

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

    dbg_dt_in = dt;
    float dt_s = limit_dt(dt);
    dbg_dt_used = dt_s;

    dbg_ax = accel[0];
    dbg_ay = accel[1];
    dbg_az = accel[2];
    dbg_acc_norm = sqrtf((accel[0] * accel[0]) +
                         (accel[1] * accel[1]) +
                         (accel[2] * accel[2]));
    dbg_acc_trusted = accel_is_trusted(accel);

    float gyro_y = gyro[1] - gyro_y_bias;
    dbg_gyro_y = gyro_y;
    dbg_pitch_zero = pitch_zero;
    dbg_pitch_acc_raw = atan2f(-accel[0], accel[2]);
    dbg_pitch_acc_zeroed = wrap_pi(dbg_pitch_acc_raw - pitch_zero);
    float pitch_acc = dbg_pitch_acc_zeroed;

    /*
     * 如果实车调试确认 pitch 符号与车体方向相反，在这里整体取反 pitch 通道：
     * gyro_y = -gyro_y;
     * pitch_acc = -pitch_acc;
     */
    float pitch_gyro = wrap_pi(pitch_rad + (gyro_y * dt_s));
    float pitch_next = pitch_gyro;

    if (dbg_acc_trusted != 0U)
    {
        pitch_next = (ATTITUDE_COMPLEMENTARY_ALPHA * pitch_gyro) +
                     ((1.0f - ATTITUDE_COMPLEMENTARY_ALPHA) * pitch_acc);
    }

    pitch_rad = wrap_pi(pitch_next);
    start_next_imu_frame_dma();

    attitude->pitch = pitch_rad;
    attitude->gyro_y = gyro_y;

    return 1;
}
