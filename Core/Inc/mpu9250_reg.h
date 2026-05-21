#ifndef __MPU9250_REG_H__
#define __MPU9250_REG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define MPU9250_WHO_AM_I_VALUE             0x71U
#define AK8963_I2C_ADDR                    0x0CU

#define MPU9250_REG_SELF_TEST_X_GYRO       0x00U
#define MPU9250_REG_SELF_TEST_Y_GYRO       0x01U
#define MPU9250_REG_SELF_TEST_Z_GYRO       0x02U
#define MPU9250_REG_SELF_TEST_X_ACCEL      0x0DU
#define MPU9250_REG_SELF_TEST_Y_ACCEL      0x0EU
#define MPU9250_REG_SELF_TEST_Z_ACCEL      0x0FU
#define MPU9250_REG_SMPLRT_DIV             0x19U
#define MPU9250_REG_CONFIG                 0x1AU
#define MPU9250_REG_GYRO_CONFIG            0x1BU
#define MPU9250_REG_ACCEL_CONFIG           0x1CU
#define MPU9250_REG_ACCEL_CONFIG2          0x1DU
#define MPU9250_REG_FIFO_EN                0x23U
#define MPU9250_REG_I2C_MST_CTRL           0x24U
#define MPU9250_REG_I2C_SLV0_ADDR          0x25U
#define MPU9250_REG_I2C_SLV0_REG           0x26U
#define MPU9250_REG_I2C_SLV0_CTRL          0x27U
#define MPU9250_REG_I2C_SLV4_ADDR          0x31U
#define MPU9250_REG_I2C_SLV4_REG           0x32U
#define MPU9250_REG_I2C_SLV4_DO            0x33U
#define MPU9250_REG_I2C_SLV4_CTRL          0x34U
#define MPU9250_REG_I2C_SLV4_DI            0x35U
#define MPU9250_REG_I2C_MST_STATUS         0x36U
#define MPU9250_REG_ACCEL_XOUT_H           0x3BU
#define MPU9250_REG_GYRO_XOUT_H            0x43U
#define MPU9250_REG_USER_CTRL              0x6AU
#define MPU9250_REG_PWR_MGMT_1             0x6BU
#define MPU9250_REG_PWR_MGMT_2             0x6CU
#define MPU9250_REG_FIFO_COUNTH            0x72U
#define MPU9250_REG_FIFO_R_W               0x74U
#define MPU9250_REG_WHO_AM_I               0x75U

#define MPU9250_USER_CTRL_FIFO_EN          0x40U
#define MPU9250_USER_CTRL_I2C_MST_EN       0x20U
#define MPU9250_USER_CTRL_I2C_IF_DIS       0x10U
#define MPU9250_USER_CTRL_FIFO_RST         0x04U

#define AK8963_REG_ST1                     0x02U
#define AK8963_REG_HXL                     0x03U
#define AK8963_REG_ASAX                    0x10U
#define AK8963_REG_CNTL1                   0x0AU
#define AK8963_REG_CNTL2                   0x0BU

#define MPU9250_FIFO_FRAME_SIZE            22U
#define MPU9250_FIFO_EN_VALUE              0xF9U

#ifdef __cplusplus
}
#endif

#endif
