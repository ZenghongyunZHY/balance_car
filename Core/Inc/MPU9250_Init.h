#ifndef __MPU9250_INIT_H__
#define __MPU9250_INIT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "mpu9250_reg.h"

/**
 * @brief MPU9250 及其内部 AK8963 磁力计的完整初始化配置
 * @details
 *  - 唤醒并设置时钟源
 *  - 配置陀螺仪和加速度计量程及滤波
 *  - 使能 I2C Master 以配置和读取 AK8963
 *  - 配置 AK8963 为连续测量模式
 *  - 配置 SLV0 自动读取磁力计数据
 *  - 配置 FIFO 并启用
 * @retval HAL 状态码
 */
HAL_StatusTypeDef MPU9250_Init(void);

/**
 * @brief 通过 I2C_SLV4 阻塞写 AK8963 寄存器
 * @param reg 寄存器地址
 * @param value 写入数据
 * @retval HAL 状态码
 */
HAL_StatusTypeDef MPU9250_AK8963_WriteReg(uint8_t reg, uint8_t value);

/**
 * @brief 通过 I2C_SLV4 阻塞读 AK8963 寄存器
 * @param reg 寄存器地址
 * @param value 读取数据指针
 * @retval HAL 状态码
 */
HAL_StatusTypeDef MPU9250_AK8963_ReadReg(uint8_t reg, uint8_t *value);

#ifdef __cplusplus
}
#endif

#endif /* __MPU9250_INIT_H__ */
