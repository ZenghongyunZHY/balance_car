#ifndef __MPU9250_LOWLEVEL_H__
#define __MPU9250_LOWLEVEL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
typedef enum{
  DATA_STATE_IDLE = 0,
  DATA_STATE_READING = 1,
  DATA_STATE_READY = 2,

} DataState;


/**
 * @brief 将 MPU9250 片选拉低，开始一次 SPI 访问。
 */
void MPU9250_CS_Low(void);

/**
 * @brief 将 MPU9250 片选拉高，结束一次 SPI 访问。
 */
void MPU9250_CS_High(void);

/**
 * @brief 向 MPU9250 单个寄存器写入 1 字节。
 * @param reg 寄存器地址（7bit 写地址）。
 * @param value 写入的数据。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef MPU9250_WriteReg(uint8_t reg, uint8_t value);

/**
 * @brief 从 MPU9250 连续读取多个寄存器字节。
 * @param reg 起始寄存器地址（函数内部会自动加读标志位）。
 * @param buffer 读取缓冲区。
 * @param size 读取字节数。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef MPU9250_ReadRegs(uint8_t reg, uint8_t *buffer, uint16_t size);

/**
 * @brief 非阻塞启动 DMA 连续读。
 * @param reg 起始寄存器地址（函数内部会自动加读标志位）。
 * @param buffer 读取缓冲区。
 * @param size 读取字节数。
 * @param timeout_ms 兼容保留参数（当前不在本函数内等待）。
 * @retval HAL_OK 表示 DMA 已成功启动。
 */
HAL_StatusTypeDef MPU9250_ReadRegsDMA(uint8_t reg, uint8_t *buffer, uint16_t size, uint32_t timeout_ms);

/**
 * @brief 轮询 DMA 连续读是否完成（非阻塞）。
 * @param timeout_ms 超时时间（毫秒）。
 * @retval HAL_OK 已完成；HAL_BUSY 进行中；HAL_TIMEOUT 超时；HAL_ERROR 发生错误。
 */
HAL_StatusTypeDef MPU9250_ReadRegsDMA_Poll(uint32_t timeout_ms);

/**
 * @brief 从 MPU9250 读取单个寄存器字节。
 * @param reg 寄存器地址。
 * @param value 读取结果指针。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef MPU9250_ReadReg(uint8_t reg, uint8_t *value);

/**
 * @brief 读取连续 6 字节并解析为 X/Y/Z 三轴 16 位有符号值。
 * @param start_reg 起始寄存器地址（高字节地址）。
 * @param out 长度为 3 的输出数组。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef MPU9250_ReadAxis3(int16_t start_reg, int16_t out[3]);

#ifdef __cplusplus
}
#endif

#endif
