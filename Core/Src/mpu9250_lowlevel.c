#include "mpu9250_lowlevel.h"

#include <stdint.h>
#include "spi.h"
#include "gpio.h"

#define MPU9250_SPI_TIMEOUT_MS 50U
#define MPU9250_CS_PORT        GPIOB
#define MPU9250_CS_PIN         GPIO_PIN_0
#define MPU9250_DMA_MAX_XFER   64U

static volatile uint8_t s_spi1_dma_rx_done = 0U;
static volatile uint8_t s_spi1_dma_rx_error = 0U;
static volatile uint8_t s_spi1_dma_in_progress = 0U; // DMA 传输状态标志，0 表示空闲，1 表示进行中
static uint32_t s_spi1_dma_start_tick = 0U;
static uint8_t s_spi1_dma_tx_dummy[MPU9250_DMA_MAX_XFER];


/**
 * @brief 将 MPU9250 片选拉低，开始 SPI 事务。
 */
void MPU9250_CS_Low(void)
{
  HAL_GPIO_WritePin(MPU9250_CS_PORT, MPU9250_CS_PIN, GPIO_PIN_RESET);
}

/**
 * @brief 将 MPU9250 片选拉高，结束 SPI 事务。
 */
void MPU9250_CS_High(void)
{
  HAL_GPIO_WritePin(MPU9250_CS_PORT, MPU9250_CS_PIN, GPIO_PIN_SET);
}

/**
 * @brief 写 MPU9250 单寄存器。
 * @param reg 寄存器地址。
 * @param value 写入值。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef MPU9250_WriteReg(uint8_t reg, uint8_t value)
{
  uint8_t frame[2] = {reg & 0x7FU, value};

  MPU9250_CS_Low();
  HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, frame, sizeof(frame), MPU9250_SPI_TIMEOUT_MS);
  MPU9250_CS_High();

  return status;
}

/**
 * @brief 连续读取 MPU9250 寄存器。
 * @param reg 起始寄存器地址。
 * @param buffer 输出缓冲区。
 * @param size 读取字节数。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef MPU9250_ReadRegs(uint8_t reg, uint8_t *buffer, uint16_t size)
{
  uint8_t reg_addr = reg | 0x80U;

  MPU9250_CS_Low();
  HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, &reg_addr, 1U, MPU9250_SPI_TIMEOUT_MS);
  if (status == HAL_OK)
  {
    status = HAL_SPI_Receive(&hspi1, buffer, size, MPU9250_SPI_TIMEOUT_MS);
  }
  MPU9250_CS_High();

  return status;
}

/**
 * @brief 使用 DMA 连续读取 MPU9250 寄存器。
 * @param reg 起始寄存器地址。
 * @param buffer 输出缓冲区。
 * @param size 读取字节数。
 * @param timeout_ms 超时时间（毫秒）。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef MPU9250_ReadRegsDMA(uint8_t reg, uint8_t *buffer, uint16_t size, uint32_t timeout_ms)
{
  if ((buffer == NULL) || (size == 0U) || (size > MPU9250_DMA_MAX_XFER))
  {
    return HAL_ERROR;
  }

  if (s_spi1_dma_in_progress != 0U)
  {
    return HAL_BUSY;
  }

  (void)timeout_ms;

  uint8_t reg_addr = reg | 0x80U;
  s_spi1_dma_rx_done = 0U;
  s_spi1_dma_rx_error = 0U;
  s_spi1_dma_start_tick = HAL_GetTick();

  MPU9250_CS_Low();

  HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, &reg_addr, 1U, MPU9250_SPI_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    MPU9250_CS_High();
    return status;
  }

  for (uint16_t i = 0; i < size; ++i)
  {
    s_spi1_dma_tx_dummy[i] = 0xFFU;
  }

  status = HAL_SPI_TransmitReceive_DMA(&hspi1, s_spi1_dma_tx_dummy, buffer, size);
  if (status != HAL_OK)
  {
    MPU9250_CS_High();
    return status;
  }

  s_spi1_dma_in_progress = 1U;
  return HAL_OK;
}

// 轮询 DMA 连续读是否完成（非阻塞）。
// 返回 HAL_OK 已完成；HAL_BUSY 进行中；HAL_TIMEOUT 超时；HAL_ERROR 发生错误。
/**
 * @brief Poll the non-blocking DMA register read transaction.
 * @param timeout_ms Transfer timeout in milliseconds.
 * @retval HAL_OK DMA transfer completed.
 * @retval HAL_BUSY DMA transfer is still running.
 * @retval HAL_TIMEOUT DMA transfer timed out.
 * @retval HAL_ERROR No transfer is active or SPI DMA reported an error.
 */
HAL_StatusTypeDef MPU9250_ReadRegsDMA_Poll(uint32_t timeout_ms)
{
  if (s_spi1_dma_in_progress == 0U)
  {
    return HAL_ERROR;
  }

  if (s_spi1_dma_rx_error != 0U)
  {
    (void)HAL_SPI_DMAStop(&hspi1);
    MPU9250_CS_High();
    s_spi1_dma_in_progress = 0U;
    return HAL_ERROR;
  }

  if (s_spi1_dma_rx_done != 0U)
  {
    MPU9250_CS_High();
    s_spi1_dma_in_progress = 0U;
    return HAL_OK;
  }

  if ((HAL_GetTick() - s_spi1_dma_start_tick) >= timeout_ms)
  {
    (void)HAL_SPI_DMAStop(&hspi1);
    MPU9250_CS_High();
    s_spi1_dma_in_progress = 0U;
    return HAL_TIMEOUT;
  }

  return HAL_BUSY;
}

/**
 * @brief 读取 MPU9250 单寄存器。
 * @param reg 寄存器地址。
 * @param value 输出值指针。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef MPU9250_ReadReg(uint8_t reg, uint8_t *value)
{
  return MPU9250_ReadRegs(reg, value, 1U);
}

/**
 * @brief 读取 X/Y/Z 三轴原始值。
 * @param start_reg 起始寄存器地址（高字节地址）。
 * @param out 三轴输出数组。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef MPU9250_ReadAxis3(int16_t start_reg, int16_t out[3])
{
  uint8_t raw[6];
  HAL_StatusTypeDef status = MPU9250_ReadRegs((uint8_t)start_reg, raw, sizeof(raw));
  if (status != HAL_OK)
  {
    return status;
  }

  out[0] = (int16_t)((raw[0] << 8U) | raw[1]);
  out[1] = (int16_t)((raw[2] << 8U) | raw[3]);
  out[2] = (int16_t)((raw[4] << 8U) | raw[5]);

  return HAL_OK;
}

/**
 * @brief SPI1 DMA 接收完成回调。
 * @param hspi SPI 句柄。
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if ((hspi != NULL) && (hspi->Instance == SPI1&&(s_spi1_dma_in_progress != 0U)))
  {
    s_spi1_dma_rx_done = 1U;
  }
}

/**
 * @brief SPI1 DMA 收发完成回调。
 * @param hspi SPI 句柄。
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if ((hspi != NULL) && (hspi->Instance == SPI1&&(s_spi1_dma_in_progress != 0U)))
  {
    s_spi1_dma_rx_done = 1U;
  }
}

/**
 * @brief SPI1 错误回调。
 * @param hspi SPI 句柄。
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if ((hspi != NULL) && (hspi->Instance == SPI1&&(s_spi1_dma_in_progress != 0U)))
  {
    s_spi1_dma_rx_error = 1U;
    s_spi1_dma_rx_done = 1U;
  }
}
