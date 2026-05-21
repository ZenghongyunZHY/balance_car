#include "MPU9250_Init.h"
#include "mpu9250_lowlevel.h"
#include "mpu9250_reg.h"

/**
 * @brief I2C_SLV4 单次事务完成轮询超时（毫秒）。
 */
#define MPU9250_I2C_TIMEOUT 20U

/**
 * @brief 通过 MPU9250 的 I2C_SLV4 阻塞写 AK8963 寄存器。
 * @param reg AK8963 寄存器地址。
 * @param value 待写入数据。
 * @retval HAL_OK 写入成功。
 * @retval HAL_TIMEOUT 在超时窗口内未检测到 I2C_SLV4_DONE。
 * @retval 其他 HAL 状态码 底层 SPI 读写失败。
 */
HAL_StatusTypeDef MPU9250_AK8963_WriteReg(uint8_t reg, uint8_t value)
{
  HAL_StatusTypeDef status;

  // 步骤1：配置 SLV4 目标地址为 AK8963（写）。
  status = MPU9250_WriteReg(MPU9250_REG_I2C_SLV4_ADDR, AK8963_I2C_ADDR);
  if (status != HAL_OK) return status;

  // 步骤2：写入目标寄存器地址。
  status = MPU9250_WriteReg(MPU9250_REG_I2C_SLV4_REG, reg);
  if (status != HAL_OK) return status;

  // 步骤3：装载写入数据。
  status = MPU9250_WriteReg(MPU9250_REG_I2C_SLV4_DO, value);
  if (status != HAL_OK) return status;

  // 步骤4：触发一次 SLV4 事务。
  status = MPU9250_WriteReg(MPU9250_REG_I2C_SLV4_CTRL, 0x80U);
  if (status != HAL_OK) return status;

  // 步骤5：阻塞轮询事务完成位。
  for (uint8_t i = 0; i < MPU9250_I2C_TIMEOUT; ++i)
  {
    uint8_t mst_status = 0U;
    status = MPU9250_ReadReg(MPU9250_REG_I2C_MST_STATUS, &mst_status);
    if (status != HAL_OK) return status;

    if ((mst_status & 0x40U) != 0U) { /* I2C_SLV4_DONE */
      return HAL_OK;
    }
    HAL_Delay(1);
  }
  return HAL_TIMEOUT;
}

/**
 * @brief 通过 MPU9250 的 I2C_SLV4 阻塞读 AK8963 寄存器。
 * @param reg AK8963 寄存器地址。
 * @param value 读回数据输出指针。
 * @retval HAL_OK 读取成功。
 * @retval HAL_ERROR 参数非法（value 为空）。
 * @retval HAL_TIMEOUT 在超时窗口内未检测到 I2C_SLV4_DONE。
 * @retval 其他 HAL 状态码 底层 SPI 读写失败。
 */
HAL_StatusTypeDef MPU9250_AK8963_ReadReg(uint8_t reg, uint8_t *value)
{
  if (value == NULL) return HAL_ERROR;

  HAL_StatusTypeDef status;

  // 步骤1：配置 SLV4 目标地址为 AK8963（读）。
  status = MPU9250_WriteReg(MPU9250_REG_I2C_SLV4_ADDR, (uint8_t)(0x80U | AK8963_I2C_ADDR));
  if (status != HAL_OK) return status;

  // 步骤2：写入待读取寄存器地址。
  status = MPU9250_WriteReg(MPU9250_REG_I2C_SLV4_REG, reg);
  if (status != HAL_OK) return status;

  // 步骤3：触发一次 SLV4 读事务。
  status = MPU9250_WriteReg(MPU9250_REG_I2C_SLV4_CTRL, 0x80U);
  if (status != HAL_OK) return status;

  // 步骤4：阻塞轮询完成位，完成后读取 I2C_SLV4_DI。
  for (uint8_t i = 0; i < MPU9250_I2C_TIMEOUT; ++i)
  {
    uint8_t mst_status = 0U;
    status = MPU9250_ReadReg(MPU9250_REG_I2C_MST_STATUS, &mst_status);
    if (status != HAL_OK) return status;

    if ((mst_status & 0x40U) != 0U) {
      return MPU9250_ReadReg(MPU9250_REG_I2C_SLV4_DI, value);
    }
    HAL_Delay(1);
  }
  return HAL_TIMEOUT;
}

/**
 * @brief 配置 MPU9250 主 IMU：电源、时钟、采样率、量程与低通滤波。
 * @retval HAL 状态码。
 */
static HAL_StatusTypeDef mpu9250_config_main_imu(void)
{
  HAL_StatusTypeDef status;

  // 步骤1：器件复位，清空上电残留状态。
  status = MPU9250_WriteReg(MPU9250_REG_PWR_MGMT_1, 0x80U);
  if (status != HAL_OK) return status;
  HAL_Delay(100);

  // 步骤2：选择 PLL 作为时钟源（常用配置，稳定性优于内部振荡）。
  status = MPU9250_WriteReg(MPU9250_REG_PWR_MGMT_1, 0x01U);
  if (status != HAL_OK) return status;

  // 步骤3：使能陀螺与加速度三轴。
  status = MPU9250_WriteReg(MPU9250_REG_PWR_MGMT_2, 0x00U);
  if (status != HAL_OK) return status;

  // 步骤4：配置陀螺 DLPF 为约 41Hz，抑制高频噪声。
  status = MPU9250_WriteReg(MPU9250_REG_CONFIG, 0x03U);
  if (status != HAL_OK) return status;

  // 步骤5：设置采样分频，输出采样率约 200Hz。
  status = MPU9250_WriteReg(MPU9250_REG_SMPLRT_DIV, 0x04U);
  if (status != HAL_OK) return status;

  // 步骤6：陀螺量程设为 ±500dps。
  status = MPU9250_WriteReg(MPU9250_REG_GYRO_CONFIG, 0x08U);
  if (status != HAL_OK) return status;

  // 步骤7：加速度量程设为 ±4g。
  status = MPU9250_WriteReg(MPU9250_REG_ACCEL_CONFIG, 0x08U);
  if (status != HAL_OK) return status;

  // 步骤8：加速度 DLPF 设为约 41Hz。
  status = MPU9250_WriteReg(MPU9250_REG_ACCEL_CONFIG2, 0x03U);
  if (status != HAL_OK) return status;

  return HAL_OK;
}

/**
 * @brief 配置 AK8963：复位、读取 ASA、切换到 16bit/100Hz 连续测量。
 * @retval HAL 状态码。
 */
static HAL_StatusTypeDef mpu9250_config_ak8963(void)
{
  HAL_StatusTypeDef status;

  // 步骤1：进入 Power-down，允许模式切换。
  status = MPU9250_AK8963_WriteReg(AK8963_REG_CNTL1, 0x00U);
  if (status != HAL_OK) return status;
  HAL_Delay(10);

  // 步骤2：软复位磁力计。
  status = MPU9250_AK8963_WriteReg(AK8963_REG_CNTL2, 0x01U);
  if (status != HAL_OK) return status;
  HAL_Delay(10);

  // 步骤3：进入 Fuse ROM 访问模式读取灵敏度修正系数 ASA。
  status = MPU9250_AK8963_WriteReg(AK8963_REG_CNTL1, 0x0FU);
  if (status != HAL_OK) return status;
  HAL_Delay(10);

  // 步骤4：读取 ASAX/ASAY/ASAZ（当前仅读取，后续可用于磁力计尺度补偿）。
  uint8_t asa[3];
  status = MPU9250_AK8963_ReadReg(AK8963_REG_ASAX, &asa[0]);
  if (status != HAL_OK) return status;
  status = MPU9250_AK8963_ReadReg(AK8963_REG_ASAX + 1U, &asa[1]);
  if (status != HAL_OK) return status;
  status = MPU9250_AK8963_ReadReg(AK8963_REG_ASAX + 2U, &asa[2]);
  if (status != HAL_OK) return status;
  (void)asa;

  // 步骤5：退出 Fuse ROM 模式，先回到 Power-down。
  status = MPU9250_AK8963_WriteReg(AK8963_REG_CNTL1, 0x00U);
  if (status != HAL_OK) return status;
  HAL_Delay(10);

  // 步骤6：配置为连续测量模式2（100Hz）+ 16bit 输出。
  status = MPU9250_AK8963_WriteReg(AK8963_REG_CNTL1, 0x16U);
  if (status != HAL_OK) return status;
  HAL_Delay(10);

  return HAL_OK;
}

/**
 * @brief 配置 I2C_SLV0 周期性抓取 AK8963 数据到 EXT_SENS_DATA。
 * @retval HAL 状态码。
 */
static HAL_StatusTypeDef mpu9250_config_slv0_mag_read(void)
{
  HAL_StatusTypeDef status;

  // 步骤1：SLV0 地址设为 AK8963 并置读标志。
  status = MPU9250_WriteReg(MPU9250_REG_I2C_SLV0_ADDR, (uint8_t)(0x80U | AK8963_I2C_ADDR));
  if (status != HAL_OK) return status;

  // 步骤2：从 ST1 起始读取，确保包含数据就绪标志。
  status = MPU9250_WriteReg(MPU9250_REG_I2C_SLV0_REG, AK8963_REG_ST1);
  if (status != HAL_OK) return status;

  // 步骤3：每周期读取 8 字节：ST1 + HXL..HZH + ST2。
  status = MPU9250_WriteReg(MPU9250_REG_I2C_SLV0_CTRL, 0x88U);
  if (status != HAL_OK) return status;

  return HAL_OK;
}

/**
 * @brief 配置 FIFO 流：把 accel/gyro/temp 及 SLV0 外部传感器数据写入 FIFO。
 * @retval HAL 状态码。
 */
static HAL_StatusTypeDef mpu9250_config_fifo_stream(void)
{
  HAL_StatusTypeDef status;

  // 步骤1：复位前先关闭 FIFO 源写入。
  status = MPU9250_WriteReg(MPU9250_REG_FIFO_EN, 0x00U);
  if (status != HAL_OK) return status;

  // 步骤2：在 USER_CTRL 中发出 FIFO 复位命令，同时保持 I2C Master 工作。
  status = MPU9250_WriteReg(MPU9250_REG_USER_CTRL, 
                            (uint8_t)(MPU9250_USER_CTRL_I2C_MST_EN | MPU9250_USER_CTRL_I2C_IF_DIS | 0x04U)); // FIFO_RST=1
  if (status != HAL_OK) return status;

  // 步骤3：使能 FIFO 数据源：TEMP/GYRO/ACCEL + SLV0 外部传感器数据。
  // 0xF8: TEMP_OUT + GYRO_X/Y/Z + ACCEL
  // 0x01: SLV0 EXT_SENS_DATA
  status = MPU9250_WriteReg(MPU9250_REG_FIFO_EN, 0xF9U);
  if (status != HAL_OK) return status;

  // 步骤4：在 USER_CTRL 打开 FIFO_EN，总线开始持续写入 FIFO。
  status = MPU9250_WriteReg(MPU9250_REG_USER_CTRL, 
                            (uint8_t)(MPU9250_USER_CTRL_I2C_MST_EN | MPU9250_USER_CTRL_I2C_IF_DIS | MPU9250_USER_CTRL_FIFO_EN));
  if (status != HAL_OK) return status;

  return HAL_OK;
}

/**
 * @brief MPU9250 初始化入口（阻塞式）：主 IMU + AK8963 + SLV0 + FIFO。
 * @retval HAL 状态码。
 */
HAL_StatusTypeDef MPU9250_Init(void)
{
  // 步骤1：先完成主 IMU 的电源、时钟、滤波和量程配置。
  HAL_StatusTypeDef status = mpu9250_config_main_imu();
  if (status != HAL_OK) return status;

  // 步骤2：设置 MPU9250 内部 I2C Master 时钟（约 400kHz）。
  status = MPU9250_WriteReg(MPU9250_REG_I2C_MST_CTRL, 0x0DU);
  if (status != HAL_OK) return status;

  // 步骤3：使能 I2C Master，关闭 I2C 从设备接口。
  status = MPU9250_WriteReg(MPU9250_REG_USER_CTRL,
                            (uint8_t)(MPU9250_USER_CTRL_I2C_MST_EN | MPU9250_USER_CTRL_I2C_IF_DIS));
  if (status != HAL_OK) return status;

  // 步骤4：初始化并启动 AK8963 连续测量。
  status = mpu9250_config_ak8963();
  if (status != HAL_OK) return status;

  // 步骤5：配置 SLV0 自动搬运磁力计数据。
  status = mpu9250_config_slv0_mag_read();
  if (status != HAL_OK) return status;

  // 步骤6：配置并开启 FIFO 数据流。
  status = mpu9250_config_fifo_stream();
  if (status != HAL_OK) return status;

  return HAL_OK;
}
