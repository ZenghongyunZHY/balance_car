#include "MPU9250_AHRS.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#include <math.h>
#include <sys/types.h>

//这里同意约定 dt 的单位为秒，外部调用时需要确保传入的 dt 是以秒为单位的时间间隔（例如 0.01 表示 10 毫秒），以保持数值稳定和算法正确性。
static uint8_t is_dma_started = 0; // DMA 传输是否已启动的标志

//存储四元数的数组

static float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // 初始四元数，表示无旋转(用于维护)
float output_quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // 输出四元数，供外部使用

//误差积分项，用于 Mahony 算法中的 PI 控制器，帮助补偿陀螺仪的零偏和其他系统误差。

static float exInt = 0, eyInt = 0, ezInt = 0; 

//积分限幅

#define e_KI_MAX 0.5f
#define e_KI_MIN -0.5f


//第一层 ： 传感器读取层

/**
 * @brief 启动或轮询一次 MPU9250 原始加速度、陀螺仪和磁力计 DMA 读取。
 * @param buffer 22 字节原始传感器数据缓冲区。
 * @param accel 输出加速度向量，单位 m/s^2。
 * @param gyro 输出陀螺仪角速度向量，单位 rad/s。
 * @param mag 输出磁力计向量，单位 uT。
 * @retval HAL_OK 数据帧读取并解析完成。
 * @retval HAL_BUSY DMA 传输仍在进行。
 * @retval HAL_ERROR 读取失败。
 * @retval HAL_TIMEOUT DMA 传输超时。
 */
HAL_StatusTypeDef MPU9250_Get_original_Data(uint8_t *buffer,float *accel, float *gyro,float *mag)
{
    //从 MPU9250 的加速度计 XOUT_H 寄存器开始连续读取 22 字节数据,分别是加速度计 X/Y/Z、温度、陀螺仪 X/Y/Z 和磁力计 X/Y/Z 以及磁力计状态寄存器 ST2。
    //后续加上状态位解决时序问题

    if(is_dma_started == 0)
    {
        HAL_StatusTypeDef status = MPU9250_ReadRegsDMA(MPU9250_REG_ACCEL_XOUT_H, buffer, 22, 20);
        if (status != HAL_OK)
        {
            // 处理读取失败的情况
            return HAL_ERROR;
        }
        is_dma_started = 1; // 标记 DMA 已启动
        return HAL_BUSY; // DMA 传输正在进行中
    }
    
    HAL_StatusTypeDef poll_status = MPU9250_ReadRegsDMA_Poll(20);
    if (poll_status == HAL_BUSY)
    {
        // DMA 传输仍在进行中，等待下一次调用时继续轮询
        return HAL_BUSY;
    }
    //HAL_OK / HAL_TIMEOUT / HAL_ERROR都代表此次的dma事务结束
    is_dma_started = 0; // 无论成功还是失败，都重置 DMA 启动标志，准备下一次读取

    if (poll_status != HAL_OK)
    {
        // 处理轮询失败的情况
        return poll_status; 
    }

    // 将原始数据从字节数组解析为 16 位整数，并存储到对应的 accel、temp、gyro 和 mag ST1、mag 、mag ST2数组中(只存储9个数据)。
    int16_t accel_raw[3], gyro_raw[3], mag_raw[3];
    for(int i = 0; i < 3; ++i)
    {
        accel_raw[i] = (int16_t)((buffer[2 * i] << 8) | buffer[2 * i + 1]);
        gyro_raw[i] = (int16_t)((buffer[8 + 2 * i] << 8) | buffer[8 + 2 * i + 1]);
        //AK8963 的数据是小端格式，低字节在前，高字节在后，所以解析时需要调整字节顺序。
        //需要检查磁力计数据的状态寄存器 ST1，确保数据就绪且无溢出错误后再解析磁力计数据;
        //以及磁力计数据的 ST2 寄存器，检查磁力计数据是否有溢出错误（如磁场过强导致的溢出），如果有错误则需要丢弃当前磁力计数据并进行相应处理。

        //1. 检查ST1和ST2寄存器，确保磁力计数据就绪且无溢出错误
        uint8_t st1 = buffer[14]; // 磁力计状态寄存器 ST1
        uint8_t st2 = buffer[21]; // 磁力计状态寄存器 ST2
        if ((st1 & 0x01) == 0) // 数据未就绪
        {
            // 处理数据未就绪的情况，例如设置 mag_raw[i] 为 0 或者其他默认值
            mag_raw[i] = 0;
        }
        else if ((st2 & 0x08) != 0) // 数据溢出错误
        {
            // 处理数据溢出错误的情况，例如设置 mag_raw[i] 为 0 或者其他默认值
            mag_raw[i] = 0;
        }
        else
        {
            // 数据就绪且无溢出错误，正常解析磁力计数据（小端格式）
            mag_raw[i] = (int16_t)(buffer[15 + 2 * i] |(buffer[16 + 2 * i] << 8));
        }
        
    }

    //将值转化成统一的单位：加速度计转换为 m/s²，陀螺仪转换为 deg/s，磁力计转换为 uT。

    for (int i = 0; i < 3; ++i)
    {
        accel[i] = (float)accel_raw[i] * turn_to_m_s2;
        gyro[i] = (float)gyro_raw[i] * turn_to_rad;
        mag[i] = (float)mag_raw[i] * turn_to_uT;
    }

    return HAL_OK;
}

//标定层 ： 通过读取原始数据进行校准，去除偏置和缩放误差，得到更准确的加速度、角速度和磁场强度值。

    // 1. 陀螺仪标定：通过静止状态下多次阻塞轮询测量，计算陀螺仪的零偏（bias），并在后续测量中减去该偏置值.

/**
 * @brief 在设备静止时标定陀螺仪零偏。
 * @param gyro_bias 输出陀螺仪零偏向量，单位 rad/s。
 * @retval None
 */
void MPU9250_Calibrate_Gyro(float *gyro_bias)
{
    int32_t gyro_sum[3] = {0};
    int16_t gyro_sample[3];
    uint8_t buffer[6];
    const int num_samples = 1000; // 采样数量，越多越准确，但耗时更长

    // 累加陀螺仪数据,避免溢出，多次进行平均值计算，如将1000 分为 10*100 次：每次累加 100 个样本，求一次平均值，最后求这10个平均值的平均。
    int32_t average_count [30] = {0};

    for(int i = 0; i < 10; i++ )
    {
        for(int j = 0; j < num_samples / 10; j++)
        {
            MPU9250_ReadRegs(MPU9250_REG_GYRO_XOUT_H, buffer, 6);
            for (int k = 0; k < 3; ++k)
            {
                gyro_sample[k] = (int16_t)((buffer[2 * k] << 8) | buffer[2 * k + 1]);
                gyro_sum[k] += gyro_sample[k];
            }
        }
        for (int k = 0; k < 3; ++k)
        {
            average_count[i*3 + k] = gyro_sum[k] / (num_samples / 10); // 每100个样本求一次平均值
            gyro_sum[k] = 0; // 重置累加器
        }
    }

    // 最后求这10个平均值的平均，得到最终的陀螺仪零偏值
    for(int k = 0; k < 3; ++k)
    {
        gyro_bias[k] = 0;
        for(int i = 0; i < 10; i++)
        {
            gyro_bias[k] += average_count[i*3 + k]; // 累加10次平均值
        }
        gyro_bias[k] /= 10; // 最后求这10个平均值的平均
        
    }
    // 将陀螺仪零偏值转换为 rad/s 单位
    for (int k = 0; k < 3; ++k)
    {
        gyro_bias[k] *= turn_to_rad;
    }

    // for (int i = 0; i < num_samples; ++i)
    // {
    //     // 读取当前陀螺仪数据
    //     MPU9250_ReadRegs(MPU9250_REG_GYRO_XOUT_H, buffer, 6);
    //     for (int j = 0; j < 3; ++j)
    //     {
    //         gyro_sample[j] = (int16_t)((buffer[2 * j] << 8) | buffer[2 * j + 1]);
    //     }

    //     // 累加陀螺仪数据
    //     for (int j = 0; j < 3; ++j)
    //     {
    //         gyro_sum[j] += gyro_sample[j];
    //     }

    //     HAL_Delay(10); // 等待一段时间以获取稳定的测量
    // }
}

    // 2. 加速度计标定：通过将 MPU9250 放置在不同的已知方向（如六面朝上）进行测量，计算加速度计的偏置和缩放因子，以校正测量值。
    //(先省略)

/**
 * @brief 等待用户按下并释放翻面提示按键。
 * @retval None
 */
void Wait_Turn(void)//等待翻转事件发生
{
    //通过轮询检查PA8引脚的状态，如果检测到翻转事件（如引脚状态变化），则执行相应的处理逻辑。
    while(1)
    {
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET)
        {
            HAL_Delay(100); // 防抖延时
            if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET)
            {
                //等待松手
                while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET)
                {
                }
                HAL_Delay(200);
                if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET)
                {
                    break; // 退出循环
                }
            }
        }
    }
}

//初始化pb8为推挽输出，在采样完成后置高电平，点亮LED，提示用户翻转设备到下一个面进行采样，出wait_turn函数后，再将pa3置低，等待下一个翻转事件发生。
/**
 * @brief 配置用于校准翻面提示的 GPIO。
 * @retval None
 */
void Init_Turn_Pin(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 使能 GPIOA 时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // 配置 PA8 为推挽输出

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}


//将数据写入flash中，供下次开机读取使用
/**
 * @brief 将加速度计偏置和比例系数保存到 Flash。
 * @param accel_bias 待保存的加速度计偏置向量。
 * @param accel_scale 待保存的加速度计比例系数向量。
 * @retval HAL_OK Flash 写入完成。
 * @retval HAL_ERROR Flash 解锁、擦除或写入失败。
 */
HAL_StatusTypeDef Save_Accel_Calib_To_Flash(float *accel_bias, float *accel_scale)
{
    AccelCalibData calib_data;
    // 设置魔数以验证数据有效性
    calib_data.magic = ACCEL_CALIB_MAGIC;
    // 将 bias 和 scale 数据复制到结构体中 ，原理：memcpy 函数用于将一块内存区域的数据复制到另一块内存区域。它的原型如下：
    // void *memcpy(void *dest, const void *src, size_t n);
    // dest：目标内存地址，即要复制到哪里。
    // src：源内存地址，即要复制的数据所在的位置。
    // n：要复制的字节数，即要复制多少数据。
    memcpy(calib_data.accel_bias, accel_bias, sizeof(calib_data.accel_bias));
    memcpy(calib_data.accel_scale, accel_scale, sizeof(calib_data.accel_scale));

    // 解锁 Flash
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return HAL_ERROR;
    }

    // 擦除目标扇区
    FLASH_EraseInitTypeDef erase_init;
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = ACCEL_CALIB_FLASH_ADDR;
    erase_init.NbPages = 1;

    uint32_t page_error;
    if (HAL_FLASHEx_Erase(&erase_init, &page_error) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    //按half word（16位）编程 Flash
    uint32_t half_word_count = (sizeof(calib_data) + 1) / 2;//向上取整，确保覆盖整个结构体数据
    uint32_t address = ACCEL_CALIB_FLASH_ADDR;
    uint16_t *data_ptr = (uint16_t *)&calib_data;
    for (uint32_t i = 0; i < half_word_count;i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, data_ptr[i]) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }
        address += 2; // 移动到下一个 half word 地址
    }

    // 锁定 Flash
    HAL_FLASH_Lock(); 
    return HAL_OK;
}

/**
 * @brief 点亮提示 LED 并等待用户切换到下一个校准面。
 * @retval None
 */
void get_data_suggestion(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); // 点亮LED
    Wait_Turn(); // 等待用户翻转设备到下一个面进行采样
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // 关闭LED
}


/**
 * @brief 执行六面加速度计校准，并按需保存校准结果。
 * @param accel_bias 输出加速度计偏置向量。
 * @param accel_scale 输出加速度计比例系数向量。
 * @param is_mag_calib_enabled 非 0 表示加速度计数据用于加速度计和磁力计联合校准。
 * @retval None
 */
void MPU9250_Calibrate_Accel(float *accel_bias, float *accel_scale,uint8_t is_mag_calib_enabled)
{
    //用于存储在六个面的采样信息
    float accel_samples[6][3] = {0};
    uint8_t buffer[6];
    Init_Turn_Pin(); // 初始化翻转提示引脚

    get_data_suggestion(); // 提示用户翻转设备到第一个面进行采样

    //6面的朝上顺序依次是：+X, -X, +Y, -Y, +Z, -Z。每面采样1000次取平均值，得到六个面的加速度计数据。
    for(uint8_t face = 0; face < 6; ++face)
    {
        // 采集当前面的加速度计数据,采多次取平均值
        int32_t accel_sum[3] = {0};
        const int num_samples = 1000; // 采样数量

        for (int k = 0; k < num_samples; ++k)
        {
            MPU9250_ReadRegs(MPU9250_REG_ACCEL_XOUT_H, buffer, 6);
            for (int j = 0; j < 3; ++j)
            {
                int16_t raw = (int16_t)((buffer[2 * j] << 8) | buffer[2 * j + 1]);
                accel_sum[j] += raw;
            }
            HAL_Delay(10); // 等待一段时间以获取稳定的测量
        }
        for (int j = 0; j < 3; ++j)
        {
            float average = (float)accel_sum[j] / num_samples; // 计算平均值
            accel_samples[face][j] = average * turn_to_m_s2; // 转换为 m/s²
        }
        if(face < 5) // 前五面采样完成后提示用户翻转设备到下一个面进行采样
        {
            get_data_suggestion(); // 提示用户翻转设备到下一个面进行采样
        }
    }

    //计算bias 公式：bias = (正面测量值 + 反面测量值) / 2
    for(int j = 0; j < 3; ++j)
    {
        accel_bias[j] = (accel_samples[2 * j][j] + accel_samples[2 * j + 1][j]) / 2.0f;
    }

    //计算scale 公式：scale = 2g / (正面测量值 - 反面测量值)

        //g是重力加速度，约为 9.81 m/s²
    const float g = 9.81f;

    for(int j = 0; j < 3; ++j)
    {
        accel_scale[j] = (2 * g) / (accel_samples[2 * j][j] - accel_samples[2 * j + 1][j]);
    }

    //将bias和scale存储到flash中
    if(!is_mag_calib_enabled)
    {
        if(Save_Accel_Calib_To_Flash(accel_bias, accel_scale) != HAL_OK)
        {
            // 处理存储失败的情况，例如记录错误日志或采取其他措施
        }
    }
    HAL_Delay(1000);
    //点亮LED提示用户校准完成
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
}
    // 3. 磁力计标定：通过在不同方向旋转 MPU9250，收集磁力计数据，计算磁力计的偏置和缩放因子，以校正测量值。
/**
 * @brief 标定磁力计硬铁偏置和软铁比例补偿参数。
 * @param mag_bias 输出磁力计偏置向量。
 * @param mag_scale 输出磁力计比例系数向量。
 * @retval None
 */
void MPU9250_Calibrate_Mag(float *mag_bias, float *mag_scale)
{
    
}


// 4. 预处理层 

    //对gyro减去0偏值，得到校准后的角速度数据。
/**
 * @brief 对陀螺仪数据进行零偏校正。
 * @param gyro 陀螺仪角速度向量，单位 rad/s，函数内原地更新。
 * @param gyro_bias 陀螺仪零偏向量，单位 rad/s。
 * @retval None
 */
void MPU9250_Get_Calibrated_Gyro(float *gyro, float *gyro_bias)
{
    for (int i = 0; i < 3; ++i)
    {
        gyro[i] = gyro[i] - gyro_bias[i];
    }
    
}

    //对accel进行bias/scale校正，归一化后检测模长是否合理；

//先从flash中读取bias和scale
/**
 * @brief 从 Flash 中读取加速度计校准数据。
 * @param accel_basic 输出加速度计偏置向量。
 * @param accel_scale 输出加速度计比例系数向量。
 * @param is_mag_calib_enabled 非 0 表示读取加速度计和磁力计联合校准数据。
 * @retval None
 */
void Get_Data_From_Flash(float *accel_basic , float *accel_scale , uint8_t is_mag_calib_enabled)
{
    if(is_mag_calib_enabled)
    {
        // 启用磁力计标定后，存储的结构体会包含磁力计的bias和scale数据，因此需要定义一个新的结构体来读取这些数据。
        // 这里先省略具体实现，假设已经定义了 AccelMagCalibData 结构体并且在 flash 中存储了相应的数据。
    }
    else
    {
        // 读取加速度计标定数据
        AccelCalibData calib_data;
        memcpy(&calib_data, (void *)ACCEL_CALIB_FLASH_ADDR, sizeof(calib_data));
        if (calib_data.magic == ACCEL_CALIB_MAGIC)
        {
            memcpy(accel_basic, calib_data.accel_bias, sizeof(calib_data.accel_bias));
            memcpy(accel_scale, calib_data.accel_scale, sizeof(calib_data.accel_scale));
        }
        else
        {
            // 处理无效数据的情况，例如使用默认值或记录错误日志
        }
    }
}
//对accel进行bias/scale校正，归一化后检测模长是否合理；
/**
 * @brief 对加速度计数据进行偏置和比例校正，并判断数据可信度。
 * @param accel 加速度向量，单位 m/s^2，函数内原地更新。
 * @param accel_bias 加速度计偏置向量。
 * @param accel_scale 加速度计比例系数向量。
 * @param credible_of_accel 输出可信度标志，1 表示可信，0 表示不可信。
 * @retval None
 */
void MPU9250_Get_Calibrated_Accel(float *accel, float *accel_bias, float *accel_scale, uint8_t *credible_of_accel)
{
    //使用从 flash 中读取的 bias 和 scale 数据对原始加速度计数据进行校正，得到校准后的加速度数据。
    for (int i = 0; i < 3; ++i)
    {
        accel[i] = (accel[i] - accel_bias[i]) * accel_scale[i];
    }
    // 计算加速度向量的模长,并判断是否可行（先不做复杂可行度判断，后续再完善）
    float accel_magnitude = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    if (fabsf(accel_magnitude - 9.81f) > 2.0f) // 如果模长偏离重力加速度超过 2 m/s²，认为数据不可靠
    {
        *credible_of_accel = 0; // 标记加速度数据不可靠
    }
    else
    {
        *credible_of_accel = 1; // 标记加速度数据可靠
    }
    
}

    //对mag进行hard/soft iron校正，坐标重映射，必要时进行低通滤波，做归一化；

/**
 * @brief 对磁力计数据进行校准处理。
 * @retval None
 */
void MPU9250_Get_Calibrated_Mag(void)
{
    //先省略具体实现
}

    //将3轴标定函数打包成一个函数接口，输入原始数据，输出校准后的数据，供后续坐标系转换和数据融合使用。
    //后续需要添加磁力计的校准数据输入参数
/**
 * @brief 对一帧传感器数据执行所有已启用的校准步骤。
 * @param gyro 陀螺仪角速度向量，单位 rad/s，函数内原地更新。
 * @param gyro_bias 陀螺仪零偏向量，单位 rad/s。
 * @param accel 加速度向量，单位 m/s^2，函数内原地更新。
 * @param accel_bias 加速度计偏置向量。
 * @param accel_scale 加速度计比例系数向量。
 * @param credible_of_accel 输出加速度计可信度标志。
 * @param mag 磁力计向量，单位 uT。
 * @retval None
 */
void MPU9250_Get_Calibrated_Data( float *gyro, float *gyro_bias, float *accel,float *accel_bias, float *accel_scale,uint8_t *credible_of_accel, float *mag )
{
    MPU9250_Get_Calibrated_Accel(accel, accel_bias, accel_scale, credible_of_accel);
    MPU9250_Get_Calibrated_Gyro(gyro, gyro_bias);
    MPU9250_Get_Calibrated_Mag();//先不写具体实现，后续再完善
}

// 5. 坐标系转换层 ： 将传感器数据从设备坐标系转换到惯性坐标系，确保加速度、角速度和磁场强度的方向和单位符合惯性导航算法的要求。

    //根据 MPU9250 的安装方向，将加速度计、陀螺仪和磁力计的数据进行坐标轴重映射，使其符合惯性导航算法使用的右手坐标系。
    
    //body frame : X轴指向前方，Y轴指向右侧，Z轴指向下方,和惯性导航算法使用的坐标系一致，因此不需要进行坐标系转换。
    //如果 MPU9250 的安装方向不同，需要根据实际安装情况进行坐标轴重映射，例如交换轴顺序或改变某些轴的符号，以确保输出数据符合惯性导航算法的输入要求。

    //world frame ：目前未启用磁力计，因此暂时不进行坐标系转换，后续启用磁力计后再根据实际安装情况进行坐标轴重映射。


// 6.数据融合层 ： 使用Mahony四元数融合算法，将加速度计、陀螺仪和磁力计的数据融合，计算出设备的姿态（如滚转角、俯仰角和偏航角）。该算法能够有效地利用各传感器的数据，提供稳定且准确的姿态估计。
/**
 * @brief 将采样周期限制在数值稳定的范围内。
 * @param dt_s 采样周期，单位 s。
 * @retval 限幅后的采样周期，单位 s。
 */
static float MPU9250_Limit_Dt(float dt_s)
{
    //限制 dt 的范围，避免数值不稳定导致的姿态估算错误。
    if(dt_s <= 0.0f)
    {
        dt_s = 0.005f; // 设置一个默认的合理采样时间间隔，例如10ms
    }
    else if(dt_s > 0.05f)
    {
        dt_s = 0.05f; // 将过大的dt限制在一个合理范围内，避免数值不稳定
    }
    return dt_s;
}

/**
 * @brief 仅使用陀螺仪数据积分更新姿态四元数。
 * @param dt 采样周期，单位 s。
 * @param gyro 陀螺仪角速度向量，单位 rad/s。
 * @retval None
 */
void MPU9250_Only_gyro_Update(float dt, float *gyro)
{
    dt = MPU9250_Limit_Dt(dt);
    // 仅使用陀螺仪数据更新四元数，适用于加速度计数据不可靠的情况。
    float omega_q[4] = {0.0f, gyro[0], gyro[1], gyro[2]}; // 角速度的四元数表示
    float q_new[4];
    float t_quaterion[4] = {quaternion[0], quaternion[1], quaternion[2], quaternion[3]}; // 当前四元数的副本，避免在计算过程中修改原始四元数
    // 计算 q_new = 0.5 * q ⊗ ω_q
    q_new[0] = 0.5f * (t_quaterion[0] * omega_q[0] - t_quaterion[1] * omega_q[1] - t_quaterion[2] * omega_q[2] - t_quaterion[3] * omega_q[3]);
    q_new[1] = 0.5f * (t_quaterion[0] * omega_q[1] + t_quaterion[1] * omega_q[0] + t_quaterion[2] * omega_q[3] - t_quaterion[3] * omega_q[2]);
    q_new[2] = 0.5f * (t_quaterion[0] * omega_q[2] - t_quaterion[1] * omega_q[3] + t_quaterion[2] * omega_q[0] + t_quaterion[3] * omega_q[1]);
    q_new[3] = 0.5f * (t_quaterion[0] * omega_q[3] + t_quaterion[1] * omega_q[2] - t_quaterion[2] * omega_q[1] + t_quaterion[3] * omega_q[0]);
    // 离散积分： q = q + q_new * dt，其中 dt 是采样时间间隔。最后对 q 进行归一化，确保其保持为单位四元数。
    for (int i = 0; i < 4; ++i)
    {
        t_quaterion[i] += q_new[i] * dt; // 更新四元数
    }
    float q_mod = sqrtf(t_quaterion[0]*t_quaterion[0] + t_quaterion[1]*t_quaterion[1] + t_quaterion[2]*t_quaterion[2] + t_quaterion[3]*t_quaterion[3]);

    //零模长保护，避免除以零
    if(q_mod < 1e-6f)
    {
        //弃用当前更新
        return;
    }
    quaternion[0] = t_quaterion[0] / q_mod;
    quaternion[1] = t_quaterion[1] / q_mod;
    quaternion[2] = t_quaterion[2] / q_mod;
    quaternion[3] = t_quaterion[3] / q_mod;
}

    //实现 Mahony 四元数融合算法，输入校准后的加速度计、陀螺仪和磁力计数据，输出设备的姿态信息。
/**
 * @brief 使用 Mahony 融合算法更新姿态四元数。
 * @param dt 采样周期，单位 s。
 * @param gyro 校准后的陀螺仪角速度向量，单位 rad/s。
 * @param accel 校准后的加速度向量，单位 m/s^2。
 * @param mag 校准后的磁力计向量，单位 uT。
 * @param credible_of_accel 非 0 表示加速度计数据可用于姿态修正。
 * @param is_mag_calib_enabled 非 0 表示启用磁力计融合路径。
 * @param kp Mahony 比例增益。
 * @param ki Mahony 积分增益。
 * @retval None
 */
void MPU9250_Mahony_Fusion(float dt, float *gyro, float *accel, float *mag, int8_t credible_of_accel, uint8_t is_mag_calib_enabled, float kp, float ki)
{
    //限制 dt 的范围，避免数值不稳定导致的姿态估算错误。
    dt = MPU9250_Limit_Dt(dt);
    //无磁力计的 Mahony 算法实现
    if(!is_mag_calib_enabled)
    {
        // 1. 将当前姿态表示为四元数 q = [q0, q1, q2, q3]，它表示“机体做标系”相对于“惯性坐标系”的旋转。
        
        //加速度可信的情况下：
        
        if(credible_of_accel)
        {
            // 2. 通过四元数预测重力加速度的方向 a_pred = q * [0, 0, 1] * q^-1，得到预测的重力加速度向量。
            // 计算预测的重力加速度向量 a_pred
            float a_pred[3];
            // 四元数 q 的分量
            float q0 = quaternion[0];
            float q1 = quaternion[1];
            float q2 = quaternion[2];
            float q3 = quaternion[3];

            // 计算 a_pred = q * [0, 0, 1] * q^-1
            a_pred[0] = 2.0f * (q1 * q3 - q0 * q2); // x 分量
            a_pred[1] = 2.0f * (q0 * q1 + q2 * q3); // y 分量
            a_pred[2] = q0*q0 - q1*q1 - q2*q2 + q3*q3; // z 分量
        
            // 3. 计算加速度计测量值
            float a_meas[3] = {accel[0], accel[1], accel[2]}; // 加速度计测量值

            float a_mod = sqrtf(a_meas[0]*a_meas[0] + a_meas[1]*a_meas[1] + a_meas[2]*a_meas[2]);
            //模长已经在之前进行过可行度判断，这里不再进行模长判断，直接归一化

            a_meas[0] /= a_mod;
            a_meas[1] /= a_mod; 
            a_meas[2] /= a_mod;
            // 4. 使用加速度计测量值与预测值之间的误差 e = a_meas × a_pred，得到误差向量。
            float e[3];

            e[0] = a_meas[1] * a_pred[2] - a_meas[2] * a_pred[1]; // x 分量
            e[1] = a_meas[2] * a_pred[0] - a_meas[0] * a_pred[2]; // y 分量
            e[2] = a_meas[0] * a_pred[1] - a_meas[1] * a_pred[0]; // z 分量
            // 5. PI校正：使用误差向量 e 来调整陀螺仪测量值，得到校正后的角速度 w_corr = w_meas + Kp * e + Ki * ∫e dt，其中 Kp 和 Ki 是比例和积分增益。

            exInt += e[0] * dt; // 误差积分
            eyInt += e[1] * dt;
            ezInt += e[2] * dt;

            if(exInt > e_KI_MAX) exInt = e_KI_MAX; // 积分限幅
            else if(exInt < e_KI_MIN) exInt = e_KI_MIN;
            if(eyInt > e_KI_MAX) eyInt = e_KI_MAX;
            else if(eyInt < e_KI_MIN) eyInt = e_KI_MIN;
            if(ezInt > e_KI_MAX) ezInt = e_KI_MAX;
            else if(ezInt < e_KI_MIN) ezInt = e_KI_MIN;

            float w_corr[3];
            w_corr[0] = gyro[0] + kp * e[0] + ki * exInt; // x 分量
            w_corr[1] = gyro[1] + kp * e[1] + ki * eyInt; // y 分量
            w_corr[2] = gyro[2] + kp * e[2] + ki * ezInt; // z 分量

            // 6. 使用校正后的角速度 w_corr 来更新四元数 q 的值，得到新的姿态估计：q_new = 0.5 * q ⊗ ω_q​ ,其中 ω_q = [0, w_corr_x, w_corr_y, w_corr_z] 是角速度的四元数表示，⊗ 表示四元数乘法。
            float omega_q[4] = {0.0f, w_corr[0], w_corr[1], w_corr[2]}; // 角速度的四元数表示
            float q_new[4];
            // 计算 q_new = 0.5 * q ⊗ ω_q
            float t_quaterion[4] = {quaternion[0], quaternion[1], quaternion[2], quaternion[3]}; // 当前四元数的副本，避免在计算过程中修改原始四元数
            q_new[0] = 0.5f * (t_quaterion[0] * omega_q[0] - t_quaterion[1] * omega_q[1] - t_quaterion[2] * omega_q[2] - t_quaterion[3] * omega_q[3]);
            q_new[1] = 0.5f * (t_quaterion[0] * omega_q[1] + t_quaterion[1] * omega_q[0] + t_quaterion[2] * omega_q[3] - t_quaterion[3] * omega_q[2]);
            q_new[2] = 0.5f * (t_quaterion[0] * omega_q[2] - t_quaterion[1] * omega_q[3] + t_quaterion[2] * omega_q[0] + t_quaterion[3] * omega_q[1]);
            q_new[3] = 0.5f * (t_quaterion[0] * omega_q[3] + t_quaterion[1] * omega_q[2] - t_quaterion[2] * omega_q[1] + t_quaterion[3] * omega_q[0]);
            // 7. 离散积分： q = q + q_new * dt，其中 dt 是采样时间间隔。最后对 q 进行归一化，确保其保持为单位四元数。
            t_quaterion[0] += q_new[0] * dt; // 注意 dt 的单位是秒，需要乘以 1000 转换为毫秒
            t_quaterion[1] += q_new[1] * dt;
            t_quaterion[2] += q_new[2] * dt;
            t_quaterion[3] += q_new[3] * dt;
            float q_mod = sqrtf(t_quaterion[0]*t_quaterion[0] + t_quaterion[1]*t_quaterion[1] + t_quaterion[2]*t_quaterion[2] + t_quaterion[3]*t_quaterion[3]);
            //零模长保护，避免除以零
            if(q_mod < 1e-6f)
            {
                //弃用当前更新，保持四元数不变
                return;
            }
            quaternion[0] = t_quaterion[0] / q_mod;
            quaternion[1] = t_quaterion[1] / q_mod;
            quaternion[2] = t_quaterion[2] / q_mod;
            quaternion[3] = t_quaterion[3] / q_mod;
        }
        else
        {
            // 加速度不可信的情况下，仅使用陀螺仪数据更新四元数，适用于加速度计数据不可靠的情况。
            MPU9250_Only_gyro_Update(dt, gyro);
        }
    }
    else
    {
        // 启用磁力计标定后，Mahony 算法的实现会更加复杂，因为需要同时处理加速度计、陀螺仪和磁力计的数据进行融合。这里先省略具体实现，后续再完善。
    }

    for(uint8_t i = 0; i < 4; ++i)
    {
        output_quaternion[i] = quaternion[i]; // 将当前的四元数输出到外部接口
    }
}


