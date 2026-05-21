/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 主程序文件
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU9250_AHRS.h"
#include "MPU9250_Init.h"
#include "stdio.h"
#include "motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t mpu9250_buffer[22]; // 存储从 MPU9250 读取的原始数据的缓冲区

float accel[3]; // 存储加速度计数据的数组
float gyro[3]; // 存储陀螺仪数据的数组
float mag[3]; // 存储磁力计数据的数组

uint8_t is_mag_calib_enabled = 0; // 磁力计校准是否启用的标志
uint8_t credible_of_accel = 0; // 加速度计数据是否可靠的标志

float accel_bias[3]; // 存储加速度计偏置的数组
float accel_scale[3]; // 存储加速度计缩放因子的数组
float gyro_bias[3]; // 存储陀螺仪偏置的数组

uint32_t last_update_tick = 0; // 上次更新的系统时钟滴答数
int16_t imu_error_count = 0; // IMU 错误计数器

float AHRS_kp = 2.0f; // Mahony 四元数融合算法的比例增益
float AHRS_ki = 0.0f; // Mahony 四元数融合算法的积分增益

static struct motor_speed the_motor_speed; // 存储电机速度的结构体变量

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  通过 USART1 以 CSV 格式发送加速度计和陀螺仪调试数据。
  * @param  ax X 轴加速度，单位 m/s^2。
  * @param  ay Y 轴加速度，单位 m/s^2。
  * @param  az Z 轴加速度，单位 m/s^2。
  * @param  gx X 轴角速度，单位 rad/s。
  * @param  gy Y 轴角速度，单位 rad/s。
  * @param  gz Z 轴角速度，单位 rad/s。
  * @retval None
  */
static void Debug_Send_IMU(float ax, float ay, float az,
                           float gx, float gy, float gz)
{
    char buf[128];

    int32_t ax_i = (int32_t)(ax * 1000.0f);
    int32_t ay_i = (int32_t)(ay * 1000.0f);
    int32_t az_i = (int32_t)(az * 1000.0f);

    int32_t gx_i = (int32_t)(gx * 1000.0f);
    int32_t gy_i = (int32_t)(gy * 1000.0f);
    int32_t gz_i = (int32_t)(gz * 1000.0f);

    int len = snprintf(buf, sizeof(buf),
                       "%ld,%ld,%ld,%ld,%ld,%ld\r\n",
                       ax_i, ay_i, az_i,
                       gx_i, gy_i, gz_i);

    if (len > 0)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 10);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  is_mag_calib_enabled = 0; // 是否启用磁力计标定，默认为0（不启用），可以根据需要修改为1（启用）
  HAL_TIM_Base_Start(&htim1); 
  motor_init(); // 初始化电机控制相关的定时器和 GPIO
  if(MPU9250_Init() != HAL_OK)
  {
    // 处理初始化失败的情况，例如记录错误日志或进入安全模式
    Error_Handler();
  }
  Get_Data_From_Flash(accel_bias, accel_scale, is_mag_calib_enabled); // 从 flash 中读取加速度计的 bias 和 scale 数据 
  MPU9250_Calibrate_Gyro(gyro_bias); // 进行陀螺仪标定，获取陀螺仪的零偏值
  //获取定时器tim1的当前计数值，作为上次更新的系统时钟滴答数
  uint8_t first_update = 1; // 是否是第一次更新的标志

  //MPU9250_Calibrate_Gyro(gyro_bias); // 进行陀螺仪标定，获取陀螺仪的零偏值

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // while (1)
  // {
  //   HAL_UART_Transmit(&huart1,
  //                     (uint8_t *)"1,2,3,4,5,6\r\n",
  //                     13,
  //                     10);
  //   HAL_Delay(100);
  // }
  while (1)
  {
    /* USER CODE END WHILE */
    motor_set_pwm(1, 5000,0);
    motor_set_pwm(2, 5000,0); 
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
