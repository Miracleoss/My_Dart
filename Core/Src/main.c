/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fdcan.h"
#include "iwdg.h"
#include "memorymap.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"


int testlez = 0;
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "arm_math.h"
#include "tsk_config_and_callback.h"
/* USER CODE END Includes */
#include "stm32h7xx.h"  // 假设已包含标准外设库头文件
typedef struct {
uint8_t por_rst   : 1;  // Power-On Reset
uint8_t pin_rst   : 1;  // NRST Pin Reset
uint8_t bor_rst   : 1;  // Brown-Out Reset
uint8_t sw_rst    : 1;  // Software Reset (SYSRESETREQ)
uint8_t iwdg_rst  : 1;  // Independent Watchdog Reset
uint8_t wwdg_rst  : 1;  // Window Watchdog Reset
uint8_t illegal_stby : 1; // Illegal Stop/Standby Entry
uint8_t obl_rst   : 1;  // Option Byte Load Reset
} rcc_reset_flags_t;
/**
* @brief 读取并解析 RCC 复位状态寄存器
* @return 解析后的复位标志结构体
*/
rcc_reset_flags_t RCC_GetResetSource(void)
{
uint32_t rsr_val = RCC->RSR;
rcc_reset_flags_t flags = {0};
flags.por_rst      = (rsr_val & RCC_RSR_PORRSTF)  ? 1U : 0U;
flags.pin_rst      = (rsr_val & RCC_RSR_PINRSTF)  ? 1U : 0U;
flags.bor_rst      = (rsr_val & RCC_RSR_BORRSTF)  ? 1U : 0U;
flags.sw_rst       = (rsr_val & RCC_RSR_SFTRSTF)  ? 1U : 0U;
flags.illegal_stby = (rsr_val & RCC_RSR_LPWRRSTF) ? 1U : 0U;
return flags;
}
/**
* @brief 清除所有 RCC 复位标志
* @note 必须在读取 RCC->RSR 后调用，否则可能丢失新复位事件
*/
void RCC_ClearResetFlags(void)
{
RCC->RSR = RCC_RSR_RMVF;  // 直接写入 RMVF 位（BIT16）
}
/**
* @brief 初始化阶段复位诊断（示例）
*/
void System_Reset_Diagnosis(void)
{
rcc_reset_flags_t rst_flags = RCC_GetResetSource();
// 输出复位源日志（实际项目中替换为 UART/ITM/SWO 输出）
if (rst_flags.por_rst)   { testlez = 1; }
if (rst_flags.pin_rst)   { testlez = 2; }
if (rst_flags.bor_rst)   { testlez = 3; }
if (rst_flags.sw_rst)    { testlez = 4; }
if (rst_flags.iwdg_rst)  { testlez = 5; }
if (rst_flags.wwdg_rst)  { testlez = 6; }
if (rst_flags.illegal_stby) { testlez = 7; }
// 清除标志，避免下次启动重复报告
RCC_ClearResetFlags();
}

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_UART7_Init();
  MX_USART1_UART_Init();
  MX_USART10_UART_Init();
  MX_USB_DEVICE_Init();
  MX_UART5_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_UART8_Init();
  MX_UART9_Init();
  //MX_IWDG1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  System_Reset_Diagnosis();
  Task_Init();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Task_Loop();
    /* USER CODE END WHILE */

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

#ifdef  USE_FULL_ASSERT
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
