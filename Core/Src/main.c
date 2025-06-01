/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

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
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define LIS3DSH_CS_PORT GPIOB
#define LIS3DSH_CS_PIN  GPIO_PIN_5

#define LIS3DSH_CS_LOW()   HAL_GPIO_WritePin(LIS3DSH_CS_PORT, LIS3DSH_CS_PIN, GPIO_PIN_RESET)
#define LIS3DSH_CS_HIGH()  HAL_GPIO_WritePin(LIS3DSH_CS_PORT, LIS3DSH_CS_PIN, GPIO_PIN_SET)

extern SPI_HandleTypeDef hspi1;
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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  uint8_t btnState = 0;
  uint8_t prevBtnState = 0;

  uint32_t pressedTime = 0;
  uint8_t blinking = 0;

  //LIS3DSH SPI Test
  LIS3DSH_CS_HIGH();
  HAL_Delay(100);

  LIS3DSH_Test();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    MX_USB_HOST_Process();

//    printf("Hello World-! \r\n");
//    HAL_Delay(1000);

    btnState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

    if (btnState == GPIO_PIN_SET && prevBtnState == GPIO_PIN_RESET)
        {
          pressedTime = HAL_GetTick();
        }

    if (btnState == GPIO_PIN_SET && (HAL_GetTick() - pressedTime >= 2000))
    {
      blinking = 1;
    }

    if (btnState == GPIO_PIN_RESET && prevBtnState == GPIO_PIN_SET)
    {
      if (HAL_GetTick() - pressedTime < 2000)
      {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);   // Green
      }
      blinking = 0;
    }

    if (blinking)
    {
      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);    // Blue
      HAL_Delay(50);
    }
    else
    {
      HAL_Delay(10);
    }

    LIS3DSH_ReadXYZ();
    HAL_Delay(500);

    prevBtnState = btnState;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* printf() - override */

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}


uint8_t LIS3DSH_ReadReg(uint8_t reg)
{
  uint8_t tx[2];
  uint8_t rx[2];

  tx[0] = reg | 0x80; // read : MSB = 1
  tx[1] = 0x00;

  LIS3DSH_CS_LOW();
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
  LIS3DSH_CS_HIGH();

  return rx[1];
}


void LIS3DSH_Test(void)
{
  uint8_t id = LIS3DSH_ReadReg(0x0F);   // read who am i reg

  if (id == 0x3F)  // LIS3DSH ID
  {
    printf("LIS3DSH detected! WHO_AM_I = 0x%02X\r\n", id);
  }
  else
  {
    printf("LIS3DSH not found. Read: 0x%02X\r\n", id);
  }
}


int16_t LIS3DSH_ReadAxis(uint8_t addr_l, uint8_t addr_h)
{
  uint8_t low = LIS3DSH_ReadReg(addr_l);      // read LSB
  uint8_t high = LIS3DSH_ReadReg(addr_h);     // read MSB
  return (uint16_t)((high << 8) | low);       // 16 bit (MSB + LSB)
}


void LIS3DSH_ReadXYZ(void)
{
  int16_t x = LIS3DSH_ReadAxis(0x28, 0x29);    // x-LSB, x-MSB
  int16_t y = LIS3DSH_ReadAxis(0x2A, 0x2B);
  int16_t z = LIS3DSH_ReadAxis(0x2C, 0x2D);

  printf("X: %d, Y: %d, Z: %d \r\n", x, y, z);
}

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
