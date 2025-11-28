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
#include "dma.h"
#include "i2c.h"
#include "lcd.h"
#include "sai.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define CS43L22_I2C_ADDR  0x94  // Adres układu audio na I2C

// Bufor z jedną pełną sinusoidą (ok. 160Hz przy próbkowaniu 16kHz)
int16_t sine_wave[100] = {
    0, 2057, 4067, 5985, 7765, 9368, 10760, 11909, 12796, 13404,
    13728, 13768, 13525, 13002, 12206, 11150, 9849, 8320, 6592, 4697,
    2673, 563, -1588, -3729, -5809, -7778, -9588, -11195, -12558, -13639,
    -14408, -14846, -14948, -14713, -14146, -13260, -12076, -10620, -8923, -7025,
    -4972, -2813, -597, 1625, 3804, 5887, 7824, 9568, 11078, 12318,
    13260, 13889, 14197, 14182, 13847, 13197, 12242, 10998, 9489, 7747,
    5812, 3731, 1555, -664, -2877, -5031, -7077, -8968, -10656, -12102,
    -13274, -14143, -14691, -14909, -14792, -14339, -13557, -12466, -11097, -9492,
    -7693, -5741, -3678, -1553, 579, 2664, 4648, 6483, 8128, 9548
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void CS43L22_Init(I2C_HandleTypeDef *hi2c) {
    // 1. OBUDZENIE UKŁADU (HARDWARE RESET)
    // Pin PE3 musi być w stanie WYSOKIM, żeby układ działał.
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_Delay(50); // Czekamy aż układ wstanie

    uint8_t txData[2];

    // 2. POWER ON (Rejestr 0x02 -> 0x9E)
    txData[0] = 0x02;
    txData[1] = 0x9E;
    HAL_I2C_Master_Transmit(hi2c, CS43L22_I2C_ADDR, txData, 2, HAL_MAX_DELAY);

    // 3. WŁĄCZENIE WYJŚCIA SŁUCHAWKOWEGO I SYGNAŁU (Rejestr 0x04 -> 0xAF)
    txData[0] = 0x04;
    txData[1] = 0xAF;
    HAL_I2C_Master_Transmit(hi2c, CS43L22_I2C_ADDR, txData, 2, HAL_MAX_DELAY);

    // 4. KONFIGURACJA ZEGARA (AUTO-DETECT) (Rejestr 0x05 -> 0x81)
    txData[0] = 0x05;
    txData[1] = 0x81;
    HAL_I2C_Master_Transmit(hi2c, CS43L22_I2C_ADDR, txData, 2, HAL_MAX_DELAY);

    // 5. USTAWIENIE GŁOŚNOŚCI (Rejestr 0x20 i 0x21 -> 0x18)
    // Wartość 0x18 to przyzwoita głośność testowa. Max to 0x00 (+12dB), Min to 0xFF.
    txData[0] = 0x20; txData[1] = 0x18; // Kanał A
    HAL_I2C_Master_Transmit(hi2c, CS43L22_I2C_ADDR, txData, 2, HAL_MAX_DELAY);

    txData[0] = 0x21; txData[1] = 0x18; // Kanał B
    HAL_I2C_Master_Transmit(hi2c, CS43L22_I2C_ADDR, txData, 2, HAL_MAX_DELAY);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
// aktualizacja
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
  MX_I2C1_Init();
  MX_SAI1_Init();
  MX_TIM1_Init();
  MX_LCD_Init();
  /* USER CODE BEGIN 2 */
  // Inicjalizujemy kodek audio (włączamy zasilanie na PE3 i konfigurujemy przez I2C)
    CS43L22_Init(&hi2c1);

    // Uruchamiamy wysyłanie danych sinusoidy przez SAI (DMA) w pętli (Circular)
    // "sine_wave" to nasze dane, "100" to liczba próbek w tablicy
    if(HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t*)sine_wave, 100) != HAL_OK)
    {
        Error_Handler(); // Jeśli tu wejdzie, coś jest nie tak z zegarami SAI lub DMA
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	HAL_GPIO_TogglePin(LD_G_GPIO_Port, LD_G_Pin);
	HAL_Delay(200);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE(); // SAI1 zazwyczaj używa DMA2

  /* DMA interrupt init */
  /* DMA2_Channel1_IRQn interrupt configuration */
  // Ustawiamy priorytet i włączamy przerwanie w NVIC
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
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
