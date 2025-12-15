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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "czujnik.h"
<<<<<<< HEAD

=======
#include <math.h>
>>>>>>> 5f7130c3d8c57bc684bc235a55fc1d00f6fd25dc
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
#define CS43L22_I2C_ADDR  0x94
#define SAMPLE_RATE       16000
#define PI                3.14159265f

// Bufor audio
#define BUFFER_SIZE       2048
int16_t tx_buffer[BUFFER_SIZE];

// Zmienne syntezy
volatile float current_freq = 0.0f; //aktualna f dzwieku
float phase_pos = 0.0f; //obecna faza
float volume = 8000.0f; //do glosnosci

// Definicje nut
#define NOTE_C4  261.63f
#define NOTE_D4 293.66f
#define NOTE_E4  329.63f
#define NOTE_F4  349.99f
#define NOTE_G4  392.00f
#define NOTE_A4  440.00f
#define NOTE_H4  493.88f
#define NOTE_C5 523.25F
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void CS43L22_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t txData[2];

    // Hardware reset
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_Delay(50); //reset na 50ms

    // Power Down
    txData[0] = 0x02;
    txData[1] = 0x01; // Power down
    HAL_I2C_Master_Transmit(hi2c, CS43L22_I2C_ADDR, txData, 2, HAL_MAX_DELAY);

    // Speaker OFF, Headpfones ON
    txData[0] = 0x04;
    txData[1] = 0xAF;
    HAL_I2C_Master_Transmit(hi2c, CS43L22_I2C_ADDR, txData, 2, HAL_MAX_DELAY);

    // Auto clock detect
    txData[0] = 0x05;
    txData[1] = 0x81;//wlacza auto-detekcje zegara
    HAL_I2C_Master_Transmit(hi2c, CS43L22_I2C_ADDR, txData, 2, HAL_MAX_DELAY);

    // DISABLE analog inputs
    txData[0] = 0x06;
    txData[1] = 0x04; // Only DAC enabled
    HAL_I2C_Master_Transmit(hi2c, CS43L22_I2C_ADDR, txData, 2, HAL_MAX_DELAY);

    // Mute both channels on startup
    txData[0] = 0x20; txData[1] = 0xFF;
    HAL_I2C_Master_Transmit(hi2c, CS43L22_I2C_ADDR, txData, 2, HAL_MAX_DELAY);
    txData[0] = 0x21; txData[1] = 0xFF;
    HAL_I2C_Master_Transmit(hi2c, CS43L22_I2C_ADDR, txData, 2, HAL_MAX_DELAY);
	//brak trzaskow na poczatku

    HAL_Delay(10);

    // POWER UP + DAC START (PLAY MODE)
    txData[0] = 0x02;
    txData[1] = 0x9E; // Playback ON + HP enabled
    HAL_I2C_Master_Transmit(hi2c, CS43L22_I2C_ADDR, txData, 2, HAL_MAX_DELAY);

    HAL_Delay(100);

    // Ustawienia glosnosci
    txData[0] = 0x20; txData[1] = 0x18;
    HAL_I2C_Master_Transmit(hi2c, CS43L22_I2C_ADDR, txData, 2, HAL_MAX_DELAY);
    txData[0] = 0x21; txData[1] = 0x18;
    HAL_I2C_Master_Transmit(hi2c, CS43L22_I2C_ADDR, txData, 2, HAL_MAX_DELAY);
}


int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 50);
    return len;
}
// generacja probek sinusa do bufora

void Fill_Sine_Buffer(int16_t *buffer, int num_samples) {
    for (int i = 0; i < num_samples; i += 2) { // Krok co 2, bo stereo (L i R)
        float sample_val = 0.0f;

        if (current_freq > 0.0f) {
            // Obliczenia wartosci sinusa  A * sin(faza)
            sample_val = volume * sinf(phase_pos);

            // Zwiekszenie fazy
            phase_pos += (2.0f * PI * current_freq) / (float)SAMPLE_RATE;

            // Zeijanie fazy w zakresie 0..2PI, zeby nie rosla w nieskonczonosc
            if (phase_pos >= 2.0f * PI) {
                phase_pos -= 2.0f * PI;
            }
        } else {
            sample_val = 0.0f; // Cisza

        }


        buffer[i]     = (int16_t)sample_val; // Left
        buffer[i + 1] = (int16_t)sample_val; // Right
    }
}

// Callbacki DMA - wywolywane automatycznie, gdy SAI przesle polowe i calosc bufora
// Dzięki temu dzwiek jest ciągly i plynny.

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai) {

    Fill_Sine_Buffer(&tx_buffer[0], BUFFER_SIZE / 2);
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai) {

    Fill_Sine_Buffer(&tx_buffer[BUFFER_SIZE / 2], BUFFER_SIZE / 2);
}
//ping-pong caly bufor -> dac, po pierwszej polowie wywolany txhalf...
//napidanie pierwszej polowy nowymi probkami i po drugiej druga -> dma caly czas wysyla dzwiek ciagly
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  CS43L22_Init(&hi2c1);//inicjalizacja

    // Wstępne wypełnienie bufora ciszą lub pierwszą nutą
    Fill_Sine_Buffer(tx_buffer, BUFFER_SIZE);

    // Start DMA w trybie CIRCULAr
    if(HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t*)tx_buffer, BUFFER_SIZE) != HAL_OK)
    {
        Error_Handler();
    }
<<<<<<< HEAD
  char tab_liter[]={'C','D','E','F','G','A','H'};
  uint8_t index=0;
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
=======

    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
>>>>>>> 5f7130c3d8c57bc684bc235a55fc1d00f6fd25dc
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	HAL_GPIO_TogglePin(LD_G_GPIO_Port, LD_G_Pin);
	read_HCSR04();
<<<<<<< HEAD
	DisplayLetter(tab_liter[index]);
	index=(index+1)%7;
	HAL_Delay(500);
	printf("Distance: %li\r\n", distance);



  }
=======
	HAL_Delay(200);
	printf("Distance: %li\r\n", distance);//czesc asi

	// --- ODTWARZANIE GAMY ---


	    // C
	    current_freq = NOTE_C4;
	    HAL_Delay(500);

	    // D
	    current_freq = NOTE_D4;
	    HAL_Delay(500);

	    // E
	    current_freq = NOTE_E4;
	    HAL_Delay(500);

	    // F
	    current_freq = NOTE_F4;
	    HAL_Delay(500);

	    // G
	    current_freq = NOTE_G4;
	    HAL_Delay(500);

	    // A
	    current_freq = NOTE_A4;
	    HAL_Delay(500);

	    // H
	    current_freq = NOTE_H4;
	    HAL_Delay(500);

	    current_freq = NOTE_C5;
	   	    HAL_Delay(500);


	    //current_freq = 0;
	    //HAL_Delay(1000);
	  
	  }


>>>>>>> 5f7130c3d8c57bc684bc235a55fc1d00f6fd25dc
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
//void MX_DMA_Init(void)
//{
//  /* DMA controller clock enable */
//  __HAL_RCC_DMA2_CLK_ENABLE(); // SAI1 zazwyczaj używa DMA2
//
//  /* DMA interrupt init */
//  /* DMA2_Channel1_IRQn interrupt configuration */
//  // Ustawiamy priorytet i włączamy przerwanie w NVIC
//  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
//}
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
