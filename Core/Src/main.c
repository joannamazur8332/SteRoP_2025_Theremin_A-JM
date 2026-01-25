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
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CS43L22_I2C_ADDR  0x94
#define SAMPLE_RATE       16000
#define PI                3.14159265f
// Definicje nut

// 2oktawa
#define NOTE_C2  65.41f
#define NOTE_Cis2 69.30f
#define NOTE_D2  73.42f
#define NOTE_Dis2 77.78f
#define NOTE_E2  82.41f
#define NOTE_F2  87.31f
#define NOTE_Fis2 92.50
#define NOTE_G2  98.00f
#define NOTE_Gis2 103.83
#define NOTE_A2  110.00f
#define NOTE_Ais2 116.54
#define NOTE_H2  123.47f
// 3 oktawa
#define NOTE_C3  130.81f
#define NOTE_Cis3 138.59f
#define NOTE_D3  146.83f
#define NOTE_Dis3 155.56f
#define NOTE_E3  164.81f
#define NOTE_F3  174.61f
#define NOTE_Fis3 185.00f
#define NOTE_G3  196.00f
#define NOTE_Gis3 207.65
#define NOTE_A3  220.00f
#define NOTE_Ais3 233.08f
#define NOTE_H3  246.94f
// 4 oktawa
#define NOTE_C4  261.63f
#define NOTE_Cis4 277.18
#define NOTE_D4 293.66f
#define NOTE_Dis4 311.13f
#define NOTE_E4  329.63f
#define NOTE_F4  349.99f
#define NOTE_Fis4 369.99f
#define NOTE_G4  392.00f
#define NOTE_Gis4 415.30f
#define NOTE_A4  440.00f
#define NOTE_Ais4 466.16f
#define NOTE_H4  493.88f
//5 oktawa
#define NOTE_C5 523.25f
#define NOTE_Cis5 554.37f
#define NOTE_D5 587.33f
#define NOTE_Dis5 622.25f
#define NOTE_E5 659.25f
#define NOTE_F5 698.46f
#define NOTE_FIs5 739.99f
#define NOTE_G5 783.99f
#define NOTE_Gis5 830.61f
#define NOTE_A5 880.00f
#define NOTE_Ais5 932.33f
#define NOTE_H5 987.77f
//6 oktawa
#define NOTE_C6 1046.50f


//wybrana ilosc skali
#define NUM_SCALES 4
#define NUM_NOTES 8
#define NUM_NOTES_PLUS 12
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


// Bufor audio
#define BUFFER_SIZE       2048
int16_t tx_buffer[BUFFER_SIZE];

// Zmienne syntezy
volatile float current_freq = 0.0f; //aktualna f dzwieku
float phase_pos = 0.0f; //obecna faza
float volume = 8000.0f; //do glosnosci
char tab_liter[]={'C','D','E','F','G','A','H', 'C'};

float notes[NUM_SCALES][NUM_NOTES]={
//inne skale
		{NOTE_C2,NOTE_D2 ,NOTE_E2, NOTE_F2 ,NOTE_G2 ,NOTE_A2, NOTE_H2, NOTE_C3},
		{NOTE_C3,NOTE_D3 ,NOTE_E3, NOTE_F3 ,NOTE_G3 ,NOTE_A3, NOTE_H3, NOTE_C4},
		{ NOTE_C4,NOTE_D4 ,NOTE_E4, NOTE_F4 ,NOTE_G4 ,NOTE_A4, NOTE_H4, NOTE_C5},
		{NOTE_C5,NOTE_D5 ,NOTE_E5, NOTE_F5 ,NOTE_G5 ,NOTE_A5, NOTE_H5, NOTE_C6}
};
float notesPlus[NUM_SCALES][NUM_NOTES_PLUS]={
		{NOTE_C2,NOTE_Cis2, NOTE_D2, NOTE_Dis2,NOTE_E2, NOTE_F2 ,NOTE_G2, NOTE_Gis2,NOTE_A2,NOTE_Ais2, NOTE_H2, NOTE_C3},
		{NOTE_C3,NOTE_Cis3, NOTE_D3, NOTE_Dis3,NOTE_E3, NOTE_F3 ,NOTE_G3, NOTE_Gis3,NOTE_A3,NOTE_Ais3, NOTE_H3, NOTE_C4},
		{NOTE_C4,NOTE_Cis4, NOTE_D4, NOTE_Dis4,NOTE_E4, NOTE_F4 ,NOTE_G4, NOTE_Gis4,NOTE_A4,NOTE_Ais4, NOTE_H4, NOTE_C5},
		{NOTE_C5,NOTE_Cis5, NOTE_D5, NOTE_Dis5,NOTE_E5, NOTE_F5 ,NOTE_G5, NOTE_Gis5,NOTE_A5,NOTE_Ais5, NOTE_H5, NOTE_C6}
		};
uint32_t min_distance=3;
uint32_t max_distance=24;
uint32_t max_distance_plus=36;
uint32_t last_distance=0;
int current_scale=0;
volatile int autotune=1;
volatile int state=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void CS43L22_Init(I2C_HandleTypeDef *hi2c);
int _write(int file, char *ptr, int len);
void Fill_Sine_Buffer(int16_t *buffer, int num_samples);
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai);
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai);
void Autotune_play(int numer_skali);
void play(int numer_skali);
void Gama(int numer_skali);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
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


void play(int numer_skali)
{
    if (numer_skali >= NUM_SCALES || numer_skali < 0) return;
    if (abs(last_distance - distance) < 3) {
        distance = last_distance;
    }
    else {
        last_distance = distance;
    }

    int index;
    if (distance >= max_distance_plus)
        index = NUM_NOTES_PLUS - 1;
    else if (distance <= min_distance)
        index = 0;
    else
        index = (distance - min_distance) *
                (NUM_NOTES_PLUS - 1) /
                (max_distance_plus - min_distance);

    current_freq = notesPlus[numer_skali][index];
    HAL_LCD_Clear(&hlcd);
}

void Autotune_play(int numer_skali){
	if(numer_skali>NUM_SCALES || numer_skali<0)return;
	if(abs(last_distance-distance)<3){
		 distance = last_distance;
	}
	else{
		last_distance = distance;
	}
	int index;
	if(distance>max_distance)index=NUM_NOTES-1;
	else if(distance<min_distance)index =0;
	else index = (distance-min_distance)/3;
	current_freq = notes[numer_skali][index];
	DisplayLetter(tab_liter[index]);
}


void Gama(int numer_skali){
	for(int i=0; i<8; i++){
		current_freq=notes[numer_skali][i];
		HAL_Delay(500);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == JOY_Center_Pin) {
		state = !state;
	}
	else if(GPIO_Pin == JOY_Up_Pin){
		if(current_scale<NUM_SCALES-1){
			current_scale++;
		}
	}
	else if(GPIO_Pin == JOY_Down_Pin){
		if(current_scale>0){
			current_scale-=1;
		}
	}
	else if(GPIO_Pin == JOY_Right_Pin){
		autotune=1;
	}
	else if(GPIO_Pin == JOY_Left_Pin){
		autotune=0;
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


  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);

  uint32_t last_measure_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	if(state){
		if(HAL_GetTick()-last_measure_time>=50){
			read_HCSR04();
			last_measure_time=HAL_GetTick();
		}

		printf("Distance: %li\r\n", distance);
		if(autotune){
			Autotune_play(current_scale);
		}
		else play(current_scale);
		HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, RESET);
	}
	else{
		HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, SET);
		current_freq = 0.0f; //zatrzymanie dźwięku - pauza
	}


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
