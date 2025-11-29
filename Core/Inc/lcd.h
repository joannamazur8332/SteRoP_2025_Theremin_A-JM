/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lcd.h
  * @brief   This file contains all the function prototypes for
  *          the lcd.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCD_H__
#define __LCD_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern LCD_HandleTypeDef hlcd;

/* USER CODE BEGIN Private defines */
/*
 * TABELA ASCII
A	65	41	01000001
B	66	42	01000010
C	67	43	01000011
D	68	44	01000100
E	69	45	01000101
F	70	46	01000110
G	71	47	01000111
H	72	48	01001000
 */

//SEGMENTY w CubeIDE
#define MCU_LCD_SEG3_SHIFT    3
#define MCU_LCD_SEG4_SHIFT    4
#define MCU_LCD_SEG5_SHIFT    5
#define MCU_LCD_SEG6_SHIFT    6
#define MCU_LCD_SEG8_SHIFT    8
#define MCU_LCD_SEG9_SHIFT    9
#define MCU_LCD_SEG12_SHIFT   12
#define MCU_LCD_SEG13_SHIFT   13
#define MCU_LCD_SEG14_SHIFT   14
#define MCU_LCD_SEG15_SHIFT   15
#define MCU_LCD_SEG17_SHIFT   17
#define MCU_LCD_SEG22_SHIFT   22
#define MCU_LCD_SEG23_SHIFT   23
#define MCU_LCD_SEG24_SHIFT   24
#define MCU_LCD_SEG25_SHIFT   25
#define MCU_LCD_SEG26_SHIFT   26
#define MCU_LCD_SEG28_SHIFT   28
#define MCU_LCD_SEG29_SHIFT   29
#define MCU_LCD_SEG30_SHIFT   30
#define MCU_LCD_SEG31_SHIFT   31
#define MCU_LCD_SEG32_SHIFT   0
#define MCU_LCD_SEG33_SHIFT   1
#define MCU_LCD_SEG34_SHIFT   2
#define MCU_LCD_SEG35_SHIFT   3

#define MCU_LCD_SEG3          (1U << MCU_LCD_SEG3_SHIFT)
#define MCU_LCD_SEG4          (1U << MCU_LCD_SEG4_SHIFT)
#define MCU_LCD_SEG5          (1U << MCU_LCD_SEG5_SHIFT)
#define MCU_LCD_SEG6          (1U << MCU_LCD_SEG6_SHIFT)
#define MCU_LCD_SEG8          (1U << MCU_LCD_SEG8_SHIFT)
#define MCU_LCD_SEG9          (1U << MCU_LCD_SEG9_SHIFT)
#define MCU_LCD_SEG12         (1U << MCU_LCD_SEG12_SHIFT)
#define MCU_LCD_SEG13         (1U << MCU_LCD_SEG13_SHIFT)
#define MCU_LCD_SEG14         (1U << MCU_LCD_SEG14_SHIFT)
#define MCU_LCD_SEG15         (1U << MCU_LCD_SEG15_SHIFT)
#define MCU_LCD_SEG17         (1U << MCU_LCD_SEG17_SHIFT)
#define MCU_LCD_SEG22         (1U << MCU_LCD_SEG22_SHIFT)
#define MCU_LCD_SEG23         (1U << MCU_LCD_SEG23_SHIFT)
#define MCU_LCD_SEG24         (1U << MCU_LCD_SEG24_SHIFT)
#define MCU_LCD_SEG25         (1U << MCU_LCD_SEG25_SHIFT)
#define MCU_LCD_SEG26         (1U << MCU_LCD_SEG26_SHIFT)
#define MCU_LCD_SEG28         (1U << MCU_LCD_SEG28_SHIFT)
#define MCU_LCD_SEG29         (1U << MCU_LCD_SEG29_SHIFT)
#define MCU_LCD_SEG30         (1U << MCU_LCD_SEG30_SHIFT)
#define MCU_LCD_SEG31         (1U << MCU_LCD_SEG31_SHIFT)
#define MCU_LCD_SEG32         (1U << MCU_LCD_SEG32_SHIFT)
#define MCU_LCD_SEG33         (1U << MCU_LCD_SEG33_SHIFT)
#define MCU_LCD_SEG34         (1U << MCU_LCD_SEG34_SHIFT)
#define MCU_LCD_SEG35         (1U << MCU_LCD_SEG35_SHIFT)


//SEGMENTY w LCD
#define LCD_SEG0_SHIFT          MCU_LCD_SEG4_SHIFT
#define LCD_SEG1_SHIFT          MCU_LCD_SEG23_SHIFT
#define LCD_SEG2_SHIFT          MCU_LCD_SEG6_SHIFT
#define LCD_SEG3_SHIFT          MCU_LCD_SEG13_SHIFT
#define LCD_SEG4_SHIFT          MCU_LCD_SEG15_SHIFT
#define LCD_SEG5_SHIFT          MCU_LCD_SEG29_SHIFT
#define LCD_SEG6_SHIFT          MCU_LCD_SEG31_SHIFT
#define LCD_SEG7_SHIFT          MCU_LCD_SEG33_SHIFT
#define LCD_SEG8_SHIFT          MCU_LCD_SEG35_SHIFT
#define LCD_SEG9_SHIFT          MCU_LCD_SEG25_SHIFT
#define LCD_SEG10_SHIFT         MCU_LCD_SEG17_SHIFT
#define LCD_SEG11_SHIFT         MCU_LCD_SEG8_SHIFT
#define LCD_SEG12_SHIFT         MCU_LCD_SEG9_SHIFT
#define LCD_SEG13_SHIFT         MCU_LCD_SEG26_SHIFT
#define LCD_SEG14_SHIFT         MCU_LCD_SEG24_SHIFT
#define LCD_SEG15_SHIFT         MCU_LCD_SEG34_SHIFT
#define LCD_SEG16_SHIFT         MCU_LCD_SEG32_SHIFT
#define LCD_SEG17_SHIFT         MCU_LCD_SEG30_SHIFT
#define LCD_SEG18_SHIFT         MCU_LCD_SEG28_SHIFT
#define LCD_SEG19_SHIFT         MCU_LCD_SEG14_SHIFT
#define LCD_SEG20_SHIFT         MCU_LCD_SEG12_SHIFT
#define LCD_SEG21_SHIFT         MCU_LCD_SEG5_SHIFT
#define LCD_SEG22_SHIFT         MCU_LCD_SEG22_SHIFT
#define LCD_SEG23_SHIFT         MCU_LCD_SEG3_SHIFT

#define LCD_SEG0          MCU_LCD_SEG4
#define LCD_SEG1          MCU_LCD_SEG23
#define LCD_SEG2          MCU_LCD_SEG6
#define LCD_SEG3          MCU_LCD_SEG13
#define LCD_SEG4          MCU_LCD_SEG15
#define LCD_SEG5          MCU_LCD_SEG29
#define LCD_SEG6          MCU_LCD_SEG31
#define LCD_SEG7          MCU_LCD_SEG33
#define LCD_SEG8          MCU_LCD_SEG35
#define LCD_SEG9          MCU_LCD_SEG25
#define LCD_SEG10         MCU_LCD_SEG17
#define LCD_SEG11         MCU_LCD_SEG8
#define LCD_SEG12         MCU_LCD_SEG9
#define LCD_SEG13         MCU_LCD_SEG26
#define LCD_SEG14         MCU_LCD_SEG24
#define LCD_SEG15         MCU_LCD_SEG34
#define LCD_SEG16         MCU_LCD_SEG32
#define LCD_SEG17         MCU_LCD_SEG30
#define LCD_SEG18         MCU_LCD_SEG28
#define LCD_SEG19         MCU_LCD_SEG14
#define LCD_SEG20         MCU_LCD_SEG12
#define LCD_SEG21         MCU_LCD_SEG5
#define LCD_SEG22         MCU_LCD_SEG22
#define LCD_SEG23         MCU_LCD_SEG3

//COMy

#define MCU_LCD_COM0          LCD_RAM_REGISTER0
#define MCU_LCD_COM1          LCD_RAM_REGISTER2
#define MCU_LCD_COM2          LCD_RAM_REGISTER4
#define MCU_LCD_COM3          LCD_RAM_REGISTER6


//#if defined (USE_STM32L476G_DISCO_REVC) || defined (USE_STM32L476G_DISCO_REVB)
#define LCD_COM0          MCU_LCD_COM0
#define LCD_COM1          MCU_LCD_COM1
#define LCD_COM2          MCU_LCD_COM2
#define LCD_COM3          MCU_LCD_COM3

//tylko dla pierwszej cyfry wyświetlacza!!!

#define LCD_DIGIT1_COM0               LCD_COM0
#define LCD_DIGIT1_COM0_SEG_MASK      ~(LCD_SEG0 | LCD_SEG1 | LCD_SEG22 | LCD_SEG23)
#define LCD_DIGIT1_COM1               LCD_COM1
#define LCD_DIGIT1_COM1_SEG_MASK      ~(LCD_SEG0 | LCD_SEG1 | LCD_SEG22 | LCD_SEG23)
#define LCD_DIGIT1_COM2               LCD_COM2
#define LCD_DIGIT1_COM2_SEG_MASK      ~(LCD_SEG0 | LCD_SEG1 | LCD_SEG22 | LCD_SEG23)
#define LCD_DIGIT1_COM3               LCD_COM3
#define LCD_DIGIT1_COM3_SEG_MASK      ~(LCD_SEG0 | LCD_SEG1 | LCD_SEG22 | LCD_SEG23)
/* USER CODE END Private defines */

void MX_LCD_Init(void);

/* USER CODE BEGIN Prototypes */
void DisplayLetter(char znak);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __LCD_H__ */

