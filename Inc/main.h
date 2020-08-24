/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM_MAX 0xFF
#define ENC2_SIGB_Pin GPIO_PIN_0
#define ENC2_SIGB_GPIO_Port GPIOC
#define ENC2_SIGA_Pin GPIO_PIN_1
#define ENC2_SIGA_GPIO_Port GPIOC
#define ENC1_SIGA_Pin GPIO_PIN_0
#define ENC1_SIGA_GPIO_Port GPIOA
#define ENC1_SIGB_Pin GPIO_PIN_1
#define ENC1_SIGB_GPIO_Port GPIOA
#define BRAKE_Pin GPIO_PIN_4
#define BRAKE_GPIO_Port GPIOA
#define DIR_B_Pin GPIO_PIN_5
#define DIR_B_GPIO_Port GPIOA
#define DIR_A_Pin GPIO_PIN_6
#define DIR_A_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_7
#define SPI_CS_GPIO_Port GPIOA
#define TEMP_ALERT_Pin GPIO_PIN_0
#define TEMP_ALERT_GPIO_Port GPIOB
#define PWM_A_Pin GPIO_PIN_1
#define PWM_A_GPIO_Port GPIOB
#define PWM_A_passive_Pin GPIO_PIN_10
#define PWM_A_passive_GPIO_Port GPIOB
#define SPI_MISO_Pin GPIO_PIN_7
#define SPI_MISO_GPIO_Port GPIOC
#define SPI_CLK_Pin GPIO_PIN_9
#define SPI_CLK_GPIO_Port GPIOA
#define ENC1_SIGA_PASSIVE_Pin GPIO_PIN_10
#define ENC1_SIGA_PASSIVE_GPIO_Port GPIOA
#define PWM_B_Pin GPIO_PIN_4
#define PWM_B_GPIO_Port GPIOB
#define SPI_MOSI_Pin GPIO_PIN_6
#define SPI_MOSI_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
