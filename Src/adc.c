/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc6;
ADC_HandleTypeDef hadc7;

/* ADC init function */
void MX_ADC6_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc6.Instance = ADC1;
  hadc6.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc6.Init.Resolution = ADC_RESOLUTION_12B;
  hadc6.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc6.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc6.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc6.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc6.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc6.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc6.Init.ContinuousConvMode = ENABLE;
  hadc6.Init.NbrOfConversion = 1;
  hadc6.Init.DiscontinuousConvMode = DISABLE;
  hadc6.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc6.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc6.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc6) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc6, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

void MX_ADC7_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc7.Instance = ADC1;
  hadc7.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc7.Init.Resolution = ADC_RESOLUTION_12B;
  hadc7.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc7.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc7.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc7.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc7.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc7.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc7.Init.ContinuousConvMode = ENABLE;
  hadc7.Init.NbrOfConversion = 1;
  hadc7.Init.DiscontinuousConvMode = DISABLE;
  hadc7.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc7.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc7.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc7) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc7, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC GPIO Configuration
    PA6     ------> ADC_IN6
    PA7     ------> ADC_IN7
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC GPIO Configuration
    PA6     ------> ADC_IN6
    PA7     ------> ADC_IN7
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
