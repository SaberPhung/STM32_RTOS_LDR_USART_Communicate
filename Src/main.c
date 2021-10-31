/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#define Transmit // Read when read data from sensor Transmit when send data to master

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Task Stack Size */
#define APP_TASK_START_STK_SIZE 128u
#define READ_DATA_TASK_STK_SIZE 128u
#define TRANSMIT_DATA_TASK_STK_SIZE 256u
#define BLINK_TASK_STK_SIZE 128u
/* Task Priority */
#define APP_TASK_START_PRIO 1u
#define READ_DATA_TASK_PRIO 2u
#define TRANSMIT_DATA_TASK_PRIO 3u
#define BLINK_TASK_PRIO 4u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Task Control Block */
static OS_TCB AppTaskStartTCB;
static OS_TCB ReadDataTaskTCB;
static OS_TCB TransmitDataTaskTCB;
static OS_TCB BlinkTaskTCB;
/* Task Stack */
static CPU_STK AppTaskStartStk[APP_TASK_START_STK_SIZE];
static CPU_STK ReadDataTaskStk[READ_DATA_TASK_STK_SIZE];
static CPU_STK TransmitDataTaskStk[TRANSMIT_DATA_TASK_STK_SIZE];
static CPU_STK BlinkTaskStk[BLINK_TASK_STK_SIZE];
/* Semaphore */
OS_SEM sem;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void AppTaskStart(void *p_arg);
static void ReadDataTask(void *p_arg);
static void TransmitDataTask(void *p_arg);
static void BlinkTask(void *p_arg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int main(void)
{
  /* To store error code */
  OS_ERR os_err;

  /* Initialize uC/OS-III */
  OSInit(&os_err);

  if (os_err != OS_ERR_NONE)
  {
    while (DEF_TRUE)
      ;
  }

  OSSemCreate(
      (OS_SEM *)&sem,
      (CPU_CHAR *)"Semaphore",
      (OS_SEM_CTR)0,
      (OS_ERR *)&os_err);

  OSTaskCreate(
      /* pointer to task control block */
      (OS_TCB *)&AppTaskStartTCB,
      /* task name can be displayed by debuggers */
      (CPU_CHAR *)"App Task Start",
      /* pointer to the task */
      (OS_TASK_PTR)AppTaskStart,
      /* pointer to an OPTIONAL data area */
      (void *)0,
      /* task priority: the lower the number, the higher the priority */
      (OS_PRIO)APP_TASK_START_PRIO,
      /* pointer to task's stack base addr */
      (CPU_STK *)&AppTaskStartStk[0],
      /* task's stack limit to monitor and ensure that the stack 
       * doesn't overflow (10%) */
      (CPU_STK_SIZE)APP_TASK_START_STK_SIZE / 10,
      /* task's stack size */
      (CPU_STK_SIZE)APP_TASK_START_STK_SIZE,
      /* max number of message that the task can receive through 
       * internal message queue (5) */
      (OS_MSG_QTY)5u,
      /* amount of clock ticks for the time quanta 
       * when round robin is enabled */
      (OS_TICK)0u,
      /* pointer to an OPTIONAL user-supplied memory location 
       * use as a TCB extension */
      (void *)0,
      /* contain task-specific option 
       * OS_OPT_TASK_STK_CHK: allow stack checking 
       * OS_OPT_TASK_STK_CLR: stack needs to be cleared */
      (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
      /* pointer to a variable that will receive an error code */
      (OS_ERR *)&os_err);

  if (os_err != OS_ERR_NONE)
  {
    while (DEF_TRUE)
      ;
  }

  /* Start Mulitasking */
  OSStart(&os_err);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void AppTaskStart(void *p_arg)
{
  OS_ERR os_err;

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_ADC6_Init();

  OSTaskCreate(
      (OS_TCB *)&BlinkTaskTCB,
      (CPU_CHAR *)"Blink Task",
      (OS_TASK_PTR)BlinkTask,
      (void *)0,
      (OS_PRIO)BLINK_TASK_PRIO,
      (CPU_STK *)&BlinkTaskStk[0],
      (CPU_STK_SIZE)BLINK_TASK_STK_SIZE / 10,
      (CPU_STK_SIZE)BLINK_TASK_STK_SIZE,
      (OS_MSG_QTY)5u,
      (OS_TICK)0u,
      (void *)0,
      (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
      (OS_ERR *)&os_err);
#ifdef Read
  OSTaskCreate(
      (OS_TCB *)&ReadDataTaskTCB,
      (CPU_CHAR *)"Read Data Task",
      (OS_TASK_PTR)ReadDataTask,
      (void *)0,
      (OS_PRIO)READ_DATA_TASK_PRIO,
      (CPU_STK *)&ReadDataTaskStk[0],
      (CPU_STK_SIZE)READ_DATA_TASK_STK_SIZE / 10,
      (CPU_STK_SIZE)READ_DATA_TASK_STK_SIZE,
      (OS_MSG_QTY)5u,
      (OS_TICK)0u,
      (void *)0,
      (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
      (OS_ERR *)&os_err);
#endif
#ifdef Transmit
  OSTaskCreate(
      (OS_TCB *)&TransmitDataTaskTCB,
      (CPU_CHAR *)"Transmit Data Task",
      (OS_TASK_PTR)TransmitDataTask,
      (void *)0,
      (OS_PRIO)TRANSMIT_DATA_TASK_PRIO,
      (CPU_STK *)&TransmitDataTaskStk[0],
      (CPU_STK_SIZE)TRANSMIT_DATA_TASK_STK_SIZE / 10,
      (CPU_STK_SIZE)TRANSMIT_DATA_TASK_STK_SIZE,
      (OS_MSG_QTY)5u,
      (OS_TICK)0u,
      (void *)0,
      (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
      (OS_ERR *)&os_err);
#endif
  OSSemPost(
    (OS_SEM*)&sem,
    (OS_OPT)OS_OPT_POST_1,
    (OS_ERR*)&os_err
  );
}

static void ReadDataTask(void *p_arg)
{
  HAL_ADC_Start(&hadc6);
  OS_ERR os_err;
  while (DEF_TRUE)
  {
    OSSemPend(
        (OS_SEM *)&sem,
        (OS_TICK)0,
        (OS_OPT)OS_OPT_PEND_BLOCKING,
        (CPU_TS *)NULL,
        (OS_ERR *)&os_err);
    read_ldr_sensor();
    OSSemPost(
        (OS_SEM *)&sem,
        (OS_OPT)OS_OPT_POST_1,
        (OS_ERR *)&os_err);
    OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &os_err);
  }
}

static void TransmitDataTask(void *p_arg)
{
  OS_ERR os_err;

  while (DEF_TRUE)
  {
    OSSemPend(
        (OS_SEM *)&sem,
        (OS_TICK)0,
        (OS_OPT)OS_OPT_PEND_BLOCKING,
        (CPU_TS *)NULL,
        (OS_ERR *)&os_err);
    ldr_print(&huart3);
    OSSemPost(
        (OS_SEM *)&sem,
        (OS_OPT)OS_OPT_POST_1,
        (OS_ERR *)&os_err);
    OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &os_err);
  }
}
static void BlinkTask(void *p_arg)
{
  OS_ERR os_err;
  unsigned char MSG[] = "\rBlink Task \n\r";
  while (DEF_TRUE)
  {
    OSSemPend(
        (OS_SEM *)&sem,
        (OS_TICK)0,
        (OS_OPT)OS_OPT_PEND_BLOCKING,
        (CPU_TS *)NULL,
        (OS_ERR *)&os_err);
    HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &os_err);
    OSSemPost(
        (OS_SEM *)&sem,
        (OS_OPT)OS_OPT_POST_1,
        (OS_ERR *)&os_err);
  }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
