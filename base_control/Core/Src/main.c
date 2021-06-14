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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_def.h"
#include "PID.h"
#include "config.h"
#include "frame_resolve.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FALSE   0
#define TRUE    1

#define DEBUG
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Arrays
uint8_t MSG[50] = "Init";
uint8_t period[200] = "\n";
uint8_t encoder_f[20] = "\n";
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t tx_buffer[TX_BUFFER_SIZE];

// Cross processes values 

#ifdef TEST_HARDWARE
volatile uint16_t cnt = 0;
volatile uint16_t last_cnt = 0;
volatile double enc_cnt = 0.0f;
volatile double last_enc_cnt = 0.0f;
#endif

#ifdef ENABLE_PID
/* PID Init Variables */

volatile uint32_t *RIGHT_DUTY_ADDR = &(TIM4->CCR3);
volatile uint32_t *RIGHT_ENCODER_ADDR = &(TIM3->CNT);


double right_set_speed = 0.0f; // RPM
double left_set_speed = 0.0f; // RPM
double right_pid_params[3] = {RIGHT_MOTOR_KP , RIGHT_MOTOR_KI, RIGHT_MOTOR_KD};
double left_pid_params[3] = {LEFT_MOTOR_KP , LEFT_MOTOR_KI, LEFT_MOTOR_KD};


// Custom typedef
MOTOR_TypeDef str_right_motor;
PID_TypeDef str_right_pid;
MOTOR_TypeDef str_left_motor;
PID_TypeDef str_left_pid;

#endif
/* Other definitions*/
uint8_t ready_flag = FALSE;
volatile uint8_t    vel_update_flag = FALSE;
float linear_vel;
float angular_vel;
float last_l_vel, last_a_vel;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void dma_rx_cplt(void);
void dma_rx_half_cplt(void);
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
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_DMA(&huart1, rx_buffer, 1);

  #ifdef ENABLE_PID

  PID_MotorInit(&str_right_motor, &str_right_pid, GPIOB, GPIO_PIN_6 , GPIO_PIN_7, 100.0f, 10.0f, &(TIM3->CNT), &(TIM4->CCR3), right_pid_params);
  PID_MotorInit(&str_left_motor, &str_left_pid, GPIOA, GPIO_PIN_6 , GPIO_PIN_7, 100.0f, 10.0f, &(TIM2->CNT), &(TIM4->CCR4), left_pid_params);

  // Start Timer4 for PWM function on channel 3 and channel 4
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  // Reset PWM Duty Cycle to zero on TIM 4 output channel 3 and channel 4
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
  
  // Start Timer3 for RIGHT Encoder Interfacing.
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  // Start Timer2 for LEFT Encoder Interfacing.
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  // Start Timer5 for sampling loop interrupt
  HAL_TIM_Base_Start_IT(&htim5);

  #endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (vel_update_flag)
    {
        resolveRxFrame(rx_buffer, &linear_vel, &angular_vel);
        #ifdef DEBUG
        if (linear_vel != last_l_vel || angular_vel != last_a_vel)
        {
            sprintf((char *)MSG, "Linear: %f | Angular: %f \n", linear_vel, angular_vel);
            HAL_UART_Transmit_DMA(&huart1, MSG, sizeof(MSG));
        }
        #endif
        last_a_vel = angular_vel;
        last_l_vel = linear_vel;
        vel_update_flag = FALSE;
    }
    HAL_Delay(100);
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim5)
  {
    #ifdef TEST_HARDWARE
    // Do sampling and calculate stuff
    cnt++;
    if (cnt == 100)
    {
        
        enc_cnt = TIM3->CNT;
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
        if (enc_cnt!=last_enc_cnt)
        {
          sprintf((char *)period, "-> %d pulses/sec\n", (enc_cnt-last_enc_cnt));
          HAL_UART_Transmit_DMA(&huart1, period, sizeof(period));
        }
        cnt = 0;
        last_enc_cnt = enc_cnt;
        
    }
    #endif

    #ifdef ENABLE_RIGHT_MOTOR
    /* =================
    * MAIN PID CLOSE-LOOP SAMPLING AND COMPUTE
    *  =================== */
    // RIGHT MOTOR PI CONTROL 
    PID_PreProcess(&str_right_motor, right_set_speed);
    PID_ComputeOutput(&str_right_motor);
    PID_SetDuty(&str_right_motor );
    PID_PreProcess(&str_left_motor, left_set_speed);
    PID_ComputeOutput(&str_left_motor);
    PID_SetDuty(&str_left_motor );
    #else 
    __NOP();
    #endif
  }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static uint8_t prev_rx, last_rx;
    
    if (ready_flag == TRUE)
    {
        // Do things
        sprintf( (char *) MSG, (char *) rx_buffer, RX_BUFFER_SIZE);
        HAL_UART_Transmit_DMA(&huart1, MSG, RX_BUFFER_SIZE);
        vel_update_flag = TRUE;
        ready_flag = FALSE;
        HAL_UART_Receive_DMA(&huart1, rx_buffer, 1);
    }
    else
    {
        if ((rx_buffer[0] | prev_rx | last_rx) == 0x16) // SYN
        {
            ready_flag = TRUE;
            memset(rx_buffer, 0, RX_BUFFER_SIZE);
            HAL_UART_Receive_DMA(&huart1, rx_buffer, RX_BUFFER_SIZE);   
        }
        else
        {
            HAL_UART_Receive_DMA(&huart1, rx_buffer, 1);
        }
    } 
    last_rx = prev_rx;
    prev_rx = rx_buffer[0];
}

void dma_rx_cplt()
{
    __NOP();
}

void dma_rx_half_cplt()
{
    __NOP();
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
