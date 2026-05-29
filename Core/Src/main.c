/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ThrustCurve.h"
#include "DRV8871.h"
#include "INA226.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    STATE_WAIT_FOR_HEADER1,
    STATE_WAIT_FOR_HEADER2,
    STATE_READ_TYPE,
    STATE_READ_DATA,
    STATE_READ_CHECKSUM,
    STATE_READ_TAIL1,
    STATE_READ_TAIL2
} SerialState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HEADER_BYTE1 0xFA
#define HEADER_BYTE2 0xAF
#define TAIL_BYTE1 0xFB
#define TAIL_BYTE2 0xBF
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t uart3_rx_buffer;
SerialState serial_state;
uint8_t command_type;
float y_thrust, x_thrust, z_thrust, yaw_thrust;
float A_pwm, B_pwm, C_pwm, D_pwm;
uint8_t calculated_checksum;
uint8_t angle;
uint8_t data_buf[17];
uint8_t data_read_index = 0;
uint8_t transmit_buffer[15];
uint16_t sg1_pwm;
DRV8871_HandleTypeDef motorA, motorB, motorC, motorD;
float current;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Process_Serial_Data(uint8_t rx_data)
{
  switch(serial_state)
  {
    case STATE_WAIT_FOR_HEADER1:
      if(rx_data == HEADER_BYTE1)
      {
        serial_state = STATE_WAIT_FOR_HEADER2;
      }
      break;
    case STATE_WAIT_FOR_HEADER2:
      if(rx_data == HEADER_BYTE2)
      {
        serial_state = STATE_READ_TYPE;
      }
      else
      {
        serial_state = STATE_WAIT_FOR_HEADER1;
      }
      break;
    case STATE_READ_TYPE:
      if(rx_data == 0x52)
      {
        command_type = rx_data;
        serial_state = STATE_READ_TAIL1;
      }
      else if(rx_data == 0x49)
      {
        command_type = rx_data;
        data_read_index = 0;
        serial_state = STATE_READ_DATA;
      }
      else
      {
        serial_state = STATE_WAIT_FOR_HEADER1;
      }
      break;
    case STATE_READ_DATA:
      data_buf[data_read_index++] = rx_data;
      if(data_read_index >= 17)
      {
        serial_state = STATE_READ_CHECKSUM;
      }
      break;
    case STATE_READ_CHECKSUM:
      calculated_checksum = command_type;
      for(int i = 0; i < 17; i++)
      {
        calculated_checksum ^= data_buf[i];
      }
      if(rx_data == calculated_checksum)
      {
        serial_state = STATE_READ_TAIL1;
      }
      else
      {
        serial_state = STATE_WAIT_FOR_HEADER1;
      }
      break;
    case STATE_READ_TAIL1:
      if(rx_data == TAIL_BYTE1)
      {
        serial_state = STATE_READ_TAIL2;
      }
      else
      {
        serial_state = STATE_WAIT_FOR_HEADER1;
      }
      break;
    case STATE_READ_TAIL2:
      if(rx_data == TAIL_BYTE2)
      {
        if(command_type == 0x52)
        {
          INA226_ReadInfo();
          current = INA226_Info.current;
          memcpy(&transmit_buffer[8], &current, 4);
          uint8_t checksum = 0;
          for(int i = 2; i < 12; i++)
          {
            checksum ^= transmit_buffer[i];
          }
          transmit_buffer[12] = checksum;
          HAL_UART_Transmit_IT(&huart3, transmit_buffer, sizeof(transmit_buffer));
        }
        else if(command_type == 0x49)
        {
          memcpy(&y_thrust,  &data_buf[0],  4);
          memcpy(&x_thrust,  &data_buf[4],  4);
          memcpy(&z_thrust,  &data_buf[8],  4);
          memcpy(&yaw_thrust, &data_buf[12], 4);
          angle = data_buf[16];
          // A左 B右 提供Y轴前进力(正方向向前) + Yaw轴偏航差动(力臂100mm)
          // C后 D前 提供Z轴升降力(正方向向下)
          // X轴预留，不参与分配
          // yaw_thrust为扭矩(N·m)，除以力臂0.1m换算为单侧差动力(N)
          float yaw_force = yaw_thrust / 0.1f;
          float motorAF = y_thrust / 2.0f + yaw_force;
          float motorBF = y_thrust / 2.0f - yaw_force;
          float motorCF = z_thrust / 2.0f;
          float motorDF = z_thrust / 2.0f;
          A_pwm = 8 * PWM_FromThrust(motorAF);
          B_pwm = 8 * PWM_FromThrust(motorBF);
          C_pwm = 8 * PWM_FromThrust(motorCF);
          D_pwm = 8 * PWM_FromThrust(motorDF);
          DRV8871_SetSpeed(&motorA, A_pwm);
          DRV8871_SetSpeed(&motorB, B_pwm);
          DRV8871_SetSpeed(&motorC, C_pwm);
          DRV8871_SetSpeed(&motorD, D_pwm);
          sg1_pwm = (uint16_t)(50.0f + angle / 256.0f * 200.0f + 0.5f);  // 四舍五入
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, sg1_pwm);
        }
      }
      serial_state = STATE_WAIT_FOR_HEADER1;
      break;
    default:
      serial_state = STATE_WAIT_FOR_HEADER1;
      break;
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart==&huart3)
  {
    Process_Serial_Data(uart3_rx_buffer);
    HAL_UART_Receive_IT(&huart3,&uart3_rx_buffer,1);
  
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  INA226_Init();
  transmit_buffer[0] = HEADER_BYTE1;
  transmit_buffer[1] = HEADER_BYTE2;
  transmit_buffer[2] = 0x53;
  transmit_buffer[3] = 0;
  transmit_buffer[4] = 0;
  transmit_buffer[5] = 0;
  transmit_buffer[6] = 0;
  transmit_buffer[7] = 0;
  transmit_buffer[13] = TAIL_BYTE1;
  transmit_buffer[14] = TAIL_BYTE2;
  DRV8871_Init(&motorA, &htim2, TIM_CHANNEL_1, TIM_CHANNEL_2);
  DRV8871_Init(&motorB, &htim2, TIM_CHANNEL_3, TIM_CHANNEL_4);
  DRV8871_Init(&motorC, &htim3, TIM_CHANNEL_4, TIM_CHANNEL_3);
  DRV8871_Init(&motorD, &htim3, TIM_CHANNEL_2, TIM_CHANNEL_1);
  DRV8871_Start(&motorA);
  DRV8871_Start(&motorB);
  DRV8871_Start(&motorC);
  DRV8871_Start(&motorD);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_UART_Receive_IT(&huart3, &uart3_rx_buffer, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
