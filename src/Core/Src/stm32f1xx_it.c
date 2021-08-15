/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "at24c256.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
int valuex = 0;	
int valuey = 0;
int valuez = 0;
uint8_t transmitData[100];
int array_index=0;
int mpu_time=0;
int uart_time=0;
extern uint16_t adc_value;
extern I2C_HandleTypeDef hi2c1;
#define MPU6050_ADDR 0xD0
#define	ACCEL_XOUT_H_REG 0x3B
extern int16_t Accel_X_RAW;
extern int16_t Accel_Y_RAW;
extern int16_t Accel_Z_RAW;

extern float Ax,Ay,Az;

#define device_addres 0xA0 // slave add
int eeprom_available=0;

extern uint8_t getUsartData[50];



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MPU6050_Read_Accell (void)
{
uint8_t Rx_data[6];
// Read 6 BYTES of data starting from ACCEL_XOUT_H register
HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG,1, Rx_data, 6, 1000);
Accel_X_RAW = (int16_t)(Rx_data[0] << 8 | Rx_data [1]);
Accel_Y_RAW = (int16_t)(Rx_data[2] << 8 | Rx_data [3]);
Accel_Z_RAW = (int16_t)(Rx_data[4] << 8 | Rx_data [5]);
Ax = Accel_X_RAW/16384.0;
Ay = Accel_Y_RAW/16384.0;
Az = Accel_Z_RAW/16384.0;
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC1 and ADC2 global interrupts.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */
  adc_value = HAL_ADC_GetValue(&hadc1);
  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */


int arr[20];
int arr_i = 0;
int j, r;  
void write_int_to_array(int t){
	
	if(t<0){
		transmitData[array_index++]='-';
		t=-t;
  }
	
	while (t != 0) {
  
	r = t % 10;

	arr[arr_i] = r;
	arr_i++;
	t = t / 10;
	}

	for (j = arr_i - 1; j > -1; j--) {
		transmitData[array_index++]=(uint8_t)arr[j]+48;
	}

	 arr_i=0;
	 r=0;
	 j=0;
}




int adc_arr[20];
int adc_i = 0;
int adc_j, adc_r;  
void write_adc_to_array(uint16_t t){
	
	while (t != 0) {
  
	adc_r = t % 10;

	adc_arr[adc_i] = adc_r;
	adc_i++;
	t = t / 10;
	}

	for (adc_j = adc_i - 1; adc_j > -1; adc_j--) {
       transmitData[array_index++]=(uint16_t)adc_arr[adc_j]+48;
   }
	adc_i=0;
	adc_j=0;
	adc_r=0;	
}

void write_to_array(){	
transmitData[array_index++]='#';
transmitData[array_index++]='x';
transmitData[array_index++]='=';
write_int_to_array(valuex);
transmitData[array_index++]=',';

transmitData[array_index++]='y';
transmitData[array_index++]='=';
write_int_to_array(valuey);
transmitData[array_index++]=',';
		
transmitData[array_index++]='z';
transmitData[array_index++]='=';
write_int_to_array(valuez);
transmitData[array_index++]='#';	
	
transmitData[array_index++]='*';
transmitData[array_index++]='a';
transmitData[array_index++]='d';
transmitData[array_index++]='c';
transmitData[array_index++]='=';
write_adc_to_array(adc_value);
transmitData[array_index++]='*';
transmitData[array_index++]='-';		
transmitData[array_index++]='\n';	
}

void TIM3_IRQHandler(void)
{
	if(mpu_time==2){
		MPU6050_Read_Accell();
		valuex = 100 * Ax;	
    valuey = 100 * Ay;
		valuez = 100 * Az;
		write_to_array();
		eeprom_available=1;
		mpu_time=0;
	}
	else{
		mpu_time++;
	}
	
  if(uart_time==6){
		HAL_UART_Transmit_IT(&huart2,(uint8_t*) &transmitData,array_index);
		for(int temp_i=0;temp_i<1000000;temp_i++){int a=1;};
		array_index=0;
		transmitData[0]=0;
		uart_time=0;
	}
	else{
		uart_time++;
	}
  /* USER CODE BEGIN TIM3_IRQn 0 */
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	HAL_UART_Receive_IT(&huart2,(uint8_t*) &getUsartData, 50);
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
