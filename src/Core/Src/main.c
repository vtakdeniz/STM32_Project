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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define device_addres 0xA0 // slave add
#include "at24c256.h"
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t adc_value;
HAL_StatusTypeDef ret;

#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19

#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define	ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define	GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

int16_t Accel_X_RAW=0;
int16_t Accel_Y_RAW=0;
int16_t Accel_Z_RAW=0;

int16_t Gyro_X_RAW=0;
int16_t Gyro_Y_RAW=0;
int16_t Gyro_Z_RAW=0;

float Ax,Ay,Az,Gx,Gy,Gz;

uint8_t retvalmain[50];

int valx=0;
int valy=0;

int eeprom_adress=0;
uint8_t receiveDataArr[50];
extern int mpu_time;
extern int eeprom_available;

int getUsartDataLength=50;
uint8_t getUsartData[50];
int getUsartIndex=0;

uint8_t received_data;

uint8_t nums1_arr[50];
int nums1index=0;

uint8_t nums2_arr[50];
int nums2index=0;
uint8_t calc_operator;
int maxUsartLength=49;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void MPU6050_Init(void){
uint8_t check, Data;
// check device id
HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG,1, &check, 1, 1000);
//if (check == 0x68){ // boyle bir cihaz varsa
// power management regisgter 0x6b power up
Data = 0;
HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG,1, &Data, 1, 1000);
//
Data = 0x07;
HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG,1, &Data, 1, 1000);
// set acc. meter configuration in ACCEL_CONFIG reg.
Data = 0x00;
HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG,1, &Data, 1, 1000);
// Set Gyroscopic configuration in GYRO_CONFIG Register
// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ? 250 ?/s
Data = 0x00;
HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
//}//
}



void MPU6050_Read_Accel (void)
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

void MPU6050_Read_Gyro (void)
{
uint8_t Rx_data[6];
// read 6 bytes of data starting from GYRO_XOUT_H reg.
HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG,1, Rx_data, 6, 1000);
Gyro_X_RAW = (int16_t)(Rx_data[0] << 8 | Rx_data [1]);Gyro_Y_RAW = (int16_t)(Rx_data[2] << 8 | Rx_data [3]);
Gyro_Z_RAW = (int16_t)(Rx_data[4] << 8 | Rx_data [5]);
Gx = Gyro_X_RAW/131.0;
Gy = Gyro_Y_RAW/131.0;
Gz = Gyro_Z_RAW/131.0;
}


uint8_t value;
float tempval;
void write_to_eeprom(){
	tempval=(255.0/4096)*adc_value;
	value = (uint8_t) tempval;
	_writeEEPROM(&hi2c1,device_addres,eeprom_adress,(uint8_t)value);
	receiveDataArr[eeprom_adress]=_readEEPROM(&hi2c1 , device_addres, eeprom_adress);
	eeprom_adress++;
}

int get_digit_value(int digit_count, int number){
	int base=1;
	for(int i=0;i<digit_count;i++){
		base=base*10;
	};
	return base*number;
}

int get_num(uint8_t arr[50],int arr_index){
		int number=0;
		for(int i=0;i<arr_index;i++){
			number+=get_digit_value(arr_index-i-1,arr[i]-48);
		}
		return number;
}

int number_to_send=0;
int temp1data=0;
int temp2data=0;
uint8_t send_arr[20];
void execute_calculation(uint8_t arr1[50],uint8_t arr2[50],int arr1index,int arr2index,uint8_t calc_operator){
	  
		int arr_i=0;
		int arr_to_send_i=0;	
		int j, r;  
		int arr[20];
	
		if(calc_operator=='+'){
			number_to_send = get_num(arr1,arr1index)+get_num(arr2,arr2index);
		}
		else if(calc_operator=='-'){
			number_to_send = get_num(arr1,arr1index)-get_num(arr2,arr2index);
		}
		else if(calc_operator=='/'){
			number_to_send = get_num(arr1,arr1index)/get_num(arr2,arr2index);
		}
		else if(calc_operator=='*'){
			number_to_send = get_num(arr1,arr1index)*get_num(arr2,arr2index);
		}
		
		if(number_to_send==0){
		 arr_to_send_i++;
			send_arr[0]=0;
		}
		if(number_to_send<0){
		    send_arr[arr_to_send_i++]='-';
				number_to_send=-number_to_send;
		}
		
	  while (number_to_send != 0) {
				r = number_to_send % 10;
				arr[arr_i] = r;
				arr_i++;
				number_to_send = number_to_send / 10;
				}
				
				for (j = arr_i - 1; j > -1; j--) {
					send_arr[arr_to_send_i++]=(uint8_t)arr[j]+48;
				}
				send_arr[arr_to_send_i++]='\n';
				HAL_UART_Transmit_IT(&huart2,(uint8_t*) &send_arr,arr_to_send_i);
				 arr_i=0;
				 r=0;
				 j=0;
				 arr_to_send_i=0;
}

int increase_uart(){
uint8_t data = getUsartData[getUsartIndex];
		if (getUsartIndex>=maxUsartLength){
			getUsartData[getUsartIndex]=0;
			getUsartIndex=0;
		}
		else getUsartData[getUsartIndex++]=0;
		return data;
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
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
		/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
	
	HAL_TIM_Base_Start_IT(&htim3);
	MPU6050_Init();



	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	
	
  while (1)
  {
		
		HAL_ADC_Start(&hadc1);
		HAL_Delay(100);
		adc_value=HAL_ADC_GetValue(&hadc1);

		if(mpu_time==2&&eeprom_available==1){
			write_to_eeprom();
			eeprom_available=0;
		}
		
		
		if(getUsartData[getUsartIndex]=='R'){
			int i=0;
			int arr_i=0;
			int arr_to_send_i=0;	
			int j, r;  
			int arr[20];
			uint8_t arr_to_send[20];
			for(i =0;i<eeprom_adress;i++){
	
						received_data = _readEEPROM(&hi2c1 , device_addres, i);
				
						while (received_data != 0) {
						r = received_data % 10;
						arr[arr_i] = r;
						arr_i++;
						received_data = received_data / 10;
						}
						
						for (j = arr_i - 1; j > -1; j--) {
							arr_to_send[arr_to_send_i++]=(uint8_t)arr[j]+48;
						}
						arr_to_send[arr_to_send_i++]='#';
						
						 arr_i=0;
						 r=0;
						 j=0;
						 
				
			}
			arr_to_send[arr_to_send_i++]='\n';
			HAL_UART_Transmit_IT(&huart2,(uint8_t*) &arr_to_send,arr_to_send_i);
			HAL_Delay(50);
			arr_to_send_i=0;
			arr_to_send[0]=0;
			eeprom_adress=0;
			_eraseEEPROM(&hi2c1 , device_addres);
			increase_uart();
		}
		
		else if(getUsartData[getUsartIndex]=='>'){
				
				increase_uart();
				
				while(getUsartData[getUsartIndex]>47&&getUsartData[getUsartIndex]<58){
					nums1_arr[nums1index++]=getUsartData[getUsartIndex];
					increase_uart();
				}
				
				
				calc_operator=getUsartData[getUsartIndex];
				increase_uart();
				
				while(getUsartData[getUsartIndex]!=0){
					nums2_arr[nums2index++]=getUsartData[getUsartIndex];
					increase_uart();
				}
				execute_calculation(nums1_arr,nums2_arr,nums1index,nums2index,calc_operator);
				nums1index=0;
				nums2index=0;
		}
		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
