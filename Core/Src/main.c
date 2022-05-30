/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define pcf8574_adress 0x4E    //0x4E// i2c address
#define WIG8                    8
#define WIG26                   26 - 1
#define WIG34                   34 - 1
#define MAX_ACTIVE_READER       8       //8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile bool tx_complete = false;
volatile bool rx_complete = false;

bool text_waiting = false;
bool buffer_flag = false;

bool input_EB5 = true;
bool input_EB6 = true;
bool input_EB7 = true;
bool input_EB8 = true;

bool flag_EB5 = true;
bool flag_EB6 = true;
bool flag_EB7 = true;
bool flag_EB8 = true;

bool input_MS5 = true;
bool input_MS6 = true;
bool input_MS7 = true;
bool input_MS8 = true;

bool flag_MS5 = true;
bool flag_MS6 = true;
bool flag_MS7 = true;
bool flag_MS8 = true;

bool WigFlag = 0;
bool WigFlag26b = 0;
short int Wcnt = 0;
//unsigned char Status;

uint8_t ReqWIGType[6];
uint8_t WIGType[16];
unsigned char SERWIGType[16];
unsigned char BitOdd;
unsigned char BitEven;
unsigned char BitLen;
unsigned char ParOdd;
unsigned char WIGIX = 50;
unsigned char SETWIGType[16];
unsigned char RI[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned char RE[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned char RawWIGData[8][36];
unsigned long WIGCardData[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned char cResult = 0;
unsigned long WIEGANDCurReaderCardID[16];
volatile unsigned short WaitForPermTime[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned char TX = 0;
unsigned long int LedCount = 0;

int c = 0;

unsigned int wiegand_1_d0 = 1;
unsigned int wiegand_1_d1 = 1;

uint8_t UART1_rxBuffer[9];                    //8// 9
uint8_t UART1_rxBuffer_data[9];                    //8
uint8_t UART1_rxBuffer_data_2[8];
uint8_t data_t[4];                    //i2c data buf

bool rxBuffer_flag = 0;
unsigned char wiegand_data[38];                    //39
unsigned char input_data[10];

int WigSay = 3;
int WigSayProx = 3;
int uartSay = 0;
int a = 0;
int say = 0;
int deneme_say = 0;
int buzzer_led_flag = 0;
unsigned char say_i2c = 0;
int control_relay = 0;
int SndCnt = 0;
int SndCnt1 = 0;
int wiegand_length = 37;

bool Relay5_ON = 0;
bool Relay6_ON = 0;
bool Relay7_ON = 0;
bool Relay8_ON = 0;
unsigned int InputSay;
int setEt=0;
/////////////////////////
uint16_t readValue;
float tCelsius;
float tFahrenheit;
/////////////////////////
uint8_t RxIndex = 0;
uint8_t RxTempBuf[1];
uint8_t RxBuffer[10];
//////////////////////////
uint8_t rxTEMBuff[10];
uint8_t rxBuff[40];
uint8_t rxBuff1[10];
uint8_t rxIndex = 0;
uint8_t rxCNT = 0;
//////////////////////////
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void WIGReader26bOr34b();
void SendWIDData();
void UART1_SendData(unsigned char *msg_string, unsigned char TLenght);
unsigned char ProceedWIGDataNew(unsigned char WRI, unsigned char TBitOdd,
		unsigned char TBitEven, unsigned char TBitLen, unsigned char TParOdd,
		unsigned char WIGType, unsigned long *TWIGCardData);
void CheckWiegandReader();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//UART Receieve_IT
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		rxBuff[rxIndex] = rxTEMBuff[0];
		rxIndex++; 	//rxIndex++;//RxIndex
		HAL_UART_Receive_IT(&huart1, rxTEMBuff, 1);
	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  tx_complete = true;
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
  MX_ADC_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
//	HAL_GPIO_EXTI_Callback(GPIO_Pin);
	HAL_UART_Receive_IT(&huart1, rxTEMBuff, 1);

	HAL_ADC_Start(&hadc);

	input_data[0] = '&';
	for(setEt =1 ;setEt<9; setEt++){
		input_data[setEt]= '1';
	}
	input_data[9] = '\n';
	HAL_Delay(10);
	HAL_UART_Transmit(&huart1,(uint8_t*)input_data, sizeof(input_data), 10U);

	ReqWIGType[0] = '?';
	ReqWIGType[1] = 'I';
	ReqWIGType[2] = 'G';
	ReqWIGType[3] = 'O';
	ReqWIGType[4] = 'K';
	ReqWIGType[5] = '\n';
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		/**************************SICAKLIK SENSOR************************/
		HAL_ADC_PollForConversion(&hadc, 1000);
		readValue = HAL_ADC_GetValue(&hadc);
		tCelsius = 357.558 - 0.187364 * readValue;
		tFahrenheit = 675.6 - 0.337255 * readValue;
		/**************************SICAKLIK SENSOR************************/
//		HAL_UART_Receive_IT(&huart1, rxTEMBuff, 1);

		if (rxBuff[0] == '1' && rxBuff[1] == '1') {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			rxIndex = 0;
			rxBuff[0] = 0x000;
			rxBuff[1] = 0x000;
		} else if (rxBuff[0] == '2' && rxBuff[1] == '2') {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			rxIndex = 0;
			rxBuff[0] = 0x000;
			rxBuff[1] = 0x000;
		}
		if (rxIndex == 40) {
			rxIndex = 0;
		}
		/////*******START WIEGAND 02*********///////
//		HAL_UART_Receive_IT(&huart1, RxTempBuf, 1); 	//8 - 9
		//HAL_Delay(20);

		if (rxBuff[0] == '%') {
			rxBuffer_flag = 1;
			for (a = 0; a < 9; a++){
				UART1_rxBuffer_data[a] = rxBuff[a];
			}
			rxIndex = 0;
			uartSay = 0;
		} else if (rxBuff[0] == '&') {
			rxBuffer_flag = 1;

			for (a = 0; a < 9; a++) {
				UART1_rxBuffer_data[a] = rxBuff[a];
			}
			rxIndex = 0;
			uartSay = 0;
		}

		if (UART1_rxBuffer_data[1] == 'N') //HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 9); 9 oldugu için datalar kayiyor
				{
			WIGType[0] = UART1_rxBuffer_data[4];   //  5
			WIGType[1] = UART1_rxBuffer_data[5];   //  6
			WIGType[2] = UART1_rxBuffer_data[6];   //  7
			WIGType[3] = UART1_rxBuffer_data[7];   //  8
			HAL_Delay(10);

			for (int cnt = 0; cnt < 10; cnt++) {
				rxBuff[cnt] = 0x00;
				UART1_rxBuffer_data[cnt] = 0x00;
			}
			SndCnt1 = 1;
//	  	      SndCnt = 1;				//Bundan dolayı  dongude takılı kalıyor
//	  	      uartSay = 0;
			RxIndex = 0;
			rxIndex = 0;

			led_12_OFF;
			led_34_OFF;
		}

		if ((UART1_rxBuffer_data[1] == 'R') && (UART1_rxBuffer_data[2] == 'C')) //role Ac onayli
				{
			HAL_GPIO_WritePin(GPIOA, relay_5_Pin, UART1_rxBuffer_data[6] - 48);
			HAL_GPIO_WritePin(GPIOA, relay_6_Pin, UART1_rxBuffer_data[5] - 48);
			HAL_GPIO_WritePin(GPIOB, relay_7_Pin, UART1_rxBuffer_data[4] - 48);
			HAL_GPIO_WritePin(GPIOB, relay_8_Pin, UART1_rxBuffer_data[3] - 48);

			for (int cnt = 0; cnt < 9; cnt++) {
				UART1_rxBuffer[cnt] = 0x00;
				UART1_rxBuffer_data[cnt] = 0x00;
				rxBuff[cnt] = 0x000;
			}

			rxIndex = 0;
		}
		//Relay8
		//////////Herşeyin sebebi bence bu////////////////////////////////////////
		if (uartSay < 16) {
			UART1_rxBuffer_data[uartSay] = 0x00;
			UART1_rxBuffer[uartSay] = 0x00;
			uartSay++;
		}
		//////////Herşeyin sebebi bence bu////////////////////////////////////////
		/////********END START WIEGAND 02***********/

		///************  START UART TASK 01  *********//////
		if (SndCnt1 == 1) {
			if (LedCount == 10) {
				HAL_GPIO_WritePin(cpu_led_GPIO_Port, cpu_led_Pin, GPIO_PIN_SET);
				HAL_Delay(100);
			}
			HAL_GPIO_WritePin(cpu_led_GPIO_Port, cpu_led_Pin, GPIO_PIN_RESET);

			if (LedCount == 10)
				LedCount = 0;

			LedCount++;
		}

		/*******************************Get WIEGAND Type********************************/
		///////////////////////////////////////////////////////////////////////////////
		if (SndCnt == 0) {
			if(SndCnt1 != 1){
				HAL_GPIO_WritePin(cpu_led_GPIO_Port, cpu_led_Pin, GPIO_PIN_RESET);
				HAL_Delay(60);	  	    	//
				HAL_GPIO_WritePin(cpu_led_GPIO_Port, cpu_led_Pin, GPIO_PIN_SET);
				HAL_Delay(20);	  	    	//
				HAL_UART_Transmit(&huart1, ReqWIGType, 6, 100);
				led_12_ON;
				led_34_ON;
			}

//			led_12_ON;
//			led_34_ON;

			HAL_Delay(100);
//			HAL_UART_Transmit(&huart1, (uint8_t*)ReqWIGType, 6, 100);
//			HAL_UART_Transmit(&huart1, ReqWIGType, 6, 100);
			if (WIGType[4] == 'W' && WIGType[6] == 'G' && WIGType[7] == 'T') {
				SndCnt = 1;

				led_12_OFF;
				led_34_OFF;

				for (Wcnt = 0; Wcnt < 9; Wcnt++) {
					UART1_rxBuffer[Wcnt] = 0;
				}
			}
			uartSay = 0;
		}

		///////////////////////////////////////////////////////////////////////////////
		/***************************End Of The Get WIEGAND Type************************/

		//WIGIX = wiegand_data[2]
		for (TX = 0; TX <= 8; TX++) {
			if (WIGType[TX] == 0)
				SETWIGType[TX] = WIG26;
			else
				SETWIGType[TX] = WIG34;
		}

		if (WIGType[0] == '1')		//0x01
			LedReader5ON;
		else
			LedReader5OFF;

		if (WIGType[1] == '1')		//0x01
			LedReader6ON;
		else
			LedReader6OFF;

		if (WIGType[2] == '1')		//0x01
			LedReader7ON;
		else
			LedReader7OFF;

		if (WIGType[3] == '1')		//0x01
			LedReader8ON;
		else
			LedReader8OFF;

		WIGReader26bOr34b();

	  ///////////////// INPPUT/ ///////////////////////
	  	    input_data[0]='&';
	  	    input_data[9]='\n';

	  	    input_EB7=HAL_GPIO_ReadPin(GPIOA,EB5_Pin);
	  	    input_EB8=HAL_GPIO_ReadPin(GPIOA,EB6_Pin);
	  	    input_EB5=HAL_GPIO_ReadPin(GPIOA,EB7_Pin);///
	  	    input_EB6=HAL_GPIO_ReadPin(GPIOA,EB8_Pin);
	  	    input_MS5=HAL_GPIO_ReadPin(GPIOA,MS7_Pin);  //MS5_Pin
	  	    input_MS6=HAL_GPIO_ReadPin(GPIOA,MS8_Pin);  //MS6_Pin
	  	    input_MS7=HAL_GPIO_ReadPin(GPIOA,MS5_Pin);  //MS7_Pin
	  	    input_MS8=HAL_GPIO_ReadPin(GPIOA,MS6_Pin);  //MS8_Pin
	  	 /********************************************/
	  	 //***                EB5-8                 ***
	  	 //***                                      ***
	  	 //***                                      ***
	  	    HAL_Delay(10);//100

	  	    if((input_EB5==0) && (flag_EB5==1))
	  	    {
	  	      flag_EB5=0;
	  	      input_data[1] = '0';
	  	      UART1_SendData(input_data, 10);
	  	    }
	  	    else if((input_EB5==1) && (flag_EB5==0))
	  	    {
	  	      flag_EB5=1;
	  	      input_data[1] = '1';
	  	       UART1_SendData(input_data, 10);

	  	    }
	  	    /***********************************************/
	  	     else if((input_EB6==0) && (flag_EB6==1))
	  	    {
	  	      flag_EB6=0;
	  	      input_data[2] = '0';
	  	      UART1_SendData(input_data, 10);

	  	    }

	  	     else if((input_EB6==1) && (flag_EB6==0))
	  	    {
	  	      flag_EB6=1;
	  	      input_data[2] = '1';
	  	      UART1_SendData(input_data, 10);
	  	    }
	  	    /***********************************************/
	  	     else if((input_EB7==0) && (flag_EB7==1))
	  	    {
	  	      flag_EB7=0;
	  	      input_data[3] = '0';
	  	      UART1_SendData(input_data, 10);
	  	    }

	  	     else if((input_EB7==1) && (flag_EB7==0))
	  	    {
	  	      flag_EB7=1;
	  	      input_data[3] = '1';
	  	      UART1_SendData(input_data, 10);
	  	    }
	  	    /***********************************************/
	  	     else if((input_EB8==0) && (flag_EB8==1))
	  	    {
	  	      flag_EB8=0;
	  	      input_data[4] = '0';
	  	      UART1_SendData(input_data, 10);

	  	    }
	  	     else if((input_EB8==1) && (flag_EB8==0))
	  	    {
	  	      flag_EB8=1;
	  	      input_data[4] = '1';
	  	      UART1_SendData(input_data, 10);

	  	    }
	  	 /*****************************************************************************/
	  	 //***                                                                      ***
	  	 //***                                 MS5-8                                ***
	  	 //***                                                                      ***
	  	 //****************************************************************************/
	  	     if((input_MS5==0) && (flag_MS5==1))
	  	    {
	  	      flag_MS5=0;
	  	      input_data[5] = '0';
	  	      UART1_SendData(input_data, 10);

	  	    }

	  	     else if((input_MS5==1) && (flag_MS5==0))
	  	    {
	  	      flag_MS5=1;
	  	      input_data[5] = '1';
	  	      UART1_SendData(input_data, 10);

	  	    }
	  	 /***********************************************/
	  	      else if((input_MS6==0) && (flag_MS6==1))
	  	    {
	  	      flag_MS6=0;
	  	       input_data[6] = '0';
	  	       UART1_SendData(input_data, 10);

	  	    }

	  	     else if((input_MS6==1) && (flag_MS6==0))
	  	    {
	  	      flag_MS6=1;
	  	      input_data[6] = '1';
	  	      UART1_SendData(input_data, 10);

	  	    }
	  	 /***********************************************/
	  	      else if((input_MS7==0) && (flag_MS7==1))
	  	    {
	  	      flag_MS7=0;
	  	      input_data[7] = '0';
	  	      UART1_SendData(input_data, 10);

	  	    }
	  	     else if((input_MS7==1) && (flag_MS7==0))
	  	    {
	  	      flag_MS7=1;
	  	      input_data[7] = '1';
	  	      UART1_SendData(input_data, 10);

	  	    }
	  	 /***********************************************/
	  	      else if((input_MS8==0) && (flag_MS8==1))
	  	    {
	  	      flag_MS8=0;
	  	      input_data[8] = '0';
	  	      UART1_SendData(input_data, 10);

	  	    }
	  	     else if((input_MS8==1) && (flag_MS8==0))
	  	    {
	  	      flag_MS8=1;
	  	      input_data[8] = '1';
	  	      UART1_SendData(input_data, 10);

	  	    }
	  	    HAL_Delay(10);//100
	  	 /***********************************************/
//	  	    if(RxIndex1 > 9){
//	  	      for(int cnt = 0; cnt < 9; cnt++)
//	  	        UART1_rxBuffer[cnt] = 0x00;
//
//	  	    }
//
//	 if(RxIndex1 > 9)
//	 {
//		 for(int cnt = 0; cnt < 9; cnt++)
//			 UART1_rxBuffer[cnt] = 0x00;
//
//		 RxIndex1 = 0;
//	 }//*11110000

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_ODD;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, cpu_led_Pin|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|OE_hc245_Pin_Pin|DIR_hc245_Pin_Pin
                          |relay_7_Pin|relay_8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, relay_5_Pin|relay_6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : cpu_led_Pin PC14 PC15 */
  GPIO_InitStruct.Pin = cpu_led_Pin|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EB5_Pin EB6_Pin EB7_Pin EB8_Pin
                           MS5_Pin MS6_Pin MS7_Pin MS8_Pin */
  GPIO_InitStruct.Pin = EB5_Pin|EB6_Pin|EB7_Pin|EB8_Pin
                          |MS5_Pin|MS6_Pin|MS7_Pin|MS8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : wiegand5_D0_Pin wiegand5_D1_Pin PB2 PB3
                           PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = wiegand5_D0_Pin|wiegand5_D1_Pin|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 OE_hc245_Pin_Pin DIR_hc245_Pin_Pin
                           relay_7_Pin relay_8_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|OE_hc245_Pin_Pin|DIR_hc245_Pin_Pin
                          |relay_7_Pin|relay_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : relay_5_Pin relay_6_Pin */
  GPIO_InitStruct.Pin = relay_5_Pin|relay_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void WIGReader26bOr34b()
{
  if(WigFlag == 1) {
    SendWIDData();
  }
  else if(WigFlag26b == 1 ){
    SendWIDData();
  }

   InputSay=0;
}


void SendWIDData()
{
    led_12_ON;
    led_34_ON;
    CheckWiegandReader();

    wiegand_data[38]='\n';                            //    wiegand_data[30]='\t';
    wiegand_data[39]='\t';                            //    wiegand_data[29]='\n';

    HAL_UART_Transmit(&huart1, (uint8_t*)wiegand_data, sizeof(wiegand_data), 38U);
    HAL_Delay(60);
    WigSay = 3;
    WigFlag = 0;
    WigFlag26b = 0;

    for(int syy = 0; syy < 8; syy++)
    RI[syy] = 0;

    for(int WCount = 0; WCount < 38; WCount++ )
      wiegand_data[WCount] = '\0';

    led_12_OFF;
    led_34_OFF;
}

void CheckWiegandReader()
{
  //WIGIX = wiegand_data[2]
  for(TX = 0; TX <= 8; TX++)
  {
    if(WIGType[TX] == 0)
        SETWIGType[TX] = WIG26;
    else
        SETWIGType[TX] = WIG34;
  }
  for(WIGIX = wiegand_data[2] - 0x31 ; WIGIX < MAX_ACTIVE_READER; WIGIX++){
        // Adjust Parity Variables
        if ( SETWIGType[WIGIX] == WIG34 )
        {
          BitOdd = 17;
          BitEven = 1;
          BitLen = 16;
          ParOdd = 33;
        }
        else
        {
          BitOdd = 13;
          BitEven = 1;
          BitLen = 12;
          ParOdd = 25;
        }
        if ( RI[WIGIX]/*WigSay*/ > SETWIGType[WIGIX] )   //WigSay ,  RI[WIGIX]
        {
          cResult = ProceedWIGDataNew(WIGIX, BitOdd, BitEven, BitLen, ParOdd, SETWIGType[WIGIX], WIGCardData);
          if ( cResult == 1 )
          {
            WIEGANDCurReaderCardID[WIGIX] = WIGCardData[WIGIX];
          }
          RI[WIGIX] = 0;
    }
  }
}

unsigned char ProceedWIGDataNew(unsigned char WRI, unsigned char TBitOdd, unsigned char TBitEven,
                       unsigned char TBitLen, unsigned char TParOdd, unsigned char WIGType,
                       unsigned long * TWIGCardData )
{
  unsigned char PBI;
  unsigned char TSum;
  unsigned char EvenOk = 0;
  unsigned char OddOk = 0;
  unsigned long long int TLCard = 0;
  unsigned long long int TWIGCardData1; ////yb
  unsigned int WDCnt;
  c = 0;
  TWIGCardData[WRI] = 0;
  TWIGCardData1=0; ////yb

  for(WDCnt = 0; WDCnt < wiegand_length; WDCnt++){
    RawWIGData[WRI][WDCnt] = wiegand_data[WRI + 3];
  }
  WDCnt = 0;

  TSum = 0;
  PBI = TBitEven;
  do
  {
    if ( RawWIGData[WRI][PBI] )
      TSum++;

    PBI++;
  }
  while ( PBI < (TBitEven + TBitLen) );

  if ( ( (TSum % 2) > 0 ) && ( RawWIGData[WRI][0] > 0 ) )
    EvenOk = 1;
  if ( ( (TSum % 2) <= 0 ) && ( RawWIGData[WRI][0] <= 0 ) )
    EvenOk = 1;

  if ( EvenOk )
  {
    // Check Odd Parity
    TSum = 0;
    PBI = TBitOdd;
    do
    {
      if ( RawWIGData[WRI][PBI] )
        TSum++;

      PBI++;
    }
    while ( PBI < (TBitOdd + TBitLen) );

    if ( ( (TSum % 2) <= 0 ) && ( RawWIGData[WRI][TParOdd] > 0 ) )
      OddOk = 1;
    if ( ( (TSum % 2) > 0 ) && ( RawWIGData[WRI][TParOdd] <= 0 ) )
      OddOk = 1;
  }

  if ( ( EvenOk ) && ( OddOk ) )
  {
    // Both Parities Are Ok
    // Card Data
    TLCard = 0;
    TWIGCardData[WRI] = 0;
    TWIGCardData1 = 0; ////yb

    PBI = 1;
    do
    {
      if ( RawWIGData[WRI][PBI] )
        TLCard |= ((unsigned long)1 << (WIGType - PBI - 1));
      //  TWIGCardData[WRI] |= ((unsigned long)1 << (WIGType - PBI - 1));

      PBI++;
    }
    while ( PBI < WIGType );

    if ( WIGType == WIG34 )
    {
#ifdef WIG34_SWAP
      TWIGCardData1 = TLCard;
#else
      TWIGCardData1 = TLCard;
#endif
    }
    else
      TWIGCardData1 = TLCard;

                TWIGCardData[WRI]= (unsigned long)TWIGCardData1;

            //TWIGCardData[WRI] = TLCard;
    return 1;
  }
  return 0;
}

/////////////////*UART SEND DATA*//////////////////////////////////

void UART1_SendData(unsigned char *msg_string, unsigned char TLenght)
{
  //InputSay=0;
  if(InputSay < 11)
  {
    HAL_UART_Transmit(&huart1, (uint8_t*)msg_string, TLenght, 100);
    InputSay++;
  }
  InputSay=0;
}

/////////////////*END UART SEND DATA*///////////////////////////////
///////HAL_GPIO_EXTI_Callback///////

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  wiegand_data[0]='#';
  wiegand_data[1]='W';
  a=0;
//********************READER 5**********************
 if (GPIO_Pin == GPIO_PIN_0)// DATA 0 , GPIO_PIN_0
  {
    wiegand_data[2]='5';

    if(RI[0] <= SETWIGType[0])
    {
      wiegand_data[RI[0]+3] = 0;
      RI[0]++;
      WigSay++;
    }
  }
  if (GPIO_Pin == GPIO_PIN_1)// DATA 1  ,  GPIO_PIN_1
  {
    wiegand_data[2]='5';

    if(RI[0] <= SETWIGType[0])
    {
      wiegand_data[RI[0]+3] = 1;
      RI[0]++;
      WigSay++;
    }
  }
//********************READER 6**********************
  if (GPIO_Pin == GPIO_PIN_2)// DATA 0
  {
    wiegand_data[2]='6';

    if(RI[1] <= SETWIGType[1])
    {
      wiegand_data[RI[1]+3] = 0;
      RI[1]++;
      WigSay++;
    }
  }

  if (GPIO_Pin == GPIO_PIN_3)// DATA 1
  {  wiegand_data[2]='6';

      if(RI[1] <= SETWIGType[1])
    {
      wiegand_data[RI[1]+3] = 1;
      RI[1]++;
      WigSay++;
    }
  }

//********************READER 7**********************
  if (GPIO_Pin == GPIO_PIN_4)// DATA 0
  {
    wiegand_data[2]='7';

    if(RI[2] <= SETWIGType[2])
    {
      wiegand_data[RI[2]+3] = 0;
      RI[2]++;
      WigSay++;
    }
  }

  if (GPIO_Pin == GPIO_PIN_5)// DATA 1
  {
	  wiegand_data[2]='7';

    if(RI[2] <= SETWIGType[2])
    {
      wiegand_data[RI[2]+3] = 1;
      RI[2]++;
      WigSay++;
    }
  }
//********************READER 8**********************
  if (GPIO_Pin == GPIO_PIN_6)// DATA 0
  {
    wiegand_data[2]='8';
    if(RI[3] <= SETWIGType[3])
    {
      wiegand_data[RI[3]+3] = 0;
      RI[3]++;
      WigSay++;
    }
  }

  if (GPIO_Pin == GPIO_PIN_7)// DATA 1
  {
    wiegand_data[2]='8';
    if(RI[3] <= SETWIGType[3])
    {
      wiegand_data[RI[3]+3] = 1;
      RI[3]++;
      WigSay++;
    }
  }

  say=0;

 if((WIGType[0] == 1) && RI[0] == 34)
   WigFlag = 1;
 else if((WIGType[1] == 1) && RI[1] == 34)
   WigFlag = 1;
 else if((WIGType[2] == 1) && RI[2] == 34)
   WigFlag = 1;
 else if((WIGType[3] == 1) && RI[3] == 34)
   WigFlag = 1;
 else if((WIGType[0] == 0) && RI[0] == 26)
   WigFlag26b = 1;
 else if((WIGType[1] == 0) && RI[1] == 26)
   WigFlag26b = 1;
 else if((WIGType[2] == 0) && RI[2] == 26)
   WigFlag26b = 1;
 else if((WIGType[3] == 0) && RI[3] == 26)
   WigFlag26b = 1;

}

///////HAL_GPIO_EXTI_Callback///////

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
