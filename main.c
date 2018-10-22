
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
const char MGC3130Init[]  = "MGC3130 initialization in progress...wait";
const char MGC3130Ready[] = "MGC3130 device is ready";

#ifdef PRINT_GESTURE_DATA
	const char TouchSouth[]  				 = "Touch South";
	const char TouchWest[]   				 = "Touch West";
	const char TouchNorth[]  				 = "Touch North";
	const char TouchEast[]   				 = "Touch East";
	const char TouchCentre[] 				 = "Touch Centre";

	const char TapSouth[]  					 = "Tap South";
	const char TapWest[]   					 = "Tap West";
	const char TapNorth[] 		 			 = "Tap North";
	const char TapEast[]   					 = "Tap East";
	const char TapCentre[] 					 = "Tap Centre";

	const char DoubleTapSouth[]  		 = "Double Tap South";
	const char DoubleTapWest[]   		 = "Double Tap West";
	const char DoubleTapNorth[]  		 = "Double Tap North";
	const char DoubleTapEast[]   		 = "Double Tap East";
	const char DoubleTapCentre[] 		 = "Double Tap Centre";
	
	const char GestureWestToEast[]   = "Gesture West to East";
	const char GestureEastToWest[]   = "Gesture East to West";
	const char GestureNorthToSouth[] = "Gesture North to South";
	const char GestureSouthToNorth[] = "Gesture South to North";
	
	const char GestureEdgeWestToEast[]   	= "Gesture Edge West to East";
	const char GestureEdgeEastToWest[]   	= "Gesture Edge East to West";
	const char GestureEdgeNorthToSouth[] 	= "Gesture Edge North to South";
	const char GestureEdgeSouthToNorth[] 	= "Gesture Edge South to North";	
	
	const char GestureClockWise[]        	= "Gesture Clock Wise";	
	const char GestureCounterClockWise[] 	= "Gesture Counter Clock Wise";
	
	const char GestureWaveX[] 			 	= "Gesture Wave X";	
	const char GestureWaveY[] 				= "Gesture Wave Y";
	const char GestureHold[] 			 	  = "Gesture Hold";	
	const char GesturePresence[]			= "Gesture Presence";

	const char GestureDoubleWestToEast[]   	= "Gesture Double West to East";
	const char GestureDoubleEastToWest[]   	= "Gesture Double East to West";
	const char GestureDoubleNorthToSouth[] 	= "Gesture Double North to South";
	const char GestureDoubleSouthToNorth[] 	= "Gesture Double South to North";
	
	const char RawInfoIndent[]      = "#####################################################################################\nRow Firmware Info from MGC3130 \n";
	const char HeaderInfo[]         = "Header: ";
	const char PayloadInfo[]        = "Payload: ";
	const char RawInfoCloseIndent[] = "\n#####################################################################################\n\n";
	
	const char RawDataIndent[]      = "#####################################################################################\nRow data from MGC3130 \n";
	const char HeaderRawData[]      = "Header: ";
	const char PayloadRawData[]     = "Payload: ";
	const char RawDataCloseIndent[] = "\n#####################################################################################\n\n";
	
uint8_t tx_data[2]; 
	char buffer[100];
	int len;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */


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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void MGC3130_SetAdd(uint8_t Addr) {
	_i2caddr = Addr;
}

void MGC3130_ResetDevice(uint8_t Rst) {
	pinMode(Rst, OUTPUT);    	// Set Reset line as Output
	digitalWrite(Rst, LOW);		// Reset MGC3130 device for 250 mSec
}

void MGC3130_ExitResetDevice(uint8_t Rst) {
	pinMode(Rst, OUTPUT);    	// Set Reset line as Output
	digitalWrite(Rst, HIGH);
}

void MGC3130_Begin(uint8_t Ts, uint8_t Rst) {
	HAL_UART_Transmit(&huart2,(uint8_t*)MGC3130Init,strlen(MGC3130Init),HAL_MAX_DELAY); //ReadStringFLASH((uint8_t *)MGC3130Init, strlen(MGC3130Init), TRUE);	// Print "MGC3130 initialization in progress...wait"
			
	//FirstStartPacket = TRUE;
	WIRE.begin(_i2caddr);		// Initialize I2c with hardware address
	pinMode(Ts, INPUT);    		// Set TS line as Input
	pinMode(Rst, OUTPUT);    	// Set Reset line as Output
	digitalWrite(Rst, LOW);		// Reset MGC3130 device for 250 mSec
	delay(250);					// Delay
	digitalWrite(Rst, HIGH);
	delay(250);					// Delay
	
	HAL_UART_Transmit(&huart2,(uint8_t*)MGC3130Ready,strlen(MGC3130Ready),HAL_MAX_DELAY);  //ReadStringFLASH((uint8_t *)MGC3130Ready, strlen(MGC3130Ready), TRUE);	// Print "MGC3130 device is ready"
}

void MGC3130_ReleaseTsLine(uint8_t Ts) {
    digitalWrite(Ts, HIGH);		//	Set TS line as Input
    pinMode(Ts, INPUT);			//	Set TS level
}

boolean MGC3130_GetTsLineStatus(uint8_t Ts) {
	if (digitalRead(Ts) == 0) {
		//	MGC3130 put TS line LOW. Data is available
		pinMode(Ts, OUTPUT);	//	Set TS line as Output
		digitalWrite(Ts, LOW);	//	Set TS level
		return TRUE;			//  Return TRUE;
	}	
	return FALSE;				//	Return FALSE
}

void MGC3130_GetEvent(void) {
	int  Counter = 0;
	char c;
	// if (FirstStartPacket == TRUE) {
		// FirstStartPacket = FALSE;
		// WIRE.requestFrom((uint8_t)_i2caddr, (uint8_t)0x84);
	// } else {
		// WIRE.requestFrom((uint8_t)_i2caddr, (uint8_t)0x1A);
	// }
	WIRE.requestFrom((uint8_t)_i2caddr, (uint8_t)0x1A);
    while(WIRE.available())	{
		data[Counter++] = WIRE.read();
    }
	
	switch (data[3])
	{
		case ID_FW_VERSION:		
#ifdef PRINT_RAW_FW_INFO	
	PrintMGC3130RawFirmwareInfo();
#endif
			if (data[4] == 0xAA) {
				// Valid Gestic Library available
				Serial.print("FW Version: ");
				for (int i = 0; i < 120; i++) {
					c = char(data[i + 10]);
					Serial.print(c);
				}
				Serial.print("\n");
			}
			break;
			
		case ID_DATA_OUTPUT:		
			// ----------------------------------------
			// Save Data into internal array
			for (int i = 0; i < 4; i++) {
				GestureInfo.GestArray[i] = data[i + 10];
				TouchInfo.TouchArray[i]  = data[i + 14];
			}		
			GestureInfo.Gesture &= MASK_GESTURE_RAW;
			TouchInfo.Touch     &= MASK_TOUCH_RAW;
			AirWheelInfo = data[18];
			for (int i = 0; i < 6; i++) {
				xyzPosition.xyzArray[i] = data[i + 20];
			}
			// ----------------------------------------
			break;
						
		default:
			break;
	}
}

void MGC3130_DecodeGesture(void) {
	uint32_t Mask = 0x00000001;

	if (((TouchInfo.Touch ^ LastTouch) > 0) || ((GestureInfo.Gesture ^ LastGesture) > 0) ) {
#ifdef PRINT_RAW_DATA
	PrintMGC3130RawData();
#endif
		GestureOutput.Gesture = 0;
		if ((TouchInfo.Touch ^ LastTouch) > 0) {
			LastTouch = TouchInfo.Touch;
			for (int i = 0; i < 15; i++) {
				if ((TouchInfo.Touch & Mask) > 0) {
					GestureOutput.Gesture |= Mask; 
				}
				Mask = Mask << 1;
			}
		} else if ((GestureInfo.Gesture ^ LastGesture) > 0) {
			LastGesture = GestureInfo.Gesture;
			switch (GestureInfo.Bit.GestureCode)
			{
				case NO_GESTURE:
				case GESTURE_GARBAGE:
					break;
				case GESTURE_EDGE_EAST_WEST:
					GestureOutput.Gesture |= GESTURE_MASK_EDGE_EAST_WEST;
					break;
				case GESTURE_EDGE_WEST_EAST:
					GestureOutput.Gesture |= GESTURE_MASK_EDGE_WEST_EAST;
					break;
				case GESTURE_EDGE_SOUTH_NORTH:
					GestureOutput.Gesture |= GESTURE_MASK_EDGE_SOUTH_NORTH;
					break;
				case GESTURE_EDGE_NORTH_SOUTH:
					GestureOutput.Gesture |= GESTURE_MASK_EDGE_NORTH_SOUTH;
					break;
				case GESTURE_WEST_EAST:
					if (GestureInfo.Bit.EdgeFlick == 0) {
						GestureOutput.Gesture |= GESTURE_MASK_WEST_EAST;
					} else {
						GestureOutput.Gesture |= GESTURE_MASK_EDGE_WEST_EAST;
					}
					break;	
				case GESTURE_EAST_WEST:
					if (GestureInfo.Bit.EdgeFlick == 0) {
						GestureOutput.Gesture |= GESTURE_MASK_EAST_WEST;
					} else {
						GestureOutput.Gesture |= GESTURE_MASK_EDGE_EAST_WEST;
					}
					break;
				case GESTURE_SOUTH_NORTH:
					if (GestureInfo.Bit.EdgeFlick == 0) {
						GestureOutput.Gesture |= GESTURE_MASK_SOUTH_NORTH;
					} else {
						GestureOutput.Gesture |= GESTURE_MASK_EDGE_SOUTH_NORTH;
					}
					break;
				case GESTURE_NORTH_SOUTH:
					if (GestureInfo.Bit.EdgeFlick == 0) {
						GestureOutput.Gesture |= GESTURE_MASK_NORTH_SOUTH;
					} else {
						GestureOutput.Gesture |= GESTURE_MASK_EDGE_NORTH_SOUTH;
					}
					break;
				case GESTURE_CLOCK_WISE:
					GestureOutput.Gesture |= GESTURE_MASK_CLOCK_WISE;
					break;
				case GESTURE_COUNTER_CLOCK_WISE:
					GestureOutput.Gesture |= GESTURE_MASK_COUNTER_CLOCK_WISE;
					break;
				case GESTURE_WAVE_X:
					GestureOutput.Gesture |= GESTURE_MASK_WAVE_X;
					break;
				case GESTURE_WAVE_Y:
					GestureOutput.Gesture |= GESTURE_MASK_WAVE_Y;
					break;
				case GESTURE_HOLD:
					GestureOutput.Gesture |= GESTURE_MASK_HOLD;
					break;
				case GESTURE_PRESENCE:
					GestureOutput.Gesture |= GESTURE_MASK_PRESENCE;
					break;
				case GESTURE_DOUBLE_WEST_EAST:
					GestureOutput.Gesture |= GESTURE_MASK_DOUBLE_WEST_EAST;
					break;
				case GESTURE_DOUBLE_EAST_WEST:
					GestureOutput.Gesture |= GESTURE_MASK_DOUBLE_EAST_WEST;
					break;
				case GESTURE_DOUBLE_SOUTH_NORTH:
					GestureOutput.Gesture |= GESTURE_MASK_DOUBLE_SOUTH_NORTH;
					break;
				case GESTURE_DOUBLE_NORTH_SOUTH:
					GestureOutput.Gesture |= GESTURE_MASK_DOUBLE_NORTH_SOUTH;
					break;
				default:
					break;
			}
		}
		//	Remove not desired Touch or Gesture Info. See MASK_FILTER_GESTURE into MGC3130.h file for details
		GestureOutput.Gesture &= ~(MASK_FILTER_GESTURE);
#ifdef PRINT_GESTURE_DATA
	PrintMGC3130Gesture();
#endif
#ifdef PRINT_XYZ
	PrintMGC3130xyz();
#endif
	}
}

#ifdef PRINT_RAW_FW_INFO
	void MGC3130_PrintMGC3130RawFirmwareInfo(void) { 
		if (data[3] = ID_FW_VERSION) {
			HAL_UART_Transmit(&huart2,(uint8_t*)RawInfoIndent,strlen(RawInfoIndent),HAL_MAX_DELAY); //ReadStringFLASH((uint8_t *)RawInfoIndent, strlen(RawInfoIndent), FALSE);
			//----------------------------------------
			//	Header
			HAL_UART_Transmit(&huart2,(uint8_t*)HeaderInfo,strlen(HeaderInfo),HAL_MAX_DELAY); //ReadStringFLASH((uint8_t *)HeaderInfo, strlen(HeaderInfo), FALSE);
			for (int i = 0; i < 4; i++) {
				SetHexPrintOutput(data[i]);
			}
			Serial.print("\n");
			//----------------------------------------
			
			//----------------------------------------
			//	Payload
			HAL_UART_Transmit(&huart2,(uint8_t*)HeaderInfo,strlen(PayloadInfo),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)PayloadInfo, strlen(PayloadInfo), FALSE);
			//----------------------------------------
			//	Firmware Valid
			SetHexPrintOutput(data[4]);
			Serial.print(" | ");
			//----------------------------------------

			//----------------------------------------
			//	HwRev
			for (int i = 5; i < 7; i++) {
				SetHexPrintOutput(data[i]);
			}
			Serial.print(" | ");
			//----------------------------------------

			//----------------------------------------
			//	ParameterStartAddr
			SetHexPrintOutput(data[7]);
			Serial.print(" | ");
			//----------------------------------------

			//----------------------------------------
			//	LibraryLoaderVersion
			for (int i = 8; i < 11; i++) {
				SetHexPrintOutput(data[i]);
			}
			Serial.print(" | ");
			//----------------------------------------

			//----------------------------------------
			//	FwStartAddr
			SetHexPrintOutput(data[11]);
			Serial.print(" | ");
			//----------------------------------------		

			//----------------------------------------
			//	FWVersion
			for (int i = 12; i < 132; i++) {
				SetHexPrintOutput(data[i]);
			}
			HAL_UART_Transmit(&huart2,(uint8_t*)RawInfoCloseIndent,strlen(RawInfoCloseIndent),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)RawInfoCloseIndent, strlen(RawInfoCloseIndent), FALSE);
			//----------------------------------------
		}
	}
#endif

#ifdef PRINT_RAW_DATA
	
	
	void MGC3130_PrintMGC3130RawData(void) { 
		if (data[3] = ID_DATA_OUTPUT) {
			HAL_UART_Transmit(&huart2,(uint8_t*)RawDataIndent,strlen(RawDataIndent),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)RawDataIndent, strlen(RawDataIndent), FALSE);
			//----------------------------------------
			//	Header
			HAL_UART_Transmit(&huart2,(uint8_t*)HeaderRawData,strlen(HeaderRawData),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)HeaderRawData, strlen(HeaderRawData), FALSE);
			for (int i = 0; i < 4; i++) {
				SetHexPrintOutput(data[i]);
			}
			Serial.print("\n");
			//----------------------------------------
			
			//----------------------------------------
			//	Payload
			HAL_UART_Transmit(&huart2,(uint8_t*)PayloadRawData,strlen(PayloadRawData),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)PayloadRawData, strlen(PayloadRawData), FALSE);	
			//----------------------------------------
			//	DataOutputConfigMask
			for (int i = 4; i < 6; i++) {
				SetHexPrintOutput(data[i]);
			}
			Serial.print(" | ");
			//----------------------------------------
			
			//----------------------------------------
			//	TimeStamp
			SetHexPrintOutput(data[6]);		
			Serial.print(" | ");
			//----------------------------------------
			
			//----------------------------------------
			//	SystemInfo
			SetHexPrintOutput(data[7]);		
			Serial.print(" | ");		
			//----------------------------------------
			
			//----------------------------------------
			//	DSPStatus
			for (int i = 8; i < 10; i++) {
				SetHexPrintOutput(data[i]);
			}
			Serial.print(" | ");
			//----------------------------------------

			//----------------------------------------
			//	GestureInfo
			for (int i = 10; i < 14; i++) {
				SetHexPrintOutput(data[i]);
			}
			Serial.print(" | ");
			//----------------------------------------
			
			//----------------------------------------
			//	TouchInfo
			for (int i = 14; i < 18; i++) {
				SetHexPrintOutput(data[i]);
			}
			Serial.print(" | ");
			//----------------------------------------
			
			//----------------------------------------
			//	AirWheelInfo
			for (int i = 18; i < 20; i++) {
				SetHexPrintOutput(data[i]);
			}
			Serial.print(" | ");
			//----------------------------------------	
			
			//----------------------------------------
			//	xyzPosition
			for (int i = 20; i < 26; i++) {
				SetHexPrintOutput(data[i]);
			}
			HAL_UART_Transmit(&huart2,(uint8_t*)RawDataCloseIndent,strlen(RawDataCloseIndent),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)RawDataCloseIndent, strlen(RawDataCloseIndent), FALSE);
			//----------------------------------------
		}	
	}
#endif


	
	void MGC3130_PrintMGC3130Gesture(void) {
		//----------------------------------------
		if (GestureOutput.Bit.TouchSouth > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)TouchSouth,strlen(TouchSouth),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)TouchSouth, strlen(TouchSouth), TRUE);
		}
		if (GestureOutput.Bit.TouchWest > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)TouchWest,strlen(TouchWest),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)TouchWest, strlen(TouchWest), TRUE);
		}
		if (GestureOutput.Bit.TouchNorth > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)TouchNorth,strlen(TouchNorth),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)TouchNorth, strlen(TouchNorth), TRUE);
		}
		if (GestureOutput.Bit.TouchEast > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)TouchEast,strlen(TouchEast),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)TouchEast, strlen(TouchEast), TRUE);
		}
		if (GestureOutput.Bit.TouchCentre > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)TouchCentre,strlen(TouchCentre),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)TouchCentre, strlen(TouchCentre), TRUE);
		}	
		//----------------------------------------

		//----------------------------------------
		if (GestureOutput.Bit.TapSouth > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)TapSouth,strlen(TapSouth),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)TapSouth, strlen(TapSouth), TRUE);
		}
		if (GestureOutput.Bit.TapWest > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)TapWest,strlen(TapWest),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)TapWest, strlen(TapWest), TRUE);
		}
		if (GestureOutput.Bit.TapNorth > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)TapNorth,strlen(TapNorth),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)TapNorth, strlen(TapNorth), TRUE);
		}
		if (GestureOutput.Bit.TapEast > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)TapEast,strlen(TapEast),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)TapEast, strlen(TapEast), TRUE);
		}
		if (GestureOutput.Bit.TapCentre > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)TapCentre,strlen(TapCentre),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)TapCentre, strlen(TapCentre), TRUE);
		}
		//----------------------------------------
		
		//----------------------------------------
		if (GestureOutput.Bit.DoubleTapSouth > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)DoubleTapSouth,strlen(DoubleTapSouth),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)DoubleTapSouth, strlen(DoubleTapSouth), TRUE);
		}
		if (GestureOutput.Bit.DoubleTapWest > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)DoubleTapWest,strlen(DoubleTapWest),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)DoubleTapWest, strlen(DoubleTapWest), TRUE);
		}
		if (GestureOutput.Bit.DoubleTapNorth > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)DoubleTapNorth,strlen(DoubleTapNorth),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)DoubleTapNorth, strlen(DoubleTapNorth), TRUE);
		}
		if (GestureOutput.Bit.DoubleTapEast > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)DoubleTapEast,strlen(DoubleTapEast),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)DoubleTapEast, strlen(DoubleTapEast), TRUE);
		}
		if (GestureOutput.Bit.DoubleTapCentre > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)DoubleTapCentre,strlen(DoubleTapCentre),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)DoubleTapCentre, strlen(DoubleTapCentre), TRUE);
		}
		//----------------------------------------

		//----------------------------------------	
		if (GestureOutput.Bit.GestWestEast > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)GestureWestToEast,strlen(GestureWestToEast),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)GestureWestToEast, strlen(GestureWestToEast), TRUE);
		}
		if (GestureOutput.Bit.GestEastWest > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)GestureEastToWest,strlen(GestureEastToWest),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)GestureEastToWest, strlen(GestureEastToWest), TRUE);
		}	
		if (GestureOutput.Bit.GestSouthNorth > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)GestureSouthToNorth,strlen(GestureSouthToNorth),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)GestureSouthToNorth, strlen(GestureSouthToNorth), TRUE);
		}
		if (GestureOutput.Bit.GestNorthSouth > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)GestureNorthToSouth,strlen(GestureNorthToSouth),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)GestureNorthToSouth, strlen(GestureNorthToSouth), TRUE);
		}
		if (GestureOutput.Bit.EdgeGestWestEast > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)GestureEdgeWestToEast,strlen(GestureEdgeWestToEast),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)GestureEdgeWestToEast, strlen(GestureEdgeWestToEast), TRUE);
		}
		if (GestureOutput.Bit.EdgeGestEastWest > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)GestureEdgeEastToWest,strlen(GestureEdgeEastToWest),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)GestureEdgeEastToWest, strlen(GestureEdgeEastToWest), TRUE);
		}
		if (GestureOutput.Bit.EdgeGestSouthNorth > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)GestureEdgeNorthToSouth,strlen(GestureEdgeNorthToSouth),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)GestureEdgeNorthToSouth, strlen(GestureEdgeNorthToSouth), TRUE);
		}
		if (GestureOutput.Bit.EdgeGestNorthSouth > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)GestureEdgeSouthToNorth,strlen(GestureEdgeSouthToNorth),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)GestureEdgeSouthToNorth, strlen(GestureEdgeSouthToNorth), TRUE);
		}
		if (GestureOutput.Bit.GestClockWise > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)GestureClockWise,strlen(GestureClockWise),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)GestureClockWise, strlen(GestureClockWise), TRUE);
		}
		if (GestureOutput.Bit.GestCounterClockWise > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)GestureCounterClockWise,strlen(GestureCounterClockWise),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)GestureCounterClockWise, strlen(GestureCounterClockWise), TRUE);
		}
		//----------------------------------------	
		
		//----------------------------------------	
		if (GestureOutput.Bit.GestWaveX > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)GestureWaveX,strlen(GestureWaveX),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)GestureWaveX, strlen(GestureWaveX), TRUE);
		}		
		if (GestureOutput.Bit.GestWaveY > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)GestureWaveY,strlen(GestureWaveY),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)GestureWaveY, strlen(GestureWaveY), TRUE);
		}
		if (GestureOutput.Bit.GestHold > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)GestureHold,strlen(GestureHold),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)GestureHold, strlen(GestureHold), TRUE);
		}		
		if (GestureOutput.Bit.GestPresence > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)GesturePresence,strlen(GesturePresence),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)GesturePresence, strlen(GesturePresence), TRUE);
		}	
		//----------------------------------------
		
		//----------------------------------------
		if (GestureOutput.Bit.DoubleGestWestEast > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)GestureDoubleWestToEast,strlen(GestureDoubleWestToEast),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)GestureDoubleWestToEast, strlen(GestureDoubleWestToEast), TRUE);
		}		
		if (GestureOutput.Bit.DoubleGestEastWest > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)GestureDoubleEastToWest,strlen(GestureDoubleEastToWest),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)GestureDoubleEastToWest, strlen(GestureDoubleEastToWest), TRUE);
		}	
		if (GestureOutput.Bit.DoubleSouthNorth > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)GestureDoubleSouthToNorth,strlen(GestureDoubleSouthToNorth),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)GestureDoubleSouthToNorth, strlen(GestureDoubleSouthToNorth), TRUE);
		}	
		if (GestureOutput.Bit.DoubleGestNorthSouth > 0) {
			HAL_UART_Transmit(&huart2,(uint8_t*)GestureDoubleNorthToSouth,strlen(GestureDoubleNorthToSouth),HAL_MAX_DELAY);//ReadStringFLASH((uint8_t *)GestureDoubleNorthToSouth, strlen(GestureDoubleNorthToSouth), TRUE);
		}	
		//----------------------------------------
	}
#endif

#ifdef PRINT_XYZ
	const char X[] PROGMEM = "The X coordinate is: ";
	const char Y[] PROGMEM = "The Y coordinate is: ";
	const char Z[] PROGMEM = "The Z coordinate is: ";

	void MGC3130::PrintMGC3130xyz(void) {
		if (Previous_x_pos != xyzPosition.xyzWord.x_pos) {
			Previous_x_pos = xyzPosition.xyzWord.x_pos;
			ReadStringFLASH((uint8_t *)X, strlen(X), TRUE);
			Serial.println(xyzPosition.xyzWord.x_pos, DEC);
		}
		if (Previous_y_pos != xyzPosition.xyzWord.y_pos) {
			Previous_y_pos = xyzPosition.xyzWord.y_pos;
			ReadStringFLASH((uint8_t *)Y, strlen(Y), TRUE);
			Serial.println(xyzPosition.xyzWord.y_pos, DEC);
		}
		if (Previous_z_pos != xyzPosition.xyzWord.z_pos) {
			Previous_z_pos = xyzPosition.xyzWord.z_pos;		
			ReadStringFLASH((uint8_t *)Z, strlen(Z), TRUE);
			Serial.println(xyzPosition.xyzWord.z_pos, DEC);		
		}
	}
#endif

//----------------------------------------
//	This function is used to read the string data by flash and print the data read on the screen
//	It is useful to save SRAM memory instead that used the "println" function with a string type parameter
//	The string type parameters are saved into FLASH memory using the "PROGMEM" function
void MGC3130_ReadStringFLASH(uint8_t *FlashPointer, uint8_t Lenght, bool PrintCR) {
	uint8_t k;
	char myChar;
	for (k = 0; k < Lenght; k++) {
		myChar = pgm_read_byte_near(FlashPointer + k);
		Serial.print(myChar);
	}
	if (PrintCR == TRUE) {
		Serial.print("\n");
	}
}
//----------------------------------------

void MGC3130_SetHexPrintOutput(uint8_t Data) {
	if (Data < 0x10) {
		Serial.print(0, HEX);
		Serial.print(Data, HEX);
	} else {
		Serial.print(Data, HEX);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
