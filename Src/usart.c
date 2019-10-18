/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
uint8_t RXBuffer[1];
uint8_t RXBuffer_3[1];
uint8_t RXBuffer_6[RxSize] = {0x00};

static char UartStartFlag = 0;
static char Uart6StartFlag = 0;
unsigned char UartRxData[9]; //缓存数据数组
unsigned char Uart6RxData[9];
static unsigned char Uart_counter = 0;  //计数

static char Uart3StartFlag = 0;
unsigned char Uart3RxData[13]; //缓存数据数组
static unsigned char Uart3_counter = 0;  //计数
static unsigned char Uart6_counter = 0;  //计数
unsigned char Uart3FeedBackData[12];


unsigned char UartFeedBackData[DataFdbkNum];
unsigned char UartRxBuf = 0;

static unsigned int upwm;

extern var_t var;
extern unsigned char Uart3Error;
extern unsigned int gErrorStatus;
extern Moter_t M1,M2,M3;
extern Ctrl_t Ctrl,CtrlM1,CtrlM2,CtrlM3;;

extern void    FLASH_PageErase(uint32_t PageAddress);
static FLASH_EraseInitTypeDef EraseInitStruct;
extern uint32_t FLASH_Address,PageError;
extern unsigned int FLASH_Store[FLASHSIZE];
extern unsigned int FLASH_Init[FLASHSIZE];
extern char FlagDebug;
extern MotorGriverForce_t MotorForce;
extern unsigned char GriverFDBKCounter;


/* USER CODE END 0 */

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 256000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}
/* USART6 init function */

void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 256000;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart6.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart6, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart6, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart6) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART2 GPIO Configuration    
    PA3     ------> USART2_RX
    PD5     ------> USART2_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();
  
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART3 GPIO Configuration    
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX 
    */
    GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();
  
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    /**USART6 GPIO Configuration    
    PC6     ------> USART6_TX
    PG9     ------> USART6_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART6;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 2, 3);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PA3     ------> USART2_RX
    PD5     ------> USART2_TX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();
  
    /**USART3 GPIO Configuration    
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX 
    */
    HAL_GPIO_DeInit(GPIOD, STLK_RX_Pin|STLK_TX_Pin);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();
  
    /**USART6 GPIO Configuration    
    PC6     ------> USART6_TX
    PG9     ------> USART6_RX 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_9);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
/* Prevent unused argument(s) compilation warning */
UNUSED(huart);

HAL_UART_Receive_IT(huart, (uint8_t *)RXBuffer, 1); //再打开中断接收
	if(huart==&huart2)	{
		USART2Interrupt(RXBuffer[0]);
		RXBuffer[0]=0xFF;
	}
	if(huart==&huart3)	{
		USART3Interrupt(RXBuffer[0]);
		RXBuffer[0] = 0xFF;
	}
	if(huart==&huart6)	{
		USART6Interrupt(RXBuffer);
		RXBuffer[0] = 0xFF;
//		HAL_UART_DMAResume(&huart6);
//		HAL_UART_Receive_DMA(&huart6, RXBuffer_6, RxSize);                        // 重新打开DMA接收数据
	}
}
void USART6Interrupt(unsigned char *UartRxBuf)
{
	unsigned char rxBuf,uartCmd,uartCmdData;
//	for(char kk=0;kk<RxSize;kk++)
//	{
//	rxBuf = UartRxBuf[kk];  UartRxBuf[kk]=0xFF;
	rxBuf = UartRxBuf[0];  UartRxBuf[0]=0xFF;
	if(rxBuf=='}')	{
		if(Uart6_counter==8) 	{ 	
			if(Uart6RxData[7]==((Uart6RxData[0]+Uart6RxData[1]+Uart6RxData[2]+Uart6RxData[3]+Uart6RxData[4]+Uart6RxData[5]+Uart6RxData[6])%100))	{
				uartCmd = Uart6RxData[0];
				uartCmdData = Uart6RxData[1];
				if(uartCmd==0x11)	{		//数据分析
					CtrlM3.sensorData = Uart6RxData[1]*100+Uart6RxData[2]-5000;
					CtrlM2.sensorData = Uart6RxData[3]*100+Uart6RxData[4]-5000;
					CtrlM1.sensorData = Uart6RxData[5]*100+Uart6RxData[6]-5000;	
				}
			}
			
		}
		Uart6_counter = 0;
		Uart6StartFlag = 0;
	}
	if(Uart6StartFlag==1)
	{
		if(Uart6_counter>7)  Uart6StartFlag = 0;
		else 
		{
			Uart6RxData[Uart6_counter] = rxBuf;  //存入数组
			Uart6_counter++;
		}	
	}
	if(rxBuf=='{')
	{
		Uart6StartFlag = 1;
		Uart6_counter = 0;
	}
		
//	}
}

void USART3Interrupt(char UartRxBuf)	//数据交换  上位机串口
{
static unsigned char i =0;	
static unsigned int scheck = 0;
unsigned char Cmd = 0;	
if(UartRxBuf=='}')	{
	if(Uart3_counter==13) 	{
		scheck = 0;
		for(i=0;i<12;i++)	{
			scheck +=Uart3RxData[i];
		}
		scheck = scheck%100;
		if(scheck==Uart3RxData[12])	{
			//暂留  上位机解算位置 得出的需要补偿的力部分的代码
//			CalcMotorForce(&MotorForce.F1,Uart3RxData[2],Uart3RxData[3],Uart3RxData[4]);
//			CalcMotorForce(&MotorForce.F2,Uart3RxData[5],Uart3RxData[6],Uart3RxData[7]);
//			CalcMotorForce(&MotorForce.F3,Uart3RxData[8],Uart3RxData[9],Uart3RxData[10]);
//			//数据排序符合先后次序  且在10ms中断中  不断刷新  保证通讯成功
//			GriverFDBKCounter = Uart3RxData[11];
//			Uart3Error &= ~(Error_Uart3RX);
			//分析指令
			Cmd = Uart3RxData[1];
			switch(Cmd)
			{
				case RET_POSITION:
				{
					retPosition();
				}
			}
			
		}
		else {
			Uart3Error |= Error_Uart3RX;
		}
	}
	
	Uart3_counter = 0;
	Uart3StartFlag = 0;
	
}

if(Uart3StartFlag==1)
{
	if(Uart3_counter>13)  Uart3StartFlag = 0;
	else 
	{
		Uart3RxData[Uart3_counter] = UartRxBuf;  //存入数组
		Uart3_counter++;
	}	
}
if(UartRxBuf=='{')
{
	Uart3StartFlag = 1;
	Uart3_counter = 0;
}

}
//将传回来的电机为重力补偿的char数据  转化为float
void CalcMotorForce(float *fData,unsigned char cData1,unsigned char cData2,unsigned char cData3)
{
	if(cData1>>7==1)	{
		cData1 &= ~(1<<7);
		*fData = -(float)((float)cData3/10 + cData2*10 + cData1*1000);
	}
	else	{
		*fData = (float)((float)cData3/10 + cData2*10 + cData1*1000);
	}
	//设定取值范围
	
}

char retPosition(void)
{
	//读取  数据转换  返回
	char status = 0;
	float x,y,z;  int Summ,ii; 
	unsigned char numm1,numm2,numm3;
	
	x = var.X0;
	y = var.Y0;
	z = var.Z0;

	UartFeedBackData[0] = 0x7B;
	UartFeedBackData[1]	= 0x31;
	PositionDataCover(&numm1,&numm2,&numm3,x);
	UartFeedBackData[2]	= (unsigned char)numm1; 
	UartFeedBackData[3] = (unsigned char)numm2;
	UartFeedBackData[4]	= (unsigned char)numm3;		
	PositionDataCover(&numm1,&numm2,&numm3,y);
	UartFeedBackData[5] = (unsigned char)numm1;
	UartFeedBackData[6] = (unsigned char)numm2;
	UartFeedBackData[7] = (unsigned char)numm3;
	PositionDataCover(&numm1,&numm2,&numm3,z);
	UartFeedBackData[8] = (unsigned char)numm1;
	UartFeedBackData[9] = (unsigned char)numm2;
	UartFeedBackData[10]= (unsigned char)numm3;
	UartFeedBackData[11]= (unsigned char)0;
	UartFeedBackData[12]=  0;
	UartFeedBackData[13]= 0;
	Summ = 0;
	for(ii=1; ii<DataFdbkNum-2;ii++)	{
		Summ +=UartFeedBackData[ii];
	}
	Summ = Summ%100;
	UartFeedBackData[14] = Summ;
	UartFeedBackData[15] = 0x7D;
	
	HAL_UART_Transmit(&huart3, (uint8_t *)&UartFeedBackData, DataFdbkNum, 0xFFFF);
	
	return status;
}


void PositionDataCover(unsigned char *num1,unsigned char *num2,unsigned char *num3,float f)
{
	if(f<0)	
	{
		f = -f;
		*num1 = 1;
	}
	else	*num1 = 0;
	
	*num2 = (unsigned char)f/10.0;
	*num3 = (unsigned char)((unsigned int)f*10%100);
}

void USART2Interrupt(char UartRxBuf)		//参数  状态控制串口
{
static unsigned char i =0;	
static unsigned int scheck = 0;

if(UartRxBuf=='}')	{
	if(Uart_counter==9) 	{
		scheck = 0;
		for(i=0;i<8;i++)	{
			scheck +=UartRxData[i];
		}
		scheck = scheck%100;
		if(scheck==UartRxData[8])	{
		switch(UartRxData[1])
		{
			case CMD_EN:	{
				if(UartRxData[2]==0x11)		{
					BoardEn(1);
				}
				else 	{
					BoardEn(0);
				}
			}break;
			case CMD_DEBUG:	{
				if(UartRxData[2]==0x11)	{
					FlagDebug = 1;	}
				else	{
					FlagDebug = 0;
				}
			}break;
			case CMD_PWMSET:	{
				upwm = UartRxData[2]*100+UartRxData[3];
				switch(UartRxData[0])
				{
					case MOTOR_1:	{
						Motor1PWM(upwm);
					}break;
					case MOTOR_2:	{
						Motor2PWM(upwm);
					}break;
					case MOTOR_3:	{
						Motor3PWM(upwm);
					}break;
					case BOARDCAST:	{
						upwm = UartRxData[2]*100+UartRxData[3];
						Motor1PWM(upwm);
						upwm = UartRxData[4]*100+UartRxData[5];
						Motor2PWM(upwm);
						upwm = UartRxData[6]*100+UartRxData[7];
						Motor3PWM(upwm);
					}break;
					default :
					{}break;
				}
			}break;
			case CMD_FDBK:	{
				switch(UartRxData[2])	
				{
					case CTL_EncoderFDBK:	{
						EncoderFdbk();
					}break;
					case CTL_GriverForceFDBK:	{
						
					}break;
				}
			}break;
			case CMD_PWMFDBK:	{
				MotorPWMFdbk();
			}break;
			case CMD_CTRLMODE:	{
				switch (UartRxData[2]){
					case CTL_FREE:	{
						Ctrl.CtrlMode = CTL_FREE;
					}break;
					case CTL_ENABLE:	{
						Ctrl.CtrlMode = CTL_ENABLE;
					}break;
					case CTL_PUSHPULL:	{
						Ctrl.CtrlMode = CTL_PUSHPULL;
					}break;
					case CTL_SINGMOTOR:	{
						Ctrl.CtrlMode = CTL_SINGMOTOR;
					}break;
					default : 
						Ctrl.CtrlMode = CTL_FREE;
						break;
				}
			}break;
			case CMD_ForceSwitch:	{
				switch (UartRxData[2]){
					case CTL_StaFreFlag:	{
						SwStaFri(UartRxData[3]);
					}break;
					case CTL_DynFreFlag:	{
						SwDynFri(UartRxData[3]);
					}break;
					case CTL_GraFlag:	{
						SwGra(UartRxData[3]);
					}break;
					case CTL_InerFlag:	{
						SwIner(UartRxData[3]);
					}break;
					default : 
						break;
				}
			}break;
			case CMD_FLASH:	{
				switch(UartRxData[2])	{
					case CTL_FLASHCHANGE:	{
						FlashChange((UartRxData[4]*256+UartRxData[5]),UartRxData[3]);
					}break;
					case CTL_FLASHFDBK:	{
						FlashFDBK(UartRxData[3]);
					}break;
					case CTL_FLASHINIT:	{
						FlashInit();
					}break;
					default :
						break;
				}
				
			}break;
			case CMD_SENSOR:
			{
				if(UartRxData[2]==0x10)		{	//不需要将传感器结果返回上位机
					SensorCtrl(UartRxData[3],UartRxData[4]);
				}
			}break;				
			default :
			{}break;
		}
	}	
	}
	
	Uart_counter = 0;
	UartStartFlag = 0;
}

if(UartStartFlag==1)
{
	if(Uart_counter>8)  UartStartFlag = 0;
	else 
	{
		UartRxData[Uart_counter] = UartRxBuf;  //存入数组
		Uart_counter++;
	}	
}
if(UartRxBuf=='{')
{
	UartStartFlag = 1;
	Uart_counter = 0;
}
	
}
//传感器控制函数
char SensorCtrl(unsigned char cmd,unsigned char cmddata)
{
	//组织数据
	unsigned char data[6]={0};
	unsigned int sum = 0;
	char cnt = 0;
	data[0] = 0x7B;
	data[1] = cmd;
	data[2] = cmddata;
	for(cnt=1;cnt<4;cnt++)	{
		sum += data[cnt];
	}
	data[4] = sum%100;
	data[5] = 0x7D;
	//Uart发送
	HAL_UART_Transmit(&huart6,data,6,0xFFFF); 
	
	return 0;
}

// 电机编码器状态回读
void EncoderFdbk(void)
{
	unsigned char numm1,numm2,numm3,ii;
	unsigned int Summ;
	
		UartFeedBackData[0] = 0x7B;
		UartFeedBackData[1]	= 0x20;
		UartDataConver(&numm1,&numm2,&numm3,M1.EnCounter);
		UartFeedBackData[2]	= (unsigned char)numm1; 
		UartFeedBackData[3] = (unsigned char)numm2;
		UartFeedBackData[4]	= (unsigned char)numm3;		
		UartDataConver(&numm1,&numm2,&numm3,M2.EnCounter);
		UartFeedBackData[5] = (unsigned char)numm1;
		UartFeedBackData[6] = (unsigned char)numm2;
		UartFeedBackData[7] = (unsigned char)numm3;
		UartDataConver(&numm1,&numm2,&numm3,M3.EnCounter);
		UartFeedBackData[8] = (unsigned char)numm1;
		UartFeedBackData[9] = (unsigned char)numm2;
		UartFeedBackData[10]= (unsigned char)numm3;
		UartFeedBackData[11]= (unsigned char)0;
		UartFeedBackData[12]=  0;
		UartFeedBackData[13]= 0;
		Summ = 0;
		for(ii=1; ii<DataFdbkNum-2;ii++)	{
			Summ +=UartFeedBackData[ii];
		}
		Summ = Summ%100;
		UartFeedBackData[14] = Summ;
		UartFeedBackData[15] = 0x7D;
		
		HAL_UART_Transmit(&huart2, (uint8_t *)&UartFeedBackData, DataFdbkNum, 0xFFFF);
}

//电机pwm状态回读
void MotorPWMFdbk(void)	
{
	unsigned char numm1,numm2,numm3,ii;
	unsigned int Summ;
	
	UartFeedBackData[0] = 0x7B;
	UartFeedBackData[1]	= 0x20;
	UartDataConver(&numm1,&numm2,&numm3,TIM1->CCR1);
	UartFeedBackData[2]	= (unsigned char)numm1; 
	UartFeedBackData[3] = (unsigned char)numm2;
	UartFeedBackData[4]	= (unsigned char)numm3;		
	UartDataConver(&numm1,&numm2,&numm3,TIM1->CCR2);
	UartFeedBackData[5] = (unsigned char)numm1;
	UartFeedBackData[6] = (unsigned char)numm2;
	UartFeedBackData[7] = (unsigned char)numm3;
	UartDataConver(&numm1,&numm2,&numm3,TIM1->CCR3);
	UartFeedBackData[8] = (unsigned char)numm1;
	UartFeedBackData[9] = (unsigned char)numm2;
	UartFeedBackData[10]= (unsigned char)numm3;
	UartFeedBackData[11]= (unsigned char)0;
	UartFeedBackData[12]=  0;
	UartFeedBackData[13]= 0;
	Summ = 0;
	for(ii=1; ii<DataFdbkNum-2;ii++)	{
		Summ +=UartFeedBackData[ii];
	}
	Summ = Summ%100;
	UartFeedBackData[14] = Summ;
	UartFeedBackData[15] = 0x7D;
	
	HAL_UART_Transmit(&huart2, (uint8_t *)&UartFeedBackData, DataFdbkNum, 0xFFFF);
}

//返回数据   将long型数据转换为协议想要的格式
void UartDataConver(unsigned char *pNum1,unsigned char *pNum2,unsigned char *pNum3,long int Data)
{
	if(Data<0)
	{
		Data = -Data;
		*pNum1 = (unsigned char)(Data/65536);	
		*pNum1 = *pNum1 | 0x80;
	}
	else	{
		*pNum1 = (unsigned char)(Data/65536);
	}
	*pNum2 = (unsigned char)((float)(Data%65536)/256);
	*pNum3 = (unsigned char)(Data%256);
}
void DataCoverInt2Char(unsigned char *pNum1,unsigned char *pNum2,int pNum_Int)
{
	if(pNum_Int<0)	//unaigned int
	{
		pNum_Int = -pNum_Int;
	}
	*pNum1 = (unsigned char)(pNum_Int/256);
	*pNum2 = (unsigned char)(pNum_Int%256);
}

//板子上电 使能
void BoardEn(char ch)
{
	if(ch==1)		{
		HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(EN_MOTOR_GPIO_Port,EN_MOTOR_Pin,GPIO_PIN_SET);
	}
	else 	{
		HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(EN_MOTOR_GPIO_Port,EN_MOTOR_Pin,GPIO_PIN_RESET);
		MotorPwmMid(0);
	}
}


void FlashChange(unsigned int data,unsigned int num)
{
	unsigned char mm;
//		(unsigned int)(UartRxData[1]%100+UartRxData[2]/100);	//分高低位
		//检测 初始化FLASH的值
		if(num>4)	{	//小于4 不允许修改
			for(mm=0;mm<FLASHSIZE;mm++)	{
				FLASH_Store[mm] = (*(unsigned int *)(FLASH_USER_START_ADDR+4*mm));	//mm为定位变量
			}
			mm = num-1;
			FLASH_Store[mm] = data;
		
			HAL_FLASH_Unlock();
			EraseInitStruct.Banks=FLASH_BANK_1;
			EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
			EraseInitStruct.Sector = FLASH_SECTOR_7;	//擦除扇区1  参考原子代码
			EraseInitStruct.NbSectors = 1;
			EraseInitStruct.VoltageRange=FLASH_VOLTAGE_RANGE_3;	//电压范围区间  2.7~3.6V之间
			
		if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) == HAL_OK)	{
			STMFLASH_Write(FLASH_USER_START_ADDR,(u32*)FLASH_Store,FLASHSIZE);
//			for(mm=0;mm<FLASHSIZE;mm++)	{	//写入 
//				FLASH_Address = (FLASH_USER_START_ADDR + 4*mm);
//				if(HAL_FLASH_Program(FLASH_TYPEERASE_SECTORS, FLASH_Address, FLASH_Store[mm])==HAL_OK)	{	
//				}
//				else	{
//					gErrorStatus |= Error_FLASHErash;
//				}
//			}
		}
		else	{
			gErrorStatus |= Error_FLASHErash;
		}
		HAL_FLASH_Lock();	//上锁
	
	}
}
void FlashInit(void)
{
	HAL_FLASH_Unlock();
	
	EraseInitStruct.Banks=FLASH_BANK_1;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.Sector = FLASH_SECTOR_7;	//擦除扇区1  参考原子代码
	EraseInitStruct.NbSectors = 1;
	EraseInitStruct.VoltageRange=FLASH_VOLTAGE_RANGE_3;	//电压范围区间  2.7~3.6V之间
	
	
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) == HAL_OK)	{
		STMFLASH_Write(FLASH_USER_START_ADDR,(u32*)FLASH_Init,FLASHSIZE);
	}
	else	{
		gErrorStatus |= Error_FLASHErash;
	}
	HAL_FLASH_Lock();	//上锁
	
}
void FlashFDBK(char num)
{
	char mm = 0,part;
	unsigned char numm1,numm2,ii;
	int Summ;
	if(num>0&&num<=6)
		part = 1;
	else if(num>6&&num<=12)
		part = 2;
	else if(num>12&&num<=18)
		part = 3;
	else if(num>18&&num<=24)
		part = 4;
	
	for(mm=0;mm<FLASHSIZE;mm++)	{
				FLASH_Store[mm] = (*(unsigned int *)(FLASH_USER_START_ADDR+4*mm));	//mm为定位变量
			}
		if(part>0&&part<5)	{	//part 1-4
			
			for(mm=0;mm<6;mm++)	{
				DataCoverInt2Char(&numm1,&numm2,(int)FLASH_Store[(part-1)*6 + mm]);
				UartFeedBackData[mm*2+2]	= (unsigned char)numm1; 
				UartFeedBackData[mm*2+3] = (unsigned char)numm2;
			}
		
			}
		else	{
			for(mm=0;mm<6;mm++)	{
				UartFeedBackData[mm*2+2]= (unsigned char)0; 
				UartFeedBackData[mm*2+3] = (unsigned char)0;
			}
		}
	
		UartFeedBackData[0] = 0x7B;
		UartFeedBackData[1]	= 0x30;		//FLASH回读
		Summ = 0;
		for(ii=1; ii<DataFdbkNum-2;ii++)	{
			Summ +=UartFeedBackData[ii];
		}
		Summ = Summ%100;
		UartFeedBackData[14] = Summ;
		UartFeedBackData[15] = 0x7D;
		
		HAL_UART_Transmit(&huart2, (uint8_t *)&UartFeedBackData, DataFdbkNum, 0xFFFF);
}


 //不同类型力补偿开关
void SwDynFri(unsigned char state)	{
	if(state==0||state==1)	{
		CtrlM1.DynFriFlag=state;
		CtrlM2.DynFriFlag = state;
		CtrlM3.DynFriFlag = state;
	}
}
void SwStaFri(unsigned char state)	{
	if(state==0||state==1)	{
		CtrlM1.StaFriFlag = state;
		CtrlM2.StaFriFlag = state;
		CtrlM3.StaFriFlag = state;
	}
}
void SwGra(unsigned char state)	{
	if(state==0||state==1)	{
		CtrlM1.GraFlag = state;
		CtrlM2.GraFlag = state;
		CtrlM3.GraFlag = state;
	}
}
void SwIner(unsigned char state)	{
	if(state==0||state==1)	{
		CtrlM1.InerFlag = state;
		CtrlM2.InerFlag = state;
		CtrlM3.InerFlag = state;
	}
}
	
//printf 实现
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}


/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
