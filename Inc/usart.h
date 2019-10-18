/**
  ******************************************************************************
  * File Name          : USART.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "tim_ctrl.h"
#include "stm32H7xx_hal_flash_ex.h"
#include "include.h"
/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */

//CMD为大类的指令    
#define 	CMD_EN 			0x10
#define		CMD_PWMSET		0x20
#define		CMD_FDBK		0x30
#define		CMD_PWMFDBK		0x31
#define 	CMD_DEBUG		0x80


#define		MOTOR_1			0x01
#define		MOTOR_2			0x02
#define		MOTOR_3			0x03
#define     BOARDCAST		0x79

#define 	CMDD_EN			0x11
#define		CMDD_DEN		0x10


#define 	CMD_CTRLMODE	0x40	//控制模式  
#define		CTL_FREE		0x10	//free Mode
#define		CTL_ENABLE		0x11	//board enable
#define 	CTL_PUSHPULL	0x12	//
#define		CTL_SINGMOTOR	0x13	//单个电机补偿 single

#define		CMD_FLASH		0x50	//FLASH相关
#define		CMD_SENSOR		0x60	//传感器相关指令

#define		CTL_FLASHFDBK		0x10	//FLASH回读
#define		CTL_FLASHCHANGE		0x11
#define		CTL_FLASHINIT		0x13
#define		CTL_FLASHINIT_CHANGE	0x14  //修改FLASH_Init 数组

#define		CMD_ForceSwitch		0x41	//不同种类力补偿类型
#define		CTL_StaFreFlag		0x11
#define		CTL_DynFreFlag		0x12
#define		CTL_GraFlag			0x13
#define		CTL_InerFlag		0x14

#define 	CTL_EncoderFDBK		0x20
#define 	CTL_GriverForceFDBK	0x60

//数据交换部分  宏定义
#define RET_POSITION  0x31		//返回当前位置
#define	RxSize	10				//传感器向主控传递数据  长度10字节

/*************************const define******************************/
#define 	DataFdbkNum		(16)	//返回的 协议长度

#define Error_Uart3Sort			(1<<3)		//通讯  是否及时  即单片机收到的数据Sort是否是最新的
#define Error_Uart3RX			(1<<4)		//单片机收到校验失败的数据  或一定时间内没有收到新数据（数据序列在一定时间内不变化）
#define Error_Uart3TX			(1<<5)		//可以由上位机置位
#define Error_UART3RXOutRange	(1<<6)		//单片机接收到的  电机力的数据  超出范围

/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void USART2Interrupt(char UartRxBuf);
void USART3Interrupt(char UartRxBuf);
void USART6Interrupt(unsigned char *UartRxBuf);
void EncoderFdbk(void);
void MotorPWMFdbk(void);
void UartDataConver(unsigned char *pNum1,unsigned char *pNum2,unsigned char *pNum3,long int Data);
void BoardEn(char ch);
void FlashChange(unsigned int data,unsigned int num);
void DataCoverInt2Char(unsigned char *pNum1,unsigned char *pNum2,int pNum_Int);
void FlashFDBK(char part);
void FlashInit(void);
void CalcMotorForce(float *fData,unsigned char cData1,unsigned char cData2,unsigned char cData3);

void PositionDataCover(unsigned char *num1,unsigned char *num2,unsigned char *num3,float f);
char retPosition(void);
char SensorCtrl(unsigned char cmd,unsigned char cmddata);

void SwDynFri(unsigned char state);
void SwStaFri(unsigned char state);
void SwGra(unsigned char state);
void SwIner(unsigned char state);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
