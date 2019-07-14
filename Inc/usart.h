/**
  ******************************************************************************
  * File Name          : USART.h
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

/* USER CODE BEGIN Private defines */

//CMDΪ�����ָ��    
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


#define 	CMD_CTRLMODE	0x40	//����ģʽ  
#define		CTL_FREE		0x10	//free Mode
#define		CTL_ENABLE		0x11	//board enable
#define 	CTL_PUSHPULL	0x12	//
#define		CTL_SINGMOTOR	0x13	//����������� single

#define		CMD_FLASH		0x50	//FLASH���

#define		CTL_FLASHFDBK		0x10	//FLASH�ض�
#define		CTL_FLASHCHANGE		0x11
#define		CTL_FLASHINIT		0x13
#define		CTL_FLASHINIT_CHANGE	0x14  //�޸�FLASH_Init ����

#define		CMD_ForceSwitch		0x41	//��ͬ��������������
#define		CTL_StaFreFlag		0x11
#define		CTL_DynFreFlag		0x12
#define		CTL_GraFlag			0x13
#define		CTL_InerFlag		0x14

#define 	CTL_EncoderFDBK		0x20
#define 	CTL_GriverForceFDBK	0x60

//���ݽ�������  �궨��
#define RET_POSITION  0x31		//���ص�ǰλ��


/*************************const define******************************/
#define 	DataFdbkNum		(16)	//���ص� Э�鳤��

#define Error_Uart3Sort			(1<<3)		//ͨѶ  �Ƿ�ʱ  ����Ƭ���յ�������Sort�Ƿ������µ�
#define Error_Uart3RX			(1<<4)		//��Ƭ���յ�У��ʧ�ܵ�����  ��һ��ʱ����û���յ������ݣ�����������һ��ʱ���ڲ��仯��
#define Error_Uart3TX			(1<<5)		//��������λ����λ
#define Error_UART3RXOutRange	(1<<6)		//��Ƭ�����յ���  �����������  ������Χ

/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void USART2Interrupt(char UartRxBuf);
void USART3Interrupt(char UartRxBuf);
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
