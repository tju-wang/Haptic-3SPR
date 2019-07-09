#ifndef __include_H
#define __include_H
#ifdef __cplusplus
 extern "C" {
#endif
	
	
#define Error_FLASHErash		(1<<20)	//第20位  置1表示FLASH擦除错误




#define PWM_MIN		(int)(100)
#define	PWM_MAX		(int)(900)
#define KEn2q		((float)(220.0f)) 
	
#define DataNum  5

///*************************** FLASH  Address **************************************************/
//#define ADDR_FLASH_PAGE_14    ((uint32_t)0x08003800) /* Base @ of Page 14, 1 Kbyte */
//#define ADDR_FLASH_PAGE_15    ((uint32_t)0x08003C00) /* Base @ of Page 15, 1 Kbyte */
//#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_14   /* Start @ of user Flash area */
//#define FLASH_USER_END_ADDR     ADDR_FLASH_PAGE_15 + FLASH_PAGE_SIZE 

/*************************** FLASH  Address **************************************************/
//#define FLASH_SECTOR_SIZE          (0x2000U)	//sector 大小为128k
#define ADDR_FLASH_SECTOR1    ((uint32_t)0x08020000) /* Base @ of sector1, 128 Kbyte */
#define ADDR_FLASH_SECTOR7    ((uint32_t)0x080E0000) /* Base @ of Bank1 sector7, 128 Kbyte */

#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR7   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     FLASH_USER_START_ADDR + FLASH_SECTOR_SIZE 

#define 	FLASHSIZE	(24)


#define         ProductNum					(*(unsigned int *)(uint32_t)0x080E0000)		//32bit    	1                        
#define         SoftwareNum            		(*(unsigned int *)(uint32_t)0x080E0004)
#define			MechanicalNum				(*(unsigned int *)(uint32_t)0x080E0008) 
#define			HardwareNum					(*(unsigned int *)(uint32_t)0x080E000C) 
	
#define			PWMMID						(*(unsigned int *)(uint32_t)0x080E0010)		//			5				
#define			PULL_PWM					(*(unsigned int *)(uint32_t)0x080E0014)	
#define			SigPWMPulse					(*(unsigned int *)(uint32_t)0x080E0018)	
#define			StopPwm						(*(unsigned int *)(uint32_t)0x080E001C)	
#define			PLPH_Para_k					((float)(*(unsigned int *)(uint32_t)0x080E0020)/1000)	
#define			PLPH_Para_b					((int)(*(unsigned int *)(uint32_t)0x080E0024))	//		10


#define			Griver_Para					((float)(*(unsigned char*)(uint32_t)0x080E0028)/1000)			
#define			InertiaPara					((float)(*(unsigned int *)(uint32_t)0x080E002C)/1000)	
	
#define			LED1Blue					(*(unsigned int *)(uint32_t)0x080E0030)	

#define			LED2Red					(*(unsigned int *)(uint32_t)0x080E0034)			
#define			LED2Green				(*(unsigned int *)(uint32_t)0x080E0038)	
#define			LED2Blue				(*(unsigned int *)(uint32_t)0x080E003C)	
	
#define			HARDWARE1				(*(unsigned int *)(uint32_t)0x080E0040) 
#define			SOFTWARE1				(*(unsigned int *)(uint32_t)0x080E0044) 

#define			HARDWARE2				(*(unsigned int *)(uint32_t)0x080E0048) 
#define			SOFTWARE2				(*(unsigned int *)(uint32_t)0x080E004C) 
#define			HARDWARE3				(*(unsigned int *)(uint32_t)0x080E0050) 
#define			SOFTWARE3				(*(unsigned int *)(uint32_t)0x080E0054)
	
#define			HARDWARE				(*(unsigned int *)(uint32_t)0x080E0058) 
#define			SOFTWARE				(*(unsigned int *)(uint32_t)0x080E005C)


//typedef struct _varq {
//    double q1,q2,q3;
//} var_q;


#include "tim.h"	
#include "tim_ctrl.h"
#include "main.h"
#include "usart.h"
#include "motor.h"
#include "stmflash.h"
	
	 




	 
	 


#endif
	 

