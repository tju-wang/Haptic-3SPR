/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define __FPU_PRESENT 1		//FPU present
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Moter_t M1,M2,M3;
Ctrl_t	Ctrl,CtrlM1,CtrlM2,CtrlM3;

unsigned int gErrorStatus;	//全局变量   用来显示错误提示
double Val_1,Val_2;


extern uint8_t RXBuffer[1];
extern uint8_t RXBuffer_3[1];

extern int Data1[],Data2[];
extern char FlagRecord,FlagPrint,FlagDebug;
extern unsigned int DataNumm;
extern unsigned char Uart3FeedBackData[];

extern var_t var;
extern var_q varq;
extern var_q TempLq;
extern force_t F1,F2,F3;
extern MotorGriverForce_t MotorForce;
extern char CalcOverFlag;


uint32_t FLASH_Address = 0, PageError = 0;
__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;

unsigned int FLASH_Store[FLASHSIZE];	//修改FLASH时临时存储数组
unsigned int FLASH_Init[FLASHSIZE]={
	0,	//产品型号											1
	0,	//软件版本											2
	0,	//机械结构版本										3
	0,	//电路硬件版本										4
	
//状态参数
	500,	//PWMMID-ui  	电机零扭力参考PWM值					5
	15,	//PULL_PWM-ui	电机同向推拉时 增加的pwm值			6
	28,	//SigPWMPulse-ui 检测到转动后  单电机摩擦力补偿		7
	30,	//StopPwm-ui	检测到静止时  正反向的摩擦力波动	8
	88,	//PLPH_Para_k-f	推拉随速度变化力补偿值 一次函数 k值	9
	4,	//PLPH_Para_b-i										10
			//return (float)(0.PLPH_Para_k*sp+PLPH_Para_b);  返回本项计算PWM值
	11,	//Griver_Para-f	  重力补偿  电机力 mN 和 PWM之间的对应关系	11								
	40,	//InertiaPara-f   惯性力计算 与加速度相乘的系数		12
	0x00,	//LED1Blue
	
	0x00,	//Led2Red		14
	0x00,	
	0x00,	
	
	0x00,	//Led3Red		17
	0x00,	

	
	0x00,	//HardWare Version	29
	0x00,	//SoftWare Version	30
	0x00,	//HardWare Version	29
	0x00,	//SoftWare Version	30
	0x00,	//HardWare Version	29
	0x00	//SoftWare Version	30
};

const unsigned int TEXT_Buffer[]={0x00,0x02,0x03,0x55,0x78,0x56};
#define TEXT_LENTH sizeof(TEXT_Buffer)	 		  	//数组长度	
#define SIZE TEXT_LENTH/4+((TEXT_LENTH%4)?1:0)

#define FLASH_SAVE_ADDR  0X08020000 	//设置FLASH 保存地址(必须为4的倍数，且所在扇区,要大于本代码所占用到的扇区.
										//否则,写操作的时候,可能会导致擦除整个扇区,从而引起部分程序丢失.引起死机.

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART3_UART_Init();
  
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  	HAL_UART_Receive_IT(&huart2, (uint8_t *)RXBuffer, 1); //打开中断接收
	HAL_UART_Receive_IT(&huart3, (uint8_t *)RXBuffer_3, 1); //打开中断接收
	
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim5);
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);

	ParaInit();
	char mm,Flag_blank = 0;	//FLASH是否为空白  标志	;
	int Flash_Value;
	 HAL_StatusTypeDef FlashStatus=HAL_OK;
	FLASH_Address = FLASH_USER_START_ADDR;
	for(mm=0;mm<FLASHSIZE;mm++)	{
			Flash_Value = *(int*)(FLASH_USER_START_ADDR+(mm)*4);
			if((Flash_Value==-1)&&(Flag_blank==0))
				Flag_blank = 0;		//此FLASH区域为空白
			else 	{
				Flag_blank = 1;  //非空白	
				break;
			}
		}
	if(Flag_blank==0)	{
		STMFLASH_Write(FLASH_USER_START_ADDR,(u32*)FLASH_Init,FLASHSIZE);
	}

	
	HAL_TIM_Base_Start_IT(&htim14); //最后开启250us中断处理
	HAL_TIM_Base_Start_IT(&htim16); //开启10ms的 重力补偿中断
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//  CalcTest();
//  while(1);
	
//	KinematicTest();
	
	unsigned int numm,kk;
//  for(numm=0;numm<20;numm++)
//	{
//		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_SET);
//		for(kk=0;kk<10;kk++)
//		{
//				varq.q1 = 271;  varq.q2 = 251; varq.q3 = 281;
//				ForwardKinematic();
//				CPointSolve(var);
//				FbSolve();
//	//			CtrlM1.Gravity = F1.Fx;
//	//			CtrlM2.Gravity = F2.Fx;
//	//			CtrlM3.Gravity = F3.Fx;
//		}
//		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_RESET);
//		
//		HAL_Delay(1);
//		printf("F1 = %6.3f  F2 = %6.3f  F3 = %6.3f \n",F1.Fx,F2.Fx,F3.Fx);
//		PrintVarStruct(var,varq);
//	}

	
	
  while (1)
  {
	  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_SET);
	  HAL_Delay(200);
	  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_RESET);
	  HAL_Delay(200);
	  
	  if(FlagPrint==1&&FlagDebug==1)
	  {
		  BoardEn(0);
		  FlagPrint = 0;
		  FlagRecord = 0;
		  FlagDebug = 0;
		  DataNumm = 0;
		 //关闭定时器中断
		  HAL_TIM_Base_Stop_IT(&htim14);
		  for(numm=0;numm<DataNum;numm++)	{
			printf("   %3d   %4d  %4d \n",numm,Data1[numm],Data2[numm]);
		  }
		  printf("数据输出完毕！ \n");
		  for(numm=0;numm<DataNum;numm++)	{
			  Data1[numm] = 0;
			  Data2[numm] = 0;
		  }
		  HAL_TIM_Base_Start_IT(&htim14); //最后开启250us中断处理
	  }

//	  M2.EnCounter = (M2.EnOverflowNum)*ENCODER_NUM+( int16_t )__HAL_TIM_GET_COUNTER(&htim3);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Supply configuration update enable 
  */
  MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);
  /**Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY) 
  {
    
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART2;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void ParaInit(void)
{
	EncoderClear();
	CtrlStruct_Init();
	var = VarDataInit(var);
}

void EncoderClear(void)
{
	//结构体初始化
	M1.EnCounter = M1.EnOverflowNum = 0;
	
	M2.EnCounter = M2.EnOverflowNum = 0;
	
	M3.EnCounter = M3.EnOverflowNum = 0;
	
	__HAL_TIM_SET_COUNTER(&htim3,0);
	__HAL_TIM_SET_COUNTER(&htim4,0);
	__HAL_TIM_SET_COUNTER(&htim5,0);
	//溢出值需要清零 在打开TIM3中断后会进入一次全局中断
	M1.EnOverflowNum = M2.EnOverflowNum = M3.EnOverflowNum = 0;
}
void CtrlStruct_Init(void)
{
	unsigned char DynFriFlag,GraFlag,InerFlag,StaFriFlag;
	DynFriFlag = 0;
	StaFriFlag = 0;		//静摩擦 尝试补偿  ==1  会有高频的噪声  效果不突出
	GraFlag = 1;
	InerFlag = 0;
	
	CtrlM1.DynFriFlag=DynFriFlag;
	CtrlM1.StaFriFlag = StaFriFlag;
	CtrlM1.GraFlag = GraFlag;
	CtrlM1.InerFlag = InerFlag;
	
	CtrlM2.DynFriFlag = DynFriFlag;
	CtrlM2.StaFriFlag = StaFriFlag;
	CtrlM2.GraFlag = GraFlag;
	CtrlM2.InerFlag = InerFlag;
	
	CtrlM3.DynFriFlag = DynFriFlag;
	CtrlM3.StaFriFlag  =StaFriFlag;
	CtrlM3.GraFlag = GraFlag;
	CtrlM3.InerFlag = InerFlag;
	
}
void KinematicTest()
{
	HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_SET);

	varq.q1 = 261;  varq.q2 = 261; varq.q3 = 261;
	ForwardKinematic();
	CPointSolve(var);
	FbSolve();

	HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_RESET);
	printf("F1 = %6.3f  F2 = %6.3f  F3 = %6.3f \n",F1.Fx,F2.Fx,F3.Fx);
	PrintVarStruct(var,varq);
	while(1);
}

void Kinematic(void)
{
	char Status = 0;
	CalcOverFlag = 0;
	Status = InputDataCheck(&var,&varq);
	if(Status==0)	{
		ForwardKinematic();
		CPointSolve(var);
		FbSolve();
	}
	Status = OutputDataCheck(&F1,&F2,&F3);
	
	if(Status==0)	{
		MotorForce.F1 = F1.Fx;
		MotorForce.F2 = F2.Fx;
		MotorForce.F3 = F3.Fx;
	}
	
//	printf("F1 = %6.3f  F2 = %6.3f  F3 = %6.3f \n",F1.Fx,F2.Fx,F3.Fx);
//	PrintVarStruct(var,varq);
	
	CalcOverFlag = 1;
}

/*输入运动学算法的数据的检验  约束
1.上一次的角度变量
2.杆长变量
*/
char InputDataCheck(var_t *var_i,var_q *varq_i)
{
	char Status = 0;
	const static float ang = 1.2;
	//0.872  转化为角度是 50度  输入的迭代初值大于50度 认为超出平台姿态范围
	if((var_i->alpha) < (-ang) | (var_i->alpha)> (ang))	{
		Status |= (1<<0);
		var_i->alpha = 0;
	}
	if((var_i->beta) < (-ang) | (var_i->beta)> (ang))	{
		Status |= (1<<1);
		var_i->beta = 0;
	}
	if((var_i->gama) < (-ang) | (var_i->gama)> (ang))	{
		Status |= (1<<2);
		var_i->gama = 0;
	}
	
	if( (varq_i->q1)<170 | (varq_i->q1)>330 )
		Status |= (1<<3);
	if( (varq_i->q2)<85 | (varq_i->q2)>290 )
		Status |= (1<<4);
	if( (varq_i->q3)<140 | (varq_i->q3)>330 )
		Status |= (1<<5);
	if(Status!=0)	//data out of range
	{
		#ifdef DebugStatusOutput
			printf("InputDataCheck Status = %c \n",Status);
		#endif
	}
	return Status;
}

/*检查输出结果  力的大小*/
char OutputDataCheck(force_t *force1,force_t *force2,force_t *force3)
{
	char Status = 0;
	const static int GriverRange = 250;		//PWM值  即 重力得出的 力的值  不允许大于 250 PWM
	if( (force1->Fx)>GriverRange/Griver_Para | (force1->Fx)< -GriverRange/Griver_Para)
		Status |= (1<<0);
	if( (force2->Fx)>GriverRange/Griver_Para | (force2->Fx)< -GriverRange/Griver_Para)
		Status |= (1<<1);
	if( (force3->Fx)>GriverRange/Griver_Para | (force3->Fx)< -GriverRange/Griver_Para)
		Status |= (1<<2);
	
	if(Status!=0)	//data out of range
	{
		#ifdef DebugStatusOutput
			printf("OutputDataCheck Status = %c \n",Status);
		#endif
	}
	
	return Status;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
