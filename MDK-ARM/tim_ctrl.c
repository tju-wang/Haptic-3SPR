#include "tim_ctrl.h"
//#include "kinematic.h"

extern Moter_t M1,M2,M3;
extern Ctrl_t	Ctrl,CtrlM1,CtrlM2,CtrlM3;
unsigned long int IRQ_Counter = 0;
unsigned int IRQTimer16Counter = 0;
unsigned char GriverFDBKCounter = 0;
unsigned char KeyValue = 0;
unsigned char Uart3Error = 0;
unsigned char CalcOverFlag = 1;
unsigned char CalcOver = 1;
unsigned int CalcTimeCounter = 0;

int Data1[DataNum]={0},Data2[DataNum]={0};
char FlagRecord = 0,FlagPrint=0,FlagDebug=0;
unsigned int DataNumm = 0;
extern unsigned char Uart3FeedBackData[];
var_q TempLq;
MotorGriverForce_t MotorForce;

//extern var_t var;
extern var_q varq;

//Time Interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)	
{
	//static unsigned char Uart3Sort;
	static unsigned int Uart3ErrCounter;
	if(htim->Instance == htim3.Instance)	{	//TIM3 EncoderFlow
		EncoderFlow(&M1,htim3);
	}
	if(htim->Instance == htim4.Instance)	{	
		EncoderFlow(&M2,htim4);
	}
	if(htim->Instance == htim5.Instance)	{	
		EncoderFlow(&M3,htim5);
	}
	if(htim->Instance == htim14.Instance)	//250us 中断
	{
		IRQ_Counter++;
		if(IRQ_Counter>=0xFFFFFFFF)
			IRQ_Counter = 0;
		
		EncoderUpdate(&M1,htim3);
		EncoderUpdate(&M2,htim4);
		EncoderUpdate(&M3,htim5);
		
//		EnCounter2q(M1.EnCounter,M2.EnCounter,M3.EnCounter,&(TempLq.q1),&(TempLq.q2),&(TempLq.q3));
			
		if(CalcOverFlag==1)	{
			EnCounter2q(M1.EnCounter,M2.EnCounter,M3.EnCounter,&(varq.q1),&(varq.q2),&(varq.q3));
			CalcOver = 1;
			CalcTimeCounter = 0;
		}
		else {
			CalcTimeCounter++;
		}
		
		MotorState(&M1);
		MotorState(&M2);
		MotorState(&M3);
		
		
		FricCalcu(&CtrlM1,&M1);
		FricCalcu(&CtrlM2,&M2);
		FricCalcu(&CtrlM3,&M3);
		
		GriverCalc(&CtrlM1,&M1);
		GriverCalc(&CtrlM2,&M2);
		GriverCalc(&CtrlM3,&M3);
		
		InertiaCalcu(&CtrlM1,&M1);
		InertiaCalcu(&CtrlM2,&M2);
		InertiaCalcu(&CtrlM3,&M3);
		
		CtrlPwmSum(&CtrlM1);
		CtrlPwmSum(&CtrlM2);
		CtrlPwmSum(&CtrlM3);
		
		SetMotorPWM();
		
		if(FlagDebug==1)	{
			if(FlagRecord==1)	{
				if(DataNumm<DataNum)	{
				Data1[DataNumm] = M1.EnCounter;
				Data2[DataNumm] = CtrlM1.PwmSum;
				DataNumm ++;
				}
				else {
					FlagPrint = 1;
					FlagRecord = 0;	
				}
			}
			if(Data1[DataNum-2]!=0|DataNumm>=DataNum-2)	{
				FlagRecord = 0;
				FlagPrint = 1;
			}
			if(FlagRecord==0&&((M1.EnCounter>200)))
				FlagRecord = 1;
		}
		
//		CtrlPwmSum();
	}//end...if(htim->Instance == h
	if(htim->Instance == htim16.Instance)	//10ms 中断  刷新重力补偿数据
	{
		if(IRQTimer16Counter<0xFFFFFFFE)
			IRQTimer16Counter++;
		else 
			IRQTimer16Counter = 0;
	
		if(CalcOver==1)		{		//运动学计算标志
			Kinematic();
		}
		else	{
			
		}
		
		
//与电脑上位机通讯部分代码  实时上传杆长数据		
//		//定时判断  收发通讯是否成功
//		if(GriverFDBKCounter==(Uart3Sort))	{
//			Uart3Error &= ~(Error_Uart3Sort);	//ErrorSort 置位
//			Uart3ErrCounter = 0;
//		}
//		else {
//			Uart3Error |= Error_Uart3Sort;	//ErrorSort 置位
//			Uart3ErrCounter++;
//		}
//		if(Uart3ErrCounter>20)	{	//连续10个周期通讯失败  失能电路
//			BoardEn(0);		//失能电路  LED红灯亮
//			Uart3ErrCounter = 0;
//		}
//		SendqData(TempLq,&Uart3Sort);	//串口3  定时返回P副长度数据
		
	}
	
}
void SendqData(var_q temp,unsigned char* psort)
{
	int i,Sum;
	Uart3FeedBackData[0] = 0x7B;
	Uart3FeedBackData[1] = 0;
	Uart3FeedBackData[2] = 0x10;
	Float2Char(temp.q1,&Uart3FeedBackData[3],&Uart3FeedBackData[4]);
	Float2Char(temp.q2,&Uart3FeedBackData[5],&Uart3FeedBackData[6]);
	Float2Char(temp.q3,&Uart3FeedBackData[7],&Uart3FeedBackData[8]);
	Uart3FeedBackData[9] = (unsigned char)Uart3Error;
	Uart3FeedBackData[10] = GriverFDBKCounter;
	*psort = (unsigned char)((unsigned int)IRQTimer16Counter%100);
	Uart3FeedBackData[11] = *psort;
	Sum = 0;
	for(i=1;i<12;i++)
	{
		Sum+=Uart3FeedBackData[i];
	}
	Sum = Sum%100;
	Uart3FeedBackData[12] = Sum;
	Uart3FeedBackData[13] = 0x7D;
	HAL_UART_Transmit(&huart3, (uint8_t *)&Uart3FeedBackData, 14, 0xFFFF);

}

void Float2Char(float fData,unsigned char *cDataH,unsigned char *cDataL)
{
	*cDataH = (unsigned char)(fData/10);
	*cDataL = (unsigned char)((int)(fData*10)%100);
}
	

void CtrlPwmSum(Ctrl_t *pMotor)	//将各因素叠加
{
	pMotor->PwmSum = PWMMID;
	//求各因素之和
	if(pMotor->DynFriFlag==1)	{
		pMotor->PwmSum +=pMotor->DynFric;
	}
	if(pMotor->GraFlag==1)	{
		pMotor->PwmSum +=pMotor->Gravity;
	}
	if(pMotor->InerFlag==1)	{
		pMotor->PwmSum +=pMotor->InertiaForce;
	}
	if(pMotor->StaFriFlag==1)	{
		pMotor->PwmSum += pMotor->StaticFric;
	}
	
}
//摩擦力测量  
void FricTest(unsigned char num)
{
	//记录一段时间内的编码器计数值   检测电机状态  速度等、
	//同时记录当前状态下pwm值
	
}
//惯性力计算  以单杆为准
void InertiaCalcu(Ctrl_t *pCtrl,Moter_t *pMotor)	{
	if(pCtrl->InerFlag==1)	{
		pCtrl->InertiaForce = -(pMotor->accelration * InertiaPara);
	}
}
//重力补偿计算  根据上位机传来的电机力
void GriverCalc(Ctrl_t *pCtrl,Moter_t *pMotor)	{
	if(pCtrl->GraFlag==1)	{
		if(pCtrl==&CtrlM1)	{
			pCtrl->Gravity = MotorForce.F1 * Griver_Para;
		}
		else if(pCtrl==&CtrlM2)	{
			pCtrl->Gravity = MotorForce.F2 * Griver_Para;
		}
		else if(pCtrl==&CtrlM3)	{
			pCtrl->Gravity = MotorForce.F3 * Griver_Para;
		}
	}
}


//摩擦力计算  电机正反转 主动加力克服动摩擦   电机停转 加正反向切换的力
void FricCalcu(Ctrl_t *pCtrl,Moter_t *pMotor)	{
	if(pCtrl->DynFriFlag==1)	{
		if(pMotor->runstate==1)		{
			pCtrl->DynFric = SigPWMPulse;
			pCtrl->StaticFric = 0;
		}
		else if(pMotor->runstate==-1)	{
			pCtrl->DynFric = -SigPWMPulse;
			pCtrl->StaticFric = 0;
		}
		else 	{	//静摩擦
			pCtrl->DynFric = 0;
			if(pCtrl->t>60000)
				pCtrl->t = 0;
			else
				pCtrl->t ++;
			
			if(pCtrl->t%2==1)
				pCtrl->StaticFric = StopPwm;
			else if(pCtrl->t%2==0)
				pCtrl->StaticFric = -StopPwm;
		}
		if(pCtrl==&CtrlM3)	{	//在三次调用中  只有最后一次进行同向检测
			if((M1.runstate==M2.runstate)&&(M1.runstate==M3.runstate)&&(M1.runstate!=0))	{	//同向推拉检测
				if(CtrlM1.DynFric>0)
				{
					CtrlM1.DynFric += PullPushSpeed(M1.speed);
					CtrlM2.DynFric += PullPushSpeed(M2.speed);
					CtrlM3.DynFric += PullPushSpeed(M3.speed);
				}
				else {
					CtrlM1.DynFric -= PullPushSpeed(M1.speed);
					CtrlM2.DynFric -= PullPushSpeed(M2.speed);
					CtrlM3.DynFric -= PullPushSpeed(M3.speed);
				}
			}	//end if((M1.runstate==...
		}
	}	//end if(pCtrl->DynFriFlag==1)
	
}

float PullPushSpeed(unsigned int sp)	//推拉主动补偿  随速度变化量
{
	return (float)(PLPH_Para_k*sp+PLPH_Para_b);		//计算估计值为 0.075+9   但是 计算的问题在于原始数据由感觉得来 不准确， 摩擦与速度间不一定为一次关系  
}									//需要修正  考虑：1.静止状态下的卡顿  2.快速推拉 操作力的大小


unsigned char MotorState(Moter_t *pState)	//电机状态监测  改变runstate及speed值  计算加速度值
{
	long int Sum1,Sum2;
	char parr,kk;
	parr = pState->pArr+EncoderStoreNum;	//在每一次编码器值更新的时候  pArr ++ 
	Sum1 = Sum2 = 0;
	for(kk=0; kk<5; kk++)	{
		Sum1 += pState->EnCoterArr[(parr-kk)%EncoderStoreNum];
		Sum2 += pState->EnCoterArr[(parr+kk+1)%EncoderStoreNum];
	}
//	for(kk=0; kk<5; kk++)	{	//尝试优化速度求取部分  需要调整各环节力的补偿参数
//		Sum1 += pState->EnCoterArr[(parr-kk)%EncoderStoreNum];
//		Sum2 += pState->EnCoterArr[(parr-kk-5)%EncoderStoreNum];
//	}
//	Sum1 = pState->EnCoterArr[(parr)%EncoderStoreNum];
//	Sum2 = pState->EnCoterArr[(parr-1)%EncoderStoreNum];
	
	pState->speed = (Sum1-Sum2);
	if((pState->speed)<-StateRef)	{
		pState->runstate = -1;
	}
	else if((pState->speed)>StateRef)	{
		pState->runstate = 1;
	}
	else	{
		pState->runstate = 0;
	}
	pState->pSpArr +=1;
	pState->pSpArr %=EncoderStoreNum;
	pState->MoterSpeedArr[pState->pSpArr] = pState->speed;	//存储之前的速度数据
	
	if(pState->speed<0)	pState->speed = -pState->speed;	 //推拉  慢速 速度大约0x50->80  快速推拉 速度约0x210 -> 528
	//计算加速度
	parr = pState->pSpArr+EncoderStoreNum;	//在每一次编码器值更新的时候  pArr ++ 
	Sum1 = Sum2 = 0;
	for(kk=0; kk<5; kk++)	{
		Sum1 += pState->MoterSpeedArr[(parr-kk)%EncoderStoreNum];
		Sum2 += pState->MoterSpeedArr[(parr+kk+11)%EncoderStoreNum];
	}
	if(Sum2>60||(Sum2<-60))	{	//低速检测  0x20 * 5 	
		pState->accelration = Sum1-Sum2;	//计算加速度 抵消惯性项  保留加速度方向
	}
	else	{
		pState->accelration = 0;
	}
	return 1;
	
}
////数据转换
//char Int2Char(unsigned char *pNum1,unsigned char *pNum2,int pNum_Int)
//{
//	if(pNum_Int>=0)	//int
//	{
//		*pNum1 = (unsigned char)(pNum_Int/256);
//		*pNum2 = (unsigned char)(pNum_Int%256);
//		return 1;
//	}
//	else	{
//		*pNum1 = (unsigned char)(-pNum_Int/256);
//		*pNum2 = (unsigned char)(-pNum_Int%256);
//		*pNum1 |= 0x80; 	//最高位置1
//		return 0;
//	}
//	
//}


void EnCounter2q(signed long int EnCounter1,signed long int EnCounter2,signed long int EnCounter3,double *q1,double *q2,double *q3)		//杆长与编码器计数值转换
{
	//根据编码器计数值结合修正参数  转化为杆长
	*q1 = (double)((float)EnCounter1 / KEn2q + 171.0f + 20.0f);		//171为 在复位位置  L杆原长   20为球铰杆长度
	
//	*q2 = (double)((float)EnCounter2 / KEn2q +86.0f + 20.0f);
//	
//	*q3 = (double)((float)EnCounter3 / KEn2q + 150.0f +20.0f);
	*q2 = (double)((float)EnCounter2 / KEn2q +70.0f + 20.0f);
	
	*q3 = (double)((float)EnCounter3 / KEn2q + 130.0f +20.0f);
}

/**************************************编码器计数****************************************/

void EncoderUpdate(Moter_t *pEncoder, TIM_HandleTypeDef htim)	//编码器计数值更新  存入编码器数组当中	
{
	pEncoder->EnCounter = (long int)((pEncoder->EnOverflowNum)*ENCODER_NUM+__HAL_TIM_GET_COUNTER(&htim));
	pEncoder->EnCoterArr[pEncoder->pArr] = pEncoder->EnCounter;
	pEncoder->pArr +=1;
	pEncoder->pArr %=EncoderStoreNum;
}

unsigned char EncoderFlow(Moter_t* pEncoder, TIM_HandleTypeDef htim)
{
	pEncoder->tEnCounter = __HAL_TIM_GET_COUNTER(&(htim)); 	
	if(pEncoder->tEnCounter<ENCODER_NUM_2)	{
		pEncoder->EnOverflowNum ++;
	}
	else 	{
		pEncoder->EnOverflowNum --;
	}
	pEncoder->EnCounter = (long int)((pEncoder->EnOverflowNum)*ENCODER_NUM+__HAL_TIM_GET_COUNTER(&htim));
	return 1;
}


/**************************************按键中断****************************************/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)	//按键检测优先级为低  （4,1）
{
  /* Prevent unused argument(s) compilation warning */
	UNUSED(GPIO_Pin);
	static int tt = 10000;
	KeyValue = HAL_GPIO_ReadPin(USER_Btn_GPIO_Port,USER_Btn_Pin);
	if(KeyValue)	{
		tt = 10000;
		while(tt)	{	//debug 时间约32ms  按键检测
			tt--;
		}
		if(HAL_GPIO_ReadPin(USER_Btn_GPIO_Port,USER_Btn_Pin)==1)	{	//确认按下
			
			HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);
			while(HAL_GPIO_ReadPin(USER_Btn_GPIO_Port,USER_Btn_Pin))	{	//放开
				EncoderClear();
				KeyValue = HAL_GPIO_ReadPin(USER_Btn_GPIO_Port,USER_Btn_Pin);
				HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_RESET);
			}
		}
	}
}




