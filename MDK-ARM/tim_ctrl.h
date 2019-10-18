#ifndef __tim_ctrl_H
#define __tim_ctrl_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "include.h"
#include "kinematic.h"


#define EncoderStoreNum		20		//计算速度值  需要参考的位置值的数量
//#define MSpArrNum			20		//计算电机加速度值 需要参考的电机速度值的数据
#define StateRef			30	//转动或静止  参考计数值
//#define	PWMMID				500
//#define PWMPulse			70	

//#define	PULL_PWM			15  //15-48
#define PUSH_PWM			(-PULL_PWM)

//#define SigPWMPulse			54	//检测到转动后  单电机摩擦力补偿

//#define StopPwm				30	//检测到静止时  正反向的摩擦力波动

typedef struct Moter{
	signed char EnOverflowNum;	//编码器溢出的次数
	signed long int EnCounter;	//编码器计数值
	int tEnCounter;
	
	char pArr;	//当前新数据的存储位置
	signed long int EnCoterArr[EncoderStoreNum];	//计算速度用
	signed char runstate;	//1 正转 -1 反转 0 静止  以编码器读值变大的方向为正
	int speed;		//电机速度
	signed int MoterSpeedArr[EncoderStoreNum];
	char pSpArr;
	int accelration;	//电机加速度
	
}Moter_t;

typedef struct Ctrl{	//控制变量结构体
	char CtrlMode;	//控制模式选择  重力补偿、摩擦力、惯性力补偿  全补偿等
	int t;
	
	int PwmSum;	//各因素叠加后  计算出的pwm和
	
	int Gravity;		
	int StaticFric;		//静摩擦
	int DynFric;	//动摩擦  	Dynamic Friction
	int InertiaForce;	//惯性力
	
	//各影响因素的影响开关标志
	char GraFlag;
	char StaFriFlag;
	char DynFriFlag;
	char InerFlag;
	
	float pullpush;
	int sensorData;
	
}Ctrl_t;
typedef struct MotorGriverForce{
	float F1,F2,F3;
}MotorGriverForce_t;


unsigned char EncoderFlow(Moter_t* pEncoder, TIM_HandleTypeDef htim);
void EncoderUpdate(Moter_t *pEncoder, TIM_HandleTypeDef htim);
unsigned char MotorState(Moter_t *pState);

void ChooseMode(unsigned char mode);

void ModeSigMotor(void);	//单电机补偿
void CtrlPwmSum(Ctrl_t *pMotor);	//将各因素叠加
void FricCalcu(Ctrl_t *pCtrl,Moter_t *pMotor);	//摩擦力计算
void InertiaCalcu(Ctrl_t *pCtrl,Moter_t *pMotor);
void GriverCalc(Ctrl_t *pCtrl,Moter_t *pMotor);

float PullPushSpeed(unsigned int sp);	//推拉主动补偿  随速度变化量
void EnCounter2q(signed long int EnCounter1,signed long int EnCounter2,signed long int EnCounter3,double *q1,double *q2,double *q3);
void SendqData(var_q temp,unsigned char* psort);
void Float2Char(float fData,unsigned char *cDataH,unsigned char *cDataL);



#endif

