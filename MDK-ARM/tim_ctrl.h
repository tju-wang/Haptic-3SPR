#ifndef __tim_ctrl_H
#define __tim_ctrl_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "include.h"
#include "kinematic.h"


#define EncoderStoreNum		20		//�����ٶ�ֵ  ��Ҫ�ο���λ��ֵ������
//#define MSpArrNum			20		//���������ٶ�ֵ ��Ҫ�ο��ĵ���ٶ�ֵ������
#define StateRef			30	//ת����ֹ  �ο�����ֵ
//#define	PWMMID				500
//#define PWMPulse			70	

//#define	PULL_PWM			15  //15-48
#define PUSH_PWM			(-PULL_PWM)

//#define SigPWMPulse			54	//��⵽ת����  �����Ħ��������

//#define StopPwm				30	//��⵽��ֹʱ  �������Ħ��������

typedef struct Moter{
	signed char EnOverflowNum;	//����������Ĵ���
	signed long int EnCounter;	//����������ֵ
	int tEnCounter;
	
	char pArr;	//��ǰ�����ݵĴ洢λ��
	signed long int EnCoterArr[EncoderStoreNum];	//�����ٶ���
	signed char runstate;	//1 ��ת -1 ��ת 0 ��ֹ  �Ա�������ֵ���ķ���Ϊ��
	int speed;		//����ٶ�
	signed int MoterSpeedArr[EncoderStoreNum];
	char pSpArr;
	int accelration;	//������ٶ�
	
}Moter_t;

typedef struct Ctrl{	//���Ʊ����ṹ��
	char CtrlMode;	//����ģʽѡ��  ����������Ħ����������������  ȫ������
	int t;
	
	int PwmSum;	//�����ص��Ӻ�  �������pwm��
	
	int Gravity;		
	int StaticFric;		//��Ħ��
	int DynFric;	//��Ħ��  	Dynamic Friction
	int InertiaForce;	//������
	
	//��Ӱ�����ص�Ӱ�쿪�ر�־
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

void ModeSigMotor(void);	//���������
void CtrlPwmSum(Ctrl_t *pMotor);	//�������ص���
void FricCalcu(Ctrl_t *pCtrl,Moter_t *pMotor);	//Ħ��������
void InertiaCalcu(Ctrl_t *pCtrl,Moter_t *pMotor);
void GriverCalc(Ctrl_t *pCtrl,Moter_t *pMotor);

float PullPushSpeed(unsigned int sp);	//������������  ���ٶȱ仯��
void EnCounter2q(signed long int EnCounter1,signed long int EnCounter2,signed long int EnCounter3,double *q1,double *q2,double *q3);
void SendqData(var_q temp,unsigned char* psort);
void Float2Char(float fData,unsigned char *cDataH,unsigned char *cDataL);



#endif

