#include "motor.h"

extern Ctrl_t	Ctrl,CtrlM1,CtrlM2,CtrlM3;

//将电机状态设置为不输出扭矩
void MotorPwmMid(char ch)
{
	if(ch==1)	{
		Motor1PWM(PWMMID);
	}
	else if(ch==2)	{
		Motor2PWM(PWMMID);
	}
	else if(ch==3)	{
		Motor3PWM(PWMMID);
	}
	else {
		Motor1PWM(PWMMID);
		Motor2PWM(PWMMID);
		Motor3PWM(PWMMID);
	}
}


void Motor1PWM(int pwm)
{
	if(pwm<PWM_MIN)
		pwm = PWM_MIN;
	else if(pwm>PWM_MAX)
		pwm = PWM_MAX;

	TIM1->CCR1 = pwm;
}

void Motor2PWM(int pwm)
{
	if(pwm<PWM_MIN)
		pwm = PWM_MIN;
	else if(pwm>PWM_MAX)
		pwm = PWM_MAX;

	TIM1->CCR2 = pwm;
}
void Motor3PWM(int pwm)
{
	if(pwm<PWM_MIN)
		pwm = PWM_MIN;
	else if(pwm>PWM_MAX)
		pwm = PWM_MAX;

	TIM1->CCR3 = pwm;
}


void SetMotorPWM(void)	//设置总PWM
{	
	Motor1PWM(CtrlM1.PwmSum);
	Motor2PWM(CtrlM2.PwmSum);
	Motor3PWM(CtrlM3.PwmSum);
	
}

