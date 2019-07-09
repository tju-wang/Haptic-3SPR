#ifndef __motor_H
#define __motor_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "include.h"


void MotorPwmMid(char ch);
void Motor1PWM(int pwm);
void Motor2PWM(int pwm);
void Motor3PWM(int pwm);
void SetMotorPWM(void);	//…Ë÷√◊‹PWM


#endif

