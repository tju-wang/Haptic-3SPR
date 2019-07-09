#ifndef _KINEMATIC_H
#define _KINEMATIC_H

#include "stdio.h"
#include "math.h"
#include "arm_math.h"
#include "matrixlib.h"
 #include "matrixadvlib.h"
//  #include "my_matrixadvlib.h"

 //控制是否打印输出的宏
// #define DebugPrint



/*定义机械结构常数*/
// #define		s	((double)(62.0f))	//机构偏距 
// #define		a	((double)(41.56f))	//动平台外接圆半径
// #define		b	((double)(80.0f)) 	//静平台


typedef struct _var {
    double alpha,beta,gama,X0,Y0,Z0;
} var_t;

typedef struct _varq {
    double q1,q2,q3;
} var_q;



var_t VarDataInit(var_t var);
void Jacobian(var_t var);
// (matrix*) Jacobian(var_t var);
double Relation(var_t var,var_t *diff_var,var_q qvar);
//matrix* Jacobian(var_t var);
void ForwardKinematic(void); 
signed char sign(double k);
void PrintVarStruct(var_t var,var_q varq);

#endif


