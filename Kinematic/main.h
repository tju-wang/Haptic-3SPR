#ifndef _MAIN_H_
#define _MAIN_H

#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <windows.h>
#include "kinematic.h"
#include "solveCoord.h"
#include "solveFbi.h"
#include "time.h"
#include "uart.h"

// extern BOOL SerialInit(HANDLE *hC);
// extern BOOL Update_q(unsigned char *data,var_q *Lq);
// extern void Double2Char(double doub,unsigned char* val_1,unsigned char* val_2,unsigned char* val_3);
// extern unsigned char dataCheck(unsigned char* buff,char len);
// extern BOOL SendData(double F1,double F2,double F3,unsigned int sort,unsigned char* wData);


void TestKinematic(void);
// void CalcForce_ave(double *data,int len,double *ave);
// CalcDiffMax(double *data,int sort,double *diffmax,double *lastMax);


#endif

