#include "main.h"

#define RxDataNum  12
#define TxDataNum	15

extern var_t var;
extern var_q varq;
extern const double pi;
extern force_t F1,F2,F3;
extern const double s;	//偏距等效


int main(void)
{

	double ForceDataF1[10],ForceDataF2[10],ForceDataF3[10];
	double ForceMaxF1,ForceMaxF2,ForceMaxF3;
	double lastForceMaxF1,lastForceMaxF2,lastForceMaxF3;
	int ForceNum;
	int i = 0;
	BOOL Status;
	char sort;
	clock_t start,finish;
	char max_time;
	start = 0;  finish = 0;
	
	HANDLE hComm;
	var = VarDataInit(var);
	unsigned char *wData = malloc(sizeof (unsigned char)*TxDataNum);	//去掉首尾
    unsigned char *rData = malloc(sizeof (unsigned char)*RxDataNum);
	unsigned int FalseNum = 0;

	// TestKinematic();
	Status = SerialInit(&hComm);
	if(Status!=TRUE)		{
		printf("Open Serial Error!");
		exit(1);
	}
	
	while(1)	
	{
		Status = TRUE;	//继续循环
		if(Status==TRUE)	{
			Status = ReadSerialPort(hComm,rData);
			Status = Update_q(rData,&varq,&sort);
			start = clock();
			// printf(" %6.3f  %6.3f  %6.3f \n",varq.q1,varq.q2,varq.q3);
		}
		if(Status==!TRUE) {
			printf("Receive data error!");
			FalseNum++;  
		}
		//计算
		if(Status==TRUE)	{
			//input data check
			ForwardKinematic();
			CPointSolve(var);
			FbSolve();
		// printf("q1 = %6.3f  q2 = %6.3f  q3 = %6.3f \n",varq.q1,varq.q2,varq.q3);
		// printf("X0 = %6.3f  Y0 = %6.3f  Z0 = %6.3f \n",var.X0,var.Y0,var.Z0);
		printf("F1 = %6.3f  F2 = %6.3f  F3 = %6.3f \n",F1.Fx,F2.Fx,F3.Fx);
		//计算F变化的范围  
		// ForceDataF1[ForceNum%10] = F1.Fx;
		// ForceDataF2[ForceNum%10] = F2.Fx;
		// ForceDataF3[ForceNum%10] = F3.Fx;

		// CalcDiffMax(ForceDataF1,ForceNum,ForceMaxF1,lastForceMaxF1);
		// CalcDiffMax(ForceDataF2,ForceNum,ForceMaxF2,lastForceMaxF1);
		// CalcDiffMax(ForceDataF3,ForceNum,ForceMaxF3,lastForceMaxF1);
		// if(ForceNum==1000)
		// {
		// 	ForceNum = 0;
		// 	printf("F_Max = %6.3f  %6.3f  %6.3f",lastForceMaxF1,lastForceMaxF2,lastForceMaxF3);
		// }

		//返回数据
			//Send Data to MCU 
			Status = SendData(hComm,F1.Fx,F2.Fx,F3.Fx,sort,wData);
			finish = clock();
			// printf("Clc time is %d ms\n",max_time);
			if((finish-start)>max_time)
				max_time = finish - start;
			FalseNum = 0;
		}
		if(Status==!TRUE)	{
			FalseNum++;          
		}
		if(FalseNum>500)	{
			printf("FALSENum > 500, exit!");   exit(1);
		}
	}

}

void TestKinematic(void)
{
	varq.q1 = 281;  varq.q2 = 291; varq.q3 = 291;
	ForwardKinematic();
	CPointSolve(var);
	FbSolve();
	PrintVarStruct(var,varq);
	printf("F1 = %6.3f  F2 = %6.3f  F3 = %6.3f \n",F1.Fx,F2.Fx,F3.Fx);
	while(1);

}

// void CalcForce_ave(double *data,int len,double *ave)
// {
// 	int numm = 0;
// 	double sum = 0;
// 	for(numm=0;numm<len;numm++)
// 	{
// 		sum += *(data++);
// 	}

// 	*ave = sum/len;
// }
// CalcDiffMax(double *data,int sort,double *diffmax,double *lastMax)
// {
// 	*diffmax = fabs(*(data+sort) - *(data+sort-1));
// 	if(*lastMax<*diffmax)
// 		*lastMax = *diffmax;
// }


