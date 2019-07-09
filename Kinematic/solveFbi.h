#ifndef _SOLVEFBI_H_
#define _SOLVEFBI_H_
#include "stdio.h"
#include "math.h"
#include "kinematic.h"
#include "solveCoord.h"
#include "my_matrixadvlib.h"



typedef struct _Force{
    double Fx,Fy,Fz;
    double F;
}force_t;

typedef struct _Torque
{
    double Tx,Ty,Tz;
    double torque;
}torque_t;


void FbSolve(void);
void GraviInit(void);
torque_t ClcTorque(point_t libi,force_t force);
point_t PpointSolve(vector_t vAC,point_t C,double length);
point_t DpointSolve(point_t P1,point_t P2);
torque_t TorqurStruSum(torque_t torq1,torque_t torq2,torque_t torq3);
torque_t TorqurStruSum2(torque_t torq1,torque_t torq2);
void PrintStrucForce(force_t Force);
void PointStruPrint(point_t Point);
void VectorStruPrint(vector_t vec);
void TorqStruPrint(torque_t Torq);
vector_t Cross(vector_t Vec1,vector_t Vec2);
force_t SolveMotorForce(vector_t vAC,vector_t vN,force_t Force);
vector_t VecUint(vector_t Vec);
vector_t MakeVector(point_t Start,point_t End);
point_t subPoint(point_t P1,point_t P2);



#endif

