#ifndef _SOLVECOORD_H_
#define _SOLVECOORD_H_
#include "stdio.h"
#include "math.h"
#include "kinematic.h"



typedef struct _point{
    double x,y,z;
}point_t;
typedef struct _vector
{
    double x,y,z,norm;
}vector_t;



char CheckPoint(point_t P,var_t var,char ChPoint,vector_t *vACC);
double norm(vector_t *Vec);
double VecCross(vector_t *pVec1,vector_t *pVec2);
void CPointSolve(var_t var);

#endif

