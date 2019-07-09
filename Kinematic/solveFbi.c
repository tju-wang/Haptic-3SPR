/*
求解C点处的受力
Fb1  Fb2  Fb3
1.定义各个点的等效重力大小  常数
2.定义B点坐标 常数
3.列力的平衡方程

*/
#include "solveFbi.h"

extern var_t var;
extern var_q varq;
extern const double a,b,s,Q;
extern vector_t vA1C1,vA2C2,vA3C3;
extern point_t C1,C2,C3;

// const point_t B1 = {0,-80.0,20},B2 = {69.282,40,20},B3 = {-69.282,40,20}; 
force_t FG_1,FG_2,FG_3,FGm;
force_t FB1,FB2,FB3,FG;
point_t P1,P2,P3;   //配重块重心
point_t D1,D2,D3,Gm;   //P副重心  Gm为操作球和动平台的质心坐标
point_t A1,A2,A3;
torque_t TG1_1,TG2_1,TG3_1,TG1_2,TG2_2,TG3_2,TG1_3,TG2_3,TG3_3,TGm;
torque_t TG,TG_1,TG_2,TG_3;
vector_t vA1A2,vA1A3,vA2A3;
force_t temp;
force_t F1,F2,F3;

void GraviInit(void)
{
    double alpha,beta,gama,X0,Y0,Z0;
    alpha = var.alpha;
    beta = var.beta;
    gama = var.gama;
    X0 = var.X0;
    Y0 = var.Y0;
    Z0 = var.Z0;
//TG1_1 扭矩-第一条支链 配重块   _2 导轨    _3 电机 滚子    Gm  末端
    FG_1.Fx = FG_1.Fz = 0;  FG_1.Fy = (438)*10;
    FG_2.Fx = FG_2.Fz = 0;  FG_2.Fy = (329.75)*10;
    FG_3.Fx = FG_3.Fz = 0;  FG_3.Fy = 342.5*10;
    FGm.Fx = FGm.Fz = 0;    FGm.Fy = (86+50+300+30)*10;
    A1.x = X0 - a*arm_sin_f32(beta)*arm_sin_f32(gama);
    A1.y = Y0 - a*(arm_cos_f32(alpha)*arm_cos_f32(gama) - arm_cos_f32(beta)*arm_sin_f32(alpha)*arm_sin_f32(gama));
    A1.z = Z0 - a*(arm_cos_f32(gama)*arm_sin_f32(alpha) + arm_cos_f32(alpha)*arm_cos_f32(beta)*arm_sin_f32(gama));

    A2.x =  X0 + (a*arm_sin_f32(beta)*arm_sin_f32(gama))/2 + (sqrt(3)*a*arm_cos_f32(beta))/2;
    A2.y = Y0 + (a*(arm_cos_f32(alpha)*arm_cos_f32(gama) - arm_cos_f32(beta)*arm_sin_f32(alpha)*arm_sin_f32(gama)))/2 + (sqrt(3)*a*arm_sin_f32(alpha)*arm_sin_f32(beta))/2;
    A2.z = Z0 + (a*(arm_cos_f32(gama)*arm_sin_f32(alpha) + arm_cos_f32(alpha)*arm_cos_f32(beta)*arm_sin_f32(gama)))/2 - (sqrt(3)*a*arm_cos_f32(alpha)*arm_sin_f32(beta))/2;

    A3.x = X0 + (a*arm_sin_f32(beta)*arm_sin_f32(gama))/2 - (sqrt(3)*a*arm_cos_f32(beta))/2;
    A3.y = Y0 + (a*(arm_cos_f32(alpha)*arm_cos_f32(gama) - arm_cos_f32(beta)*arm_sin_f32(alpha)*arm_sin_f32(gama)))/2 - (sqrt(3)*a*arm_sin_f32(alpha)*arm_sin_f32(beta))/2;
    A3.z = Z0 + (a*(arm_cos_f32(gama)*arm_sin_f32(alpha) + arm_cos_f32(alpha)*arm_cos_f32(beta)*arm_sin_f32(gama)))/2 + (sqrt(3)*a*arm_cos_f32(alpha)*arm_sin_f32(beta))/2;
    
}


void FbSolve(void)
{
    GraviInit();
    //计算
    point_t B0;
    B0.x = 0;  B0.y = 0;  B0.z = 20;
    //  printf("vAC x = %6.3f  y = %6.3f  z = %6.3f norm = %6.3f \n",vA1C1.x,vA1C1.y,vA1C1.z,vA1C1.norm);
    P1 = PpointSolve(vA1C1,C1,Q-sqrt(pow(varq.q1,2)-pow(s,2)));
    P2 = PpointSolve(vA2C2,C2,Q-sqrt(pow(varq.q2,2)-pow(s,2)));
    P3 = PpointSolve(vA3C3,C3,Q-sqrt(pow(varq.q3,2)-pow(s,2)));
    VectorStruPrint(vA3C3);

    D1 = DpointSolve(P1,A1);
    D2 = DpointSolve(P2,A2);
    D3 = DpointSolve(P3,A3);

    Gm.x = var.X0;
    Gm.y = var.Y0;
    Gm.z = var.Z0;
    // PointStruPrint(Gm);
    // PointStruPrint(D2);
    // PointStruPrint(D3);

    //求解出了各等效质点坐标  后面开始  先将程序移植到单片机上
    //求解扭矩
    TG1_1 = ClcTorque(subPoint(P1,B0),FG_1);
    TG2_1 = ClcTorque(subPoint(P2,B0),FG_1);
    TG3_1 = ClcTorque(subPoint(P3,B0),FG_1);

    TG1_2 = ClcTorque(subPoint(D1,B0),FG_2);
    TG2_2 = ClcTorque(subPoint(D2,B0),FG_2);
    TG3_2 = ClcTorque(subPoint(D3,B0),FG_2);
    // PrintStrucForce(FG_2);

    TG1_3 = ClcTorque(subPoint(C1,B0),FG_3);
    TG2_3 = ClcTorque(subPoint(C2,B0),FG_3);
    TG3_3 = ClcTorque(subPoint(C3,B0),FG_3);
    TGm = ClcTorque(subPoint(Gm,B0),FGm);

    TG_1 = TorqurStruSum(TG1_1,TG2_1,TG3_1);
    TG_2 = TorqurStruSum(TG1_2,TG2_2,TG3_2);
    TG_3 = TorqurStruSum(TG1_3,TG2_3,TG3_3);


    TG = TorqurStruSum(TG_1,TG_2,TG_3);
    TG = TorqurStruSum2(TG,TGm);
    
    
    FG.Fy = FG_1.Fy*3 + FG_2.Fy*3 +FG_3.Fy*3+FGm.Fy;

    //求解对x轴的扭矩  求得B1 2 3 点Fz的大小
    FB2.Fz = -TG.Tx/(3*b);
    FB3.Fz = FB2.Fz;
    FB1.Fz = -2*FB2.Fz;
    //求解Fx  绕z轴的转矩平衡方程
    FB2.Fx = -TG.Tz/3/b;
    FB1.Fx = -2*FB2.Fx;
    FB3.Fx = FB2.Fx;
    //求解Fy  列重力方程
    FB1.Fy = -(FG.Fy/3);
    FB2.Fy = FB3.Fy = FB1.Fy;

#ifdef DebugPrint
    PrintStrucForce(FB1);
    PrintStrucForce(FB2);
    PrintStrucForce(FB3);
#endif

    vA1A2 = MakeVector(A2,A1);
    vA1A3 = MakeVector(A3,A1);
    vA2A3 = MakeVector(A3,A2);

 //求得的MotorForce  只有x坐标保证了一致的方向  y  z力的方向由单位矢量决定 不关注
    temp.Fx = FB1.Fx + FG_3.Fx;
    temp.Fy = FB1.Fy + FG_3.Fy;
    temp.Fz = FB1.Fz + FG_3.Fz;

    F1 = SolveMotorForce(vA1C1,vA2A3,temp);

    temp.Fx = FB2.Fx + FG_3.Fx;
    temp.Fy = FB2.Fy + FG_3.Fy;
    temp.Fz = FB2.Fz + FG_3.Fz;
    F2 = SolveMotorForce(vA2C2,vA1A3,temp);

    temp.Fx = FB3.Fx + FG_3.Fx; 
    temp.Fy = FB3.Fy + FG_3.Fy;
    temp.Fz = FB3.Fz + FG_3.Fz;
    F3 = SolveMotorForce(vA3C3,vA1A2,temp);
}



torque_t ClcTorque(point_t libi,force_t force)
{
    torque_t ret;
    ret.Tx = libi.y*force.Fz-libi.z*force.Fy;
    ret.Ty = -(libi.x*force.Fz-libi.z*force.Fx);
    ret.Tz = libi.x*force.Fy-libi.y*force.Fx;

    return ret;
}
//P点坐标求解  P点是移动副配重块的质心
//输出 P 点坐标  输入 vAC  C点坐标  Q-q(P副末端长度)
point_t PpointSolve(vector_t vAC,point_t C,double length) 
{
    point_t ret;
    vAC.norm = sqrt(pow(vAC.x,2)+pow(vAC.y,2)+pow(vAC.z,2));
    vAC.x /= vAC.norm;
    vAC.y /= vAC.norm;
    vAC.z /= vAC.norm;
    // printf("vAC x = %6.3f  y = 6.3f  z = 6.3f norm = 6.3f \n",vAC.x,vAC.y,vAC.z,vAC.norm);
    if(vAC.z>0)
    {
        vAC.x = -vAC.x;
        vAC.y = -vAC.y;
        vAC.z = -vAC.z;
    }
    ret.x = C.x+length*vAC.x;
    ret.y = C.y+length*vAC.y;
    ret.z = C.z+length*vAC.z;
    return ret;
}
point_t DpointSolve(point_t P1,point_t P2)
{
    point_t ret;
    ret.x = (P1.x + P2.x)/2;
    ret.y = (P1.y + P2.y)/2;
    ret.z = (P1.z + P2.z)/2;

    return ret;
}

//结构体求和函数
torque_t TorqurStruSum(torque_t torq1,torque_t torq2,torque_t torq3)
{
    torque_t ret;

    ret.Tx = torq1.Tx + torq2.Tx + torq3.Tx;
    ret.Ty = torq1.Ty + torq2.Ty + torq3.Ty;
    ret.Tz = torq1.Tz + torq2.Tz + torq3.Tz;

    return ret;
}

torque_t TorqurStruSum2(torque_t torq1,torque_t torq2)
{
    torque_t ret;

    ret.Tx = torq1.Tx + torq2.Tx;
    ret.Ty = torq1.Ty + torq2.Ty;
    ret.Tz = torq1.Tz + torq2.Tz;

    return ret;
}

void PrintStrucForce(force_t Force)
{
    #ifdef DebugPrint
        printf("Force.Fx = %6.3f    Force.Fy = %6.3f   Force.Fz = %6.3f  Force.norm = %6.3f \n"\
        ,Force.Fx,Force.Fy,Force.Fz,sqrt(pow(Force.Fx,2)+pow(Force.Fy,2)+pow(Force.Fz,2)));
    #endif
}

void PointStruPrint(point_t Point)
{
    #ifdef DebugPrint
        printf("P Point.x = %6.3f  Point.y = %6.3f  Point.z = %6.3f \n"\
        ,Point.x,Point.y,Point.z);
    #endif
}

void VectorStruPrint(vector_t vec)
{
    #ifdef DebugPrint
        printf("vec.x = %6.3f  vec.y = %6.3f  vec.z = %6.3f  vec.norm = %6.3f\n"\
        ,vec.x,vec.y,vec.z,vec.norm);
    #endif
}
void TorqStruPrint(torque_t Torq)
{
     #ifdef DebugPrint
        printf("Torq.Tx = %6.3f  Torq.Ty = %6.3f  Torq.Tz = %6.3f  \n"\
        ,Torq.Tx,Torq.Ty,Torq.Tz);
    #endif
}

//求解电机应该补偿的力  Matrixadvlib中 有用LU分解解方程的函数
force_t SolveMotorForce(vector_t vAC,vector_t vN,force_t Force)
{
    vector_t n1,n2,n3,Vec;  //n1 为AC方向单位向量  n2为R副方向单位向量  n3为垂直于两者方向单位向量
    force_t ret;
//    unsigned int i,j;
   
    n1 = VecUint(vAC);
    if(n1.z>0)  {
        n1.x = -n1.x;  n1.y = -n1.y;  n1.z = -n1.z;
    }
    n2 = VecUint(vN);
    n3 = Cross(vAC,vN);     
    n3 = VecUint(n3);
    Matrix_t_3x3 A;
    Matrix_t_3x1 bN,X;
    A.height = A.width = 3;
    X.height = 3;
    X.width = 1;
    bN.height = 3;
    bN.width = 1;
    // A = makeMatrix(3,3);
    // X = makeMatrix(1,3);
    // bN = makeMatrix(1,3);

    // VectorStruPrint(n1);
    // VectorStruPrint(vAC);
    // VectorStruPrint(n3);

    //矩阵赋值  数据指针  横行填充矩阵
    A.data[0*3+0] = n1.x;
    A.data[0*3+1] = n2.x;
    A.data[0*3+2] = n3.x;

    A.data[1*3+0] = n1.y;
    A.data[1*3+1] = n2.y;
    A.data[1*3+2] = n3.y;

    A.data[2*3+0] = n1.z;
    A.data[2*3+1] = n2.z;
    A.data[2*3+2] = n3.z;

    bN.data[0] = -Force.Fx;
    bN.data[1] = -Force.Fy;
    bN.data[2] = -Force.Fz;

    // for(i=0;i<9;i++)
    // {
    //     printf("A.data[%d] = %8.5f  \n",i,A.data[i]);
    // }
    // for(i = 0;i < 3;i++)
    // {
    //     printf("bN.data[%d] = %6.3f  \n",i,bN.data[i]);
    // }
    
    //定方向  
    X = m_solver_3x3(A,bN);
    #ifdef DebugPrint
        printf("Fbx = %6.3f  Fby = %6.3f  Fbz = %6.3f \n",X.data[0],X.data[1],X.data[2]);
    #endif
    ret.Fx = X.data[0];
    ret.Fy = X.data[1];
    ret.Fz = X.data[2];
    
    return ret;
}

vector_t Cross(vector_t Vec1,vector_t Vec2)
{
    vector_t ret;
    ret.x = Vec1.y*Vec2.z-Vec1.z*Vec2.y;
    ret.y = -(Vec1.x*Vec2.z-Vec1.z*Vec2.x);
    ret.z = Vec1.x*Vec2.y-Vec1.y*Vec2.x;

    return ret;
}

vector_t VecUint(vector_t Vec)
{
    vector_t ret;
    Vec.norm = sqrt(pow(Vec.x,2)+pow(Vec.y,2)+pow(Vec.z,2));
    ret.x = Vec.x/Vec.norm;
    ret.y = Vec.y/Vec.norm;
    ret.z = Vec.z/Vec.norm;
    return ret;
}
vector_t MakeVector(point_t Start,point_t End)
{
    vector_t ret;
    ret.x = End.x - Start.x;
    ret.y = End.y - Start.y;
    ret.z = End.z - Start.z;

    return ret;
}
point_t subPoint(point_t P1,point_t P2)
{
    point_t ret;
    ret.x = P1.x - P2.x;
    ret.y = P1.y - P2.y;
    ret.z = P1.z - P2.z;

    return ret;
}

