#ifndef WEIGHT_FUSION_H
#define WEIGHT_FUSION_H

# include "types.h"

// 笛卡尔坐标转成球坐标
PolarCoord3D cartesianToPolar(Vector3D cartesian);

// 球坐标转成笛卡尔坐标
Vector3D polarToCartesian(PolarCoord3D polar);

// 递推最小二乘估计
void ditui_ls(Matrix_2* sensor_fusion, Matrix_2* P_fusion, PolarCoord3D* sensor1, PolarCoord3D* sensor2, Matrix_2* H, Matrix_2* P1, Matrix_2* P2);

// 加权融合
void weight_ls(Matrix_2* c, Matrix_2* C, 
               Matrix_2* stateA, Matrix_2* covMatrixA, 
               Matrix_2* stateB, Matrix_2* covMatrixB);

#endif
