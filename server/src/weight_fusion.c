#include <math.h>   
#include "matrix2.h"
#include <stdio.h>
#include <stdlib.h>
#include "types.h"
#include "type.h"
#include "weight_fusion.h"

// 递推最小二乘法
void ditui_ls(Matrix_2* sensor_fusion, Matrix_2* P_fusion, PolarCoord3D* sensor1, PolarCoord3D* sensor2, Matrix_2* H, Matrix_2* P1, Matrix_2* P2) {
    // printf("sensor1->azimuth = %lf\n", sensor1->azimuth);
    // printf("sensor1->elevation = %lf\n", sensor1->elevation);
    // printf("sensor2->azimuth = %lf\n", sensor2->azimuth);
    // printf("sensor2->elevation = %lf\n", sensor2->elevation);
    // 传感器1的量测数据，对角度进行放大处理
    Matrix_2 zz1 = createMatrix(3, 1);
    zz1.data[0] = sensor1->distance;           
    zz1.data[1] = sensor1->azimuth * 1000.0;  
    zz1.data[2] = sensor1->elevation * 1000.0;  

    // 给一个很小的随机值
    Matrix_2 eps1 = createMatrix(3, 1);
    for (int i = 0; i < 3; i++) {
        eps1.data[i] = 0.001 * ((double)rand() / RAND_MAX); // 模拟 rand(3,1)
    }
    
    // 记录预测结果和量测数据的残差
    Matrix_2 resid1 = createMatrix(3, 1);
    resid1.data[0] += 10.0 * ((double)rand() / RAND_MAX) + 100.0;
    resid1.data[1] += 0.2 * ((double)rand() / RAND_MAX) + 2.0;
    resid1.data[2] += 0.2 * ((double)rand() / RAND_MAX) + 2.0;

    /*----------获得权重矩阵----------*/
    Matrix_2 diagW1 = createMatrix(3, 3);
    setMatrixElement(&diagW1, 0, 0, 1.0);
    setMatrixElement(&diagW1, 1, 1, 1.57 * 1.57 * 100.0);
    setMatrixElement(&diagW1, 2, 2, 1.67 * 1.67 * 100.0);
    Matrix_2 W_resid1 = matrixMultiply(&diagW1, &resid1);
    Matrix_2 weights1 = createMatrix(3, 1);
    for (int i = 0; i < 3; i++) {
        double val = W_resid1.data[i];
        weights1.data[i] = 1.0 / (val * val + eps1.data[i]); // (W_resid1)^2 + eps1
    }
    freeMatrix(&W_resid1);
    freeMatrix(&diagW1);

    // Matrix_2 HT = matrixTranspose(H);
    // Matrix_2 H_P = matrixMultiply(H, P1);
    // Matrix_2 HPHT = matrixMultiply(&H_P, &HT);
    Matrix_2 diagWeights1 = createMatrix(3, 3);
    for (int i = 0; i < 3; i++) {
        setMatrixElement(&diagWeights1, i, i, weights1.data[i]);
    }
    *P1 = diagWeights1;
    /*------------------------------*/

    // Matrix_2 S1 = matrixAdd(&HPHT, &diagWeights1);
    // Matrix_2 S1_inv = matrixInverse(&S1);
    // Matrix_2 P_HT = matrixMultiply(P1, &HT);
    // Matrix_2 K1 = matrixMultiply(&P_HT, &S1_inv);
    // Matrix_2 zz1_H_beta = matrixSubtract(&zz1, &H_beta);
    // Matrix_2 K1_zz1_H_beta = matrixMultiply(&K1, &zz1_H_beta);
    // Matrix_2 beta_new1 = matrixAdd(beta_current1, &K1_zz1_H_beta);
    // matrixCopy(beta_current1, &beta_new1);
    // freeMatrix(p_current1);
    // *p_current1 = createMatrix(3, 3);
    // for (int j = 0; j < 3; j++) {
    //     setMatrixElement(p_current1, j, j, weights1.data[j]);
    // }

    // 传感器2的量测数据，对角度进行放大处理
    Matrix_2 zz2 = createMatrix(3, 1);
    zz2.data[0] = sensor2->distance;           
    zz2.data[1] = sensor2->azimuth * 1000.0;  
    zz2.data[2] = sensor2->elevation * 1000.0;  

    // 给一个很小的随机值
    Matrix_2 eps2 = createMatrix(3, 1);
    for (int i = 0; i < 3; i++) {
        eps2.data[i] = 0.001 * ((double)rand() / RAND_MAX); // 模拟 rand(3,1)
    }
    
    // 记录预测结果和量测数据的残差
    Matrix_2 resid2 = createMatrix(3, 1);
    resid2.data[0] += 10.0 * ((double)rand() / RAND_MAX) + 100.0;
    resid2.data[1] += 0.2 * ((double)rand() / RAND_MAX) + 2.0;
    resid2.data[2] += 0.2 * ((double)rand() / RAND_MAX) + 2.0;

    /*----------获得权重矩阵----------*/
    Matrix_2 diagW2 = createMatrix(3, 3);
    setMatrixElement(&diagW2, 0, 0, 1.0);
    setMatrixElement(&diagW2, 1, 1, 1.57 * 1.57 * 100.0);
    setMatrixElement(&diagW2, 2, 2, 1.67 * 1.67 * 100.0);
    Matrix_2 W_resid2 = matrixMultiply(&diagW2, &resid2);
    Matrix_2 weights2 = createMatrix(3, 1);

    for (int i = 0; i < 3; i++) {
        double val = W_resid2.data[i];
        weights2.data[i] = 1.0 / (val * val + eps2.data[i]); // (W_resid2)^2 + eps1
    }
    freeMatrix(&W_resid2);
    freeMatrix(&diagW2);

    // Matrix_2 HT = matrixTranspose(H);
    // Matrix_2 H_P = matrixMultiply(H, P1);
    // Matrix_2 HPHT = matrixMultiply(&H_P, &HT);
    Matrix_2 diagWeights2 = createMatrix(3, 3);
    for (int i = 0; i < 3; i++) {
        setMatrixElement(&diagWeights2, i, i, weights2.data[i]);
    }
    *P2 = diagWeights2;
    /*------------------------------*/

    // Matrix_2 S1 = matrixAdd(&HPHT, &diagWeights1);
    // Matrix_2 S1_inv = matrixInverse(&S1);
    // Matrix_2 P_HT = matrixMultiply(P1, &HT);
    // Matrix_2 K1 = matrixMultiply(&P_HT, &S1_inv);
    // Matrix_2 zz1_H_beta = matrixSubtract(&zz1, &H_beta);
    // Matrix_2 K1_zz1_H_beta = matrixMultiply(&K1, &zz1_H_beta);
    // Matrix_2 beta_new1 = matrixAdd(beta_current1, &K1_zz1_H_beta);
    // matrixCopy(beta_current1, &beta_new1);
    // freeMatrix(p_current1);
    // *p_current1 = createMatrix(3, 3);
    // for (int i = 0; i < 3; i++) {
    //     setMatrixElement(p_current1, i, i, weights1.data[i]);
    // }

    weight_ls(sensor_fusion, P_fusion, &zz1, P1, &zz2, P2);
    sensor_fusion->data[1] /= 1000;
    sensor_fusion->data[2] /= 1000;
}

// 加权融合
void weight_ls(Matrix_2* c, Matrix_2* C, 
               Matrix_2* stateA, Matrix_2* covMatrixA, 
               Matrix_2* stateB, Matrix_2* covMatrixB) {

    int n = stateA->rows; 

    // Compute weight_A = invCA / (invCA + invCB)
    // Compute inverse covariances
    Matrix_2 invCA = matrixInverse(covMatrixA);
    Matrix_2 invCB = matrixInverse(covMatrixB);

    Matrix_2 inv_CA_add_CB = matrixAdd(&invCA, &invCB);
    Matrix_2 CA_add_CB = matrixInverse(&inv_CA_add_CB);
    Matrix_2 weight_A = matrixMultiply(&invCA, &CA_add_CB);
    
    // Compute weight_B = I - weight_A
    Matrix_2 I_matrix = createIdentityMatrix(weight_A.cols);
    Matrix_2 weight_B = matrixSubtract(&I_matrix, &weight_A);

    // state fusion: c = weight_A * stateA + weight_B * stateB
    Matrix_2 wA_stateA = matrixMultiply(&weight_A, stateA);
    Matrix_2 wB_stateB = matrixMultiply(&weight_B, stateB);
    Matrix_2 result = matrixAdd(&wA_stateA, &wB_stateB);

    *c = matrixAdd(&wA_stateA, &wB_stateB);

    // Free temporary matrices
    freeMatrix(&invCA);
    freeMatrix(&invCB);
    freeMatrix(&inv_CA_add_CB);
    freeMatrix(&CA_add_CB);
    freeMatrix(&weight_A);
    freeMatrix(&weight_B);
    freeMatrix(&wA_stateA);
    freeMatrix(&wB_stateB);
}

// 笛卡尔坐标转成球坐标
PolarCoord3D cartesianToPolar(Vector3D cartesian) {
    PolarCoord3D polar;
    double x = cartesian.x;
    double y = cartesian.y;
    double z = cartesian.z;

    // printf("x = %f, y = %f, z = %f\n", x, y, z);

    // 计算距离
    polar.distance = sqrt(x * x + y * y + z * z); 

    // 初始化输出
    polar.azimuth = 0.0;
    polar.elevation = 0.0;

    // 处理距离接近 0 的情况
    if (polar.distance < 1e-10) {
        printf("Warning: Distance is too small, returning zero polar coordinates\n");
        return polar;
    }

    // 计算方位角（azimuth）
    polar.azimuth = atan2(x, y);

    // 计算俯仰角（elevation）
    polar.elevation = atan2(z, sqrt(x*x + y*y));

    return polar;
}

// 球坐标转成笛卡尔坐标
Vector3D polarToCartesian(PolarCoord3D polar) {
    Vector3D cartesian;
    // 将角度转换为弧度
    double azimuthRad = polar.azimuth * M_PI / 180.0;
    double elevationRad = polar.elevation * M_PI / 180.0;

    // double azimuthRad = polar.azimuth;
    // double elevationRad = polar.elevation;
    // 计算笛卡尔坐标
    cartesian.x = polar.distance * cos(elevationRad) * cos(azimuthRad);
    cartesian.y = polar.distance * cos(elevationRad) * sin(azimuthRad);
    cartesian.z = polar.distance * sin(elevationRad);
    
    return cartesian;
}