#ifndef MATRIX2_H
#define MATRIX2_H

#include "types.h"

/* ========== 基本矩阵操作函数 ========== */
// 创建矩阵
Matrix_2 createMatrix(int rows, int cols);

// 创建并返回单位矩阵
Matrix_2 createIdentityMatrix(int size);

// 释放矩阵内存
void freeMatrix(Matrix_2* mat);

// 获取矩阵元素 M(i,j)
double getMatrixElement(Matrix_2* mat, int i, int j);

// 矩阵复制 dst = src
void matrixCopy(Matrix_2 *dst, Matrix_2 *src);

// 设置矩阵元素 M(i,j) = value
void setMatrixElement(Matrix_2* mat, int i, int j, double value);


/* ========== 矩阵代数运算 ========== */
// 矩阵加法: C = A + B
Matrix_2 matrixAdd(Matrix_2* A, Matrix_2* B);

// 矩阵减法: C = A - B
Matrix_2 matrixSubtract(Matrix_2* A, Matrix_2* B);

// 矩阵乘法: C = A * B
Matrix_2 matrixMultiply(Matrix_2* A, Matrix_2* B);

// 矩阵标量乘法：B = alpha * A
Matrix_2 matrixScalarMultiply(double alpha, Matrix_2 *A);

// 矩阵转置
Matrix_2 matrixTranspose(Matrix_2* A);

double matrixTrace(Matrix_2 *A); // 计算矩阵的迹（对角线元素之和）



/* ========== 矩阵高级操作 ========== */
// 计算小矩阵行列式（2x2或3x3）
double matrixDeterminant(Matrix_2 *A);

// 提取矩阵子矩阵（用于余子式计算）
Matrix_2 getSubMatrix(Matrix_2 *A, int excludeRow, int excludeCol);

// 计算余子式
double cofactor(Matrix_2 *A, int i, int j);

// 计算伴随矩阵
Matrix_2 adjointMatrix(Matrix_2 *A);

// 矩阵求逆：B = A^(-1)，使用伴随矩阵法
Matrix_2 matrixInverse(Matrix_2* A);


/* ========== 特殊矩阵操作（卡尔曼滤波相关）========== */
// 计算两个向量的外积：C = a * b^T（卡尔曼滤波协方差更新中常用）
Matrix_2 outerProduct(Matrix_2 *a, Matrix_2 *b);

// 计算马氏距离: sqrt((x-m)^T * S^(-1) * (x-m))
// 用于量测验证与数据关联
double mahalanobisDistance(Matrix_2 *x, Matrix_2 *m, Matrix_2 *S);

/* ========== 针对卡尔曼滤波器的优化函数 ========== */
// 卡尔曼滤波器协方差更新: P = F*P*F^T + Q
// 优化版本，避免中间矩阵存储
void updateCovariance(Matrix_2 *P, Matrix_2 *F, Matrix_2 *Q);

// 计算卡尔曼增益: K = P*H^T * (H*P*H^T + R)^(-1)
Matrix_2 calculateKalmanGain(Matrix_2 *P, Matrix_2 *H, Matrix_2 *R);

/* ========== 辅助函数 ========== */
// 打印矩阵（调试用）
void printMatrix(Matrix_2* mat);

void printMatrixICI(Matrix_2 *mat, const char *name); // 辅助函数：打印矩阵


#endif /* MATRIX_H */
