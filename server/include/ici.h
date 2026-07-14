#ifndef ICI_H
#define ICI_H

#include "matrix2.h"

// 线性组合: result = omega * A + (1-omega) * B
Matrix_2 linearCombination(double omega, Matrix_2* A, Matrix_2* B);

// 评估函数，用于优化
double evaluateFunction(double omega, Matrix_2* covMatrixA, Matrix_2* covMatrixB);

// 黄金分割搜索优化
double goldenSectionSearch(double (*f)(double, Matrix_2*, Matrix_2*), 
                          double a, double b, double tol, 
                          Matrix_2* covMatrixA, Matrix_2* covMatrixB);

// 逆协方差交叉（ICI）算法实现
void ICI(Matrix_2* c, Matrix_2* C, double* omega, 
    Matrix_2* stateA, Matrix_2* covMatrixA, Matrix_2* stateB, Matrix_2* covMatrixB);


#endif