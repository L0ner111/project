#include <math.h>   
#include "matrix2.h"

// 线性组合: result = omega * A + (1-omega) * B
Matrix_2 linearCombination(double omega, Matrix_2* A, Matrix_2* B) {

    int rows = A->rows;
    int cols = A->cols;
    Matrix_2 result = createMatrix(rows, cols);
    for (int i = 0; i < A->rows * A->cols; i++) {
        result.data[i] = omega * A->data[i] + (1-omega) * B->data[i];
    }

    return result;
}

// 评估函数，用于优化
double evaluateFunction(double omega, Matrix_2* covMatrixA, Matrix_2* covMatrixB) {
    int n = covMatrixA->rows;
    Matrix_2 omegacovMatrixA_plus_1minusomegaCB = createMatrix(n, n);
    Matrix_2 inv_omegacovMatrixA_plus_1minusomegaCB = createMatrix(n, n);
    Matrix_2 invcovMatrixA = createMatrix(n, n);
    Matrix_2 invCB = createMatrix(n, n);
    Matrix_2 sum = createMatrix(n, n);
    Matrix_2 diff = createMatrix(n, n);
    Matrix_2 result = createMatrix(n, n);
    
    // 计算 inv(covMatrixA) 和 inv(covMatrixB)
    invcovMatrixA = matrixInverse(covMatrixA);
    invCB = matrixInverse(covMatrixB);
    
    // 计算 omega*covMatrixA+(1-omega)*covMatrixB
    omegacovMatrixA_plus_1minusomegaCB = linearCombination(omega, covMatrixA, covMatrixB);
    
    // 计算 inv(omega*covMatrixA+(1-omega)*covMatrixB)
    inv_omegacovMatrixA_plus_1minusomegaCB = matrixInverse(&omegacovMatrixA_plus_1minusomegaCB);
    
    // 计算 inv(covMatrixA)+inv(covMatrixB)
    sum = matrixAdd(&invcovMatrixA, &invCB);
    
    // 计算 inv(covMatrixA)+inv(covMatrixB)-inv(omega*covMatrixA+(1-omega)*covMatrixB)
    diff = matrixSubtract(&sum, &inv_omegacovMatrixA_plus_1minusomegaCB);
    
    // 计算 inv(inv(covMatrixA)+inv(covMatrixB)-inv(omega*covMatrixA+(1-omega)*covMatrixB))
    result = matrixInverse(&diff);
    
    // 计算迹
    double trace = matrixTrace(&result);
    
    // 释放内存
    freeMatrix(&omegacovMatrixA_plus_1minusomegaCB);
    freeMatrix(&inv_omegacovMatrixA_plus_1minusomegaCB);
    freeMatrix(&invcovMatrixA);
    freeMatrix(&invCB);
    freeMatrix(&sum);
    freeMatrix(&diff);
    freeMatrix(&result);
    
    return trace;
}

// 黄金分割搜索优化
double goldenSectionSearch(double (*f)(double, Matrix_2*, Matrix_2*), 
                          double a, double b, double tol, 
                          Matrix_2* covMatrixA, Matrix_2* covMatrixB) {
    const double phi = (1 + sqrt(5)) / 2;
    double c = b - (b - a) / phi;
    double d = a + (b - a) / phi;
    
    while (fabs(b - a) > tol) {
        double fc = f(c, covMatrixA, covMatrixB);
        double fd = f(d, covMatrixA, covMatrixB);
        
        if (fc < fd) {
            b = d;
        } else {
            a = c;
        }
        
        c = b - (b - a) / phi;
        d = a + (b - a) / phi;
    }
    
    return (a + b) / 2;
}

// 逆协方差交叉（ICI）算法实现
void ICI(Matrix_2* c, Matrix_2* C, double* omega, 
         Matrix_2* stateA, Matrix_2* covMatrixA, Matrix_2* stateB, Matrix_2* covMatrixB) {
    int n = covMatrixA->rows;
    
    // 查找最优omega值使用黄金分割搜索
    *omega = goldenSectionSearch(evaluateFunction, 0, 1, 1e-10, covMatrixA, covMatrixB);
    
    // 临时矩阵
    Matrix_2 invcovMatrixA = createMatrix(n, n);
    Matrix_2 invCB = createMatrix(n, n);
    Matrix_2 omegacovMatrixA_plus_1minusomegaCB = createMatrix(n, n);
    Matrix_2 inv_omegacovMatrixA_plus_1minusomegaCB = createMatrix(n, n);
    Matrix_2 sum = createMatrix(n, n);
    Matrix_2 diff = createMatrix(n, n);
    Matrix_2 KICI = createMatrix(n, n);
    Matrix_2 LICI = createMatrix(n, n);
    Matrix_2 temp1 = createMatrix(n, n);
    Matrix_2 temp2 = createMatrix(n, n);
    Matrix_2 KICIxA = createMatrix(n, 1);
    Matrix_2 LICIxB = createMatrix(n, 1);
    
    // 计算inv(covMatrixA)和inv(covMatrixB)
    invcovMatrixA = matrixInverse(covMatrixA);
    invCB = matrixInverse(covMatrixB);
    
    // 计算omega*covMatrixA+(1-omega)*covMatrixB
    omegacovMatrixA_plus_1minusomegaCB = linearCombination(*omega, covMatrixA, covMatrixB);
    
    // 计算inv(omega*covMatrixA+(1-omega)*covMatrixB)
    inv_omegacovMatrixA_plus_1minusomegaCB = matrixInverse(&omegacovMatrixA_plus_1minusomegaCB);
    
    // 计算inv(covMatrixA)+inv(covMatrixB)
    sum = matrixAdd(&invcovMatrixA, &invCB);
    
    // 计算inv(covMatrixA)+inv(covMatrixB)-inv(omega*covMatrixA+(1-omega)*covMatrixB)
    diff = matrixSubtract(&sum, &inv_omegacovMatrixA_plus_1minusomegaCB);
    
    // 计算C = inv(inv(covMatrixA)+inv(covMatrixB)-inv(omega*covMatrixA+(1-omega)*covMatrixB))
    *C = matrixInverse(&diff);
    
    // 计算KICI = C*(inv(covMatrixA)-omega*inv(omega*covMatrixA+(1-omega)*covMatrixB))
    temp1 = matrixScalarMultiply(*omega, &inv_omegacovMatrixA_plus_1minusomegaCB);
    temp2 = matrixSubtract(&invcovMatrixA, &temp1);
    KICI = matrixMultiply(C, &temp2);
    
    // 计算LICI = C*(inv(covMatrixB)-(1-omega)*inv(omega*covMatrixA+(1-omega)*covMatrixB))
    temp1 = matrixScalarMultiply(1-*omega, &inv_omegacovMatrixA_plus_1minusomegaCB);
    temp2 = matrixSubtract(&invCB, &temp1);
    LICI = matrixMultiply(C, &temp2);
    
    // 计算c = KICI*stateA + LICI*stateB
    KICIxA = matrixMultiply(&KICI, stateA);
    LICIxB = matrixMultiply(&LICI, stateB);
    *c = matrixAdd(&KICIxA, &LICIxB);
    
    // 释放内存
    freeMatrix(&invcovMatrixA);
    freeMatrix(&invCB);
    freeMatrix(&omegacovMatrixA_plus_1minusomegaCB);
    freeMatrix(&inv_omegacovMatrixA_plus_1minusomegaCB);
    freeMatrix(&sum);
    freeMatrix(&diff);
    freeMatrix(&KICI);
    freeMatrix(&LICI);
    freeMatrix(&temp1);
    freeMatrix(&temp2);
    freeMatrix(&KICIxA);
    freeMatrix(&LICIxB);
}