/*
 * matrix.c - IMM-MHT算法的矩阵运算库
 *
 * 提供IMM-MHT算法所需的基础矩阵运算功能
 * 优化内存使用和计算效率
 */

#include "matrix2.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* ========== 基本矩阵操作函数 ========== */

// 创建指定大小的矩阵并初始化为0
Matrix_2 createMatrix(int rows, int cols)
{
    Matrix_2 mat;
    mat.rows = rows;
    mat.cols = cols;
    mat.data = (double *)malloc(rows * cols * sizeof(double));

    // 检查内存分配是否成功
    if (mat.data == NULL)
    {
        printf("Error: Matrix_2 memory allocation failed\n");
        mat.rows = 0;
        mat.cols = 0;
        return mat;
    }

    // 初始化为0
    for (int i = 0; i < rows * cols; i++)
    {
        mat.data[i] = 0.0;
    }

    return mat;
}

// 创建并返回单位矩阵
Matrix_2 createIdentityMatrix(int size)
{
    Matrix_2 mat = createMatrix(size, size);

    // 设置对角线元素为1
    for (int i = 0; i < size; i++)
    {
        mat.data[i * size + i] = 1.0;
    }

    return mat;
}

// 释放矩阵内存
void freeMatrix(Matrix_2 *mat)
{
    if (mat && mat->data)
    {
        free(mat->data);
        mat->data = NULL;
    }
    if (mat)
    {
        mat->rows = 0;
        mat->cols = 0;
    }
}

// 获取矩阵元素 M(i,j)
double getMatrixElement(Matrix_2 *mat, int i, int j)
{
    if (!mat || !mat->data || i < 0 || i >= mat->rows || j < 0 || j >= mat->cols)
    {
        printf("Error: Matrix_2 index out of bounds [%d,%d]\n", i, j);
        return 0.0;
    }
    return mat->data[i * mat->cols + j];
}

// 设置矩阵元素 M(i,j) = value
void setMatrixElement(Matrix_2 *mat, int i, int j, double value)
{
    if (!mat || !mat->data || i < 0 || i >= mat->rows || j < 0 || j >= mat->cols)
    {
        printf("Error: Matrix_2 index out of bounds [%d,%d]\n", i, j);
        return;
    }
    mat->data[i * mat->cols + j] = value;
}

// 矩阵复制 dst = src
void matrixCopy(Matrix_2 *dst, Matrix_2 *src)
{
    if (!dst || !src || !dst->data || !src->data)
    {
        printf("Error: Invalid parameters for matrix copying\n");
        return;
    }

    // 如果尺寸不同，先释放目标矩阵并重新分配
    if (dst->rows != src->rows || dst->cols != src->cols)
    {
        freeMatrix(dst);
        *dst = createMatrix(src->rows, src->cols);
    }

    // 复制数据
    for (int i = 0; i < src->rows * src->cols; i++)
    {
        dst->data[i] = src->data[i];
    }
}

/* ========== 矩阵代数运算 ========== */

// 矩阵加法：C = A + B
Matrix_2 matrixAdd(Matrix_2 *A, Matrix_2 *B)
{
    // 检查尺寸是否匹配
    if (A->rows != B->rows || A->cols != B->cols)
    {
        printf("Error: Matrix_2 addition dimensions mismatch (%dx%d) + (%dx%d)\n", 
            A->rows, A->cols, B->rows, B->cols);
        Matrix_2 result = {0, 0, NULL};
        return result;
    }

    Matrix_2 C = createMatrix(A->rows, A->cols);

    // 逐元素相加
    for (int i = 0; i < A->rows * A->cols; i++)
    {
        C.data[i] = A->data[i] + B->data[i];
    }

    return C;
}

// 矩阵减法：C = A - B
Matrix_2 matrixSubtract(Matrix_2 *A, Matrix_2 *B)
{
    // 检查尺寸是否匹配
    if (A->rows != B->rows || A->cols != B->cols)
    {
        printf("Error: Matrix_2 subtraction dimensions mismatch (%dx%d) - (%dx%d)\n", 
            A->rows, A->cols, B->rows, B->cols);
        Matrix_2 result = {0, 0, NULL};
        return result;
    }

    Matrix_2 C = createMatrix(A->rows, A->cols);

    // 逐元素相减
    for (int i = 0; i < A->rows * A->cols; i++)
    {
        C.data[i] = A->data[i] - B->data[i];
    }

    return C;
}

// 矩阵乘法：C = A * B
Matrix_2 matrixMultiply(Matrix_2 *A, Matrix_2 *B)
{
    // 检查乘法尺寸要求
    if (A->cols != B->rows)
    {
        printf("Error: Matrix_2 multiplication dimensions mismatch (%dx%d) * (%dx%d)\n", 
            A->rows, A->cols, B->rows, B->cols);
        Matrix_2 result = {0, 0, NULL};
        return result;
    }

    Matrix_2 C = createMatrix(A->rows, B->cols);

    // 执行矩阵乘法
    for (int i = 0; i < A->rows; i++)
    {
        for (int j = 0; j < B->cols; j++)
        {
            double sum = 0.0;
            for (int k = 0; k < A->cols; k++)
            {
                sum += A->data[i * A->cols + k] * B->data[k * B->cols + j];
            }
            C.data[i * C.cols + j] = sum;
        }
    }

    return C;
}

// 矩阵标量乘法：B = alpha * A
Matrix_2 matrixScalarMultiply(double alpha, Matrix_2 *A)
{
    Matrix_2 B = createMatrix(A->rows, A->cols);

    // 逐元素乘以标量
    for (int i = 0; i < A->rows * A->cols; i++)
    {
        B.data[i] = alpha * A->data[i];
    }

    return B;
}

// 矩阵转置：B = A^T
Matrix_2 matrixTranspose(Matrix_2 *A)
{
    Matrix_2 B = createMatrix(A->cols, A->rows);

    // 转置矩阵
    for (int i = 0; i < A->rows; i++)
    {
        for (int j = 0; j < A->cols; j++)
        {
            B.data[j * B.cols + i] = A->data[i * A->cols + j];
        }
    }

    return B;
}

// 计算矩阵的迹（对角线元素之和）
double matrixTrace(Matrix_2* A) {
    double trace = 0;
    int minDim = (A->rows < A->cols) ? A->rows : A->cols;
    for (int i = 0; i < minDim; i++) {
        trace += A->data[i * A->cols + i];
    }
    return trace;
}


/* ========== 矩阵高级操作 ========== */

// 计算小矩阵行列式（2x2或3x3）
double matrixDeterminant(Matrix_2 *A)
{
    // 矩阵必须是方阵
    if (A->rows != A->cols)
    {
        printf("Error: Determinant can only be calculated for square matrices\n");
        return 0.0;
    }

    // 1x1矩阵
    if (A->rows == 1)
    {
        return A->data[0];
    }

    // 2x2矩阵
    if (A->rows == 2)
    {
        return A->data[0] * A->data[3] - A->data[1] * A->data[2];
    }

    // 3x3矩阵
    if (A->rows == 3)
    {
        return A->data[0] * (A->data[4] * A->data[8] - A->data[5] * A->data[7]) - A->data[1] * (A->data[3] * A->data[8] - A->data[5] * A->data[6]) + A->data[2] * (A->data[3] * A->data[7] - A->data[4] * A->data[6]);
    }

    // 对于更大的矩阵，这里应该实现更复杂的算法
    // 如LU分解、高斯消元等，但考虑到内存限制，我们可以假设
    // 不会处理大于3x3的行列式
    printf("Warning: Matrices > 3x3 not supported.\n");
    return 0.0;
}

// 提取矩阵子矩阵（用于余子式计算）
Matrix_2 getSubMatrix(Matrix_2 *A, int excludeRow, int excludeCol)
{
    if (A->rows <= 1 || A->cols <= 1)
    {
        printf("Error: Can't extract submatrix from 1x1 matrix.\n");
        Matrix_2 nullMat = {0, 0, NULL};
        return nullMat;
    }

    Matrix_2 sub = createMatrix(A->rows - 1, A->cols - 1);

    int subRow = 0;
    for (int i = 0; i < A->rows; i++)
    {
        if (i == excludeRow)
            continue;

        int subCol = 0;
        for (int j = 0; j < A->cols; j++)
        {
            if (j == excludeCol)
                continue;

            sub.data[subRow * sub.cols + subCol] = A->data[i * A->cols + j];
            subCol++;
        }
        subRow++;
    }

    return sub;
}

// 计算余子式
double cofactor(Matrix_2 *A, int i, int j)
{
    Matrix_2 sub = getSubMatrix(A, i, j);
    double det = matrixDeterminant(&sub);
    freeMatrix(&sub);

    // 添加符号
    return ((i + j) % 2 == 0) ? det : -det;
}

// 计算伴随矩阵
Matrix_2 adjointMatrix(Matrix_2 *A)
{
    // 矩阵必须是方阵
    if (A->rows != A->cols)
    {
        printf("Error: Only square matrices have adjugate matrices.\n");
        Matrix_2 nullMat = {0, 0, NULL};
        return nullMat;
    }

    Matrix_2 adj = createMatrix(A->rows, A->cols);

    // 计算每个元素的余子式
    for (int i = 0; i < A->rows; i++)
    {
        for (int j = 0; j < A->cols; j++)
        {
            // 计算余子式
            adj.data[j * adj.cols + i] = cofactor(A, i, j); // 注意转置
        }
    }

    return adj;
}

// 矩阵求逆：B = A^(-1)，使用伴随矩阵法
// Matrix_2 matrixInverse(Matrix_2 *A)
// {
//     // 矩阵必须是方阵
//     if (A->rows != A->cols)
//     {
//         printf("Error: Only square matrices are invertible.\n");
//         Matrix_2 nullMat = {0, 0, NULL};
//         return nullMat;
//     }

//     // 计算行列式
//     double det = matrixDeterminant(A);

//     // 检查行列式是否接近于0（奇异矩阵）
//     if (fabs(det) < 1e-10)
//     {
//         printf("Error: Matrix_2 is nearly singular, cannot invert.\n");
//         Matrix_2 nullMat = {0, 0, NULL};
//         return nullMat;
//     }

//     // 1x1矩阵的特殊情况
//     if (A->rows == 1)
//     {
//         Matrix_2 inv = createMatrix(1, 1);
//         inv.data[0] = 1.0 / A->data[0];
//         return inv;
//     }

//     // 计算伴随矩阵
//     Matrix_2 adj = adjointMatrix(A);

//     // 计算逆矩阵 A^(-1) = adj(A) / det(A)
//     Matrix_2 inv = matrixScalarMultiply(1.0 / det, &adj);

//     // 释放临时矩阵
//     freeMatrix(&adj);

//     return inv;
// }

// 使用高斯消元法计算方阵的逆矩阵 - 与正确版本一致
Matrix_2 matrixInverse(Matrix_2* A) {
    int n = A->rows;
    // if (n != A->cols) return 0;  // 只处理方阵
    
    // 创建增广矩阵 [A|I]
    Matrix_2 aug = createMatrix(n, 2*n);
    Matrix_2 result = createMatrix(n, n);
    
    // 填充增广矩阵
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            aug.data[i * (2*n) + j] = A->data[i * n + j];
        }
        aug.data[i * (2*n) + (n + i)] = 1.0;  // 单位矩阵部分
    }
    
    // 高斯消元法
    for (int i = 0; i < n; i++) {
        // 寻找主元
        int pivotRow = i;
        double maxPivot = fabs(aug.data[i * (2*n) + i]);
        
        for (int j = i + 1; j < n; j++) {
            if (fabs(aug.data[j * (2*n) + i]) > maxPivot) {
                pivotRow = j;
                maxPivot = fabs(aug.data[j * (2*n) + i]);
            }
        }
        
        if (maxPivot < 1e-10) {
            freeMatrix(&aug);
            Matrix_2 nullMat = {0, 0, NULL};
            return nullMat;  // 奇异矩阵
        }
        
        // 如果需要，交换行
        if (pivotRow != i) {
            for (int j = 0; j < 2*n; j++) {
                double temp = aug.data[i * (2*n) + j];
                aug.data[i * (2*n) + j] = aug.data[pivotRow * (2*n) + j];
                aug.data[pivotRow * (2*n) + j] = temp;
            }
        }
        
        // 缩放主元行
        double pivot = aug.data[i * (2*n) + i];
        for (int j = 0; j < 2*n; j++) {
            aug.data[i * (2*n) + j] /= pivot;
        }
        
        // 消除其他行
        for (int j = 0; j < n; j++) {
            if (j != i) {
                double factor = aug.data[j * (2*n) + i];
                for (int k = 0; k < 2*n; k++) {
                    aug.data[j * (2*n) + k] -= factor * aug.data[i * (2*n) + k];
                }
            }
        }
    }
    
    // 提取逆矩阵
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            result.data[i * n + j] = aug.data[i * (2*n) + (n + j)];
        }
    }
    
    freeMatrix(&aug);
    return result;
}

/* ========== 特殊矩阵操作（卡尔曼滤波相关）========== */
// 计算两个向量的外积：C = a * b^T（卡尔曼滤波协方差更新中常用）
Matrix_2 outerProduct(Matrix_2* a, Matrix_2* b) {
    // 确定输入是否为向量（行向量或列向量）
    int aIsVector = (a->rows == 1 || a->cols == 1);
    int bIsVector = (b->rows == 1 || b->cols == 1);
    
    if (!aIsVector || !bIsVector) {
        printf("Error: Outer product can only be calculated between vectors\n");
        Matrix_2 nullMat = {0, 0, NULL};
        return nullMat;
    }
    
    // 获取向量长度
    int aLen = (a->rows > a->cols) ? a->rows : a->cols;
    int bLen = (b->rows > b->cols) ? b->rows : b->cols;
    
    // 创建结果矩阵
    Matrix_2 C = createMatrix(aLen, bLen);
    
    // 计算外积
    for (int i = 0; i < aLen; i++) {
        double aVal;
        if (a->cols == 1) { // 列向量
            aVal = a->data[i];
        } else { // 行向量
            aVal = a->data[i * a->cols];
        }
        
        for (int j = 0; j < bLen; j++) {
            double bVal;
            if (b->rows == 1) { // 行向量
                bVal = b->data[j];
            } else { // 列向量
                bVal = b->data[j * b->cols];
            }
            C.data[i * C.cols + j] = aVal * bVal;
        }
    }
    
    return C;
}

// 计算马氏距离: sqrt((x-m)^T * S^(-1) * (x-m))
// 用于量测验证与数据关联
double mahalanobisDistance(Matrix_2 *x, Matrix_2 *m, Matrix_2 *S)
{
    // 确保所有输入都是向量或矩阵
    if (!x || !m || !S || !x->data || !m->data || !S->data)
    {
        printf("Error: Invalid parameters for Mahalanobis distance calculation.\n");
        return INFINITY;
    }

    // 检查x和m的尺寸
    if (x->rows != m->rows || x->cols != m->cols)
    {
        printf("Error: Vector sizes do not match.\n");
        return INFINITY;
    }

    // 计算差值向量
    Matrix_2 diff = matrixSubtract(x, m);

    // 计算协方差矩阵的逆
    Matrix_2 SInv = matrixInverse(S);

    // 计算(x-m)^T * S^(-1)
    Matrix_2 diffT = matrixTranspose(&diff);
    Matrix_2 temp = matrixMultiply(&diffT, &SInv);

    // 最后计算(x-m)^T * S^(-1) * (x-m)
    Matrix_2 result = matrixMultiply(&temp, &diff);

    // 提取结果（标量）并计算平方根
    double distance = sqrt(result.data[0]);

    // 释放临时矩阵
    freeMatrix(&diff);
    freeMatrix(&diffT);
    freeMatrix(&SInv);
    freeMatrix(&temp);
    freeMatrix(&result);

    return distance;
}

/* ========== 针对卡尔曼滤波器的优化函数 ========== */
// 卡尔曼滤波器协方差预测: P = F*P*F^T + Q
// void updateCovariance(Matrix_2* P, Matrix_2* F, Matrix_2* Q) {
//     // 计算F*P
//     Matrix_2 FP = matrixMultiply(F, P);
    
//     // 计算F*P*F^T
//     Matrix_2 FT = matrixTranspose(F);
//     Matrix_2 FPFT = matrixMultiply(&FP, &FT);
    
//     // 计算F*P*F^T + Q
//     Matrix_2 result = matrixAdd(&FPFT, Q);
    
//     // 将结果复制回P
//     matrixCopy(P, &result);
    
//     // 释放临时矩阵
//     freeMatrix(&FP);
//     freeMatrix(&FT);
//     freeMatrix(&FPFT);
//     freeMatrix(&result);
// }

// 卡尔曼滤波器协方差更新: P = F*P*F^T + Q
void updateCovariance(Matrix_2* P, Matrix_2* F, Matrix_2* Q) {
    // 检查矩阵维度兼容性
    if (P->rows != P->cols || F->rows != F->cols || Q->rows != Q->cols ||
        P->rows != F->rows || P->rows != Q->rows) {
        printf("Error: Matrix_2 dimensions mismatch for covariance update\n");
        return;
    }
    // 创建临时矩阵
    int n = P->rows;
    Matrix_2 temp = createMatrix(n, n);
    // 执行 P = F*P*F^T + Q
    // 步骤 1: 计算 F*P
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            temp.data[i*n + j] = 0;
            for (int k = 0; k < n; k++) {
                temp.data[i*n + j] += F->data[i*n + k] * P->data[k*n + j];
            }
        }
    }
    // 步骤 2: 计算 (F*P)*F^T 并加上 Q
    Matrix_2 originalP = createMatrix(n, n);
    matrixCopy(&originalP, P);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            P->data[i*n + j] = Q->data[i*n + j];
            for (int k = 0; k < n; k++) {
                P->data[i*n + j] += temp.data[i*n + k] * F->data[j*n + k];
            }
        }
    }
    // 释放临时矩阵
    freeMatrix(&temp);
    freeMatrix(&originalP);
}

// 计算卡尔曼增益: K = P*H^T * (H*P*H^T + R)^(-1)
Matrix_2 calculateKalmanGain(Matrix_2 *P, Matrix_2 *H, Matrix_2 *R)
{
    // 计算H^T
    Matrix_2 HT = matrixTranspose(H);

    // 计算P*H^T
    Matrix_2 PHT = matrixMultiply(P, &HT);

    // 计算H*P*H^T
    Matrix_2 HPHT = matrixMultiply(H, &PHT);

    // 计算S = H*P*H^T + R
    Matrix_2 S = matrixAdd(&HPHT, R);

    // 计算S^(-1)
    Matrix_2 SInv = matrixInverse(&S);

    // 计算K = P*H^T * S^(-1)
    Matrix_2 K = matrixMultiply(&PHT, &SInv);

    // 释放临时矩阵
    freeMatrix(&HT);
    freeMatrix(&PHT);
    freeMatrix(&HPHT);
    freeMatrix(&S);
    freeMatrix(&SInv);

    return K;
}

/* ========== 辅助函数 ========== */

// 打印矩阵（调试用）
void printMatrix(Matrix_2 *mat)
{
    if (!mat || !mat->data)
    {
        printf("Error: Invalid matrix\n");
        return;
    }

    printf("Matrix_2 (%dx%d):\n", mat->rows, mat->cols);
    for (int i = 0; i < mat->rows; i++)
    {
        for (int j = 0; j < mat->cols; j++)
        {
            printf("%8.4f ", mat->data[i * mat->cols + j]);
        }
        printf("\n");
    }
    printf("\n");
}

// 辅助函数：打印矩阵
void printMatrixICI(Matrix_2* mat, const char* name) {
    printf("%s (%dx%d):\n", name, mat->rows, mat->cols);
    for (int i = 0; i < mat->rows; i++) {
        for (int j = 0; j < mat->cols; j++) {
            printf("%f ", mat->data[i * mat->cols + j]);
        }
        printf("\n");
    }
    printf("\n");
}
