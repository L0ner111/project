// matrix.h
#ifndef MATRIX_H
#define MATRIX_H

// 矩阵结构体（移到文件开头）
typedef struct Matrix {
    float **data;
    int rows;
    int cols;
} Matrix;

// 矩阵创建
Matrix* matrix_create(int rows, int cols);

// 矩阵释放
void matrix_free(Matrix* mat);

// 矩阵复制
void matrix_copy(Matrix* src, Matrix* dst);

// 矩阵加法
void matrix_add(Matrix* A, Matrix* B, Matrix* result);

// 矩阵乘法
void matrix_multiply(Matrix* A, Matrix* B, Matrix* result);

// 矩阵转置
void matrix_transpose(Matrix* A, Matrix* result);

// 矩阵标量乘法
void matrix_scalar_multiply(Matrix* A, float scalar, Matrix* result);

// 矩阵单位矩阵
void matrix_identity(Matrix* mat);

#endif