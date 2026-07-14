#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "sensor_data.h"

// 矩阵创建
Matrix* matrix_create(int rows, int cols) {
    Matrix* mat = (Matrix*)malloc(sizeof(Matrix));
    mat->rows = rows;
    mat->cols = cols;
    mat->data = (float**)malloc(rows * sizeof(float*));
    for (int i = 0; i < rows; i++) {
        mat->data[i] = (float*)calloc(cols, sizeof(float));
    }
    return mat;
}

// 矩阵释放
void matrix_free(Matrix* mat) {
    for (int i = 0; i < mat->rows; i++) {
        free(mat->data[i]);
    }
    free(mat->data);
    free(mat);
}

// 矩阵复制
void matrix_copy(Matrix* src, Matrix* dst) {
    for (int i = 0; i < src->rows; i++) {
        for (int j = 0; j < src->cols; j++) {
            dst->data[i][j] = src->data[i][j];
        }
    }
}

// 矩阵加法
void matrix_add(Matrix* A, Matrix* B, Matrix* result) {
    if (A->rows != B->rows || A->cols != B->cols) return;
    for (int i = 0; i < A->rows; i++) {
        for (int j = 0; j < A->cols; j++) {
            result->data[i][j] = A->data[i][j] + B->data[i][j];
        }
    }
}

// 矩阵乘法
void matrix_multiply(Matrix* A, Matrix* B, Matrix* result) {
    if (A->cols != B->rows || result->rows != A->rows || result->cols != B->cols) return;
    for (int i = 0; i < A->rows; i++) {
        for (int j = 0; j < B->cols; j++) {
            result->data[i][j] = 0;
            for (int k = 0; k < A->cols; k++) {
                result->data[i][j] += A->data[i][k] * B->data[k][j];
            }
        }
    }
}

// 矩阵转置
void matrix_transpose(Matrix* A, Matrix* result) {
    for (int i = 0; i < A->rows; i++) {
        for (int j = 0; j < A->cols; j++) {
            result->data[j][i] = A->data[i][j];
        }
    }
}

// 矩阵标量乘法
void matrix_scalar_multiply(Matrix* A, float scalar, Matrix* result) {
    for (int i = 0; i < A->rows; i++) {
        for (int j = 0; j < A->cols; j++) {
            result->data[i][j] = A->data[i][j] * scalar;
        }
    }
}

// 矩阵单位矩阵
void matrix_identity(Matrix* mat) {
    for (int i = 0; i < mat->rows; i++) {
        for (int j = 0; j < mat->cols; j++) {
            mat->data[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}
