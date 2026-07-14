/*
 * kalman.c - 卡尔曼滤波器实现
 * 适用于三维目标跟踪（XYZ坐标）
 */

#include "kalman.h"
#include "matrix2.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* ========== 常量定义 ========== */
#define PI 3.14159265358979323846

/* ========== 内部函数原型 ========== */
void initializeConstantVelocityModel(KalmanModel *model, double dt);
void initializeConstantAccelerationModel(KalmanModel *model, double dt);
void initializeCoordinatedTurnModel(KalmanModel *model, double dt, double turnRate);
Matrix_2 measurementToStateVector(Measurement *z);
double calculateGaussianPDF(double dist, int dim);

/* ========== 卡尔曼滤波器基础功能 ========== */

// 初始化卡尔曼滤波器模型 - 带附加参数版本
void initializeKalmanModel(KalmanModel *model, int modelType, double dt, double *additionalParams)
{
    double turnRate;
    if (!model)
    {
        printf("Error: Kalman model pointer is null\n");
        return;
    }

    // 根据模型类型初始化
    switch (modelType)
    {
    case MODEL_CONSTANT_VELOCITY:
        initializeConstantVelocityModel(model, dt);
        break;
    case MODEL_CONSTANT_ACCELERATION:
        initializeConstantAccelerationModel(model, dt);
        break;
    case MODEL_COORDINATED_TURN:
        // 使用提供的转弯率，如果为NULL则使用默认值0.1 rad/s
        turnRate = (additionalParams != NULL) ? additionalParams[0] : 0.1;
        initializeCoordinatedTurnModel(model, dt, turnRate);
        break;
    default:
        printf("Error: Unknown model type %d\n", modelType);
        // 默认使用匀速模型
        initializeConstantVelocityModel(model, dt);
    }
}

// 初始化卡尔曼滤波器模型 - 使用默认参数
void initializeKalmanModelDefault(KalmanModel *model, int modelType, double dt)
{
    initializeKalmanModel(model, modelType, dt, NULL);
}

// 卡尔曼滤波器预测步骤：x = F*x + B*u, P = F*P*F^T + Q
void predictKalman(KalmanModel *model, Matrix_2 *x, Matrix_2 *P)
{
    if (!model || !x || !P || !x->data || !P->data)
    {
        printf("Error: Invalid parameters for Kalman prediction\n");
        return;
    }

    // 1. 状态预测: x = F*x
    Matrix_2 xNew = matrixMultiply(&model->F, x);
    matrixCopy(x, &xNew);
    freeMatrix(&xNew);

    // 2. 协方差预测: P = F*P*F^T + Q
    updateCovariance(P, &model->F, &model->Q);
}

// 卡尔曼滤波器更新步骤
// 1. 计算卡尔曼增益: K = P*H^T * (H*P*H^T + R)^(-1)
// 2. 更新状态: x = x + K*(z - H*x)
// 3. 更新协方差: P = (I - K*H)*P
void updateKalman(KalmanModel *model, Matrix_2 *x, Matrix_2 *P, Measurement *z)
{
    if (!model || !x || !P || !z || !x->data || !P->data)
    {
        printf("Error: Invalid parameters for Kalman update\n");
        return;
    }

    // 将测量转换为状态向量格式
    Matrix_2 zVector = measurementToStateVector(z);

    // 1. 计算预测测量: z_pred = H*x
    Matrix_2 zPred = matrixMultiply(&model->H, x);

    // 2. 计算测量残差: y = z - z_pred
    Matrix_2 y = matrixSubtract(&zVector, &zPred);

    // 3. 计算卡尔曼增益
    Matrix_2 K = calculateKalmanGain(P, &model->H, &model->R);

    // 4. 更新状态: x = x + K*y
    Matrix_2 Ky = matrixMultiply(&K, &y);
    Matrix_2 xNew = matrixAdd(x, &Ky);
    matrixCopy(x, &xNew);

    // 5. 更新协方差
    // 5.1 计算 K*H
    Matrix_2 KH = matrixMultiply(&K, &model->H);

    // 5.2 计算 I - K*H
    Matrix_2 I = createIdentityMatrix(KH.rows);
    Matrix_2 IKH = matrixSubtract(&I, &KH);

    // 5.3 计算 P = (I - K*H)*P
    Matrix_2 PNew = matrixMultiply(&IKH, P);
    matrixCopy(P, &PNew);

    // 确保协方差矩阵对称
    for (int i = 0; i < P->rows; i++)
    {
        for (int j = i + 1; j < P->cols; j++)
        {
            double avg = (P->data[i * P->cols + j] + P->data[j * P->cols + i]) / 2.0;
            P->data[i * P->cols + j] = avg;
            P->data[j * P->cols + i] = avg;
        }
    }

    // 释放临时矩阵
    freeMatrix(&zVector);
    freeMatrix(&zPred);
    freeMatrix(&y);
    freeMatrix(&K);
    freeMatrix(&Ky);
    freeMatrix(&xNew);
    freeMatrix(&KH);
    freeMatrix(&I);
    freeMatrix(&IKH);
    freeMatrix(&PNew);
}

// 计算测量与状态预测之间的马氏距离（用于门限判断）
double calculateMahalanobisDistance(KalmanModel *model, Matrix_2 *x, Matrix_2 *P, Measurement *z)
{
    if (!model || !x || !P || !z || !x->data || !P->data)
    {
        printf("Error: Invalid parameters for Mahalanobis distance calculation\n");
        return INFINITY;
    }

    // 将测量转换为状态向量格式
    Matrix_2 zVector = measurementToStateVector(z);

    // 计算预测测量: z_pred = H*x
    Matrix_2 zPred = matrixMultiply(&model->H, x);

    // 计算创新协方差: S = H*P*H^T + R
    Matrix_2 HT = matrixTranspose(&model->H);
    Matrix_2 PHT = matrixMultiply(P, &HT);
    Matrix_2 HPHT = matrixMultiply(&model->H, &PHT);
    Matrix_2 S = matrixAdd(&HPHT, &model->R);

    // 计算马氏距离
    double dist = mahalanobisDistance(&zVector, &zPred, &S);

    // 释放临时矩阵
    freeMatrix(&zVector);
    freeMatrix(&zPred);
    freeMatrix(&HT);
    freeMatrix(&PHT);
    freeMatrix(&HPHT);
    freeMatrix(&S);

    return dist;
}

// 计算似然度（用于数据关联和模型概率更新）
double calculateLikelihood(KalmanModel *model, Matrix_2 *x, Matrix_2 *P, Measurement *z)
{
    if (!model || !x || !P || !z || !x->data || !P->data)
    {
        printf("Error: Invalid parameters for likelihood calculation\n");
        return 0.0;
    }

    // 将测量转换为状态向量格式
    Matrix_2 zVector = measurementToStateVector(z);

    // 计算预测测量: z_pred = H*x
    Matrix_2 zPred = matrixMultiply(&model->H, x);

    // 计算创新向量: v = z - z_pred
    Matrix_2 v = matrixSubtract(&zVector, &zPred);

    // 计算创新协方差: S = H*P*H^T + R
    Matrix_2 HT = matrixTranspose(&model->H);
    Matrix_2 PHT = matrixMultiply(P, &HT);
    Matrix_2 HPHT = matrixMultiply(&model->H, &PHT);
    Matrix_2 S = matrixAdd(&HPHT, &model->R);

    // 计算马氏距离
    double dist = mahalanobisDistance(&zVector, &zPred, &S);

    // 计算似然度（多元高斯分布PDF）
    double likelihood = calculateGaussianPDF(dist, MEAS_DIM);

    // 释放临时矩阵
    freeMatrix(&zVector);
    freeMatrix(&zPred);
    freeMatrix(&v);
    freeMatrix(&HT);
    freeMatrix(&PHT);
    freeMatrix(&HPHT);
    freeMatrix(&S);

    return likelihood;
}

/* ========== 模型初始化 ========== */

// 初始化匀速模型 (CV - Constant Velocity)
void initializeConstantVelocityModel(KalmanModel *model, double dt)
{
    // 状态向量: [x, y, z, vx, vy, vz]
    int stateDim = STATE_DIM;  // 9维
    int measDim = MEAS_DIM; // 3维

    // 保存时间步长
    model->dt = dt;

    // 初始化状态转移矩阵 F
    model->F = createMatrix(stateDim, stateDim);

    // 位置更新
    setMatrixElement(&model->F, 0, 0, 1.0);
    setMatrixElement(&model->F, 0, 3, dt);
    setMatrixElement(&model->F, 1, 1, 1.0);
    setMatrixElement(&model->F, 1, 4, dt);
    setMatrixElement(&model->F, 2, 2, 1.0);
    setMatrixElement(&model->F, 2, 5, dt);

    // 速度保持不变
    setMatrixElement(&model->F, 3, 3, 1.0);
    setMatrixElement(&model->F, 4, 4, 1.0);
    setMatrixElement(&model->F, 5, 5, 1.0);

    // 加速度部分（匀速加速度不变）
    setMatrixElement(&model->F, 6, 6, 1.0);
    setMatrixElement(&model->F, 7, 7, 1.0);
    setMatrixElement(&model->F, 8, 8, 1.0);

    // 初始化测量矩阵 H
    model->H = createMatrix(measDim, stateDim);

    // 我们只测量位置
    setMatrixElement(&model->H, 0, 0, 1.0);
    setMatrixElement(&model->H, 1, 1, 1.0);
    setMatrixElement(&model->H, 2, 2, 1.0);

    // 初始化过程噪声协方差矩阵 Q
    model->Q = createMatrix(stateDim, stateDim);

    // 使用离散白噪声加速度模型
    double q = CV_PROCESS_NOISE;  // 使用配置文件中的噪声参数

    // 位置-位置块
    setMatrixElement(&model->Q, 0, 0, q * dt * dt * dt / 3.0);
    setMatrixElement(&model->Q, 1, 1, q * dt * dt * dt / 3.0);
    setMatrixElement(&model->Q, 2, 2, q * dt * dt * dt / 3.0);

    // 位置-速度块
    setMatrixElement(&model->Q, 0, 3, q * dt * dt / 2.0);
    setMatrixElement(&model->Q, 1, 4, q * dt * dt / 2.0);
    setMatrixElement(&model->Q, 2, 5, q * dt * dt / 2.0);
    setMatrixElement(&model->Q, 3, 0, q * dt * dt / 2.0);
    setMatrixElement(&model->Q, 4, 1, q * dt * dt / 2.0);
    setMatrixElement(&model->Q, 5, 2, q * dt * dt / 2.0);

    // 速度-速度块
    setMatrixElement(&model->Q, 3, 3, q * dt);
    setMatrixElement(&model->Q, 4, 4, q * dt);
    setMatrixElement(&model->Q, 5, 5, q * dt);

    // 加速度噪声
    setMatrixElement(&model->Q, 6, 6, 0.01);
    setMatrixElement(&model->Q, 7, 7, 0.01);
    setMatrixElement(&model->Q, 8, 8, 0.01);

    // 初始化测量噪声协方差矩阵 R
    model->R = createMatrix(measDim, measDim);

    // 假设测量噪声在各个方向上相等且不相关
    double r = MEASUREMENT_NOISE; // 使用配置文件中的噪声参数
    setMatrixElement(&model->R, 0, 0, r);
    setMatrixElement(&model->R, 1, 1, r);
    setMatrixElement(&model->R, 2, 2, r);
}

// 初始化匀加速模型 (CA - Constant Acceleration)
void initializeConstantAccelerationModel(KalmanModel *model, double dt)
{
    // 状态向量: [x, y, z, vx, vy, vz, ax, ay, az]
    int stateDim = STATE_DIM;
    int measDim = MEAS_DIM; // [x, y, z]

    // 保存时间步长
    model->dt = dt;

    // 初始化状态转移矩阵 F
    model->F = createMatrix(stateDim, stateDim);

    // 位置更新
    setMatrixElement(&model->F, 0, 0, 1.0);
    setMatrixElement(&model->F, 0, 3, dt);
    setMatrixElement(&model->F, 0, 6, 0.5 * dt * dt);
    setMatrixElement(&model->F, 1, 1, 1.0);
    setMatrixElement(&model->F, 1, 4, dt);
    setMatrixElement(&model->F, 1, 7, 0.5 * dt * dt);
    setMatrixElement(&model->F, 2, 2, 1.0);
    setMatrixElement(&model->F, 2, 5, dt);
    setMatrixElement(&model->F, 2, 8, 0.5 * dt * dt);

    // 速度更新
    setMatrixElement(&model->F, 3, 3, 1.0);
    setMatrixElement(&model->F, 3, 6, dt);
    setMatrixElement(&model->F, 4, 4, 1.0);
    setMatrixElement(&model->F, 4, 7, dt);
    setMatrixElement(&model->F, 5, 5, 1.0);
    setMatrixElement(&model->F, 5, 8, dt);

    // 加速度保持不变
    setMatrixElement(&model->F, 6, 6, 1.0);
    setMatrixElement(&model->F, 7, 7, 1.0);
    setMatrixElement(&model->F, 8, 8, 1.0);

    // 初始化测量矩阵 H
    model->H = createMatrix(measDim, stateDim);

    // 我们只测量位置
    setMatrixElement(&model->H, 0, 0, 1.0);
    setMatrixElement(&model->H, 1, 1, 1.0);
    setMatrixElement(&model->H, 2, 2, 1.0);

    // 初始化过程噪声协方差矩阵 Q
    model->Q = createMatrix(stateDim, stateDim);

    // 假设加速度变化率是白噪声
    double q = 0.1; // 过程噪声强度（可调）

    // 简化的噪声模型 - 只在加速度上添加噪声
    setMatrixElement(&model->Q, 6, 6, q);
    setMatrixElement(&model->Q, 7, 7, q);
    setMatrixElement(&model->Q, 8, 8, q);

    // 初始化测量噪声协方差矩阵 R
    model->R = createMatrix(measDim, measDim);

    // 假设测量噪声在各个方向上相等且不相关
    double r = 1.0; // 测量噪声强度（可调）
    setMatrixElement(&model->R, 0, 0, r);
    setMatrixElement(&model->R, 1, 1, r);
    setMatrixElement(&model->R, 2, 2, r);
}

// 初始化协调转弯模型 (CT - Coordinated Turn)
void initializeCoordinatedTurnModel(KalmanModel *model, double dt, double turnRate)
{
    // 简化的CT模型，固定转弯率
    // 状态向量: [x, y, z, vx, vy, vz]
    int stateDim = STATE_DIM;  // 9维
    int measDim = MEAS_DIM;  // 3维

    // 保存时间步长
    model->dt = dt;

    // 初始化状态转移矩阵 F
    model->F = createMatrix(stateDim, stateDim);

    // 计算转弯参数
    double omega = turnRate; // 转弯角速度
    double sinOmegaT = sin(omega * dt);
    double cosOmegaT = cos(omega * dt);

    // X-Y平面内的转弯
    setMatrixElement(&model->F, 0, 0, 1.0);
    setMatrixElement(&model->F, 0, 3, sinOmegaT / omega);
    setMatrixElement(&model->F, 0, 4, (1 - cosOmegaT) / omega);
    setMatrixElement(&model->F, 1, 1, 1.0);
    setMatrixElement(&model->F, 1, 3, (cosOmegaT - 1) / omega);
    setMatrixElement(&model->F, 1, 4, sinOmegaT / omega);

    // Z轴上匀速运动
    setMatrixElement(&model->F, 2, 2, 1.0);
    setMatrixElement(&model->F, 2, 5, dt);

    // 速度更新
    setMatrixElement(&model->F, 3, 3, cosOmegaT);
    setMatrixElement(&model->F, 3, 4, -sinOmegaT);
    setMatrixElement(&model->F, 4, 3, sinOmegaT);
    setMatrixElement(&model->F, 4, 4, cosOmegaT);
    setMatrixElement(&model->F, 5, 5, 1.0);

    // 加速度部分
    setMatrixElement(&model->F, 6, 6, 1.0);
    setMatrixElement(&model->F, 7, 7, 1.0);
    setMatrixElement(&model->F, 8, 8, 1.0);

    // 初始化测量矩阵 H
    model->H = createMatrix(measDim, stateDim);

    // 我们只测量位置
    setMatrixElement(&model->H, 0, 0, 1.0);
    setMatrixElement(&model->H, 1, 1, 1.0);
    setMatrixElement(&model->H, 2, 2, 1.0);

    // 初始化过程噪声协方差矩阵 Q
    model->Q = createMatrix(stateDim, stateDim);

    // 使用与匀速模型相似的噪声模型
    double q = CT_PROCESS_NOISE; // 过程噪声强度（可调）

    // 位置-位置块
    setMatrixElement(&model->Q, 0, 0, q * dt * dt * dt / 3.0);
    setMatrixElement(&model->Q, 1, 1, q * dt * dt * dt / 3.0);
    setMatrixElement(&model->Q, 2, 2, q * dt * dt * dt / 3.0);

    // 位置-速度块
    setMatrixElement(&model->Q, 0, 3, q * dt * dt / 2.0);
    setMatrixElement(&model->Q, 1, 4, q * dt * dt / 2.0);
    setMatrixElement(&model->Q, 2, 5, q * dt * dt / 2.0);
    setMatrixElement(&model->Q, 3, 0, q * dt * dt / 2.0);
    setMatrixElement(&model->Q, 4, 1, q * dt * dt / 2.0);
    setMatrixElement(&model->Q, 5, 2, q * dt * dt / 2.0);

    // 速度-速度块
    setMatrixElement(&model->Q, 3, 3, q * dt);
    setMatrixElement(&model->Q, 4, 4, q * dt);
    setMatrixElement(&model->Q, 5, 5, q * dt);

    // 加速度部分的噪声
    setMatrixElement(&model->Q, 6, 6, 0.01);
    setMatrixElement(&model->Q, 7, 7, 0.01);
    setMatrixElement(&model->Q, 8, 8, 0.01);

    // 初始化测量噪声协方差矩阵 R
    model->R = createMatrix(measDim, measDim);

    // 假设测量噪声在各个方向上相等且不相关
    double r = MEASUREMENT_NOISE; // 使用配置文件中的噪声参数
    setMatrixElement(&model->R, 0, 0, r);
    setMatrixElement(&model->R, 1, 1, r);
    setMatrixElement(&model->R, 2, 2, r);
}

/* ========== 工具函数 ========== */

// 从测量创建状态向量（只包含位置部分）
Matrix_2 measurementToStateVector(Measurement *z)
{
    // 创建一个3维向量
    Matrix_2 zVector = createMatrix(3, 1);

    // 填充位置数据
    setMatrixElement(&zVector, 0, 0, z->position.x);
    setMatrixElement(&zVector, 1, 0, z->position.y);
    setMatrixElement(&zVector, 2, 0, z->position.z);

    return zVector;
}

// 计算多元高斯分布概率密度函数值
// dist: 马氏距离
// dim: 维度数
double calculateGaussianPDF(double dist, int dim)
{
    double exponent = -0.5 * dist * dist;
    double normalizer = pow(2.0 * PI, dim / 2.0);

    return exp(exponent) / normalizer;
}

/* ========== 航迹初始化函数 ========== */

// 从测量初始化一个新的航迹状态
void initializeTrackFromMeasurement(Track *track, Measurement *z, KalmanModel *model, int trackId)
{
    if (!track || !z || !model)
    {
        printf("Error: Invalid parameters for track initialization\n");
        return;
    }

    // 获取状态向量维度
    int stateDim = model->F.rows;

    // 初始化状态向量
    track->x = createMatrix(stateDim, 1);

    // 设置位置
    setMatrixElement(&track->x, 0, 0, z->position.x);
    setMatrixElement(&track->x, 1, 0, z->position.y);
    setMatrixElement(&track->x, 2, 0, z->position.z);

    // 初始速度和加速度设为0
    for (int i = 3; i < stateDim; i++)
    {
        setMatrixElement(&track->x, i, 0, 0.0);
    }

    // 初始化协方差矩阵
    track->P = createMatrix(stateDim, stateDim);

    // 位置初始协方差来自测量噪声
    setMatrixElement(&track->P, 0, 0, getMatrixElement(&model->R, 0, 0));
    setMatrixElement(&track->P, 1, 1, getMatrixElement(&model->R, 1, 1));
    setMatrixElement(&track->P, 2, 2, getMatrixElement(&model->R, 2, 2));

    // 速度和加速度初始协方差设为较大值（表示不确定性大）
    for (int i = 3; i < stateDim; i++)
    {
        setMatrixElement(&track->P, i, i, 100.0);
    }

    // 设置航迹ID和时间
    track->id = trackId;
    track->birthTime = (int)z->time;
    track->lastUpdateTime = (int)z->time;
    track->numMisses = 0;

    // 如果是IMM，初始模型概率为均匀分布
    for (int i = 0; i < MAX_MODELS; i++)
    {
        track->modelProbabilities[i] = 1.0 / MAX_MODELS;
    }
}
