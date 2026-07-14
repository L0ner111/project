#ifndef KALMAN_H
#define KALMAN_H

#include "types.h"

/* ===== 卡尔曼滤波函数原型 ===== */

// 初始化卡尔曼滤波模型
// modelType: 模型类型 (MODEL_CONSTANT_VELOCITY, MODEL_CONSTANT_ACCELERATION, MODEL_COORDINATED_TURN)
// additionalParams: 附加参数数组，用于模型特定参数
// - 对于COORDINATED_TURN模型，additionalParams[0]为转弯率(rad/s)
void initializeKalmanModel(KalmanModel* model, int modelType, double dt, double* additionalParams);

// 初始化卡尔曼滤波模型（兼容旧版本，使用默认参数）
void initializeKalmanModelDefault(KalmanModel* model, int modelType, double dt);

// 预测步骤
// model: 卡尔曼滤波模型
// x: 状态向量
// P: 状态协方差
void predictKalman(KalmanModel* model, Matrix_2* x, Matrix_2* P);

// 更新步骤
// model: 卡尔曼滤波模型
// x: 状态向量
// P: 状态协方差
// z: 测量
void updateKalman(KalmanModel* model, Matrix_2* x, Matrix_2* P, Measurement* z);

// 计算马氏距离
// model: 卡尔曼滤波模型
// x: 状态向量
// z: 测量
// 返回: 马氏距离
double calculateMahalanobisDistance(KalmanModel* model, Matrix_2* x, Matrix_2* P, Measurement* z);

// 计算似然度
// model: 卡尔曼滤波模型
// x: 状态向量
// P: 状态协方差
// z: 测量
// 返回: 似然度值
double calculateLikelihood(KalmanModel* model, Matrix_2* x, Matrix_2* P, Measurement* z);

// 从测量初始化航迹
// track: 航迹指针
// z: 测量
// model: 卡尔曼滤波模型
// trackId: 航迹ID
void initializeTrackFromMeasurement(Track* track, Measurement* z, KalmanModel* model, int trackId);

#endif /* KALMAN_H */