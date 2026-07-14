#ifndef IMM_H
#define IMM_H

#include "types.h"
#include "matrix2.h"
#include "kalman.h"
#include "config.h"

/* ===== 模型参数结构 ===== */
typedef struct {
    double cvProcessNoise;       // 匀速模型过程噪声
    double caProcessNoise;       // 匀加速模型过程噪声
    double ctProcessNoise;       // 协调转弯模型过程噪声
    double measurementNoise;     // 测量噪声强度
    double turnRate;             // 协调转弯模型转弯率(rad/s)
    double selfTransProb;        // 模型自转移概率
} ImmParameters;

/* ===== 函数原型 ===== */
// 初始化默认IMM参数
void initializeDefaultImmParameters(ImmParameters* params);

// 初始化IMM模型集合并设置转移概率矩阵 - 带参数
void initializeImmModelsWithParams(TrackingSystem* system, double dt, ImmParameters* params);

// 初始化IMM模型集合 - 向后兼容，使用默认参数
void initializeImmModels(TrackingSystem* system, double dt);

// 初始化模型转移概率矩阵 - 带自转移概率参数
void initializeModelTransitionMatrixWithProb(TrackingSystem* system, double selfTransProb);

// 初始化模型转移概率矩阵 - 向后兼容
void initializeModelTransitionMatrix(TrackingSystem* system);

// 从测量初始化一个新的IMM航迹
void initializeImmTrackFromMeasurement(TrackingSystem* system, Track* track, Measurement* z, int trackId);

// 释放IMM航迹资源
void freeImmTrack(Track* track);

/*          IMM核心算法函数         */ 
// 执行IMM交互过程
void immInteraction(TrackingSystem* system);

// 计算混合概率
void calculateMixingProbabilities(TrackingSystem* system, Track* track, double mixProb[MAX_MODELS][MAX_MODELS]);

// 计算混合状态和协方差（使用track内部的mixedStates和mixedCovariances）
void calculateMixedEstimates(TrackingSystem* system, Track* track, double mixProb[MAX_MODELS][MAX_MODELS]);

// 使用所有模型进行预测
void immModePrediction(TrackingSystem* system, Track* track);

// 使用所有模型进行更新
void immModeUpdate(TrackingSystem* system, Track* track, Measurement* z);

// 更新模型概率（使用track内部的modelLikelihoods）
void updateModelProbabilities(TrackingSystem* system, Track* track);

// 合并所有模型结果
void combineImmEstimates(TrackingSystem* system, Track* track);

// IMM接口函数
void immCycle(TrackingSystem* system, Track* track, Measurement* z);

// 检查IMM模型状态向量维度一致性
int checkModelDimensionConsistency(TrackingSystem* system);

// 模型评估功能 - 用于分析IMM性能
void evaluateImmPerformance(TrackingSystem* system, Track* track);

#endif /* IMM_H */
