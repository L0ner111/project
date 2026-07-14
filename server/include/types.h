#ifndef TYPES_H
#define TYPES_H

#include <stdio.h>
#include <math.h>
#include "config.h"

/* ===== 相关结构体定义 ===== */

// 三维向量
typedef struct {
    double x;
    double y;
    double z;
} Vector3D;

// 极坐标结构
typedef struct
{
    double distance;
    double azimuth;  // 方位角
    double elevation;  // 俯仰角
} PolarCoord3D;

// 矩阵结构
typedef struct {
    int rows;    // 行数
    int cols;    // 列数
    double* data; // 扁平化数组（行优先）
} Matrix_2;

// 测量结构
typedef struct {
    Vector3D position; // 位置测量
    double time;       // 时间戳
    int id;            // 测量ID
} Measurement;

// 卡尔曼滤波模型
typedef struct {
    Matrix_2 F;      // 状态转移矩阵
    Matrix_2 H;      // 测量矩阵
    Matrix_2 Q;      // 过程噪声协方差
    Matrix_2 R;      // 测量噪声协方差
    double dt;     // 时间步长
} KalmanModel;

// 航迹状态
typedef struct {
    // 基本航迹信息
    int id;                // 唯一航迹ID
    double birthTime;         // 创建时间
    double lastUpdateTime;    // 上次更新时间
    int numMisses;         // 连续丢失次数
    
    // IMM相关状态
    Matrix_2 x;                  // 合并后的状态向量，即最终估计
    Matrix_2 P;                  // 合并后的状态协方差，即最终估计
    
    // 各模型的特定状态和协方差
    Matrix_2 modelStates[MAX_MODELS];      // 每个模型的状态向量
    Matrix_2 modelCovariances[MAX_MODELS]; // 每个模型的协方差矩阵
    
    // 用于IMM混合的临时状态和协方差
    Matrix_2 mixedStates[MAX_MODELS];      // 每个模型的混合状态
    Matrix_2 mixedCovariances[MAX_MODELS]; // 每个模型的混合协方差
    
    double modelProbabilities[MAX_MODELS]; // 每个模型的概率
    
    double modelLikelihoods[MAX_MODELS];  // 每个模型的似然度
} Track;


// 全局数据结构
typedef struct {
    // 测量数据
    Measurement measurements[MAX_MEASUREMENTS];
    int numMeasurements;
    
    // 航迹
    Track tracks[MAX_TRACKS];
    int numTracks;
    int nextTrackId;
    
    // IMM模型
    KalmanModel models[MAX_MODELS];
    int numModels;
    double modelTransitionMatrix[MAX_MODELS][MAX_MODELS]; // 模型转移概率
    
    // 算法参数
    double detectionProbability; // 检测概率
    double gatingThreshold;      // 门限阈值
    

    int currentTime;  // 当前时间
} TrackingSystem;

#endif /* TYPES_H */

