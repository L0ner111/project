/*
 * imm.c - 交互多模型(IMM)算法完善实现
 */

#include "imm.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "config.h"

/* ===== 内部函数原型 ===== */
double calculateNormalizationFactor(double weights[], int count);
void initializeTrackIMM(Track *track, int stateDim);

/* ===== IMM初始化函数 ===== */
// 初始化默认IMM参数
void initializeDefaultImmParameters(ImmParameters *params)
{
    if (!params)
    {
        printf("Error: IMM parameters pointer is NULL\n");
        return;
    }

    // 设置默认参数值
    params->cvProcessNoise = 1.0;   // 匀速模型过程噪声强度
    params->caProcessNoise = 0.1;   // 匀加速模型过程噪声强度
    params->ctProcessNoise = 1.0;   // 协调转弯模型过程噪声强度
    params->measurementNoise = 1.0; // 测量噪声强度
    params->turnRate = 0.1;         // 协调转弯率(rad/s)
    params->selfTransProb = 0.95;   // 模型自转移概率
}

// 初始化IMM模型集合 - 带参数版本
void initializeImmModelsWithParams(TrackingSystem *system, double dt, ImmParameters *params)
{
    if (!system)
    {
        printf("Error: System pointer is NULL\n");
        return;
    }

    // 如果未提供参数，创建默认参数
    ImmParameters defaultParams;
    if (params == NULL)
    {
        initializeDefaultImmParameters(&defaultParams);
        params = &defaultParams;
    }

    // 设置使用的模型数量
    system->numModels = MAX_MODELS; // 使用三种模型: CV, CA, CT

    // 初始化各个模型
    double modelParams[1]; // 用于传递其他参数

    // 初始化匀速模型
    initializeKalmanModelDefault(&system->models[MODEL_CONSTANT_VELOCITY], MODEL_CONSTANT_VELOCITY, dt);

    // 初始化匀加速模型
    initializeKalmanModelDefault(&system->models[MODEL_CONSTANT_ACCELERATION], MODEL_CONSTANT_ACCELERATION, dt);

    // 初始化协调转弯模型 - 使用指定的转弯率
    modelParams[0] = params->turnRate;
    initializeKalmanModel(&system->models[MODEL_COORDINATED_TURN], MODEL_COORDINATED_TURN, dt, modelParams);

    // 初始化模型转移概率矩阵
    initializeModelTransitionMatrixWithProb(system, params->selfTransProb);

    printf("IMM model set initialized with %d models (Turn rate: %.3f rad/s)\n",
           system->numModels, params->turnRate);

    // 检查模型维度一致性
    if (!checkModelDimensionConsistency(system))
    {
        printf("Warning: Model dimension inconsistency detected. IMM may not function correctly.\n");
    }
    else
    {
        printf("Model dimension consistency check passed.\n");
    }
}

// 初始化IMM模型集合 - 兼容旧版本，使用默认参数
void initializeImmModels(TrackingSystem *system, double dt)
{
    initializeImmModelsWithParams(system, dt, NULL);
}

// 初始化模型转移概率矩阵 - 带自转移概率参数
void initializeModelTransitionMatrixWithProb(TrackingSystem *system, double selfTransProb)
{
    if (!system)
    {
        printf("Error: System pointer is NULL\n");
        return;
    }

    int n = system->numModels;

    // 计算转移到其他模型的平均概率
    double otherTransProb = (1.0 - selfTransProb) / (n - 1);

    // 设置转移概率矩阵
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (i == j)
            {
                // 自转移概率
                system->modelTransitionMatrix[i][j] = selfTransProb;
            }
            else
            {
                // 转移到其他模型的概率
                system->modelTransitionMatrix[i][j] = otherTransProb;
            }
        }
    }

    printf("Model transition matrix initialized (Self-transition prob: %.3f)\n", selfTransProb);
}

// 初始化模型转移概率矩阵 - 向后兼容
void initializeModelTransitionMatrix(TrackingSystem *system)
{
    initializeModelTransitionMatrixWithProb(system, 0.95);
}

// 初始化航迹的IMM相关状态
void initializeTrackIMM(Track *track, int stateDim)
{
    if (!track)
    {
        printf("Error: Track pointer is NULL\n");
        return;
    }

    // 初始化合并状态和协方差，对应矩阵初始化0
    track->x = createMatrix(stateDim, 1);
    track->P = createMatrix(stateDim, stateDim);

    // 初始化各模型的状态和协方差，对应矩阵初始化0
    for (int i = 0; i < MAX_MODELS; i++)
    {
        track->modelStates[i] = createMatrix(stateDim, 1);
        track->modelCovariances[i] = createMatrix(stateDim, stateDim);
        track->mixedStates[i] = createMatrix(stateDim, 1);
        track->mixedCovariances[i] = createMatrix(stateDim, stateDim);

        // 初始化模型概率为均匀分布
        track->modelProbabilities[i] = 1.0 / MAX_MODELS;
        track->modelLikelihoods[i] = 0.0;
    }
}

/* ===== IMM算法核心函数 ===== */

// 执行IMM交互过程 - IMM算法的主循环
void immInteraction(TrackingSystem *system)
{
    if (!system)
    {
        printf("Error: System pointer is NULL\n");
        return;
    }

    // 对每个航迹执行IMM算法
    for (int t = 0; t < system->numTracks; t++)
    {
        Track *track = &system->tracks[t];

        // 计算混合概率
        double mixProb[MAX_MODELS][MAX_MODELS];
        calculateMixingProbabilities(system, track, mixProb);

        // 计算混合状态和协方差
        calculateMixedEstimates(system, track, mixProb);
    }
}

// 计算混合概率,(模拟混合后各个模型预测结果占比的权重)
void calculateMixingProbabilities(TrackingSystem *system, Track *track, double mixProb[MAX_MODELS][MAX_MODELS])
{
    int n = system->numModels;

    // 计算预测模型概率 c_j
    double c[MAX_MODELS];
    for (int j = 0; j < n; j++)
    {
        c[j] = 0.0;
        for (int i = 0; i < n; i++)
        {
            c[j] += system->modelTransitionMatrix[i][j] * track->modelProbabilities[i];
        }
    }

    // 计算混合概率 μ_{i|j}
    for (int j = 0; j < n; j++)
    {
        if (c[j] < 1e-10)
        {
            // 防止除以0
            for (int i = 0; i < n; i++)
            {
                mixProb[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }
        else
        {
            for (int i = 0; i < n; i++)
            {
                mixProb[i][j] = system->modelTransitionMatrix[i][j] * track->modelProbabilities[i] / c[j];
            }
        }
    }
}

// 计算混合状态和协方差 - 直接使用track中的模型状态
void calculateMixedEstimates(TrackingSystem *system, Track *track, double mixProb[MAX_MODELS][MAX_MODELS])
{
    int n = system->numModels;
    int stateDim = track->x.rows; // 状态维度

    // 为每个模型计算混合状态和协方差
    for (int j = 0; j < n; j++)
    { // 这里j表示第j个运动模型
        // 初始化混合状态为0
        for (int d = 0; d < stateDim; d++)
        {
            setMatrixElement(&track->mixedStates[j], d, 0, 0.0); // 直接全部置零
        }

        // 加权平均各模型状态
        for (int i = 0; i < n; i++)
        {
            for (int d = 0; d < stateDim; d++)  // 状态向量中的每个元素进行基于元素级别的相同处理，
            {
                double oldVal = getMatrixElement(&track->mixedStates[j], d, 0);  // 逐个获取混合状态向量的元素
                double modelVal = getMatrixElement(&track->modelStates[i], d, 0);  // 逐个获取模型状态向量中的元素
                setMatrixElement(&track->mixedStates[j], d, 0, oldVal + mixProb[i][j] * modelVal);
            }
        }

        // 初始化混合协方差为0
        for (int r = 0; r < stateDim; r++)
        {
            for (int c = 0; c < stateDim; c++)
            {
                setMatrixElement(&track->mixedCovariances[j], r, c, 0.0);
            }
        }

        // 计算协方差第一部分 - 加权平均各模型协方差
        for (int i = 0; i < n; i++)
        {
            for (int r = 0; r < stateDim; r++)
            {
                for (int c = 0; c < stateDim; c++)
                {
                    double oldVal = getMatrixElement(&track->mixedCovariances[j], r, c);
                    double modelVal = getMatrixElement(&track->modelCovariances[i], r, c);
                    setMatrixElement(&track->mixedCovariances[j], r, c, oldVal + mixProb[i][j] * modelVal);
                }
            }
        }

        // 计算协方差第二部分 - 状态差的平方的加权平均
        for (int i = 0; i < n; i++)
        {
            // 计算状态差
            Matrix_2 stateDiff = createMatrix(stateDim, 1);
            for (int d = 0; d < stateDim; d++)
            {
                double mixedVal = getMatrixElement(&track->mixedStates[j], d, 0);
                double modelVal = getMatrixElement(&track->modelStates[i], d, 0);
                setMatrixElement(&stateDiff, d, 0, modelVal - mixedVal);
            }

            // 计算状态差的外积
            Matrix_2 diffOuter = outerProduct(&stateDiff, &stateDiff);

            // 将加权外积加到混合协方差上
            for (int r = 0; r < stateDim; r++)
            {
                for (int c = 0; c < stateDim; c++)
                {
                    double oldVal = getMatrixElement(&track->mixedCovariances[j], r, c);
                    double diffVal = getMatrixElement(&diffOuter, r, c);
                    setMatrixElement(&track->mixedCovariances[j], r, c, oldVal + mixProb[i][j] * diffVal);
                }
            }

            // 释放临时矩阵
            freeMatrix(&stateDiff);
            freeMatrix(&diffOuter);
        }
    }
}

// 使用所有模型进行预测
void immModePrediction(TrackingSystem *system, Track *track)
{
    int n = system->numModels;

    // 对每个模型执行预测步骤，使用混合后的状态和协方差
    for (int j = 0; j < n; j++)
    {
        KalmanModel *model = &system->models[j];

        // 将混合状态和协方差复制到模型状态
        matrixCopy(&track->modelStates[j], &track->mixedStates[j]);
        matrixCopy(&track->modelCovariances[j], &track->mixedCovariances[j]);

        // 使用模型进行预测
        predictKalman(model, &track->modelStates[j], &track->modelCovariances[j]);
    }
}

// 使用所有模型进行更新
void immModeUpdate(TrackingSystem *system, Track *track, Measurement *z)
{
    int n = system->numModels;

    // 对每个模型执行更新步骤
    for (int j = 0; j < n; j++)
    {
        KalmanModel *model = &system->models[j];

        // 计算模型似然度
        track->modelLikelihoods[j] = calculateLikelihood(model, &track->modelStates[j], &track->modelCovariances[j], z);

        // 使用模型进行更新
        updateKalman(model, &track->modelStates[j], &track->modelCovariances[j], z);
    }

    // 更新模型概率
    updateModelProbabilities(system, track);

    // 合并各模型状态和协方差
    combineImmEstimates(system, track);
}

// 更新模型概率
void updateModelProbabilities(TrackingSystem *system, Track *track)
{
    int n = system->numModels;
    double newProbs[MAX_MODELS];

    // 找到最大似然度以提高数值稳定性
    double maxLikelihood = -1.0;
    for (int j = 0; j < n; j++)
    {
        if (track->modelLikelihoods[j] > maxLikelihood)
        {
            maxLikelihood = track->modelLikelihoods[j];
        }
    }

    // 如果所有似然度都太小，重置为均匀分布
    if (maxLikelihood < 1e-10)
    {
        for (int j = 0; j < n; j++)
        {
            track->modelProbabilities[j] = 1.0 / n;
        }

        printf("Model probabilities reset to uniform distribution (low likelihoods)\n");
        return;
    }

    // 计算归一化常数
    double c = 0.0;
    for (int j = 0; j < n; j++)
    {
        double predProb = 0.0;
        for (int i = 0; i < n; i++)
        {
            predProb += system->modelTransitionMatrix[i][j] * track->modelProbabilities[i];
        }

        // 更新模型概率（使用对数似然差来提高数值稳定性）
        newProbs[j] = track->modelLikelihoods[j] / maxLikelihood * predProb;

        // 累加归一化常数
        c += newProbs[j];
    }

    // 归一化模型概率
    if (c > 1e-10)
    {
        for (int j = 0; j < n; j++)
        {
            track->modelProbabilities[j] = newProbs[j] / c;
        }
    }
    else
    {
        // 如果归一化常数太小，重置为均匀分布
        for (int j = 0; j < n; j++)
        {
            track->modelProbabilities[j] = 1.0 / n;
        }
        printf("Model probabilities reset to uniform distribution (normalization issue)\n");
    }

    // 打印模型概率用于调试
    printf("Model probabilities: ");
    for (int j = 0; j < n; j++)
    {
        printf("%.3f ", track->modelProbabilities[j]);
    }
    printf("\n");
}

// 合并所有模型结果
void combineImmEstimates(TrackingSystem *system, Track *track)
{
    int n = system->numModels;
    int stateDim = track->x.rows;

    // 初始化合并状态为0
    for (int d = 0; d < stateDim; d++)
    {
        setMatrixElement(&track->x, d, 0, 0.0);
    }

    // 计算合并状态 - 各模型状态的加权平均
    for (int j = 0; j < n; j++)
    {
        for (int d = 0; d < stateDim; d++)
        {
            double oldVal = getMatrixElement(&track->x, d, 0);
            double modelVal = getMatrixElement(&track->modelStates[j], d, 0);
            setMatrixElement(&track->x, d, 0, oldVal + track->modelProbabilities[j] * modelVal);
        }
    }

    // 初始化合并协方差为0
    for (int r = 0; r < stateDim; r++)
    {
        for (int c = 0; c < stateDim; c++)
        {
            setMatrixElement(&track->P, r, c, 0.0);
        }
    }

    // 计算协方差第一部分 - 加权平均各模型协方差
    for (int j = 0; j < n; j++)
    {
        for (int r = 0; r < stateDim; r++)
        {
            for (int c = 0; c < stateDim; c++)
            {
                double oldVal = getMatrixElement(&track->P, r, c);
                double modelVal = getMatrixElement(&track->modelCovariances[j], r, c);
                setMatrixElement(&track->P, r, c, oldVal + track->modelProbabilities[j] * modelVal);
            }
        }
    }

    // 计算协方差第二部分 - 状态差的平方的加权平均
    for (int j = 0; j < n; j++)
    {
        // 计算状态差
        Matrix_2 stateDiff = createMatrix(stateDim, 1);
        for (int d = 0; d < stateDim; d++)
        {
            double combVal = getMatrixElement(&track->x, d, 0);
            double modelVal = getMatrixElement(&track->modelStates[j], d, 0);
            setMatrixElement(&stateDiff, d, 0, modelVal - combVal);
        }

        // 计算状态差的外积
        Matrix_2 diffOuter = outerProduct(&stateDiff, &stateDiff);

        // 将加权外积加到合并协方差上
        for (int r = 0; r < stateDim; r++)
        {
            for (int c = 0; c < stateDim; c++)
            {
                double oldVal = getMatrixElement(&track->P, r, c);
                double diffVal = getMatrixElement(&diffOuter, r, c);
                setMatrixElement(&track->P, r, c, oldVal + track->modelProbabilities[j] * diffVal);
            }
        }

        // 释放临时矩阵
        freeMatrix(&stateDiff);
        freeMatrix(&diffOuter);
    }
}

/* ===== 工具函数 ===== */

// 计算归一化因子
double calculateNormalizationFactor(double weights[], int count)
{
    double sum = 0.0;
    for (int i = 0; i < count; i++)
    {
        sum += weights[i];
    }
    return sum;
}

/* ===== IMM接口函数 ===== */

// 执行完整的IMM周期
void immCycle(TrackingSystem *system, Track *track, Measurement *z)
{
    if (!system || !track)
    {
        printf("Error: Invalid parameters for IMM cycle\n");
        return;
    }

    printf("IMM cycle for track %d (time: %d)\n",
           track->id, system->currentTime);

    // 1. 计算混合概率
    double mixProb[MAX_MODELS][MAX_MODELS];
    calculateMixingProbabilities(system, track, mixProb);

    // 2. 计算混合状态和协方差
    calculateMixedEstimates(system, track, mixProb);

    // 3. 使用所有模型进行预测
    immModePrediction(system, track);

    // 4. 使用所有模型进行更新（如果有测量）
    if (z != NULL)
    {
        // 打印测量信息
        printf("Processing measurement: (%.2f, %.2f, %.2f) at time %.2f\n",
               z->position.x, z->position.y, z->position.z, z->time);

        // 更新航迹的最后更新时间
        track->lastUpdateTime = z->time;
        track->numMisses = 0; // 重置连续丢失计数

        immModeUpdate(system, track, z);
    }
    else
    {
        // 没有关联测量，仅使用预测结果，增加丢失计数
        track->numMisses++;
        printf("No measurement associated with track %d (misses: %d)\n",
               track->id, track->numMisses);
        
        // 更新模型概率
        updateModelProbabilities(system, track);
        // 没有关联的量测，使用预测结果直接合并各模型状态和协方差
        combineImmEstimates(system, track);
    }
}

// 从测量初始化一个新的IMM航迹
void initializeImmTrackFromMeasurement(TrackingSystem *system, Track *track, Measurement *z, int trackId)
{
    if (!system || !track || !z)
    {
        printf("Error: Invalid parameters for track initialization\n");
        return;
    }

    // 获取状态向量维度
    int stateDim = STATE_DIM;

    // 初始化IMM相关的矩阵
    // 即创建相应矩阵并置为0，后续用第一个量测赋值，得到一条轨迹的起点
    initializeTrackIMM(track, stateDim);

    // 设置基本航迹信息
    track->id = trackId;
    track->birthTime = z->time;
    track->lastUpdateTime = z->time;
    track->numMisses = 0;

    // 初始位置（从测量中获取）
    for (int m = 0; m < system->numModels; m++)
    {
        // 设置位置
        setMatrixElement(&track->modelStates[m], 0, 0, z->position.x);
        setMatrixElement(&track->modelStates[m], 1, 0, z->position.y);
        setMatrixElement(&track->modelStates[m], 2, 0, z->position.z);

        // 初始速度和加速度设为0
        for (int i = 3; i < stateDim; i++)
        {
            setMatrixElement(&track->modelStates[m], i, 0, 0.0);
        }

        // 初始协方差
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                setMatrixElement(&track->modelCovariances[m], i, j, (i == j) ? 1.0 : 0.0);
            }
        }

        // 速度和加速度协方差（较大，表示不确定性）
        for (int i = 3; i < stateDim; i++)
        {
            setMatrixElement(&track->modelCovariances[m], i, i, 100.0);
        }
    }

    // 初始合并状态等于第一个模型的状态
    matrixCopy(&track->x, &track->modelStates[0]);
    matrixCopy(&track->P, &track->modelCovariances[0]);
}

// 释放IMM航迹资源
void freeImmTrack(Track *track)
{
    if (!track)
    {
        return;
    }

    // 释放合并状态和协方差
    freeMatrix(&track->x);
    freeMatrix(&track->P);

    // 释放各模型的状态和协方差
    for (int i = 0; i < MAX_MODELS; i++)
    {
        freeMatrix(&track->modelStates[i]);
        freeMatrix(&track->modelCovariances[i]);
        freeMatrix(&track->mixedStates[i]);
        freeMatrix(&track->mixedCovariances[i]);
    }
}

/* ===== 模型状态检查函数 ===== */

// 检查IMM模型状态向量维度一致性（应加在初始化代码中）
int checkModelDimensionConsistency(TrackingSystem *system)
{
    if (!system || system->numModels <= 0)
    {
        printf("Error: Invalid system or no models defined\n");
        return 0;
    }

    int refDim = system->models[0].F.rows;

    for (int i = 1; i < system->numModels; i++)
    {
        if (system->models[i].F.rows != refDim)
        {
            printf("Warning: Models have different state dimensions (%d vs %d)\n",
                   refDim, system->models[i].F.rows);
            return 0;
        }
    }

    return 1; // 所有模型维度一致
}
