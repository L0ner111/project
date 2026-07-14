// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <math.h>
// #include "imm.h"
// #include "kalman.h"
// #include "matrix.h"
// #include "types.h"
// #include "config.h"
// #include "tool.h"
// #include "ici.h"
// #define MAX_LINE_LENGTH 256

// // 从状态向量提取位置和速度信息
// void extractStateInfo(Matrix_2* x, Vector3D* position, Vector3D* velocity) {
//     position->x = getMatrixElement(x, 0, 0);
//     position->y = getMatrixElement(x, 1, 0);
//     position->z = getMatrixElement(x, 2, 0);
    
//     velocity->x = getMatrixElement(x, 3, 0);
//     velocity->y = getMatrixElement(x, 4, 0);
//     velocity->z = getMatrixElement(x, 5, 0);
// }

// // 计算欧氏距离
// double calculateDistance(Vector3D* predicted, Vector3D* measured) {
//     double dx = predicted->x - measured->x;
//     double dy = predicted->y - measured->y;
//     double dz = predicted->z - measured->z;
    
//     return sqrt(dx*dx + dy*dy + dz*dz);
// }

// // 计算动态门限阈值
// double compute_gate_threshold(double x, double y, double z, double theta) {
//     // 计算径向距离
//     double r = sqrt(x * x + y * y + z * z);
//     double gateThreshold = r * theta * 1.2;
//     return gateThreshold;
// }

// // 读取双传感器量测数据
// int readMeasurements(const char* filename, 
//                      Measurement* measurements1, double* ranges1,
//                      Measurement* measurements2, double* ranges2,
//                      int maxMeasurements) {
//     FILE* file = fopen(filename, "r");
//     if (!file) {
//         printf("Error: Could not open file %s\n", filename);
//         return 0;
//     }

//     char line[MAX_LINE_LENGTH];
//     int count = 0;
    
//     while (count < maxMeasurements && fgets(line, MAX_LINE_LENGTH, file)) {
//         double time, x1, y1, z1, az1, el1, range1, x2, y2, z2, az2, el2, range2;
        
//         // 解析行数据 - 13个字段 (时间 + 2*6个传感器数据)
//         if (sscanf(line, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", 
//                    &time, 
//                    &x1, &y1, &z1, &az1, &el1, &range1, 
//                    &x2, &y2, &z2, &az2, &el2, &range2) == 13) {
            
//             // 传感器1数据
//             measurements1[count].time = time;
//             measurements1[count].position.x = x1;
//             measurements1[count].position.y = y1;
//             measurements1[count].position.z = z1;
//             measurements1[count].id = count;
//             ranges1[count] = range1;
            
//             // 传感器2数据
//             measurements2[count].time = time;
//             measurements2[count].position.x = x2;
//             measurements2[count].position.y = y2;
//             measurements2[count].position.z = z2;
//             measurements2[count].id = count;
//             ranges2[count] = range2;
            
           
//         }
//         count++;
//     }
    
//     fclose(file);
//     return count;
// }

// // 保存预测结果到文件 - 添加更多信息
// void savePredictions(const char* filename, double* times, 
//                      Vector3D* positions1, Vector3D* velocities1, double* gateThresholds1,
//                      Vector3D* positions2, Vector3D* velocities2, double* gateThresholds2,
//                      Vector3D* fusedPositions, Vector3D* fusedVelocities, double* fusedGateThresholds,
//                      double* omegas, int count) {
//     FILE* file = fopen(filename, "w");
//     if (!file) {
//         printf("Error: Could not open output file %s\n", filename);
//         return;
//     }
    
//     // 写入标题行
//     fprintf(file, "Time,X1,Y1,Z1,VX1,VY1,VZ1,Gate1,X2,Y2,Z2,VX2,VY2,VZ2,Gate2,XF,YF,ZF,VXF,VYF,VZF,GateF,Omega\n");
    
//     for (int i = 0; i < count; i++) {
//         fprintf(file, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
//                 times[i],
//                 // 传感器1
//                 positions1[i].x, positions1[i].y, positions1[i].z,
//                 velocities1[i].x, velocities1[i].y, velocities1[i].z,
//                 gateThresholds1[i],
//                 // 传感器2
//                 positions2[i].x, positions2[i].y, positions2[i].z,
//                 velocities2[i].x, velocities2[i].y, velocities2[i].z,
//                 gateThresholds2[i],
//                 // 融合结果
//                 fusedPositions[i].x, fusedPositions[i].y, fusedPositions[i].z,
//                 fusedVelocities[i].x, fusedVelocities[i].y, fusedVelocities[i].z,
//                 fusedGateThresholds[i],
//                 // ICI融合权重
//                 omegas[i]);
//     }
    
//     fclose(file);
//     printf("Predictions saved to %s\n", filename);
// }


// void run(const char* inputFile, const char* outputFile) {
//     // 读取量测数据和径向距离
//     Measurement measurements1[1500]; // 传感器1，假设最多1500个量测点
//     Measurement measurements2[1500]; // 传感器2
//     double ranges1[1500];  // 传感器1径向距离
//     double ranges2[1500];  // 传感器2径向距离
    
//     int numMeasurements = readMeasurements(inputFile, measurements1, ranges1, measurements2, ranges2, 1500);
//     if (numMeasurements == 0) {
//         printf("No measurements read from file.\n");
//         return ;
//     }
    
//     printf("Read %d measurements from file.\n", numMeasurements);
    
//     // 初始化两个跟踪系统
//     TrackingSystem system1, system2;
//     memset(&system1, 0, sizeof(TrackingSystem));
//     memset(&system2, 0, sizeof(TrackingSystem));
    
//     // 计算采样间隔
//     double dt = 0.0;
//     if (numMeasurements >= 2) {
//         dt = measurements1[1].time - measurements1[0].time;
//     } else {
//         dt = 0.02; // 默认值
//     }
    
//     printf("Using sampling interval dt = %.3f\n", dt);
    
//     // 设置IMM参数
//     ImmParameters immParams;
//     initializeDefaultImmParameters(&immParams);
//     immParams.cvProcessNoise = 5.0;   // 调整过程噪声参数
//     immParams.caProcessNoise = 1.0;
//     immParams.ctProcessNoise = 5.0;
//     immParams.measurementNoise = 10.0;
//     immParams.turnRate = 0.1;         // 角速度，单位为rad/s
//     immParams.selfTransProb = 0.95;   // 模型自转移概率
    
//     // 初始化传感器1的IMM模型
//     initializeImmModelsWithParams(&system1, dt, &immParams);
//     // 初始化传感器2的IMM模型
//     initializeImmModelsWithParams(&system2, dt, &immParams);
    
//     // 初始化跟踪
//     system1.numTracks = 1; // 只跟踪一个目标
//     system1.nextTrackId = 1;
//     system2.numTracks = 1;
//     system2.nextTrackId = 1;
    
//     // 使用第一个测量初始化航迹
//     initializeImmTrackFromMeasurement(&system1, &system1.tracks[0], &measurements1[0], 0);
//     initializeImmTrackFromMeasurement(&system2, &system2.tracks[0], &measurements2[0], 0);
    
//     // 存储预测结果
//     double times[1500];
//     Vector3D predictedPositions1[1500];
//     Vector3D predictedVelocities1[1500];
//     double gateThresholds1[1500];
//     Vector3D predictedPositions2[1500];
//     Vector3D predictedVelocities2[1500];
//     double gateThresholds2[1500];
//     Vector3D fusedPositions[1500];
//     Vector3D fusedVelocities[1500];
//     double fusedGateThresholds[1500];
//     double omegas[1500];
//     int resultCount = 0;
    
//     // 存储第一个结果
//     times[resultCount] = measurements1[0].time;
    
//     // 提取传感器1的状态
//     extractStateInfo(&system1.tracks[0].x, &predictedPositions1[resultCount], &predictedVelocities1[resultCount]);
//     gateThresholds1[resultCount] = ranges1[0] * 0.00157 * 1.2; // 1.57毫弧度
    
//     // 提取传感器2的状态
//     extractStateInfo(&system2.tracks[0].x, &predictedPositions2[resultCount], &predictedVelocities2[resultCount]);
//     gateThresholds2[resultCount] = ranges2[0] * 0.00167 * 1.2; // 1.67毫弧度
    
//     // 使用ICI算法融合第一个测量点的状态估计
//     Matrix_2 fusedState = createMatrix(STATE_DIM, 1); // 融合后的状态向量
//     Matrix_2 fusedCovariance = createMatrix(STATE_DIM, STATE_DIM); // 融合后的协方差矩阵
//     double omega; // ICI算法的融合权重
    
//     // ICI融合
//     ICI(&fusedState, &fusedCovariance, &omega, 
//         &system1.tracks[0].x, &system1.tracks[0].P, 
//         &system2.tracks[0].x, &system2.tracks[0].P);
    
//     printf("ICI fusion for first measurement, omega = %.3f\n", omega);
    
//     // 提取融合后的状态
//     extractStateInfo(&fusedState, &fusedPositions[resultCount], &fusedVelocities[resultCount]);
//     fusedGateThresholds[resultCount] = (gateThresholds1[resultCount] + gateThresholds2[resultCount]) / 2.0;
//     omegas[resultCount] = omega;
//     resultCount++;
    
//     // 释放临时矩阵
//     freeMatrix(&fusedState);
//     freeMatrix(&fusedCovariance);
    
//     // 处理后续量测
//     for (int i = 1; i < numMeasurements; i++) {
//         // for (int i = 1; i < 500; i++) {
//         Measurement* currentMeasurement1 = &measurements1[i];
//         Measurement* currentMeasurement2 = &measurements2[i];
//         Track* track1 = &system1.tracks[0];
//         Track* track2 = &system2.tracks[0];
        
//         // === 传感器1处理 ===
//         // 执行IMM循环的预测部分
//         double mixProb1[MAX_MODELS][MAX_MODELS];
//         calculateMixingProbabilities(&system1, track1, mixProb1);
//         calculateMixedEstimates(&system1, track1, mixProb1);
//         immModePrediction(&system1, track1);
        
//         // 获取预测的位置
//         Vector3D predictedPosition1;
//         Vector3D predictedVelocity1;
//         extractStateInfo(&track1->x, &predictedPosition1, &predictedVelocity1);
        
//         // 计算动态阈门
//         double gateThreshold1 = compute_gate_threshold(
//             predictedPosition1.x, predictedPosition1.y, predictedPosition1.z, 0.00167); // 1.57毫弧度
        
//         // 计算距离
//         double distance1 = calculateDistance(&predictedPosition1, &currentMeasurement1->position);
        
//         // 应用阈门逻辑
//         if (distance1 <= gateThreshold1) {
//             // 量测在阈门内，执行更新
//             immModeUpdate(&system1, track1, currentMeasurement1);
//             printf("Sensor 1, Measurement %d accepted: distance=%.2f, threshold=%.2f\n", 
//                    i, distance1, gateThreshold1);
//         } else {
//             // 量测在阈门外，仅使用预测结果
//             updateModelProbabilities(&system1, track1);
//             combineImmEstimates(&system1, track1);
//             printf("Sensor 1, Measurement %d rejected: distance=%.2f, threshold=%.2f\n", 
//                    i, distance1, gateThreshold1);
//         }
        
//         // 保存传感器1的结果
//         extractStateInfo(&track1->x, &predictedPositions1[resultCount], &predictedVelocities1[resultCount]);
//         gateThresholds1[resultCount] = gateThreshold1;
        
//         // === 传感器2处理 ===
//         // 执行IMM循环的预测部分
//         double mixProb2[MAX_MODELS][MAX_MODELS];
//         calculateMixingProbabilities(&system2, track2, mixProb2);
//         calculateMixedEstimates(&system2, track2, mixProb2);
//         immModePrediction(&system2, track2);
        
//         // 获取预测的位置
//         Vector3D predictedPosition2;
//         Vector3D predictedVelocity2;
//         extractStateInfo(&track2->x, &predictedPosition2, &predictedVelocity2);
        
//         // 计算动态阈门
//         double gateThreshold2 = compute_gate_threshold(
//             predictedPosition2.x, predictedPosition2.y, predictedPosition2.z, 0.00157); // 1.57毫弧度
        
//         // 计算距离
//         double distance2 = calculateDistance(&predictedPosition2, &currentMeasurement2->position);
        
//         // 应用阈门逻辑
//         if (distance2 <= gateThreshold2) {
//             // 量测在阈门内，执行更新
//             immModeUpdate(&system2, track2, currentMeasurement2);
//             printf("Sensor 2, Measurement %d accepted: distance=%.2f, threshold=%.2f\n", 
//                    i, distance2, gateThreshold2);
//         } else {
//             // 量测在阈门外，仅使用预测结果
//             updateModelProbabilities(&system2, track2);
//             combineImmEstimates(&system2, track2);
//             printf("Sensor 2, Measurement %d rejected: distance=%.2f, threshold=%.2f\n", 
//                    i, distance2, gateThreshold2);
//         }
        
//         // 保存传感器2的结果
//         extractStateInfo(&track2->x, &predictedPositions2[resultCount], &predictedVelocities2[resultCount]);
//         gateThresholds2[resultCount] = gateThreshold2;
        
//         // === 使用ICI算法融合两个传感器的状态估计 ===
//         Matrix_2 fusedState = createMatrix(STATE_DIM, 1);
//         Matrix_2 fusedCovariance = createMatrix(STATE_DIM, STATE_DIM);
//         double omega;
        
//         ICI(&fusedState, &fusedCovariance, &omega, 
//             &track1->x, &track1->P, 
//             &track2->x, &track2->P);
        
//         printf("ICI fusion for measurement %d, omega = %.3f\n", i, omega);
        
//         // 保存融合结果
//         times[resultCount] = currentMeasurement1->time;
//         extractStateInfo(&fusedState, &fusedPositions[resultCount], &fusedVelocities[resultCount]);
//         fusedGateThresholds[resultCount] = (gateThreshold1 + gateThreshold2) / 2.0;
//         omegas[resultCount] = omega;
//         resultCount++;
        
//         // 释放临时矩阵
//         freeMatrix(&fusedState);
//         freeMatrix(&fusedCovariance);
//     }
    
//     // 保存预测结果
//     savePredictions(outputFile, times, 
//                     predictedPositions1, predictedVelocities1, gateThresholds1,
//                     predictedPositions2, predictedVelocities2, gateThresholds2,
//                     fusedPositions, fusedVelocities, fusedGateThresholds,
//                     omegas, resultCount);
    
//     // 释放资源
//     freeImmTrack(&system1.tracks[0]);
//     freeImmTrack(&system2.tracks[0]);

// }


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "imm.h"
#include "kalman.h"
#include "matrix2.h"
#include "types.h"
#include "config.h"
#include "tool.h"
#include "ici.h"
#include "weight_fusion.h"
#define MAX_LINE_LENGTH 256

#include <unistd.h>

void deal(int t, double* a1, double *a2) {
    switch (t)
    {
        case 1:
            *a1 *= 10;
            *a2 *= 2;
            break;
        case 2:
            *a1 *= 2;
            *a2 *= 10;
            break;
        case 3:
            *a1 *= 2;
            *a2 *= 10;
            break;
        case 4:
            *a1 /= 1;
            *a2 *= 10;
            break;
        case 5:
            *a2 *= 10;
            break;
        case 6:
            *a1 *= 2;
            *a2 *= 10;
            break;
        default:
            break;
    }
}

double max(double a, double b)
{
    if (a > b)
        return a;
    else
        return b;
}

// 通过角度误差是否过大，判断是否需要转变成上一时刻的结果
int check_is_transform(Vector3D m1, Vector3D m2)
{
    double x1 = m1.x, y1 = m1.y, z1 = m1.z;
    double x2 = m2.x, y2 = m2.y, z2 = m2.z;

    double a1 = atan2(y1, x1), a2 = atan2(y2, x2);
    double e1 = asin(z1 / sqrt(x1 * x1 + y1 * y1 + z1 * z1)), e2 = asin(z2 / sqrt(x2 * x2 + y2 * y2 + z2 * z2));

    printf("a1 = %lf, a2 = %lf\n", a1, a2);
    printf("e1 = %lf, e2 = %lf\n", e1, e2);

    printf("max = %lf\n", max(fabs(a1 - a2), fabs(e1 - e2)));

    if (max(fabs(a1 - a2), fabs(e1 - e2)) >= 0.0005)
        return 1;
    else
        return 0;
}

// 从状态向量提取位置和速度信息
void extractStateInfo(Matrix_2* x, Vector3D* position, Vector3D* velocity) {
    position->x = getMatrixElement(x, 0, 0);
    position->y = getMatrixElement(x, 1, 0);
    position->z = getMatrixElement(x, 2, 0);
    velocity->x = getMatrixElement(x, 3, 0);
    velocity->y = getMatrixElement(x, 4, 0);
    velocity->z = getMatrixElement(x, 5, 0);
}

// 计算欧氏距离
double calculateDistance(Vector3D* predicted, Vector3D* measured) {
    double dx = predicted->x - measured->x;
    double dy = predicted->y - measured->y;
    double dz = predicted->z - measured->z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

// 计算动态门限阈值
double compute_gate_threshold(double x, double y, double z, double theta) {
    double r = sqrt(x * x + y * y + z * z);
    return r * theta * 1.2;
}

// 逐行读取双传感器量测数据
int readOneMeasurement(FILE *file, Measurement *measurement1, Measurement *measurement2)
{
    char line[MAX_LINE_LENGTH];
    if (!fgets(line, MAX_LINE_LENGTH, file))
    {
        return 0;
    }

    // double time, x1, y1, z1, x2, y2, z2;
    double time, x1, y1, z1, x2, y2, z2;
    double a1, e1, r1, a2, e2, r2;
    // if (sscanf(line, "%lf %lf %lf %lf %lf %lf %lf",
    //            &time, &x1, &y1, &z1, &x2, &y2, &z2) != 7)
    if (sscanf(line, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
               &time, &x1, &y1, &z1, &a1, &e1, &r1, &x2, &y2, &z2, &a2, &e2, &r2) != 13)
    {

        return 0;
    }

    measurement1->time = time;
    measurement1->position.x = x1;
    measurement1->position.y = y1;
    measurement1->position.z = z1;
    measurement1->id = 0;

    measurement2->time = time;
    measurement2->position.x = x2;
    measurement2->position.y = y2;
    measurement2->position.z = z2;
    measurement2->id = 0;

    return 1;
}

// 保存单行预测结果到文件（追加模式）
void save_LS(const char *filename, double time, PolarCoord3D *sensor1, PolarCoord3D *sensor2, PolarCoord3D *sensor_fusion, int isFirst)
{
    FILE *file = fopen(filename, isFirst ? "w" : "a");
    if (!file)
    {
        printf("Error: Could not open output file %s\n", filename);
        return;
    }

    if (isFirst)
    {
        fprintf(file, "Time r1 a1 e1 r2 a2 e2 r_f a_f e_f\n");
    }

    fprintf(file, "%.3f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",
            time, sensor1->distance, sensor1->azimuth, sensor1->elevation,
            sensor2->distance, sensor2->azimuth, sensor2->elevation,
            sensor_fusion->distance, sensor_fusion->azimuth, sensor_fusion->elevation);

    fclose(file);
}

// 保存单行预测结果到文件（追加模式）
void savePrediction(const char *filename, double time, Vector3D *position1, Vector3D *position2, int isFirst)
{
    FILE *file = fopen(filename, isFirst ? "w" : "a");
    if (!file)
    {
        printf("Error: Could not open output file %s\n", filename);
        return;
    }

    if (isFirst)
    {
        fprintf(file, "Time X1 Y1 Z1 X2 Y2 Z2\n");
    }

    fprintf(file, "%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
            time,
            position1->x, position1->y, position1->z,
            position2->x, position2->y, position2->z);

    fclose(file);
}



// 系统对量测进行处理
void predOnce(TrackingSystem* system, Measurement* meas) {
    Track* track1 = &system->tracks[0];
            
    double mixProb1[MAX_MODELS][MAX_MODELS];
    calculateMixingProbabilities(system, track1, mixProb1);
    calculateMixedEstimates(system, track1, mixProb1);
    immModePrediction(system, track1);

    Vector3D predPos1, predVel1;
    extractStateInfo(&track1->x, &predPos1, &predVel1);
    double gate1 = compute_gate_threshold(predPos1.x, predPos1.y, predPos1.z, 0.00157);
    double dist1 = calculateDistance(&predPos1, &meas->position);

    if (dist1 <= gate1) {
        immModeUpdate(system, track1, meas);
    } else {
        updateModelProbabilities(system, track1);
        combineImmEstimates(system, track1);
    }

}

// 将结果写入文件中
void writeResultsToFile(const char* outputFile, char** lines, int count) {
    FILE* output = fopen(outputFile, "a"); // 追加模式
    if (!output) {
        printf("Error: Could not open output file %s\n", outputFile);
        return;
    }
    for (int i = 0; i < count; i++) {
        fputs(lines[i], output);
        free(lines[i]);
    }
    fclose(output);
}

// void run(const char *inputFile, const char *outputFile, const char *predFile, int ganrao_type)
void run(const char *inputFile, const char *outputFile, const char *predFile)
{
    
    // 打开输入文件
    FILE *file = fopen(inputFile, "r");
    if (!file)
    {
        printf("Error: Could not open file %s\n", inputFile);
        return;
    }

    // 初始化两个跟踪系统
    TrackingSystem system1, system2;
    memset(&system1, 0, sizeof(TrackingSystem));
    memset(&system2, 0, sizeof(TrackingSystem));

    // 设置 IMM 参数
    ImmParameters immParams;
    initializeDefaultImmParameters(&immParams);
    // immParams.cvProcessNoise = 5.0;
    // immParams.caProcessNoise = 1.0;
    // immParams.ctProcessNoise = 5.0;
    immParams.cvProcessNoise = 100.0;
    immParams.caProcessNoise = 0.1;
    immParams.ctProcessNoise = 1.0;
    immParams.measurementNoise = 100.0;
    immParams.turnRate = 0.1;
    immParams.selfTransProb = 0.95;

    // 初始化传感器1和传感器2的 IMM 模型
    double dt = 0.02;
    initializeImmModelsWithParams(&system1, dt, &immParams);
    initializeImmModelsWithParams(&system2, dt, &immParams);

    // 初始化跟踪
    system1.numTracks = 1;
    system1.nextTrackId = 1;
    system2.numTracks = 1;
    system2.nextTrackId = 1;

    // 缓冲区结构：最小二乘法的估计结果（球坐标）
    typedef struct
    {
        double time;
        Vector3D position1, position2;
        int isFirst;
    } BatchData_Pred;

    typedef struct
    {
        double time;
        PolarCoord3D sensor1, sensor2, sensor_fusion;
        int isFirst;
    } BatchData_LS;

    BatchData_Pred batch_pred[BATCH_SIZE];  // 存放航迹关联的估计结果
    BatchData_LS batch_LS[BATCH_SIZE];  // 存放最小二乘法的估计结果和权重融合结果
    int batchIndex = 0;                 // 缓冲区下标
    double lastTime = -1.0;
    int firstMeasurement = 1;           // 判断是否是第一个量测数据
    int measurementCount = 0;           // 记录测量集的数量

    int cnt = 0; // 记录次数

    // 单位矩阵
    Matrix_2 H = createIdentityMatrix(MEAS_DIM);      
    // 二乘法估计的传感器1的协方差矩阵
    Matrix_2 P1 = createMatrix(MEAS_DIM, MEAS_DIM);   
    setMatrixElement(&P1, 0, 0, 100);
    setMatrixElement(&P1, 1, 1, 2);
    setMatrixElement(&P1, 2, 2, 2);

    // 二乘法估计的传感器2的协方差矩阵
    Matrix_2 P2 = createMatrix(MEAS_DIM, MEAS_DIM);   
    setMatrixElement(&P2, 0, 0, 100);
    setMatrixElement(&P2, 1, 1, 2);
    setMatrixElement(&P2, 2, 2, 2);

    int queren1 = 0;
    int queren2 = 0;
    // 用于记录两个量测数据
    Measurement measurement1, measurement2;
    double range1 = 0.0, range2 = 0.0; // 初始化以避免未定义行为
    // readOneMeasurement(file, &measurement1, &measurement2);  // 有表头，先读取一次
    while (readOneMeasurement(file, &measurement1, &measurement2))
    {
        cnt++;
        // 更新采样间隔 dt
        if (lastTime >= 0)
        {
            dt = measurement1.time - lastTime;
            if (fabs(dt - 0.02) > 1e-6)
            {
                freeImmTrack(&system1.tracks[0]);
                freeImmTrack(&system2.tracks[0]);
                initializeImmModelsWithParams(&system1, dt, &immParams);
                initializeImmModelsWithParams(&system2, dt, &immParams);
                initializeImmTrackFromMeasurement(&system1, &system1.tracks[0], &measurement1, measurementCount);
                initializeImmTrackFromMeasurement(&system2, &system2.tracks[0], &measurement2, measurementCount);
            }
        }
        lastTime = measurement1.time;

        // 若是第一个量测结果，用第一个测量点初始化航迹
        if (firstMeasurement)
        {
            initializeImmTrackFromMeasurement(&system1, &system1.tracks[0], &measurement1, 0);
            initializeImmTrackFromMeasurement(&system2, &system2.tracks[0], &measurement2, 0);

            // 提取两个传感器对应的数据：位置信息和速度信息以及阈值信息
            Vector3D position1, velocity1, position2, velocity2;
            extractStateInfo(&system1.tracks[0].x, &position1, &velocity1);
            double gateThreshold1 = range1 * 0.00167 * 120;

            extractStateInfo(&system2.tracks[0].x, &position2, &velocity2);
            double gateThreshold2 = range2 * 0.00167 * 120;

            /*------------航迹关联的结果保存------------*/
            batch_pred[batchIndex].time = measurement1.time;
            batch_pred[batchIndex].position1 = position1;
            batch_pred[batchIndex].position2 = position2;
            batch_pred[batchIndex].isFirst = 1;
            /*---------------------------------------*/

            /*------------------最小二乘法融合------------------*/
            // 航及关联得到的笛卡尔位置信息转成球坐标系下的位置信息
            PolarCoord3D sensor1_polar = cartesianToPolar(position1);
            printf("sensor1: distance = %lf, azimuth = %lf, elevation = %lf\n", sensor1_polar.distance, sensor1_polar.azimuth, sensor1_polar.elevation);
            PolarCoord3D sensor2_polar = cartesianToPolar(position2);
            printf("sensor2: distance = %lf, azimuth = %lf, elevation = %lf\n", sensor2_polar.distance, sensor2_polar.azimuth, sensor2_polar.elevation);

            // 记录融合后的状态信息
            Matrix_2 sensor_fusion =  createMatrix(3, 1);
            // 记录融合后的协方差矩阵
            Matrix_2 P_fusion =  createMatrix(3, 3);
            ditui_ls(&sensor_fusion, &P_fusion, &sensor1_polar, &sensor2_polar, &H, &P1, &P2);
            PolarCoord3D sensor_fusion_polar = {sensor_fusion.data[0], sensor_fusion.data[1], sensor_fusion.data[2]};
            printf("ditui_ls and fusion finish!\n");

            // 存储到缓冲区
            batch_LS[batchIndex].time = measurement1.time;
            batch_LS[batchIndex].sensor1 = sensor1_polar;
            batch_LS[batchIndex].sensor2 = sensor2_polar;
            batch_LS[batchIndex].sensor_fusion = sensor_fusion_polar;
            batch_LS[batchIndex].isFirst = 1;
            batchIndex++;

            freeMatrix(&sensor_fusion);
            freeMatrix(&P_fusion);
            firstMeasurement = 0;
            measurementCount++;

            // 如果缓冲区已满，保存
            if (batchIndex == BATCH_SIZE)
            {
                for (int i = 0; i < batchIndex; i++)
                {
                    save_LS(outputFile, batch_LS[i].time,
                                &batch_LS[i].sensor1, &batch_LS[i].sensor2, &batch_LS[i].sensor_fusion, batch_LS[i].isFirst);

                    savePrediction(predFile, batch_pred[i].time, &batch_pred[i].position1, &batch_pred[i].position2, batch_pred[i].isFirst);
                }
                batchIndex = 0;
            }
            continue;
            /*-------------------*/
        }

        // 处理后续测量点
        Track *track1 = &system1.tracks[0];
        Track *track2 = &system2.tracks[0];

        // ====================== 传感器1数据 ======================
        double mixProb1[MAX_MODELS][MAX_MODELS];
        calculateMixingProbabilities(&system1, track1, mixProb1);  // 混合概率计算
        calculateMixedEstimates(&system1, track1, mixProb1);       // 混合估计结果
        immModePrediction(&system1, track1);                       // 每个模型进行kalman滤波的预测阶段

        /*=======这个应该有问题，预测的结果并没有记录在混合的x对象中，=========*/
        // 提取预测的状态信息(基于匀速模型)
        Vector3D predictedPosition1, predictedVelocity1;
        extractStateInfo(&track1->modelStates[0], &predictedPosition1, &predictedVelocity1);  // 用匀速模型的预测
        // extractStateInfo(&track1->x, &predictedPosition1, &predictedVelocity1);  // 有问题，track->x并没有被更新，可以理解为当前的结果和下一时刻的差距

        // 求阈值作为门限筛选条件
        double gateThreshold1 = compute_gate_threshold(
            predictedPosition1.x, predictedPosition1.y, predictedPosition1.z, 0.00167 * GATE);
        double distance1 = calculateDistance(&predictedPosition1, &measurement1.position);
        /*=========================================================*/

        // ====================== 传感器2处理 ======================
        double mixProb2[MAX_MODELS][MAX_MODELS];
        calculateMixingProbabilities(&system2, track2, mixProb2);
        calculateMixedEstimates(&system2, track2, mixProb2);
        immModePrediction(&system2, track2);

        Vector3D predictedPosition2, predictedVelocity2;
        extractStateInfo(&track2->modelStates[0], &predictedPosition2, &predictedVelocity2);
        // extractStateInfo(&track2->x, &predictedPosition2, &predictedVelocity2);  // 有问题，track->x并没有被更新，可以理解为当前的结果和下一时刻的差距

        double gateThreshold2 = compute_gate_threshold(
            predictedPosition2.x, predictedPosition2.y, predictedPosition2.z, 0.00167 * GATE);
        double distance2 = calculateDistance(&predictedPosition2, &measurement2.position);
        /*=========================================================*/

        // 记录更新前的参数
        Track temp_track1 = *track1;
        Track temp_track2 = *track2;

        if (distance1 <= gateThreshold1 || cnt <= 200)
        {
            immModeUpdate(&system1, track1, &measurement1);
            printf("Sensor 1, Measurement %d accepted: distance=%.2f, threshold=%.2f\n",
                   measurementCount, distance1, gateThreshold1);
        } 
        else
        {
            // if(ganrao_type == 3) {
            //     for (int i = 0; i < 3; i ++) {
            //         track1->modelStates[i] = track2->modelStates[i];
            //         track1->modelCovariances[i] = track2->modelCovariances[i];
            //         track1->mixedStates[i] = track2->mixedStates[i];
            //         track1->mixedCovariances[i] = track2->mixedCovariances[i];
            //         track1->modelProbabilities[i] = track2->modelProbabilities[i];
            //         track1->modelLikelihoods[i] = track2->modelLikelihoods[i];
            //     }
            // }
            track1 = &temp_track2;
            immModeUpdate(&system1, track1, &measurement2);
            queren1 ++;

            printf("rejected measurement1, change accepted measurement2\n");
        }

        // 提取航迹关联的估计结果中的位置和速度
        Vector3D position1, velocity1;
        extractStateInfo(&track1->x, &position1, &velocity1);
        // =======================================================
        // usleep(100000);

        if (distance2 <= gateThreshold2 || cnt <= 200)
        {
            immModeUpdate(&system2, track2, &measurement2);
            printf("Sensor 2, Measurement %d accepted: distance=%.2f, threshold=%.2f\n",
                   measurementCount, distance2, gateThreshold2);
        }
        else
        {
            track2 = &temp_track1;
            immModeUpdate(&system2, track2, &measurement1);
            printf("rejected measurement2, change accepted measurement1\n");
            queren2 ++;
        }

        Vector3D position2, velocity2;
        extractStateInfo(&track2->x, &position2, &velocity2);

        /*---------------保存航迹关联的预测结果------------------*/
        batch_pred[batchIndex].time = measurement1.time;
        batch_pred[batchIndex].position1 = position1;
        batch_pred[batchIndex].position2 = position2;
        batch_pred[batchIndex].isFirst = 0;
        /*--------------------------------------------------------*/
        
        /*--------------------最小二乘法估计和融合--------------------*/
        // 航及关联得到的笛卡尔坐标位置信息转成球坐标系下的位置信息
        PolarCoord3D sensor1_polar = cartesianToPolar(position1);
        PolarCoord3D sensor2_polar = cartesianToPolar(position2);

        // 记录融合后的状态信息
        Matrix_2 sensor_fusion = createMatrix(3, 1);
        // 记录融合后的协方差矩阵
        Matrix_2 P_fusion = createMatrix(3, 3);
        ditui_ls(&sensor_fusion, &P_fusion, &sensor1_polar, &sensor2_polar, &H, &P1, &P2);
        PolarCoord3D sensor_fusion_polar = {sensor_fusion.data[0], sensor_fusion.data[1], sensor_fusion.data[2]};
        printf("ditui_ls and fusion finish!\n");


        // 存储到缓冲区
        batch_LS[batchIndex].time = measurement1.time;
        batch_LS[batchIndex].sensor1 = sensor1_polar;
        batch_LS[batchIndex].sensor2 = sensor2_polar;
        batch_LS[batchIndex].sensor_fusion = sensor_fusion_polar;
        batch_LS[batchIndex].isFirst = 0;
        batchIndex++;

        // 如果缓冲区已满，保存
        if (batchIndex == BATCH_SIZE)
        {
            for (int i = 0; i < batchIndex; i++)
            {
                save_LS(outputFile, batch_LS[i].time,
                        &batch_LS[i].sensor1, &batch_LS[i].sensor2, &batch_LS[i].sensor_fusion,
                        batch_LS[i].isFirst);

                savePrediction(predFile, batch_pred[i].time, &batch_pred[i].position1, &batch_pred[i].position2, batch_pred[i].isFirst);
            }
            batchIndex = 0;
        }

        freeMatrix(&sensor_fusion);
        freeMatrix(&P_fusion);
        measurementCount++;
        /*--------------------------------------------------------*/
    }

    /*----------最小二乘法----------*/
    // 保存剩余的缓冲区数据
    if (batchIndex > 0)
    {
        for (int i = 0; i < batchIndex; i++)
        {
            save_LS(outputFile, batch_LS[i].time,
                    &batch_LS[i].sensor1, &batch_LS[i].sensor2, &batch_LS[i].sensor_fusion,
                    batch_LS[i].isFirst);

            savePrediction(predFile, batch_pred[i].time, &batch_pred[i].position1, &batch_pred[i].position2, batch_pred[i].isFirst);
        }
    }
    /*---------------------------*/
    printf("%d\n", queren1);
    printf("%d\n", queren2);

    // 关闭文件
    fclose(file);

    // 释放资源
    freeImmTrack(&system1.tracks[0]);
    freeImmTrack(&system2.tracks[0]);

    printf("Processing complete. Results saved to %s\n", outputFile);
}
