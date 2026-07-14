#ifndef TOOL_H
#define TOOL_H

#include "types.h"
#define BATCH_SIZE 20

// 从状态向量提取位置和速度信息
void extractStateInfo(Matrix_2* x, Vector3D* position, Vector3D* velocity);

// 计算欧氏距离
double calculateDistance(Vector3D* predicted, Vector3D* measured);

// 计算动态门限阈值
double compute_gate_threshold(double x, double y, double z, double theta);

// // 保存预测结果到文件 - 添加更多信息
// void savePredictions(const char* filename, double* times, 
//     Vector3D* positions1, Vector3D* velocities1, double* gateThresholds1,
//     Vector3D* positions2, Vector3D* velocities2, double* gateThresholds2,
//     Vector3D* fusedPositions, Vector3D* fusedVelocities, double* fusedGateThresholds,
//     double* omegas, int count);

// // 保存预测结果到文件 - 添加更多信息
// void savePredictions(const char* filename, double* times, 
//     Vector3D* positions1, Vector3D* velocities1, double* gateThresholds1,
//     Vector3D* positions2, Vector3D* velocities2, double* gateThresholds2,
//     Vector3D* fusedPositions, Vector3D* fusedVelocities, double* fusedGateThresholds,
//     double* omegas, int count);


// 系统对量测进行处理
void predOnce(TrackingSystem* system, Measurement* meas);

// 将结果写入文件中
void writeResultsToFile(const char* outputFile, char** lines, int count);

// 运行程序
// void run(const char* inputFile, const char* outputFile);
void run(const char *inputFile, const char *outputFile, const char *predFile);


#endif