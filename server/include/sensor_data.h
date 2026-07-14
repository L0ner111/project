#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H
#include "matrix.h"
// 光电数据结构体
typedef struct {
    float timestamp;         // 时间戳
    float work_status;       // 工作状态
    float azimuth_deviation; // 方位偏差
    float elevation_deviation; // 俯仰偏差
} Photoelectric;

// 激光数据结构体
typedef struct {
    float timestamp;         // 时间戳
    float radial_distance;   // 径向距离
} Laser;

// 雷达数据结构体
typedef struct {
    float timestamp;         // 时间戳
    float work_status;       // 工作状态
    float azimuth;           // 方位角
    float elevation;         // 俯仰角
    float radial_distance;   // 径向距离
} Radar;

// 伺服器数据结构体
typedef struct {
    float timestamp;         // 时间戳
    float azimuth;           // 方位角
    float elevation;         // 俯仰角
} Servo;

// 完整光电数据结构体
typedef struct {
    float timestamp;         // 时间戳
    float work_status;       // 工作状态
    float azimuth;           // 方位角
    float elevation;         // 俯仰角
} FullPhotoelectric;

// 卡尔曼滤波状态结构体（每个轴独立）
typedef struct {
    float pos;               // 位置（x, y, z 或激光距离）
    float vel;               // 速度
} KalmanStateAxis;

// 卡尔曼滤波协方差矩阵（每个轴独立）
typedef struct {
    float P[2][2];           // 协方差矩阵
} KalmanCovAxis;

// 极坐标数据结构体
typedef struct {
    float timestamp;         // 时间戳
    float work_status;       // 工作状态
    float azimuth;           // 方位角
    float elevation;         // 俯仰角
    float radial_distance;   // 径向距离
} PolarData;

// 雷达配准数据结构体
typedef struct {
    float timestamp;         // 配准时间
    float x, y, z;           // 配准后的笛卡尔坐标
    float vx,vy,vz;          // xyz方向速度（新增）
    float azimuth;           // 配准后的极坐标方位角
    float elevation;         // 配准后的极坐标俯仰角
    float radial_distance;   // 配准后的极坐标径向距离
} RadarAlignedData;

//激光配准数据结构体
typedef struct {
    float timestamp;        // 对齐时间戳
    float vr;
    float radial_distance;  // 对齐的径向距离
} LaserAlignedData;

// 极坐标配准数据结构体
typedef struct {
    float timestamp;         // 配准时间
    float x, y, z;           // 配准后的笛卡尔坐标
    float vx,vy,vz;
    float azimuth;           // 配准后的极坐标方位角
    float elevation;         // 配准后的极坐标俯仰角
    float radial_distance;   // 配准后的极坐标径向距离
} PolarAlignedData;

// 定义返回类型包含两个对齐数据
typedef struct {
    float timestamp;
    float x;
    float y;
    float z;
    float azimuth;
    float elevation;
    float radial_distance;
} SensorAlignedData;

// 跳跟目标结构体
typedef struct {
    double start_time; // 目标开始时间
    double duration;   // 持续时间（秒）
} JumpTarget;

// 跳跟目标结构体
typedef struct {
    double start_time;         // 目标开始时间
    double duration;           // 持续时间（秒）
    float azimuth_start;       // 初始方位偏差（弧度）
    float azimuth_end;         // 终止方位偏差（弧度）
    float elevation_start;     // 初始仰角偏差（弧度）
    float elevation_end;       // 终止仰角偏差（弧度）
} JumpTarget2;

// ADDED: 存储上一行数据的结构体
typedef struct {
    double photo_time, photo_status, photo_azim, photo_elev;
    double laser_time, laser_dist;
    double radar_time, radar_status, radar_azim, radar_elev, radar_dist;
    double servo_time, servo_azim, servo_elev;
    int is_first_line;
} PrevData;

// 新状态结构，包含三维位置和速度
typedef struct {
    Matrix* state; // 6x1 状态向量 [x, y, z, vx, vy, vz]
} KalmanState;


// 新协方差结构，6x6 矩阵表示位置和速度的协方差
typedef struct {
    Matrix* P; // 6x6 协方差矩阵
} KalmanCov;


// 时间组件枚举
typedef enum {
    TIME_YEAR,
    TIME_MONTH,
    TIME_DAY,
    TIME_HOUR,
    TIME_MINUTE,
    TIME_SECOND
} TimeComponent;



#endif /* SENSOR_DATA_H */