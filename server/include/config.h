#ifndef CONFIG_H
#define CONFIG_H

/* ===== 数学常量 ===== */
#define PI 3.14159265358979323846

#define GATE_RADAR 5   // 阈门设定
#define GATE_TV 50   // 阈门设定
#define GATE 2   // 阈门设定

#define MAX_LINES 50000  // 假设最大行数
#define NAN_VALUE -9999.0  // 定义NaN的替代值

#define DT 0.001           // 激光和雷达配准时间步长（s）
#define ALIGNMENT_INTERVAL 0.02  // 配准时间间隔（s）
#define POLAR_DT 0.02      // 极坐标数据配准时间步长（s）
#define MAX_INTERFERS 6  // 固定的 interfer 数量
#define PORT 7060
#define BUFFER_SIZE 1313
#define EPSILON 0.0001

#define MAX_LINE_LENGTH 256


/* ===== 常量定义 ===== */
#define MAX_MODELS 3        // IMM中的最大模型数量
#define MAX_MEASUREMENTS 20 // 每次扫描的最大测量数
#define MAX_TRACKS 10       // 最大航迹数量
#define MAX_HYPOTHESES 50   // 最大假设数量
#define STATE_DIM 9         // 状态向量维度 (x, y, z, vx, vy, vz, ax, ay, az)
#define MEAS_DIM 3          // 测量维度 (x, y, z)

/* ===== 运动模型类型定义 ===== */
#define MODEL_CONSTANT_VELOCITY  0  // 匀速模型
#define MODEL_CONSTANT_ACCELERATION 1  // 匀加速模型
#define MODEL_COORDINATED_TURN  2  // 协调转弯模型

/* ===== 滤波器参数 ===== */
#define CV_PROCESS_NOISE 1.0  // 匀速模型过程噪声强度
#define CA_PROCESS_NOISE 0.1  // 匀加速模型过程噪声强度   
#define CT_PROCESS_NOISE 1.0  // 协调转弯模型过程噪声强度
#define MEASUREMENT_NOISE 1.0  // 测量噪声强度

// 默认门限阈值（马氏距离的平方）
#define DEFAULT_GATING_THRESHOLD 9.0  // 对应3-sigma置信区间

#endif /* CONFIG_H */
