#ifndef TIME_REGISTRATION_H
#define TIME_REGISTRATION_H
#include "sensor_data.h"

// double gate(float x, float y, float z, float theta) ;
    
// double calculateDistance(float pre_x,float pre_y,float pre_z, float meas_x, float meas_y, float meas_z);

// 卡尔曼滤波初始化（激光）
void kalman_init_laser(KalmanStateAxis *state, KalmanCovAxis *cov);

// 新增：初始化多维雷达卡尔曼滤波状态和协方差
void new_kalman_init_radar(KalmanState *state, KalmanCov *cov);

void new_kalman_init_polar(KalmanState *state, KalmanCov *cov);

// 卡尔曼滤波预测步骤（每个轴独立）
void kalman_predict_axis(KalmanStateAxis *state, KalmanCovAxis *cov, float dt);

// 卡尔曼滤波更新步骤（每个轴独立）
void kalman_update_axis(KalmanStateAxis *state, KalmanCovAxis *cov, float measurement);

// 新增：卡尔曼滤波预测步骤（多维）
void new_kalman_predict(KalmanState *state, KalmanCov *cov, float dt);

// 新增：卡尔曼滤波更新步骤（多维）
void new_kalman_update(KalmanState *state, KalmanCov *cov, Matrix* measurement);

// 激光数据时间配准
// void align_laser_data(Laser *laser, float current_time, KalmanStateAxis *state, KalmanCovAxis *cov, float last_time, LaserAlignedData *aligned_data, int line_count);
void align_laser_data(Laser *laser, float current_time, KalmanStateAxis *state, KalmanCovAxis *cov, float last_time, LaserAlignedData *aligned_data,
     LaserAlignedData *aligned_data2, int line_count);

// 修改后的雷达时间配准函数
void new_align_radar_data(Radar *radar1, Radar *radar2, float current_time, 
    KalmanState *state, KalmanCov *cov,
    float last_time, RadarAlignedData *aligned_data, RadarAlignedData *aligned_data2, int line_count);

void new_align_polar_data(PolarData *polar, float current_time, 
    KalmanState *state, KalmanCov *cov,
    float last_time, PolarAlignedData *aligned_data, PolarAlignedData *aligned_data2, int line_count);

#endif