#include "sensor_data.h"
#include "coord_trans.h"
#include "matrix.h"
#include <math.h>
#include <stdio.h>
#define DT 0.001           // 激光和雷达配准时间步长（s）
#define POLAR_DT 0.02      // 极坐标数据配准时间步长（s）
#define NAN_VALUE -9999.0  // 定义NaN的替代值
#define PI 3.1415
//计算阈门
double gate(float x, float y, float z, float theta) {
    double r = sqrt(x * x + y * y + z * z); // 计算径向距离
    double gateThreshold = r * theta; // 计算动态门限阈值
    return gateThreshold; // 返回计算结果
}

// double calculateDistance(float pre_x,float pre_y,float pre_z, float meas_x, float meas_y, float meas_z){
//     double dx = pre_x - meas_x;
//     double dy = pre_y - meas_x;
//     double dz = pre_x - meas_x;
//     return sqrt(dx* dx + dy * dy + dz * dz);
// }

// 卡尔曼滤波初始化（激光）
void kalman_init_laser(KalmanStateAxis *state, KalmanCovAxis *cov) {
    state->pos = 8650.0;
    state->vel = 0.0;
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++) {
            cov->P[i][j] = (i == j) ? 10.0 : 0.0;
        }
}

// 新增：初始化多维雷达卡尔曼滤波状态和协方差
void new_kalman_init_radar(KalmanState *state, KalmanCov *cov) {
    float alpha = 0.78; // 方位角（弧度）
    float beta = 0.61;  // 仰角（弧度）
    float r = 8650.254; // 径向距离
    float x, y, z;
    
    // 转换为笛卡尔坐标
    polar2Descartes(alpha, beta, r, &x, &y, &z);
    
    // 初始化状态向量（6x1）：[x, y, z, vx, vy, vz]
    state->state = matrix_create(6, 1);
    state->state->data[0][0] = x;   // x 位置
    state->state->data[1][0] = y;   // y 位置
    state->state->data[2][0] = z;   // z 位置
    state->state->data[3][0] = 0.0; // vx 初始速度
    state->state->data[4][0] = 0.0; // vy 初始速度
    state->state->data[5][0] = 0.0; // vz 初始速度
    
    // 初始化协方差矩阵（6x6）：对角矩阵，初始不确定性
    cov->P = matrix_create(6, 6);
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            cov->P->data[i][j] = (i == j) ? 10.0 : 0.0; // 对角线为 10，其余为 0
        }
    }
}

// 新增：初始化多维卡尔曼滤波状态和协方差（极坐标）
void new_kalman_init_polar(KalmanState *state, KalmanCov *cov) {
    float alpha = 21.7; // 方位角（弧度）
    float beta = 17.1;  // 仰角（弧度）
    float r = 838.0;    // 径向距离
    float x, y, z;
    
    // 转换为笛卡尔坐标
    polar2Descartes(alpha, beta, r, &x, &y, &z);
    
    // 初始化状态向量（6x1）：[x, y, z, vx, vy, vz]
    state->state = matrix_create(6, 1);
    state->state->data[0][0] = x;   // x 位置
    state->state->data[1][0] = y;   // y 位置
    state->state->data[2][0] = z;   // z 位置
    state->state->data[3][0] = 1.0; // vx 初始速度
    state->state->data[4][0] = 1.0; // vy 初始速度
    state->state->data[5][0] = 1.0; // vz 初始速度
    
    // 初始化协方差矩阵（6x6）：对角矩阵，初始不确定性
    cov->P = matrix_create(6, 6);
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            cov->P->data[i][j] = (i == j) ? 10.0 : 0.0; // 对角线为 10，其余为 0
        }
    }
}

// *****卡尔曼滤波预测步骤（每个轴独立）
void kalman_predict_axis(KalmanStateAxis *state, KalmanCovAxis *cov, float dt) {
    state->pos += state->vel * dt;
    
    float Q[2][2] = {{10, 0.0}, {0.0, 10 }};
    float P_pred[2][2];
    P_pred[0][0] = cov->P[0][0] + dt * (cov->P[1][0] + cov->P[0][1] + dt * cov->P[1][1]) + Q[0][0];
    P_pred[0][1] = cov->P[0][1] + dt * cov->P[1][1] + Q[0][1];
    P_pred[1][0] = cov->P[1][0] + dt * cov->P[1][1] + Q[1][0];
    P_pred[1][1] = cov->P[1][1] + Q[1][1];
    
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++)
            cov->P[i][j] = P_pred[i][j];
}

// *****卡尔曼滤波更新步骤（每个轴独立）
void kalman_update_axis(KalmanStateAxis *state, KalmanCovAxis *cov, float measurement) {
    float R = 10.0;
    float S = cov->P[0][0] + R;
    float K[2];
    K[0] = cov->P[0][0] / S;
    K[1] = cov->P[1][0] / S;
    
    float innovation = measurement - state->pos;
    state->pos += K[0] * innovation;
    state->vel += K[1] * innovation;
    
    // printf("K[0] = %.2f\tK[1] = %.2f\n",K[0],K[1]);

    float P_temp[2][2];
    P_temp[0][0] = cov->P[0][0] * (1.0 - K[0]);
    P_temp[0][1] = cov->P[0][1] * (1.0 - K[0]);
    P_temp[1][0] = cov->P[1][0] - K[1] * cov->P[0][0];
    P_temp[1][1] = cov->P[1][1] - K[1] * cov->P[0][1];
    
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++)
            cov->P[i][j] = P_temp[i][j];
}

// 新增：卡尔曼滤波预测步骤（多维）
void new_kalman_predict(KalmanState *state, KalmanCov *cov, float dt) {
    // 定义状态转移矩阵 F (6x6)
    Matrix* F = matrix_create(6, 6);
    matrix_identity(F);
    F->data[0][3] = dt; // x 受 vx 影响
    F->data[1][4] = dt; // y 受 vy 影响
    F->data[2][5] = dt; // z 受 vz 影响

    // 状态预测: x = F * x
    Matrix* state_pred = matrix_create(6, 1);
    matrix_multiply(F, state->state, state_pred);

    // 协方差预测: P = F * P * F' + Q
    Matrix* Q = matrix_create(6, 6);
    matrix_identity(Q);
    matrix_scalar_multiply(Q, 10.0f, Q); // 假设过程噪声

    Matrix* FP = matrix_create(6, 6);
    Matrix* FPF = matrix_create(6, 6);
    Matrix* F_trans = matrix_create(6, 6);
    
    matrix_multiply(F, cov->P, FP);
    matrix_transpose(F, F_trans);
    matrix_multiply(FP, F_trans, FPF);
    matrix_add(FPF, Q, cov->P);

    // 更新状态
    matrix_copy(state_pred, state->state);

    // 释放临时矩阵
    matrix_free(F);
    matrix_free(state_pred);
    matrix_free(Q);
    matrix_free(FP);
    matrix_free(F_trans);
    matrix_free(FPF);
}

// 新增：卡尔曼滤波更新步骤（多维）
void new_kalman_update(KalmanState *state, KalmanCov *cov, Matrix* measurement) {
    // 观测矩阵 H (3x6，只观测位置)
    Matrix* H = matrix_create(3, 6);
    matrix_identity(H); // 前 German: 仅观测 x,y,z
    for (int i = 3; i < 6; i++) {
        H->data[i-3][i] = 0.0f;
    }

    // 计算卡尔曼增益
    float R_scalar = 10.0f;
    Matrix* R = matrix_create(3, 3);
    matrix_identity(R);
    matrix_scalar_multiply(R, R_scalar, R);

    Matrix* HP = matrix_create(3, 6);
    Matrix* H_trans = matrix_create(6, 3);
    Matrix* HPH = matrix_create(3, 3);
    Matrix* HPH_R = matrix_create(3, 3);
    Matrix* K = matrix_create(6, 3);

    matrix_multiply(H, cov->P, HP);
    matrix_transpose(H, H_trans);
    matrix_multiply(HP, H_trans, HPH);
    matrix_add(HPH, R, HPH_R);
    
    // 假设 HPH_R 是可逆的，这里需要更复杂的逆矩阵计算，简化处理
    for (int i = 0; i < 3; i++) {
        K->data[i][i] = cov->P->data[i][i] / (HPH_R->data[i][i] + 0.0001f); // 简单近似
        K->data[i+3][i] = cov->P->data[i+3][i] / (HPH_R->data[i][i] + 0.0001f);
    }

    // 状态更新
    Matrix* innovation = matrix_create(3, 1);
    Matrix* Hx = matrix_create(3, 1);
    matrix_multiply(H, state->state, Hx);
    for (int i = 0; i < 3; i++) {
        innovation->data[i][0] = measurement->data[i][0] - Hx->data[i][0];
    }

    Matrix* K_innov = matrix_create(6, 1);
    matrix_multiply(K, innovation, K_innov);
    matrix_add(state->state, K_innov, state->state);

    // 协方差更新
    Matrix* KH = matrix_create(6, 6);
    Matrix* I = matrix_create(6, 6);
    Matrix* I_KH = matrix_create(6, 6);
    matrix_multiply(K, H, KH);
    matrix_identity(I);
    matrix_scalar_multiply(KH, -1.0f, KH);
    matrix_add(I, KH, I_KH);
    Matrix* P_temp = matrix_create(6, 6);
    matrix_multiply(I_KH, cov->P, P_temp);
    matrix_copy(P_temp, cov->P);

    // 释放临时矩阵
    matrix_free(H);
    matrix_free(R);
    matrix_free(HP);
    matrix_free(H_trans);
    matrix_free(HPH);
    matrix_free(HPH_R);
    matrix_free(K);
    matrix_free(innovation);
    matrix_free(Hx);
    matrix_free(K_innov);
    matrix_free(KH);
    matrix_free(I);
    matrix_free(I_KH);
    matrix_free(P_temp);
}

// // 激光数据时间配准
// void align_laser_data(Laser *laser, float current_time, 
//     KalmanStateAxis *state, KalmanCovAxis *cov, float last_time, 
//     LaserAlignedData *aligned_data, int line_count) {
//     // if(aligned_data == NULL) {
//     //     printf("NULL\n");
//     // }
//     // 初始化输出
//     aligned_data->timestamp = current_time;
//     if(line_count = 1){
//         aligned_data->radial_distance = state->pos;
//         aligned_data->vr = state->vel;// 输出初始速度（新增）
//     }

//     // 计算时间步数
//     int steps = (int)((current_time - last_time) / DT);
//     float t = last_time;

//     // 时间步循环
//     for (int i = 0; i < steps; i++) {
//         t += DT;
//         int has_measurement = 0;
//         float measurement = 0.0;

//         // 检查激光数据
//         if (fabs(t - (current_time + laser->timestamp)) <= DT*1.2 ){
//             if (laser->radial_distance != NAN_VALUE) {
//                 has_measurement = 1;
//                 measurement = laser->radial_distance;
//             }
//         }

//         // 卡尔曼滤波预测
//         kalman_predict_axis(state, cov, DT);


//         // 卡尔曼滤波更新
//         if (has_measurement) {
//             kalman_update_axis(state, cov, measurement);
//             // printf("时刻：%.3f，已更新卡尔曼滤波\n",current_time);
//         }

//         // 每次循环更新输出
//         aligned_data->timestamp = current_time;
//         aligned_data->radial_distance = state->pos;
//         aligned_data->vr = state->vel;

//     }

// }

// 激光数据时间配准
void align_laser_data(Laser *laser, float current_time, 
    KalmanStateAxis *state, KalmanCovAxis *cov, float last_time, 
    LaserAlignedData *aligned_data, LaserAlignedData *aligned_data2, int line_count) {
    // 初始化输出
    aligned_data->timestamp = current_time;
    if(line_count = 1){
        aligned_data->radial_distance = state->pos;
        aligned_data->vr = state->vel;// 输出初始速度（新增）
    }else{
        aligned_data->radial_distance = aligned_data2->radial_distance;
        aligned_data->vr = aligned_data2->vr;
    }

    // 计算时间步数
    int steps = (int)((current_time - last_time) / DT);
    float t = last_time;

    // 时间步循环
    for (int i = 0; i < steps; i++) {
        t += DT;
        int has_measurement = 0;
        float measurement = 0.0;

        // 检查激光数据
        // if (fabs(t - (current_time + laser->timestamp)) <= DT*1.2 ){
            if (laser->radial_distance != NAN_VALUE) {
                has_measurement = 1;
                measurement = laser->radial_distance;
                // printf("laser->radial_distance1 = %f\n", laser->radial_distance);
            }
            // printf("laser->radial_distance2 = %f\n", laser->radial_distance);
        // }
        // printf("laser->radial_distance3 = %f\n", laser->radial_distance);

        // 卡尔曼滤波预测
        kalman_predict_axis(state, cov, DT);


        // 卡尔曼滤波更新
        if (has_measurement) {
            kalman_update_axis(state, cov, measurement);
            // printf("时刻：%.3f，已更新卡尔曼滤波\n",current_time);
        }

        // 每次循环更新输出
        aligned_data->timestamp = current_time;
        aligned_data->radial_distance = state->pos;
        aligned_data->vr = state->vel;

    }

}

// 修改后的雷达时间配准函数
void new_align_radar_data(Radar *radar1, Radar *radar2, float current_time, 
    KalmanState *state, KalmanCov *cov,
    float last_time, RadarAlignedData *aligned_data, RadarAlignedData *aligned_data2, int line_count) {
    
    aligned_data->timestamp = current_time;
    if(line_count == 1){
        aligned_data->x = state->state->data[0][0];
        aligned_data->y = state->state->data[1][0];
        aligned_data->z = state->state->data[2][0];
        aligned_data->vx = state->state->data[3][0];
        aligned_data->vy = state->state->data[4][0];
        aligned_data->vz = state->state->data[5][0];
    } else {
        aligned_data->x = aligned_data2->x;
        aligned_data->y = aligned_data2->y;
        aligned_data->z = aligned_data2->z;
        aligned_data->vx = aligned_data2->vx;
        aligned_data->vy = aligned_data2->vy;
        aligned_data->vz = aligned_data2->vz;
    }
    
    // 转换为极坐标
    Descartes2polar(state->state->data[0][0], state->state->data[1][0], state->state->data[2][0], 
        &aligned_data->azimuth, &aligned_data->elevation, &aligned_data->radial_distance);

    // 计算时间步数
    int steps = (int)((current_time - last_time) / DT);
    float t = last_time;

    // 时间步循环
    for (int i = 0; i < steps; i++) {
        t += DT;
        int has_measurement = 0;
        Matrix* measurement = matrix_create(3, 1);

        if (fabs(t - (current_time + radar1->timestamp)) <= DT*1.2 ) {
            if (radar1->radial_distance != NAN_VALUE && radar1->work_status == 1.0) {
                has_measurement = 1;
                polar2Descartes(radar1->azimuth, radar1->elevation, radar1->radial_distance, 
                    &measurement->data[0][0], &measurement->data[1][0], &measurement->data[2][0]);
            }
        }

        if (fabs(t - (current_time + radar2->timestamp)) <= DT*1.2 ) {
            if (radar2->radial_distance != NAN_VALUE && radar2->work_status == 1.0) {
                has_measurement = 1;
                polar2Descartes(radar2->azimuth, radar2->elevation, radar2->radial_distance, 
                    &measurement->data[0][0], &measurement->data[1][0], &measurement->data[2][0]);
            }
        }

        // 卡尔曼滤波预测和更新
        new_kalman_predict(state, cov, DT);
        if (has_measurement) {
            new_kalman_update(state, cov, measurement);
        }

        // 更新输出数据
        aligned_data->timestamp = current_time;
        aligned_data->x = state->state->data[0][0];
        aligned_data->y = state->state->data[1][0];
        aligned_data->z = state->state->data[2][0];
        aligned_data->vx = state->state->data[3][0];
        aligned_data->vy = state->state->data[4][0];
        aligned_data->vz = state->state->data[5][0];

        Descartes2polar(state->state->data[0][0], state->state->data[1][0], state->state->data[2][0], 
            &aligned_data->azimuth, &aligned_data->elevation, &aligned_data->radial_distance);

        matrix_free(measurement);
    }
}

// 新增：多维极坐标数据时间配准函数
void new_align_polar_data(PolarData *polar, float current_time, 
    KalmanState *state, KalmanCov *cov,
    float last_time, PolarAlignedData *aligned_data, PolarAlignedData *aligned_data2, int line_count) {
    
    aligned_data->timestamp = current_time;
    if (line_count == 1) {
        aligned_data->x = state->state->data[0][0];
        aligned_data->y = state->state->data[1][0];
        aligned_data->z = state->state->data[2][0];
        aligned_data->vx = state->state->data[3][0];
        aligned_data->vy = state->state->data[4][0];
        aligned_data->vz = state->state->data[5][0];
    } else {
        aligned_data->x = aligned_data2->x;
        aligned_data->y = aligned_data2->y;
        aligned_data->z = aligned_data2->z;
        aligned_data->vx = aligned_data2->vx;
        aligned_data->vy = aligned_data2->vy;
        aligned_data->vz = aligned_data2->vz;
    }
    
    polar->timestamp = 0.0;
    Descartes2polar(state->state->data[0][0], state->state->data[1][0], state->state->data[2][0], 
        &aligned_data->azimuth, &aligned_data->elevation, &aligned_data->radial_distance);

    // 计算时间步数
    int steps = (int)((current_time - last_time) / DT);
    float t = last_time;

    // 时间步循环
    for (int i = 0; i < steps; i++) {
        t += DT;
        int has_measurement = 0;
        Matrix* measurement = matrix_create(3, 1);

        // 检查数据
        if (fabs(t - (current_time + polar->timestamp)) <= DT * 1.2) {
            if (polar->radial_distance != NAN_VALUE) {
                has_measurement = 1;
                polar2Descartes(polar->azimuth, polar->elevation, polar->radial_distance, 
                    &measurement->data[0][0], &measurement->data[1][0], &measurement->data[2][0]);
            }
        }

        // 卡尔曼滤波预测
        new_kalman_predict(state, cov, DT);

        // 计算门限和距离（保留原有逻辑）
        // double yumen = gate(state->state->data[0][0], state->state->data[1][0], state->state->data[2][0], 0.00167 * 10);
        // double distence = calculateDistance(state->state->data[0][0], state->state->data[1][0], state->state->data[2][0],
        //                                     measurement->data[0][0], measurement->data[1][0], measurement->data[2][0]);

        // 卡尔曼滤波更新
        if (has_measurement) {
            new_kalman_update(state, cov, measurement);
        }

        // 更新输出数据
        aligned_data->timestamp = current_time;
        aligned_data->x = state->state->data[0][0];
        aligned_data->y = state->state->data[1][0];
        aligned_data->z = state->state->data[2][0];
        aligned_data->vx = state->state->data[3][0];
        aligned_data->vy = state->state->data[4][0];
        aligned_data->vz = state->state->data[5][0];

        Descartes2polar(state->state->data[0][0], state->state->data[1][0], state->state->data[2][0], 
            &aligned_data->azimuth, &aligned_data->elevation, &aligned_data->radial_distance);

        matrix_free(measurement);
    }
}

