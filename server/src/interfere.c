#include "sensor_data.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "coord_trans.h"
#include "constant.h"

const float NOISE_SIGMA = 0.001; // 高斯噪声标准差

// 生成标准正态分布随机数（Box-Muller变换）
float gaussian_noise(float sigma) {
    static int have_spare = 0;
    static float spare;
    
    if (have_spare) {
        have_spare = 0;
        return sigma * spare;
    }
    
    have_spare = 1;
    float u1 = (float)rand() / RAND_MAX;
    float u2 = (float)rand() / RAND_MAX;
    float r = sqrt(-2.0 * log(u1));
    float theta = 2.0 * PI * u2;
    
    spare = r * sin(theta);
    return sigma * r * cos(theta);
}

// 生成指定范围内的随机浮点数
float random_float(float min, float max) {
    return min + ((float)rand() / RAND_MAX) * (max - min);
}

// ------------------------------------------------------雷达干扰函数-------------------------------------------------------
void Radar_Interfering(Radar *radar, double start_time, double end_time, double interfere_freq, int line_count,int *count) {
    // 检查基本参数是否有效
    if (radar->timestamp == NAN_VALUE || radar->work_status == NAN_VALUE ||
        radar->azimuth == NAN_VALUE || radar->elevation == NAN_VALUE ||
        radar->radial_distance == NAN_VALUE) {
        return;
    }
    double radar_time = radar->timestamp + line_count*0.02;

    // 检查当前时间是否在干扰时间范围内
    if (radar_time < start_time || radar->timestamp > end_time) {
        return;  // 如果不在干扰时间段内，直接返回，不进行干扰
    }

    // // 确保干扰频率在有效范围内 (0-100)
    // if (interfere_freq < 0 || interfere_freq > 1) {
    //     interfere_freq = 0.05;  // 默认值，如果输入无效则使用5%
    // }

    // 生成随机数并判断是否进行干扰
    int rand_num = rand() % 100;
    if (rand_num < (int)(interfere_freq * 100)) {
        radar->work_status = 0.0;  // 在指定频率下将雷达工作状态置为0
        *count = *count + 1;
    }
    printf("应用旧干扰成功");
}



// ------------------------------------------------------光电跟丢干扰函数------------------------------------------------------
void Photoelectric_Lost_with(Photoelectric *photoelectric, double start_time, double end_time, double interference_freq, 
    int current_line, int *interfere_count, int *interfere_start,int *count) {
    double photoelectric_time = photoelectric->timestamp + current_line*0.02;
    // 如果干扰已经触发，检查是否在持续时间内
    if (*interfere_count >= 1) {
        if (current_line >= *interfere_start && current_line < *interfere_start + 11) {
            if (photoelectric_time != NAN_VALUE) {
                photoelectric->work_status = 0.0;  // 持续干扰，工作状态置为 0
            }
        }else *interfere_count = 0;
    return;
    }

    // 检查输入数据是否有效
    if (photoelectric->timestamp == NAN_VALUE || photoelectric->work_status == NAN_VALUE ||
    photoelectric->azimuth_deviation == NAN_VALUE || photoelectric->elevation_deviation == NAN_VALUE) {
        return;
    }

    // 检查当前时间是否在干扰时间范围内
    if (photoelectric_time < start_time || photoelectric_time > end_time) {
        return;  // 时间不在干扰范围内，不施加干扰
    }

    // 根据干扰频率决定是否触发干扰
    int rand_num = rand() % 100;  // 生成 0-99 的随机数
    if (rand_num < (int)(interference_freq * 100)) {  // interference_freq 是 0.0-1.0 的概率
        photoelectric->work_status = 0.0;  // 干扰生效，工作状态置为 0
        *interfere_count = 1;              // 记录干扰触发
        *interfere_start = current_line;   // 记录干扰开始的行号
        *count = *count + 1;
    }
}



// ------------------------------------------------------光电拉偏a干扰函数------------------------------------------------------
void Photoelectric_pull_a(Photoelectric *photoelectric, double start_time, double end_time, double interference_freq, 
    int current_line, int *interfere_flag, int *interfere_start, 
    float *fixed_azimuth, float *fixed_elevation,int *count) {
    double photoelectric_time = photoelectric->timestamp + current_line*0.02;
    // 如果干扰已经触发，检查是否在持续时间内
    if (*interfere_flag >= 1) {
        if (current_line >= *interfere_start && current_line < *interfere_start + 11) {
            if (photoelectric->timestamp != NAN_VALUE) {
                photoelectric->azimuth_deviation = *fixed_azimuth;    // 固定方位偏差
                photoelectric->elevation_deviation = *fixed_elevation; // 固定俯仰偏差
            }
        }else *interfere_flag = 0;
        return;
    }

    // 检查输入数据是否有效
    if (photoelectric->timestamp == NAN_VALUE || photoelectric->work_status == NAN_VALUE ||
    photoelectric->azimuth_deviation == NAN_VALUE || photoelectric->elevation_deviation == NAN_VALUE) {
        return;
    }

    // 检查当前时间是否在干扰时间范围内
    if (photoelectric_time < start_time || photoelectric_time > end_time) {
        return;  // 时间不在干扰范围内，不施加干扰
    }

    // 根据干扰频率决定是否触发干扰
    int rand_num = rand() % 100;  // 生成 0-99 的随机数
    // printf("rand_num=%d\ninterference_freq=%.2f\n",rand_num,interference_freq);
    if (rand_num < (int)(interference_freq * 100) ) {  // interference_freq 是 0-100 的概率
        photoelectric->elevation_deviation += 0.002;  // 施加拉偏，俯仰偏差增加 0.002
        *fixed_azimuth = photoelectric->azimuth_deviation;    // 记录固定的方位偏差
        *fixed_elevation = photoelectric->elevation_deviation; // 记录固定的俯仰偏差
        *interfere_flag = 1;              // 标记干扰触发
        *interfere_start = current_line;   // 记录干扰开始的行号
        *count = *count + 1;
    }
}



// ------------------------------------------------------光电拉偏b干扰函数------------------------------------------------------
void Photoelectric_pull_b(Photoelectric *photoelectric, double start_time, double end_time, double interference_freq, 
    int current_line, int *interfere_flag, int *interfere_start,int *count) {
    double photoelectric_time = photoelectric->timestamp + current_line*0.02;
    const float AZIMUTH_OFFSET = 0.001;    // 方位偏差偏移量
    const float ELEVATION_OFFSET = -0.0004; // 俯仰偏差偏移量

    // 如果干扰已经触发，检查是否在持续时间内
    if (*interfere_flag >= 1) {
        if (current_line >= *interfere_start && current_line < *interfere_start + 11) {
            if (photoelectric->timestamp != NAN_VALUE) {
                photoelectric->azimuth_deviation += AZIMUTH_OFFSET;    // 持续施加方位偏移
                photoelectric->elevation_deviation += ELEVATION_OFFSET; // 持续施加俯仰偏移
            }
        }else *interfere_flag = 0;
        return;
    }

    // 检查输入数据是否有效
    if (photoelectric->timestamp == NAN_VALUE || photoelectric->work_status == NAN_VALUE ||
    photoelectric->azimuth_deviation == NAN_VALUE || photoelectric->elevation_deviation == NAN_VALUE) {
        return;
    }

    // 检查当前时间是否在干扰时间范围内
    if (photoelectric_time < start_time || photoelectric_time > end_time) {
        return;  // 时间不在干扰范围内，不施加干扰
    }

    // 根据干扰频率决定是否触发干扰
    int rand_num = rand() % 100;  // 生成 0-99 的随机数
    if (rand_num < (int)(interference_freq * 100)) {  // interference_freq 是 0.0-1.0 的概率
        photoelectric->azimuth_deviation += AZIMUTH_OFFSET;    // 施加方位偏移
        photoelectric->elevation_deviation += ELEVATION_OFFSET; // 施加俯仰偏移
        *interfere_flag = 1;              // 标记干扰触发
        *interfere_start = current_line;   // 记录干扰开始的行号
        *count = *count + 1;
    }
}




// ------------------------------------------------------光电跳变a干扰函数------------------------------------------------------
void Photoelectric_Jump_around_a(Photoelectric *photoelectric, double start_time, double end_time, double interference_freq, 
    int current_line, int *interfere_flag, int *interfere_start,int *count) {
    const float NOISE_SIGMA = 0.001;  // 高斯噪声的标准差
    double photoelectric_time = photoelectric->timestamp + current_line*0.02;
    // 如果干扰已经触发，检查是否在持续时间内
    if (*interfere_flag >= 1) {
        if (current_line >= *interfere_start && current_line < *interfere_start + 11) {
            if (photoelectric->timestamp != NAN_VALUE) {
                photoelectric->azimuth_deviation += gaussian_noise(NOISE_SIGMA);    // 添加方位噪声
                photoelectric->elevation_deviation += gaussian_noise(NOISE_SIGMA);   // 添加俯仰噪声
            }
        }else *interfere_flag = 0;
        return;
    }

    // 检查输入数据是否有效
    if (photoelectric->timestamp == NAN_VALUE || photoelectric->work_status == NAN_VALUE ||
    photoelectric->azimuth_deviation == NAN_VALUE || photoelectric->elevation_deviation == NAN_VALUE) {
        return;
    }

    // 检查当前时间是否在干扰时间范围内
    if (photoelectric_time < start_time || photoelectric_time > end_time) {
        return;  // 时间不在干扰范围内，不施加干扰
    }

    // 根据干扰频率决定是否触发干扰
    int rand_num = rand() % 100;  // 生成 0-99 的随机数
    if (rand_num < (int)(interference_freq * 100)) {  // interference_freq 是 0.0-1.0 的概率
        photoelectric->azimuth_deviation += gaussian_noise(NOISE_SIGMA);    // 添加方位噪声
        photoelectric->elevation_deviation += gaussian_noise(NOISE_SIGMA);   // 添加俯仰噪声
        *interfere_flag = 1;              // 标记干扰触发
        *interfere_start = current_line;   // 记录干扰开始的行号
        *count = *count + 1;
    }
}

// 生成高斯噪声（Box-Muller 变换）
// float gaussian_noise(float sigma) {
//     static int has_spare = 0;
//     static float spare;
//     if (has_spare) {
//         has_spare = 0;
//         return sigma * spare;
//     }
//     has_spare = 1;
//     float u1 = (float)rand() / RAND_MAX;
//     float u2 = (float)rand() / RAND_MAX;
//     float r = sqrt(-2.0 * log(u1));
//     float theta = 2.0 * PI * u2;
//     spare = r * sin(theta);
//     return sigma * r * cos(theta);
// }



// ------------------------------------------------------光电跳跃干扰函数b------------------------------------------------------
// 模拟跟踪到飞机附近虚假目标1（前5行）和虚假目标2（后5行）
// 虚假目标1：俯仰角偏差逐渐增大
// 虚假目标2：从虚假目标1终点开始，俯仰角偏差逐渐减小，最终接近真实目标
// 方位角添加小幅随机噪声
// 光电跳变b干扰函数
void Photoelectric_Jump_around_b(Photoelectric *photoelectric, double start_time, double end_time, double interference_freq, 
    int current_line, int *interfere_flag, int *interfere_start, 
    float *original_azimuth, float *original_elevation,int *count) {
    double photoelectric_time = photoelectric->timestamp + current_line*0.02;
    const float MAX_ELEVATION_OFFSET = 0.005;  // 虚假目标1最大俯仰偏差
    const float AZIMUTH_NOISE_SIGMA = 0.0002;  // 方位角噪声标准差

    // 如果干扰已经触发，检查是否在持续时间内
    if (*interfere_flag >= 1) {
    if (current_line >= *interfere_start && current_line < *interfere_start + 10) {
        if (photoelectric->timestamp != NAN_VALUE) {
            int offset = current_line - *interfere_start;  // 当前行相对于干扰起始行的偏移

            // 保存原始偏差角
            float true_azimuth = photoelectric->azimuth_deviation;
            float true_elevation = photoelectric->elevation_deviation;

            // 方位角始终添加小幅随机噪声
            photoelectric->azimuth_deviation += gaussian_noise(AZIMUTH_NOISE_SIGMA);

            // 前5行：虚假目标1，俯仰角偏差线性增大
            if (offset < 5) {
                float t = (float)offset / 4.0;  // 0到1的线性插值
                float elevation_offset = t * MAX_ELEVATION_OFFSET;
                photoelectric->elevation_deviation = *original_elevation + elevation_offset;
            }
            // 后5行：虚假目标2，俯仰角偏差线性减小
            else {
                float t = (float)(offset - 5) / 4.0;  // 0到1的线性插值
                float elevation_offset = (1.0 - t) * MAX_ELEVATION_OFFSET;
                photoelectric->elevation_deviation = *original_elevation + elevation_offset;
            }
        }
    }
    return;
    }

    // 检查输入数据是否有效
    if (photoelectric->timestamp == NAN_VALUE || photoelectric->work_status == NAN_VALUE ||
    photoelectric->azimuth_deviation == NAN_VALUE || photoelectric->elevation_deviation == NAN_VALUE) {
        return;
    }

    // 检查当前时间是否在干扰时间范围内
    if (photoelectric_time < start_time || photoelectric_time > end_time) {
        return;  // 时间不在干扰范围内，不施加干扰
    }

    // 根据干扰频率决定是否触发干扰
    int rand_num = rand() % 100;  // 生成 0-99 的随机数
    if (rand_num < (int)(interference_freq * 100)) {  // interference_freq 是 0.0-1.0 的概率
        *original_azimuth = photoelectric->azimuth_deviation;    // 记录触发时的真实方位偏差
        *original_elevation = photoelectric->elevation_deviation; // 记录触发时的真实俯仰偏差
        photoelectric->azimuth_deviation += gaussian_noise(AZIMUTH_NOISE_SIGMA); // 首次添加噪声
        photoelectric->elevation_deviation = *original_elevation; // 首次俯仰偏差不变
        *interfere_flag = 1;              // 标记干扰触发
        *interfere_start = current_line;   // 记录干扰开始的行号
        *count = *count + 1;
    }
}

/*------------------------------------------------------------------------------------新干扰--------------------------------------------------------------------------------------------------------*/

void Interfering1(Radar *radar, double start_time, double end_time, int line_count, int *count,double level) {
    // 检查基本参数是否有效
    if (radar->timestamp == NAN_VALUE || radar->work_status == NAN_VALUE ||
        radar->azimuth == NAN_VALUE || radar->elevation == NAN_VALUE ||
        radar->radial_distance == NAN_VALUE) {
        return;
    }
    // printf("line_count:%d\n",line_count);
    // 计算当前雷达时间
    double radar_time = radar->timestamp + line_count * 0.02;

    // 检查当前时间是否在干扰时间范围内
    if (radar_time < start_time || radar_time > end_time) {
        return;  // 如果不在干扰时间段内，直接返回
    }

    // 施加 4 倍精度误差的干扰信号
    float azimuth_error;   // 4 * 1.57 mrad = 6.28 mrad
    double value1 = level * 1.57;
    double value2 = level * 1.67;
    if (random_float(0, 1) < 0.5) {
        azimuth_error = random_float(-1* value1, -3.0); // 50% 概率选 (-6.28, -3)
    } else {
        azimuth_error = random_float(3.0, value1);   // 50% 概率选 (3, 6.28)
    }
    float elevation_error ; // 4 * 1.67 mrad = 6.68 mrad
    if (random_float(0, 1) < 0.5) {
        elevation_error = random_float(-1* value2, -3.0); // 50% 概率选 (-6.28, -3)
    } else {
        elevation_error = random_float(3.0, value2);   // 50% 概率选 (3, 6.28)
    }

    radar->azimuth += azimuth_error / 1000.0;    // 转换为 rad (mrad -> rad)
    radar->elevation += elevation_error / 1000.0; // 转换为 rad (mrad -> rad)
    // radar->work_status = 0.0;                   // 工作状态置为 0
    *count = *count + 1;                        // 干扰次数加 1
    // printf("应用新干扰成功\n");
}

void Interfering2(Photoelectric *photoelectric, double start_time, double end_time, 
    int current_line, int *count) {
    // 检查指针有效性
    if (!photoelectric || !count) {
        return;
    }

    // 计算当前时间
    double photoelectric_time = photoelectric->timestamp + current_line * 0.02;

    // 检查时间范围
    if (photoelectric_time >= start_time && photoelectric_time <= end_time) {
        // 在时间范围内，数据断开，所有字段置为 NAN_VALUE
        photoelectric->timestamp = 0;
        photoelectric->work_status = 0;
        photoelectric->azimuth_deviation = 0;
        photoelectric->elevation_deviation = 0;
        (*count)++; // 记录干扰次数
    }
}

// void Interfering3_1(Photoelectric *photoelectric, double start_time, double end_time, 
//     int current_line, float *fixed_azimuth, float *fixed_elevation, int *count) {
//     // 检查指针有效性
//     if (!photoelectric || !fixed_azimuth || !fixed_elevation || !count) {
//         return;
//     }

//     // 计算当前时间
//     double photoelectric_time = photoelectric->timestamp + current_line * 0.02;

//     // 检查时间范围
//     if (photoelectric_time >= start_time && photoelectric_time <= end_time) {
//         // 如果是第一次进入干扰时间范围，记录当前偏差作为目标位置
//         if (isnan(*fixed_azimuth) || isnan(*fixed_elevation)) {
//             *fixed_azimuth = photoelectric->azimuth_deviation;
//             *fixed_elevation = photoelectric->elevation_deviation;
//         }

//         // 应用干扰：固定偏差到目标位置
//         photoelectric->azimuth_deviation += 0.01;
//         photoelectric->elevation_deviation += 0.01;
//         (*count)++; // 记录干扰次数
//     }
// }

void Interfering3_1(Photoelectric *photoelectric, Servo *servo, Laser *laser,double start_time, double end_time, 
    int current_line, float *fixed_azimuth, float *fixed_elevation, float *alpha, float *beta,float *r, int *count) {
    // 检查指针有效性
    if (!photoelectric || !fixed_azimuth || !fixed_elevation || !count) {
        return;
    }

    // 计算当前时间
    double photoelectric_time = photoelectric->timestamp + current_line * 0.02;
    // 检查时间范围
    if (photoelectric_time >= start_time && photoelectric_time <= end_time) {
        // 如果是第一次进入干扰时间范围，记录当前偏差作为目标位置
        if (!*fixed_azimuth || !*fixed_elevation) {
            *fixed_azimuth = photoelectric->azimuth_deviation;
            *fixed_elevation = photoelectric->elevation_deviation;
            *alpha = servo->azimuth;
            *beta = servo->elevation;
            *r = laser->radial_distance;
        }

        // 应用干扰：固定偏差到目标位置
        photoelectric->azimuth_deviation = *fixed_azimuth;
        photoelectric->elevation_deviation = *fixed_elevation;
        servo->azimuth = *alpha;
        servo->elevation = *beta;
        laser->radial_distance = *r;
        (*count)++; // 记录干扰次数
    }
}

// void Interfering3_2(Photoelectric *photoelectric, Laser *laser, double start_time, double end_time, double freqs,
//     int current_line, float *fixed_azimuth, float *fixed_elevation,float *fixed_r, int *count) {
//     // 检查指针有效性
//     if (!photoelectric || !fixed_azimuth || !fixed_elevation || !count) {
//         return;
//     }

//     // 计算当前时间
//     double photoelectric_time = photoelectric->timestamp + current_line * 0.02;

//      // 检查时间范围
//      if (photoelectric_time < start_time || photoelectric_time > end_time) {
//         return; // 不在干扰时间范围内，直接返回
//     }

//     // 静态变量保存干扰状态
//     static int is_interfering = 0; // 是否正在干扰
//     static double interfere_end_time = 0.0; // 干扰结束时间
//     static float target_azimuth = 0.0; // 固定的目标方位角偏差
//     static float target_elevation = 0.0; // 固定的目标俯仰角偏差
//     static float target_r = 0.0;//固定的目标径向距离

//     // 如果当前不在干扰状态，尝试以概率 freqs 触发新干扰
//     if (!is_interfering) {
//         float rand_prob = (float)rand() / RAND_MAX; // 生成 0-1 的随机概率
//         if (rand_prob <= freqs) { // 以概率 freqs 触发干扰
//             is_interfering = 1; // 进入干扰状态
//             (*count)++; // 记录干扰次数
//             // 随机生成 1-3 秒的干扰持续时间
//             double interfere_duration = 1.0 + ((double)rand() / RAND_MAX) * 2.0;
//             interfere_end_time = photoelectric_time + interfere_duration; // 设置干扰结束时间
//             // 记录当前偏差作为目标位置
//             target_azimuth = photoelectric->azimuth_deviation + 0.01;
//             target_elevation = photoelectric->elevation_deviation + 0.01;
//             target_r = laser->radial_distance;
//             *fixed_azimuth = target_azimuth;
//             *fixed_elevation = target_elevation;
//             *fixed_r = target_r;
//         } else {
//             return; // 未触发干扰，直接返回
//         }
//     }

//     // 如果正在干扰，检查是否超过干扰结束时间
//     if (is_interfering && photoelectric_time > interfere_end_time) {
//         is_interfering = 0; // 结束干扰
//         return;
//     }

//     // 应用干扰：固定偏差到目标位置
//     photoelectric->azimuth_deviation = target_azimuth;
//     photoelectric->elevation_deviation = target_elevation;
//     laser->radial_distance;

// }

//多次跳跟电视屏幕内非雷达跟踪目标,跳跟目标数量范围[10,50],每个目标跟踪持续时间范围[10,80]ms
void Interfering4(Photoelectric *photoelectric, double start_time, double end_time, 
    int current_line, int *interfere_flag, int *interfere_start, int *count,double level) {
    // 计算当前光电设备时间
    double photoelectric_time = photoelectric->timestamp + current_line * 0.02;

    // 检查输入数据是否有效
    if (isnan(photoelectric->timestamp) || isnan(photoelectric->work_status) ||
    isnan(photoelectric->azimuth_deviation) || isnan(photoelectric->elevation_deviation)) {
        return;
    }

    // 检查当前时间是否在干扰时间范围内
    if (photoelectric_time < start_time || photoelectric_time > end_time) {
        return; // 时间不在干扰范围内，不施加干扰
    }

    // 静态变量存储跳跟目标
    static JumpTarget targets[50]; // 最大支持 50 个目标
    static int target_count = 0;

    // 初始化跳跟目标（仅在首次调用时）
    if (*interfere_flag == 0) {
        *interfere_flag = 1;
        *interfere_start = current_line;

        // 随机生成跳跟目标数量 [10, 50]
        target_count = 10 + rand() % 41; // 10 到 50

        // 为每个目标生成随机起始时间和持续时间
        double time_range = end_time - start_time;
        for (int i = 0; i < target_count; i++) {
            targets[i].start_time = start_time + ((double)rand() / RAND_MAX) * time_range;
            // 持续时间 [10, 80] ms 转换为秒
            targets[i].duration = (10 + rand() % 71) / 1000.0; // 0.01 到 0.08 秒
        }
    }

    // 检查当前时间是否有活跃的跳跟目标
    int active_targets = 0;
    for (int i = 0; i < target_count; i++) {
        if (photoelectric_time >= targets[i].start_time && 
        photoelectric_time < targets[i].start_time + targets[i].duration) {
            // 施加随机偏差 [0, 10] mrad
            float azimuth_noise = random_float(0.0, 10.0*level) / 1000.0;  // mrad -> rad
            float elevation_noise = random_float(0.0, 10.0*level) / 1000.0; // mrad -> rad
            photoelectric->azimuth_deviation += azimuth_noise;
            photoelectric->elevation_deviation += elevation_noise;
            active_targets++;
        }
    }

    // 更新 count 为当前活跃目标数量
    *count = active_targets;
}

void Interfering5(Photoelectric *photoelectric, double start_time, double end_time, 
    int current_line, int *interfere_flag, int *interfere_start, 
    float *original_azimuth, float *original_elevation, int *count,double level) {
    // 计算当前光电设备时间
    double photoelectric_time = photoelectric->timestamp + current_line * 0.02;

    // 检查输入数据是否有效
    if (isnan(photoelectric->timestamp) || isnan(photoelectric->work_status) ||
        isnan(photoelectric->azimuth_deviation) || isnan(photoelectric->elevation_deviation)) {
        return;
    }

    // 检查当前时间是否在干扰时间范围内
    if (photoelectric_time < start_time || photoelectric_time > end_time) {
        return; // 时间不在干扰范围内，不施加干扰
    }

    // 静态变量存储跳跟目标
    static JumpTarget2 targets[5]; // 最大支持 5 个目标
    static int target_count = 0;

    // 初始化跳跟目标（仅在首次调用时）
    if (*interfere_flag == 0) {
        *interfere_flag = 1;
        *interfere_start = current_line;

        // 保存原始偏差
        *original_azimuth = photoelectric->azimuth_deviation;
        *original_elevation = photoelectric->elevation_deviation;

        // 随机生成跳跟目标数量 [2, 5]
        target_count = 2 + rand() % 4; // 2 到 5

        // 为每个目标生成随机起始时间、持续时间和偏差
        double time_range = end_time - start_time;
        for (int i = 0; i < target_count; i++) {
            targets[i].start_time = start_time + ((double)rand() / RAND_MAX) * time_range;
            targets[i].duration = random_float(1.0, 3.0); // 持续时间 [1, 3] 秒
            // 初始和终止偏差 [0, 10] mrad，转换为弧度
            targets[i].azimuth_start = random_float(0.0, 10.0*level) / 1000.0;
            targets[i].azimuth_end = random_float(0.0, 10.0*level) / 1000.0;
            targets[i].elevation_start = random_float(0.0, 10.0*level) / 1000.0;
            targets[i].elevation_end = random_float(0.0, 10.0*level) / 1000.0;
        }
    }

    // 检查当前时间是否有活跃的跳跟目标
    int active_targets = 0;
    float total_azimuth_deviation = 0.0;
    float total_elevation_deviation = 0.0;
    for (int i = 0; i < target_count; i++) {
        if (photoelectric_time >= targets[i].start_time && 
            photoelectric_time < targets[i].start_time + targets[i].duration) {
            // 计算插值比例
            double t = (photoelectric_time - targets[i].start_time) / targets[i].duration;
            // 线性插值计算当前偏差
            float azimuth_dev = targets[i].azimuth_start + t * (targets[i].azimuth_end - targets[i].azimuth_start);
            float elevation_dev = targets[i].elevation_start + t * (targets[i].elevation_end - targets[i].elevation_start);
            total_azimuth_deviation += azimuth_dev;
            total_elevation_deviation += elevation_dev;
            // 更新偏差
            photoelectric->azimuth_deviation = *original_azimuth + total_azimuth_deviation;
            photoelectric->elevation_deviation = *original_elevation + total_elevation_deviation;
            active_targets++;
        }
    }



    // 更新 count 为当前活跃目标数量
    *count = active_targets;
}

// void Interfering5(Photoelectric *photoelectric, double start_time, double end_time, 
//     int current_line, int *interfere_flag, int *interfere_start, 
//     float *original_azimuth, float *original_elevation, int *count) {
//     // 计算当前光电设备时间
//     double photoelectric_time = photoelectric->timestamp + current_line * 0.02;

//     // 检查输入数据是否有效
//     if (isnan(photoelectric->timestamp) || isnan(photoelectric->work_status) ||
//         isnan(photoelectric->azimuth_deviation) || isnan(photoelectric->elevation_deviation)) {
//         return;
//     }

//     // 检查当前时间是否在干扰时间范围内
//     if (photoelectric_time < start_time || photoelectric_time > end_time) {
//         return; // 时间不在干扰范围内，不施加干扰
//     }

//     // 静态变量存储跳跟目标
//     static JumpTarget2 targets[5]; // 最大支持 5 个目标
//     static int target_count = 0;

//     // 初始化跳跟目标（仅在首次调用时）
//     if (*interfere_flag == 0) {
//         *interfere_flag = 1;
//         *interfere_start = current_line;

//         // 保存原始偏差
//         *original_azimuth = photoelectric->azimuth_deviation;
//         *original_elevation = photoelectric->elevation_deviation;

//         // 随机生成跳跟目标数量 [2, 5]
//         target_count = 2 + rand() % 4; // 2 到 5

//         // 为每个目标生成随机起始时间、持续时间和偏差
//         double time_range = end_time - start_time;//持续时间

//          // 确保时间范围足够容纳所有目标（每个目标最短 1 秒）
//         if (time_range < target_count * 1.0) {
//             // 如果时间范围太短，调整目标数量
//             target_count = (int)(time_range / 1.0);
//             if (target_count < 2) target_count = 2; // 至少 2 个目标
//         }

//         // 平均分配时间范围给每个目标
//         double segment_duration = time_range / target_count;

//         for (int i = 0; i < target_count; i++) {
//             // 每个目标的子区间为 [start_time + i * segment_duration, start_time + (i+1) * segment_duration)
//             double segment_start = start_time + i * segment_duration;
//             double segment_end = segment_start + segment_duration;

//             // 在子区间内随机生成 start_time
//             // 确保 start_time + duration 不超过 segment_end
//             double max_start = segment_end - 1.0; // 至少留 1 秒给 duration
//             if (max_start < segment_start) max_start = segment_start;
//             targets[i].start_time = segment_start + ((double)rand() / RAND_MAX) * (max_start - segment_start);

//             // 随机生成 duration，确保不超过子区间剩余时间
//             double max_duration = segment_end - targets[i].start_time;
//             if (max_duration > 3.0) max_duration = 3.0; // duration 上限 3 秒
//             if (max_duration < 1.0) max_duration = 1.0; // duration 下限 1 秒
//             targets[i].duration = random_float(1.0, max_duration);

//             // 生成偏差（保持原逻辑）
//             targets[i].azimuth_start = random_float(0.0, 10.0) / 1000.0;
//             targets[i].azimuth_end = random_float(0.0, 10.0) / 1000.0;
//             targets[i].elevation_start = random_float(0.0, 10.0) / 1000.0;
//             targets[i].elevation_end = random_float(0.0, 10.0) / 1000.0;
//         }
//     }

//     // 检查当前时间是否有活跃的跳跟目标
//     int active_targets = 0;
//     float total_azimuth_deviation = 0.0;
//     float total_elevation_deviation = 0.0;
//     for (int i = 0; i < target_count; i++) {
//         if (photoelectric_time >= targets[i].start_time && 
//             photoelectric_time < targets[i].start_time + targets[i].duration) {
//             // 计算插值比例
//             double t = (photoelectric_time - targets[i].start_time) / targets[i].duration;
//             // 线性插值计算当前偏差
//             float azimuth_dev = targets[i].azimuth_start + t * (targets[i].azimuth_end - targets[i].azimuth_start);
//             float elevation_dev = targets[i].elevation_start + t * (targets[i].elevation_end - targets[i].elevation_start);
//             total_azimuth_deviation += azimuth_dev;
//             total_elevation_deviation += elevation_dev;
//             // 更新偏差
//             photoelectric->azimuth_deviation = *original_azimuth + total_azimuth_deviation;
//             photoelectric->elevation_deviation = *original_elevation + total_elevation_deviation;
//             active_targets++;
//         }
//     }

//     // 更新 count 为当前活跃目标数量
//     *count = active_targets;
// }

void Interfering6(Photoelectric *photoelectric, double start_time, double end_time, 
    int current_line, int *interfere_flag, int *interfere_start, int *count,double level) {
    const float AZIMUTH_OFFSET = 0.0015 * level;    // 方位偏差偏移量
    // const float ELEVATION_OFFSET = -0.002; // 俯仰偏差偏移量
    const float ELEVATION_OFFSET = 0.0015 * level; // 俯仰偏差偏移量

    // 计算当前光电设备时间
    double photoelectric_time = photoelectric->timestamp + current_line * 0.02;

    // 检查输入数据是否有效
    if (isnan(photoelectric->timestamp) || isnan(photoelectric->work_status) ||
    isnan(photoelectric->azimuth_deviation) || isnan(photoelectric->elevation_deviation)) {
        return;
    }

    // 检查当前时间是否在干扰时间范围内
    if (photoelectric_time < start_time || photoelectric_time > end_time) {
        return;  // 时间不在干扰范围内，不施加干扰
    }

    // 施加干扰
    // printf("bef:photoelectric->azimuth_deviation=%.4f\n",photoelectric->azimuth_deviation);
    photoelectric->azimuth_deviation += AZIMUTH_OFFSET;    // 施加方位偏移
    // printf("aft:photoelectric->azimuth_deviation=%.4f\n",photoelectric->azimuth_deviation);
    // printf("\n");
    photoelectric->elevation_deviation += ELEVATION_OFFSET; // 施加俯仰偏移

    // 如果是首次干扰，更新标志和计数
    if (*interfere_flag == 0) {
        *interfere_flag = 1;            // 标记干扰开始
        *interfere_start = current_line; // 记录干扰开始的行号
        *count = *count + 1;            // 干扰次数加 1
    }
}


/*---------------------------------------------船摇+海杂波10.30-------------------------------------*/
/* ====================== 辅助：欧拉角 → 旋转矩阵 ====================== */
static void euler_to_rot(float roll, float pitch, float yaw, float R[3][3])
{
    float cr = cosf(roll), sr = sinf(roll);
    float cp = cosf(pitch), sp = sinf(pitch);
    float cy = cosf(yaw), sy = sinf(yaw);

    R[0][0] = cp*cy;           R[0][1] = sr*sp*cy - cr*sy;  R[0][2] = cr*sp*cy + sr*sy;
    R[1][0] = cp*sy;           R[1][1] = sr*sp*sy + cr*cy;  R[1][2] = cr*sp*sy - sr*cy;
    R[2][0] = -sp;             R[2][1] = sr*cp;            R[2][2] = cr*cp;
}

/* ====================== 真实船摇模型 ====================== */
static void get_ship_attitude(double t, float *roll, float *pitch, float *yaw)
{
    const float DEG2RAD = PI / 180.0f;

    float phi_s_deg = 0.5f * sinf(0.6f  * (float)t) +
                      0.3f * sinf(0.63f * (float)t) +
                      0.25f * gaussian_noise(1.0f);

    float alpha_s_deg = 2.5f * sinf(0.5f  * (float)t) +
                        3.0f * sinf(0.52f * (float)t) +
                        0.5f * gaussian_noise(1.0f);

    *pitch = phi_s_deg   * DEG2RAD;
    *roll  = alpha_s_deg * DEG2RAD;
    *yaw   = 0.0f;
}

/* ====================== 核心：地面 → 船载（坐标系适配） ====================== */
void ground_to_ship_measurement(double t,
                                float az_ground, float el_ground, float rng,
                                float *az_ship, float *el_ship)
{
    /* ---------- 1. 空值处理 ---------- */
    if (az_ground == NAN_VALUE || el_ground == NAN_VALUE || rng <= 0.0f) {
        *az_ship = *el_ship = NAN_VALUE;
        return;
    }

    /* ---------- 2. 缓存旋转矩阵 ---------- */
    static float R[3][3] = {{0}}, RT[3][3] = {{0}};
    static double last_t = -1.0;
    if (t != last_t) {
        float roll, pitch, yaw;
        get_ship_attitude(t, &roll, &pitch, &yaw);
        euler_to_rot(roll, pitch, yaw, R);  // 船体 → 世界（标准x前y右）
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                RT[i][j] = R[j][i];  // 世界 → 船体
        last_t = t;
    }

    /* ---------- 3. 【关键】雷达标准 → 你的坐标系 (y前 x右) ---------- */
    float alpha_in = PI/2.0f - az_ground;  // az=0°(前) → alpha=90°(右) → y轴
    float beta_in  = el_ground;

    /* ---------- 4. 你的函数：球坐标 → 笛卡尔（y前 x右） ---------- */
    float x_your, y_your, z_your;
    polar2Descartes(alpha_in, beta_in, rng, &x_your, &y_your, &z_your);

    /* ---------- 5. 你的笛卡尔 → 标准笛卡尔 (y前→x前) ---------- */
    float p_world[3] = { y_your, x_your, z_your };  // 交换 x/y

    /* ---------- 6. 标准流程：世界 → 船体（标准旋转） ---------- */
    float p_ship_std[3];
    for (int i = 0; i < 3; ++i)
        p_ship_std[i] = RT[i][0]*p_world[0] + RT[i][1]*p_world[1] + RT[i][2]*p_world[2];

    /* ---------- 7. 标准笛卡尔 → 你的笛卡尔 ---------- */
    float p_ship_your[3] = { p_ship_std[1], p_ship_std[0], p_ship_std[2] };  // 交换回

    /* ---------- 8. 你的函数：笛卡尔 → 球坐标 ---------- */
    float alpha_out, beta_out, r_out;
    Descartes2polar(p_ship_your[0], p_ship_your[1], p_ship_your[2],
                    &alpha_out, &beta_out, &r_out);

    /* ---------- 9. 你的坐标系 → 雷达标准 ---------- */
    *az_ship = PI/2.0f - alpha_out;
    *el_ship = beta_out;

    /* ---------- 10. 防 NaN ---------- */
    if (isnan(*az_ship) || isnan(*el_ship)) {
        *az_ship = *el_ship = NAN_VALUE;
    }
}
/*------------------------------------------------------------------------------保存干扰后结果----------------------------------------------------------------------------------------------------*/
void Radar_interfering_result_to_file(const char *filename, Radar rader,double sockte_time) {
    FILE *file = fopen(filename, "a");  // 以追加模式打开文件
    if (!file) {
        perror("ERROR opening output file");
        return;
    }
    fprintf(file, "%.4f %.4f %.4f %.4f %.4f \n", rader.timestamp+sockte_time, rader.work_status, rader.azimuth, rader.elevation, rader.radial_distance);
    fclose(file);
}

void Photoelectric_interfering_result_to_file(const char *filename, Photoelectric photoelectric) {
    FILE *file = fopen(filename, "a");  // 以追加模式打开文件
    if (!file) {
        perror("ERROR opening output file");
        return;
    }
    fprintf(file, "%.4f %.4f %.4f %.4f \n", photoelectric.timestamp, photoelectric.work_status, photoelectric.azimuth_deviation, photoelectric.elevation_deviation);
    fclose(file);
}