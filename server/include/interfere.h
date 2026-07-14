#ifndef INTERFERE_H
#define INTERFERE_H
#include "sensor_data.h"
// 生成标准正态分布随机数（Box-Muller变换）
float gaussian_noise(float sigma);
// 生成指定范围内的随机浮点数
float random_float(float min, float max);

// 雷达干扰函数
void Radar_Interfering(Radar *radar, double start_time, double end_time, double interfere_freq, int line_count,int *count);


// 光电跟丢干扰函数
void Photoelectric_Lost_with(Photoelectric *photoelectric, double start_time, double end_time, double interference_freq, 
    int current_line, int *interfere_count, int *interfere_start,int *count);




// 光电拉偏a干扰函数
void Photoelectric_pull_a(Photoelectric *photoelectric, double start_time, double end_time, double interference_freq, 
    int current_line, int *interfere_flag, int *interfere_start, 
    float *fixed_azimuth, float *fixed_elevation,int *count);


// 光电拉偏b干扰函数
void Photoelectric_pull_b(Photoelectric *photoelectric, double start_time, double end_time, double interference_freq, 
    int current_line, int *interfere_flag, int *interfere_start,int *count);


// 光电跳变a干扰函数
void Photoelectric_Jump_around_a(Photoelectric *photoelectric, double start_time, double end_time, double interference_freq, 
    int current_line, int *interfere_flag, int *interfere_start,int *count);


// 光电跳变b干扰函数
void Photoelectric_Jump_around_b(Photoelectric *photoelectric, double start_time, double end_time, double interference_freq, 
    int current_line, int *interfere_flag, int *interfere_start, 
    float *original_azimuth, float *original_elevation,int *count);



void Interfering1(Radar *radar, double start_time, double end_time, int line_count, int *count, double level);

void Interfering2(Photoelectric *photoelectric, double start_time, double end_time, 
    int current_line, int *count);

void Interfering3_1(Photoelectric *photoelectric, Servo *servo, Laser *laser,double start_time, double end_time, 
    int current_line, float *fixed_azimuth, float *fixed_elevation, float *alpha, float *beta,float *r, int *count);

    void Interfering3_2(Photoelectric *photoelectric, Laser *laser, double start_time, double end_time, double freqs,
        int current_line, float *fixed_azimuth, float *fixed_elevation,float *fixed_r, int *count);    

void Interfering4(Photoelectric *photoelectric, double start_time, double end_time, 
    int current_line, int *interfere_flag, int *interfere_start, int *count,double level);

void Interfering5(Photoelectric *photoelectric, double start_time, double end_time, 
    int current_line, int *interfere_flag, int *interfere_start, 
    float *original_azimuth, float *original_elevation, int *count,double level);

void Interfering6(Photoelectric *photoelectric, double start_time, double end_time, 
    int current_line, int *interfere_flag, int *interfere_start, int *count, double level);

/*---------------------------------------------------------------------船摇和海杂波10.30---------------------------------------------------------------------*/

void ground_to_ship_measurement(double t,
                                float az_ground, float el_ground, float rng,
                                float *az_ship, float *el_ship);


/*------------------------------------------------------------------------------保存干扰后结果----------------------------------------------------------------------------------------------------*/
void Radar_interfering_result_to_file(const char *filename, Radar rader,double sockte_time) ;

void Photoelectric_interfering_result_to_file(const char *filename, Photoelectric photoelectric);
#endif