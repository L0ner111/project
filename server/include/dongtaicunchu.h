// dongtaicunchu.h
#ifndef DONGTAICUNCHU_H
#define DONGTAICUNCHU_H

#include <stdlib.h>
#include "sensor_data.h"
// 初始行数
#define INITIAL_LINES 1000

// 全局指针声明（extern 避免在头文件中定义变量）
extern Photoelectric *photoelectric_data;
extern Laser *laser_data;
extern Radar *radar_data1;
extern Radar *radar_data2;
extern Servo *servo_data;
extern FullPhotoelectric *full_photoelectric_data;
extern PolarData *polar_data;
extern RadarAlignedData *radar_aligned_data;
extern PolarAlignedData *polar_aligned_data;
extern LaserAlignedData *laser_aligned_data;

// 函数声明
int allocate_arrays(size_t initial_size);
void check_and_resize_arrays(size_t *allocated_size, size_t line_count);
void free_arrays();

#endif // DONGTAICUNCHU_H