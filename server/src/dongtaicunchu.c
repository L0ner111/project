// dongtaicunchu.c
#include <stdio.h>
#include "dongtaicunchu.h"
#include "sensor_data.h"
// 全局指针定义
Photoelectric *photoelectric_data = NULL;
Laser *laser_data = NULL;
Radar *radar_data1 = NULL;
Radar *radar_data2 = NULL;
Servo *servo_data = NULL;
FullPhotoelectric *full_photoelectric_data = NULL;
PolarData *polar_data = NULL;
RadarAlignedData *radar_aligned_data = NULL;
PolarAlignedData *polar_aligned_data = NULL;
LaserAlignedData *laser_aligned_data = NULL;

// 分配初始内存
int allocate_arrays(size_t initial_size) {
    photoelectric_data = (Photoelectric *)malloc(initial_size * sizeof(Photoelectric));
    laser_data = (Laser *)malloc(initial_size * sizeof(Laser));
    radar_data1 = (Radar *)malloc(initial_size * sizeof(Radar));
    radar_data2 = (Radar *)malloc(initial_size * sizeof(Radar));
    servo_data = (Servo *)malloc(initial_size * sizeof(Servo));
    full_photoelectric_data = (FullPhotoelectric *)malloc(initial_size * sizeof(FullPhotoelectric));
    polar_data = (PolarData *)malloc(initial_size * sizeof(PolarData));
    radar_aligned_data = (RadarAlignedData *)malloc(initial_size * sizeof(RadarAlignedData));
    polar_aligned_data = (PolarAlignedData *)malloc(initial_size * sizeof(PolarAlignedData));
    laser_aligned_data = (LaserAlignedData *)malloc(initial_size * sizeof(LaserAlignedData));

    // 检查分配是否成功
    if (!photoelectric_data || !laser_data || !radar_data1 || !radar_data2 ||
        !servo_data || !full_photoelectric_data || !polar_data ||
        !radar_aligned_data || !polar_aligned_data || !laser_aligned_data) {
        printf("内存分配失败！\n");
        free_arrays(); // 释放已分配的内存
        return 0; // 失败
    }

    return 1; // 成功
}

// 动态调整数组大小
void *resize_array(void *array, size_t *current_size, size_t new_size, size_t element_size) {
    void *temp = realloc(array, new_size * element_size);
    if (!temp) {
        printf("内存重新分配失败！\n");
        free(array);
        return NULL;
    }
    *current_size = new_size;
    return temp;
}

// 检查并调整所有数组大小
void check_and_resize_arrays(size_t *allocated_size, size_t line_count) {
    if (line_count >= *allocated_size) {
        size_t new_size = *allocated_size * 2; // 扩展为当前大小的 2 倍
        // printf("扩展数组：从 %zu 行到 %zu 行\n", *allocated_size, new_size); // 调试日志
        
        photoelectric_data = (Photoelectric *)resize_array(photoelectric_data, allocated_size, new_size, sizeof(Photoelectric));
        laser_data = (Laser *)resize_array(laser_data, allocated_size, new_size, sizeof(Laser));
        radar_data1 = (Radar *)resize_array(radar_data1, allocated_size, new_size, sizeof(Radar));
        radar_data2 = (Radar *)resize_array(radar_data2, allocated_size, new_size, sizeof(Radar));
        servo_data = (Servo *)resize_array(servo_data, allocated_size, new_size, sizeof(Servo));
        full_photoelectric_data = (FullPhotoelectric *)resize_array(full_photoelectric_data, allocated_size, new_size, sizeof(FullPhotoelectric));
        polar_data = (PolarData *)resize_array(polar_data, allocated_size, new_size, sizeof(PolarData));
        radar_aligned_data = (RadarAlignedData *)resize_array(radar_aligned_data, allocated_size, new_size, sizeof(RadarAlignedData));
        polar_aligned_data = (PolarAlignedData *)resize_array(polar_aligned_data, allocated_size, new_size, sizeof(PolarAlignedData));
        laser_aligned_data = (LaserAlignedData *)resize_array(laser_aligned_data, allocated_size, new_size, sizeof(LaserAlignedData));

        // 检查是否所有数组都成功扩展
        if (!photoelectric_data || !laser_data || !radar_data1 || !radar_data2 ||
            !servo_data || !full_photoelectric_data || !polar_data ||
            !radar_aligned_data || !polar_aligned_data || !laser_aligned_data) {
            printf("数组扩展失败，程序退出！\n");
            exit(1);
        }
    }
}

// 释放所有动态分配的内存
void free_arrays() {
    free(photoelectric_data);
    free(laser_data);
    free(radar_data1);
    free(radar_data2);
    free(servo_data);
    free(full_photoelectric_data);
    free(polar_data);
    free(radar_aligned_data);
    free(polar_aligned_data);
    free(laser_aligned_data);

    photoelectric_data = NULL;
    laser_data = NULL;
    radar_data1 = NULL;
    radar_data2 = NULL;
    servo_data = NULL;
    full_photoelectric_data = NULL;
    polar_data = NULL;
    radar_aligned_data = NULL;
    polar_aligned_data = NULL;
    laser_aligned_data = NULL;
}