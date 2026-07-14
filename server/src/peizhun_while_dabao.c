#include <stdio.h>
#include "sensor_data.h"
#include "time_registration.h"
#include "coord_trans.h"
#define MAX_LINES 10000
#define NAN_VALUE -9999.0
#define PI 3.1415926535
#define DT 0.001
#define ALIGNMENT_INTERVAL 0.02
#define POLAR_DT 0.02
#define MAX_INTERFERS 6
#define BUFFER_SIZE 1024

SensorAlignedData* process_sensor_data(
    int line_count,
    float* last_time,
    Laser* laser_data,
    Radar* radar_data1,
    Radar* radar_data2,
    FullPhotoelectric* full_photoelectric_data,
    PolarData* polar_data,
    // FILE* output_file,
    KalmanStateAxis* laser_state,
    KalmanCovAxis* laser_cov,
    KalmanState* radar_state,/**/
    KalmanCov* radar_cov,/**/
    KalmanState* polar_state,/**/
    KalmanCov* polar_cov,/**/
    RadarAlignedData* radar_aligned_data,  // 添加参数
    PolarAlignedData* polar_aligned_data,   // 添加参数
    LaserAlignedData* laser_aligned_data,
    double socket_time
) {
    static SensorAlignedData result[2] = {{0}, {0}};
    
    // 激光数据时间配准
    // if (line_count > 0) {
    //     printf("line_count > 0? %d\n", line_count);
    //     align_laser_data(&laser_data[line_count], socket_time, laser_state, laser_cov, 
    //                 *last_time, &laser_aligned_data[line_count],
    //                 &laser_aligned_data[line_count-1], line_count);
    // } else {
    //     printf("line_count = 0? %d\n", line_count);
    //     // 对于第一行数据，不需要前一个元素
    //     align_laser_data(&laser_data[0], socket_time, laser_state, laser_cov,
    //                     *last_time, &laser_aligned_data[0], NULL, 0);
        
    // }
    // align_laser_data(&laser_data[line_count], socket_time, laser_state, laser_cov, *last_time, &laser_aligned_data[line_count],line_count);
    align_laser_data(&laser_data[line_count], socket_time, laser_state, laser_cov, *last_time,&laser_aligned_data[line_count],&laser_aligned_data[line_count-1],line_count);

    // 结合完整光电数据和配准后的激光数据
    polar_data[line_count].timestamp = full_photoelectric_data[line_count].timestamp+socket_time;
    polar_data[line_count].work_status = full_photoelectric_data[line_count].work_status;
    polar_data[line_count].azimuth = full_photoelectric_data[line_count].azimuth;
    polar_data[line_count].elevation = full_photoelectric_data[line_count].elevation;
    polar_data[line_count].radial_distance = laser_aligned_data[line_count].radial_distance;

    // 雷达数据时间配准
    new_align_radar_data(&radar_data1[line_count], &radar_data2[line_count], socket_time,
                        radar_state, radar_cov, *last_time, &radar_aligned_data[line_count], &radar_aligned_data[line_count-1], line_count);

    // 极坐标数据时间配准
    new_align_polar_data(&polar_data[line_count], socket_time, polar_state, polar_cov, 
                        *last_time, &polar_aligned_data[line_count], &polar_aligned_data[line_count-1], line_count);

    // 将 polar_aligned_data 存入 result[0]
    result[0].timestamp = polar_aligned_data[line_count].timestamp;
    result[0].x = polar_aligned_data[line_count].x;
    result[0].y = polar_aligned_data[line_count].y;
    result[0].z = polar_aligned_data[line_count].z;
    result[0].azimuth = polar_aligned_data[line_count].azimuth;
    result[0].elevation = polar_aligned_data[line_count].elevation;
    result[0].radial_distance = polar_aligned_data[line_count].radial_distance;
    // printf("调试，%.2f\t%.2f\n",result[0].radial_distance,radar_aligned_data[line_count].radial_distance);//调试

    // 将 radar_aligned_data 存入 result[1]
    result[1].timestamp = radar_aligned_data[line_count].timestamp;
    result[1].x = radar_aligned_data[line_count].x;
    result[1].y = radar_aligned_data[line_count].y;
    result[1].z = radar_aligned_data[line_count].z;
    result[1].azimuth = radar_aligned_data[line_count].azimuth;
    result[1].elevation = radar_aligned_data[line_count].elevation;
    result[1].radial_distance = radar_aligned_data[line_count].radial_distance;
    // printf("调试，%.2f\t%.2f\n",result[1].radial_distance,radar_aligned_data[line_count].radial_distance);//调试

    *last_time = socket_time;
    return result;
}