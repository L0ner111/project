#ifndef PEIZHUN_WHILE_DABAO_H
#define PEIZHUN_WHILE_DABAO_H
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
);
#endif