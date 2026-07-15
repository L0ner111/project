// 添加原始数据的保存操作
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "constant.h"
#include <stdint.h>
#include <time.h>
#include <sys/time.h>
#include "mysql_db.h"

// #define PORT 5005
// #define BUFFER_SIZE 1024
// #define MAX_LINES 100
#define TARGET_IP "127.0.0.1"
// #define TARGET_IP "192.168.2.10"
// #define TARGET_IP "10.22.74.144"
//#define TARGET_IP "169.254.96.236"
#define TARGET_PORT 5005
#define OFFSET -80

#define LOG_FILE "../../save_data/udp_data.txt"
#define LOG_BINARY "../../save_data/udp_binary.txt"
// 保存融合后的极坐标（系统时间 毫秒级, azimuth, elevation）
#define SENSOR_FUSION_LOG "../../save_data/sensor_fusion_polar.txt"

// 将当前本地时间（精确到毫秒）和传入的方位/俯仰追加写入文件，列格式：时间 azimuth elevation
void save_sensor_fusion(const PolarCoord3D *p, const SensorAlignedData *aligned_data) {
    struct timeval tv;
    if (gettimeofday(&tv, NULL) != 0) {
        perror("gettimeofday failed");
        return;
    }
    struct tm tm_info;
    localtime_r(&tv.tv_sec, &tm_info);
    char timestr[64];
    int ms = tv.tv_usec / 1000;
    /* 仅使用 时:分:秒.毫秒，去掉年月日信息 */
    snprintf(timestr, sizeof(timestr), "%02d:%02d:%02d.%03d",
             tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec, ms);

    FILE *f = fopen(SENSOR_FUSION_LOG, "a");
    if (!f) {
        perror("无法打开 sensor fusion 日志文件");
        return;
    }

    // 新的保存顺序：时间, aligned_data[0] 的 (x y z azimuth elevation radial_distance),
    //              aligned_data[1] 的 (x y z azimuth elevation radial_distance), 融合方位 融合俯仰
    fprintf(f, "%s", timestr);
    if (aligned_data) {
        // aligned_data 顺序为 x, y, z, azimuth, elevation, radial_distance
        fprintf(f, " %.6f %.6f %.6f %.6f %.6f %.6f",
                aligned_data[0].x, aligned_data[0].y, aligned_data[0].z,
                aligned_data[0].azimuth, aligned_data[0].elevation, aligned_data[0].radial_distance);
        fprintf(f, " %.6f %.6f %.6f %.6f %.6f %.6f",
                aligned_data[1].x, aligned_data[1].y, aligned_data[1].z,
                aligned_data[1].azimuth, aligned_data[1].elevation, aligned_data[1].radial_distance);
    } else {
        // aligned_data 为 NULL 时写入占位符（12 个字段）
        fprintf(f, " %.6f %.6f %.6f %.6f %.6f %.6f",
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        fprintf(f, " %.6f %.6f %.6f %.6f %.6f %.6f",
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    // 最后写入融合方位和俯仰
    fprintf(f, " %.6f %.6f\n", p->azimuth, p->elevation);
    fclose(f);
}

// #define PORT 8080
// #define BUFFER_SIZE 1313
void save_to_file_binary(int n, const uint16_t *buffer) {
    // 格式化数据为二进制字符串
    char bin_data[BUFFER_SIZE * 17 + 1]; // 每个 uint16_t 需要 16 位 + 空格
    bin_data[0] = '\0';
    for (int i = 0; i < n; i++) {
        char word_str[18]; // 16 位 + 空格 + 空字符
        uint16_t value = buffer[i];
        for (int j = 15; j >= 0; j--) {
            word_str[15 - j] = (value & (1 << j)) ? '1' : '0';
        }
        word_str[16] = ' ';
        word_str[17] = '\0';
        strcat(bin_data, word_str);
    }
    strcat(bin_data, "\n");

    // 打开文件以追加模式写入
    FILE *file = fopen(LOG_BINARY, "a");
    if (!file) {
        perror("无法打开二进制日志文件");
        return;
    }

    // 写入新记录
    if (fputs(bin_data, file) == EOF) {
        fprintf(stderr, "写入二进制数据到文件失败\n");
    }

    fclose(file);
}

void save_to_file(int n, const uint16_t *buffer) {
    // 格式化数据为十六进制字符串
    char hex_data[BUFFER_SIZE * 5 + 1]; // 每个 uint16_t 需要 4 位 + 空格
    hex_data[0] = '\0';
    for (int i = 0; i < n; i++) {
        char word_str[6]; // 4 位十六进制 + 空格 + 空字符
        snprintf(word_str, sizeof(word_str), "%04X ", buffer[i]);
        strcat(hex_data, word_str);
    }
    strcat(hex_data, "\n");

    // 打开文件以追加模式写入
    FILE *file = fopen(LOG_FILE, "a");
    if (!file) {
        perror("无法打开日志文件");
        return;
    }

    // 写入新记录
    if (fputs(hex_data, file) == EOF) {
        fprintf(stderr, "写入十六进制数据到文件失败\n");
    }

    fclose(file);
}


// 缓冲区结构：最小二乘法的估计结果（球坐标）
typedef struct
{
    double time;
    Vector3D position1, position2;
    int isFirst;
} BatchData_Pred;

typedef struct
{
    double time;
    PolarCoord3D sensor1, sensor2, sensor_fusion;
    int isFirst;
} BatchData_LS;

int main() {
    int sockfd;
    struct sockaddr_in serv_addr, cli_addr, target_addr;
    socklen_t cli_len = sizeof(cli_addr);
    uint8_t byte_buffer[BUFFER_SIZE]; // 用于接收字节流
    uint16_t buffer[BUFFER_SIZE / 2]; // 用于存储 uint16_t 数据
    char response[BUFFER_SIZE];
    size_t allocated_size = MAX_LINES;

    // 分配动态数组
    if (!allocate_arrays(allocated_size)) {
        printf("动态内存分配失败！\n");
        return 1;
    }

    // 创建 UDP 套接字
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("ERROR opening socket");
        exit(1);
    }

    // 配置服务器地址（接收端）
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(PORT);

    // 绑定套接字
    if (bind(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("ERROR on binding");
        exit(1);
    }

    // 配置目标服务端地址（发送端）
    memset(&target_addr, 0, sizeof(target_addr));
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(TARGET_PORT);
    inet_pton(AF_INET, TARGET_IP, &target_addr.sin_addr);

    printf("UDP server listening on port %d...\n", PORT);

    /*初始化*/
    uint8_t ldzt, hwcj, dscj, jgcj;//雷达状态，HW测角数据有效性，DS测角数据有效性，JG测距数据有效性。

    // 卡尔曼滤波状态（激光、雷达、极坐标分开）
    static KalmanStateAxis laser_state;
    static KalmanCovAxis laser_cov;
    static KalmanState radar_state, polar_state;
    static KalmanCov radar_cov, polar_cov;
    static double socket_time = 0.02;

    // 调用传感器数据处理（需要静态变量保存卡尔曼滤波状态）
    static float last_time = -0.02;
    static int initialized = 0;

    // 初始化两个跟踪系统
    TrackingSystem system1, system2;
    memset(&system1, 0, sizeof(TrackingSystem));
    memset(&system2, 0, sizeof(TrackingSystem));

    // 设置 IMM 参数
    ImmParameters immParams;
    initializeDefaultImmParameters(&immParams);
    immParams.cvProcessNoise = 100.0;
    immParams.caProcessNoise = 0.1;
    immParams.ctProcessNoise = 1.0;
    immParams.measurementNoise = 100.0;
    immParams.turnRate = 0.1;
    immParams.selfTransProb = 0.95;

    // 初始化传感器1和传感器2的 IMM 模型
    double dt = 0.02;
    initializeImmModelsWithParams(&system1, dt, &immParams);
    initializeImmModelsWithParams(&system2, dt, &immParams);

    // 初始化跟踪
    system1.numTracks = 1;
    system1.nextTrackId = 1;
    system2.numTracks = 1;
    system2.nextTrackId = 1;

    int firstMeasurement = 1;
    int measurementCount = 0;           // 记录测量集的数量
    double range1 = 0.0, range2 = 0.0; // 初始化以避免未定义行为
    
    /*----*/
    int cnt = 0;

    // 单位矩阵
    Matrix_2 H = createIdentityMatrix(MEAS_DIM);      
    // 二乘法估计的传感器1的协方差矩阵
    Matrix_2 P1 = createMatrix(MEAS_DIM, MEAS_DIM);   
    setMatrixElement(&P1, 0, 0, 100);
    setMatrixElement(&P1, 1, 1, 2);
    setMatrixElement(&P1, 2, 2, 2);

    // 二乘法估计的传感器2的协方差矩阵
    Matrix_2 P2 = createMatrix(MEAS_DIM, MEAS_DIM);   
    setMatrixElement(&P2, 0, 0, 100);
    setMatrixElement(&P2, 1, 1, 2);
    setMatrixElement(&P2, 2, 2, 2);
    
    // 用于记录两个量测数据
    Measurement measurement1, measurement2;
    
    // 声明并初始化一个队列0910
    Queue* queue = initQueue();
    double avg = 0;
    int zero_count = 0;
    static int line_count = 0;  // 静态变量保存状态

    /* ===== 初始化 MySQL 连接 ===== */
    MYSQL* db_conn = db_connect();
    if (db_conn == NULL) {
        printf("⚠️ 警告: MySQL 连接失败，将跳过数据库写入，继续运行\n");
        /* 不退出程序，允许离线运行 */
    }

    while (1) {
        cnt ++;
        printf("cnt = %d\n", cnt);
	 	// uint8_t byte_buffer[BUFFER_SIZE]; // 用于接收字节流
        // uint16_t buffer[BUFFER_SIZE / 2]; // 用于存储 uint16_t 数据
        // char response[BUFFER_SIZE];
        // 接收客户端数据
        ssize_t num_bytes = recvfrom(sockfd, byte_buffer, BUFFER_SIZE - 1, 0,
        // ssize_t num_bytes = recvfrom(sockfd, byte_buffer, BUFFER_SIZE - 1, MSG_TRUNC,
                                     (struct sockaddr *)&cli_addr, &cli_len);
        if (num_bytes < 0) {
            perror("ERROR in recvfrom");
            continue;
        }
        printf("Received %ld bytes from client.\n", num_bytes);
        // 确保接收的字节数是 2 的倍数
        if (num_bytes % 2 != 0) {
            printf("Received odd number of bytes (%ld), skipping...\n", num_bytes);
            continue;
        }

        // 将字节流转换为 uint16_t 数组（考虑网络字节序）
        int word_count = num_bytes / 2;
        // printf("字段数量: %d\n", word_count);
        for (int i = 0; i < word_count; i++) {
            buffer[i] = ntohs(((uint16_t)byte_buffer[2 * i] << 8) | byte_buffer[2 * i + 1]);
        }

        // 打印客户端信息
        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &cli_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
        if (buffer[0] != 0x5555) { // 检查第一个 uint16_t 是否为 0x5555
            // printf("%04x\n", buffer[0]);
            // printf("不是目标信息\n");
            continue;
        }

        printf("bytes = %ld, Received binary data from %s:%d:\n", num_bytes, client_ip, ntohs(cli_addr.sin_port));

        // 打印原始十六进制数据
        for (int i = 0; i < word_count; i++) {
            printf("%04X ", buffer[i]);
        }
        printf("\n");

        // 将数据转换为字符串处理
        char hex_str[BUFFER_SIZE * 2 + 1];
        int pos = 0;
        for (int i = 0; i < word_count; i++) {
            pos += sprintf(hex_str + pos, "%04X", buffer[i]);
        }
        hex_str[pos] = '\0';

        // 打印 hex_str 数据
        //  printf("hex_str: %s\n", hex_str);

        // 检查是否以 "0000" 结尾
        size_t len = strlen(hex_str);
        if (len >= 4 && strcmp(hex_str + len - 4, "0000") == 0) {
            hex_str[len - 4] = '\0';
        }
        // printf("处理后hex_str: %s\n", hex_str);

        // 分配二进制数据存储空间
        char *binary = (char *)malloc(1024);
        // char binary[1024];
        if (!binary) {
            printf("内存分配失败！\n");
            close(sockfd);
            return 1;
        }
        memset(binary, 0, 1024);

        // 转换为二进制
        hex_to_bin(hex_str, binary);
        // printf("binary: %s\n", binary);
        if (strcmp(hex_str, "END") == 0) {
            printf("Received END signal, closing connection.\n");
            // free(binary);
            break;
        }

        /* 算法 */
        uint16_t frame[45];
        float values[19];
        




        // 解码二进制数据
        for (int i = 0; i < 45; i++) {
            frame[i] = binaryToUint16(binary, i * 16);
            // printf("frame[%d] = %04X ", i, frame[i]);
        }


        // 检查帧头
        decode_status(frame[3], &ldzt, &hwcj, &dscj, &jgcj);//解码状态字1
        if (frame[0] != 0x5555 || frame[1] != 0xAAAA) {
            printf("%04X   %04X\n", frame[0], frame[1]);
            sprintf(response, "Invalid frame header");
            // return;
            continue;
        }
        printf("Frame header valid.\n");

        // 解码到values数组（与第一段代码一致）
        
        // values[0] = decodeValue(frame[35], 0, 0, 1);  // 时戳1 (光电时间戳)
        values[0] = 0;
        values[1] = 1.0;// 状态字1的D7位 (光电工作状态)
        values[2] = decodeValue(frame[33], 1, 0, 0, 0);  // 电视方位角误差 (光电方位偏差角)
        values[3] = decodeValue(frame[34], 1, 0, 0, 0);  // 电视方位角误差 (光电方位偏差角)

        // 第5列~第6列：激光时间戳 激光径向距离
        values[4] = decodeValue(frame[38], 0, 0, 1, 0);  // 时戳2 (激光时间戳)
        // values[4] = 0;

        values[5] = decodeValue(frame[32], 0, 1, 0, 0);  // 激光测距距离 (激光径向距离)


        // 第7列~第11列：雷达1时间戳 雷达1工作状态 雷达1方位角 雷达1俯仰角 雷达1径向距离
        values[6] = 0;
        values[7] = decodeRadarStatus(frame[12]);  // 雷达跟踪1状态 (雷达1工作状态)
        values[8] = decodeValue(frame[14], 1, 0, 0, 1);  // 雷达测角（方位）1 (雷达1方位角)
        values[9] = decodeValue(frame[15], 1, 0, 0, 1);  // 雷达测角（俯仰）1 (雷达1俯仰角)
        values[10] = decodeValue(frame[13], 0, 1, 0, 1) ; // 雷达跟踪距离1 (雷达1径向距离)

        // 第12列~第16列：雷达2时间戳 雷达2工作状态 雷达2方位角 雷达2俯仰角 雷达2径向距离
        values[11] = decodeValue(frame[21], 0, 0, 1, 1);  // 跟踪数据2时间偏差 (雷达2时间戳)
        values[12] = decodeRadarStatus(frame[17]);  // 雷达跟踪2状态 (雷达2工作状态)
        values[13] = decodeValue(frame[19], 1, 0, 0, 1);  // 雷达测角（方位）2 (雷达2方位角)
        values[14] = decodeValue(frame[20], 1, 0, 0, 1);  // 雷达测角（俯仰）2 (雷达2俯仰角)
        values[15] = decodeValue(frame[18], 0, 1, 0, 1);  // 雷达跟踪距离2 (雷达2径向距离)


        // d:\20s-2\测试c代码出的数据\new_all_data.txt
        values[16] = 0;
        values[17] = decodeValue(frame[36], 1, 0, 0, 1);  // 雷达架位信息（方位） (伺服器方位角)
        values[18] = decodeValue(frame[37], 1, 0, 0, 1);  // 雷达架位信息（俯仰） (伺服器俯仰角)


        // 年月日时分秒的解析
        float nian,yue,ri,shi,fen,miao;
        nian = decodeTime(frame[42],frame[43],frame[44],TIME_YEAR);
        yue = decodeTime(frame[42],frame[43],frame[44],TIME_MONTH);
        ri = decodeTime(frame[42],frame[43],frame[44],TIME_DAY);
        shi = decodeTime(frame[42],frame[43],frame[44],TIME_HOUR);
        fen = decodeTime(frame[42],frame[43],frame[44],TIME_MINUTE);
        miao = decodeTime(frame[42],frame[43],frame[44],TIME_SECOND);
        // printf("nian:%.2f\tyue:%.2f\tri:%.2f\tshi:%.2f\tfen:%.2f\tmiao:%.2f\n\n",nian,yue,ri,shi,fen,miao);ii
        float total_second  = shi * 3600 + fen * 60 + miao;
        // float total_second  = 0.7;


        // 填充传感器数据结构
        photoelectric_data[line_count].timestamp = processTimestamp(values[0]);
        photoelectric_data[line_count].work_status = values[1];
        photoelectric_data[line_count].azimuth_deviation = processAngle(values[2]);
        photoelectric_data[line_count].elevation_deviation = processAngle(values[3]);
        laser_data[line_count].timestamp = processTimestamp(values[4]);
        laser_data[line_count].radial_distance = values[5];
        radar_data1[line_count].timestamp = processTimestamp(values[6]);
        radar_data1[line_count].work_status = values[7];
        radar_data1[line_count].azimuth = processAngle(values[8]);
        radar_data1[line_count].elevation = processAngle(values[9]);
        radar_data1[line_count].radial_distance = values[10] + OFFSET;
        radar_data2[line_count].timestamp = processTimestamp(values[11]);
        radar_data2[line_count].work_status = values[12];
        radar_data2[line_count].azimuth = processAngle(values[13]);
        radar_data2[line_count].elevation = processAngle(values[14]);
        radar_data2[line_count].radial_distance = values[15] + OFFSET;
        servo_data[line_count].timestamp = processTimestamp(values[16]);
        servo_data[line_count].azimuth = processAngle(values[17]);
        servo_data[line_count].elevation = processAngle(values[18]);

        /* ===== 写入解析结果到 MySQL ===== */
        if (db_conn != NULL) {
            db_insert_parsed_data(db_conn, line_count,
                                &photoelectric_data[line_count],
                                &laser_data[line_count],
                                &radar_data1[line_count],
                                &radar_data2[line_count],
                                &servo_data[line_count],
                                &full_photoelectric_data[line_count]);
        }

        /*平滑处理0910*/
        // 向队列中添加元素
        //if(radar_data1[line_count].radial_distance != 110){
        //enqueue(queue, radar_data1[line_count].radial_distance);
       // }
        //if(radar_data2[line_count].radial_distance != 110){
        //    enqueue(queue, radar_data2[line_count].radial_distance);
        //}
       // if(isEmpty(queue) != 1){
        //    avg = getAverage(queue);
       //     radar_data1[line_count].radial_distance = avg;
       // }

        // 新增：处理激光数据为0的情况0910
        if (laser_data[line_count].radial_distance == 0) {
            zero_count++;
            if (zero_count < 10) {
                // 工作状态：使用卡尔曼滤波补值（由 align_laser_data 处理）
                laser_data[line_count].radial_distance = NAN_VALUE; // 保持为0，交给 align_laser_data 处理
            } else {
                // 休息状态：使用雷达1的径向距离
                laser_data[line_count].radial_distance = radar_data1[line_count].radial_distance;
            }
        } else {
            zero_count = 0; // 非零值，重置计数器
        }

        // 光电
        if(dscj==0){
            full_photoelectric_data[line_count].timestamp = 0;
            full_photoelectric_data[line_count].work_status = 0;
            full_photoelectric_data[line_count].azimuth = 90;
            full_photoelectric_data[line_count].elevation = 90;
        }else{
            full_photoelectric_data[line_count].timestamp = photoelectric_data[line_count].timestamp;
            full_photoelectric_data[line_count].work_status = photoelectric_data[line_count].work_status;
            full_photoelectric_data[line_count].azimuth = photoelectric_data[line_count].azimuth_deviation + servo_data[line_count].azimuth;
            full_photoelectric_data[line_count].elevation = photoelectric_data[line_count].elevation_deviation + servo_data[line_count].elevation;
        }

        float az_radar_ship, el_radar_ship, az_ele_ship, el_ele_ship;
        ground_to_ship_measurement(socket_time, radar_data1[line_count].azimuth, radar_data1[line_count].elevation, radar_data1[line_count].radial_distance, &az_radar_ship, & el_radar_ship);
        ground_to_ship_measurement(socket_time, full_photoelectric_data[line_count].azimuth, full_photoelectric_data[line_count].elevation, laser_data[line_count].radial_distance, &az_ele_ship, &el_ele_ship);

        // printf("az_radar_ship = %f\n", az_radar_ship);

        if (!initialized) {
            kalman_init_laser(&laser_state, &laser_cov);
            new_kalman_init_radar(&radar_state, &radar_cov);
            new_kalman_init_polar(&polar_state, &polar_cov);
            initialized = 1;
        }

        SensorAlignedData* aligned_data = process_sensor_data(
                line_count,
                &last_time,
                laser_data,
                radar_data1,
                radar_data2,
                full_photoelectric_data,
                polar_data,
                // output_file,
                &laser_state,
                &laser_cov,
                &radar_state,
                &radar_cov,
                &polar_state,
                &polar_cov,
                radar_aligned_data,  // 添加参数
                polar_aligned_data,   // 添加参数
                laser_aligned_data,
                socket_time
            );

       

        measurement1.position.x = aligned_data[0].x;
        measurement1.position.y = aligned_data[0].y;
        measurement1.position.z = aligned_data[0].z;
        // printf("measurement1.position.x = %f\n", measurement1.position.x);
        // printf("measurement1.position.y = %f\n", measurement1.position.y);
        // printf("measurement1.position.z = %f\n", measurement1.position.z);

        measurement1.time = aligned_data[0].timestamp;
        measurement1.id = 0;

        measurement2.position.x = aligned_data[1].x;
        measurement2.position.y = aligned_data[1].y;
        measurement2.position.z = aligned_data[1].z;
        measurement2.time = aligned_data[0].timestamp;
        measurement2.id = 0;

        if(fabs(aligned_data[0].radial_distance - aligned_data[1].radial_distance) > 5.0) {
            measurement1.position.x = aligned_data[1].x;
            measurement1.position.y = aligned_data[1].y;
            measurement1.position.z = aligned_data[1].z;
        }

        // printf("measurement2.position.x = %f\n", measurement2.position.x);
        // printf("measurement2.position.y = %f\n", measurement2.position.y);
        // printf("measurement2.position.z = %f\n", measurement2.position.z);
        //   measurement1 = {{aligned_data[0].x, aligned_data[0].y, aligned_data[0].z}, aligned_data[0].timestamp, 0};
        //   measurement2 = {{aligned_data[1].x, aligned_data[1].y, aligned_data[1].z}, aligned_data[0].timestamp, 0};

        if (firstMeasurement)
        {
            initializeImmTrackFromMeasurement(&system1, &system1.tracks[0], &measurement1, 0);
            initializeImmTrackFromMeasurement(&system2, &system2.tracks[0], &measurement2, 0);

            // 提取两个传感器对应的数据：位置信息和速度信息以及阈值信息
            Vector3D position1, velocity1, position2, velocity2;
            extractStateInfo(&system1.tracks[0].x, &position1, &velocity1);
            // double gateThreshold1 = range1 * 0.00167 * 120;
            extractStateInfo(&system2.tracks[0].x, &position2, &velocity2);
            // double gateThreshold2 = range2 * 0.00167 * 120;

            /*------------------最小二乘法融合------------------*/
            // 航及关联得到的笛卡尔位置信息转成球坐标系下的位置信息
            PolarCoord3D sensor1_polar = cartesianToPolar(position1);
            printf("sensor1: distance = %lf, azimuth = %lf, elevation = %lf\n", sensor1_polar.distance, sensor1_polar.azimuth, sensor1_polar.elevation);
            PolarCoord3D sensor2_polar = cartesianToPolar(position2);
            printf("sensor2: distance = %lf, azimuth = %lf, elevation = %lf\n", sensor2_polar.distance, sensor2_polar.azimuth, sensor2_polar.elevation);

            // 记录融合后的状态信息
            Matrix_2 sensor_fusion = createMatrix(3, 1);
            // 记录融合后的协方差矩阵
            Matrix_2 P_fusion = createMatrix(3, 3);
            ditui_ls(&sensor_fusion, &P_fusion, &sensor1_polar, &sensor2_polar, &H, &P1, &P2);
            PolarCoord3D sensor_fusion_polar = {sensor_fusion.data[0], sensor_fusion.data[1], sensor_fusion.data[2]};
            printf("ditui_ls and fusion finish!\n");

            freeMatrix(&sensor_fusion);
            freeMatrix(&P_fusion);
            firstMeasurement = 0;
            measurementCount++;
        }

        // 处理后续测量点
        Track *track1 = &system1.tracks[0];
        Track *track2 = &system2.tracks[0];

        // ====================== 传感器1数据 ======================
        double mixProb1[MAX_MODELS][MAX_MODELS];
        calculateMixingProbabilities(&system1, track1, mixProb1);  // 混合概率计算
        calculateMixedEstimates(&system1, track1, mixProb1);       // 混合估计结果
        immModePrediction(&system1, track1);                       // 每个模型进行kalman滤波的预测阶段

        /*=======这个应该有问题，预测的结果并没有记录在混合的x对象中，=========*/
        // 提取预测的状态信息(基于匀速模型)
        Vector3D predictedPosition1, predictedVelocity1;
        extractStateInfo(&track1->modelStates[0], &predictedPosition1, &predictedVelocity1);  // 用匀速模型的预测
        // extractStateInfo(&track1->x, &predictedPosition1, &predictedVelocity1);  // 有问题，track->x并没有被更新，可以理解为当前的结果和下一时刻的差距

        // 求阈值作为门限筛选条件
        double gateThreshold1 = compute_gate_threshold(
            predictedPosition1.x, predictedPosition1.y, predictedPosition1.z, 0.00167 * GATE_TV);
        double distance1 = calculateDistance(&predictedPosition1, &measurement1.position);
        /*=========================================================*/

        // ====================== 传感器2处理 ======================
        double mixProb2[MAX_MODELS][MAX_MODELS];
        calculateMixingProbabilities(&system2, track2, mixProb2);
        calculateMixedEstimates(&system2, track2, mixProb2);
        immModePrediction(&system2, track2);

        Vector3D predictedPosition2, predictedVelocity2;
        extractStateInfo(&track2->modelStates[0], &predictedPosition2, &predictedVelocity2);
        // extractStateInfo(&track2->x, &predictedPosition2, &predictedVelocity2);  // 有问题，track->x并没有被更新，可以理解为当前的结果和下一时刻的差距

        double gateThreshold2 = compute_gate_threshold(
            predictedPosition2.x, predictedPosition2.y, predictedPosition2.z, 0.00167 * GATE_RADAR);
        double distance2 = calculateDistance(&predictedPosition2, &measurement2.position);
        /*=========================================================*/


        // 记录更新前的参数
        Track temp_track1 = *track1;
        Track temp_track2 = *track2;

        if (distance1 <= gateThreshold1 || cnt <= 200)
        {
            immModeUpdate(&system1, track1, &measurement1);
            // printf("Sensor 1, Measurement %d accepted: distance=%.2f, threshold=%.2f\n",
                //    measurementCount, distance1, gateThreshold1);
            printf("Sensor 1, Measurement %d accepted: distance=%.2f\n",
                   measurementCount, distance1);
        } 
        else
        {
            track1 = &temp_track2;
            immModeUpdate(&system1, track1, &measurement2);
            printf("rejected measurement1, change accepted measurement2\n");
        }

        // 提取航迹关联的估计结果中的位置和速度
        Vector3D position1, velocity1;
        extractStateInfo(&track1->x, &position1, &velocity1);
        // =======================================================

        // if (distance2 <= gateThreshold2 || cnt <= 50000)
        // {
        //     immModeUpdate(&system2, track2, &measurement2);
            // printf("Sensor 2, Measurement %d accepted: distance=%.2f, threshold=%.2f\n",
                //    measurementCount, distance2, gateThreshold2);
        //     printf("Sensor 2, Measurement %d accepted: distance=%.2f\n",
        //            measurementCount, distance2);
        // }
        // else
        // {
        //     track2 = &temp_track1;
        //     immModeUpdate(&system2, track2, &measurement1);
        //     printf("rejected measurement2, change accepted measurement1\n");
        // }

        immModeUpdate(&system2, track2, &measurement2);
        printf("Sensor 2, Measurement %d accepted: distance=%.2f\n",
                measurementCount, distance2);

        Vector3D position2, velocity2;
        extractStateInfo(&track2->x, &position2, &velocity2);

        /*--------------------最小二乘法估计和融合--------------------*/
        // 航及关联得到的笛卡尔坐标位置信息转成球坐标系下的位置信息
        PolarCoord3D sensor1_polar = cartesianToPolar(position1);
        PolarCoord3D sensor2_polar = cartesianToPolar(position2);

        // 记录融合后的状态信息
        Matrix_2 sensor_fusion = createMatrix(3, 1);
        // 记录融合后的协方差矩阵
        Matrix_2 P_fusion = createMatrix(3, 3);
        ditui_ls(&sensor_fusion, &P_fusion, &sensor1_polar, &sensor2_polar, &H, &P1, &P2);
        PolarCoord3D sensor_fusion_polar = {sensor_fusion.data[0], sensor_fusion.data[1], sensor_fusion.data[2]};

        double azimuth_error_fusion_radar = fabs(sensor_fusion_polar.azimuth - sensor2_polar.azimuth);
        double elevation_error_fusion_radar = fabs(sensor_fusion_polar.elevation - sensor2_polar.elevation);
        if (azimuth_error_fusion_radar >= 0.00157 || elevation_error_fusion_radar >= 0.00157) 
        {
            sensor_fusion_polar = sensor2_polar;
        }
        if (fabs(sensor_fusion_polar.azimuth) >= 1)
        {
            sensor_fusion_polar = sensor2_polar;
        }

        // 将融合后的极坐标信息转换成笛卡尔坐标
        Vector3D fusion_xyz =  polarToCartesian(sensor_fusion_polar);
        // 将融合后的方位/仰角与本地时间（毫秒级）保存到文本文件，列：系统时间 方位 俯仰
        save_sensor_fusion(&sensor_fusion_polar, aligned_data);

        /* ===== 写入融合结果到 MySQL ===== */
        if (db_conn != NULL) {
            db_insert_fusion_result(db_conn, line_count++,
                                    &sensor1_polar, 
                                    &sensor2_polar,
                                    &sensor_fusion_polar, 
                                    aligned_data);
        }


        // 极坐标信息
        float r_jiguang = values[5], a_tv = values[2] + values[17], e_tv = values[3] + values[18];
        float r_radar = values[10], a_radar = values[8], e_radar = values[9];

        // 使用系统当前时间（时:分:秒.毫秒）
        struct timeval tv_now;
        if (gettimeofday(&tv_now, NULL) != 0) {
            perror("gettimeofday failed");
        }
        struct tm tm_now;
        localtime_r(&tv_now.tv_sec, &tm_now);
        int ms_now = tv_now.tv_usec / 1000;
        /* 将时分秒毫秒统一转换为以秒为单位的浮点数 */
        double time_seconds = tm_now.tm_hour * 3600.0 + tm_now.tm_min * 60.0 + tm_now.tm_sec + ms_now / 1000.0;
        // printf("时间信息: time_seconds: %.3f\n", time_seconds);

        printf("sensor_fusion_polar.elevation, = %f\n", sensor_fusion_polar.elevation);

        // 第一个字段为以秒为单位的浮点数时间，后面字段保持原有数值顺序和格式
        // snprintf(response, BUFFER_SIZE, "%.3f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
        //             time_seconds,
        //             values[8]*PI/180,
        //             (values[2]+values[17])*PI/180,
        //             sensor_fusion_polar.azimuth,
        //             values[9]*PI/180,
        //             (values[3] + values[18])*PI/180,
        //             sensor_fusion_polar.elevation,
        //             values[10] + OFFSET,
        //             values[5],
        //             az_radar_ship,
        //             el_radar_ship,
        //             az_ele_ship, 
        //             el_ele_ship);
        snprintf(response, BUFFER_SIZE, "%.3f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                    time_seconds,
                    aligned_data[1].azimuth,
                    aligned_data[0].azimuth,
                    sensor_fusion_polar.azimuth,
                    aligned_data[1].elevation,
                    aligned_data[0].elevation,
                    sensor_fusion_polar.elevation,
                    aligned_data[1].radial_distance,
                    aligned_data[0].radial_distance,
                    az_radar_ship,
                    el_radar_ship,
                    az_ele_ship, 
                    el_ele_ship);
        

        // 发送 response 到目标服务端
        ssize_t sent_bytes = sendto(sockfd, response, strlen(response), 0,
                                    (struct sockaddr *)&target_addr, sizeof(target_addr));
        if (sent_bytes < 0) {
            perror("ERROR in sendto");
        } else {
            printf("Sent response to %s:%d: \n%s\n", TARGET_IP, TARGET_PORT, response);
        }
        socket_time+=0.02;
        // printf("Processed result: %s\n", response);
        // cnt ++;
        // printf("cnt = %d\n", cnt);

        

        // 保存数据到文件
        save_to_file(word_count, buffer);
        save_to_file_binary(word_count, buffer);
        printf("Data saved to file.\n");

        free(binary);
    }

    /* ===== 关闭 MySQL 连接 ===== */
    db_disconnect(db_conn);

    close(sockfd);
    return 0;
}
