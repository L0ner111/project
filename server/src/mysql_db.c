#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mysql_db.h"
#include "types.h"

/**
 * ===== 初始化数据库连接 =====
 * 
 * 流程：
 * 1. mysql_init()     — 分配并初始化 MYSQL 句柄
 * 2. mysql_real_connect() — 建立 TCP 连接到 MySQL 服务器
 * 3. 设置字符集为 utf8mb4
 */
MYSQL* db_connect(void) {
    MYSQL* conn = mysql_init(NULL);
    if (conn == NULL) {
        fprintf(stderr, "mysql_init() 失败: 内存不足\n");
        return NULL;
    }

    /* 
     * mysql_real_connect 参数说明：
     *   1. MYSQL 句柄
     *   2. 主机名（127.0.0.1 表示本机）
     *   3. 用户名
     *   4. 密码
     *   5. 数据库名
     *   6. 端口号（MySQL 默认 3306）
     *   7. Unix socket（NULL 表示使用 TCP）
     *   8. 客户端标志位（0 表示默认）
     */
    if (mysql_real_connect(conn, DB_HOST, DB_USER, DB_PASS, 
                           DB_NAME, DB_PORT, NULL, 0) == NULL) {
        fprintf(stderr, "mysql_real_connect() 失败: %s\n", 
                mysql_error(conn));
        mysql_close(conn);
        return NULL;
    }

    /* 设置字符集 — 支持中文 */
    if (mysql_set_character_set(conn, "utf8mb4")) {
        fprintf(stderr, "设置字符集失败: %s\n", mysql_error(conn));
    }

    printf("✅ MySQL 连接成功: %s@%s:%d/%s\n", 
           DB_USER, DB_HOST, DB_PORT, DB_NAME);
    return conn;
}

/**
 * ===== 关闭数据库连接 =====
 */
void db_disconnect(MYSQL* conn) {
    if (conn) {
        mysql_close(conn);
        printf("✅ MySQL 连接已关闭\n");
    }
}

/**
 * ===== 插入报文解析结果 =====
 * 
 * 知识点—mysql_real_query 与 mysql_query 的区别：
 *   - mysql_query() 只能处理 C 字符串（遇到 \0 截断）
 *   - mysql_real_query() 需要显式指定长度，可以处理二进制数据
 *   - 我们使用 mysql_real_query() 因为它更安全
 * 
 * 知识点—snprintf 格式化 SQL：
 *   用 snprintf 构造 SQL 字符串时，注意缓冲区大小。
 *   对于浮点数，%f 可能输出 "nan" 或 "inf"，MySQL 不接受这些，
 *   所以我们需要用 isnormal() 检查并替换为 NULL。
 */
int db_insert_parsed_data(MYSQL* conn,
                          int line_count,
                          const Photoelectric* photo,
                          const Laser* laser,
                          const Radar* radar1,
                          const Radar* radar2,
                          const Servo* servo,
                          const FullPhotoelectric* full_photo) {
    if (conn == NULL) return -1;

    /*
     * 辅助宏：将 float 转为 SQL 可接受的字面值
     * 如果值不是常规浮点数（NaN, Inf），输出 NULL
     */
    #define FLOAT_OR_NULL(v) (isnormal(v) || v == 0.0f ? \
        ({ char _b[32]; snprintf(_b, sizeof(_b), "%.6f", v); _b; }) : "NULL")

    char query[2048];
    int len = snprintf(query, sizeof(query),
        "INSERT INTO parsed_sensor_data "
        "(line_count, "
        " photo_timestamp, photo_work_status, photo_azim_dev, photo_elev_dev, "
        " laser_timestamp, laser_radial_dist, "
        " radar1_timestamp, radar1_work_status, radar1_azimuth, "
        " radar1_elevation, radar1_radial_dist, "
        " radar2_timestamp, radar2_work_status, radar2_azimuth, "
        " radar2_elevation, radar2_radial_dist, "
        " servo_timestamp, servo_azimuth, servo_elevation, "
        " full_photo_timestamp, full_photo_work_status, "
        " full_photo_azimuth, full_photo_elevation) "
        "VALUES (%d, "
        " %.6f, %.6f, %.6f, %.6f, "
        " %.6f, %.6f, "
        " %.6f, %.6f, %.6f, %.6f, %.6f, "
        " %.6f, %.6f, %.6f, %.6f, %.6f, "
        " %.6f, %.6f, %.6f, "
        " %.6f, %.6f, %.6f, %.6f)",
        line_count,
        photo->timestamp, photo->work_status, 
        photo->azimuth_deviation, photo->elevation_deviation,
        laser->timestamp, laser->radial_distance,
        radar1->timestamp, radar1->work_status, 
        radar1->azimuth, radar1->elevation, radar1->radial_distance,
        radar2->timestamp, radar2->work_status,
        radar2->azimuth, radar2->elevation, radar2->radial_distance,
        servo->timestamp, servo->azimuth, servo->elevation,
        full_photo->timestamp, full_photo->work_status,
        full_photo->azimuth, full_photo->elevation);

    if (len < 0 || len >= (int)sizeof(query)) {
        fprintf(stderr, "❌ SQL 语句构造失败: 缓冲区不足\n");
        return -1;
    }

    /*
     * 执行 SQL
     * mysql_real_query 返回值：0 成功，非0 失败
     */
    if (mysql_real_query(conn, query, len) != 0) {
        fprintf(stderr, "❌ 插入解析结果失败: %s\n", mysql_error(conn));
        fprintf(stderr, "   SQL: %s\n", query);
        return -1;
    }

    return 0;
}

/**
 * ===== 插入融合结果 =====
 */
int db_insert_fusion_result(MYSQL* conn,
                            int line_count,
                            const PolarCoord3D* sensor1,
                            const PolarCoord3D* sensor2,
                            const PolarCoord3D* fusion,
                            const SensorAlignedData* aligned_data) {
    if (conn == NULL) return -1;

    char query[2048];
    int len = snprintf(query, sizeof(query),
        "INSERT INTO fusion_results "
        "(line_count, "
        " sensor1_distance, sensor1_azimuth, sensor1_elevation, "
        " sensor2_distance, sensor2_azimuth, sensor2_elevation, "
        " fusion_distance, fusion_azimuth, fusion_elevation, "
        " aligned1_x, aligned1_y, aligned1_z, "
        " aligned1_azimuth, aligned1_elevation, aligned1_radial_dist, "
        " aligned2_x, aligned2_y, aligned2_z, "
        " aligned2_azimuth, aligned2_elevation, aligned2_radial_dist) "
        "VALUES (%d, "
        " %.6f, %.6f, %.6f, "
        " %.6f, %.6f, %.6f, "
        " %.6f, %.6f, %.6f, "
        " %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, "
        " %.6f, %.6f, %.6f, %.6f, %.6f, %.6f)",
        line_count,
        sensor1->distance, sensor1->azimuth, sensor1->elevation,
        sensor2->distance, sensor2->azimuth, sensor2->elevation,
        fusion->distance, fusion->azimuth, fusion->elevation,
        aligned_data[0].x, aligned_data[0].y, aligned_data[0].z,
        aligned_data[0].azimuth, aligned_data[0].elevation, 
        aligned_data[0].radial_distance,
        aligned_data[1].x, aligned_data[1].y, aligned_data[1].z,
        aligned_data[1].azimuth, aligned_data[1].elevation, 
        aligned_data[1].radial_distance);

    if (len < 0 || len >= (int)sizeof(query)) {
        fprintf(stderr, "❌ SQL 语句构造失败: 缓冲区不足\n");
        return -1;
    }

    if (mysql_real_query(conn, query, len) != 0) {
        fprintf(stderr, "❌ 插入融合结果失败: %s\n", mysql_error(conn));
        fprintf(stderr, "   SQL: %s\n", query);
        return -1;
    }

    return 0;
}

#undef FLOAT_OR_NULL
