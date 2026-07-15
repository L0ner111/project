#ifndef MYSQL_DB_H
#define MYSQL_DB_H

#include <mysql/mysql.h>
#include "sensor_data.h"
#include "types.h"

/* ===== 数据库连接配置 ===== */
#define DB_HOST     "127.0.0.1"
#define DB_PORT     3306
#define DB_USER     "navicat_user"
#define DB_PASS     "123456"
#define DB_NAME     "study_db"

/* ===== 函数声明 ===== */

/**
 * @brief 初始化 MySQL 连接
 * @return 成功返回 MYSQL 连接句柄，失败返回 NULL
 * 
 * 知识点：MYSQL 是 MySQL C API 的核心结构体，
 * 所有数据库操作都需要通过它来完成。
 */
MYSQL* db_connect(void);

/**
 * @brief 关闭 MySQL 连接
 * @param conn 连接句柄
 */
void db_disconnect(MYSQL* conn);

/**
 * @brief 插入一条报文解析结果
 * @param conn       数据库连接
 * @param line_count 当前行号
 * @param photo      光电数据
 * @param laser      激光数据
 * @param radar1     雷达1数据
 * @param radar2     雷达2数据
 * @param servo      伺服器数据
 * @param full_photo 完整光电数据
 * @return 成功返回0，失败返回-1
 * 
 * 知识点：使用参数化方式防止 SQL 注入和格式化字符串问题。
 * 虽然这里都是数值，但养成参数化查询的习惯很重要。
 */
int db_insert_parsed_data(MYSQL* conn, 
                          int line_count,
                          const Photoelectric* photo,
                          const Laser* laser,
                          const Radar* radar1,
                          const Radar* radar2,
                          const Servo* servo,
                          const FullPhotoelectric* full_photo);

/**
 * @brief 插入一条融合结果
 * @param conn         数据库连接
 * @param line_count   当前行号
 * @param sensor1      传感器1航迹(极坐标)
 * @param sensor2      传感器2航迹(极坐标)
 * @param fusion       融合结果(极坐标)
 * @param aligned_data 配准后的传感器数据(含笛卡尔坐标)
 * @return 成功返回0，失败返回-1
 */
int db_insert_fusion_result(MYSQL* conn,
                            int line_count,
                            const PolarCoord3D* sensor1,
                            const PolarCoord3D* sensor2,
                            const PolarCoord3D* fusion,
                            const SensorAlignedData* aligned_data);

#endif /* MYSQL_DB_H */
