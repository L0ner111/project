#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "sensor_data.h"
#include "dec2bin.h"
#include "constant.h"
// 函数：从二进制字符串中提取16位数据
uint16_t binaryToUint16(const char *binary, int offset) {
    uint16_t value = 0;
    for (int i = 0; i < 16; i++) {
        value = (value << 1) | (binary[offset + i] - '0');
    }
    return value;
}

// 函数：将16位数据解码为浮点数（时间戳为有符号）
float decodeValue(uint16_t value, int isAngle, int isDistance, int isTimestamp, int flag) {
    // if (value == 65535) {  // NaN 表示
    //     return NAN_VALUE;
    // }

    // if (isAngle) {  // 角度解码：value * 360 / 65536
    //     return (float)value * 360.0f / 65536.0f;
    // }

    if (isAngle) {  // 角度解码
        if (value <= 32768) {
            // [0, 32768] 映射到 [0°, 180°]
            return (float)value * 180.0f / 32768.0f;
        } else {
            // [32768, 65535] 映射到 [-180°, 0°]
            return (float)value * 180.0f / 32768.0f - 360.0f;
        }
    }
    
    // 非角度值（包括距离和时间戳）使用相同的映射解码
    int16_t signed_value ; // 逆映射：value - 32768
    if (isDistance) {  // 距离解码：应用缩放因子
        if(flag){
            return (float)value * 1.8735f;
        }else   return (float)value * 1.875f;

    }
    if (isTimestamp) {  // 时间戳解码：返回有符号整数
        signed_value = (int16_t)(value - 32768); // 逆映射：value - 32768
        return (float)signed_value;
    }
    return (float)signed_value;  // 其他值返回解码后的有符号值
}

// 函数：解码雷达状态
float decodeRadarStatus(uint16_t value) {
    if (value == 65535) {  // NaN
        return NAN_VALUE;
    }
    if (value == 0x81) {  // 1000 0001 表示有效
        return 1.0f;
    }
    return 0.0f;  // 0000 0000 表示无效
}

void decode_status(uint16_t data, uint8_t *ldzt, uint8_t *hwcj, uint8_t *dscj, uint8_t *jgcj) {
    // 提取D0-D7，每两个位为一组
    uint8_t D0_D2 = (data & 0x0007); // 取低3位 (D0-D2)
    uint8_t D3_D4 = (data >> 3) & 0x0003; // 取第4-5位 (D3-D4)
    uint8_t D5 = (data >> 5) & 0x0001; // 取第6位 (D5)
    uint8_t D6 = (data >> 6) & 0x0001; // 取第7位 (D6)
    uint8_t D7 = (data >> 7) & 0x0001; // 取第8位 (D7)

    *ldzt = D3_D4;//雷达状态
    *hwcj = D5;//HW测角数据有效性
    *dscj = D6;//DS测角数据有效性
    *jgcj = D7;//JG测距数据有效性
    // // 解码D0-D2 (GZ方式)
    // printf("D0-D2 (GZ方式): ");
    // switch (D0_D2) {
    //     case 1:
    //         printf("LD测角测距\n");
    //         break;
    //     case 2:
    //         printf("DS测角，LD测距\n");
    //         break;
    //     case 3:
    //         printf("HW测角，LD测距\n");
    //         break;
    //     case 4:
    //         printf("DS测角，JG测距\n");
    //         break;
    //     case 5:
    //         printf("HW测角，JG测距\n");
    //         break;
    //     case 6:
    //         printf("复合模式\n");
    //         break;
    //     default:
    //         printf("无效或未定义模式 (值: %u)\n", D0_D2);
    //         break;
    // }

    // // 解码D3-D4 (LD状态)
    // printf("D3-D4 (LD状态): ");
    // switch (D3_D4) {
    //     case 0:
    //         printf("LD未GZ好\n");
    //         break;
    //     case 1:
    //         printf("LDGZ好\n");
    //         break;
    //     case 2:
    //         printf("目标丢失\n");
    //         break;
    //     default:
    //         printf("无效状态 (值: %u)\n", D3_D4);
    //         break;
    // }

    // // 解码D5 (HW测角数据有效性)
    // printf("D5 (HW测角数据): %s\n", D5 ? "有效" : "无效");

    // // 解码D6 (DS测角数据有效性)
    // printf("D6 (DS测角数据): %s\n", D6 ? "有效" : "无效");

    // // 解码D7 (JG测距数据有效性)
    // printf("D7 (JG测距数据): %s\n", D7 ? "有效" : "无效");
}


//------------------------------------------------------------------------------------------------------------------------------------------

// ADDED: 全局变量存储上一行数据
static PrevData prev_data = { .is_first_line = 1 };

// 检查工作状态（前两位是否非零）
int check_status(char *bin) {
    return (bin[0] == '1' || bin[1] == '1') ? 1 : 0;
}

// 角度解码：16进制转10进制后 * 360 / 65536，若大于180则减360
double decode_angle(char *hex) {
    unsigned long dec = strtoul(hex, NULL, 16);
    double angle = dec * 360.0 / 65536.0;
    if (angle > 180.0) angle -= 360.0;
    return angle;
}


// 解码时间函数
float decodeTime(uint16_t yearMonth, uint16_t dayHour, uint16_t minuteSecond, TimeComponent component) {
    if (yearMonth == 65535 || dayHour == 65535 || minuteSecond == 65535) {  // NaN 表示
        return NAN_VALUE;
    }
    switch (component) {
        case TIME_YEAR:
            return (float)(yearMonth & 0xFF);         // 年：年、月字段的低 8 位
        case TIME_MONTH:
            return (float)((yearMonth >> 8) & 0xFF);  // 月：年、月字段的高 8 位
        case TIME_DAY:
            return (float)(dayHour & 0xFF);           // 日：日、小时字段的低 8 位
        case TIME_HOUR:
            return (float)((dayHour >> 8) & 0xFF);    // 小时：日、小时字段的高 8 位
        case TIME_MINUTE:
            return (float)(minuteSecond & 0xFF);       // 分：分、秒字段的低 8 位
        case TIME_SECOND:
            return (float)((minuteSecond >> 8) & 0xFF); // 秒：分、秒字段的高 8 位
        default:
            return NAN_VALUE;  // 无效组件
    }
}


// 解码一行数据并写入输出文件
void decode_line(char *line, FILE *output) {
    char *hex_data[11];
    char *token = strtok(line, " \t\n");
    int i = 0;
    while (token && i < 11) {
        hex_data[i++] = token;
        token = strtok(NULL, " \t\n");
    }
    if (i < 8) return; // 确保至少有8组数据

    // 解码激光径向距离
    char bin_laser[65] = "";
    hex_to_bin(hex_data[0], bin_laser);
    double laser_status = check_status(bin_laser);
    double laser_dist = bin_to_dec(bin_laser) * 1.875;

    // 解码雷达径向距离
    char bin_radar[65] = "";
    hex_to_bin(hex_data[3], bin_radar);
    int radar_status = check_status(bin_radar);
    double radar_dist = bin_to_dec(bin_radar) * 1.8735 + 70.0;

    // 解码角度数据
    double elec_azim = decode_angle(hex_data[1]);
    double elec_elev = decode_angle(hex_data[2]);
    double servo_azim = decode_angle(hex_data[4]);
    double servo_elev = decode_angle(hex_data[5]);
    double radar_azim = decode_angle(hex_data[6]);
    double radar_elev = decode_angle(hex_data[7]);

    // 雷达时间戳（假设为-2.00）
    double radar_time = -2.00;

    // //ADDED
    double photo_time = 0.00;
    double laser_time = 0.00;
    double servo_time = 0.00;
    double radar_status_fixed = 1.00;

    // // ADDED: 检查是否与上一行数据重复
    int photo_is_repeat = 0, laser_is_repeat = 0, radar_is_repeat = 0, servo_is_repeat = 0;


    if (!prev_data.is_first_line) {
        // if (photo_time == prev_data.photo_time &&
        //     laser_status == prev_data.photo_status &&
        //     elec_azim == prev_data.photo_azim &&
        //     elec_elev == prev_data.photo_elev) {
        //     photo_is_repeat = 1;
        // }
        
        if (laser_time == prev_data.laser_time &&
            laser_dist == prev_data.laser_dist) {
            laser_is_repeat = 1;
        }
        // if (radar_time == prev_data.radar_time &&
        //     radar_status_fixed == prev_data.radar_status &&
        //     radar_azim == prev_data.radar_azim &&
        //     radar_elev == prev_data.radar_elev &&
        //     radar_dist == prev_data.radar_dist) {
        //     radar_is_repeat = 1;
        // }
        // if (servo_time == prev_data.servo_time &&
        //     servo_azim == prev_data.servo_azim &&
        //     servo_elev == prev_data.servo_elev) {
        //     servo_is_repeat = 1;
        // }
    }

    // ADDED: 更新上一行数据
    // prev_data.photo_time = photo_time;
    // prev_data.photo_status = laser_status;
    // prev_data.photo_azim = elec_azim;
    // prev_data.photo_elev = elec_elev;
    prev_data.laser_time = laser_time;
    prev_data.laser_dist = laser_dist;
    // prev_data.radar_time = radar_time;
    // prev_data.radar_status = radar_status_fixed;
    // prev_data.radar_azim = radar_azim;
    // prev_data.radar_elev = radar_elev;
    // prev_data.radar_dist = radar_dist;
    // prev_data.servo_time = servo_time;
    // prev_data.servo_azim = servo_azim;
    // prev_data.servo_elev = servo_elev;
    prev_data.is_first_line = 0;

    // 将 NaN 存储为字符型变量（字符串）
    const char *nan_str = "NaN";

    // // // 如果光电组和伺服器组都重复，使用雷达数据填充（新增）
    // if (photo_is_repeat && servo_is_repeat) {
    //     photo_time = 0.0;
    //     laser_status = 1.0;
    //     elec_azim = 0.0;
    //     elec_elev = 0.0;
    //     servo_time = 0.0;
    //     servo_azim = radar_is_repeat ? 0.0 : radar_azim; // 如果雷达也重复，设为 0
    //     servo_elev = radar_is_repeat ? 0.0 : radar_elev; // 如果雷达也重复，设为 0
    //     photo_is_repeat = 0; // 重置标志，避免输出 NaN
    //     servo_is_repeat = 0;
    // }

    // // 如果激光组重复，使用雷达距离
    if (laser_is_repeat||laser_dist == 0.0 ){
        laser_time = 0.0;
        laser_dist = radar_is_repeat ? 0.0 : radar_dist; // 如果雷达也重复，设为 0
        laser_is_repeat = 0; // 重置标志，避免输出 NaN
    }

    // 写入输出文件
    fprintf(output, "%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%s\t%s\t%s\t%s\t%s\t%.5f\t%.5f\t%.5f\n",
            0.00, laser_status, elec_azim, elec_elev, // 光电时间戳、工作状态、光电方位角、光电俯仰角
            0.00, laser_dist,                   // 激光时间戳、激光径向距离
            radar_time, 1.00, radar_azim, radar_elev, radar_dist, // 雷达时间戳、工作状态、雷达方位角、雷达俯仰角、雷达径向距离
            nan_str,nan_str,nan_str,nan_str,nan_str,
            0.00, servo_azim, servo_elev);      // 伺服器时间戳、伺服器方位角、伺服器俯仰角
    
    // ADDED: 写入输出文件，处理重复数据
    // fprintf(output, "%s\t%s\t%s\t%s\t" // 光电组
    //             "%s\t%s\t"         // 激光组
    //             "%s\t%s\t%s\t%s\t%s\t" // 雷达组
    //             "%s\t%s\t%s\t%s\t%s\t" // NaN 组
    //             "%s\t%s\t%s\n",    // 伺服器组
    //     photo_is_repeat ? nan_str : "0.00000",
    //     photo_is_repeat ? nan_str : "1.00000",
    //     photo_is_repeat ? nan_str : ({ char buf[32] = {0}; sprintf(buf, "%.5f", elec_azim); buf; }),
    //     photo_is_repeat ? nan_str : ({ char buf[32] = {0}; sprintf(buf, "%.5f", elec_elev); buf; }),
    //     laser_is_repeat ? nan_str : "0.00000",
    //     laser_is_repeat ? nan_str : ({ char buf[32] = {0}; sprintf(buf, "%.5f", laser_dist); buf; }),
    //     radar_is_repeat ? nan_str : "-2.00000",
    //     radar_is_repeat ? nan_str : "1.00000",
    //     radar_is_repeat ? nan_str : ({ char buf[32] = {0}; sprintf(buf, "%.5f", radar_azim); buf; }),
    //     radar_is_repeat ? nan_str : ({ char buf[32] = {0}; sprintf(buf, "%.5f", radar_elev); buf; }),
    //     radar_is_repeat ? nan_str : ({ char buf[32] = {0}; sprintf(buf, "%.5f", radar_dist); buf; }),
    //     nan_str, nan_str, nan_str, nan_str, nan_str,
    //     servo_is_repeat ? nan_str : "0.00000",
    //     servo_is_repeat ? nan_str : ({ char buf[32] = {0}; sprintf(buf, "%.5f", servo_azim); buf; }),
    //     servo_is_repeat ? nan_str : ({ char buf[32] = {0}; sprintf(buf, "%.5f", servo_elev); buf; }));
}

/* smooth 函数：计算输入数组最后winsize个元素的均值 */
double smooth(double *x, int M, int winsize) {
    if (winsize >= M) {
        winsize = M;
    }
    double sum = 0.0;
    for (int i = M - winsize; i < M; i++) {
        sum += x[i];
    }
    return sum / winsize;
}