#include <stdint.h>
#ifndef JIEMA_H
#define JIEMA_H
// 函数：从二进制字符串中提取16位数据
uint16_t binaryToUint16(const char *binary, int offset);

// 函数：将16位数据解码为浮点数
float decodeValue(uint16_t value, int isAngle, int isDistance, int isTimestamp,int flag);

// 函数：解码雷达状态
float decodeRadarStatus(uint16_t value);

//解码状态字1
void decode_status(uint16_t data, uint8_t *ldzt, uint8_t *hwcj, uint8_t *dscj, uint8_t *jgcj) ;

// 检查工作状态（前两位是否非零）
int check_status(char *bin);

// 角度解码：16进制转10进制后 * 360 / 65536，若大于180则减360
double decode_angle(char *hex) ;

// 解码一行数据并写入输出文件
void decode_line(char *line, FILE *output);

float decodeTime(uint16_t yearMonth, uint16_t dayHour, uint16_t minuteSecond, TimeComponent component);

double smooth(double *x, int M, int winsize);

#endif