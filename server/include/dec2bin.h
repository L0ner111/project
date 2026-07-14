#include <stdint.h>
#ifndef DEC2BIN_H
#define DEC2BIN_H
// 函数：将16位整数转换为二进制字符串
void printBinary16(uint16_t num);

// 检查并处理NaN的函数，返回16位值
uint16_t processValue(double value, int isAngle);

// 处理雷达状态的函数
uint16_t processRadarStatus(double value);

// 处理径向距离的函数（除以1.8735）
uint16_t processDistance(double value);

// 【新增】函数：将16位整数追加到二进制字符串缓冲区
void appendBinary16(uint16_t num, char *buffer, int *offset);

// 16进制字符串转2进制字符串
void hex_to_bin(char *hex, char *bin);

// 2进制字符串转10进制（忽略前两位）
unsigned long bin_to_dec(char *bin);
#endif