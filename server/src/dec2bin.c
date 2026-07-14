#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// 函数：将16位整数转换为二进制字符串
void printBinary16(uint16_t num) {
    for (int i = 15; i >= 0; i--) {
        printf("%d", (num >> i) & 1);
    }
    printf("\n");
}

// 检查并处理NaN的函数，返回16位值
uint16_t processValue(double value, int isAngle) {
    if (isnan(value)) {
        return 65535;  // 空数据用65535表示
    }


    // if (isAngle) {
    //     int result = (int)(value * 65536 / 360);
    //     return (uint16_t)(result & 0xFFFF);  // 限制为16位
    // }

    if (isAngle) {
        if (value >= 0) {
            // 正角度 [0°, 180°] 映射到 [0, 32768]
            if (value > 180.0) {
                value = 180.0; // 限制最大值为 180°
            }
            int result = (int)(value * 32768 / 180.0);
            return (uint16_t)(result & 0xFFFF); // 确保 16 位
        } else {
            // 负角度 [0°, -180°] 映射到 [65535, 32768]
            if (value < -180.0) {
                value = -180.0; // 限制最小值为 -180°
            }
            int result = (int)(65535.0 - (-value) * 32768 / 180.0);
            return (uint16_t)(result & 0xFFFF); // 确保 16 位
        }
    }


    int int_value = (int)value;
    if (int_value < -32768 || int_value > 32767) {
        return 0;  // 超出范围的值映射到 0
    }
    return (uint16_t)(int_value + 32768);  // 映射 [-32768, 32767] 到 [0, 65534]
}

// 处理雷达状态的函数
uint16_t processRadarStatus(double value) {
    if (isnan(value)) {
        return 65535;  // 空数据用65535表示
    }
    int status = (int)value;
    if (status == 1) {
        return 0x81;  // 1转为1000 0001 (129)
    }
    return 0;  // 0保持为0000 0000
}

// 处理径向距离的函数（除以1.8735）
uint16_t processDistance(double value) {
    if (isnan(value)) {
        return 65535;  // 空数据用65535表示
    }
    int result = (int)(value / 1.8735);
    return (uint16_t)(result & 0xFFFF);  // 限制为16位
}

// 【新增】函数：将16位整数追加到二进制字符串缓冲区
void appendBinary16(uint16_t num, char *buffer, int *offset) {
    for (int i = 15; i >= 0; i--) {
        buffer[*offset] = ((num >> i) & 1) ? '1' : '0';
        (*offset)++;
    }
}

// 16进制字符串转2进制字符串
void hex_to_bin(char *hex, char *bin) {
    for (int i = 0; hex[i]; i++) {
        switch (hex[i]) {
            case '0': strcat(bin, "0000"); break;
            case '1': strcat(bin, "0001"); break;
            case '2': strcat(bin, "0010"); break;
            case '3': strcat(bin, "0011"); break;
            case '4': strcat(bin, "0100"); break;
            case '5': strcat(bin, "0101"); break;
            case '6': strcat(bin, "0110"); break;
            case '7': strcat(bin, "0111"); break;
            case '8': strcat(bin, "1000"); break;
            case '9': strcat(bin, "1001"); break;
            case 'a': case 'A': strcat(bin, "1010"); break;
            case 'b': case 'B': strcat(bin, "1011"); break;
            case 'c': case 'C': strcat(bin, "1100"); break;
            case 'd': case 'D': strcat(bin, "1101"); break;
            case 'e': case 'E': strcat(bin, "1110"); break;
            case 'f': case 'F': strcat(bin, "1111"); break;
        }
    }
}

// 2进制字符串转10进制（忽略前两位）
unsigned long bin_to_dec(char *bin) {
    unsigned long dec = 0;
    for (int i = 2; i < strlen(bin); i++) {
        dec = dec * 2 + (bin[i] - '0');
    }
    return dec;
}