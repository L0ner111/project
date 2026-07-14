# include "config.h"

// #define PI 3.1415926535    // 定义π
// #define NAN_VALUE -9999.0  // 定义NaN的替代值
// 将角度转换为弧度
double degreesToRadians(double degrees) {
    return degrees * PI / 180.0;
}

//处理时间步
double processTimestamp(double value) {
    return (value != NAN_VALUE) ? value / 1000.00 : NAN_VALUE;
}

//处理角度
double processAngle(double value) {
    return (value != NAN_VALUE) ? degreesToRadians(value) : NAN_VALUE;
}

