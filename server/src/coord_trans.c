// #include <stdio.h>
// #include <math.h>
#include "constant.h"

// #define PI 3.1415926535    // 定义π
// #define NAN_VALUE -9999.0  // 定义NaN的替代值


// 极坐标转笛卡尔坐标
void polar2Descartes(float alpha, float beta, float r, float *x, float *y, float *z) {
    *x = r * cos(beta) * sin(alpha);
    *y = r * cos(beta) * cos(alpha);
    *z = r * sin(beta);
}

// 笛卡尔坐标转极坐标
void Descartes2polar(float x, float y, float z, float *alpha, float *beta, float *r) {
    *r = sqrt(x * x + y * y + z * z);
    if (*r == 0) {
        *alpha = 0.0;
        *beta = 0.0;
        return;
    }
    *beta = asin(z / *r);
    float theta = atan2(y, x);
    if (x == 0 && y == 0) {
        theta = PI / 2.0;
    }
    *alpha = fmod(PI / 2.0 - theta, 2 * PI);
}
