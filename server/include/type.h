#ifndef TYPE_H
#define TYPE_H
#define M_PI 3.14159265358979323846

typedef struct
{
    double time;
    double x;
    double y;
    double z;
} CartesianCoord;

typedef struct
{
    double time;
    double distance;
    double azimuth;  // 方位角
    double elevation;  // 俯仰角
} PolarCoord;

#endif