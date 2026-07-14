#ifndef COORD_TRANS_H
#define COORD_TRANS_H
// 极坐标转笛卡尔坐标
void polar2Descartes(float alpha, float beta, float r, float *x, float *y, float *z);

// 笛卡尔坐标转极坐标
void Descartes2polar(float x, float y, float z, float *alpha, float *beta, float *r);
#endif
