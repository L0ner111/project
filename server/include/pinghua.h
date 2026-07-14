#include <stdio.h>
#ifndef PINGHUA_H
#define PINGHUA_H

#define MAX_SIZE 70 // 队列最大长度

// 定义队列节点结构
typedef struct Node {
    float data;            // 存储数据（float类型）
    struct Node* next;     // 指向下一个节点的指针
} Node;

// 定义队列结构
typedef struct Queue {
    Node* front;           // 头指针
    Node* rear;            // 尾指针
    int size;              // 当前队列元素个数
} Queue;

// 初始化队列
Queue* initQueue();

// 判断队列是否满
int isFull(Queue* queue);

// 入队
void enqueue(Queue* queue, float data);

// 出队
float dequeue(Queue* queue);

// 判断队列是否为空
int isEmpty(Queue* queue);

// 获取队列头元素
float peek(Queue* queue);

// 求队列中所有元素的平均值
float getAverage(Queue* queue);

// 销毁队列
void destroyQueue(Queue* queue);
#endif