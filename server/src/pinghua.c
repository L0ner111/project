#include <stdio.h>
#include <stdlib.h>

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
Queue* initQueue() {
    Queue* queue = (Queue*)malloc(sizeof(Queue));
    if (queue == NULL) {
        printf("内存分配失败！\n");
        exit(1);
    }
    queue->front = NULL;
    queue->rear = NULL;
    queue->size = 0;
    return queue;
}

// 判断队列是否满
int isFull(Queue* queue) {
    return queue->size >= MAX_SIZE;
}

// 出队
float dequeue(Queue* queue) {
    if (queue->front == NULL) {
        printf("队列为空！\n");
        exit(1);
    }
    
    Node* temp = queue->front;
    float data = temp->data;
    queue->front = queue->front->next;
    // printf("出列：%f\n",data);
    if (queue->front == NULL) {  // 队列变为空
        queue->rear = NULL;
    }
    
    free(temp);
    queue->size--;
    return data;
}

// 入队
void enqueue(Queue* queue, float data) {
    if (isFull(queue)) {
        // printf("队列已满，移除队头元素以插入新元素\n");
        dequeue(queue); // 移除队头元素
    }
    
    Node* newNode = (Node*)malloc(sizeof(Node));
    if (newNode == NULL) {
        printf("内存分配失败！\n");
        exit(1);
    }
    newNode->data = data;
    newNode->next = NULL;
    
    if (queue->rear == NULL) {  // 队列为空
        queue->front = newNode;
        queue->rear = newNode;
    } else {
        queue->rear->next = newNode;
        queue->rear = newNode;
    }
    queue->size++;
    // printf("入列：%f\n",data);

}


// 判断队列是否为空
int isEmpty(Queue* queue) {
    return queue->front == NULL;
}

// 获取队列头元素
float peek(Queue* queue) {
    if (queue->front == NULL) {
        printf("队列为空！\n");
        exit(1);
    }
    return queue->front->data;
}

// 求队列中所有元素的平均值
float getAverage(Queue* queue) {
    if (isEmpty(queue)) {
        printf("队列为空，无法计算平均值！\n");
        return 0.0;
    }
    
    float sum = 0.0;
    int count = 0;
    Node* current = queue->front;
    
    while (current != NULL) {
        sum += current->data;
        count++;
        current = current->next;
    }
    // printf("avg：%f\n",sum / count);
    return sum / count;
}

// 销毁队列
void destroyQueue(Queue* queue) {
    while (!isEmpty(queue)) {
        dequeue(queue);
    }
    free(queue);
}