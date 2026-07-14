#include "constant.h"
// #define BUFFER_SIZE 1024
// 修改后的 parseXMLFile 函数
void parseXMLFile(const char* filename, int* id, double times[2], double* freqs) {
    FILE *file;
    char buffer[BUFFER_SIZE];
    
    file = fopen(filename, "r");
    if (file == NULL) {
        printf("无法打开文件 %s\n", filename);
        return;
    }
    
    while (fgets(buffer, BUFFER_SIZE, file) != NULL) {
        remove_newline(buffer); // 移除换行符
        
        // 解析 <id>
        if (my_strstr(buffer, "<id>") != NULL) {
            char* start = my_strstr(buffer, "<id>") + 4;
            char* end = my_strstr(buffer, "</id>");
            char value[20];
            my_strncpy(value, start, end - start);
            value[end - start] = '\0';
            *id = atoi(value); // 转换为整数
        }
        
        // 解析 <startime>
        if (my_strstr(buffer, "<startime>") != NULL) {
            char* start = my_strstr(buffer, "<startime>") + 10;
            char* end = my_strstr(buffer, "</startime>");
            char value[20];
            my_strncpy(value, start, end - start);
            value[end - start] = '\0';
            times[0] = atof(value); // 存储到 times[0]
        }
        
        // 解析 <endtime>
        if (my_strstr(buffer, "<endtime>") != NULL) {
            char* start = my_strstr(buffer, "<endtime>") + 9;
            char* end = my_strstr(buffer, "</endtime>");
            char value[20];
            my_strncpy(value, start, end - start);
            value[end - start] = '\0';
            times[1] = atof(value); // 存储到 times[1]
        }
        
        // 解析 <interferfreq>
        if (my_strstr(buffer, "<interferfreq>") != NULL) {
            char* start = my_strstr(buffer, "<interferfreq>") + 14;
            char* end = my_strstr(buffer, "</interferfreq>");
            char value[20];
            my_strncpy(value, start, end - start);
            value[end - start] = '\0';
            *freqs = atof(value); // 存储到 freqs
        }
    }
    
    fclose(file);
}

void printResults(int id, double times[2], double freqs) {
    printf("解析结果:\n");
    printf("interfer%d:\n", id); // 使用解析得到的 id
    printf("startime: %.2f\n", times[0]);
    printf("endtime: %.2f\n", times[1]);
    printf("interferfreq: %.2f\n", freqs);
}