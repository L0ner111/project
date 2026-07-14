// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <unistd.h>
// #include <sys/socket.h>
// #include <netinet/in.h>
// #include <arpa/inet.h>

// #define PORT 7060
// #define BUFFER_SIZE 1024  // 接收的数据缓冲区大小
// #define SERVER_IP "127.0.0.1"

// // 函数：将十六进制字符串转换为字节流（uint8_t 数组）
// void hex_to_bytes(const char *hex_str, uint8_t *byte_buffer) {
//     size_t len = strlen(hex_str);
//     for (size_t i = 0; i < len; i += 2) {
//         sscanf(hex_str + i, "%2hhx", &byte_buffer[i / 2]);
//     }
// }

// int main() {
//     int sockfd;
//     struct sockaddr_in serv_addr;
//     char buffer[BUFFER_SIZE];
//     FILE *file;

//     // 创建UDP套接字
//     sockfd = socket(AF_INET, SOCK_DGRAM, 0);
//     if (sockfd < 0) {
//         perror("ERROR opening socket");
//         exit(1);
//     }

//     // 配置服务器地址
//     memset(&serv_addr, 0, sizeof(serv_addr));
//     serv_addr.sin_family = AF_INET;
//     serv_addr.sin_port = htons(PORT);
//     if (inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr) <= 0) {
//         perror("ERROR invalid server address");
//         exit(1);
//     }

//     // 打开数据文件
//     file = fopen("./udp_data_no_spaces.txt", "r");
//     if (file == NULL) {
//         perror("ERROR opening file");
//         exit(1);
//     }

//     printf("Client starting, sending binary data to server...\n");

//     // 逐行读取文件并发送数据
//     while (fgets(buffer, sizeof(buffer), file) != NULL) {
//         // 去除换行符
//         buffer[strcspn(buffer, "\n")] = '\0';  // 去除换行符
        
//         printf("Original Hex String: %s\n", buffer);

//         // 准备一个字节流缓冲区
//         uint8_t byte_buffer[BUFFER_SIZE / 2];  // 十六进制每两个字符表示一个字节

//         // 将十六进制字符串转换为字节流
//         hex_to_bytes(buffer, byte_buffer);

//         // 发送字节流数据到服务器
//         if (sendto(sockfd, byte_buffer, strlen(buffer) / 2, 0,
//                    (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
//             perror("ERROR in sendto");
//             continue;
//         }
//         printf("Sent data to server, length is: %ld bytes\n", strlen(buffer) / 2);

//         usleep(2000000);  // 2秒的间隔
//     }

//     fclose(file);
//     close(sockfd);
//     printf("Client finished.\n");
//     return 0;
// }



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define PORT 7060
#define BUFFER_SIZE 1024  // 接收的数据缓冲区大小
#define SERVER_IP "127.0.0.1"

// 函数：将十六进制字符串转换为字节流（uint8_t 数组），并考虑字节序
void hex_to_bytes(const char *hex_str, uint8_t *byte_buffer, size_t *byte_len) {
    size_t len = strlen(hex_str);
    *byte_len = len / 2;  // 每两个十六进制字符表示一个字节

    for (size_t i = 0; i < len; i += 2) {
        unsigned int byte;
        sscanf(hex_str + i, "%2x", &byte);
        byte_buffer[i / 2] = (uint8_t)byte;
    }
}

// 函数：将字节流按大端序重新排列（假设数据为 16 位整数序列）
void to_network_byte_order(uint8_t *byte_buffer, size_t byte_len, uint8_t *output_buffer) {
    // 假设数据为 16 位整数序列
    for (size_t i = 0; i < byte_len; i += 2) {
        if (i + 1 < byte_len) { // 确保有足够的字节组成 16 位整数
            uint16_t value = (byte_buffer[i] << 8) | byte_buffer[i + 1]; // 本地字节序
            value = htons(value); // 转换为网络字节序（大端）
            output_buffer[i] = (value >> 8) & 0xFF; // 高字节
            output_buffer[i + 1] = value & 0xFF;    // 低字节
        } else {
            // 如果剩余字节不足 16 位，复制剩余字节
            output_buffer[i] = byte_buffer[i];
        }
    }
}

int main() {
    int sockfd;
    struct sockaddr_in serv_addr;
    char buffer[BUFFER_SIZE];
    FILE *file;

    // 创建 UDP 套接字
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("ERROR opening socket");
        exit(1);
    }

    // 配置服务器地址
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT); // 端口号已转换为网络字节序
    if (inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr) <= 0) {
        perror("ERROR invalid server address");
        exit(1);
    }

    // 打开数据文件
    // file = fopen("./udp_data_no_spaces.txt", "r");
    // file = fopen("../0927实测数据/数据4/udp_data_no_spaces.txt", "r");
    // file = fopen("/home/zgx/桌面/1109实测数据/data1/udp_data_no_spaces.txt", "r");
    file = fopen("../dataset/1109实测数据/data1/udp_data_no_spaces.txt", "r");

    if (file == NULL) {
        perror("ERROR opening file");
        exit(1);
    }

    printf("客户端启动，向服务器发送二进制数据...\n");
    // int cnt = 0;

    // 逐行读取文件并发送数据
    while (fgets(buffer, sizeof(buffer), file) != NULL) {
        // 去除换行符
        buffer[strcspn(buffer, "\n")] = '\0';
        
        printf("原始十六进制字符串: %s\n", buffer);

        // 准备字节流缓冲区
        uint8_t byte_buffer[BUFFER_SIZE / 2];
        uint8_t network_buffer[BUFFER_SIZE / 2];
        size_t byte_len;

        // 将十六进制字符串转换为字节流
        hex_to_bytes(buffer, byte_buffer, &byte_len);

        // 转换为网络字节序（假设数据为 16 位整数序列）
        to_network_byte_order(byte_buffer, byte_len, network_buffer);

        // 发送字节流数据到服务器
        if (sendto(sockfd, network_buffer, byte_len, 0,
                   (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
            perror("ERROR in sendto");
            continue;
        }
        printf("已发送数据到服务器，长度: %ld 字节\n", byte_len);
        // cnt ++;
        // printf("cnt = %d\n", cnt);

        usleep(200000);  // 2秒间隔
    }

    fclose(file);
    close(sockfd);
    printf("客户端结束。\n");
    return 0;
}
