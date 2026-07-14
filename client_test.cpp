#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <unistd.h>

// 目标服务器地址：127.0.0.1 表示本机。
static const char *kServerIp = "127.0.0.1";
// 服务端监听的 UDP 端口，客户端需要发到这个端口。
static const uint16_t kServerPort = 7060;

int main() {
	// 创建一个 UDP socket。
	// AF_INET 表示 IPv4，SOCK_DGRAM 表示 UDP，不需要像 TCP 那样先建立连接。
	int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockfd < 0) {
		perror("socket");
		return 1;
	}

	// 组装服务端地址。
	// sockaddr_in 是 IPv4 的地址结构，里面包含 IP 和端口。
	sockaddr_in server_addr{};
	server_addr.sin_family = AF_INET;
	// 端口号要转成网络字节序。
	server_addr.sin_port = htons(kServerPort);
	// 把字符串形式的 IP 地址转换成二进制形式，填入结构体。
	if (inet_pton(AF_INET, kServerIp, &server_addr.sin_addr) != 1) {
		std::cerr << "invalid server ip\n";
		close(sockfd);
		return 1;
	}

	// 要发送的内容。这里不再模拟复杂数据，直接发一段文本。
	const char *message = "Hello Word";

	// sendto 会把这段数据作为一个 UDP 数据报发送到指定地址。
	// UDP 是“发包”模型，发送时不需要先连接。
	ssize_t sent = sendto(sockfd, message, std::strlen(message), 0,
						  reinterpret_cast<sockaddr *>(&server_addr), sizeof(server_addr));
	if (sent < 0) {
		perror("sendto");
		close(sockfd);
		return 1;
	}

	std::cout << "client sent " << sent << " bytes\n";

	// 这里演示双向通信：客户端在发完以后，再等服务端回一个 ACK。
	// recvfrom 会阻塞，直到收到一个 UDP 数据报。
	char reply[256] = {};
	sockaddr_in from_addr{};
	socklen_t from_len = sizeof(from_addr);
	ssize_t recv_len = recvfrom(sockfd, reply, sizeof(reply) - 1, 0,
								reinterpret_cast<sockaddr *>(&from_addr), &from_len);
	if (recv_len >= 0) {
		// UDP 收到的数据不是以 '\0' 结尾的字符串，手动补一个结束符。
		reply[recv_len] = '\0';
		std::cout << "client got reply: " << reply << "\n";
	}

	// 关闭 socket，释放系统资源。
	close(sockfd);
	return 0;
}
