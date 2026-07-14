#include <arpa/inet.h>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <unistd.h>

// 服务端监听的 UDP 端口，客户端必须发到这个端口才能收到。
static const uint16_t kListenPort = 7060;

int main() {
	// 创建 UDP socket。
	int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockfd < 0) {
		perror("socket");
		return 1;
	}

	// 绑定本地地址和端口。
	// INADDR_ANY 表示监听本机所有网卡上的 7060 端口。
	sockaddr_in server_addr{};
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	// 端口号也必须转成网络字节序。
	server_addr.sin_port = htons(kListenPort);

	// bind 的作用是把 socket 和本地端口绑定起来。
	// 绑定成功后，内核会把发到 7060 的 UDP 包交给这个进程。
	if (bind(sockfd, reinterpret_cast<sockaddr *>(&server_addr), sizeof(server_addr)) < 0) {
		perror("bind");
		close(sockfd);
		return 1;
	}

	// 服务器进入常驻监听状态。
	std::cout << "server listening on port " << kListenPort << "\n";

	while (true) {
		// 接收缓冲区，用来存放客户端发来的文本。
		char buffer[1024] = {};
		// 记录客户端地址，这样服务端才能把 ACK 回发给原发送者。
		sockaddr_in client_addr{};
		socklen_t client_len = sizeof(client_addr);

		// recvfrom 会阻塞等待数据。
		// 收到后，buffer 里就是客户端发来的原始字节。
		ssize_t recv_len = recvfrom(sockfd, buffer, sizeof(buffer), 0,
									reinterpret_cast<sockaddr *>(&client_addr), &client_len);
		if (recv_len < 0) {
			perror("recvfrom");
			continue;
		}

		// 把客户端 IP 从二进制形式转换回字符串，方便打印。
		char client_ip[INET_ADDRSTRLEN] = {};
		inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip));
		std::cout << "server got " << recv_len << " bytes from "
				  << client_ip << ":" << ntohs(client_addr.sin_port) << "\n";
		// UDP 收到的内容不一定自动带字符串结尾符，所以这里手动补 '\0'。
		buffer[recv_len] = '\0';
		// 直接打印收到的文本。
		std::cout << "payload text: " << buffer << "\n";

		// 这里演示“回显”：服务端把一段确认信息发回给客户端。
		// 这也是 UDP 的典型用法之一：收包后根据源地址直接 sendto 回去。
		const char *ack = "ACK: Hello Word received";
		if (sendto(sockfd, ack, std::strlen(ack), 0,
				   reinterpret_cast<sockaddr *>(&client_addr), client_len) < 0) {
			perror("sendto");
		}
	}

	// 正常情况下这里不会走到，因为上面是无限循环。
	close(sockfd);
	return 0;
}
