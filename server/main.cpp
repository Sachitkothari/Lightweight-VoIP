#include <iostream>
#include <vector>
#include <string>
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

#include "../packet.h"   // MUST match client version exactly

#define PORT 8080

struct Client {
    sockaddr_in addr;
};

bool sameClient(const sockaddr_in& a, const sockaddr_in& b) {
    return a.sin_family == b.sin_family &&
           a.sin_port   == b.sin_port   &&
           a.sin_addr.s_addr == b.sin_addr.s_addr;
}

int main() {
    // ---- Winsock init ----
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2,2), &wsa) != 0) {
        std::cerr << "WSAStartup failed\n";
        return 1;
    }

    SOCKET sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == INVALID_SOCKET) {
        std::cerr << "socket creation failed\n";
        WSACleanup();
        return 1;
    }

    sockaddr_in servaddr{}, cliaddr{};
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);

    if (bind(sockfd, (sockaddr*)&servaddr, sizeof(servaddr)) == SOCKET_ERROR) {
        std::cerr << "bind failed: " << WSAGetLastError() << "\n";
        closesocket(sockfd);
        WSACleanup();
        return 1;
    }

    std::vector<Client> clients;
    int len = sizeof(cliaddr);

    std::cout << "Server listening on port " << PORT << "\n";

    while (true) {
        Packet p{};
        int n = recvfrom(sockfd, (char*)&p, sizeof(Packet), 0,
                         (sockaddr*)&cliaddr, &len);

        if (n <= 0)
            continue;

        // ---- Register new client ----
        bool known = false;
        for (auto& c : clients) {
            if (sameClient(c.addr, cliaddr)) {
                known = true;
                break;
            }
        }

        if (!known) {
            Client c{};
            c.addr = cliaddr;
            clients.push_back(c);

            char ip[64];
            inet_ntop(AF_INET, &cliaddr.sin_addr, ip, sizeof(ip));
            std::cout << "New client: " << ip << ":" << ntohs(cliaddr.sin_port)
                      << " (senderId=" << p.senderId << ")\n";
        }

        // ---- Broadcast to all OTHER clients ----
        for (auto& c : clients) {
            if (!sameClient(c.addr, cliaddr)) {
                int packetSize = sizeof(Packet) - MAX_PAYLOAD + p.size;
                sendto(sockfd, (char*)&p, packetSize, 0,
                       (sockaddr*)&c.addr, sizeof(c.addr));
            }
        }
    }

    closesocket(sockfd);
    WSACleanup();
    return 0;
}
