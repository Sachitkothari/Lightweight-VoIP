#include <bits/stdc++.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "../packet.h"

#define PORT 8080
#define MAXLINE 1024

struct Client {
    sockaddr_in addr;
};

bool sameClient(const sockaddr_in& a, const sockaddr_in& b) {
    return a.sin_family == b.sin_family &&
           a.sin_port   == b.sin_port   &&
           a.sin_addr.s_addr == b.sin_addr.s_addr;
}

int main() {
    int sockfd;
    struct sockaddr_in servaddr, cliaddr;

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    servaddr.sin_family      = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port        = htons(PORT);

    if (bind(sockfd, (const struct sockaddr*)&servaddr,
             sizeof(servaddr)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    std::vector<Client> clients;
    socklen_t len = sizeof(cliaddr);

    std::cout << "Server listening on port " << PORT << "\n";

    while (true) {
        Packet p{};
        int n = recvfrom(sockfd, &p, sizeof(Packet), 0,
                         (struct sockaddr*)&cliaddr, &len);
        if (n <= 0) continue;

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
            std::cout << "New client: " << ip << ":" << ntohs(cliaddr.sin_port) << "\n";
        }

        // broadcast to all other clients
        for (auto& c : clients) {
            if (!sameClient(c.addr, cliaddr)) {
                sendto(sockfd, &p, sizeof(Packet), 0,
                       (struct sockaddr*)&c.addr, sizeof(c.addr));
            }
        }
    }

    close(sockfd);
    return 0;
}
