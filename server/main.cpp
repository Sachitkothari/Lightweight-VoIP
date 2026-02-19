#include <iostream>
#include <vector>
#include <cstring>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#pragma comment(lib, "ws2_32.lib")

#include "../packet.h"
#include "../opus.h"

#define PORT 8080
#define SAMPLE_RATE 48000
#define CHANNELS 1
#define FRAME_MS 40
#define FRAME_SAMPLES (SAMPLE_RATE * FRAME_MS / 1000)
#define MAX_OPUS_PACKET 4000

struct Client {
    sockaddr_in addr;
    OpusDecoder* decoder = nullptr;
    OpusEncoder* encoder = nullptr;

    // Last decoded PCM frame (for mixing / PLC)
    float pcm[FRAME_SAMPLES]{};
    bool hasPcm = false;

    // Pending Opus packet from recv loop
    unsigned char pendingData[MAX_OPUS_PACKET]{};
    int pendingSize = 0;
    bool hasPending = false;

    DWORD lastActiveMs = 0;
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

    // Make socket non-blocking
    u_long mode = 1;
    ioctlsocket(sockfd, FIONBIO, &mode);

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

    DWORD lastMixMs = GetTickCount();

    while (true) {
        // ---- Receive loop (non-blocking) ----
        while (true) {
            Packet p{};
            int n = recvfrom(sockfd, (char*)&p, sizeof(Packet), 0,
                             (sockaddr*)&cliaddr, &len);
            if (n <= 0)
                break; // no more data right now

            // Find or create client
            Client* sender = nullptr;
            for (auto& c : clients) {
                if (sameClient(c.addr, cliaddr)) {
                    sender = &c;
                    break;
                }
            }

            if (!sender) {
                Client c{};
                c.addr = cliaddr;

                int err = 0;
                c.decoder = opus_decoder_create(SAMPLE_RATE, CHANNELS, &err);
                if (err != OPUS_OK || !c.decoder) {
                    std::cerr << "opus_decoder_create failed: " << err << "\n";
                    continue;
                }

                c.encoder = opus_encoder_create(SAMPLE_RATE, CHANNELS,
                                                OPUS_APPLICATION_VOIP, &err);
                if (err != OPUS_OK || !c.encoder) {
                    std::cerr << "opus_encoder_create failed: " << err << "\n";
                    opus_decoder_destroy(c.decoder);
                    continue;
                }

                opus_encoder_ctl(c.encoder, OPUS_SET_BITRATE(32000));
                opus_encoder_ctl(c.encoder, OPUS_SET_COMPLEXITY(5));

                clients.push_back(c);
                sender = &clients.back();

                char ip[64];
                inet_ntop(AF_INET, &cliaddr.sin_addr, ip, sizeof(ip));
                std::cout << "New client: " << ip << ":" << ntohs(cliaddr.sin_port)
                          << " (senderId=" << p.senderId << ")\n";
            }

            // Store pending Opus packet for this client
            if (p.size > 0 && p.size <= MAX_OPUS_PACKET) {
                memcpy(sender->pendingData, p.data, p.size);
                sender->pendingSize = p.size;
                sender->hasPending = true;
                sender->lastActiveMs = GetTickCount();
            }
        }

        // ---- 40 ms mixing tick ----
        DWORD now = GetTickCount();
        if (now - lastMixMs < FRAME_MS)
            continue;
        lastMixMs = now;

        if (clients.empty())
            continue;

        // 1) Ensure each client has a PCM frame (decode or PLC)
        for (auto& c : clients) {
            if (c.decoder == nullptr)
                continue;

            if (c.hasPending) {
                int frameCount = opus_decode_float(
                    c.decoder,
                    c.pendingData,
                    c.pendingSize,
                    c.pcm,
                    FRAME_SAMPLES,
                    0
                );
                if (frameCount <= 0) {
                    // decode failed, generate silence
                    std::memset(c.pcm, 0, sizeof(c.pcm));
                }
                c.hasPending = false;
                c.hasPcm = true;
            } else if (c.hasPcm) {
                // No new packet this tick: use PLC to extend
                int frameCount = opus_decode_float(
                    c.decoder,
                    nullptr,
                    0,
                    c.pcm,
                    FRAME_SAMPLES,
                    0
                );
                if (frameCount <= 0) {
                    // PLC failed, keep last or silence
                    // here we just keep last c.pcm
                }
            } else {
                // No history yet: silence
                std::memset(c.pcm, 0, sizeof(c.pcm));
                c.hasPcm = true;
            }
        }

        // 2) For each destination client, mix all other clients and send
        for (auto& dst : clients) {
            if (dst.encoder == nullptr)
                continue;

            float mix[FRAME_SAMPLES]{};
            int contributors = 0;

            for (auto& src : clients) {
                if (&src == &dst)
                    continue;
                if (!src.hasPcm)
                    continue;

                for (int i = 0; i < FRAME_SAMPLES; i++)
                    mix[i] += src.pcm[i];

                contributors++;
            }

            if (contributors == 0)
                continue;

            // Normalize
            for (int i = 0; i < FRAME_SAMPLES; i++)
                mix[i] /= contributors;

            // Encode mixed frame
            unsigned char opusData[MAX_OPUS_PACKET];
            int bytes = opus_encode_float(
                dst.encoder,
                mix,
                FRAME_SAMPLES,
                opusData,
                MAX_OPUS_PACKET
            );
            if (bytes <= 0)
                continue;

            // Build packet (senderId here is "mixed", you can set 0 or any policy)
            Packet out{};
            out.senderId = 0; // mixed stream, not a single speaker
            out.sequence = 0;
            out.timestamp_ms = now;
            out.size = bytes;
            memcpy(out.data, opusData, bytes);

            int packetSize = sizeof(Packet) - MAX_PAYLOAD + bytes;

            sendto(sockfd,
                   (char*)&out,
                   packetSize,
                   0,
                   (sockaddr*)&dst.addr,
                   sizeof(dst.addr));
        }
    }

    // Cleanup (not really reached)
    for (auto& c : clients) {
        if (c.decoder) opus_decoder_destroy(c.decoder);
        if (c.encoder) opus_encoder_destroy(c.encoder);
    }

    closesocket(sockfd);
    WSACleanup();
    return 0;
}
