#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#pragma comment(lib, "ws2_32.lib")

#include "../packet.h"
#include "../opus.h"

#define PORT 8080
#define SAMPLE_RATE 48000
#define CHANNELS 1
#define FRAME_MS 20
#define FRAME_SAMPLES (SAMPLE_RATE * FRAME_MS / 1000)
#define MAX_OPUS_PACKET 4000

struct Client {
    sockaddr_in addr{};
    OpusDecoder* decoder = nullptr;
    OpusEncoder* encoder = nullptr;

    float pcm[FRAME_SAMPLES]{};   // last decoded frame
    bool hasPcm = false;

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

    // Non-blocking socket
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
                break;

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

                // Opus tuning for clarity
                opus_encoder_ctl(c.encoder, OPUS_SET_BITRATE(48000));          // 48 kbps
                opus_encoder_ctl(c.encoder, OPUS_SET_VBR(1));
                opus_encoder_ctl(c.encoder, OPUS_SET_COMPLEXITY(10));
                opus_encoder_ctl(c.encoder, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
                opus_encoder_ctl(c.encoder, OPUS_SET_DTX(0));

                clients.push_back(c);
                sender = &clients.back();

                char ip[64];
                inet_ntop(AF_INET, &cliaddr.sin_addr, ip, sizeof(ip));
                std::cout << "New client: " << ip << ":" << ntohs(cliaddr.sin_port)
                          << " (senderId=" << p.senderId << ")\n";
            }

            if (p.size > 0 && p.size <= MAX_OPUS_PACKET) {
                memcpy(sender->pendingData, p.data, p.size);
                sender->pendingSize = p.size;
                sender->hasPending = true;
                sender->lastActiveMs = GetTickCount();
            }
        }

        // ---- 20 ms mixing tick ----
        DWORD now = GetTickCount();
        if (now - lastMixMs < FRAME_MS)
            continue;
        lastMixMs = now;

        if (clients.empty())
            continue;

        // 1) Ensure each client has a PCM frame (decode or PLC/silence)
        for (auto& c : clients) {
            if (!c.decoder)
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
                    std::memset(c.pcm, 0, sizeof(c.pcm));
                }
                c.hasPending = false;
                c.hasPcm = true;
            } else if (c.hasPcm) {
                // If recently active, use PLC; otherwise silence
                if (now - c.lastActiveMs < 200) {
                    int frameCount = opus_decode_float(
                        c.decoder,
                        nullptr,
                        0,
                        c.pcm,
                        FRAME_SAMPLES,
                        0
                    );
                    if (frameCount <= 0) {
                        // keep last or silence; here we keep last
                    }
                } else {
                    std::memset(c.pcm, 0, sizeof(c.pcm));
                }
            } else {
                std::memset(c.pcm, 0, sizeof(c.pcm));
                c.hasPcm = true;
            }
        }

        // 2) For each destination client, mix others and send
        for (auto& dst : clients) {
            if (!dst.encoder)
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

            // Soft normalization: scale by 1/sqrt(N) instead of 1/N
            float scale = 1.0f / std::sqrt((float)contributors);
            for (int i = 0; i < FRAME_SAMPLES; i++)
                mix[i] *= scale;

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

            Packet out{};
            out.senderId = 0; // mixed stream
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

    for (auto& c : clients) {
        if (c.decoder) opus_decoder_destroy(c.decoder);
        if (c.encoder) opus_encoder_destroy(c.encoder);
    }

    closesocket(sockfd);
    WSACleanup();
    return 0;
}