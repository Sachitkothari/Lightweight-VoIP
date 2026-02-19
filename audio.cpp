#include <iostream>
#include <thread>
#include <mutex>
#include <queue>
#include <vector>
#include <atomic>
#include <cstring>
#include <chrono>
#include <fstream>
#include <string>
#include <random>

#include "portaudio.h"
#include "opus.h"
#include "packet.h"

#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

std::string CFG_SERVER_IP = "127.0.0.1";
int CFG_SERVER_PORT = 8080;

void loadConfig() {
    std::ifstream f("config.txt");
    if (!f.is_open()) {
        std::cerr << "config.txt not found. Using defaults.\n";
        return;
    }

    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind("SERVER_IP=", 0) == 0) {
            CFG_SERVER_IP = line.substr(strlen("SERVER_IP="));
        }
        else if (line.rfind("SERVER_PORT=", 0) == 0) {
            CFG_SERVER_PORT = std::stoi(line.substr(strlen("SERVER_PORT=")));
        }
    }
}

// ---- Config ----

static const int SAMPLE_RATE     = 48000;
static const int CHANNELS        = 1;
static const int FRAME_MS        = 40;
static const int FRAME_SAMPLES   = SAMPLE_RATE * FRAME_MS / 1000; // 1920
static const int MAX_OPUS_PACKET = 4000;

// ---- Globals ----
static OpusEncoder* g_encoder = nullptr;
static OpusDecoder* g_decoder = nullptr;

static SOCKET       g_sock = INVALID_SOCKET;
static sockaddr_in  g_serverAddr{};
static std::atomic<uint32_t> g_sequence{0};
static uint32_t     g_senderId = 0;   // NEW: unique ID for this client

static std::mutex g_queueMutex;
static std::queue<std::vector<float>> g_playQueue;
static std::atomic<bool> g_running{true};

// ---- Utility: current time in ms ----
uint32_t now_ms() {
    using namespace std::chrono;
    return (uint32_t)duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()
    ).count();
}

void applyCompression(float* samples, int count) {
    // 1. Compute RMS loudness
    float sum = 0.0f;
    for (int i = 0; i < count; i++)
        sum += samples[i] * samples[i];

    float rms = sqrtf(sum / count);

    // 2. Target RMS loudness (comfortable level)
    const float target = 0.1f;

    // 3. Compute gain factor
    float gain = 1.0f;
    if (rms > 0.0001f)
        gain = target / rms;

    // 4. Limit maximum gain (prevents over-amplifying noise)
    if (gain > 5.0f)
        gain = 5.0f;

    // 5. Apply gain
    for (int i = 0; i < count; i++)
        samples[i] *= gain;

    // 6. Hard limiter to prevent clipping
    for (int i = 0; i < count; i++) {
        if (samples[i] > 1.0f) samples[i] = 1.0f;
        if (samples[i] < -1.0f) samples[i] = -1.0f;
    }
}

// ---- UDP receive thread ----
void recvThreadFunc() {
    uint32_t expectedSeq = 0;
    bool firstPacket = true;

    while (g_running.load()) {
        Packet p{};
        sockaddr_in from{};
        int fromLen = sizeof(from);

        int n = recvfrom(g_sock, (char*)&p, sizeof(Packet), 0,
                         (sockaddr*)&from, &fromLen);
        if (n <= 0) continue;
        //std::cout << "Got packet from senderId=" << p.senderId << "\n";
        // NEW: drop any packet that originated from this client
        if (p.senderId == g_senderId) {
            continue;
        }

        float pcm[FRAME_SAMPLES];

        // First packet: accept sequence number
        if (firstPacket) {
            expectedSeq = p.sequence + 1;
            firstPacket = false;
        }

        // Detect packet loss
        if (p.sequence != expectedSeq) {
            int lost = (int)(p.sequence - expectedSeq);
            if (lost > 0 && lost < 50) {
                // Generate PLC frames for each lost packet
                for (int i = 0; i < lost; i++) {
                    opus_decode_float(
                        g_decoder,
                        NULL, 0,          // PLC mode
                        pcm,
                        FRAME_SAMPLES,
                        0
                    );

                    std::vector<float> frame(pcm, pcm + FRAME_SAMPLES);
                    {
                        std::lock_guard<std::mutex> lock(g_queueMutex);
                        g_playQueue.push(std::move(frame));
                    }
                }
            }
        }

        expectedSeq = p.sequence + 1;

        // Decode the received packet
        int frameCount = opus_decode_float(
            g_decoder,
            p.data,
            p.size,
            pcm,
            FRAME_SAMPLES,
            0
        );
        if (frameCount <= 0) continue;
        applyCompression(pcm, frameCount);
        std::vector<float> frame(pcm, pcm + frameCount);

        {
            std::lock_guard<std::mutex> lock(g_queueMutex);
            g_playQueue.push(std::move(frame));

            // Prevent lag buildup
            while (g_playQueue.size() > 20)
                g_playQueue.pop();
        }
    }
}

// ---- PortAudio callback ----
static int audioCallback(
    const void* inputBuffer,
    void* outputBuffer,
    unsigned long framesPerBuffer,
    const PaStreamCallbackTimeInfo* timeInfo,
    PaStreamCallbackFlags statusFlags,
    void* userData )
{
    const float* in  = (const float*)inputBuffer;
    float* out       = (float*)outputBuffer;

    // 1) Encode and send mic audio
    if (in) {
        unsigned char opusData[MAX_OPUS_PACKET];

        int bytes = opus_encode_float(
            g_encoder,
            in,
            (int)framesPerBuffer,
            opusData,
            MAX_OPUS_PACKET
        );

        if (bytes > 0) {
            Packet p{};
            p.senderId    = g_senderId;                 // NEW
            p.sequence    = g_sequence.fetch_add(1);
            p.timestamp_ms= now_ms();
            p.size        = (uint16_t)bytes;
            p.reserved    = 0;
            memcpy(p.data, opusData, bytes);
            int packetSize = sizeof(Packet) - MAX_PAYLOAD + p.size;
            sendto(g_sock,
                   (const char*)&p,
                   packetSize,
                   0,
                   (sockaddr*)&g_serverAddr,
                   sizeof(g_serverAddr));
        }
    }

    // 2) Adaptive jitter buffer playback
    static bool bufferPrimed = false;
    static int targetBuffer = 5; // adaptive target

    std::vector<float> frame;

    {
        std::lock_guard<std::mutex> lock(g_queueMutex);

        // Adaptive jitter: grow if needed
        if (!bufferPrimed) {
            if (g_playQueue.size() >= targetBuffer)
                bufferPrimed = true;
            else {
                for (unsigned long i = 0; i < framesPerBuffer; i++)
                    out[i] = 0.0f;
                return paContinue;
            }
        }

        // If queue is too small, increase buffer target
        if (g_playQueue.size() < 2 && targetBuffer < 10)
            targetBuffer++;

        // If queue is too large, shrink buffer target
        if (g_playQueue.size() > 8 && targetBuffer > 3)
            targetBuffer--;

        if (!g_playQueue.empty()) {
            frame = std::move(g_playQueue.front());
            g_playQueue.pop();
        }
    }

    if (!frame.empty()) {
        unsigned long toCopy = std::min<unsigned long>(framesPerBuffer, frame.size());
        memcpy(out, frame.data(), toCopy * sizeof(float));

        for (unsigned long i = toCopy; i < framesPerBuffer; ++i)
            out[i] = 0.0f;
    } else {
        // PLC fallback
        float plc[FRAME_SAMPLES];
        opus_decode_float(g_decoder, NULL, 0, plc, FRAME_SAMPLES, 0);

        unsigned long toCopy = std::min<unsigned long>(framesPerBuffer, (unsigned long)FRAME_SAMPLES);
        memcpy(out, plc, toCopy * sizeof(float));

        for (unsigned long i = toCopy; i < framesPerBuffer; ++i)
            out[i] = 0.0f;
    }

    return paContinue;
}

int main() {
    loadConfig();
    std::cout << "CLIENT CONNECTING TO " << CFG_SERVER_IP << ":" << CFG_SERVER_PORT << "\n";

    // ---- Generate unique senderId ----
    {
        std::random_device rd;
        std::mt19937 rng(rd());
        std::uniform_int_distribution<uint32_t> dist;
        g_senderId = dist(rng);
        std::cout << "Sender ID: " << g_senderId << "\n";
    }

    // ---- Winsock init ----
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2,2), &wsaData) != 0) {
        std::cerr << "WSAStartup failed\n";
        return 1;
    }

    g_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (g_sock == INVALID_SOCKET) {
        std::cerr << "socket() failed\n";
        WSACleanup();
        return 1;
    }

    memset(&g_serverAddr, 0, sizeof(g_serverAddr));
    g_serverAddr.sin_family = AF_INET;
    g_serverAddr.sin_port = htons(CFG_SERVER_PORT);
    if (InetPtonA(AF_INET, CFG_SERVER_IP.c_str(), &g_serverAddr.sin_addr) != 1) {
        std::cerr << "Invalid SERVER_IP in config.txt: " << CFG_SERVER_IP << "\n";
        return 1;
    }

    // ---- Opus init ----
    int opusErr = 0;
    g_encoder = opus_encoder_create(
        SAMPLE_RATE,
        CHANNELS,
        OPUS_APPLICATION_VOIP,
        &opusErr
    );
    if (opusErr != OPUS_OK) {
        std::cerr << "opus_encoder_create failed: " << opusErr << "\n";
        return 1;
    }

    // Opus tuning
    opus_encoder_ctl(g_encoder, OPUS_SET_BITRATE(64000));
    opus_encoder_ctl(g_encoder, OPUS_SET_COMPLEXITY(10));
    opus_encoder_ctl(g_encoder, OPUS_SET_INBAND_FEC(1));
    opus_encoder_ctl(g_encoder, OPUS_SET_DTX(1));
    opus_encoder_ctl(g_encoder, OPUS_SET_FORCE_CHANNELS(1));

    g_decoder = opus_decoder_create(
        SAMPLE_RATE,
        CHANNELS,
        &opusErr
    );
    if (opusErr != OPUS_OK) {
        std::cerr << "opus_decoder_create failed: " << opusErr << "\n";
        return 1;
    }

    // ---- PortAudio init ----
    PaError err = Pa_Initialize();
    if (err != paNoError) {
        std::cerr << "Pa_Initialize failed: " << Pa_GetErrorText(err) << "\n";
        return 1;
    }

    PaStream* stream;

    err = Pa_OpenDefaultStream(
        &stream,
        1,              // input channels
        1,              // output channels
        paFloat32,      // sample format
        SAMPLE_RATE,
        FRAME_SAMPLES,  // frames per buffer = 40ms
        audioCallback,
        nullptr
    );

    if (err != paNoError) {
        std::cerr << "Pa_OpenDefaultStream failed: " << Pa_GetErrorText(err) << "\n";
        Pa_Terminate();
        return 1;
    }

    // ---- Start receiver thread ----
    std::thread recvThread(recvThreadFunc);

    // ---- Start audio ----
    err = Pa_StartStream(stream);
    if (err != paNoError) {
        std::cerr << "Pa_StartStream failed: " << Pa_GetErrorText(err) << "\n";
        Pa_CloseStream(stream);
        Pa_Terminate();
        g_running = false;
        recvThread.join();
        return 1;
    }

    std::cout << "VoIP client running. Press Enter to exit.\n";
    std::cin.get();

    // ---- Shutdown ----
    g_running = false;
    closesocket(g_sock);
    recvThread.join();

    Pa_StopStream(stream);
    Pa_CloseStream(stream);
    Pa_Terminate();

    opus_encoder_destroy(g_encoder);
    opus_decoder_destroy(g_decoder);

    WSACleanup();

    return 0;
}