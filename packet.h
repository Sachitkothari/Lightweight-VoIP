#pragma once
#include <cstdint>

static const int MAX_PAYLOAD = 4000;

struct Packet {
    uint32_t senderId;        // NEW: unique ID per client
    uint32_t sequence;        // increasing counter
    uint32_t timestamp_ms;    // capture time in ms
    uint16_t size;            // opus payload size
    uint16_t reserved;        // padding / alignment
    uint8_t  data[MAX_PAYLOAD];
};
