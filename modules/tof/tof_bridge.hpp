#pragma once

#include <cstdint>

enum class ToFInitState : uint8_t
{
    Uninitialized,
    Ready,
    Failed,
};

struct ToFSnapshot
{
    uint16_t raw_mm;
    uint16_t filtered_mm;
    uint8_t range_status;
    bool valid;
    bool has_sample;
    bool fresh;
    ToFInitState init_state;
    int8_t last_error;
    uint32_t sample_sequence;
    uint64_t sample_timestamp_us;
    uint32_t sample_age_ms;
};

// Initializes and starts ranging. Safe to call more than once.
bool tof_setup();

// Non-blocking data-ready poll. Returns true only when a measurement was read.
bool tof_poll();

// Copies a coherent snapshot. Each reader owns its last_sequence value.
// fresh is true when snapshot.sample_sequence differs from *last_sequence.
// On return, *last_sequence is advanced to the copied sequence.
bool tof_get_snapshot(ToFSnapshot &snapshot, uint32_t *last_sequence = nullptr);
