#include "tof_bridge.hpp"

#include <climits>

#include "pico/stdlib.h"
#include "pico/critical_section.h"
#include "tof_vl53l1x.hpp"

namespace
{
ToF_VL53L1X sensor(0x29);
ToFSnapshot shared_snapshot = {
    0, 0, 255, false, false, false,
    ToFInitState::Uninitialized, 0, 0, 0, UINT32_MAX
};
critical_section_t snapshot_section;

void publish(const ToFSnapshot &snapshot)
{
    critical_section_enter_blocking(&snapshot_section);
    shared_snapshot = snapshot;
    critical_section_exit(&snapshot_section);
}
} // namespace

bool tof_setup()
{
    if (!critical_section_is_initialized(&snapshot_section))
    {
        critical_section_init(&snapshot_section);
    }

    ToFSnapshot next;
    tof_get_snapshot(next);
    if (next.init_state == ToFInitState::Ready) return true;

    bool ok = sensor.init();
    if (ok) ok = sensor.configure();
    if (ok) ok = sensor.start();

    next.init_state = ok ? ToFInitState::Ready : ToFInitState::Failed;
    next.last_error = sensor.last_error();
    next.valid = false;
    publish(next);
    return ok;
}

bool tof_poll()
{
    ToFSnapshot current;
    if (!tof_get_snapshot(current) || current.init_state != ToFInitState::Ready)
    {
        return false;
    }

    if (!sensor.poll())
    {
        if (sensor.last_error() != 0)
        {
            current.last_error = sensor.last_error();
            publish(current);
        }
        return false;
    }

    current.raw_mm = sensor.raw_mm();
    current.filtered_mm = sensor.filtered_mm();
    current.range_status = sensor.range_status();
    current.valid = sensor.valid();
    current.has_sample = true;
    current.last_error = sensor.last_error();
    current.sample_sequence++;
    current.sample_timestamp_us = time_us_64();
    current.sample_age_ms = 0;
    publish(current);
    return true;
}

bool tof_get_snapshot(ToFSnapshot &snapshot, uint32_t *last_sequence)
{
    critical_section_enter_blocking(&snapshot_section);
    snapshot = shared_snapshot;
    critical_section_exit(&snapshot_section);

    snapshot.fresh = last_sequence != nullptr
        && snapshot.has_sample
        && snapshot.sample_sequence != *last_sequence;
    if (last_sequence != nullptr) *last_sequence = snapshot.sample_sequence;

    if (snapshot.sample_timestamp_us == 0)
    {
        snapshot.sample_age_ms = UINT32_MAX;
    }
    else
    {
        const uint64_t age_ms = (time_us_64() - snapshot.sample_timestamp_us) / 1000ULL;
        snapshot.sample_age_ms = age_ms > UINT32_MAX
            ? UINT32_MAX
            : static_cast<uint32_t>(age_ms);
    }
    return snapshot.init_state == ToFInitState::Ready;
}
