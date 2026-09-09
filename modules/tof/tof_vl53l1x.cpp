#include "tof_vl53l1x.hpp"

#include <algorithm>

#include "pico/stdlib.h"
#include "vl53l1_error_codes.h"

ToF_VL53L1X::ToF_VL53L1X(uint16_t dev_addr) : dev_(dev_addr) {}

bool ToF_VL53L1X::init(uint32_t boot_timeout_ms)
{
    i2c_init(TOF_I2C_PORT, TOF_I2C_CLOCK_HZ);
    gpio_set_function(TOF_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(TOF_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(TOF_SDA_GPIO);
    gpio_pull_up(TOF_SCL_GPIO);

    const uint64_t deadline_us = time_us_64() + boot_timeout_ms * 1000ULL;
    uint8_t booted = 0;
    do
    {
        last_error_ = VL53L1X_BootState(dev_, &booted);
        if (last_error_ != 0)
        {
            return false;
        }
        if ((booted & 1U) != 0U)
        {
            break;
        }
        sleep_ms(10);
    } while (time_us_64() < deadline_us);

    if ((booted & 1U) == 0U)
    {
        last_error_ = VL53L1_ERROR_TIME_OUT;
        return false;
    }

    last_error_ = VL53L1X_SensorInit(dev_);
    return last_error_ == 0;
}

bool ToF_VL53L1X::configure(uint16_t distance_mode,
                            uint16_t timing_budget_ms,
                            uint16_t inter_measurement_ms)
{
    last_error_ = VL53L1X_SetDistanceMode(dev_, distance_mode);
    if (last_error_ != 0) return false;

    last_error_ = VL53L1X_SetTimingBudgetInMs(dev_, timing_budget_ms);
    if (last_error_ != 0) return false;

    last_error_ = VL53L1X_SetInterMeasurementInMs(dev_, inter_measurement_ms);
    if (last_error_ != 0) return false;

    last_error_ = VL53L1X_SetROI(dev_, 16, 16);
    return last_error_ == 0;
}

bool ToF_VL53L1X::start()
{
    last_error_ = VL53L1X_StartRanging(dev_);
    started_ = last_error_ == 0;
    return started_;
}

bool ToF_VL53L1X::stop()
{
    last_error_ = VL53L1X_StopRanging(dev_);
    if (last_error_ == 0) started_ = false;
    return last_error_ == 0;
}

bool ToF_VL53L1X::poll()
{
    if (!started_) return false;

    uint8_t ready = 0;
    last_error_ = VL53L1X_CheckForDataReady(dev_, &ready);
    if (last_error_ != 0 || ready == 0) return false;

    last_error_ = VL53L1X_GetRangeStatus(dev_, &range_status_);
    if (last_error_ != 0) return false;

    last_error_ = VL53L1X_GetDistance(dev_, &last_raw_mm_);
    if (last_error_ != 0) return false;

    last_error_ = VL53L1X_ClearInterrupt(dev_);
    if (last_error_ != 0) return false;

    valid_ = range_status_ == 0;
    if (valid_)
    {
        if (filled_ < K) ring_[filled_++] = last_raw_mm_;
        else ring_[idx_] = last_raw_mm_;
        idx_ = (idx_ + 1) % K;

        uint16_t sorted[K];
        for (int i = 0; i < filled_; ++i) sorted[i] = ring_[i];
        std::sort(sorted, sorted + filled_);
        const uint16_t median = sorted[filled_ / 2];
        if (ema_ < 0.0f) ema_ = static_cast<float>(median);
        else ema_ = alpha_ * median + (1.0f - alpha_) * ema_;
        filtered_mm_ = static_cast<uint16_t>(ema_ + 0.5f);
    }
    return true;
}

int8_t ToF_VL53L1X::last_error() const
{
    return last_error_;
}
