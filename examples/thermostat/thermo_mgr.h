#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

class thermo_mgr {
public:
    static constexpr std::size_t kMaxMeasurements = 50;
    static constexpr std::size_t kCheckWindow = 5;

    struct Measurement {
        float temperature;
        uint32_t timestamp;
    };

    thermo_mgr(float target = 21.0f, float hysteresis = 0.5f);

    void set_target_temperature(float target);
    float get_target_temperature() const;

    void set_hysteresis(float hysteresis);
    float get_hysteresis() const;

    void add_measurement(float temperature, uint32_t timestamp);

    bool is_over(float current_temperature) const;
    bool is_under(float current_temperature) const;

    const Measurement* get_measurements(std::size_t& out_count) const;

private:
    float target_temperature_;
    float hysteresis_;

    std::array<Measurement, kMaxMeasurements> measurements_{};
    std::size_t next_index_ = 0;
    std::size_t count_ = 0;

    bool last_n_measurements_outside_bound(bool check_over) const;
};