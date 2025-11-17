#include "thermo_mgr.h"

thermo_mgr::thermo_mgr(float target, float hysteresis)
    : target_temperature_(target),
      hysteresis_(hysteresis) {}

void thermo_mgr::set_target_temperature(float target) {
    target_temperature_ = target;
}

float thermo_mgr::get_target_temperature() const {
    return target_temperature_;
}

void thermo_mgr::set_hysteresis(float hysteresis) {
    hysteresis_ = hysteresis;
}

float thermo_mgr::get_hysteresis() const {
    return hysteresis_;
}

void thermo_mgr::add_measurement(float temperature, uint32_t timestamp) {
    measurements_[next_index_] = Measurement{temperature, timestamp};
    next_index_ = (next_index_ + 1) % kMaxMeasurements;
    if (count_ < kMaxMeasurements) {
        ++count_;
    }
}

bool thermo_mgr::is_over(float /*current_temperature*/) const {
    return last_n_measurements_outside_bound(true);
}

bool thermo_mgr::is_under(float /*current_temperature*/) const {
    return last_n_measurements_outside_bound(false);
}

const thermo_mgr::Measurement* thermo_mgr::get_measurements(std::size_t& out_count) const {
    out_count = count_;
    return measurements_.data();
}

bool thermo_mgr::last_n_measurements_outside_bound(bool check_over) const {
    if (count_ == 0) {
        return false;
    }

    std::size_t samples_to_check = count_ < kCheckWindow ? count_ : kCheckWindow;
    float upper_bound = target_temperature_ + hysteresis_;
    float lower_bound = target_temperature_ - hysteresis_;

    for (std::size_t i = 0; i < samples_to_check; ++i) {
        std::size_t index = (next_index_ + kMaxMeasurements - 1 - i) % kMaxMeasurements;
        float temp = measurements_[index].temperature;

        if (check_over) {
            if (temp <= upper_bound) {
                return false;
            }
        } else {
            if (temp >= lower_bound) {
                return false;
            }
        }
    }

    return samples_to_check == kCheckWindow;
}