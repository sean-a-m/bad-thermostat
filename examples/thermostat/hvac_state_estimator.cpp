#include "hvac_state_estimator.h"

#include <cmath>

namespace {
constexpr double kLog2Pi = 1.8378770664093453;

// millis()-style wraparound-safe "a is earlier than b"
bool time_before(uint32_t a, uint32_t b) {
    return static_cast<int32_t>(a - b) < 0;
}

double clamp(double value, double low, double high) {
    return value < low ? low : (value > high ? high : value);
}
}  // namespace

hvac_state_estimator::hvac_state_estimator() = default;

void hvac_state_estimator::begin(bool heat_mode, bool believed_on, uint32_t now_ms) {
    *this = hvac_state_estimator();
    effect_sign_ = heat_mode ? 1.0 : -1.0;
    effect_prior_mean_ = effect_sign_ * kDefaultEffect;
    effect_prior_variance_ = kDefaultEffectVariance;
    mode_probability_[kOn] = believed_on ? kBootConfidence : 1.0 - kBootConfidence;
    mode_probability_[kOff] = 1.0 - mode_probability_[kOn];
    unsettled_p_on_ = mode_probability_[kOn];
    settle_ms_ = heat_mode ? kHeatSettleMs : kCoolSettleMs;
    settle_until_ms_ = now_ms + settle_ms_;
}

void hvac_state_estimator::restore_effect(float mean, float variance) {
    if (!std::isfinite(mean) || !std::isfinite(variance) || variance <= 0.0f) {
        return;
    }
    if (effect_sign_ * mean < kMinEffect) {
        return;
    }
    effect_prior_mean_ = mean;
    effect_prior_variance_ = variance + kRestoredEffectVarianceInflation;
}

bool hvac_state_estimator::add_measurement(float temperature, uint32_t now_ms) {
    if (!std::isfinite(temperature)) {
        return false;
    }

    if (!initialized_) {
        init_filters(temperature);
        last_step_ms_ = now_ms;
        initialized_ = true;
        return false;
    }

    reading_sum_ += temperature;
    ++reading_count_;
    if (now_ms - last_step_ms_ < kStepMs) {
        return false;
    }

    double dt_min = (now_ms - last_step_ms_) / 60000.0;
    if (dt_min > kMaxStepMinutes) {
        dt_min = kMaxStepMinutes;
    }
    step(reading_sum_ / reading_count_, dt_min, now_ms);

    reading_sum_ = 0.0;
    reading_count_ = 0;
    last_step_ms_ = now_ms;
    return true;
}

void hvac_state_estimator::set_known_state(bool on, uint32_t now_ms) {
    mode_probability_[kOn] = on ? kBootConfidence : 1.0 - kBootConfidence;
    mode_probability_[kOff] = 1.0 - mode_probability_[kOn];
    unsettled_p_on_ = mode_probability_[kOn];
    press_pending_ = false;
    settle_until_ms_ = now_ms + settle_ms_;
    confirming_ = false;
    disagreeing_ = false;
}

void hvac_state_estimator::note_press(bool expect_on, uint32_t now_ms) {
    press_pending_ = true;
    settle_until_ms_ = now_ms + settle_ms_;
    confirming_ = true;
    confirm_expect_on_ = expect_on;
    confirm_evidence_ = 0.0;
    confirm_deadline_ms_ = settle_until_ms_ + kConfirmWindowMs;
    disagreeing_ = false;
}

bool hvac_state_estimator::should_correct(bool believed_on, uint32_t now_ms) {
    if (!initialized_ || is_settling(now_ms) || !is_calibrated()) {
        disagreeing_ = false;
        return false;
    }

    double p_wrong = believed_on ? mode_probability_[kOff] : mode_probability_[kOn];
    if (p_wrong < kCorrectionProbability) {
        disagreeing_ = false;
        return false;
    }

    if (!disagreeing_) {
        disagreeing_ = true;
        disagree_since_ms_ = now_ms;
    }
    if (now_ms - disagree_since_ms_ < kSustainMs) {
        return false;
    }

    if (corrections_since_confirm_ > 0 &&
        now_ms - last_correction_ms_ < correction_backoff_ms()) {
        return false;
    }

    return true;
}

void hvac_state_estimator::note_correction(uint32_t now_ms) {
    last_correction_ms_ = now_ms;
    ++corrections_since_confirm_;
    disagreeing_ = false;
}

bool hvac_state_estimator::take_effect_to_persist(uint32_t now_ms, float& mean, float& variance) {
    if (!initialized_ || is_settling(now_ms) || !is_calibrated()) {
        return false;
    }
    // Don't persist a blend of two hypotheses that disagree.
    if (mode_probability_[kOn] < kCorrectionProbability &&
        mode_probability_[kOff] < kCorrectionProbability) {
        return false;
    }
    if (persisted_ && now_ms - last_persist_ms_ < kPersistIntervalMs) {
        return false;
    }

    Filter estimate = combined();
    if (persisted_ && std::fabs(estimate.x[kC] - last_persisted_effect_) < kPersistMinChange) {
        return false;
    }

    mean = static_cast<float>(estimate.x[kC]);
    variance = static_cast<float>(estimate.P[kC][kC]);
    persisted_ = true;
    last_persist_ms_ = now_ms;
    last_persisted_effect_ = estimate.x[kC];
    return true;
}

float hvac_state_estimator::p_on() const {
    return static_cast<float>(mode_probability_[kOn]);
}

float hvac_state_estimator::p_on_ignoring_settle() const {
    return static_cast<float>(unsettled_p_on_);
}

float hvac_state_estimator::drift() const {
    return initialized_ ? static_cast<float>(combined().x[kD]) : 0.0f;
}

float hvac_state_estimator::effect() const {
    return initialized_ ? static_cast<float>(combined().x[kC])
                        : static_cast<float>(effect_prior_mean_);
}

float hvac_state_estimator::effect_stddev() const {
    double variance = initialized_ ? combined().P[kC][kC] : effect_prior_variance_;
    return static_cast<float>(std::sqrt(variance));
}

bool hvac_state_estimator::is_calibrated() const {
    if (!initialized_) {
        return false;
    }
    Filter estimate = combined();
    double signed_effect = effect_sign_ * estimate.x[kC];
    return signed_effect >= kMinEffect &&
           signed_effect >= kCalibrationSigmas * std::sqrt(estimate.P[kC][kC]);
}

bool hvac_state_estimator::is_settling(uint32_t now_ms) const {
    return time_before(now_ms, settle_until_ms_);
}

hvac_state_estimator::Filter hvac_state_estimator::mix(const Filter& a, double weight_a,
                                                       const Filter& b, double weight_b) {
    Filter mixed{};
    for (std::size_t i = 0; i < kStateSize; ++i) {
        mixed.x[i] = weight_a * a.x[i] + weight_b * b.x[i];
    }
    for (std::size_t i = 0; i < kStateSize; ++i) {
        for (std::size_t j = 0; j < kStateSize; ++j) {
            double spread_a = (a.x[i] - mixed.x[i]) * (a.x[j] - mixed.x[j]);
            double spread_b = (b.x[i] - mixed.x[i]) * (b.x[j] - mixed.x[j]);
            mixed.P[i][j] = weight_a * (a.P[i][j] + spread_a) + weight_b * (b.P[i][j] + spread_b);
        }
    }
    return mixed;
}

void hvac_state_estimator::predict(Filter& filter, bool on, double dt_min) {
    double retain = std::exp(-dt_min / kLagMinutes);
    double approach = 1.0 - retain;

    Matrix F{};
    for (std::size_t i = 0; i < kStateSize; ++i) {
        F[i][i] = 1.0;
    }
    F[kT][kD] = dt_min;
    F[kT][kE] = dt_min;
    F[kE1][kE1] = retain;
    F[kE1][kC] = on ? approach : 0.0;
    F[kE][kE] = retain;
    F[kE][kE1] = approach;

    Vector x{};
    for (std::size_t i = 0; i < kStateSize; ++i) {
        for (std::size_t k = 0; k < kStateSize; ++k) {
            x[i] += F[i][k] * filter.x[k];
        }
    }

    Matrix FP{};
    for (std::size_t i = 0; i < kStateSize; ++i) {
        for (std::size_t j = 0; j < kStateSize; ++j) {
            for (std::size_t k = 0; k < kStateSize; ++k) {
                FP[i][j] += F[i][k] * filter.P[k][j];
            }
        }
    }
    Matrix P{};
    for (std::size_t i = 0; i < kStateSize; ++i) {
        for (std::size_t j = 0; j < kStateSize; ++j) {
            for (std::size_t k = 0; k < kStateSize; ++k) {
                P[i][j] += FP[i][k] * F[j][k];
            }
        }
    }
    P[kT][kT] += kTemperatureNoise * dt_min;
    P[kD][kD] += kDriftNoise * dt_min;
    P[kE1][kE1] += kEffectNoise * dt_min;
    P[kE][kE] += kEffectNoise * dt_min;
    P[kC][kC] += kRoomEffectNoise * dt_min;

    filter.x = x;
    filter.P = P;
}

// Measurement update for a direct temperature reading. Returns the log
// likelihood of the reading under this filter's prediction.
double hvac_state_estimator::update(Filter& filter, double measurement) {
    double innovation = measurement - filter.x[kT];
    double innovation_variance = filter.P[kT][kT] + kMeasurementVariance;

    Vector gain{};
    for (std::size_t i = 0; i < kStateSize; ++i) {
        gain[i] = filter.P[i][kT] / innovation_variance;
        filter.x[i] += gain[i] * innovation;
    }

    Matrix P = filter.P;
    for (std::size_t i = 0; i < kStateSize; ++i) {
        for (std::size_t j = 0; j < kStateSize; ++j) {
            P[i][j] -= gain[i] * filter.P[kT][j];
        }
    }
    // Keep P symmetric against rounding drift over long uptimes.
    for (std::size_t i = 0; i < kStateSize; ++i) {
        for (std::size_t j = i + 1; j < kStateSize; ++j) {
            double average = 0.5 * (P[i][j] + P[j][i]);
            P[i][j] = average;
            P[j][i] = average;
        }
    }
    filter.P = P;

    return -0.5 * (kLog2Pi + std::log(innovation_variance) +
                   innovation * innovation / innovation_variance);
}

void hvac_state_estimator::init_filters(double temperature) {
    for (std::size_t mode = kOff; mode <= kOn; ++mode) {
        Filter& filter = filters_[mode];
        filter = Filter{};
        filter.x[kT] = temperature;
        filter.x[kC] = effect_prior_mean_;
        filter.P[kT][kT] = kMeasurementVariance;
        filter.P[kD][kD] = kInitialDriftVariance;
        filter.P[kC][kC] = effect_prior_variance_;

        if (mode == kOn) {
            // Assume steady state: the effect has fully ramped up to c, so
            // e1, e and c are the same uncertain quantity.
            filter.x[kE1] = effect_prior_mean_;
            filter.x[kE] = effect_prior_mean_;
            for (std::size_t i : {kE1, kE, kC}) {
                for (std::size_t j : {kE1, kE, kC}) {
                    filter.P[i][j] = effect_prior_variance_;
                }
            }
        } else {
            filter.P[kE1][kE1] = kInitialOffEffectVariance;
            filter.P[kE][kE] = kInitialOffEffectVariance;
        }
    }
}

void hvac_state_estimator::step(double measurement, double dt_min, uint32_t now_ms) {
    double flip = press_pending_ ? kPressFlipProbability : kSpontaneousFlipPerStep;
    press_pending_ = false;
    double transition[2][2] = {{1.0 - flip, flip}, {flip, 1.0 - flip}};

    // IMM mixing: each hypothesis starts from a blend of both filters,
    // weighted by how likely each was to have transitioned into it.
    std::array<double, 2> predicted{};
    for (std::size_t to = kOff; to <= kOn; ++to) {
        predicted[to] = transition[kOff][to] * mode_probability_[kOff] +
                        transition[kOn][to] * mode_probability_[kOn];
    }
    std::array<Filter, 2> mixed;
    for (std::size_t to = kOff; to <= kOn; ++to) {
        double weight_off = transition[kOff][to] * mode_probability_[kOff] / predicted[to];
        double weight_on = transition[kOn][to] * mode_probability_[kOn] / predicted[to];
        mixed[to] = mix(filters_[kOff], weight_off, filters_[kOn], weight_on);
    }

    std::array<double, 2> log_likelihood{};
    for (std::size_t mode = kOff; mode <= kOn; ++mode) {
        filters_[mode] = mixed[mode];
        predict(filters_[mode], mode == kOn, dt_min);
        log_likelihood[mode] = update(filters_[mode], measurement);
    }

    double evidence_on = clamp(log_likelihood[kOn] - log_likelihood[kOff],
                               -kMaxStepEvidence, kMaxStepEvidence);

    // Diagnostic only: the same recursion, but never skipping the evidence.
    double unsettled_predicted_on = transition[kOff][kOn] * (1.0 - unsettled_p_on_) +
                                    transition[kOn][kOn] * unsettled_p_on_;
    double unsettled_weight_on = unsettled_predicted_on * std::exp(evidence_on);
    unsettled_p_on_ = clamp(unsettled_weight_on / (unsettled_weight_on + 1.0 - unsettled_predicted_on),
                            kProbabilityFloor, 1.0 - kProbabilityFloor);

    if (is_settling(now_ms)) {
        mode_probability_ = predicted;
    } else {
        double weight_on = predicted[kOn] * std::exp(evidence_on);
        mode_probability_[kOn] = weight_on / (weight_on + predicted[kOff]);

        if (confirming_) {
            if (time_before(confirm_deadline_ms_, now_ms)) {
                confirming_ = false;
            } else {
                confirm_evidence_ += confirm_expect_on_ ? evidence_on : -evidence_on;
                if (confirm_evidence_ >= kConfirmEvidence) {
                    confirming_ = false;
                    corrections_since_confirm_ = 0;
                }
            }
        }
    }

    mode_probability_[kOn] = clamp(mode_probability_[kOn], kProbabilityFloor, 1.0 - kProbabilityFloor);
    mode_probability_[kOff] = 1.0 - mode_probability_[kOn];
}

hvac_state_estimator::Filter hvac_state_estimator::combined() const {
    return mix(filters_[kOff], mode_probability_[kOff], filters_[kOn], mode_probability_[kOn]);
}

uint32_t hvac_state_estimator::correction_backoff_ms() const {
    uint32_t backoff = kMinCorrectionIntervalMs;
    for (uint32_t i = 1; i < corrections_since_confirm_ && backoff < kMaxCorrectionIntervalMs; ++i) {
        backoff *= 2;
    }
    return backoff < kMaxCorrectionIntervalMs ? backoff : kMaxCorrectionIntervalMs;
}
