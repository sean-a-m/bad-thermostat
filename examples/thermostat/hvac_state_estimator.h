#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

// Bayesian estimate of whether the HVAC unit is actually on, inferred from
// room temperature and the presses we deliver. Catches presses that didn't
// take and manual toggles, which would otherwise leave machine_on wrong.
//
// Interacting multiple model (IMM) filter with two hypotheses, OFF and ON.
// Each hypothesis runs a Kalman filter over
//   T   room temperature (F)
//   d   drift from everything except the HVAC (F/min), slowly varying
//   e1  HVAC effect, first lag stage (F/min)
//   e   HVAC effect currently acting on the room (F/min)
//   c   full HVAC effect in this room (F/min), learned online
// with dT/dt = d + e, and e following (on ? c : 0) through two first-order
// lags to approximate the delay before a PTAC starts or stops moving heat.
// A delivered press flips the mode with kPressFlipProbability; otherwise the
// mode flips only with the small kSpontaneousFlipPerStep (manual toggles).
class hvac_state_estimator {
public:
    // Filter cadence. Raw readings are averaged over each step.
    static constexpr uint32_t kStepMs = 15000;
    static constexpr double kMaxStepMinutes = 2.0;

    // Process model
    static constexpr double kLagMinutes = 1.5;               // per lag stage
    static constexpr double kMeasurementVariance = 0.0025;   // (0.05 F)^2, averaged reading
    static constexpr double kTemperatureNoise = 0.0016;      // F^2 per min
    static constexpr double kDriftNoise = 1.5e-5;            // ~0.03 F/min per sqrt(hour)
    static constexpr double kEffectNoise = 4e-4;             // short-term HVAC output variation
    static constexpr double kRoomEffectNoise = 7e-6;         // ~0.02 F/min per sqrt(hour)
    static constexpr double kInitialDriftVariance = 0.0225;  // (0.15 F/min)^2
    static constexpr double kInitialOffEffectVariance = 1e-4;

    // Prior on the room's HVAC effect when nothing was restored from flash.
    // Broad enough that the estimator is uncalibrated until it has watched
    // a few presses.
    static constexpr double kDefaultEffect = 0.15;           // magnitude, F/min
    static constexpr double kDefaultEffectVariance = 0.09;   // (0.3 F/min)^2
    static constexpr double kRestoredEffectVarianceInflation = 0.0004;

    // Mode transitions
    static constexpr double kPressFlipProbability = 0.95;
    static constexpr double kSpontaneousFlipPerStep = 1e-4;
    static constexpr double kBootConfidence = 0.95;
    static constexpr double kProbabilityFloor = 1e-6;
    // Caps how far one step can move the mode probability, so a single
    // reading the model explains badly (door opened, sensor glitch) can't
    // decide the mode on its own.
    static constexpr double kMaxStepEvidence = 2.0;          // nats

    // No mode evidence is taken for this long after boot or a press. Cooling
    // has to cover compressor restart delay and coil cool-down. Heating is a
    // fan over a steam/hot water coil, so it responds almost immediately.
    static constexpr uint32_t kCoolSettleMs = 5UL * 60UL * 1000UL;
    static constexpr uint32_t kHeatSettleMs = 30UL * 1000UL;

    // Correction policy
    static constexpr double kCorrectionProbability = 0.95;
    static constexpr uint32_t kSustainMs = 2UL * 60UL * 1000UL;
    // Corrections not followed by a confirmed press back off from the
    // minimum, doubling up to the maximum, so a unit that isn't responding
    // at all (e.g. no heat source, fan only) isn't toggled endlessly.
    static constexpr uint32_t kMinCorrectionIntervalMs = 30UL * 60UL * 1000UL;
    static constexpr uint32_t kMaxCorrectionIntervalMs = 8UL * 60UL * 60UL * 1000UL;
    // A press is confirmed when, within this window after settling, the
    // accumulated evidence favors the expected state by kConfirmEvidence.
    static constexpr uint32_t kConfirmWindowMs = 15UL * 60UL * 1000UL;
    static constexpr double kConfirmEvidence = 4.6;          // ~100:1

    // The estimator only acts once the learned effect is clearly nonzero
    // with the right sign for the current mode.
    static constexpr double kMinEffect = 0.03;               // F/min
    static constexpr double kCalibrationSigmas = 3.0;

    // Persisting the learned effect
    static constexpr uint32_t kPersistIntervalMs = 60UL * 60UL * 1000UL;
    static constexpr double kPersistMinChange = 0.005;       // F/min

    hvac_state_estimator();

    // heat_mode sets the expected sign of the HVAC effect; believed_on seeds
    // the mode probability. Call once at boot.
    void begin(bool heat_mode, bool believed_on, uint32_t now_ms);

    // Seed the learned HVAC effect from flash. Call after begin() and before
    // the first measurement. Ignored if implausible or the wrong sign for
    // the current mode.
    void restore_effect(float mean, float variance);

    // Feed every raw reading. Returns true when the filter advanced a step.
    bool add_measurement(float temperature, uint32_t now_ms);

    // Call when a press was delivered. expect_on is the state the controller
    // believes the press produced.
    void note_press(bool expect_on, uint32_t now_ms);

    // True when the estimate has confidently disagreed with believed_on for
    // a while and a correction is allowed. Call note_correction() after
    // acting on it.
    bool should_correct(bool believed_on, uint32_t now_ms);
    void note_correction(uint32_t now_ms);

    // True (and fills mean/variance) when the learned effect should be
    // written to flash. Rate limited to keep NVS writes rare.
    bool take_effect_to_persist(uint32_t now_ms, float& mean, float& variance);

    float p_on() const;
    // Diagnostic only: what p_on would be if temperature evidence were also
    // taken during settling. Uses the real filters, so it is approximate.
    float p_on_ignoring_settle() const;
    float drift() const;
    float effect() const;
    float effect_stddev() const;
    bool is_calibrated() const;
    bool is_settling(uint32_t now_ms) const;

private:
    static constexpr std::size_t kStateSize = 5;
    static constexpr std::size_t kT = 0;
    static constexpr std::size_t kD = 1;
    static constexpr std::size_t kE1 = 2;
    static constexpr std::size_t kE = 3;
    static constexpr std::size_t kC = 4;

    static constexpr std::size_t kOff = 0;
    static constexpr std::size_t kOn = 1;

    using Vector = std::array<double, kStateSize>;
    using Matrix = std::array<Vector, kStateSize>;

    struct Filter {
        Vector x;
        Matrix P;
    };

    static Filter mix(const Filter& a, double weight_a, const Filter& b, double weight_b);
    static void predict(Filter& filter, bool on, double dt_min);
    static double update(Filter& filter, double measurement);

    void init_filters(double temperature);
    void step(double measurement, double dt_min, uint32_t now_ms);
    Filter combined() const;
    uint32_t correction_backoff_ms() const;

    double effect_sign_ = -1.0;
    double effect_prior_mean_ = -kDefaultEffect;
    double effect_prior_variance_ = kDefaultEffectVariance;

    bool initialized_ = false;
    std::array<Filter, 2> filters_{};
    std::array<double, 2> mode_probability_{{0.5, 0.5}};
    double unsettled_p_on_ = 0.5;

    double reading_sum_ = 0.0;
    uint32_t reading_count_ = 0;
    uint32_t last_step_ms_ = 0;

    bool press_pending_ = false;
    uint32_t settle_ms_ = kCoolSettleMs;
    uint32_t settle_until_ms_ = 0;

    bool confirming_ = false;
    bool confirm_expect_on_ = false;
    double confirm_evidence_ = 0.0;
    uint32_t confirm_deadline_ms_ = 0;

    bool disagreeing_ = false;
    uint32_t disagree_since_ms_ = 0;
    uint32_t last_correction_ms_ = 0;
    uint32_t corrections_since_confirm_ = 0;

    bool persisted_ = false;
    uint32_t last_persist_ms_ = 0;
    double last_persisted_effect_ = 0.0;
};
