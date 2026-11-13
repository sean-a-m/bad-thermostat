#pragma once

#include <TFT_eSPI.h>

class DisplayManager {
public:
    DisplayManager();

    // Wraps tft.init(), setRotation(1), and the initial clear to black.
    void initialize();

    // Full redraw of the main thermostat screen: target temp large in the
    // center, last-read temp small in the corner, and HVAC estimator status
    // (certainty in the believed on/off state, correction count, heat/cool
    // mode) in tiny text in the top left.
    void drawDisplay(float target_temp_f, float last_read_temp_f,
                     bool hvac_on, float hvac_certainty, uint32_t hvac_corrections,
                     bool heat_mode);

    // Clear the screen and show a single status/error line ("booting...",
    // sensor init errors, etc.).
    void writeQuickMessage(const char* message);

    // Access to the underlying driver, in case callers need raw drawing.
    TFT_eSPI& tft() { return tft_; }

private:
    // Format a float into a caller-provided buffer with a fixed number of
    // decimals. Returns the number of chars written (excluding the NUL).
    static size_t formatFloat(float value, char* buffer, size_t bufferSize, uint8_t decimals = 1);

    TFT_eSPI tft_;
};
