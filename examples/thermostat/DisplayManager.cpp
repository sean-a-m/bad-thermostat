#include "DisplayManager.h"

#include <cstdio>

DisplayManager::DisplayManager() : tft_() {}

void DisplayManager::initialize() {
    tft_.init();
    tft_.setRotation(1);
    tft_.fillScreen(TFT_BLACK);
}

void DisplayManager::writeQuickMessage(const char* message) {
    tft_.fillScreen(TFT_BLACK);
    tft_.setTextColor(TFT_WHITE);
    tft_.setCursor(10, 10);
    tft_.print(message);
}

void DisplayManager::drawDisplay(float target_temp_f, float last_read_temp_f,
                                 bool hvac_on, float hvac_certainty, uint32_t hvac_corrections) {
    char buffer[24];

    tft_.fillScreen(TFT_BLACK);
    tft_.setTextColor(TFT_WHITE);
    tft_.setTextFont(7);
    tft_.setTextSize(1);
    tft_.setCursor(80, 80);
    formatFloat(target_temp_f, buffer, sizeof(buffer));
    tft_.print(buffer);

    tft_.setTextColor(TFT_LIGHTGREY);
    tft_.setTextFont(2);
    tft_.setTextSize(3);
    tft_.setCursor(200, 20);
    formatFloat(last_read_temp_f, buffer, sizeof(buffer));
    tft_.print(buffer);

    // Font 1 is the 6x8 px GLCD font, the smallest available.
    tft_.setTextColor(TFT_DARKGREY);
    tft_.setTextFont(1);
    tft_.setTextSize(1);
    tft_.setCursor(2, 2);
    std::snprintf(buffer, sizeof(buffer), "%s %.0f%%", hvac_on ? "on" : "off",
                  (double)(hvac_certainty * 100.0f));
    tft_.print(buffer);
    tft_.setCursor(2, 12);
    std::snprintf(buffer, sizeof(buffer), "corr %lu", (unsigned long)hvac_corrections);
    tft_.print(buffer);
}

size_t DisplayManager::formatFloat(float value, char* buffer, size_t bufferSize, uint8_t decimals) {
    return std::snprintf(buffer, bufferSize, "%.*f", (int)decimals, (double)value);
}
