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

void DisplayManager::drawDisplay(float target_temp_f, float last_read_temp_f) {
    char buffer[16];

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
}

size_t DisplayManager::formatFloat(float value, char* buffer, size_t bufferSize, uint8_t decimals) {
    return std::snprintf(buffer, bufferSize, "%.*f", (int)decimals, (double)value);
}
