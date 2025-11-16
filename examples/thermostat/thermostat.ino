#include <Arduino.h>
#include <TFT_eSPI.h>
#include "pin_config.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 21

// Setup OneWire instance
OneWire oneWire(ONE_WIRE_BUS);

// Pass OneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);


TFT_eSPI tft = TFT_eSPI();

void setup()
{
    Serial.begin(115200);
    sensors.begin();
    
    pinMode(PIN_POWER_ON, OUTPUT);
    digitalWrite(PIN_POWER_ON, HIGH);
    
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(10, 10);
    tft.print("test");
    
    Serial.println("Thermostat project initialized");
}

void loop()
{
    sensors.requestTemperatures();
        // Get temperature in Celsius
    float temperatureC = sensors.getTempCByIndex(0);
    
    // Get temperature in Fahrenheit
    float temperatureF = sensors.getTempFByIndex(0);
    
    // Print results
    Serial.print("Temperature: ");
    Serial.print(temperatureC);
    Serial.print("°C / ");
    Serial.print(temperatureF);
    Serial.println("°F");

    delay(1000);
}
