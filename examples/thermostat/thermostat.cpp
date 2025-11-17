#include <Arduino.h>
#include <TFT_eSPI.h>
#include "pin_config.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "OneButton.h"
#include "BluetoothManager.h"
#include "thermo_mgr.h"

#define ONE_WIRE_BUS 21

// Setup OneWire instance
OneWire oneWire(ONE_WIRE_BUS);

// Pass OneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);

TFT_eSPI tft = TFT_eSPI();
BluetoothManager bluetoothManager;
OneButton button1(PIN_BUTTON_1, true);
thermo_mgr thermos; 

float desired_temp_f = 70.5;
float hysteresis = 0.7;

static uint32_t last_tick;
    


void setup()
{
    Serial.begin(115200);
    sensors.begin();

    pinMode(PIN_POWER_ON, OUTPUT);
    digitalWrite(PIN_POWER_ON, HIGH);

    button1.attachClick([]() {
        bluetoothManager.sendMessage();
    });

    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);

    tft.setTextColor(TFT_WHITE);
    tft.setCursor(10, 10);
    tft.print("test");

    bluetoothManager.initialize();
    last_tick = millis();
    thermos.set_target_temperature(72.0);
    thermos.set_hysteresis(0.8);

    Serial.println("Thermostat project initialized");
}

void loop()
{
    bluetoothManager.maintainConnection();

    // sensors.requestTemperatures();
    // float temperatureC = sensors.getTempCByIndex(0);
    // float temperatureF = sensors.getTempFByIndex(0);

    if (bluetoothManager.isConnected()) {
        button1.tick();
    }

    if(millis() - last_tick > 60000) {
        Serial.print("\nConnection status: ");
        Serial.println(bluetoothManager.isConnected());
        sensors.requestTemperatures();
        float tempF = sensors.getTempFByIndex(0);
        thermos.add_measurement(tempF, millis());
        Serial.println(tempF);
        if (thermos.is_under(tempF)) {
            Serial.println("Thermostat is under");
        } else {
            Serial.println("Thermostat is in bounds");
        }

        last_tick = millis();
    }

    // delay(1000);
}
