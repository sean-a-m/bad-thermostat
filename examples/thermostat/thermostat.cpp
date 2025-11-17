#include <Arduino.h>
#include <TFT_eSPI.h>
#include "pin_config.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "OneButton.h"
#include "BluetoothManager.h"
#include "thermo_mgr.h"


#define ONE_WIRE_BUS 21

// Display is 170 * 320

// Setup OneWire instance
OneWire oneWire(ONE_WIRE_BUS);

// Pass OneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);

TFT_eSPI tft = TFT_eSPI();
BluetoothManager bluetoothManager;
OneButton button1(PIN_BUTTON_1, true);
OneButton button2(PIN_BUTTON_2, true);
thermo_mgr thermos; 

float desired_temp_f = 70.5;
float hysteresis = 0.7;
float last_read_temp_f = 0.0;

static uint32_t last_temperature_request_tick;
static uint32_t last_temperature_read_tick;
static uint32_t last_ble_tick;


static boolean message_send_queued = false;
static boolean heat_mode = true;
static boolean machine_on = false;

static boolean should_redraw_display = false;
    


void setup()
{
    Serial.begin(115200);
    sensors.begin();
    sensors.setWaitForConversion(false);

    pinMode(PIN_POWER_ON, OUTPUT);
    digitalWrite(PIN_POWER_ON, HIGH);

    button1.attachClick([]() {
        float current_target_temperature = thermos.get_target_temperature();
        thermos.set_target_temperature(current_target_temperature - 0.5);
        should_redraw_display = true;
    });

    button2.attachClick([]() {
        float current_target_temperature = thermos.get_target_temperature();
        thermos.set_target_temperature(current_target_temperature + 0.5);
        should_redraw_display = true;
    });

    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);

    tft.setTextColor(TFT_WHITE);
    tft.setCursor(10, 10);
    tft.print("test");
    

    bluetoothManager.initialize();
    last_temperature_read_tick = millis();
    last_temperature_request_tick = millis();
    last_ble_tick = millis();
    thermos.set_target_temperature(72.0);
    thermos.set_hysteresis(0.8);

    Serial.println("Thermostat project initialized");
}

void print_formatted_float(float number) {
    char buffer[16];
    std::snprintf(buffer, sizeof(buffer), "%.1f", number);
    std::string s(buffer);
    tft.print(buffer);

}

void drawDisplay() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextFont(7);
    tft.setTextSize(1);
    tft.setCursor(80, 80);

    print_formatted_float(thermos.get_target_temperature());


    //tft.drawNumber(temperature, 1, 80, 30);
    tft.setTextColor(TFT_LIGHTGREY);
    tft.setTextFont(2);
    tft.setTextSize(3);
    tft.setCursor(200, 20);
    print_formatted_float(last_read_temp_f);
}

void queue_message_if_needed(float temperature) {
    if (message_send_queued == true) {
        return;
    }
    if (machine_on && thermos.is_over(temperature)) {
        machine_on = false;
        message_send_queued = true;

    }

    if (!machine_on && thermos.is_under(temperature)) {
        machine_on = true;
        message_send_queued = true;
    }

}


void loop()
{

    if (millis() - last_ble_tick > 60000 || message_send_queued) {
        Serial.print("\nConnection status: ");
        Serial.println(bluetoothManager.isConnected());
        bluetoothManager.maintainConnection();
        last_ble_tick = millis();
    }

    // sensors.requestTemperatures();
    // float temperatureC = sensors.getTempCByIndex(0);
    // float temperatureF = sensors.getTempFByIndex(0);

    button1.tick();
    button2.tick();

    if (message_send_queued && bluetoothManager.isConnected()) {
        if(bluetoothManager.sendMessage()) {
            message_send_queued = false;
        }
    }

    if(millis() - last_temperature_request_tick > 1000) {
        sensors.requestTemperatures();
        last_temperature_request_tick = millis();
    }

    if(millis() - last_temperature_read_tick > 1000 && sensors.isConversionComplete()) {
        sensors.requestTemperatures();
        float tempF = sensors.getTempFByIndex(0);
        thermos.add_measurement(tempF, millis());
        Serial.println(tempF);
        if (thermos.is_under(tempF)) {
            Serial.println("Thermostat is under");
        } else {
            Serial.println("Thermostat is in bounds");
        }
        last_read_temp_f = tempF;

        queue_message_if_needed(tempF);

        should_redraw_display = true;

        last_temperature_read_tick = millis();
    }

    if (should_redraw_display) {
        drawDisplay();
        should_redraw_display = false;
    }



    // delay(1000);
}
