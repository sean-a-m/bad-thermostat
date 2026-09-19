# define USE_LIVING_ROOM_ADDRESS

#include <Arduino.h>
#include "DisplayManager.h"
#include "pin_config.h"
#include "OneButton.h"
#include "BluetoothManager.h"
#include "thermo_mgr.h"
#include <esp_sleep.h>
#include <esp_task_wdt.h>
#include <Preferences.h>

// Define which sensor to use - comment/uncomment to switch
// #define USE_ONEWIRE_SENSOR
#define USE_SHT45_SENSOR


#ifdef USE_ONEWIRE_SENSOR
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 21
#endif

#ifdef USE_SHT45_SENSOR
#include <Adafruit_SHT4x.h>
#include <Wire.h>
#define SHT45_SDA 18
#define SHT45_SCL 17
#endif

#define WDT_TIMEOUT 10  // Watchdog timeout in seconds

// Periodically reboot to recover from the BLE stack wedging itself after
// very long uptimes. All persisted state lives in NVS, so a reboot is safe.
static const uint32_t REBOOT_INTERVAL_MS = 24UL * 60UL * 60UL * 1000UL;  // 24 hours

// Escape hatch for a wedged BLE stack: if the connection keeps failing this
// many times in a row (despite backoff), reboot rather than retry forever.
// With the backoff capping at 60s this is roughly 8-10 minutes of failures.
static const uint32_t BLE_MAX_CONSECUTIVE_FAILURES = 12;

// constexpr uint64_t    uS_TO_S_FACTOR = 1000000ULL;
// constexpr uint64_t    WAKE_INTERVAL_S = 10; // seconds

static const uint32_t NVS_MAGIC = 0xBAD0CAFE;
Preferences prefs;
boolean machine_on = false;
float target_temp_f = 70.5;

void nvs_save_machine_on() {
    prefs.putBool("machine_on", machine_on);
}

void nvs_save_target_temp() {
    prefs.putFloat("target_temp", target_temp_f);
}


// Display is 170 * 320

#ifdef USE_ONEWIRE_SENSOR
// Setup OneWire instance
OneWire oneWire(ONE_WIRE_BUS);
// Pass OneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);
#endif

#ifdef USE_SHT45_SENSOR
// Setup SHT45 sensor
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
#endif

sensors_event_t humidity, temp;

DisplayManager display;
BluetoothManager bluetoothManager;
OneButton button1(PIN_BUTTON_1, true);
OneButton button2(PIN_BUTTON_2, true);
thermo_mgr thermos; 

// float desired_temp_f = 70.5;
float hysteresis = 0.75;
float last_read_temp_f = 0.0;

static uint32_t last_temperature_request_tick;
static uint32_t last_temperature_read_tick;
static uint32_t last_ble_tick;


static boolean message_send_queued = false;
static boolean heat_mode = false;
// static boolean machine_on = false;

static boolean should_redraw_display = false;
    
void scanForDevices() {
   byte error, address;
  int nDevices = 0;

  Serial.println("Scanning...");

  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

}

void setup()
{
    Serial.begin(115200);
    
    // Initialize watchdog timer
    esp_task_wdt_init(WDT_TIMEOUT, true);  // 10 second timeout, panic on timeout
    esp_task_wdt_add(NULL);  // Add current thread to WDT watch

    // Restore persisted state from NVS
    prefs.begin("thermostat", false);
    if (prefs.getUInt("magic", 0) != NVS_MAGIC) {
        Serial.println("First boot: initializing NVS with defaults");
        prefs.putUInt("magic", NVS_MAGIC);
        prefs.putBool("machine_on", machine_on);
        prefs.putFloat("target_temp", target_temp_f);
    } else {
        machine_on = prefs.getBool("machine_on", false);
        target_temp_f = prefs.getFloat("target_temp", 70.5);
        Serial.print("Restored from NVS: machine_on=");
        Serial.print(machine_on);
        Serial.print(" target_temp=");
        Serial.println(target_temp_f);
    }
    thermos.set_target_temperature(target_temp_f);
    
    // Initialize temperature sensor
#ifdef USE_ONEWIRE_SENSOR
    Serial.println("Initializing OneWire DS18B20 sensor...");
    sensors.begin();
    sensors.setWaitForConversion(false);
#endif

#ifdef USE_SHT45_SENSOR
    Serial.println("Initializing SHT45 sensor...");
    display.writeQuickMessage("Initializing SHT45 sensor");
    Wire.begin(43, 44);
    scanForDevices();
    //Wire.begin();
    if (!sht4.begin()) {
        Serial.println("ERROR: Couldn't find SHT45 sensor!");
        display.writeQuickMessage("ERROR: Couldn't find SHT45 sensor!");

        while (1) delay(1);
    }
        Serial.println("Found SHT45 sensor");
        Serial.print("Serial number 0x");
        Serial.println(sht4.readSerial(), HEX);
        
        // Set high precision for accurate readings
        sht4.setPrecision(SHT4X_HIGH_PRECISION);
        sht4.setHeater(SHT4X_NO_HEATER);
#endif

    pinMode(PIN_POWER_ON, OUTPUT);
    digitalWrite(PIN_POWER_ON, HIGH);

    button1.attachClick([]() {
        float current_target_temperature = thermos.get_target_temperature();
        thermos.set_target_temperature(current_target_temperature - 0.5);
        target_temp_f = thermos.get_target_temperature();
        nvs_save_target_temp();
        should_redraw_display = true;
    });

    button2.attachClick([]() {
        float current_target_temperature = thermos.get_target_temperature();
        thermos.set_target_temperature(current_target_temperature + 0.5);
        target_temp_f = thermos.get_target_temperature();
        nvs_save_target_temp();
        should_redraw_display = true;
    });

//    esp_sleep_enable_ext1_wakeup(1ULL << PIN_BUTTON_1, ESP_EXT1_WAKEUP_ALL_LOW);
//    esp_sleep_enable_ext1_wakeup(1ULL << PIN_BUTTON_2, ESP_EXT1_WAKEUP_ALL_LOW);
//    esp_sleep_enable_timer_wakeup(WAKE_INTERVAL_S * uS_TO_S_FACTOR);

 //   ledcAttachPin(PIN_LCD_BL, 0);
 //   ledcWrite(0, 10); /* Screen brightness can be modified by adjusting this parameter. (0-255) */

    display.initialize();
    display.writeQuickMessage("booting...");
    

    bluetoothManager.initialize();
    last_temperature_read_tick = millis();
    last_temperature_request_tick = millis();
    last_ble_tick = millis();
    thermos.set_target_temperature(target_temp_f);
    thermos.set_hysteresis(hysteresis);

    Serial.println("Thermostat project initialized");
}

void queue_message_if_needed(float temperature) {
    if (heat_mode) {
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
    } else {
        if (message_send_queued == true) {
            return;
        }
        if (!machine_on && thermos.is_over(temperature)) {
            machine_on = true;
            message_send_queued = true;
        }

        if (machine_on && thermos.is_under(temperature)) {
            machine_on = false;
            message_send_queued = true;
        }
    }

}

boolean isTemperatureReadReady() {
    #ifdef USE_ONEWIRE_SENSOR
    return sensors.isConversionComplete();
    #endif

    #ifdef USE_SHT45_SENSOR
    return true;
    #endif

}

float getTempF() {
    #ifdef USE_ONEWIRE_SENSOR
    return sensors.getTempFByIndex(0);
    #endif

    #ifdef USE_SHT45_SENSOR
    sht4.getEvent(&humidity, &temp);
    return temp.temperature * (9.0f/5.0f) + 32.0;
    #endif

}


void loop()
{
    // Feed the watchdog timer to prevent reset
    esp_task_wdt_reset();

    // Scheduled reboot to recover from the BLE stack failing after long
    // uptimes. State is persisted in NVS so this is non-destructive.
    if (millis() > REBOOT_INTERVAL_MS) {
        Serial.println("24h uptime reached, rebooting...");
        Serial.flush();
        esp_restart();
    }

    // Periodic keepalive, plus a backoff-gated retry when a message is
    // waiting. Gating the retry through beginConnectionRetryIfDue() stops the
    // old behavior where a queued message hammered maintainConnection() every
    // single loop iteration once the link couldn't be (re)established.
    bool keepalive_due = (millis() - last_ble_tick > 60000);
    bool retry_due = message_send_queued && bluetoothManager.beginConnectionRetryIfDue();

    if (keepalive_due || retry_due) {
        Serial.print("\nConnection status: ");
        Serial.println(bluetoothManager.isConnected());
        bluetoothManager.maintainConnection();
        last_ble_tick = millis();
    }

    button1.tick();
    button2.tick();

    if (message_send_queued) {
        if (bluetoothManager.isConnected() && bluetoothManager.sendMessage()) {
            message_send_queued = false;
            bluetoothManager.noteConnectionSuccess();
            nvs_save_machine_on();
        } else if (retry_due) {
            // Backoff window elapsed and we still couldn't deliver.
            bluetoothManager.noteConnectionFailure();
        }
    }

    if (bluetoothManager.consecutiveConnectionFailures() >= BLE_MAX_CONSECUTIVE_FAILURES) {
        Serial.println("BLE unrecoverable after repeated failures, rebooting...");
        Serial.flush();
        esp_restart();
    }

    // only need to queue up reading for onewire
    #ifdef USE_ONEWIRE_SENSOR
    if(millis() - last_temperature_request_tick > 1000) {
        sensors.requestTemperatures();
        last_temperature_request_tick = millis();
    }
    #endif

    if(millis() - last_temperature_read_tick > 1000 && isTemperatureReadReady()) {
        // sensors.requestTemperatures(); probably not needed since we did it earlier
        float tempF = getTempF();
        thermos.add_measurement(tempF, millis());
        Serial.println(tempF);
        if (thermos.is_over(tempF)) {
            Serial.println("Thermostat is over");
        } else {
            Serial.println("Thermostat is in bounds");
        }
        last_read_temp_f = tempF;

        queue_message_if_needed(tempF);

        should_redraw_display = true;

        last_temperature_read_tick = millis();
    }

    if (should_redraw_display) {
        display.drawDisplay(thermos.get_target_temperature(), last_read_temp_f);
        should_redraw_display = false;
    }

    // TODO: need to disable bluetooth explicitly?
    // esp_deep_sleep_start();



    // delay(1000);
}


