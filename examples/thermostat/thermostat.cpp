# define USE_LIVING_ROOM_ADDRESS

#include <Arduino.h>
#include "DisplayManager.h"
#include "pin_config.h"
#include "OneButton.h"
#include "BluetoothManager.h"
#include "thermo_mgr.h"
#include "hvac_state_estimator.h"
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

// Escape hatch for a wedged BLE stack: if delivering a press keeps failing
// this many times in a row (despite backoff), reboot rather than retry
// forever. With the backoff capping at 60s this is roughly 10 minutes.
static const uint32_t BLE_MAX_CONSECUTIVE_FAILURES = 12;

// constexpr uint64_t    uS_TO_S_FACTOR = 1000000ULL;
// constexpr uint64_t    WAKE_INTERVAL_S = 10; // seconds

static const uint32_t NVS_MAGIC = 0xBAD0CAFE;
Preferences prefs;
boolean machine_on = false;
float target_temp_f = 70.5;
// Times the HVAC estimator found machine_on wrong and resynced it.
uint32_t hvac_correction_count = 0;

void nvs_save_machine_on() {
    prefs.putBool("machine_on", machine_on);
}

void nvs_save_target_temp() {
    prefs.putFloat("target_temp", target_temp_f);
}

void nvs_save_correction_count() {
    prefs.putUInt("corrections", hvac_correction_count);
}

// The learned HVAC effect is kept per mode: heating and cooling have
// opposite signs and different strengths.
const char* nvs_effect_key(bool heat) {
    return heat ? "hvac_effect" : "cool_effect";
}

const char* nvs_effect_variance_key(bool heat) {
    return heat ? "hvac_effect_var" : "cool_effect_var";
}

void nvs_save_hvac_effect(bool heat, float mean, float variance) {
    prefs.putFloat(nvs_effect_key(heat), mean);
    prefs.putFloat(nvs_effect_variance_key(heat), variance);
}

void nvs_clear_hvac_effect(bool heat) {
    prefs.remove(nvs_effect_key(heat));
    prefs.remove(nvs_effect_variance_key(heat));
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
hvac_state_estimator hvac_estimator;

float hysteresis = 0.75;
float last_read_temp_f = 0.0;

static uint32_t last_temperature_request_tick;
static uint32_t last_temperature_read_tick;


static boolean message_send_queued = false;
static boolean heat_mode = true;

static boolean should_redraw_display = false;

// Set once a long press has been acted on, until both buttons are released,
// so one gesture can't trigger a second action.
static boolean long_press_consumed = false;

// Restart the estimator for the current mode from what flash holds for it.
void start_hvac_estimator() {
    hvac_estimator.begin(heat_mode, machine_on, millis());
    const char* key = nvs_effect_key(heat_mode);
    const char* variance_key = nvs_effect_variance_key(heat_mode);
    if (prefs.isKey(key) && prefs.isKey(variance_key)) {
        hvac_estimator.restore_effect(prefs.getFloat(key), prefs.getFloat(variance_key));
    }
}

// Long press, bottom button: forget what the estimator has learned for the
// current mode, in flash and in memory, and start again from the default
// prior.
void reset_hvac_estimator() {
    nvs_clear_hvac_effect(heat_mode);
    start_hvac_estimator();
    Serial.println("HVAC estimator reset by long press");
    display.writeQuickMessage("estimator reset");
}

// Long press, both buttons: switch between heating and cooling. machine_on
// is kept, since the unit's power state doesn't change with its mode.
void toggle_heat_mode() {
    heat_mode = !heat_mode;
    prefs.putBool("heat_mode", heat_mode);
    start_hvac_estimator();
    Serial.print("Mode set by long press to ");
    Serial.println(heat_mode ? "heat" : "cool");
    display.writeQuickMessage(heat_mode ? "mode: HEAT" : "mode: COOL");
}

// Long press, top button: the unit is really in the other state (e.g. it was
// unplugged). Flips machine_on without sending a press.
void toggle_machine_on_manually() {
    // While a press is queued, machine_on already reflects the undelivered
    // press, so flipping it here would be ambiguous.
    if (message_send_queued) {
        Serial.println("Manual toggle ignored: a press is being delivered");
        display.writeQuickMessage("busy, try again");
        return;
    }

    machine_on = !machine_on;
    nvs_save_machine_on();
    hvac_estimator.set_known_state(machine_on, millis());
    Serial.print("machine_on set by long press to ");
    Serial.println(machine_on);
    display.writeQuickMessage(machine_on ? "state set to ON" : "state set to OFF");
}
    
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
        heat_mode = prefs.getBool("heat_mode", true);
        Serial.print("Restored from NVS: mode=");
        Serial.print(heat_mode ? "heat" : "cool");
        Serial.print(" machine_on=");
        Serial.print(machine_on);
        Serial.print(" target_temp=");
        Serial.println(target_temp_f);
    }
    thermos.set_target_temperature(target_temp_f);

    hvac_correction_count = prefs.getUInt("corrections", 0);
    start_hvac_estimator();
    Serial.print("HVAC estimator: corrections=");
    Serial.print(hvac_correction_count);
    Serial.print(" effect=");
    Serial.print(hvac_estimator.effect(), 3);
    Serial.print(" +/- ");
    Serial.println(hvac_estimator.effect_stddev(), 3);
    
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

    // Bottom button lowers the target, top button raises it. Holding both
    // switches between heating and cooling. The buttons are active low.
    button1.attachLongPressStart([]() {
        if (long_press_consumed) {
            return;
        }
        long_press_consumed = true;
        if (digitalRead(PIN_BUTTON_2) == LOW) {
            toggle_heat_mode();
        } else {
            reset_hvac_estimator();
        }
    });
    button2.attachLongPressStart([]() {
        if (long_press_consumed) {
            return;
        }
        long_press_consumed = true;
        if (digitalRead(PIN_BUTTON_1) == LOW) {
            toggle_heat_mode();
        } else {
            toggle_machine_on_manually();
        }
    });

 //   ledcAttachPin(PIN_LCD_BL, 0);
 //   ledcWrite(0, 10); /* Screen brightness can be modified by adjusting this parameter. (0-255) */

    display.initialize();
    display.writeQuickMessage("booting...");
    

    bluetoothManager.initialize();
    last_temperature_read_tick = millis();
    last_temperature_request_tick = millis();
    thermos.set_target_temperature(target_temp_f);
    thermos.set_hysteresis(hysteresis);

    Serial.println("Thermostat initialized");
}

void queue_message_if_needed(float temperature) {
    if (message_send_queued) {
        return;
    }


    bool turn_on  = heat_mode ? thermos.is_under(temperature) : thermos.is_over(temperature);
    bool turn_off = heat_mode ? thermos.is_over(temperature)  : thermos.is_under(temperature);

    if ((turn_on && !machine_on) || (turn_off && machine_on)) {
        machine_on = !machine_on;
        message_send_queued = true;
    }
}

// If the estimator is confident the unit isn't in the state we think it is
// (a press that didn't take, or someone toggled it by hand), adopt its
// estimate. queue_message_if_needed() then presses again if the temperature
// still calls for it.
void correct_machine_on_if_needed() {
    // While a press is queued, machine_on already reflects the undelivered
    // press, so it can't be compared against the estimate.
    if (message_send_queued) {
        return;
    }

    uint32_t now = millis();
    if (!hvac_estimator.should_correct(machine_on, now)) {
        return;
    }

    machine_on = !machine_on;
    nvs_save_machine_on();
    ++hvac_correction_count;
    nvs_save_correction_count();
    hvac_estimator.note_correction(now);

    Serial.print("HVAC estimator disagrees, machine_on corrected to ");
    Serial.print(machine_on);
    Serial.print(", corrections=");
    Serial.println(hvac_correction_count);
}

void log_hvac_estimate() {
    Serial.printf("HVAC estimate: p_on=%.3f p_on_nosettle=%.3f drift=%.3f effect=%.3f+/-%.3f calibrated=%d settling=%d\n",
                  hvac_estimator.p_on(),
                  hvac_estimator.p_on_ignoring_settle(),
                  hvac_estimator.drift(),
                  hvac_estimator.effect(),
                  hvac_estimator.effect_stddev(),
                  hvac_estimator.is_calibrated(),
                  hvac_estimator.is_settling(millis()));
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

    // Avoid timer issues after long uptimes
    if (millis() > REBOOT_INTERVAL_MS) {
        Serial.println("24h uptime reached, rebooting...");
        Serial.flush();
        esp_restart();
    }

    button1.tick();
    button2.tick();
    if (long_press_consumed && digitalRead(PIN_BUTTON_1) == HIGH &&
        digitalRead(PIN_BUTTON_2) == HIGH) {
        long_press_consumed = false;
    }

    // A queued press is delivered over a connection made just for it, so
    // nothing is held open between presses. Failed deliveries are retried
    // with backoff. deliverPress() blocks for several seconds.
    if (message_send_queued && bluetoothManager.beginConnectionRetryIfDue()) {
        if (bluetoothManager.deliverPress()) {
            message_send_queued = false;
            bluetoothManager.noteConnectionSuccess();
            nvs_save_machine_on();
            hvac_estimator.note_press(machine_on, millis());
        } else {
            bluetoothManager.noteConnectionFailure();
            Serial.print("Press not delivered, consecutive failures=");
            Serial.println(bluetoothManager.consecutiveConnectionFailures());
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

        if (hvac_estimator.add_measurement(tempF, millis())) {
            log_hvac_estimate();

            float effect_mean, effect_variance;
            if (hvac_estimator.take_effect_to_persist(millis(), effect_mean, effect_variance)) {
                nvs_save_hvac_effect(heat_mode, effect_mean, effect_variance);
                Serial.printf("Saved HVAC effect for %s: %.3f (variance %.4f)\n",
                              heat_mode ? "heat" : "cool", effect_mean, effect_variance);
            }
        }

        correct_machine_on_if_needed();
        queue_message_if_needed(tempF);

        should_redraw_display = true;

        last_temperature_read_tick = millis();
    }

    if (should_redraw_display) {
        float p_on = hvac_estimator.p_on();
        display.drawDisplay(thermos.get_target_temperature(), last_read_temp_f,
                            machine_on, machine_on ? p_on : 1.0f - p_on,
                            hvac_correction_count, heat_mode);
        should_redraw_display = false;
    }

}


