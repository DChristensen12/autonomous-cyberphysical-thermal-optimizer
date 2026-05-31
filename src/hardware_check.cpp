// Hardware bring-up test. Flash this instead of the real firmware to confirm
// every piece is wired right before you trust the optimizer with it.
//
// It stays away from the GP, the PID, and the rest of the control code on
// purpose. It pokes the pins directly and does its own little thermistor
// conversion, so when a test misbehaves you know to go look at the wiring and
// not at the software.
//
// Flash it with:   pio run -e hardware_check -t upload
// Then open the serial monitor at 115200 and follow the menu.

#include <Arduino.h>
#include <math.h>
#include "acpto_config.h"

// Average a batch of ADC reads and turn it into Celsius, same Beta math the
// real firmware uses. Kept as its own copy here so this test does not depend
// on anything else compiling correctly.
static float read_temp_c() {
    uint32_t acc = 0;
    for (int i = 0; i < thermistor::ADC_OVERSAMPLE; ++i) {
        acc += analogRead(pins::THERMISTOR_ADC);
    }
    const float counts = (float)acc / (float)thermistor::ADC_OVERSAMPLE;
    if (counts <= 1.0f || counts >= (float)thermistor::ADC_MAX - 1.0f) {
        return -999.0f;  // the tap is pinned to a rail, wiring is off
    }
    const float frac = counts / (float)thermistor::ADC_MAX;
    const float r_therm = thermistor::SERIES_RESISTOR * (1.0f / frac - 1.0f);
    const float t0_k = thermistor::NOMINAL_TEMP_C + 273.15f;
    const float inv_t = 1.0f / t0_k
                      + (1.0f / thermistor::BETA) * logf(r_therm / thermistor::NOMINAL_RES);
    return (1.0f / inv_t) - 273.15f;
}

static float read_raw_counts() {
    uint32_t acc = 0;
    for (int i = 0; i < thermistor::ADC_OVERSAMPLE; ++i) {
        acc += analogRead(pins::THERMISTOR_ADC);
    }
    return (float)acc / (float)thermistor::ADC_OVERSAMPLE;
}

static void all_off() {
    analogWrite(pins::HEATER_PWM, 0);
    analogWrite(pins::FAN_PWM, 0);
}

static void print_menu() {
    Serial.println();
    Serial.println(F("=== ACPTO hardware check ==="));
    Serial.println(F("send one character:"));
    Serial.println(F("  l  blink the onboard LED"));
    Serial.println(F("  s  stream the sensor reading"));
    Serial.println(F("  h  heater test, watch the temp rise"));
    Serial.println(F("  f  fan test, hear it spin or watch temp drop"));
    Serial.println(F("  x  everything off"));
    Serial.println(F("============================"));
}

void setup() {
    pinMode(pins::HEATER_PWM, OUTPUT);
    pinMode(pins::FAN_PWM, OUTPUT);
    pinMode(pins::STATUS_LED, OUTPUT);
    analogWriteResolution(control::PWM_RESOLUTION_BITS);
    analogReadResolution(thermistor::ADC_BITS);
    all_off();

    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    print_menu();
}

// If you see the LED blink, the board is alive and your flash landed. The
// simplest possible confidence check.
static void test_led() {
    Serial.println(F("blinking 5 times"));
    for (int i = 0; i < 5; ++i) {
        digitalWrite(pins::STATUS_LED, HIGH);
        delay(200);
        digitalWrite(pins::STATUS_LED, LOW);
        delay(200);
    }
    Serial.println(F("done"));
}

// Prints raw ADC counts and temperature twice a second until you send another
// key. Pinch the thermistor between your fingers and the temperature should
// climb. If it drops when you warm it, the divider is wired the other way
// round and the math needs flipping.
static void test_sensor() {
    Serial.println(F("streaming, send any key to stop"));
    Serial.println(F("counts,temp_c"));
    while (!Serial.available()) {
        const float counts = read_raw_counts();
        const float temp = read_temp_c();
        Serial.print(counts, 1);
        Serial.print(',');
        if (temp < -900.0f) {
            Serial.println(F("PINNED, check divider wiring"));
        } else {
            Serial.println(temp, 2);
        }
        delay(500);
    }
    while (Serial.available()) Serial.read();
}

// Runs the heater at a gentle duty and prints temperature each second. You
// should see a slow climb of a degree or two over the run. That proves the
// L293D heater channel, the 9V supply, and the resistor are all wired right.
// It bails the moment the temperature crosses the safety limit, or if the
// sensor reads pinned, since a bad sensor makes the safety check meaningless.
static void test_heater() {
    const int duty = (int)(0.6f * control::PWM_MAX);
    const int seconds = 30;
    Serial.println(F("heater on at 60% for 30s"));
    Serial.println(F("t_sec,temp_c"));

    analogWrite(pins::HEATER_PWM, duty);
    for (int s = 0; s < seconds; ++s) {
        const float temp = read_temp_c();
        Serial.print(s);
        Serial.print(',');
        Serial.println(temp, 2);

        if (temp < -900.0f) {
            Serial.println(F("sensor pinned, killing heater to be safe"));
            break;
        }
        if (temp > control::SAFETY_MAX_C) {
            Serial.println(F("safety limit hit, killing heater"));
            break;
        }
        if (Serial.available()) {
            while (Serial.available()) Serial.read();
            Serial.println(F("aborted"));
            break;
        }
        delay(1000);
    }
    all_off();
    Serial.println(F("heater off"));
}

// Runs the fan at its capped duty. You should hear it spin, and if the block
// is warm from the heater test you should see the temperature start to fall.
// Proves the L293D fan channel and the fan wiring.
static void test_fan() {
    const int duty = (int)(control::FAN_MAX_DUTY * control::PWM_MAX);
    const int seconds = 15;
    Serial.println(F("fan on at capped duty for 15s"));
    Serial.println(F("t_sec,temp_c"));

    analogWrite(pins::FAN_PWM, duty);
    for (int s = 0; s < seconds; ++s) {
        Serial.print(s);
        Serial.print(',');
        Serial.println(read_temp_c(), 2);
        if (Serial.available()) {
            while (Serial.available()) Serial.read();
            Serial.println(F("aborted"));
            break;
        }
        delay(1000);
    }
    all_off();
    Serial.println(F("fan off"));
}

void loop() {
    if (!Serial.available()) return;
    const char c = Serial.read();
    switch (c) {
        case 'l': test_led();    break;
        case 's': test_sensor(); break;
        case 'h': test_heater(); break;
        case 'f': test_fan();    break;
        case 'x': all_off(); Serial.println(F("all off")); break;
        case '\n':
        case '\r': break;
        default:
            Serial.print(F("unknown command: "));
            Serial.println(c);
            print_menu();
            break;
    }
}