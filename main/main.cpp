#include <Preferences.h>
#include <driver/ledc.h>
#include "driver/timer.h"
#include "Arduino.h"

/********************************************************************
          BASIC SETTINGS
********************************************************************/
#define TIMER_GROUP TIMER_GROUP_0
#define TIMER_IDX   TIMER_0
// Stepper motor
#define NOFMICROSTEPS 16      // The number of microsteps per step
#define NOFSTEPSPER360 200    // The number of steps per revolution
#define MAXRPM 240            // Maximum RPM (rotations per minute)
#define INVERTDIRECTION true  // If the carriage moves in the opposite direction, change to 'false'

// Leadscrew
#define MMPER360 2  // Leadscrew pitch, mm per revolution

// ESP32 pins
#define STEP_PIN GPIO_NUM_17       // STEP pin (A4988)
#define DIRECTION_PIN GPIO_NUM_18  // DIR pin (A4988)
#define ENABLE_PIN GPIO_NUM_16     // EN pin (A4988)
#define ENDSTOP_PIN1 GPIO_NUM_34   // Endstop pin (forward movement)
#define ENDSTOP_PIN2 GPIO_NUM_35   // Endstop pin (backward movement)
//#define BUZZER_PIN GPIO_NUM_42     // Buzzer pin

/********************************************************************/

float stepsPERmm = NOFSTEPSPER360 / MMPER360;
float coef;
uint8_t endStopPin;
volatile uint32_t ustepCounter;
uint32_t ustepCounterLimit;

bool IRAM_ATTR onTimer(void* arg);
void initTimer(uint16_t countsPERustepActual);
bool checkEndstop(int8_t pumpingDirection);
void pumpSingly(int8_t pumpingDirection, float flowrate, float volume);
uint8_t pump(int8_t pumpingDirection, float flowrate, float volume);
void infuseVolume();
void refillVolume();
float getFloatInput(const char* prompt);
uint32_t getIntInput(const char* prompt);

extern "C" {
    void app_main(void);
}

bool IRAM_ATTR onTimer(void* arg) {
    gpio_set_level(STEP_PIN, 1);
    ustepCounter++;
    gpio_set_level(STEP_PIN, 0);

    if (ustepCounter >= ustepCounterLimit) {
        Serial.println("Timer: Counter limit reached, pausing timer");
        timer_pause(TIMER_GROUP_0, TIMER_0);
    }
    if (gpio_get_level((gpio_num_t)endStopPin) == 0) {
        Serial.println("Timer: Endstop reached, pausing timer");
        timer_pause(TIMER_GROUP_0, TIMER_0);
    }

    return true;
}

void initTimer(uint16_t countsPERustepActual) {
    timer_config_t config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = 80  // 1 us per tick (80 MHz / 80)
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, countsPERustepActual - 1);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, onTimer, NULL, 0);
    timer_start(TIMER_GROUP_0, TIMER_0);

    Serial.println("Timer initialized and started");
}

void app_main(void) {
    Serial.begin(115200);

    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIRECTION_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(ENDSTOP_PIN1, INPUT_PULLUP);
    pinMode(ENDSTOP_PIN2, INPUT_PULLUP);
    //pinMode(BUZZER_PIN, OUTPUT);

    digitalWrite(ENABLE_PIN, HIGH);

    float diameter = getFloatInput("Enter Syringe Diameter (in mm): ");
    float length = getFloatInput("Enter Syringe Length (in mm): ");

    coef = (length * stepsPERmm) * 1e-6; // Adjusted for mL
    Serial.println("Syringe setup complete. Choose operation: (1) Infuse (2) Refill");

    while (true) {
        if (Serial.available()) {
            int operation = getIntInput("");
            if (operation == 1) {
                infuseVolume();
            } else if (operation == 2) {
                refillVolume();
            } else {
                Serial.println("Invalid option. Choose operation: (1) Infuse (2) Refill");
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Yield to other tasks
    }
}

float getFloatInput(const char* prompt) {
    Serial.print(prompt);
    while (!Serial.available()) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    float value = Serial.parseFloat();
    Serial.readStringUntil('\n');  // Clear the buffer
    return value;
}

uint32_t getIntInput(const char* prompt) {
    if (strlen(prompt) > 0) {
        Serial.print(prompt);
    }
    while (!Serial.available()) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    uint32_t value = Serial.parseInt();
    Serial.readStringUntil('\n');  // Clear the buffer
    return value;
}

bool checkEndstop(int8_t pumpingDirection) {
    bool canMove = true;
    if (digitalRead(ENDSTOP_PIN1) == LOW && pumpingDirection > 0) canMove = false;
    else if (digitalRead(ENDSTOP_PIN2) == LOW && pumpingDirection < 0) canMove = false;

    if (!canMove) {
        Serial.println("\n\nEndstop Reached");
        if (pumpingDirection > 0) Serial.println("SYRINGE IS EMPTY");
        else if (pumpingDirection < 0) Serial.println("SYRINGE IS FULL");
        return false;
    }
    return true;
}

void infuseVolume() {
    float flowrate = getFloatInput("Enter flowrate (mL/min): ");
    float volume = getFloatInput("Enter volume to infuse (mL): ");
    pumpSingly(1, flowrate, volume);
}

void refillVolume() {
    float flowrate = getFloatInput("Enter flowrate (mL/min): ");
    float volume = getFloatInput("Enter volume to refill (mL): ");
    pumpSingly(-1, flowrate, volume);
}

void pumpSingly(int8_t pumpingDirection, float flowrate, float volume) {
    if (checkEndstop(pumpingDirection)) {
        pump(pumpingDirection, flowrate, volume);
        Serial.println("Pumping completed.");
    }
}

uint8_t pump(int8_t pumpingDirection, float flowrate, float volume) {
    ustepCounterLimit = (uint32_t)(coef * volume + 0.5) * NOFMICROSTEPS;
    float ustepsPERmin = coef * NOFMICROSTEPS * flowrate * 1000; // Adjusted for mL
    if (ustepsPERmin < 14.4) ustepsPERmin = 14.4;
    float multiplier = (ustepsPERmin >= 229) ? 15000000 : 937500;
    float tempFloat = multiplier / ustepsPERmin;
    uint16_t countsPERustepActual = (uint16_t)(tempFloat + 0.5);

    Serial.println("\n\n----- Pumping -----");
    if (pumpingDirection > 0) {
        endStopPin = ENDSTOP_PIN1;
        digitalWrite(DIRECTION_PIN, INVERTDIRECTION ? LOW : HIGH);
        Serial.println(" >> INFUSING >> ");
    } else {
        endStopPin = ENDSTOP_PIN2;
        digitalWrite(DIRECTION_PIN, INVERTDIRECTION ? HIGH : LOW);
        Serial.println(" << REFILLING <<");
    }

    initTimer(countsPERustepActual);
    digitalWrite(ENABLE_PIN, LOW);
    digitalWrite(STEP_PIN, LOW);

    ustepCounter = 0;
    bool isManualStop = false;
    uint32_t nextMillis = millis() + 1000;

    while (ustepCounter < ustepCounterLimit) {
        if (Serial.available()) {
            String input = Serial.readStringUntil('\n');
            input.trim();
            if (input == "stop") {
                timer_pause(TIMER_GROUP_0, TIMER_0);
                isManualStop = true;
                break;
            }
        }

        if (nextMillis <= millis()) {
            float pumpingVolume = (ustepCounter / coef) * 1e-6; // Adjusted for mL
            Serial.print("Volume: ");
            Serial.println(pumpingVolume);
            nextMillis += 1000;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Yield to other tasks
    }

    digitalWrite(ENABLE_PIN, HIGH);

    float pumpingVolume = (ustepCounter / coef) * 1e-6; // Adjusted for mL
    Serial.print("Final Volume: ");
    Serial.println(pumpingVolume);

    if (isManualStop) {
        Serial.println("MANUAL STOP");
        return 2;
    }
    if (ustepCounter >= ustepCounterLimit) {
        Serial.println("FINISHED");
        return 0;
    } else {
        Serial.println("UNEXPECTED STOP");
        return 1;
    }
}
