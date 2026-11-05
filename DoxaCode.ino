#include <Arduino.h>
#include <ZMPT101B.h>
#include "ACS712.h"

// --- PINS ---
const int V_PINS[3]     = {1, 2, 4}; // ADC1 pins for ZMPT101B (voltage sensors)
const int I_PINS[3]     = {5, 6, 7}; // ADC1 pins for ACS712 (current sensors)
const int RELAY_PINS[3] = {15, 16, 17}; // relay outputs

// ACTIVE LOW RELAY (change if it's the opposite)
const int RELAY_ON  = LOW;
const int RELAY_OFF = HIGH;

// --- SETTINGS ---
const float POWER_FACTOR   = 0.7;    // assumed average
const float CURRENT_CUTOFF = 0.05;   // below this (Amps) is treated as 0 (fuh the noise)

// Safety limits: values just for example, not the real ones!
const float MAX_VOLTAGE = 250.0;
const float MIN_VOLTAGE = 180.0;
const float MAX_CURRENT = 2.0;       // must be below what the ACS712 / relay / wiring can handle
const unsigned long RETRY_DELAY_MS = 10000; // wait after a fault (and after boot) before relay can turn on
const uint8_t       MAX_TRIPS     = 3;      // over-current trips before latching
const unsigned long TRIP_RESET_MS = 60000;  // run this long without a fault to clear the trip count

// Voltage: 50.0 Hz (Masry Standard bitch)
ZMPT101B voltageSensors[3] = {
  ZMPT101B(V_PINS[0], 50.0),
  ZMPT101B(V_PINS[1], 50.0),
  ZMPT101B(V_PINS[2], 50.0)
};

// Current: 3.3V Logic, 12-bit ADC (4095)
// Sensitivity 44.0 assumes a 30A ACS712 (66 mV/A) with a 10k/20k voltage divider
ACS712 currentSensors[3] = {
  ACS712(I_PINS[0], 3.3, 4095, 44.0),
  ACS712(I_PINS[1], 3.3, 4095, 44.0),
  ACS712(I_PINS[2], 3.3, 4095, 44.0)
};

// --- DATA STRUCTURE ---
struct PlugMetrics {
  float voltage;            // Volts
  float current;            // Amps
  float power;              // Watts (V * I * PF)
  float energyKWh;          // Accumulated energy (kWh)
  bool  relayOn;            // current relay state
  unsigned long lastTripMs; // when the relay last turned off (FOR SAFETY)
  const char* status;       // text shown in Serial Monitor (for displays)
  uint8_t tripCount;        // consecutive over-current trips
  bool latched;             // true = stays off until reset
  unsigned long relayOnMs;  // when the relay last turned on
};

PlugMetrics plugs[3];

unsigned long lastCycleMs = 0;
unsigned long lastPrintMs = 0;

void setRelay(int i, bool on) {
  plugs[i].relayOn = on;
  if (on) plugs[i].relayOnMs = millis();
  digitalWrite(RELAY_PINS[i], on ? RELAY_ON : RELAY_OFF);
}

void setup() {
  Serial.begin(115200);

  // 1. Relays OFF first, so no current flows while we find the zero-current level
  for (int i = 0; i < 3; i++) {
    digitalWrite(RELAY_PINS[i], RELAY_OFF);
    pinMode(RELAY_PINS[i], OUTPUT);
    plugs[i] = { 0.0, 0.0, 0.0, 0.0, false, millis(), "STARTING", 0, false, 0 };
  }

  // 2. Calibrate sensors
  for (int i = 0; i < 3; i++) {
    currentSensors[i].autoMidPoint();          // zero-current level
    voltageSensors[i].setSensitivity(500.0);   // check against a multimeter
  }

  lastCycleMs = millis();
  lastPrintMs = millis();
}

void loop() {
  unsigned long now = millis(); // <- variable for the current time
  float hoursPassed = (float)(now - lastCycleMs) / 3600000.0;
  lastCycleMs = now;

  for (int i = 0; i < 3; i++) {
    // --- READ ---
    plugs[i].voltage = voltageSensors[i].getRmsVoltage();

    float currentAmps = currentSensors[i].mA_AC(50.0, 5) / 1000.0;
    if (currentAmps < CURRENT_CUTOFF) currentAmps = 0.0;
    plugs[i].current = currentAmps;

    // --- COMPUTE ---
    plugs[i].power = plugs[i].voltage * plugs[i].current * POWER_FACTOR;
    plugs[i].energyKWh += (plugs[i].power / 1000.0) * hoursPassed;

    // --- SAFETY ---
    bool voltageFault     = (plugs[i].voltage > MAX_VOLTAGE || plugs[i].voltage < MIN_VOLTAGE);
    bool overCurrentFault = (plugs[i].current > MAX_CURRENT);

    if (plugs[i].relayOn) {
      if (voltageFault || overCurrentFault) {
        setRelay(i, false);
        plugs[i].lastTripMs = now;

        if (overCurrentFault) {
          plugs[i].tripCount++;
          if (plugs[i].tripCount >= MAX_TRIPS) {
            plugs[i].latched = true;
            plugs[i].status = "LATCHED: OVER-CURRENT";
          } else {
            plugs[i].status = "FAULT: OVER-CURRENT";
          }
        } else {
          plugs[i].status = "FAULT: VOLTAGE";
        }
      } else {
        plugs[i].status = "NORMAL";
        // stable long enough -> forget old trips
        if (now - plugs[i].relayOnMs >= TRIP_RESET_MS) plugs[i].tripCount = 0;
      }
    } else {
      if (!plugs[i].latched &&
          now - plugs[i].lastTripMs >= RETRY_DELAY_MS &&
          !voltageFault) {
        setRelay(i, true);
        plugs[i].status = "NORMAL";
      }
    }
  }


  if (now - lastPrintMs >= 1000) {
    lastPrintMs = now;
    for (int i = 0; i < 3; i++) {
      Serial.print("[Plug "); Serial.print(i + 1); Serial.print("] ");
      Serial.print(plugs[i].voltage, 1);   Serial.print(" V | ");
      Serial.print(plugs[i].current, 2);   Serial.print(" A | ");
      Serial.print(plugs[i].power, 1);     Serial.print(" W | ");
      Serial.print(plugs[i].energyKWh, 5); Serial.print(" kWh | Relay: ");
      Serial.print(plugs[i].relayOn ? "ON" : "OFF");
      Serial.print(" | "); Serial.println(plugs[i].status);
    }
    Serial.println("------------------------------------------------------------");
  }

  if (Serial.available() && Serial.read() == 'r') {
    for (int i = 0; i < 3; i++) { plugs[i].latched = false; plugs[i].tripCount = 0; }
  }

}
