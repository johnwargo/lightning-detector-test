/*******************************************************************
* Beetle ESP32 - C3 Lightning Detector Test
* By John M. Wargo
* https://johnwargo.com
********************************************************************/
// References:
// https://wiki.dfrobot.com/Gravity%3A%20Lightning%20Sensor%20SKU%3A%20SEN0290
// https://github.com/DFRobot/DFRobot_AS3935
// https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/intr_alloc.html
// https://lastminuteengineers.com/handling-esp32-gpio-interrupts-tutorial/

#include <Wire.h>
#include "DFRobot_AS3935_I2C.h"

// #if defined(ESP32) || defined(ESP8266)
// #define IRQ_PIN 0
// #else
// #define IRQ_PIN 2
// #endif
#define IRQ_PIN 27

// Antenna tuning capcitance (must be integer multiple of 8, 8 - 120 pf)
#define AS3935_CAPACITANCE 96

// Indoor/outdoor mode selection
#define AS3935_INDOORS 0
#define AS3935_OUTDOORS 1
#define AS3935_MODE AS3935_INDOORS

// Enable/disable disturber detection
#define AS3935_DIST_DIS 0
#define AS3935_DIST_EN 1
#define AS3935_DIST AS3935_DIST_EN

// I2C address
#define AS3935_I2C_ADDR AS3935_ADD3

DFRobot_AS3935_I2C lightning0((uint8_t)IRQ_PIN, (uint8_t)AS3935_I2C_ADDR);

// internal variables and constants
// #define lightningInterval 300000  // 5 minutes before clearing the display
#define lightningInterval 60000  // 1 minute before clearing the display

String dashes = "=============================";

volatile bool IRQ_EVENT = false;
unsigned long lastLightning;
int counter = 0;
int counterLimit = 50;

// https://lastminuteengineers.com/handling-esp32-gpio-interrupts-tutorial/
void IRAM_ATTR isr() {
  IRQ_EVENT = true;
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println();
  Serial.println(dashes);
  Serial.println("| Lightning Detector Tester |");
  Serial.println("| By John M. Wargo          |");
  Serial.println(dashes);

  Serial.println("Initializing Lightning Sensor");
  while (lightning0.begin() != 0) {
    Serial.print(".");
  }
  if (lightning0.defInit() != 0) {
    Serial.println("Reset failed");
  }

#if defined(ESP32) || defined(ESP8266)
  attachInterrupt(digitalPinToInterrupt(IRQ_PIN), isr, RISING);
#else
  attachInterrupt(/*Interrupt No*/ 0, isr, RISING);
#endif

  // Configure sensor
  lightning0.manualCal(AS3935_CAPACITANCE, AS3935_MODE, AS3935_DIST);
  // Enable interrupt (connect IRQ pin IRQ_PIN: 2, default)
  // Initialize this to zero meaning no lightning
  lastLightning = 0;
}

void loop() {
  if (IRQ_EVENT) {
    IRQ_EVENT = false;
    logLightningEvent();
  }
  checkTimer();
  // Keep updating the display
  // Serial.print(".");
  // counter += 1;
  // if (counter > counterLimit) {
  //   counter = 0;
  //   Serial.println();
  // }
  // do a little wait here so the ESP32 has time to do housekeeping chores
  // https://randomerrors.dev/posts/2023/esp32-watchdog-got-triggered/
  delay(100);
}

void logLightningEvent() {
  uint8_t intSrc;
  uint8_t lightningDistKm;
  uint32_t lightningEnergyVal;

  Serial.println("\nTriggered");
  intSrc = lightning0.getInterruptSrc();  // Get interrupt source
  Serial.print("Interrupt source: ");
  Serial.println(String(intSrc));
  switch (intSrc) {
    case 1:
      lastLightning = millis();  // set the last lightning time to now
      lightningDistKm = lightning0.getLightningDistKm();
      Serial.println("Lightning: " + String(lightningDistKm) + "km");
      lightningEnergyVal = lightning0.getStrikeEnergyRaw();
      Serial.println("Intensity: " + String(lightningEnergyVal));
      break;
    case 2:
      Serial.println("Disturbance detected");
      break;
    case 3:
      Serial.println("Noise level too high!");
      break;
    default:
      Serial.println("<UNKNOWN>");
      break;
  }
}

void checkTimer() {
  // have we had lightning in the last lightningInterval milliseconds?
  if (lastLightning > 0) {
    // yes, how long has it been?
    if ((millis() - lastLightning) > lightningInterval) {
      // counter expired, reset the lightning timer to zero
      lastLightning = 0;
      Serial.println("\nResetting lightning timer");
    }
  }
}
