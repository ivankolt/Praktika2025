#include <Arduino.h>
#include <HardwareSerial.h>

// Protocol: <int16 steer><int16 speed><uint32 crc32> little-endian
// LED on PC13 (built-in BluePill), active low
// Buzzer pin configurable below (active HIGH by default)

// ===== Configuration =====
#ifndef BUZZER_PIN
#define BUZZER_PIN PB0      // change to your buzzer pin
#endif
#ifndef BUZZER_ACTIVE_HIGH
#define BUZZER_ACTIVE_HIGH 1 // set to 0 if buzzer is active LOW
#endif
#ifndef BUZZER_BOOT_MS
#define BUZZER_BOOT_MS 80    // power-up beep duration
#endif
#ifndef BUZZER_CHIRP_MS
#define BUZZER_CHIRP_MS 30   // chirp duration on valid packet
#endif

static uint8_t rxBuffer[8];
static size_t rxCount = 0;

static uint32_t calculateCRC32(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xEDB88320u;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc ^ 0xFFFFFFFFu;
}

static unsigned long ledOffAtMs = 0;
static unsigned long buzzerOffAtMs = 0;

static inline void buzzerOn() {
#if BUZZER_ACTIVE_HIGH
  digitalWrite(BUZZER_PIN, HIGH);
#else
  digitalWrite(BUZZER_PIN, LOW);
#endif
}

static inline void buzzerOff() {
#if BUZZER_ACTIVE_HIGH
  digitalWrite(BUZZER_PIN, LOW);
#else
  digitalWrite(BUZZER_PIN, HIGH);
#endif
}

void setup() {
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, HIGH); // off

  pinMode(BUZZER_PIN, OUTPUT);
  buzzerOff();

  // Power-up beep
  buzzerOn();
  buzzerOffAtMs = millis() + BUZZER_BOOT_MS;

  // USART1: PA9 (TX), PA10 (RX)
  Serial.begin(115200);
}

void loop() {
  // Read bytes
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();
    if (rxCount < sizeof(rxBuffer)) {
      rxBuffer[rxCount++] = b;
    }
    if (rxCount == sizeof(rxBuffer)) {
      // Validate CRC32
      uint32_t receivedCrc = (uint32_t)rxBuffer[4] | ((uint32_t)rxBuffer[5] << 8) | ((uint32_t)rxBuffer[6] << 16) | ((uint32_t)rxBuffer[7] << 24);
      uint32_t calcCrc = calculateCRC32(rxBuffer, 4);
      if (receivedCrc == calcCrc) {
        // Blink LED for 50 ms
        digitalWrite(PC13, LOW); // on
        ledOffAtMs = millis() + 50;
        // Chirp buzzer
        buzzerOn();
        buzzerOffAtMs = millis() + BUZZER_CHIRP_MS;
      }
      rxCount = 0; // reset buffer for next packet
    }
  }

  // Turn LED off after timeout
  if (ledOffAtMs != 0 && millis() >= ledOffAtMs) {
    digitalWrite(PC13, HIGH); // off
    ledOffAtMs = 0;
  }

  // Turn buzzer off after timeout
  if (buzzerOffAtMs != 0 && millis() >= buzzerOffAtMs) {
    buzzerOff();
    buzzerOffAtMs = 0;
  }
}
