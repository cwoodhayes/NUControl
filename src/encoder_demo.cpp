#include <Arduino.h>
#include <SPI.h>

#include "spi_encoder.hpp"

namespace {
constexpr uint16_t kEncoderReadCmd = (0b11 << 14) | 0x3FFF;
constexpr int kEncoderChipSelectPin = 10;
constexpr unsigned long kPrintIntervalMs = 100;

SPIEncoder g_encoder{kEncoderReadCmd, SPI, kEncoderChipSelectPin};
unsigned long g_lastPrintMs = 0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
  }

  g_encoder.set_glitch_filter_state(true);

  Serial.println("AS5047P SPI0 encoder demo");
  Serial.print("CS pin: ");
  Serial.println(kEncoderChipSelectPin);
  Serial.println("raw\tradians");
}

void loop() {
  const unsigned long now = millis();
  if (now - g_lastPrintMs < kPrintIntervalMs) {
    return;
  }
  g_lastPrintMs = now;

  const uint16_t raw = g_encoder.read_raw();
  const float radians = g_encoder.read();

  Serial.print(raw);
  Serial.print('\t');
  Serial.println(radians, 6);
}
