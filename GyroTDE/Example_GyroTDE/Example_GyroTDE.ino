#include <Wire.h>
#include "GyroTDE.h"

// Pines Nextion en M5StickC Plus 2
#define NEX_RX_PIN 26
#define NEX_TX_PIN 0

GyroTDE gyro(nex, NEX_RX_PIN, NEX_TX_PIN);

void setup() {
  if (!gyro.begin()) {
    while (1) delay(100);  // error de inicialización
  }
}

void loop() {
  gyro.update();
  delay(50);
}
