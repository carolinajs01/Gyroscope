// GyroTDE.h

#ifndef GYROTDE_H
#define GYROTDE_H

#include <Wire.h>
#include "I2C_MPU6886.h"
#include <HardwareSerial.h>

class GyroTDE {
public:
  // Constructor: recibe UART (2 en M5StickC Plus 2) y pines RX/TX de Nextion
  GyroTDE(HardwareSerial& serialPort, uint8_t nexRxPin, uint8_t nexTxPin,
          int calSamples = 200, float alpha = 0.9f,
          float deadband = 0.05f, float damping = 0.9995f);

  // Inicializa IMU y Nextion; realiza calibración
  bool begin(unsigned long baudrate = 9600);

  // Llamar en loop() para actualizar medición y enviarla al display
  void update();

  // Opcional: acceder a velocidad y aceleración filtrada
  float getVelocity() const;
  float getFilteredAccel() const;

private:
  HardwareSerial& _nex;
  uint8_t        _nexRxPin, _nexTxPin;
  I2C_MPU6886    _imu;

  // Parámetros
  int    _calSamples;
  float  _alpha;
  float  _deadband;
  float  _damping;

  // Estado interno
  float  _offAxG    = 0.0f;   // offset acelerómetro en g
  float  _aPrev     = 0.0f;   // última aceleración filtrada (m/s²)
  float  _velX      = 0.0f;   // velocidad (m/s)
  unsigned long _lastTime = 0; // micros()

  // Envía comando Nextion (añade 0xFF 0xFF 0xFF)
  void sendCmd(const char* cmd);
};

#endif // GYROTDE_H
