// GyroTDE.cpp

#include "GyroTDE.h"

GyroTDE::GyroTDE(HardwareSerial& serialPort, uint8_t nexRxPin, uint8_t nexTxPin,
                 int calSamples, float alpha, float deadband, float damping)
  : _nex(serialPort)
  , _nexRxPin(nexRxPin)
  , _nexTxPin(nexTxPin)
  , _calSamples(calSamples)
  , _alpha(alpha)
  , _deadband(deadband)
  , _damping(damping)
{}

bool GyroTDE::begin(unsigned long baudrate) {
  // Inicia serie del host para debug
  Serial.begin(115200);
  while (!Serial) {}

  // I2C
  Wire.begin();

  // IMU
  if (_imu.begin() != 0) {
    Serial.println("ERROR: MPU6886 no detectado");
    return false;
  }
  Serial.println("IMU OK, calibrando acelerómetro...");

  // Calibración offset (eje X)
  float sum = 0.0f;
  for (int i = 0; i < _calSamples; i++) {
    float ax, ay, az;
    _imu.getAccel(&ax, &ay, &az);
    sum += ax;
    delay(5);
  }
  _offAxG = sum / _calSamples;
  Serial.print("Offset accel (g): ");
  Serial.println(_offAxG, 4);

  // Nextion
  _nex.begin(baudrate, SERIAL_8N1, _nexRxPin, _nexTxPin);
  delay(300);
  sendCmd("page 0");
  delay(200);

  _lastTime = micros();
  return true;
}

void GyroTDE::update() {
  // 1) Leer aceleración y compensar offset
  float ax_g, ay_g, az_g;
  _imu.getAccel(&ax_g, &ay_g, &az_g);
  float linG = ax_g - _offAxG;

  // 2) pasar a m/s²
  constexpr float G = 9.80665f;
  float ax = linG * G;

  // 3) EMA
  float aFilt = _alpha * _aPrev + (1.0f - _alpha) * ax;
  _aPrev = aFilt;

  // 4) dead-band
  if (fabs(aFilt) < _deadband) {
    aFilt = 0.0f;
    _velX = 0.0f;
  }

  // 5) dt
  unsigned long now = micros();
  float dt = (now - _lastTime) * 1e-6f;
  _lastTime = now;

  // 6) integrar velocidad
  _velX += aFilt * dt;

  // 7) damping
  _velX *= _damping;

  // 8) enviar a Nextion
  char buf[64];
  sprintf(buf, "v0.txt=\"%.2f\"", _velX);
  sendCmd(buf);

  // envío aceleración filtrada (opcional)
  sprintf(buf, "t0.txt=\"%.2f\"", aFilt);
  sendCmd(buf);
}

float GyroTDE::getVelocity() const {
  return _velX;
}

float GyroTDE::getFilteredAccel() const {
  return _aPrev;
}

void GyroTDE::sendCmd(const char* cmd) {
  _nex.print(cmd);
  _nex.write(0xFF);
  _nex.write(0xFF);
  _nex.write(0xFF);
}
