#include <Wire.h>
#include "I2C_MPU6886.h"

// Pines Nextion en M5StickC Plus 2
#define NEX_RX_PIN 26   // Nextion TX → GPIO26
#define NEX_TX_PIN 0    // Nextion RX ← GPIO0

HardwareSerial nex(2);  // UART2 para Nextion
I2C_MPU6886    imu;

// Offset de acelerómetro (en g)
float off_ax_g = 0.0;
const int CAL_SAMPLES = 200;

// Variables de velocidad (m/s)
float velX = 0.0;

// Filtro EMA
float aPrev = 0.0;        // última aceleración filtrada (m/s²)
const float alpha = 0.9;  // coeficiente EMA

// Dead-band y damping
const float THRESH = 0.05;    // m/s² umbral para considerar cero
const float DAMP   = 0.9995; // factor de reducción de drift

unsigned long lastTime;

// Envía comando Nextion terminado en 0xFF 0xFF 0xFF
void sendCmd(const char* cmd) {
  nex.print(cmd);
  nex.write(0xFF);
  nex.write(0xFF);
  nex.write(0xFF);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Inicia I2C
  Wire.begin();

  // Inicializa IMU integrado
  if (imu.begin() != 0) {
    Serial.println("ERROR: MPU6886 NO DETECTADO");
    while (1) delay(100);
  }
  Serial.println("IMU OK, calibrando acelerómetro...");

  // Calibración de offset en g con el dispositivo en reposo
  float sum = 0.0;
  for (int i = 0; i < CAL_SAMPLES; i++) {
    float ax, ay, az;
    imu.getAccel(&ax, &ay, &az);  // lectura en g
    sum += ax;
    delay(5);
  }
  off_ax_g = sum / CAL_SAMPLES;
  Serial.print("Offset accel (g): ");
  Serial.println(off_ax_g, 4);

  // Inicializa UART2 para Nextion
  nex.begin(9600, SERIAL_8N1, NEX_RX_PIN, NEX_TX_PIN);
  delay(300);  // espera arranque de Nextion

  // Fuerza la página 0
  sendCmd("page 0");
  delay(200);

  lastTime = micros();
}

void loop() {
  // 1) Lectura de aceleración en g y compensación de offset
  float ax_g, ay_g, az_g;
  imu.getAccel(&ax_g, &ay_g, &az_g);
  float lin_g = ax_g - off_ax_g;

  // 2) Convertir a m/s²
  const float G = 9.80665;
  float ax = lin_g * G;

  // 3) Filtrado EMA
  float aFilt = alpha * aPrev + (1.0 - alpha) * ax;
  aPrev = aFilt;

  // 4) Dead-band: si muy pequeño, considerar cero y reset de velocidad
  if (fabs(aFilt) < THRESH) {
    aFilt = 0;
    velX = 0;
  }

  // 5) Cálculo de dt
  unsigned long now = micros();
  float dt = (now - lastTime) * 1e-6;
  lastTime = now;

  // 6) Integración para obtener velocidad
  velX += aFilt * dt;

  // 7) Damping para reducir drift
  velX *= DAMP;

  // 8) Enviar velocidad a Nextion en v0
  char buf[64];
  sprintf(buf, "v0.txt=\"%.2f\"", velX);
  sendCmd(buf);

  // (Opcional) Enviar aceleración filtrada a t0
  sprintf(buf, "t0.txt=\"%.2f\"", aFilt);
  sendCmd(buf);

  delay(50);
}
