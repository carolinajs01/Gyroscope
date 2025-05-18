```markdown
# GyroTDE

Librería Arduino para el M5StickC Plus 2 (ESP32) con IMU MPU-6886 y pantalla Nextion NX4832T035_011.  
Permite calibrar el acelerómetro, filtrar la señal (EMA), integrar para obtener velocidad, mitigar drift y visualizar los datos en Nextion vía UART.

---

## 📂 Estructura del repositorio

```

/
├── libraries/
│   └── GyroTDE/
│       ├── GyroTDE.h
│       ├── GyroTDE.cpp
├── examples/
│   └── Demo\_GyroTDE/
│       └── Demo\_GyroTDE.ino
└── README.md

````

---

## 🔧 Requisitos

- **Hardware**  
  - M5StickC Plus 2 (ESP32)  
  - Pantalla Nextion NX4832T035_011  
- **Software**  
  - Arduino IDE ≥ 1.8  
  - Nextion Editor ≥ v0.60  
  - Librería `I2C_MPU6886` (gestor de librerías de Arduino)

---

## ⚙️ Instalación

1. Clona este repositorio:  
   ```bash
   git clone https://github.com/tu_usuario/GyroTDE.git
````

2. Copia la carpeta `GyroTDE` dentro de tu carpeta de librerías de Arduino (`.../Documents/Arduino/libraries/`).
3. Abre el ejemplo `Demo_GyroTDE.ino` desde **Archivo → Ejemplos → GyroTDE → Demo\_GyroTDE**.
4. Asegúrate de tener instalada la librería **I2C\_MPU6886**.

---

## 🔌 Conexiones

| M5StickC Plus 2 | Nextion NX4832T035\_011 |
| --------------- | ----------------------- |
| 5 V (VBUS)      | VCC                     |
| GND             | GND                     |
| GPIO 0 (TX2)    | RX                      |
| GPIO 26 (RX2)   | TX                      |

---

## 🚀 Uso básico

```cpp
#include <Wire.h>
#include "GyroTDE.h"

// Pines Nextion
#define NEX_RX_PIN 26
#define NEX_TX_PIN 0

// Instancia de GyroTDE
GyroTDE gyro(nex, NEX_RX_PIN, NEX_TX_PIN);

void setup() {
  // Inicializa IMU, calibración y Nextion
  if (!gyro.begin()) {
    Serial.println("Error al inicializar GyroTDE");
    while (1) delay(100);
  }
}

void loop() {
  // Actualiza aceleración, velocidad y envía datos a Nextion
  gyro.update();
  delay(50); // ~20 Hz de refresco
}
```

### Parámetros configurables

```cpp
// GyroTDE(serial, rxPin, txPin, calSamples, alpha, deadband, damping)
GyroTDE gyro(nex, 26, 0,
             200,    // muestras para calibración
             0.9f,   // coeficiente EMA
             0.05f,  // umbral dead-band (m/s²)
             0.9995f // factor damping
);
```

---

## 📖 Documentación y ejemplos

* Informe técnico detallado: `report/Informe_Tecnico.md`
* Ejemplo de uso completo: `examples/Demo_GyroTDE/Demo_GyroTDE.ino`
* Diagrama de conexionado: `wiring/conexion_diagrama.png`

---



> **Autor:** Carolina Jiménez Santano — **Fecha:** mayo 2025
> Proyecto desarrollado para M5StickC Plus 2 con MPU-6886 y Nextion NX4832T035\_011.

```
```
