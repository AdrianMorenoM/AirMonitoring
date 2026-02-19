# 🌍 Sistema de Telemetría Ambiental con LoRa y FreeRTOS

Sistema de monitoreo de calidad del aire basado en 2 nodos **Heltec LoRa V3 (ESP32-S3 + SX1262)**.

- **TX (Transmisor):** Captura variables ambientales.
- **RX (Receptor):** Recibe por LoRa y publica en MQTT hacia internet.

Arquitectura optimizada con **doble núcleo del ESP32-S3 + FreeRTOS**, separando procesos críticos de radio y procesos de red. Resultado: comunicación confiable sin interferencias entre tareas.

---

## 🎯 Objetivo

Desarrollar un sistema de telemetría ambiental que garantice:

- Comunicación LoRa confiable.
- Confirmación de entrega mediante ACK.
- Publicación en la nube vía MQTT.
- Arquitectura concurrente robusta (FreeRTOS + multicore).

---

# ✨ Características

- ✔ Protocolo **Stop-and-Wait** con hasta 3 retransmisiones.
- ✔ Multitarea real con afinidad de núcleo.
- ✔ Sensores:
  - MQ-2
  - MQ-135
  - GP2Y1010AU0F
- ✔ Visualización en pantalla OLED.
- ✔ Publicación en broker MQTT (HiveMQ).
- ✔ Logging detallado por puerto serie.

---

# 🛠️ Hardware

## Dispositivos

- 2 × Heltec LoRa V3 (ESP32-S3 + SX1262)

## Sensores (solo TX)

| Sensor | Variable |
|--------|----------|
| MQ-2 | Gases inflamables / humo |
| MQ-135 | CO₂, NH₃, VOC |
| GP2Y1010AU0F | Partículas (polvo) |

## Complementos

- Protoboard
- Cables Dupont
- Fuente 5V para sensores

---

# 🧠 Arquitectura del Software

Ambos nodos ejecutan **FreeRTOS** con tareas fijadas a núcleos específicos.

---

## 🔵 Transmisor (TX)

| Núcleo | Tarea | Prioridad | Función |
|--------|--------|-----------|----------|
| Core 1 | sensorTask | 1 | Lectura sensores + OLED |
| Core 0 | loraTask | 2 | Envío LoRa + ACK + reintentos |

### Operación

- Lectura cada 2 s.
- Ventana de congestión = 1 paquete en vuelo.
- Control de RTT.
- Manejo automático de retransmisiones.

---

## 🟢 Receptor (RX)

| Núcleo | Tarea | Prioridad | Función |
|--------|--------|-----------|----------|
| Core 0 | loraTask | 2 | Recepción + envío ACK |
| Core 1 | mqttTask | 1 | WiFi + publicación MQTT |

---

# 📡 Protocolo Stop-and-Wait

LoRa es **half-duplex**, por lo que se implementa el siguiente flujo:

1. TX envía paquete con número de secuencia.
2. RX responde con `{"ack":seq}`.
3. TX espera 2 segundos.
4. Si no recibe ACK → retransmite (máx 3 intentos).
5. RX detecta duplicados pero siempre envía ACK.

## Beneficios

- Entrega garantizada al menos una vez.
- Control de pérdidas.
- Evita tormentas de retransmisión.

---

# 🔄 Flujo de Datos

## Transmisor

### sensorTask
- Lee sensores.
- Protege datos con mutex.
- Marca datos como válidos.

### loraTask
- Copia segura de datos.
- Genera JSON.
- Encola paquete.
- Envía si radio libre.
- Controla timeout y retransmisión.
- Procesa ACK y calcula RTT.

---

## Receptor

### Callback OnRxDone
- Parsea JSON.
- Valida ID.
- Detecta pérdida por secuencia.
- Actualiza RSSI/SNR.
- Activa flags:
  - mqttFlag
  - ackPending

### loraTask
- Envía ACK cuando radio esté libre.
- Reporta estadísticas cada 5 s.

### mqttTask
- Mantiene conexión WiFi.
- Publica cuando mqttFlag está activo.

---

# 🔐 Control de Concurrencia

Uso de `xSemaphore` (mutex) para proteger estructura `sensorData`.

Ventajas:
- Evita condiciones de carrera.
- Garantiza coherencia entre tareas.

---

# 🧩 Ejemplo de Logs

## TX

[TX][SEND] ✉️ Seq:5 | InFlight:1 | Q:0
[TX][ACK] ✅ Seq:5 RTT:132ms Try:0 | RSSI:-57
[TX][RETRY] 🔁 Seq:6 Try:1/3
[TX][DROP] ❌ Seq:7 agotado 3 reintentos

## RX

[RX] RAW: {"seq":5,"id":"TX_AMBIENTAL_01","mq2":210,"mq135":345,"dust":120} | RSSI:-57 SNR:8
[RX] SEQ:5 MQ2:210 MQ135:345 DUST:120
[RX] Enviando ACK: {"ack":5}
[MQTT] Publish AirMonitoring OK


---

# 🚀 Cómo Probar

## 1. Cargar Firmware

- TX → `TX_Ambiental.ino`
- RX → `RX_Ambiental.ino`

Monitor serie: **115200 baudios**

---

## 2. Conexiones (TX)

| Sensor | Pin |
|--------|-----|
| MQ-2 | 1 |
| MQ-135 | 2 |
| GP2Y1010 LED | 7 |
| GP2Y1010 Vo | 6 |

Alimentación: 5V y GND.

---

## 3. Verificación

- RX debe mostrar paquetes recibidos.
- Suscribirse al tópico `AirMonitoring` con un cliente MQTT.
- Broker: HiveMQ.

---

# ⚙️ Parámetros Críticos

Ambos dispositivos deben coincidir en:

- RF_FREQUENCY
- LORA_SPREADING_FACTOR
- LORA_BANDWIDTH
- LORA_CODINGRATE

Configurar credenciales WiFi en RX (`wifiList`).

---

# 📊 Arquitectura General

[Sensores] → [TX Heltec] → LoRa → [RX Heltec] → WiFi → MQTT → Nube

---

# 🏁 Estado

- Comunicación LoRa estable
- ACK funcional
- MQTT operativo
- Arquitectura multitarea consolidada
