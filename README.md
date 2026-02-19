# 📡 Sistema de Telemetría Ambiental con LoRa y FreeRTOS

Sistema de monitoreo de calidad del aire basado en dos dispositivos **Heltec LoRa V3 (ESP32-S3)**. Un nodo transmisor (TX) recolecta datos de sensores ambientales y los envía por radio LoRa a un nodo receptor (RX), que los publica en un broker MQTT para visualización remota.

---

## ✨ Características principales

- **Protocolo Stop-and-Wait** con ACKs y reintentos automáticos sobre canal half-duplex.
- **Multitarea real con FreeRTOS**: dos tareas por dispositivo, una por núcleo del ESP32-S3.
- **Deduplicación** de paquetes mediante buffer circular de secuencias recientes.
- **Detección de pérdidas** por brechas en números de secuencia.
- **Métricas de enlace en tiempo real**: RSSI, SNR, RTT, tasa de pérdida.
- **Pantalla OLED** con estado del sistema actualizado continuamente.
- **Publicación en la nube** vía WiFi + MQTT (broker HiveMQ público).
- **Multi-red WiFi**: el receptor intenta conectarse a varias redes conocidas automáticamente.
- **Control de congestión** (TCP Tahoe simplificado) para manejo de fallos.

---

## 🛠️ Hardware necesario

| Componente | Cantidad | Notas |
|---|---|---|
| Heltec WiFi LoRa 32 V3 | 2 | ESP32-S3 + SX1262 |
| Sensor MQ-2 | 1 | Gases combustibles y humo |
| Sensor MQ-135 | 1 | CO₂, NH₃, compuestos orgánicos |
| Sensor GP2Y1010AU0F | 1 | Polvo y material particulado |
| Resistor 150 Ω + Condensador 220 µF | 1 c/u | Circuito de alimentación del GP2Y |
| Cables y protoboard | — | — |
| Fuente 5V externa | Opcional | Para sensores MQ en campo |

> ⚠️ **Importante:** Los sensores MQ requieren un período de calentamiento de al menos 24–48 horas para lecturas estables. Durante ese tiempo los valores son aproximados.

---

## 📌 Conexiones del transmisor (TX)

| Sensor | Pin del sensor | Pin ESP32 |
|---|---|---|
| MQ-2 | AOUT | GPIO 1 |
| MQ-135 | AOUT | GPIO 2 |
| GP2Y1010 | LED | GPIO 7 |
| GP2Y1010 | Vo (salida) | GPIO 6 |
| MQ-2 / MQ-135 | VCC | 5V |
| GP2Y1010 | VCC | 5V (con RC) |
| Todos | GND | GND |

---

## 🧠 Arquitectura del software

Ambos dispositivos usan FreeRTOS para ejecutar tareas en paralelo. Cada tarea está **fijada a un núcleo** (`xTaskCreatePinnedToCore`) para evitar migraciones y garantizar determinismo en la radio.

### Transmisor (TX)

```
Core 0 ──► loraTask  (prioridad 2)  → Radio LoRa: envíos, ACKs, timeouts, cola
Core 1 ──► sensorTask (prioridad 1) → Lectura de sensores + pantalla OLED
               ↕ mutex (sensorMutex)
          [sensorData compartida]
```

### Receptor (RX)

```
Core 0 ──► loraTask  (prioridad 2)  → Radio LoRa: recepción + envío de ACKs
Core 1 ──► mqttTask  (prioridad 1)  → WiFi + MQTT + pantalla OLED
              ↕ volatile flags
         [mqttFlag + mqttPayloadBuf]
```

---

## 📡 Protocolo Stop-and-Wait

LoRa es **half-duplex**: no puede transmitir y recibir al mismo tiempo. El protocolo garantiza entrega confiable con estas reglas:

```
TX                                    RX
 |                                     |
 |──── {"seq":N, "mq2":..., ...} ────►|
 |                                     |── valida, deduplicación
 |                                     |── guarda datos, activa mqttFlag
 |◄─────────── {"ack": N} ────────────|
 |── elimina de inflight               |
 |── calcula RTT                       |
 |                                     |
 |──── (si no llega ACK en 2s) ───────| × hasta 3 reintentos
 |──── (si agota reintentos) ─────────| descarta, cwnd = 1
```

### Parámetros del protocolo

| Parámetro | Valor | Descripción |
|---|---|---|
| `ACK_TIMEOUT_MS` | 2000 ms | Tiempo máximo esperando ACK |
| `MAX_RETRIES` | 3 | Reintentos antes de descartar |
| `RX_GUARD_MS` | 200 ms | Pausa post-TX para que el RX envíe su ACK |
| `cwnd` | 1 (fijo) | Stop-and-Wait puro: 1 paquete en vuelo |
| `SEQ_BUFFER_SIZE` | 64 | Secuencias recientes para deduplicación |
| `SAMPLE_INTERVAL_MS` | 2000 ms | Frecuencia de muestreo y envío |

---

## 🔄 Flujo de datos detallado

### Transmisor — ciclo de `loraTask` (cada 10 ms)

1. **`Radio.IrqProcess()`** — procesa interrupciones de radio pendientes.
2. **Snapshot de sensores** — copia `sensorData` con mutex para no bloquearlo.
3. **Encolar paquete** — si los datos son frescos y pasó el intervalo de envío:
   - Incrementa `sequenceNumber`.
   - Serializa JSON: `{"seq":N,"id":"TX_AMBIENTAL_01","mq2":X,"mq135":Y,"dust":Z}`.
   - Añade a la cola circular `pendingPackets`.
4. **Enviar** — si la radio está libre, el guard expiró y `inflightCount < cwnd`:
   - Mueve el paquete de la cola a `inflight[]`.
   - Llama a `Radio.Send()`.
5. **Timeouts** — recorre `inflight[]`:
   - Si superó `ACK_TIMEOUT_MS` y tiene reintentos → retransmite.
   - Si agotó reintentos → descarta, aplica control de congestión.

### Receptor — callback `OnRxDone`

1. Copia y parsea el JSON recibido con `sscanf`.
2. Verifica que el `id` sea `TX_AMBIENTAL_01`.
3. Detecta brechas de secuencia → contabiliza paquetes perdidos.
4. Verifica duplicado en buffer circular → si es duplicado, solo envía ACK de cortesía.
5. Si es nuevo: actualiza `sensorData`, prepara payload MQTT en `mqttPayloadBuf`, activa `mqttFlag`.
6. Activa `ackPending` para que `loraTask` envíe el ACK.

---

## 📦 Formato de los mensajes

### Paquete TX → RX
```json
{
  "seq": 42,
  "id": "TX_AMBIENTAL_01",
  "mq2": 215,
  "mq135": 340,
  "dust": 118
}
```

### ACK RX → TX
```json
{ "ack": 42 }
```

### Payload publicado en MQTT
```json
{
  "seq": 42,
  "nodeId": "TX_AMBIENTAL_01",
  "mq2": 215,
  "mq135": 340,
  "dust": 118,
  "rssi": -62,
  "snr": 9
}
```

**Topic MQTT:** `AirMonitoring`  
**Broker:** `broker.hivemq.com:1883`

---

## 🧩 Sincronización entre tareas

### Transmisor — Mutex
El transmisor comparte `sensorData` entre dos tareas que corren en paralelo. Se usa un **mutex** (semáforo binario con propiedad) para acceso exclusivo:

```cpp
// sensorTask escribe
if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(100))) {
    sensorData.mq2 = mq2;
    sensorData.valid = true;
    xSemaphoreGive(sensorMutex);
}

// loraTask lee (timeout corto para no bloquear la radio)
SensorData cur;
if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10))) {
    cur = sensorData;
    xSemaphoreGive(sensorMutex);
}
```

### Receptor — Banderas `volatile`
El receptor usa variables `volatile` en lugar de mutex porque la escritura viene de un contexto quasi-ISR (callback de radio) y es atómica para tipos simples:

```cpp
// En OnRxDone (ISR):
snprintf(mqttPayloadBuf, sizeof(mqttPayloadBuf), "{...}", ...);
mqttFlag = true;   // señal para mqttTask

// En mqttTask:
if (mqttFlag && mqttClient.connected()) {
    mqttFlag = false;  // limpiar ANTES de publicar
    mqttClient.publish(MQTT_TOPIC, mqttPayloadBuf);
}
```

> ⚠️ Este patrón es seguro **únicamente** porque solo un productor escribe `mqttFlag` y solo un consumidor la lee. Para escenarios más complejos usar `FreeRTOS Queue` o `Event Groups`.

---

## 📨 Monitor Serial (115200 baudios)

### Transmisor
```
[TX][SEND]  ✉️ Seq:5 | InFlight:1 | Q:0
[TX][ACK]   ✅ Seq:5 RTT:132ms Try:0 | RSSI:-57
[TX][RETRY] 🔁 Seq:6 Try:1/3
[TX][DROP]  ❌ Seq:7 agotado 3 reintentos
[TX][CONG]  cwnd->1 ssthresh->1
🌡️ [SENSORES] MQ2:210 ⚠️ MQ135:340 🏭 DUST:118
```

### Receptor
```
[RX] RAW: {"seq":5,"id":"TX_AMBIENTAL_01","mq2":210,"mq135":340,"dust":118} | RSSI:-57 SNR:8
[RX] SEQ:5 MQ2:210 MQ135:340 DUST:118
[RX] Enviando ACK: {"ack":5}
[MQTT] Publish AirMonitoring: OK
[RX] Duplicado SEQ=5 — ACK cortesia
[RX] Brecha: esperaba 8 llego 10 (2 perdidos)
```

---

## 📊 Métricas disponibles

### En el transmisor
| Métrica | Variable | Descripción |
|---|---|---|
| Paquetes enviados | `stats.packetsSent` | Total incluyendo retransmisiones |
| Paquetes ACKed | `stats.packetsAcked` | Confirmados exitosamente |
| Retransmisiones | `stats.retransmissions` | Reenvíos por timeout |
| Timeouts totales | `stats.timeouts` | Paquetes descartados |
| RTT mín/avg/máx | `stats.minRTT`, etc. | Latencia del enlace en ms |
| Desbordamientos | `stats.queueOverflows` | Cola llena al intentar encolar |

### En el receptor
| Métrica | Variable | Descripción |
|---|---|---|
| Paquetes recibidos | `stats.packetsReceived` | Total físico (con duplicados) |
| Paquetes únicos | `stats.packetsUnique` | Sin contar duplicados |
| Duplicados | `stats.packetsDuplicated` | Retransmisiones del TX recibidas |
| Perdidos | `stats.packetsLost` | Estimado por brechas de seq |
| Tasa de pérdida | `stats.lossRate` | 0.0 = 0%, 1.0 = 100% |
| RSSI mín/máx/avg | `stats.minRSSI`, etc. | Potencia de señal recibida |
| SNR mín/máx/avg | `stats.minSNR`, etc. | Relación señal/ruido |

---

## 🚀 Cómo ejecutar el proyecto

### 1. Configurar el entorno
- Instalar **Arduino IDE** o **PlatformIO**.
- Agregar el soporte para Heltec ESP32: URL del gestor de paquetes:
  ```
  https://resource.heltec.cn/download/package_heltec_esp32_index.json
  ```
- Instalar la librería **PubSubClient** (solo para el RX).

### 2. Configurar credenciales (RX)
Editar el array `wifiList` en el código del receptor:
```cpp
WiFiCred wifiList[] = {
    {"NombreDeTuRed", "ContraseñaWiFi"},
    // Agregar más redes como respaldo...
};
```

### 3. Compilar y cargar
- Cargar `TX_Ambiental.ino` en la placa con los sensores.
- Cargar `RX_Ambiental.ino` en la otra placa.

### 4. Verificar comunicación
1. Abrir el Monitor Serial en ambos a **115200 baudios**.
2. El TX debe mostrar `[TX][ACK] ✅ Seq:N` al recibir confirmaciones.
3. El RX debe mostrar `[MQTT] Publish AirMonitoring: OK`.
4. Suscribirse al topic `AirMonitoring` con un cliente MQTT (ej. [MQTT Explorer](https://mqtt-explorer.com/)) para ver los datos en la nube.

---

## ⚙️ Parámetros configurables

### Radio LoRa (deben coincidir en TX y RX)
| Parámetro | Valor actual | Descripción |
|---|---|---|
| `RF_FREQUENCY` | 915 MHz | Frecuencia (usar 868 MHz en Europa) |
| `LORA_SPREADING_FACTOR` | 7 | SF más alto = más alcance, menos velocidad |
| `LORA_BANDWIDTH` | 0 (125 kHz) | BW más alto = más velocidad, menos alcance |
| `LORA_SYNC_WORD` | `0x2A` | "Contraseña" de red — cambiar en producción |
| `TX_OUTPUT_POWER` (TX) | 14 dBm | Potencia de transmisión de datos |
| `TX_OUTPUT_POWER` (RX) | 5 dBm | Potencia de transmisión de ACKs |

### Protocolo
| Parámetro | Valor | Dónde cambiarlo |
|---|---|---|
| `SAMPLE_INTERVAL_MS` | 2000 ms | `TX_Ambiental.ino` |
| `ACK_TIMEOUT_MS` | 2000 ms | `TX_Ambiental.ino` |
| `MAX_RETRIES` | 3 | `TX_Ambiental.ino` |
| `RX_GUARD_MS` | 200 ms | `TX_Ambiental.ino` |
| `EXPECTED_NODE_ID` | `TX_AMBIENTAL_01` | `RX_Ambiental.ino` |

---

## ⚠️ Limitaciones y puntos de mejora

### Limitaciones actuales
- **Sin cifrado:** Los datos viajan en texto plano por el aire. Cualquier dispositivo con el mismo `SYNC_WORD` puede interceptarlos.
- **Sin autenticación MQTT:** El broker público HiveMQ no tiene usuario ni contraseña.
- **`volatile` sin barreras de memoria:** El patrón funciona en el ESP32 pero no es portable a otros sistemas sin hardware memory barriers.
- **Un solo nodo:** El `EXPECTED_NODE_ID` está hardcodeado; para múltiples transmisores se necesita una tabla de nodos conocidos.

### Mejoras sugeridas

**Seguridad**
- Agregar autenticación MQTT (`mqttClient.connect(id, user, pass)`).
- Implementar cifrado liviano (AES-128) en el payload LoRa.
- Cambiar `LORA_SYNC_WORD` a un valor personalizado por instalación.

**Robustez**
- Usar `FreeRTOS Queue` (`xQueueSend` / `xQueueReceive`) en lugar de `volatile bool` para la comunicación ISR → mqttTask, lo que elimina la condición de carrera teórica.
- Agregar un **watchdog** (`esp_task_wdt`) para reiniciar automáticamente si una tarea se bloquea.
- Persistir estadísticas en NVS (flash) para no perderlas al reiniciar.

**Sensores**
- Implementar **calibración automática de R0** al arrancar (promedio de N lecturas en aire limpio).
- Convertir valores ADC a **ppm** usando las curvas del datasheet de cada sensor MQ.
- Agregar filtro de media móvil para suavizar las lecturas ruidosas del GP2Y.

**Protocolo**
- Implementar **timestamp UTC** en el payload MQTT usando NTP (`configTime`).
- Para mayor alcance, subir `LORA_SPREADING_FACTOR` a 10–12 (a costa de velocidad).
- Habilitar el modo pipeline (`cwnd > 1`) con control de flujo si se migra a full-duplex.

---

## 📁 Estructura del proyecto

```
📦 LoRa-Ambiental/
├── TX_Ambiental/
│   └── TX_Ambiental.ino     # Firmware del transmisor (sensores + LoRa TX)
├── RX_Ambiental/
│   └── RX_Ambiental.ino     # Firmware del receptor (LoRa RX + WiFi + MQTT)
└── README.md                # Este documento
```

---

## 📄 Licencia

Proyecto educativo de código abierto. Libre para modificar y distribuir con atribución.
