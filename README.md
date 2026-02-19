# 🌍 Sistema de Telemetría Ambiental con LoRa y FreeRTOS

Este proyecto implementa un sistema de monitoreo de calidad del aire utilizando dos dispositivos **Heltec LoRa V3 (ESP32‑S3)**. Un dispositivo actúa como **transmisor (TX)** equipado con sensores de gases y partículas, y el otro como **receptor (RX)** que recibe los datos por radio LoRa y los publica en un broker MQTT para su visualización en internet.

El sistema aprovecha la capacidad de **doble núcleo** del ESP32 y el sistema operativo en tiempo real **FreeRTOS** para ejecutar tareas en paralelo, garantizando que la comunicación por radio sea confiable y que la publicación por WiFi no interfiera con la recepción de datos.

---

## ✨ Características

- **Comunicación LoRa robusta** – Protocolo **Stop‑and‑Wait** con acuses de recibo (ACK) y reintentos automáticos (hasta 3 veces).
- **Multitarea real** – Dos tareas FreeRTOS en cada dispositivo, fijadas a núcleos específicos para optimizar el rendimiento.
- **Sensores analógicos** – Mide gases inflamables (MQ‑2), calidad del aire (MQ‑135) y partículas en suspensión (GP2Y1010).
- **Visualización en pantalla OLED** – Muestra valores actuales de sensores, estado del enlace y estadísticas de comunicación.
- **Publicación en la nube** – El receptor se conecta a WiFi y publica los datos en un broker MQTT público (HiveMQ).
- **Monitorización serie** – Mensajes detallados de cada evento para depuración y análisis de rendimiento.

---

## 🛠️ Hardware necesario

- 2 × **Heltec LoRa V3** (ESP32‑S3 + módulo LoRa SX1262)
- Sensores para el transmisor:
  - MQ‑2 (gases combustibles, humo)
  - MQ‑135 (amoníaco, CO₂, compuestos orgánicos volátiles)
  - GP2Y1010AU0F (sensor óptico de polvo)
- Cables, protoboard y alimentación (5V para sensores)

---

## 🧠 Arquitectura del software

Ambos dispositivos ejecutan **FreeRTOS**, que permite dividir el trabajo en tareas independientes. Cada tarea tiene su propia prioridad y está fijada a un núcleo para evitar migraciones y mejorar la determinación temporal.

### Transmisor (TX)

| Núcleo | Tarea         | Prioridad | Función                                                                 |
|--------|---------------|-----------|-------------------------------------------------------------------------|
| 1      | `sensorTask`  | 1 (normal)| Lee los tres sensores cada 2 segundos y actualiza la pantalla OLED.    |
| 0      | `loraTask`    | 2 (alta)  | Gestiona la radio LoRa: envía paquetes, maneja ACK, timeouts y colas.  |

### Receptor (RX)

| Núcleo | Tarea         | Prioridad | Función                                                                 |
|--------|---------------|-----------|-------------------------------------------------------------------------|
| 0      | `loraTask`    | 2 (alta)  | Escucha la radio, recibe paquetes, envía ACK y actualiza estadísticas. |
| 1      | `mqttTask`    | 1 (normal)| Mantiene WiFi, publica en MQTT y actualiza la pantalla OLED.           |

---

## 📡 Protocolo de comunicación: Stop‑and‑Wait

LoRa es **half‑duplex**, es decir, no puede transmitir y recibir al mismo tiempo. Para evitar colisiones y garantizar la entrega, se implementa un protocolo sencillo pero eficaz:

1. El **transmisor** envía un paquete con un número de secuencia único.
2. Espera un tiempo (2 segundos) la confirmación (ACK) del receptor.
3. El **receptor**, al recibir el paquete, responde inmediatamente con un ACK que contiene el mismo número de secuencia.
4. Si el transmisor no recibe el ACK dentro del tiempo límite, **retransmite** el mismo paquete (hasta 3 veces). Si agota los reintentos, lo descarta y pasa al siguiente.
5. El receptor **detecta duplicados** mediante un buffer circular de secuencias recientes y envía un ACK aunque no procese el dato de nuevo (así el TX deja de retransmitir).

Este mecanismo asegura que cada paquete llegue al menos una vez (o se notifique su pérdida).

---

## 🔄 Flujo de datos paso a paso

### Transmisor (TX)

1. **`sensorTask`** (cada 2 s):
   - Lee los tres sensores.
   - Protege los datos con un **mutex** y los guarda en la variable compartida `sensorData`.
   - Actualiza la pantalla OLED con los valores.

2. **`loraTask`** (cada 10 ms):
   - Toma una copia segura de `sensorData`.
   - Si hay datos frescos y ha pasado el intervalo de envío, crea un paquete JSON y lo encola.
   - Si la radio está libre, hay paquetes en cola y se respeta la ventana de congestión (fijada en 1), envía un paquete y lo mueve a la lista **“en vuelo”** (pendiente de ACK).
   - Comprueba si algún paquete en vuelo ha superado el tiempo de espera; si es así, lo retransmite o lo descarta.
   - Cuando llega un ACK (manejado por el callback `OnRxDone`), elimina el paquete de la lista en vuelo y actualiza estadísticas (RTT, etc.).

### Receptor (RX)

1. **Callback `OnRxDone`** (se ejecuta al recibir un paquete):
   - Parsea el JSON recibido y verifica que el `id` del nodo sea el esperado.
   - Actualiza estadísticas de RSSI/SNR y detecta pérdidas mediante números de secuencia.
   - Si el paquete es **nuevo** (no duplicado), guarda los datos en `sensorData` y prepara un payload para MQTT en un buffer compartido, activando una bandera (`mqttFlag`).
   - Siempre activa la bandera `ackPending` para que **`loraTask`** envíe la confirmación.
   - Si es duplicado, igual activa `ackPending` (para que el TX deje de retransmitir) pero no actualiza los datos ni publica.

2. **`loraTask`** (Core 0, cada 10 ms):
   - Procesa interrupciones de la radio.
   - Si hay `ackPending` y la radio está libre, envía un ACK con el número de secuencia correspondiente.
   - Imprime estadísticas cada 5 segundos.

3. **`mqttTask`** (Core 1, cada 100 ms):
   - Mantiene la conexión WiFi y MQTT (reconexiones automáticas).
   - Si `mqttFlag` está activa y MQTT conectado, publica el payload en el tópico correspondiente y desactiva la bandera.
   - Actualiza la pantalla OLED con los últimos datos recibidos.

---

## 🧩 Fragmentos de código destacados

### Envío de un paquete (transmisor)

```cpp
// Dentro de loraTask, cuando se cumplen las condiciones
PacketData pkt;
if (dequeuePacket(&pkt)) {
    pkt.lastAttempt = millis();
    inflight[inflightCount++] = pkt;  // Pasa a "en vuelo"
    stats.packetsSent++;
    snprintf(txpacket, BUFFER_SIZE, "%s", pkt.data);
    Radio.Send((uint8_t*)txpacket, strlen(txpacket));
    lora_idle = false;
    logEvent("TX", "SEND", "✉️ Seq:%u", pkt.seq);
}
