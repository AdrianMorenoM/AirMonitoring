// ============================================================
// LoRa Heltec V3 — RECEPTOR AMBIENTAL
// Este archivo es el lado RECEPTOR del sistema.
// Usa dos núcleos del ESP32:
//   - Core 0: maneja la radio LoRa (recepción y ACKs)
//   - Core 1: maneja WiFi y publicación MQTT
// ============================================================
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#include "HT_SSD1306Wire.h"  // Driver para la pantalla OLED integrada
#include <WiFi.h>
#include <PubSubClient.h>    // Biblioteca para protocolo MQTT
#include <stdarg.h>

// ====================== CONFIGURACIÓN LoRa ==================
// Estos parámetros DEBEN ser idénticos en el transmisor (TX)
// para que los dos dispositivos se "entiendan" por radio.
#define RF_FREQUENCY              915000000  // Frecuencia 915 MHz (banda ISM América)
#define TX_OUTPUT_POWER           5          // Potencia de transmisión (dBm) — para el ACK
#define LORA_BANDWIDTH            0          // BW 125 kHz
#define LORA_SPREADING_FACTOR     7          // SF7: mayor velocidad, menor alcance
#define LORA_CODINGRATE           1          // 4/5 — redundancia para corrección de errores
#define LORA_PREAMBLE_LENGTH      8          // Bytes de preámbulo para sincronizar trama
#define LORA_FIX_LENGTH_PAYLOAD   false      // Payload de longitud variable
#define LORA_IQ_INVERSION         false      // Sin inversión IQ
#define LORA_SYMBOL_TIMEOUT       5          // Timeout en símbolos para modo RX continuo
#define LORA_SYNC_WORD            0x2A       // "Contraseña" de red LoRa (filtra redes ajenas)

// ====================== PARÁMETROS PROTOCOLO ================
#define BUFFER_SIZE         256             // Tamaño máximo de un paquete recibido (bytes)
#define EXPECTED_NODE_ID    "TX_AMBIENTAL_01" // Solo acepta paquetes de este nodo transmisor
#define DATA_TIMEOUT_MS     5000            // Si no llega un paquete en 5s, pantalla muestra "..."
#define SEQ_BUFFER_SIZE     64              // Cuántos números de secuencia recientes se recuerdan
                                            // (para detectar paquetes duplicados)

// ====================== CREDENCIALES WiFi ===================
// Lista de redes WiFi conocidas; el receptor intentará
// conectarse a cada una en orden hasta encontrar una disponible.
struct WiFiCred { const char* ssid; const char* password; };
WiFiCred wifiList[] = {
    {"LAPTOP-HO7CN3DU 1641", "12345678910"},
    {"ALUMNOS - ITSOEH",     "SomosLobos"},
    {"Docentes-ITSEOH",      "5ddmil331ri8"},
    {"INFINITUMB2DF",        "YqFPE4cEDc"}
};
const int WIFI_COUNT = sizeof(wifiList) / sizeof(wifiList[0]); // Calcula el total automáticamente

// ====================== MQTT ================================
// Broker MQTT público (HiveMQ). En producción conviene usar
// un broker privado con autenticación.
const char*  MQTT_SERVER = "broker.hivemq.com";
const int    MQTT_PORT   = 1883;              // Puerto estándar MQTT sin cifrado
const char*  MQTT_TOPIC  = "AirMonitoring";   // Topic donde se publican los datos del sensor

WiFiClient   espClient;                       // Socket TCP subyacente
PubSubClient mqttClient(espClient);           // Cliente MQTT sobre ese socket

// ====================== ESTRUCTURAS =========================

// Almacena el último paquete de datos de sensores recibido por LoRa.
struct SensorData {
    int      mq2   = 0;    // Lectura sensor MQ-2  (gases inflamables/humo)
    int      mq135 = 0;    // Lectura sensor MQ-135 (calidad de aire / CO2, NH3, etc.)
    int      dust  = 0;    // Lectura sensor de polvo (partículas en suspensión)
    uint16_t seq   = 0;    // Número de secuencia del paquete
    char     nodeId[32];   // ID del nodo transmisor que envió el dato
    int16_t  rssi  = 0;    // RSSI al momento de recibir (potencia de señal, dBm)
    int8_t   snr   = 0;    // SNR (relación señal/ruido, dB)
    unsigned long timestamp = 0; // Tiempo en ms (millis()) en que se recibió
    bool     valid = false; // true = hay datos válidos; false = aún no se recibió nada
};

// Contadores de diagnóstico para evaluar la calidad del enlace LoRa.
struct Stats {
    uint32_t packetsReceived    = 0; // Total de paquetes recibidos (incluyendo duplicados)
    uint32_t packetsUnique      = 0; // Paquetes nuevos únicos (sin contar duplicados)
    uint32_t packetsDuplicated  = 0; // Paquetes repetidos detectados
    uint32_t packetsLost        = 0; // Paquetes estimados como perdidos (por brechas en seq)
    uint32_t ackSent            = 0; // Confirmaciones (ACK) enviadas de vuelta al TX
    uint32_t lastSeq            = 0; // Último número de secuencia procesado
    int16_t  minRSSI            = 0;    // RSSI mínimo histórico
    int16_t  maxRSSI            = -1000;// RSSI máximo histórico
    int8_t   minSNR             = 127;  // SNR mínimo histórico
    int8_t   maxSNR             = -128; // SNR máximo histórico
    float    avgRSSI            = 0.0f; // Promedio acumulado de RSSI
    float    avgSNR             = 0.0f; // Promedio acumulado de SNR
    uint32_t packetsWithRSSI    = 0;    // Paquetes que contribuyeron al promedio
    uint32_t consecutiveLoss    = 0;    // Pérdidas consecutivas actuales
    uint32_t maxConsecutiveLoss = 0;    // Racha máxima de pérdidas consecutivas
    float    lossRate           = 0.0f; // Tasa de pérdida (0.0 = 0%, 1.0 = 100%)
} stats;

// Resumen simplificado de calidad de enlace (usado en paralelo con Stats).
struct LinkQuality {
    uint32_t totalPackets   = 0;
    uint32_t lostPackets    = 0;
    float    packetLossRate = 0.0f;
    unsigned long lastUpdate = 0;
} linkQuality;

// ====================== VARIABLES GLOBALES ==================
static RadioEvents_t RadioEvents; // Estructura de callbacks de la radio LoRa
char rxpacket[BUFFER_SIZE];       // Buffer temporal donde se copia el payload recibido

// Banderas para coordinar el envío de ACK entre el callback de radio y loraTask.
// 'volatile' indica al compilador que otro contexto (ISR/otra tarea) puede cambiarlas.
volatile bool     ackPending  = false; // true = hay un ACK pendiente por enviar
volatile uint16_t ackSequence = 0;    // Número de secuencia al que va ese ACK
volatile bool     lora_idle   = true; // true = la radio no está transmitiendo (libre)

uint16_t      lastProcessedSeq = 0;   // Último seq válido procesado (detectar brechas)
SensorData    sensorData;             // Últimos datos de sensores recibidos
unsigned long lastPacketTime   = 0;   // Timestamp del último paquete válido recibido

// Buffer circular para deduplicación: guarda los últimos SEQ_BUFFER_SIZE
// números de secuencia vistos para no procesar el mismo paquete dos veces.
uint16_t recentSeqs[SEQ_BUFFER_SIZE];
uint8_t  seqIndex = 0; // Índice circular (avanza y da la vuelta al llegar al límite)

// ─────────────────────────────────────────────────────────────
// Mecanismo de comunicación segura entre el callback de radio
// (que corre en contexto de ISR/interrupción) y mqttTask
// (que corre en Core 1 y puede llamar funciones de red).
//
// OnRxDone NO puede llamar funciones WiFi/MQTT directamente
// porque son bloqueantes y no son seguras desde una ISR.
// Solución: OnRxDone escribe el payload en mqttPayloadBuf
// y activa mqttFlag; mqttTask detecta la bandera y publica.
// ─────────────────────────────────────────────────────────────
volatile bool mqttFlag = false;       // Señal: "hay un payload listo para publicar"
char          mqttPayloadBuf[192];    // Buffer con el JSON listo para enviar por MQTT

// Objeto de la pantalla OLED (dirección I2C 0x3c, 500kHz, pines definidos por la placa)
SSD1306Wire factory_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// ====================== UTILIDADES ==========================

// Enciende la alimentación externa (Vext) necesaria para la pantalla OLED en Heltec V3.
void VextON() { pinMode(Vext, OUTPUT); digitalWrite(Vext, LOW); }

// Imprime una línea de separadores en el Monitor Serial (útil para depuración visual).
void printSeparator(char c = '=', int len = 60) {
    for (int i = 0; i < len; i++) Serial.print(c);
    Serial.println();
}

// Imprime un encabezado con título enmarcado en separadores.
void printHeader(const char* title) {
    Serial.println();
    printSeparator();
    Serial.printf(">> %s\n", title);
    printSeparator();
}

// ====================== DEDUPLICACIÓN =======================

// Verifica si el número de secuencia 'seq' ya fue procesado recientemente.
// Recorre el buffer circular de secuencias; si lo encuentra → es duplicado.
bool isDuplicateSeq(uint16_t seq) {
    for (int i = 0; i < SEQ_BUFFER_SIZE; i++)
        if (recentSeqs[i] == seq) return true;
    return false;
}

// Agrega un número de secuencia al buffer circular.
// Cuando el índice llega al final, vuelve a 0 (sobrescribe el más antiguo).
void addToSeqBuffer(uint16_t seq) {
    recentSeqs[seqIndex] = seq;
    seqIndex = (seqIndex + 1) % SEQ_BUFFER_SIZE;
}

// ====================== CALIDAD DE ENLACE ===================

// Actualiza todas las métricas de calidad cada vez que llega (o se pierde) un paquete.
// Si 'received' es false → se contabiliza como paquete perdido y se incrementa la racha.
// Si 'received' es true  → se actualizan promedios de RSSI/SNR con media móvil incremental.
void updateLinkQuality(bool received, int16_t rssi = 0, int8_t snr = 0) {
    linkQuality.totalPackets++;
    if (!received) {
        linkQuality.lostPackets++;
        stats.consecutiveLoss++;
        if (stats.consecutiveLoss > stats.maxConsecutiveLoss)
            stats.maxConsecutiveLoss = stats.consecutiveLoss;
    } else {
        // Media móvil incremental: avg_new = (avg_old * n + nuevo) / (n + 1)
        float n = (float)stats.packetsWithRSSI;
        stats.avgRSSI = (n > 0) ? (stats.avgRSSI * n + rssi) / (n + 1) : (float)rssi;
        stats.avgSNR  = (n > 0) ? (stats.avgSNR  * n + snr)  / (n + 1) : (float)snr;
        stats.packetsWithRSSI++;
        stats.consecutiveLoss = 0; // Resetea la racha de pérdidas consecutivas
    }
    // Recalcula la tasa de pérdida acumulada
    linkQuality.packetLossRate = (linkQuality.totalPackets > 0)
        ? (float)linkQuality.lostPackets / linkQuality.totalPackets : 0.0f;
    stats.lossRate         = linkQuality.packetLossRate;
    linkQuality.lastUpdate = millis();
}

// ====================== PANTALLA ============================

// Refresca completamente la pantalla OLED con el estado actual del sistema.
// Se llama periódicamente desde mqttTask (cada ~100 ms).
void drawDisplay() {
    factory_display.clear();
    factory_display.setTextAlignment(TEXT_ALIGN_LEFT);
    factory_display.setFont(ArialMT_Plain_10);

    // Fila 0: título + estado WiFi (W:OK/W:--) + estado MQTT (M:OK/M:--)
    factory_display.drawString(0, 0, "RX AMBIENTAL");
    factory_display.drawString(75, 0,
        WiFi.status() == WL_CONNECTED ? "W:OK" : "W:--");
    factory_display.drawString(104, 0,
        mqttClient.connected() ? "M:OK" : "M:--");

    // Solo muestra datos si el último paquete llegó hace menos de DATA_TIMEOUT_MS
    bool fresh = sensorData.valid && (millis() - lastPacketTime < DATA_TIMEOUT_MS);
    if (fresh) {
        char buf[32];
        snprintf(buf, sizeof(buf), "M2:%d M5:%d", sensorData.mq2, sensorData.mq135);
        factory_display.drawString(0, 12, buf); // Fila 1: lecturas MQ-2 y MQ-135

        snprintf(buf, sizeof(buf), "DUST:%d", sensorData.dust);
        factory_display.drawString(0, 24, buf); // Fila 2: lectura de polvo

        snprintf(buf, sizeof(buf), "R:%d S:%d", sensorData.rssi, sensorData.snr);
        factory_display.drawString(0, 36, buf); // Fila 3: RSSI y SNR del enlace

        // Fila 4: paquetes únicos recibidos + tasa de pérdida + último SEQ
        snprintf(buf, sizeof(buf), "P:%lu L:%.1f%%",
            stats.packetsUnique, stats.lossRate * 100.0f);
        factory_display.drawString(0, 48, buf);
        snprintf(buf, sizeof(buf), "S:%u", sensorData.seq);
        factory_display.drawString(80, 48, buf);
    } else {
        // Sin datos recientes: muestra indicador de espera
        factory_display.setFont(ArialMT_Plain_16);
        factory_display.drawString(20, 20, "...");
        factory_display.setFont(ArialMT_Plain_10);
        factory_display.drawString(0, 48, "Esperando datos...");
    }
    factory_display.display(); // Envía el buffer a la pantalla física
}

// ====================== STATUS SERIAL =======================

// Imprime un resumen de estado en el Monitor Serial cada 5 segundos.
// Útil para depurar sin necesidad de pantalla externa.
void printCompactStatusRX() {
    printSeparator();
    Serial.printf("[RX] UPTIME:%lu ms\n", millis());
    printSeparator('-');
    if (sensorData.valid)
        Serial.printf("SEQ:%u | MQ2:%d | MQ135:%d | DUST:%d | RSSI:%d | SNR:%d\n",
            sensorData.seq, sensorData.mq2, sensorData.mq135,
            sensorData.dust, sensorData.rssi, sensorData.snr);
    else
        Serial.println("SIN DATOS");
    Serial.printf("RX:%lu UNI:%lu DUP:%lu LOST:%lu | ACK:%lu | LOSS:%.1f%%\n",
        stats.packetsReceived, stats.packetsUnique, stats.packetsDuplicated,
        stats.packetsLost, stats.ackSent, stats.lossRate * 100.0f);
    Serial.printf("WiFi:%s  MQTT:%s\n",
        WiFi.status() == WL_CONNECTED ? "OK" : "DESCONECTADO",
        mqttClient.connected()        ? "OK" : "DESCONECTADO");
    printSeparator();
}

// ====================== WiFi ================================

// Recorre la lista de redes WiFi y trata de conectarse a cada una.
// Espera hasta 6 segundos por red. Retorna true si logra conectar.
// Usa vTaskDelay (no delay) para no bloquear el scheduler de FreeRTOS.
bool tryConnectWiFi() {
    for (int i = 0; i < WIFI_COUNT; i++) {
        Serial.printf("[WiFi] Intentando %s... ", wifiList[i].ssid);
        WiFi.begin(wifiList[i].ssid, wifiList[i].password);
        unsigned long t = millis();
        while (millis() - t < 6000) {
            if (WiFi.status() == WL_CONNECTED) {
                Serial.printf("OK  IP:%s\n", WiFi.localIP().toString().c_str());
                return true;
            }
            vTaskDelay(pdMS_TO_TICKS(500)); // Cede el CPU mientras espera
        }
        WiFi.disconnect(true); // Descarta la conexión fallida antes de intentar la siguiente
        Serial.println("fallo");
    }
    Serial.println("[WiFi] Sin conexion disponible");
    return false;
}

// ====================== MQTT ================================

// Intenta reconectar al broker MQTT UNA sola vez (no bloqueante).
// Genera un clientId aleatorio para evitar conflictos de sesión.
// Si falla, retorna false; mqttTask reintentará según su temporizador.
bool reconnectMQTT() {
    if (mqttClient.connected()) return true;   // Ya conectado, nada que hacer
    if (WiFi.status() != WL_CONNECTED) return false; // Sin WiFi no hay MQTT

    char clientId[32];
    // ID único usando los 16 bits bajos de un número aleatorio del hardware
    snprintf(clientId, sizeof(clientId),
             "ESP32_RX_%04X", (uint16_t)esp_random());

    Serial.printf("[MQTT] Conectando como %s a %s:%d... ",
                  clientId, MQTT_SERVER, MQTT_PORT);

    if (mqttClient.connect(clientId)) {
        Serial.println("OK");
        return true;
    }

    Serial.printf("fallo rc=%d\n", mqttClient.state()); // rc: código de error MQTT
    return false;
}

// ====================== CALLBACKS LoRa ======================

// Llamado automáticamente cuando termina de enviarse un ACK.
// Devuelve la radio al modo recepción continua.
void OnTxDone() {
    lora_idle = true;
    Radio.Rx(0); // 0 = timeout infinito (escucha continuamente)
}

// Llamado si el envío del ACK supera el tiempo límite.
void OnTxTimeout() {
    Serial.println("[RX] Timeout enviando ACK");
    lora_idle = true;
    Radio.Rx(0);
}

// ─────────────────────────────────────────────────────────────
// OnRxDone — Callback de la radio LoRa (contexto quasi-ISR).
// Se ejecuta CADA VEZ que llega un paquete por aire.
// Reglas: NO llamar funciones de red, NO usar delay(),
//         NO hacer operaciones lentas.
// ─────────────────────────────────────────────────────────────
void OnRxDone(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr) {
    // 1. Copiar payload al buffer local y asegurar terminación de cadena
    if (size >= BUFFER_SIZE) size = BUFFER_SIZE - 1;
    memcpy(rxpacket, payload, size);
    rxpacket[size] = '\0';

    Serial.printf("[RX] RAW: %s | RSSI:%d SNR:%d\n", rxpacket, rssi, snr);

    // ── 1. Parsear el JSON recibido ──────────────────────────
    // Formato esperado: {"seq":N,"id":"TX_AMBIENTAL_01","mq2":N,"mq135":N,"dust":N}
    uint16_t seq;
    int mq2, mq135, dust;
    char id[32];
    if (sscanf(rxpacket,
            "{\"seq\":%hu,\"id\":\"%31[^\"]\",\"mq2\":%d,\"mq135\":%d,\"dust\":%d}",
            &seq, id, &mq2, &mq135, &dust) != 5) // sscanf devuelve 5 si parseó los 5 campos
    {
        Serial.println("[RX] Paquete mal formado");
        stats.packetsLost++;
        updateLinkQuality(false);
        Radio.Rx(0); // Volver a escuchar de inmediato
        return;
    }

    // ── 2. Verificar que el paquete venga del nodo esperado ─
    if (strcmp(id, EXPECTED_NODE_ID) != 0) {
        Serial.printf("[RX] NodeID desconocido: %s\n", id);
        Radio.Rx(0);
        return; // Ignorar paquetes de nodos desconocidos
    }

    // ── 3. Actualizar contadores y estadísticas de señal ────
    stats.packetsReceived++;
    stats.packetsWithRSSI++;
    if (rssi < stats.minRSSI) stats.minRSSI = rssi;
    if (rssi > stats.maxRSSI) stats.maxRSSI = rssi;
    if (snr  < stats.minSNR)  stats.minSNR  = snr;
    if (snr  > stats.maxSNR)  stats.maxSNR  = snr;

    // ── 4. Detectar brechas de secuencia (paquetes perdidos) ─
    // Si el seq recibido es mayor al esperado, hubo pérdidas en el aire.
    if (lastProcessedSeq > 0 && seq > (uint16_t)(lastProcessedSeq + 1)) {
        uint16_t lost = seq - lastProcessedSeq - 1;
        stats.packetsLost += lost;
        Serial.printf("[RX] Brecha: esperaba %u llego %u (%u perdidos)\n",
            lastProcessedSeq + 1, seq, lost);
        for (uint16_t k = 0; k < lost; k++) updateLinkQuality(false); // Registrar cada pérdida
    }

    // ── 5. Detectar duplicados ───────────────────────────────
    // El transmisor puede reenviar si no recibe ACK; aquí se descarta el duplicado
    // pero igual se envía un ACK de cortesía para que el TX sepa que llegó.
    if (isDuplicateSeq(seq)) {
        stats.packetsDuplicated++;
        Serial.printf("[RX] Duplicado SEQ=%u — ACK cortesia\n", seq);
        ackSequence = seq;
        ackPending  = true; // Notifica a loraTask que envíe el ACK
        Radio.Rx(0);
        return; // No se procesa de nuevo el dato del sensor
    }

    // ── 6. Procesar paquete nuevo válido ─────────────────────
    stats.packetsUnique++;
    stats.lastSeq    = seq;
    lastProcessedSeq = seq;
    addToSeqBuffer(seq); // Recordar este seq para futura deduplicación

    // Guardar datos del sensor en la estructura global
    sensorData.mq2   = mq2;  sensorData.mq135 = mq135;
    sensorData.dust  = dust; sensorData.seq   = seq;
    sensorData.rssi  = rssi; sensorData.snr   = snr;
    sensorData.valid = true; sensorData.timestamp = millis();
    strncpy(sensorData.nodeId, id, sizeof(sensorData.nodeId) - 1);
    sensorData.nodeId[sizeof(sensorData.nodeId) - 1] = '\0'; // Asegurar null-terminator
    lastPacketTime = millis();
    updateLinkQuality(true, rssi, snr);

    Serial.printf("[RX] SEQ:%u MQ2:%d MQ135:%d DUST:%d\n", seq, mq2, mq135, dust);

    // ── Preparar payload JSON para MQTT ─────────────────────
    // Se escribe en el buffer compartido y se activa la bandera.
    // mqttTask (Core 1) leerá esto de forma segura y publicará.
    snprintf(mqttPayloadBuf, sizeof(mqttPayloadBuf),
        "{\"seq\":%u,\"nodeId\":\"%s\","
        "\"mq2\":%d,\"mq135\":%d,\"dust\":%d,"
        "\"rssi\":%d,\"snr\":%d}",
        seq, id, mq2, mq135, dust, rssi, snr);
    mqttFlag = true; // Activa la señal para mqttTask

    // ── Programar envío de ACK ───────────────────────────────
    ackSequence = seq;
    ackPending  = true; // loraTask verá esta bandera y enviará el ACK
    Radio.Rx(0);        // Volver a modo recepción
}

// ====================== TAREA LoRa (CORE 0) =================
// Tarea dedicada exclusivamente a la radio.
// Corre en Core 0 con prioridad alta (2) para no perder interrupciones.
// NO hace ninguna llamada de red (WiFi/MQTT).
void loraTask(void* pv) {
    Radio.Rx(0); // Iniciar escucha continua
    Serial.println("[TASK] LoRa task iniciada (Core 0)");

    unsigned long lastAckTime    = 0; // Anti-rebote: evita enviar ACKs demasiado seguidos
    unsigned long lastStatusTime = 0; // Control del reporte periódico serial

    while (1) {
        Radio.IrqProcess(); // Procesa interrupciones pendientes de la radio (obligatorio)

        // ── Enviar ACK si hay uno pendiente y la radio está libre ─
        // El retardo mínimo de 50 ms evita colisiones con la transmisión anterior.
        if (ackPending && lora_idle && (millis() - lastAckTime > 50)) {
            uint16_t seqToAck = ackSequence;
            ackPending  = false; // Limpiar bandera antes de transmitir
            lora_idle   = false; // Marcar radio como ocupada
            lastAckTime = millis();
            stats.ackSent++;

            char ackBuf[32];
            snprintf(ackBuf, sizeof(ackBuf), "{\"ack\":%u}", seqToAck);
            Serial.printf("[RX] Enviando ACK: %s\n", ackBuf);
            Radio.Send((uint8_t*)ackBuf, strlen(ackBuf)); // Enviar ACK por aire al TX
        }

        // ── Status periódico en Serial cada 5 segundos ───────
        if (millis() - lastStatusTime > 5000) {
            lastStatusTime = millis();
            printCompactStatusRX();
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Cede CPU 10 ms para que otras tareas puedan correr
    }
}

// ====================== TAREA WiFi/MQTT (CORE 1) ============
// Tarea separada para todo lo relacionado con red.
// Corre en Core 1 con prioridad normal (1).
// Nunca llama funciones de la radio LoRa.
//
// Razón de la separación:
//  - WiFi puede bloquearse segundos durante reconexión
//  - mqttClient.loop() necesita ejecutarse muy frecuentemente
//  - Si estuvieran en la misma tarea, una reconexión WiFi
//    haría que se pierdan interrupciones de LoRa
void mqttTask(void* pv) {
    Serial.println("[TASK] MQTT task iniciada (Core 1)");

    mqttClient.setServer(MQTT_SERVER, MQTT_PORT); // Configurar broker (una sola vez)

    tryConnectWiFi(); // Intentar conexión WiFi inicial al arrancar

    unsigned long lastReconnectAttempt = 0;
    const unsigned long RECONNECT_INTERVAL = 10000; // Reintentar MQTT cada 10 segundos

    while (1) {
        // ── Verificar estado WiFi ────────────────────────────
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("[WiFi] Conexion perdida, reconectando...");
            tryConnectWiFi();
            lastReconnectAttempt = millis(); // Reiniciar timer para no intentar MQTT de inmediato
        }

        // ── Reconectar MQTT si es necesario (sin delay bloqueante) ─
        if (!mqttClient.connected()) {
            if (millis() - lastReconnectAttempt >= RECONNECT_INTERVAL) {
                lastReconnectAttempt = millis();
                reconnectMQTT(); // Un solo intento; si falla, reintentará en 10 s
            }
        }

        // ── Loop MQTT: mantiene el keepalive y procesa mensajes entrantes ─
        if (mqttClient.connected())
            mqttClient.loop();

        // ── Publicar datos si LoRa recibió un paquete nuevo ──
        if (mqttFlag && mqttClient.connected()) {
            mqttFlag = false; // Limpiar bandera ANTES de publicar (evita doble publicación)
            bool ok = mqttClient.publish(MQTT_TOPIC, mqttPayloadBuf);
            Serial.printf("[MQTT] Publish %s: %s\n",
                          MQTT_TOPIC, ok ? "OK" : "FALLO");
        }

        // ── Actualizar pantalla OLED ─────────────────────────
        drawDisplay();

        vTaskDelay(pdMS_TO_TICKS(100)); // Ciclo cada 100 ms (10 Hz de refresco)
    }
}

// ====================== SETUP ===============================
// Función que se ejecuta una sola vez al encender el dispositivo.
// Configura hardware, radio y lanza las dos tareas de FreeRTOS.
void setup() {
    Serial.begin(115200);
    delay(1000);
    printHeader("SISTEMA AMBIENTAL RX - INICIANDO");

    VextON();            // Encender alimentación de la pantalla
    delay(100);
    Wire.begin(SDA_OLED, SCL_OLED); // Inicializar bus I2C para la OLED
    delay(50);

    // Mostrar pantalla de bienvenida mientras inicializa
    factory_display.init();
    factory_display.clear();
    factory_display.setFont(ArialMT_Plain_10);
    factory_display.drawString(0, 10, "WiFi LoRa 32 V3");
    factory_display.drawString(0, 25, "Receptor Ambiental");
    factory_display.drawString(0, 40, "Inicializando...");
    factory_display.display();
    delay(2000);

    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE); // Inicializar MCU Heltec

    // Asignar los callbacks de la radio LoRa
    RadioEvents.RxDone    = OnRxDone;    // Paquete recibido
    RadioEvents.TxDone    = OnTxDone;    // ACK enviado correctamente
    RadioEvents.TxTimeout = OnTxTimeout; // ACK tardó demasiado

    Radio.Init(&RadioEvents);
    Radio.SetPublicNetwork(false);      // Red privada (no LoRaWAN público)
    Radio.SetSyncWord(LORA_SYNC_WORD);  // Filtro de red (0x2A)
    Radio.SetChannel(RF_FREQUENCY);     // Frecuencia 915 MHz

    // Configurar parámetros de transmisión (para enviar ACKs)
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0,
        LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE,
        LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD,
        true, 0, 0, LORA_IQ_INVERSION, 3000);

    // Configurar parámetros de recepción (deben coincidir exactamente con el TX)
    Radio.SetRxConfig(MODEM_LORA,
        LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE,
        0, LORA_PREAMBLE_LENGTH, LORA_SYMBOL_TIMEOUT,
        LORA_FIX_LENGTH_PAYLOAD, 0, true, 0, 0, LORA_IQ_INVERSION, true);

    // Inicializar buffer de deduplicación con valores inválidos (0xFFFF no es un seq normal)
    memset(recentSeqs, 0xFF, sizeof(recentSeqs));

    // Inicializar estructura de calidad de enlace
    linkQuality.totalPackets   = 0;
    linkQuality.lostPackets    = 0;
    linkQuality.packetLossRate = 0.0f;
    linkQuality.lastUpdate     = millis();

    // Crear las dos tareas FreeRTOS y fijarlas a sus respectivos núcleos:
    //   loraTask → Core 0, stack 4 KB, prioridad 2 (alta)
    //   mqttTask → Core 1, stack 8 KB, prioridad 1 (normal, necesita más stack por WiFi/MQTT)
    xTaskCreatePinnedToCore(loraTask, "LoRaTask", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(mqttTask, "MqttTask", 8192, NULL, 1, NULL, 1);

    Serial.println("[SETUP] Sistema receptor listo\n");
}

// loop() queda vacío porque todo el trabajo lo hacen las tareas FreeRTOS.
// portMAX_DELAY suspende loop() indefinidamente sin consumir CPU.
void loop() { vTaskDelay(portMAX_DELAY); }
