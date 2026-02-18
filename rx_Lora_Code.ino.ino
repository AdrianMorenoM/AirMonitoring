// ============================================================
// LoRa Heltec V3 — RECEPTOR AMBIENTAL
// LoRa (Core 0) + WiFi/MQTT (Core 1, tarea dedicada)
// ============================================================
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <stdarg.h>

// ====================== CONFIGURACIÓN LoRa ==================
#define RF_FREQUENCY              915000000
#define TX_OUTPUT_POWER           5
#define LORA_BANDWIDTH            0
#define LORA_SPREADING_FACTOR     7
#define LORA_CODINGRATE           1
#define LORA_PREAMBLE_LENGTH      8
#define LORA_FIX_LENGTH_PAYLOAD   false
#define LORA_IQ_INVERSION         false
#define LORA_SYMBOL_TIMEOUT       5
#define LORA_SYNC_WORD            0x2A

// ====================== PARÁMETROS PROTOCOLO ================
#define BUFFER_SIZE         256
#define EXPECTED_NODE_ID    "TX_AMBIENTAL_01"
#define DATA_TIMEOUT_MS     5000
#define SEQ_BUFFER_SIZE     64

// ====================== CREDENCIALES WiFi ===================
struct WiFiCred { const char* ssid; const char* password; };
WiFiCred wifiList[] = {
    {"LAPTOP-HO7CN3DU 1641", "12345678910"},
    {"ALUMNOS - ITSOEH",     "SomosLobos"},
    {"Docentes-ITSEOH",      "5ddmil331ri8"},
    {"INFINITUMB2DF",        "YqFPE4cEDc"}
};
const int WIFI_COUNT = sizeof(wifiList) / sizeof(wifiList[0]);

// ====================== MQTT ================================
// ⚠️ Cambiar a la IP real del broker en la red activa
const char*  MQTT_SERVER = "broker.hivemq.com";
const int    MQTT_PORT   = 1883;
const char*  MQTT_TOPIC  = "AirMonitoring";

WiFiClient   espClient;
PubSubClient mqttClient(espClient);

// ====================== ESTRUCTURAS =========================
struct SensorData {
    int      mq2   = 0;
    int      mq135 = 0;
    int      dust  = 0;
    uint16_t seq   = 0;
    char     nodeId[32];
    int16_t  rssi  = 0;
    int8_t   snr   = 0;
    unsigned long timestamp = 0;
    bool     valid = false;
};

struct Stats {
    uint32_t packetsReceived    = 0;
    uint32_t packetsUnique      = 0;
    uint32_t packetsDuplicated  = 0;
    uint32_t packetsLost        = 0;
    uint32_t ackSent            = 0;
    uint32_t lastSeq            = 0;
    int16_t  minRSSI            = 0;
    int16_t  maxRSSI            = -1000;
    int8_t   minSNR             = 127;
    int8_t   maxSNR             = -128;
    float    avgRSSI            = 0.0f;
    float    avgSNR             = 0.0f;
    uint32_t packetsWithRSSI    = 0;
    uint32_t consecutiveLoss    = 0;
    uint32_t maxConsecutiveLoss = 0;
    float    lossRate           = 0.0f;
} stats;

struct LinkQuality {
    uint32_t totalPackets  = 0;
    uint32_t lostPackets   = 0;
    float    packetLossRate = 0.0f;
    unsigned long lastUpdate = 0;
} linkQuality;

// ====================== VARIABLES GLOBALES ==================
static RadioEvents_t RadioEvents;
char rxpacket[BUFFER_SIZE];

volatile bool     ackPending  = false;
volatile uint16_t ackSequence = 0;
volatile bool     lora_idle   = true;

uint16_t      lastProcessedSeq = 0;
SensorData    sensorData;
unsigned long lastPacketTime   = 0;

// Buffer deduplicación
uint16_t recentSeqs[SEQ_BUFFER_SIZE];
uint8_t  seqIndex = 0;

// ─────────────────────────────────────────────────────────────
// mqttFlag / mqttPayloadBuf:
// OnRxDone (ISR) escribe el payload aquí y activa la bandera.
// mqttTask (Core 1) lee la bandera y publica.
// Evita llamar funciones de red dentro del callback de radio.
// ─────────────────────────────────────────────────────────────
volatile bool mqttFlag = false;
char          mqttPayloadBuf[192];

SSD1306Wire factory_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// ====================== UTILIDADES ==========================
void VextON() { pinMode(Vext, OUTPUT); digitalWrite(Vext, LOW); }

void printSeparator(char c = '=', int len = 60) {
    for (int i = 0; i < len; i++) Serial.print(c);
    Serial.println();
}

void printHeader(const char* title) {
    Serial.println();
    printSeparator();
    Serial.printf(">> %s\n", title);
    printSeparator();
}

// ====================== DEDUPLICACIÓN =======================
bool isDuplicateSeq(uint16_t seq) {
    for (int i = 0; i < SEQ_BUFFER_SIZE; i++)
        if (recentSeqs[i] == seq) return true;
    return false;
}

void addToSeqBuffer(uint16_t seq) {
    recentSeqs[seqIndex] = seq;
    seqIndex = (seqIndex + 1) % SEQ_BUFFER_SIZE;
}

// ====================== CALIDAD DE ENLACE ===================
void updateLinkQuality(bool received, int16_t rssi = 0, int8_t snr = 0) {
    linkQuality.totalPackets++;
    if (!received) {
        linkQuality.lostPackets++;
        stats.consecutiveLoss++;
        if (stats.consecutiveLoss > stats.maxConsecutiveLoss)
            stats.maxConsecutiveLoss = stats.consecutiveLoss;
    } else {
        float n = (float)stats.packetsWithRSSI;
        stats.avgRSSI = (n > 0) ? (stats.avgRSSI * n + rssi) / (n + 1) : (float)rssi;
        stats.avgSNR  = (n > 0) ? (stats.avgSNR  * n + snr)  / (n + 1) : (float)snr;
        stats.packetsWithRSSI++;
        stats.consecutiveLoss = 0;
    }
    linkQuality.packetLossRate = (linkQuality.totalPackets > 0)
        ? (float)linkQuality.lostPackets / linkQuality.totalPackets : 0.0f;
    stats.lossRate         = linkQuality.packetLossRate;
    linkQuality.lastUpdate = millis();
}

// ====================== PANTALLA ============================
void drawDisplay() {
    factory_display.clear();
    factory_display.setTextAlignment(TEXT_ALIGN_LEFT);
    factory_display.setFont(ArialMT_Plain_10);

    // Header: estado WiFi y MQTT
    factory_display.drawString(0, 0, "RX AMBIENTAL");
    factory_display.drawString(75, 0,
        WiFi.status() == WL_CONNECTED ? "W:OK" : "W:--");
    factory_display.drawString(104, 0,
        mqttClient.connected() ? "M:OK" : "M:--");

    bool fresh = sensorData.valid && (millis() - lastPacketTime < DATA_TIMEOUT_MS);
    if (fresh) {
        char buf[32];
        snprintf(buf, sizeof(buf), "M2:%d M5:%d", sensorData.mq2, sensorData.mq135);
        factory_display.drawString(0, 12, buf);
        snprintf(buf, sizeof(buf), "DUST:%d", sensorData.dust);
        factory_display.drawString(0, 24, buf);
        snprintf(buf, sizeof(buf), "R:%d S:%d", sensorData.rssi, sensorData.snr);
        factory_display.drawString(0, 36, buf);
        snprintf(buf, sizeof(buf), "P:%lu L:%.1f%%",
            stats.packetsUnique, stats.lossRate * 100.0f);
        factory_display.drawString(0, 48, buf);
        snprintf(buf, sizeof(buf), "S:%u", sensorData.seq);
        factory_display.drawString(80, 48, buf);
    } else {
        factory_display.setFont(ArialMT_Plain_16);
        factory_display.drawString(20, 20, "...");
        factory_display.setFont(ArialMT_Plain_10);
        factory_display.drawString(0, 48, "Esperando datos...");
    }
    factory_display.display();
}

// ====================== STATUS SERIAL =======================
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
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        WiFi.disconnect(true);
        Serial.println("fallo");
    }
    Serial.println("[WiFi] Sin conexion disponible");
    return false;
}

// ====================== MQTT ================================
// ─────────────────────────────────────────────────────────────
// reconnectMQTT — NO usa delay() bloqueante.
// Solo intenta UNA vez por llamada; si falla, retorna false.
// El reintento lo gestiona el temporizador en mqttTask.
// ─────────────────────────────────────────────────────────────
bool reconnectMQTT() {
    if (mqttClient.connected()) return true;
    if (WiFi.status() != WL_CONNECTED) return false;

    char clientId[32];
    snprintf(clientId, sizeof(clientId),
             "ESP32_RX_%04X", (uint16_t)esp_random());

    Serial.printf("[MQTT] Conectando como %s a %s:%d... ",
                  clientId, MQTT_SERVER, MQTT_PORT);

    if (mqttClient.connect(clientId)) {
        Serial.println("OK");
        return true;
    }

    Serial.printf("fallo rc=%d\n", mqttClient.state());
    return false;
}

// ====================== CALLBACKS LoRa ======================
void OnTxDone() {
    lora_idle = true;
    Radio.Rx(0);
}

void OnTxTimeout() {
    Serial.println("[RX] Timeout enviando ACK");
    lora_idle = true;
    Radio.Rx(0);
}

// ─────────────────────────────────────────────────────────────
// OnRxDone — contexto ISR: NO llamar funciones de red aquí.
// Solo: parsear, actualizar estado, armar payload en buffer,
// activar bandera mqttFlag para que mqttTask lo publique.
// ─────────────────────────────────────────────────────────────
void OnRxDone(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr) {
    if (size >= BUFFER_SIZE) size = BUFFER_SIZE - 1;
    memcpy(rxpacket, payload, size);
    rxpacket[size] = '\0';

    Serial.printf("[RX] RAW: %s | RSSI:%d SNR:%d\n", rxpacket, rssi, snr);

    // ── 1. Parsear ───────────────────────────────────────────
    uint16_t seq;
    int mq2, mq135, dust;
    char id[32];
    if (sscanf(rxpacket,
            "{\"seq\":%hu,\"id\":\"%31[^\"]\",\"mq2\":%d,\"mq135\":%d,\"dust\":%d}",
            &seq, id, &mq2, &mq135, &dust) != 5)
    {
        Serial.println("[RX] Paquete mal formado");
        stats.packetsLost++;
        updateLinkQuality(false);
        Radio.Rx(0);
        return;
    }

    // ── 2. Verificar origen ──────────────────────────────────
    if (strcmp(id, EXPECTED_NODE_ID) != 0) {
        Serial.printf("[RX] NodeID desconocido: %s\n", id);
        Radio.Rx(0);
        return;
    }

    // ── 3. Contadores físicos ────────────────────────────────
    stats.packetsReceived++;
    stats.packetsWithRSSI++;
    if (rssi < stats.minRSSI) stats.minRSSI = rssi;
    if (rssi > stats.maxRSSI) stats.maxRSSI = rssi;
    if (snr  < stats.minSNR)  stats.minSNR  = snr;
    if (snr  > stats.maxSNR)  stats.maxSNR  = snr;

    // ── 4. Detectar brechas ──────────────────────────────────
    if (lastProcessedSeq > 0 && seq > (uint16_t)(lastProcessedSeq + 1)) {
        uint16_t lost = seq - lastProcessedSeq - 1;
        stats.packetsLost += lost;
        Serial.printf("[RX] Brecha: esperaba %u llego %u (%u perdidos)\n",
            lastProcessedSeq + 1, seq, lost);
        for (uint16_t k = 0; k < lost; k++) updateLinkQuality(false);
    }

    // ── 5. Duplicado ─────────────────────────────────────────
    if (isDuplicateSeq(seq)) {
        stats.packetsDuplicated++;
        Serial.printf("[RX] Duplicado SEQ=%u — ACK cortesia\n", seq);
        ackSequence = seq;
        ackPending  = true;
        Radio.Rx(0);
        return;
    }

    // ── 6. Paquete nuevo válido ──────────────────────────────
    stats.packetsUnique++;
    stats.lastSeq    = seq;
    lastProcessedSeq = seq;
    addToSeqBuffer(seq);

    sensorData.mq2   = mq2;  sensorData.mq135 = mq135;
    sensorData.dust  = dust; sensorData.seq   = seq;
    sensorData.rssi  = rssi; sensorData.snr   = snr;
    sensorData.valid = true; sensorData.timestamp = millis();
    strncpy(sensorData.nodeId, id, sizeof(sensorData.nodeId) - 1);
    sensorData.nodeId[sizeof(sensorData.nodeId) - 1] = '\0';
    lastPacketTime = millis();
    updateLinkQuality(true, rssi, snr);

    Serial.printf("[RX] SEQ:%u MQ2:%d MQ135:%d DUST:%d\n", seq, mq2, mq135, dust);

    // ✅ Preparar payload MQTT en buffer compartido
    // mqttTask lo publicará de forma segura fuera del ISR
    snprintf(mqttPayloadBuf, sizeof(mqttPayloadBuf),
        "{\"seq\":%u,\"nodeId\":\"%s\","
        "\"mq2\":%d,\"mq135\":%d,\"dust\":%d,"
        "\"rssi\":%d,\"snr\":%d}",
        seq, id, mq2, mq135, dust, rssi, snr);
    mqttFlag = true;  // señal para mqttTask

    // ── Programar ACK ────────────────────────────────────────
    ackSequence = seq;
    ackPending  = true;
    Radio.Rx(0);
}

// ====================== TAREA LoRa (CORE 0) =================
// Solo maneja radio: IRQ, ACKs y status. SIN red.
void loraTask(void* pv) {
    Radio.Rx(0);
    Serial.println("[TASK] LoRa task iniciada (Core 0)");

    unsigned long lastAckTime    = 0;
    unsigned long lastStatusTime = 0;

    while (1) {
        Radio.IrqProcess();

        // ── Enviar ACK pendiente ─────────────────────────────
        if (ackPending && lora_idle && (millis() - lastAckTime > 50)) {
            uint16_t seqToAck = ackSequence;
            ackPending  = false;
            lora_idle   = false;
            lastAckTime = millis();
            stats.ackSent++;

            char ackBuf[32];
            snprintf(ackBuf, sizeof(ackBuf), "{\"ack\":%u}", seqToAck);
            Serial.printf("[RX] Enviando ACK: %s\n", ackBuf);
            Radio.Send((uint8_t*)ackBuf, strlen(ackBuf));
        }

        // ── Status periódico ─────────────────────────────────
        if (millis() - lastStatusTime > 5000) {
            lastStatusTime = millis();
            printCompactStatusRX();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ====================== TAREA WiFi/MQTT (CORE 1) ============
// ─────────────────────────────────────────────────────────────
// Separar WiFi/MQTT de LoRa es crítico:
//  - WiFi usa mucho stack y puede tardar segundos
//  - delay() en reconnect bloquearía Radio.IrqProcess()
//  - mqttClient.loop() necesita ejecutarse frecuentemente
//
// Esta tarea corre en Core 1 y nunca llama a Radio.
// ─────────────────────────────────────────────────────────────
void mqttTask(void* pv) {
    Serial.println("[TASK] MQTT task iniciada (Core 1)");

    // ── Configurar servidor una sola vez ─────────────────────
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);

    // ── Conexión inicial WiFi ────────────────────────────────
    tryConnectWiFi();

    unsigned long lastReconnectAttempt = 0;
    const unsigned long RECONNECT_INTERVAL = 10000; // reintentar cada 10s

    while (1) {
        // ── Verificar WiFi ───────────────────────────────────
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("[WiFi] Conexion perdida, reconectando...");
            tryConnectWiFi();
            lastReconnectAttempt = millis(); // reiniciar timer MQTT
        }

        // ── Verificar/reconectar MQTT (sin delay bloqueante) ─
        if (!mqttClient.connected()) {
            if (millis() - lastReconnectAttempt >= RECONNECT_INTERVAL) {
                lastReconnectAttempt = millis();
                reconnectMQTT();
            }
        }

        // ── Loop MQTT (keepalive, callbacks entrantes) ───────
        if (mqttClient.connected())
            mqttClient.loop();

        // ── Publicar si hay dato nuevo de LoRa ───────────────
        if (mqttFlag && mqttClient.connected()) {
            mqttFlag = false; // limpiar bandera primero
            bool ok = mqttClient.publish(MQTT_TOPIC, mqttPayloadBuf);
            Serial.printf("[MQTT] Publish %s: %s\n",
                          MQTT_TOPIC, ok ? "OK" : "FALLO");
        }

        // ── Actualizar display ───────────────────────────────
        drawDisplay();

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ====================== SETUP ===============================
void setup() {
    Serial.begin(115200);
    delay(1000);
    printHeader("SISTEMA AMBIENTAL RX - INICIANDO");

    VextON();
    delay(100);
    Wire.begin(SDA_OLED, SCL_OLED);
    delay(50);

    factory_display.init();
    factory_display.clear();
    factory_display.setFont(ArialMT_Plain_10);
    factory_display.drawString(0, 10, "WiFi LoRa 32 V3");
    factory_display.drawString(0, 25, "Receptor Ambiental");
    factory_display.drawString(0, 40, "Inicializando...");
    factory_display.display();
    delay(2000);

    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

    RadioEvents.RxDone    = OnRxDone;
    RadioEvents.TxDone    = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;

    Radio.Init(&RadioEvents);
    Radio.SetPublicNetwork(false);
    Radio.SetSyncWord(LORA_SYNC_WORD);
    Radio.SetChannel(RF_FREQUENCY);

    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0,
        LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE,
        LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD,
        true, 0, 0, LORA_IQ_INVERSION, 3000);

    Radio.SetRxConfig(MODEM_LORA,
        LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE,
        0, LORA_PREAMBLE_LENGTH, LORA_SYMBOL_TIMEOUT,
        LORA_FIX_LENGTH_PAYLOAD, 0, true, 0, 0, LORA_IQ_INVERSION, true);

    memset(recentSeqs, 0xFF, sizeof(recentSeqs));

    linkQuality.totalPackets   = 0;
    linkQuality.lostPackets    = 0;
    linkQuality.packetLossRate = 0.0f;
    linkQuality.lastUpdate     = millis();

    // LoRa en Core 0 (prioridad alta), WiFi/MQTT en Core 1
    xTaskCreatePinnedToCore(loraTask, "LoRaTask", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(mqttTask, "MqttTask", 8192, NULL, 1, NULL, 1);

    Serial.println("[SETUP] Sistema receptor listo\n");
}

void loop() { vTaskDelay(portMAX_DELAY); }