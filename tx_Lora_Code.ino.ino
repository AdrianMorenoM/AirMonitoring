// ============================================================
// LoRa Heltec V3 — TRANSMISOR AMBIENTAL
// Stop-and-Wait confiable sobre canal half-duplex
// ============================================================
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include <stdarg.h>

// ====================== CONFIGURACIÓN PINES =================
constexpr uint8_t PIN_MQ2       = 1;
constexpr uint8_t PIN_MQ135     = 2;
constexpr uint8_t PIN_GP2Y_LED  = 7;
constexpr uint8_t PIN_GP2Y_VO   = 6;

// ====================== CONSTANTES SENSORES =================
constexpr uint16_t GP2Y_DELAY_BEFORE_READ_US = 280;
constexpr uint16_t GP2Y_DELAY_AFTER_READ_US  = 40;
constexpr uint16_t SAMPLE_INTERVAL_MS        = 2000;

constexpr float ADC_MAX  = 4095.0f;
constexpr float VREF     = 3.3f;
constexpr float RL_VALUE = 10.0f;
constexpr float R0_MQ2   = 10.0f;  // ⚠️ Calibrar
constexpr float R0_MQ135 = 10.0f;  // ⚠️ Calibrar

// ====================== CONFIGURACIÓN LoRa ==================
#define RF_FREQUENCY              915000000
#define TX_OUTPUT_POWER           14
#define LORA_BANDWIDTH            0
#define LORA_SPREADING_FACTOR     7
#define LORA_CODINGRATE           1
#define LORA_PREAMBLE_LENGTH      8
#define LORA_FIX_LENGTH_PAYLOAD   false
#define LORA_IQ_INVERSION         false
#define LORA_SYMBOL_TIMEOUT       5
#define LORA_SYNC_WORD            0x2A

// ====================== PARÁMETROS PROTOCOLO ================
#define BUFFER_SIZE       256
#define NODE_ID           "TX_AMBIENTAL_01"
#define MAX_RETRIES       3
#define ACK_TIMEOUT_MS    2000   // Reducido: RSSI excelente, ACK llega rápido
#define DATA_TIMEOUT_MS   3000
#define MAX_INFLIGHT      10
#define MAX_QUEUE_SIZE    10

// ─────────────────────────────────────────────────────────────
// CWND = 1 (stop-and-wait)
// En canal LoRa half-duplex, enviar múltiples paquetes antes
// de recibir el ACK del primero causa colisiones inevitables:
// el RX no puede recibir mientras transmite el ACK anterior.
// Con cwnd=1 se garantiza: TX envía → espera ACK → TX envía.
// ssthresh se mantiene en 1 para no crecer automáticamente.
// Si en el futuro se desea pipeline, incrementar con cuidado.
// ─────────────────────────────────────────────────────────────
uint32_t cwnd     = 1;
uint32_t ssthresh = 1;

// ─────────────────────────────────────────────────────────────
// RX_GUARD_MS: tiempo mínimo que espera el TX después de
// OnTxDone antes de poder enviar el siguiente paquete.
// Permite al RX terminar de procesar + transmitir su ACK.
// Estimado: tiempo aire ACK (~50ms SF7 BW125) + overhead ~50ms
// ─────────────────────────────────────────────────────────────
#define RX_GUARD_MS  200

// ====================== ESTRUCTURAS =========================
struct PacketData {
    uint16_t      seq;
    uint32_t      timestamp;
    char          data[BUFFER_SIZE];
    uint8_t       retries;
    bool          acked;
    unsigned long lastAttempt;
};

struct SensorData {
    int  mq2       = 0;
    int  mq135     = 0;
    int  dust      = 0;
    unsigned long timestamp = 0;
    bool valid     = false;
};

struct Stats {
    uint32_t packetsSent     = 0;
    uint32_t packetsAcked    = 0;
    uint32_t retransmissions = 0;
    uint32_t timeouts        = 0;
    uint32_t queueOverflows  = 0;
    uint32_t minRTT          = 0xFFFFFFFF;
    uint32_t maxRTT          = 0;
    uint32_t totalRTT        = 0;
    uint32_t rttSamples      = 0;
} stats;

// ====================== VARIABLES GLOBALES ==================
static RadioEvents_t RadioEvents;
char txpacket[BUFFER_SIZE];

PacketData    inflight[MAX_INFLIGHT];
uint8_t       inflightCount  = 0;
uint16_t      sequenceNumber = 0;

PacketData    pendingPackets[MAX_QUEUE_SIZE];
uint8_t       queueHead = 0, queueTail = 0, queueCount = 0;

// lora_idle: radio disponible para enviar
// txDoneTime: timestamp del último TxDone (para RX_GUARD)
volatile bool          lora_idle   = true;
volatile unsigned long txDoneTime  = 0;

SensorData        sensorData;
SemaphoreHandle_t sensorMutex;
unsigned long     lastDataTime = 0;
unsigned long     lastSendTime = 0;

SSD1306Wire factory_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// ====================== UTILIDADES ==========================
void VextON() { pinMode(Vext, OUTPUT); digitalWrite(Vext, LOW); }

float adcToVoltage(int adc) { return (adc / ADC_MAX) * VREF; }

float calculateRS(float v) {
    if (v <= 0.0f) return 0.0f;
    return ((VREF - v) / v) * RL_VALUE;
}

void logEvent(const char* role, const char* event, const char* fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.printf("[%s][%s] %s\n", role, event, buf);
}

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

// ====================== PANTALLA ============================
void drawDisplay() {
    factory_display.clear();
    factory_display.setTextAlignment(TEXT_ALIGN_LEFT);
    factory_display.setFont(ArialMT_Plain_10);

    factory_display.drawString(6, 4, "ENV MONITOR");
    factory_display.drawString(92, 4, lora_idle ? "OK" : "TX");
    factory_display.drawHorizontalLine(0, 14, 128);

    char buf[20];
    if (sensorData.valid) {
        snprintf(buf, sizeof(buf), "%4d", sensorData.mq2);
        factory_display.drawString(8, 20, buf);
        snprintf(buf, sizeof(buf), "%4d", sensorData.mq135);
        factory_display.drawString(48, 20, buf);
        snprintf(buf, sizeof(buf), "%4d", sensorData.dust);
        factory_display.drawString(88, 20, buf);
        factory_display.drawString(12, 32, "AIR");
        factory_display.drawString(52, 32, "VOC");
        factory_display.drawString(96, 32, "PM");
    } else {
        factory_display.drawString(36, 24, "NO DATA");
    }

    factory_display.drawHorizontalLine(0, 48, 128);
    snprintf(buf, sizeof(buf), "IN:%d Q:%d ACK:%lu",
             inflightCount, queueCount, stats.packetsAcked);
    factory_display.drawString(4, 54, buf);
    factory_display.display();
}

// ====================== STATS ===============================
void updateRTTStats(uint32_t rtt) {
    stats.rttSamples++;
    stats.totalRTT += rtt;
    if (rtt < stats.minRTT) stats.minRTT = rtt;
    if (rtt > stats.maxRTT) stats.maxRTT = rtt;
}

void printSystemStatus() {
    Serial.println("📡 [ESTADO TRANSMISOR]");
    Serial.printf("✉️ Enviados: %lu | ✅ ACKed: %lu (%.1f%%)\n", stats.packetsSent, stats.packetsAcked,
    stats.packetsSent ? (stats.packetsAcked*100.0f/stats.packetsSent):0);
    Serial.printf("🔁 Retransmisiones: %lu | ⏱️ Timeouts: %lu | 📥 Q overflow: %lu\n",
    stats.retransmissions, stats.timeouts, stats.queueOverflows);
    if (stats.rttSamples > 0)
        Serial.printf("⏲️ RTT min/avg/max: %lu/%lu/%lu ms\n", stats.minRTT, stats.totalRTT / stats.rttSamples, stats.maxRTT);
        Serial.printf("CWND:%lu | INFLIGHT:%d | QUEUE:%d\n", cwnd, inflightCount, queueCount);
    printSeparator();
}

// ====================== COLA CIRCULAR =======================
bool queuePacket(PacketData* pkt) {
    if (queueCount >= MAX_QUEUE_SIZE) { stats.queueOverflows++; return false; }
    pendingPackets[queueTail] = *pkt;
    queueTail = (queueTail + 1) % MAX_QUEUE_SIZE;
    queueCount++;
    return true;
}

bool dequeuePacket(PacketData* pkt) {
    if (queueCount == 0) return false;
    *pkt = pendingPackets[queueHead];
    queueHead = (queueHead + 1) % MAX_QUEUE_SIZE;
    queueCount--;
    return true;
}

// ====================== CONTROL CONGESTIÓN ==================
// Con cwnd=1 fijo no se llama a este en éxito normalmente,
// pero se mantiene para futura escalabilidad.
void handleCongestionControl(bool success) {
    if (!success) {
        ssthresh = max((uint32_t)1, cwnd / 2);
        cwnd = 1;
        logEvent("TX", "CONG", "cwnd->1 ssthresh->%lu", ssthresh);
    }
    // No incrementar cwnd en éxito: mantenemos stop-and-wait
}

// ====================== ELIMINAR DEL INFLIGHT ===============
void removeInflight(int i) {
    for (int j = i; j < inflightCount - 1; j++)
        inflight[j] = inflight[j + 1];
    inflightCount--;
}

// ====================== CALLBACKS LoRa ======================
void OnTxDone() {
    // Registrar tiempo de fin de TX para aplicar RX_GUARD
    txDoneTime = millis();
    lora_idle  = true;
    Radio.Rx(0);
    // NO loguear aquí para no añadir latencia al callback
}

void OnTxTimeout() {
    logEvent("TX", "TXTIMEOUT", "Radio timeout");
    Radio.Sleep();
    txDoneTime = millis();
    lora_idle  = true;
    Radio.Rx(0);
}

// ─────────────────────────────────────────────────────────────
// OnRxDone — procesa exclusivamente ACKs
// Busca el seq en inflight, calcula RTT, elimina y ajusta cwnd.
// ─────────────────────────────────────────────────────────────
void OnRxDone(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr) {
    char buf[BUFFER_SIZE];
    if (size >= BUFFER_SIZE) size = BUFFER_SIZE - 1;
    memcpy(buf, payload, size);
    buf[size] = '\0';

    uint16_t ackSeq = 0;
    if (sscanf(buf, "{\"ack\":%hu}", &ackSeq) != 1) {
        logEvent("TX", "RXBAD", "Formato invalido: %s", buf);
        Radio.Rx(0);
        return;
    }

    bool found = false;
    for (int i = 0; i < inflightCount; i++) {
        if (inflight[i].seq != ackSeq) continue;

        uint32_t rtt = (uint32_t)(millis() - inflight[i].lastAttempt);
        updateRTTStats(rtt);
        stats.packetsAcked++;

        logEvent("TX", "ACK", "✅ Seq:%u RTT:%lums Try:%d | RSSI:%d", ackSeq, rtt, inflight[i].retries, rssi);

        removeInflight(i);
        found = true;
        break;
    }

    if (!found)
        logEvent("TX", "ACKDUP", "Seq:%u tardio/dup", ackSeq);

    Radio.Rx(0);
}

// ====================== HARDWARE ============================
void initHardware() {
    pinMode(PIN_GP2Y_LED, OUTPUT);
    digitalWrite(PIN_GP2Y_LED, HIGH);
    analogReadResolution(12);
    analogSetPinAttenuation(PIN_MQ2,     ADC_11db);
    analogSetPinAttenuation(PIN_MQ135,   ADC_11db);
    analogSetPinAttenuation(PIN_GP2Y_VO, ADC_11db);
}

int readDustSensor() {
    digitalWrite(PIN_GP2Y_LED, LOW);
    delayMicroseconds(GP2Y_DELAY_BEFORE_READ_US);
    int v = analogRead(PIN_GP2Y_VO);
    delayMicroseconds(GP2Y_DELAY_AFTER_READ_US);
    digitalWrite(PIN_GP2Y_LED, HIGH);
    return v;
}

void readSensors(int& mq2, int& mq135, int& gp2y) {
    mq2   = analogRead(PIN_MQ2);
    mq135 = analogRead(PIN_MQ135);
    gp2y  = readDustSensor();

    bool valid = (mq2   >= 0 && mq2   <= 4095) &&
                 (mq135 >= 0 && mq135 <= 4095) &&
                 (gp2y  >= 0 && gp2y  <= 4095);

    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(100))) {
        if (valid) {
            sensorData.mq2       = mq2;
            sensorData.mq135     = mq135;
            sensorData.dust      = gp2y;
            sensorData.timestamp = millis();
            sensorData.valid     = true;
            lastDataTime         = millis();
        }
        xSemaphoreGive(sensorMutex);
    }
    if (valid)
        Serial.printf("🌡️ [SENSORES] MQ2:%d ⚠️ MQ135:%d 🏭 DUST:%d\n", mq2, mq135, gp2y);
}

// ====================== TAREA SENSORES (CORE 1) =============
void sensorTask(void* pv) {
    int mq2, mq135, gp2y;
    Serial.println("[TASK] Sensor task iniciada (Core 1)");
    while (1) {
        readSensors(mq2, mq135, gp2y);

        if (millis() - lastDataTime > DATA_TIMEOUT_MS * 2) {
            if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(100))) {
                sensorData.valid = false;
                xSemaphoreGive(sensorMutex);
            }
            logEvent("SENSOR", "TIMEOUT", "Sin datos validos");
        }

        drawDisplay();
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
    }
}

// ====================== TAREA LoRa (CORE 0) =================
void loraTask(void* pv) {
    Serial.println("[TASK] LoRa task iniciada (Core 0)");

    while (1) {
        Radio.IrqProcess();

        // ── 1. Snapshot seguro de sensores ───────────────────
        SensorData cur;
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10))) {
            cur = sensorData;
            xSemaphoreGive(sensorMutex);
        }

        // ── 2. Encolar un paquete por intervalo ──────────────
        bool dataFresh  = cur.valid && (millis() - lastDataTime < DATA_TIMEOUT_MS);
        bool timeToSend = (millis() - lastSendTime) >= SAMPLE_INTERVAL_MS;
        bool spaceAvail = (queueCount + inflightCount) < MAX_QUEUE_SIZE;

        if (dataFresh && timeToSend && spaceAvail) {
            PacketData pkt;
            pkt.seq         = ++sequenceNumber;
            pkt.timestamp   = millis();
            pkt.retries     = 0;
            pkt.acked       = false;
            pkt.lastAttempt = 0;

            snprintf(pkt.data, BUFFER_SIZE,
                "{\"seq\":%u,\"id\":\"%s\",\"mq2\":%d,\"mq135\":%d,\"dust\":%d}",
                pkt.seq, NODE_ID, cur.mq2, cur.mq135, cur.dust);

            if (queuePacket(&pkt))
                lastSendTime = millis();
        }

        // ── 3. Enviar respetando ventana Y guarda post-TX ────
        // RX_GUARD_MS: dar tiempo al receptor para procesar y
        // enviar su ACK antes de ocupar el canal de nuevo.
        bool guardExpired = (millis() - txDoneTime) >= RX_GUARD_MS;

        while (lora_idle &&
               guardExpired &&
               inflightCount < (int)cwnd &&
               queueCount > 0)
        {
            PacketData pkt;
            if (!dequeuePacket(&pkt)) break;

            pkt.lastAttempt           = millis();
            inflight[inflightCount++] = pkt;
            stats.packetsSent++;

            snprintf(txpacket, BUFFER_SIZE, "%s", pkt.data);
            Radio.Send((uint8_t*)txpacket, strlen(txpacket));
            lora_idle = false;

            logEvent("TX", "SEND", "✉️ Seq:%u | InFlight:%d | Q:%d", pkt.seq, inflightCount, queueCount);
        }

        // ── 4. Timeouts individuales ─────────────────────────
        for (int i = 0; i < inflightCount; i++) {
            if ((millis() - inflight[i].lastAttempt) <= ACK_TIMEOUT_MS) continue;

            if (inflight[i].retries < MAX_RETRIES) {
                inflight[i].retries++;
                inflight[i].lastAttempt = millis();
                stats.retransmissions++;

                snprintf(txpacket, BUFFER_SIZE, "%s", inflight[i].data);
                Radio.Send((uint8_t*)txpacket, strlen(txpacket));
                lora_idle = false;

                logEvent("TX", "RETRY", "🔁 Seq:%u Try:%d/%d", inflight[i].seq, inflight[i].retries, MAX_RETRIES);
            } else {
                stats.timeouts++;
                logEvent("TX", "DROP", "❌ Seq:%u agotado %d reintentos", inflight[i].seq, MAX_RETRIES);

                handleCongestionControl(false);
                removeInflight(i);
                i--;
            }
        }

        // ── 5. Status periódico ──────────────────────────────
        static unsigned long lastStatus = 0;
        if (millis() - lastStatus > 5000) {
            lastStatus = millis();
            printSystemStatus();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ====================== SETUP ===============================
void setup() {
    Serial.begin(115200);
    delay(1000);
    printHeader("SISTEMA AMBIENTAL TX - INICIANDO");

    initHardware();

    VextON();
    delay(100);
    Wire.begin(SDA_OLED, SCL_OLED);
    delay(50);

    factory_display.init();
    factory_display.clear();
    factory_display.setFont(ArialMT_Plain_10);
    factory_display.drawString(0, 10, "WiFi LoRa 32 V3");
    factory_display.drawString(0, 25, "Sistema Ambiental TX");
    factory_display.drawString(0, 40, "Inicializando...");
    factory_display.display();
    delay(2000);

    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

    RadioEvents.TxDone    = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone    = OnRxDone;

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

    Radio.Rx(0);
    Serial.println("[SETUP] Radio LoRa inicializada");

    sensorMutex = xSemaphoreCreateMutex();
    configASSERT(sensorMutex);

    xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(loraTask,   "LoRaTask",   4096, NULL, 2, NULL, 0);

    Serial.println("[SETUP] Sistema transmisor listo\n");
    printSystemStatus();
}

void loop() { vTaskDelay(portMAX_DELAY); }