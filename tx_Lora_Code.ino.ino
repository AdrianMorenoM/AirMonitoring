// ============================================================
// LoRa Heltec V3 — TRANSMISOR AMBIENTAL
// Este archivo es el lado TRANSMISOR del sistema.
// Implementa el protocolo Stop-and-Wait sobre LoRa half-duplex:
//   enviar 1 paquete → esperar ACK → enviar el siguiente.
//
// Usa dos núcleos del ESP32:
//   - Core 0: maneja la radio LoRa (envíos, ACKs, timeouts)
//   - Core 1: lee los tres sensores de calidad de aire
// ============================================================
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include <stdarg.h>

// ====================== CONFIGURACIÓN PINES =================
// Pines del ESP32 a los que están conectados los sensores.
constexpr uint8_t PIN_MQ2       = 1;  // Salida analógica sensor MQ-2
constexpr uint8_t PIN_MQ135     = 2;  // Salida analógica sensor MQ-135
constexpr uint8_t PIN_GP2Y_LED  = 7;  // Pin que controla el LED infrarrojo del sensor de polvo GP2Y
constexpr uint8_t PIN_GP2Y_VO   = 6;  // Salida analógica del sensor de polvo GP2Y

// ====================== CONSTANTES SENSORES =================
// Tiempos de control del sensor GP2Y1010:
//   El LED se enciende brevemente, se espera 280 µs, se lee el ADC,
//   luego 40 µs más y se apaga. Esto es lo que indica su datasheet.
constexpr uint16_t GP2Y_DELAY_BEFORE_READ_US = 280;
constexpr uint16_t GP2Y_DELAY_AFTER_READ_US  = 40;
constexpr uint16_t SAMPLE_INTERVAL_MS        = 2000; // Leer sensores y enviar cada 2 segundos

// Parámetros ADC y cálculo de RS para los MQ (actualmente se usan valores raw)
constexpr float ADC_MAX  = 4095.0f; // Resolución 12 bits (0–4095)
constexpr float VREF     = 3.3f;    // Tensión de referencia del ADC
constexpr float RL_VALUE = 10.0f;   // Resistencia de carga en kΩ (en el circuito del sensor)
constexpr float R0_MQ2   = 10.0f;   // ⚠️ Resistencia base en aire limpio — debe calibrarse
constexpr float R0_MQ135 = 10.0f;   // ⚠️ Ídem para MQ-135

// ====================== CONFIGURACIÓN LoRa ==================
// Deben coincidir exactamente con los del receptor (RX).
#define RF_FREQUENCY              915000000 // 915 MHz (banda ISM América)
#define TX_OUTPUT_POWER           14        // 14 dBm — mayor potencia que el RX (5 dBm)
#define LORA_BANDWIDTH            0         // BW 125 kHz
#define LORA_SPREADING_FACTOR     7         // SF7: buena velocidad en distancias cortas/medias
#define LORA_CODINGRATE           1         // 4/5
#define LORA_PREAMBLE_LENGTH      8
#define LORA_FIX_LENGTH_PAYLOAD   false
#define LORA_IQ_INVERSION         false
#define LORA_SYMBOL_TIMEOUT       5
#define LORA_SYNC_WORD            0x2A      // "Contraseña" de red (igual que en RX)

// ====================== PARÁMETROS PROTOCOLO ================
#define BUFFER_SIZE       256
#define NODE_ID           "TX_AMBIENTAL_01"  // Identificador de este nodo (el RX lo verifica)
#define MAX_RETRIES       3                  // Si no llega ACK en 3 intentos, descarta el paquete
#define ACK_TIMEOUT_MS    2000               // Espera máxima por ACK: 2 segundos
#define DATA_TIMEOUT_MS   3000               // Tiempo máximo sin datos válidos de sensores
#define MAX_INFLIGHT      10                 // Máximo de paquetes en vuelo simultáneos
#define MAX_QUEUE_SIZE    10                 // Tamaño máximo de la cola de espera

// ─────────────────────────────────────────────────────────────
// Protocolo Stop-and-Wait (cwnd = 1):
// LoRa es half-duplex: no puede recibir y transmitir al mismo
// tiempo. Si TX envía varios paquetes antes de recibir el ACK
// del primero, el RX colisiona al intentar responder.
// Con cwnd=1 se asegura: TX envía → espera ACK → TX envía.
// ssthresh=1 evita que el cwnd crezca automáticamente.
// ─────────────────────────────────────────────────────────────
uint32_t cwnd     = 1; // Ventana de congestión: solo 1 paquete en vuelo
uint32_t ssthresh = 1; // Umbral slow-start: no crecer por encima de 1

// ─────────────────────────────────────────────────────────────
// RX_GUARD_MS: pausa obligatoria después de que TX termina de
// enviar, para que el receptor tenga tiempo de:
//   1. Procesar el paquete recibido
//   2. Preparar y enviar su ACK
// Sin esta pausa, TX podría ocupar el canal justo cuando RX
// intenta transmitir el ACK. Estimado: ~100ms de aire + overhead.
// ─────────────────────────────────────────────────────────────
#define RX_GUARD_MS  200

// ====================== ESTRUCTURAS =========================

// Representa un paquete durante su ciclo de vida (encolado → enviado → ACK).
struct PacketData {
    uint16_t      seq;          // Número de secuencia único (incrementa con cada envío)
    uint32_t      timestamp;    // Cuándo se creó el paquete (ms)
    char          data[BUFFER_SIZE]; // JSON con los datos del sensor
    uint8_t       retries;      // Cuántas retransmisiones se han hecho
    bool          acked;        // true si ya fue confirmado por el RX
    unsigned long lastAttempt;  // Timestamp del último intento de envío (para calcular RTT)
};

// Últimos valores leídos de los sensores (compartido entre tareas vía mutex).
struct SensorData {
    int  mq2       = 0;
    int  mq135     = 0;
    int  dust      = 0;
    unsigned long timestamp = 0;
    bool valid     = false; // false = no hay lecturas confiables aún
};

// Contadores de rendimiento del sistema de transmisión.
struct Stats {
    uint32_t packetsSent     = 0; // Total de envíos (incluyendo retransmisiones)
    uint32_t packetsAcked    = 0; // Paquetes confirmados exitosamente
    uint32_t retransmissions = 0; // Cuántas veces se reenvió un paquete por timeout
    uint32_t timeouts        = 0; // Paquetes descartados por agotar MAX_RETRIES
    uint32_t queueOverflows  = 0; // Veces que la cola estaba llena y se descartó un paquete
    uint32_t minRTT          = 0xFFFFFFFF; // RTT mínimo observado (ms)
    uint32_t maxRTT          = 0;          // RTT máximo observado (ms)
    uint32_t totalRTT        = 0;          // Suma acumulada para calcular el promedio
    uint32_t rttSamples      = 0;          // Número de muestras de RTT tomadas
} stats;

// ====================== VARIABLES GLOBALES ==================
static RadioEvents_t RadioEvents; // Callbacks de la radio LoRa
char txpacket[BUFFER_SIZE];       // Buffer temporal para el paquete a transmitir

// Array de paquetes "en vuelo" (enviados pero sin ACK aún).
// Con cwnd=1 siempre tendrá como máximo 1 elemento activo.
PacketData    inflight[MAX_INFLIGHT];
uint8_t       inflightCount  = 0; // Cuántos paquetes hay actualmente en vuelo
uint16_t      sequenceNumber = 0; // Contador global de secuencia (incrementa por cada paquete nuevo)

// Cola circular FIFO de paquetes pendientes de enviar.
PacketData    pendingPackets[MAX_QUEUE_SIZE];
uint8_t       queueHead = 0, queueTail = 0, queueCount = 0;

// lora_idle: true = radio libre para transmitir; false = transmitiendo.
// txDoneTime: momento en que terminó el último TX (para aplicar RX_GUARD).
// 'volatile' necesario porque los callbacks de radio corren en contexto de ISR.
volatile bool          lora_idle   = true;
volatile unsigned long txDoneTime  = 0;

SensorData        sensorData;   // Datos actuales de los sensores
SemaphoreHandle_t sensorMutex;  // Mutex para acceso seguro a sensorData entre tareas
unsigned long     lastDataTime = 0; // Timestamp de la última lectura válida de sensores
unsigned long     lastSendTime = 0; // Timestamp del último paquete encolado

SSD1306Wire factory_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// ====================== UTILIDADES ==========================

void VextON() { pinMode(Vext, OUTPUT); digitalWrite(Vext, LOW); }

// Convierte un valor del ADC (0–4095) a voltaje (0.0–3.3 V).
float adcToVoltage(int adc) { return (adc / ADC_MAX) * VREF; }

// Calcula la resistencia RS del sensor MQ dado su voltaje de salida.
// Fórmula del datasheet: RS = ((VREF - Vo) / Vo) * RL
float calculateRS(float v) {
    if (v <= 0.0f) return 0.0f;
    return ((VREF - v) / v) * RL_VALUE;
}

// Logger genérico con formato: [ROL][EVENTO] mensaje
// Ejemplo: [TX][ACK] ✅ Seq:42 RTT:120ms
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
// Actualiza la pantalla OLED con el estado actual.
// Llamada periódicamente desde sensorTask (~2 Hz).
void drawDisplay() {
    factory_display.clear();
    factory_display.setTextAlignment(TEXT_ALIGN_LEFT);
    factory_display.setFont(ArialMT_Plain_10);

    // Encabezado: título + estado de la radio (OK=libre / TX=transmitiendo)
    factory_display.drawString(6, 4, "ENV MONITOR");
    factory_display.drawString(92, 4, lora_idle ? "OK" : "TX");
    factory_display.drawHorizontalLine(0, 14, 128);

    char buf[20];
    if (sensorData.valid) {
        // Fila de valores: MQ2 | MQ135 | DUST
        snprintf(buf, sizeof(buf), "%4d", sensorData.mq2);
        factory_display.drawString(8, 20, buf);
        snprintf(buf, sizeof(buf), "%4d", sensorData.mq135);
        factory_display.drawString(48, 20, buf);
        snprintf(buf, sizeof(buf), "%4d", sensorData.dust);
        factory_display.drawString(88, 20, buf);
        // Etiquetas debajo de los valores
        factory_display.drawString(12, 32, "AIR");  // MQ-2: gases inflamables
        factory_display.drawString(52, 32, "VOC");  // MQ-135: compuestos orgánicos volátiles
        factory_display.drawString(96, 32, "PM");   // GP2Y: material particulado
    } else {
        factory_display.drawString(36, 24, "NO DATA");
    }

    // Pie: paquetes en vuelo | en cola | total ACKed
    factory_display.drawHorizontalLine(0, 48, 128);
    snprintf(buf, sizeof(buf), "IN:%d Q:%d ACK:%lu",
             inflightCount, queueCount, stats.packetsAcked);
    factory_display.drawString(4, 54, buf);
    factory_display.display();
}

// ====================== STATS ===============================

// Actualiza las estadísticas de RTT (Round-Trip Time) cada vez que llega un ACK.
// RTT = tiempo desde el último envío del paquete hasta la recepción del ACK.
void updateRTTStats(uint32_t rtt) {
    stats.rttSamples++;
    stats.totalRTT += rtt;
    if (rtt < stats.minRTT) stats.minRTT = rtt;
    if (rtt > stats.maxRTT) stats.maxRTT = rtt;
}

// Imprime un resumen completo del estado del transmisor en el Monitor Serial.
void printSystemStatus() {
    Serial.println("📡 [ESTADO TRANSMISOR]");
    // Porcentaje de éxito: paquetes ACKed / enviados
    Serial.printf("✉️ Enviados: %lu | ✅ ACKed: %lu (%.1f%%)\n", stats.packetsSent, stats.packetsAcked,
    stats.packetsSent ? (stats.packetsAcked*100.0f/stats.packetsSent):0);
    Serial.printf("🔁 Retransmisiones: %lu | ⏱️ Timeouts: %lu | 📥 Q overflow: %lu\n",
    stats.retransmissions, stats.timeouts, stats.queueOverflows);
    // RTT mínimo / promedio / máximo en ms
    if (stats.rttSamples > 0)
        Serial.printf("⏲️ RTT min/avg/max: %lu/%lu/%lu ms\n", stats.minRTT, stats.totalRTT / stats.rttSamples, stats.maxRTT);
        Serial.printf("CWND:%lu | INFLIGHT:%d | QUEUE:%d\n", cwnd, inflightCount, queueCount);
    printSeparator();
}

// ====================== COLA CIRCULAR =======================

// Agrega un paquete al final de la cola FIFO.
// Retorna false si la cola está llena (desbordamiento).
bool queuePacket(PacketData* pkt) {
    if (queueCount >= MAX_QUEUE_SIZE) { stats.queueOverflows++; return false; }
    pendingPackets[queueTail] = *pkt;
    queueTail = (queueTail + 1) % MAX_QUEUE_SIZE; // Avance circular
    queueCount++;
    return true;
}

// Extrae el paquete más antiguo del frente de la cola FIFO.
// Retorna false si la cola está vacía.
bool dequeuePacket(PacketData* pkt) {
    if (queueCount == 0) return false;
    *pkt = pendingPackets[queueHead];
    queueHead = (queueHead + 1) % MAX_QUEUE_SIZE; // Avance circular
    queueCount--;
    return true;
}

// ====================== CONTROL CONGESTIÓN ==================
// Con cwnd=1 fijo, solo se llama cuando un paquete falla (timeout).
// En caso de fallo: ssthresh = cwnd/2, cwnd = 1 (TCP Tahoe simplificado).
// El incremento en éxito está intencionalmente deshabilitado
// para mantener siempre stop-and-wait puro.
void handleCongestionControl(bool success) {
    if (!success) {
        ssthresh = max((uint32_t)1, cwnd / 2);
        cwnd = 1;
        logEvent("TX", "CONG", "cwnd->1 ssthresh->%lu", ssthresh);
    }
    // No incrementar cwnd en éxito: mantener stop-and-wait estricto
}

// ====================== ELIMINAR DEL INFLIGHT ===============
// Elimina el paquete en la posición 'i' del array inflight
// desplazando los elementos posteriores hacia la izquierda.
void removeInflight(int i) {
    for (int j = i; j < inflightCount - 1; j++)
        inflight[j] = inflight[j + 1];
    inflightCount--;
}

// ====================== CALLBACKS LoRa ======================

// Llamado cuando la radio termina de transmitir un paquete exitosamente.
// Registra el momento para el RX_GUARD y devuelve la radio a modo RX.
void OnTxDone() {
    txDoneTime = millis(); // Guardar cuándo terminó el TX (para aplicar la pausa RX_GUARD)
    lora_idle  = true;
    Radio.Rx(0);           // Pasar a escuchar el ACK del receptor
    // ⚠️ No loguear aquí: añade latencia al callback y puede afectar el timing
}

// Llamado si la radio excede el tiempo límite al transmitir.
void OnTxTimeout() {
    logEvent("TX", "TXTIMEOUT", "Radio timeout");
    Radio.Sleep();        // Apagar radio antes de reiniciar
    txDoneTime = millis();
    lora_idle  = true;
    Radio.Rx(0);
}

// ─────────────────────────────────────────────────────────────
// OnRxDone — contexto quasi-ISR.
// Solo procesa ACKs: parsea el JSON {"ack": N}, busca el paquete
// correspondiente en inflight, calcula el RTT y lo elimina.
// ─────────────────────────────────────────────────────────────
void OnRxDone(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr) {
    char buf[BUFFER_SIZE];
    if (size >= BUFFER_SIZE) size = BUFFER_SIZE - 1;
    memcpy(buf, payload, size);
    buf[size] = '\0';

    // Parsear el ACK: formato esperado → {"ack":N}
    uint16_t ackSeq = 0;
    if (sscanf(buf, "{\"ack\":%hu}", &ackSeq) != 1) {
        logEvent("TX", "RXBAD", "Formato invalido: %s", buf);
        Radio.Rx(0);
        return; // Ignorar mensajes que no sean ACKs válidos
    }

    // Buscar el paquete ACKed en la lista de paquetes en vuelo
    bool found = false;
    for (int i = 0; i < inflightCount; i++) {
        if (inflight[i].seq != ackSeq) continue;

        // Calcular RTT: tiempo desde el último intento de envío hasta ahora
        uint32_t rtt = (uint32_t)(millis() - inflight[i].lastAttempt);
        updateRTTStats(rtt);
        stats.packetsAcked++;

        logEvent("TX", "ACK", "✅ Seq:%u RTT:%lums Try:%d | RSSI:%d",
                 ackSeq, rtt, inflight[i].retries, rssi);

        removeInflight(i); // Liberar el slot en inflight
        found = true;
        break;
    }

    if (!found)
        // ACK llegó tarde (ya se descartó por timeout) o es duplicado
        logEvent("TX", "ACKDUP", "Seq:%u tardio/dup", ackSeq);

    Radio.Rx(0); // Volver a escuchar
}

// ====================== HARDWARE ============================

// Configura los pines de los sensores y la resolución del ADC.
void initHardware() {
    pinMode(PIN_GP2Y_LED, OUTPUT);
    digitalWrite(PIN_GP2Y_LED, HIGH); // LED del GP2Y apagado por defecto (activo LOW)
    analogReadResolution(12);         // ADC de 12 bits → 0–4095
    // Atenuación 11dB para soportar el rango completo 0–3.3V en cada pin analógico
    analogSetPinAttenuation(PIN_MQ2,     ADC_11db);
    analogSetPinAttenuation(PIN_MQ135,   ADC_11db);
    analogSetPinAttenuation(PIN_GP2Y_VO, ADC_11db);
}

// Lee el sensor de polvo GP2Y1010AU0F siguiendo el protocolo exacto del datasheet:
// encender LED → esperar 280µs → leer ADC → esperar 40µs → apagar LED.
// El LED IR ilumina las partículas; el fotodetector mide la luz dispersada.
int readDustSensor() {
    digitalWrite(PIN_GP2Y_LED, LOW);              // Encender LED IR
    delayMicroseconds(GP2Y_DELAY_BEFORE_READ_US); // Esperar 280 µs (estabilización)
    int v = analogRead(PIN_GP2Y_VO);              // Leer voltaje del fotodetector
    delayMicroseconds(GP2Y_DELAY_AFTER_READ_US);  // Esperar 40 µs adicionales
    digitalWrite(PIN_GP2Y_LED, HIGH);             // Apagar LED
    return v;
}

// Lee los tres sensores y actualiza sensorData de forma segura con mutex.
// Solo actualiza si todas las lecturas están dentro del rango ADC válido (0–4095).
void readSensors(int& mq2, int& mq135, int& gp2y) {
    mq2   = analogRead(PIN_MQ2);
    mq135 = analogRead(PIN_MQ135);
    gp2y  = readDustSensor();

    // Validación básica: descarta lecturas fuera del rango del ADC
    bool valid = (mq2   >= 0 && mq2   <= 4095) &&
                 (mq135 >= 0 && mq135 <= 4095) &&
                 (gp2y  >= 0 && gp2y  <= 4095);

    // Proteger el acceso a sensorData con mutex (también lo lee loraTask)
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(100))) {
        if (valid) {
            sensorData.mq2       = mq2;
            sensorData.mq135     = mq135;
            sensorData.dust      = gp2y;
            sensorData.timestamp = millis();
            sensorData.valid     = true;
            lastDataTime         = millis();
        }
        xSemaphoreGive(sensorMutex); // Liberar mutex obligatoriamente
    }
    if (valid)
        Serial.printf("🌡️ [SENSORES] MQ2:%d ⚠️ MQ135:%d 🏭 DUST:%d\n", mq2, mq135, gp2y);
}

// ====================== TAREA SENSORES (CORE 1) =============
// Lee periódicamente los sensores y actualiza la pantalla OLED.
// Corre en Core 1 cada SAMPLE_INTERVAL_MS (2 segundos).
// El acceso a sensorData está protegido por sensorMutex.
void sensorTask(void* pv) {
    int mq2, mq135, gp2y;
    Serial.println("[TASK] Sensor task iniciada (Core 1)");
    while (1) {
        readSensors(mq2, mq135, gp2y);

        // Si llevan demasiado tiempo sin datos válidos, marcar como inválido
        // para que loraTask no envíe datos desactualizados
        if (millis() - lastDataTime > DATA_TIMEOUT_MS * 2) {
            if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(100))) {
                sensorData.valid = false;
                xSemaphoreGive(sensorMutex);
            }
            logEvent("SENSOR", "TIMEOUT", "Sin datos validos");
        }

        drawDisplay();                              // Refrescar pantalla OLED
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS)); // Ceder CPU y esperar 2 s
    }
}

// ====================== TAREA LoRa (CORE 0) =================
// Corazón del protocolo Stop-and-Wait. Corre en Core 0 con
// prioridad alta (2) para no perder eventos de la radio.
// Maneja 5 responsabilidades en cada ciclo de 10 ms:
//   1. Leer datos del sensor (snapshot seguro)
//   2. Encolar un paquete si hay datos frescos y es tiempo
//   3. Enviar desde la cola respetando cwnd y RX_GUARD
//   4. Detectar y manejar timeouts por falta de ACK
//   5. Imprimir estado periódico en Serial
void loraTask(void* pv) {
    Serial.println("[TASK] LoRa task iniciada (Core 0)");

    while (1) {
        Radio.IrqProcess(); // Procesar interrupciones pendientes de la radio (obligatorio)

        // ── 1. Snapshot seguro de sensores ───────────────────
        // Se copia sensorData a una variable local para no
        // mantener el mutex ocupado durante el resto del ciclo.
        SensorData cur;
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10))) {
            cur = sensorData;
            xSemaphoreGive(sensorMutex);
        }

        // ── 2. Crear y encolar un nuevo paquete ──────────────
        // Condiciones para encolar:
        //   - Los datos del sensor son frescos (llegaron hace < DATA_TIMEOUT_MS)
        //   - Han pasado al menos SAMPLE_INTERVAL_MS desde el último envío
        //   - Hay espacio en la cola + inflight combinados
        bool dataFresh  = cur.valid && (millis() - lastDataTime < DATA_TIMEOUT_MS);
        bool timeToSend = (millis() - lastSendTime) >= SAMPLE_INTERVAL_MS;
        bool spaceAvail = (queueCount + inflightCount) < MAX_QUEUE_SIZE;

        if (dataFresh && timeToSend && spaceAvail) {
            PacketData pkt;
            pkt.seq         = ++sequenceNumber; // Incrementar antes de usar
            pkt.timestamp   = millis();
            pkt.retries     = 0;
            pkt.acked       = false;
            pkt.lastAttempt = 0;

            // Formato JSON del paquete: el RX espera exactamente este formato
            snprintf(pkt.data, BUFFER_SIZE,
                "{\"seq\":%u,\"id\":\"%s\",\"mq2\":%d,\"mq135\":%d,\"dust\":%d}",
                pkt.seq, NODE_ID, cur.mq2, cur.mq135, cur.dust);

            if (queuePacket(&pkt))
                lastSendTime = millis();
        }

        // ── 3. Enviar respetando ventana (cwnd) y RX_GUARD ──
        // guardExpired: han pasado al menos RX_GUARD_MS desde el último TX,
        // dando tiempo al receptor de enviar su ACK sin colisionar.
        bool guardExpired = (millis() - txDoneTime) >= RX_GUARD_MS;

        // El while permite enviar hasta cwnd paquetes de una vez,
        // pero con cwnd=1 solo envía uno por ciclo.
        while (lora_idle &&
               guardExpired &&
               inflightCount < (int)cwnd &&
               queueCount > 0)
        {
            PacketData pkt;
            if (!dequeuePacket(&pkt)) break;

            pkt.lastAttempt           = millis(); // Para calcular RTT cuando llegue el ACK
            inflight[inflightCount++] = pkt;      // Mover a la lista de "en vuelo"
            stats.packetsSent++;

            snprintf(txpacket, BUFFER_SIZE, "%s", pkt.data);
            Radio.Send((uint8_t*)txpacket, strlen(txpacket)); // ← Transmitir por radio
            lora_idle = false; // Marcar radio como ocupada hasta OnTxDone

            logEvent("TX", "SEND", "✉️ Seq:%u | InFlight:%d | Q:%d",
                     pkt.seq, inflightCount, queueCount);
        }

        // ── 4. Manejar timeouts de ACK ───────────────────────
        // Recorre todos los paquetes en vuelo y verifica si
        // superaron el tiempo ACK_TIMEOUT_MS sin respuesta.
        for (int i = 0; i < inflightCount; i++) {
            if ((millis() - inflight[i].lastAttempt) <= ACK_TIMEOUT_MS) continue;

            if (inflight[i].retries < MAX_RETRIES) {
                // Retransmitir: incrementar contador y reenviar el mismo dato
                inflight[i].retries++;
                inflight[i].lastAttempt = millis(); // Reiniciar el cronómetro de timeout
                stats.retransmissions++;

                snprintf(txpacket, BUFFER_SIZE, "%s", inflight[i].data);
                Radio.Send((uint8_t*)txpacket, strlen(txpacket));
                lora_idle = false;

                logEvent("TX", "RETRY", "🔁 Seq:%u Try:%d/%d",
                         inflight[i].seq, inflight[i].retries, MAX_RETRIES);
            } else {
                // Paquete descartado: agotó todos los reintentos sin ACK
                stats.timeouts++;
                logEvent("TX", "DROP", "❌ Seq:%u agotado %d reintentos",
                         inflight[i].seq, MAX_RETRIES);

                handleCongestionControl(false); // Reducir cwnd por fallo
                removeInflight(i);              // Eliminar del inflight
                i--; // Corregir índice porque el array se compactó
            }
        }

        // ── 5. Status periódico en Serial (cada 5 s) ─────────
        static unsigned long lastStatus = 0;
        if (millis() - lastStatus > 5000) {
            lastStatus = millis();
            printSystemStatus();
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Ceder CPU 10 ms al scheduler de FreeRTOS
    }
}

// ====================== SETUP ===============================
// Se ejecuta una sola vez al encender. Configura todo el hardware,
// inicializa la radio y lanza las dos tareas de FreeRTOS.
void setup() {
    Serial.begin(115200);
    delay(1000);
    printHeader("SISTEMA AMBIENTAL TX - INICIANDO");

    initHardware(); // Configurar pines y ADC de los sensores

    VextON();            // Encender alimentación de la pantalla OLED
    delay(100);
    Wire.begin(SDA_OLED, SCL_OLED); // Inicializar bus I2C
    delay(50);

    // Pantalla de bienvenida mientras el sistema arranca
    factory_display.init();
    factory_display.clear();
    factory_display.setFont(ArialMT_Plain_10);
    factory_display.drawString(0, 10, "WiFi LoRa 32 V3");
    factory_display.drawString(0, 25, "Sistema Ambiental TX");
    factory_display.drawString(0, 40, "Inicializando...");
    factory_display.display();
    delay(2000);

    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE); // Inicializar MCU Heltec

    // Asignar los callbacks de la radio LoRa
    RadioEvents.TxDone    = OnTxDone;    // Transmisión completada
    RadioEvents.TxTimeout = OnTxTimeout; // Transmisión tardó demasiado
    RadioEvents.RxDone    = OnRxDone;    // ACK recibido del receptor

    Radio.Init(&RadioEvents);
    Radio.SetPublicNetwork(false);
    Radio.SetSyncWord(LORA_SYNC_WORD);
    Radio.SetChannel(RF_FREQUENCY);

    // Configurar parámetros de transmisión (datos de sensores)
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0,
        LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE,
        LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD,
        true, 0, 0, LORA_IQ_INVERSION, 3000);

    // Configurar parámetros de recepción (para recibir ACKs del RX)
    Radio.SetRxConfig(MODEM_LORA,
        LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE,
        0, LORA_PREAMBLE_LENGTH, LORA_SYMBOL_TIMEOUT,
        LORA_FIX_LENGTH_PAYLOAD, 0, true, 0, 0, LORA_IQ_INVERSION, true);

    Radio.Rx(0); // Poner la radio en modo recepción desde el inicio
    Serial.println("[SETUP] Radio LoRa inicializada");

    // Crear el mutex ANTES de lanzar las tareas (ambas lo usan)
    sensorMutex = xSemaphoreCreateMutex();
    configASSERT(sensorMutex); // Detiene el sistema si falla la creación

    // Lanzar tareas FreeRTOS:
    //   sensorTask → Core 1, stack 4 KB, prioridad 1 (normal)
    //   loraTask   → Core 0, stack 4 KB, prioridad 2 (alta)
    xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(loraTask,   "LoRaTask",   4096, NULL, 2, NULL, 0);

    Serial.println("[SETUP] Sistema transmisor listo\n");
    printSystemStatus(); // Estado inicial (todo en 0)
}

// loop() vacío: todo el trabajo está en las tareas FreeRTOS.
// portMAX_DELAY suspende el loop indefinidamente sin quemar CPU.
void loop() { vTaskDelay(portMAX_DELAY); }
