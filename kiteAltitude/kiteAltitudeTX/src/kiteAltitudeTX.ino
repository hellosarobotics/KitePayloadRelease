// kiteAltitudeTX.ino
// Nodo TX: da montare sull'aquilone. Legge altitudine e temperatura dal BME280, filtra
// l'altitudine (stessa catena spike-guard + mediana-3 + EMA usata in kitepayloadrelease-ESP32-C3)
// e trasmette entrambe via LoRa (SX1276) alla stazione di terra (kiteAltitudeRX).
//
// Nodo headless: nessun Wi-Fi/BLE/WebServer per contenere peso e consumo a batteria.
// Si calibra per convenzione: va acceso con l'aquilone ancora a terra, nel punto di lancio
// (quota zero = punto di accensione). La pressione di riferimento al livello del mare è
// fissa perché si cancella nella sottrazione altitudine-baseAltitude: qui conta solo il
// delta relativo, non la quota assoluta.
//
// Intervallo di campionamento/trasmissione parametrico: vedi SAMPLE_INTERVAL_MS sotto.
// Cambiandolo, spike-guard/max-step ed EMA si riscalano automaticamente per restare corretti
// (vedi kiteAltitude/README.md, sezione "Intervallo di campionamento" per le conseguenze).

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <RadioLib.h>
#include "PacketFormat.h"

// Definire questa macro (o passarla come build flag) per bench-test senza volare:
// sostituisce la lettura reale del BME280 con una rampa sintetica ciclica
// (salita 0->20m in 20s, plateau 10s, discesa a ~3m/s in 7s, plateau 5s).
// #define SIMULATE_ALTITUDE

// ======= Pin BME280 (I2C) =======
#define I2C_SDA 8
#define I2C_SCL 9

// ======= Pin sensing batteria (partitore 100k/100k dal + della LiPo) =======
#define BATTERY_ADC_PIN 3

// ======= Intervallo di campionamento/trasmissione — L'UNICA COSTANTE DA CAMBIARE =======
// Ogni SAMPLE_INTERVAL_MS: si legge il BME280, si filtra l'altitudine e si invia il pacchetto LoRa.
// Le costanti di filtro qui sotto sono espresse come RATEI (m/s) e vengono moltiplicate per
// l'intervallo, quindi restano corrette a qualunque valore si scelga (1000 = comportamento
// identico alla versione originale a 1 Hz). Aumentarlo riduce il duty-cycle radio e il consumo
// batteria, ma rende più lenta la reazione dell'allarme di discesa sull'RX (che calcola il
// rateo di discesa sul tempo reale tra i pacchetti: più pacchetti radi = stima più tardiva).
static const uint32_t SAMPLE_INTERVAL_MS = 2000;

// ======= Costanti filtro altitudine (identiche in spirito a kitepayloadrelease-ESP32-C3.ino,
// ma espresse come rateo così restano valide indipendentemente da SAMPLE_INTERVAL_MS) =======
static const float SEALEVEL_PRESSURE_HPA = 1013.25f; // fisso: si cancella in altitude-baseAltitude
static const float MAX_CLIMB_RATE_MPS = 2.0f;   // passo massimo plausibile per campione, in m/s
static const float SPIKE_RATE_MPS = 5.0f;       // sopra questo rateo il campione è uno spike da limitare
static const uint8_t MEDIAN_N = 3;              // finestra mediana: N * SAMPLE_INTERVAL_MS di durata reale
static const float ALT_TIME_CONSTANT_S = 1.4427f; // costante di tempo EMA (= 1/ln(2), dà alpha=0.5 a 1000ms)

static const float MAX_STEP_M = MAX_CLIMB_RATE_MPS * (SAMPLE_INTERVAL_MS / 1000.0f);
static const float SPIKE_THRESHOLD_M = SPIKE_RATE_MPS * (SAMPLE_INTERVAL_MS / 1000.0f);
// alpha derivato dalla costante di tempo: la "reattività" reale del filtro non cambia
// cambiando l'intervallo (a differenza di un alpha fisso, che diventerebbe più lento in
// proporzione all'intervallo).
static const float ALT_ALPHA = 1.0f - expf(-(SAMPLE_INTERVAL_MS / 1000.0f) / ALT_TIME_CONSTANT_S);

Adafruit_BME280 bme;
bool bmeAvailable = false;
float baseAltitude = 0;

float g_relAltitude = 0;
bool altFilterInit = false;
float relAltWindow[MEDIAN_N];
uint8_t relAltIdx = 0;
bool relAltFilled = false;

uint32_t lastBmeRead = 0;
uint16_t seq = 0;

SX1276 radio = new Module(LORA_CS_PIN, LORA_DIO0_PIN, LORA_RST_PIN, RADIOLIB_NC);

// ---------------------- Filtro (stessa logica del progetto esistente) ----------------------
float medianOfWindow() {
  uint8_t n = relAltFilled ? MEDIAN_N : relAltIdx;
  if (n == 0) return g_relAltitude;
  float tmp[MEDIAN_N];
  for (uint8_t i = 0; i < n; i++) tmp[i] = relAltWindow[i];
  for (uint8_t i = 0; i < n; i++) {
    uint8_t minIdx = i;
    for (uint8_t j = i + 1; j < n; j++) if (tmp[j] < tmp[minIdx]) minIdx = j;
    float t = tmp[i]; tmp[i] = tmp[minIdx]; tmp[minIdx] = t;
  }
  if (n & 1) return tmp[n / 2];
  return 0.5f * (tmp[n / 2 - 1] + tmp[n / 2]);
}

void pushWindow(float v) {
  relAltWindow[relAltIdx++] = v;
  if (relAltIdx >= MEDIAN_N) { relAltIdx = 0; relAltFilled = true; }
}

#ifdef SIMULATE_ALTITUDE
float simulatedRelAltitude(uint32_t nowMs) {
  const float CLIMB_M = 20.0f;
  const uint32_t T_CLIMB = 20000, T_PLATEAU1 = 10000, T_DESCEND = 7000, T_PLATEAU2 = 5000;
  const uint32_t CYCLE = T_CLIMB + T_PLATEAU1 + T_DESCEND + T_PLATEAU2;
  uint32_t t = nowMs % CYCLE;
  if (t < T_CLIMB) return CLIMB_M * (t / (float)T_CLIMB);
  t -= T_CLIMB;
  if (t < T_PLATEAU1) return CLIMB_M;
  t -= T_PLATEAU1;
  if (t < T_DESCEND) return CLIMB_M * (1.0f - t / (float)T_DESCEND);
  return 0.0f;
}
#endif

uint16_t readBatteryMillivolts() {
  uint32_t mv = analogReadMilliVolts(BATTERY_ADC_PIN);
  return (uint16_t)(mv * 2); // partitore 1:1
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(I2C_SDA, I2C_SCL);
  bmeAvailable = bme.begin(0x76, &Wire) || bme.begin(0x77, &Wire);
  Serial.println(bmeAvailable ? "BME280 trovato!" : "ATTENZIONE: BME280 non trovato!");

  baseAltitude = bmeAvailable ? bme.readAltitude(SEALEVEL_PRESSURE_HPA) : 0;

  for (uint8_t i = 0; i < MEDIAN_N; i++) relAltWindow[i] = 0;
  relAltIdx = 0; relAltFilled = true; altFilterInit = true; g_relAltitude = 0;

  analogReadResolution(12);

  SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_CS_PIN);
  int state = radio.begin(LORA_FREQUENCY_MHZ, LORA_BANDWIDTH_KHZ, LORA_SPREADING_FACTOR,
                           LORA_CODING_RATE, LORA_SYNC_WORD, LORA_TX_POWER_DBM, LORA_PREAMBLE_LENGTH);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("Errore inizializzazione radio SX1276: %d\n", state);
  }
  radio.setCRC(true);
  radio.sleep();

  Serial.println("kiteAltitudeTX pronto.");
}

void loop() {
  uint32_t now = millis();

#ifndef SIMULATE_ALTITUDE
  if (!bmeAvailable) { delay(1); return; } // niente sensore, niente telemetria da inviare
#endif

  if (now - lastBmeRead >= SAMPLE_INTERVAL_MS) {
    lastBmeRead = now;

    float altitude, temperature;
#ifdef SIMULATE_ALTITUDE
    altitude = baseAltitude + simulatedRelAltitude(now);
    temperature = 20.0f;
#else
    altitude = bme.readAltitude(SEALEVEL_PRESSURE_HPA);
    temperature = bme.readTemperature(); // nessun filtro: la temperatura varia lentamente ed è solo un readout
#endif
    float relAltRaw = altitude - baseAltitude;

    // spike-guard
    float candidate = relAltRaw;
    if (altFilterInit) {
      float delta = candidate - g_relAltitude;
      if (fabs(delta) > SPIKE_THRESHOLD_M) {
        candidate = g_relAltitude + (delta > 0 ? MAX_STEP_M : -MAX_STEP_M);
      }
    }

    // mediana su 3 campioni
    pushWindow(candidate);
    float med = medianOfWindow();

    // EMA finale
    if (!altFilterInit) {
      g_relAltitude = med;
      altFilterInit = true;
    } else {
      g_relAltitude = (1.0f - ALT_ALPHA) * g_relAltitude + ALT_ALPHA * med;
    }

    seq++;
    TelemetryPacket pkt;
    pkt.magic = PACKET_MAGIC;
    pkt.seq = seq;
    pkt.relAltitude_mm = (int32_t)lroundf(g_relAltitude * 1000.0f);
    pkt.temperature_c10 = (int16_t)lroundf(temperature * 10.0f);
    pkt.batteryMv = readBatteryMillivolts();
    pkt.txUptimeMs = now;

    int state = radio.transmit((uint8_t*)&pkt, sizeof(pkt));
    radio.sleep(); // risparmio energetico tra una trasmissione e la successiva

    Serial.printf("seq=%u relAlt=%.2fm temp=%.1fC batt=%umV tx=%s\n",
                  pkt.seq, g_relAltitude, temperature, pkt.batteryMv,
                  state == RADIOLIB_ERR_NONE ? "ok" : "ERR");
  }

  delay(1);
}
