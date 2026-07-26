#pragma once
// PacketFormat.h — DEVE ESSERE IDENTICO in kiteAltitudeTX/ e kiteAltitudeRX/.
// Se lo modifichi in una cartella, copia la stessa modifica anche nell'altra:
// TX e RX devono avere la stessa struct e gli stessi parametri radio o non si capiscono.

#include <stdint.h>

// ======= Pin SPI / LoRa (SX1276) — identici su TX e RX =======
#define LORA_SCK_PIN   4
#define LORA_MISO_PIN  5
#define LORA_MOSI_PIN  6
#define LORA_CS_PIN    7
#define LORA_RST_PIN   10
#define LORA_DIO0_PIN  1

// ======= Parametri radio (EU868, sotto-banda h1.5: 869.40-869.65 MHz, duty cycle fino al 10%) =======
// Verificare la normativa italiana/ERC attuale prima di trasmettere davvero.
#define LORA_FREQUENCY_MHZ     869.525f
#define LORA_BANDWIDTH_KHZ     125.0f
#define LORA_SPREADING_FACTOR  7
#define LORA_CODING_RATE       5      // 4/5
#define LORA_SYNC_WORD         0x12   // sync word privata (diversa da 0x34, quella pubblica LoRaWAN)
#define LORA_TX_POWER_DBM      14
#define LORA_PREAMBLE_LENGTH   8

// ======= Pacchetto telemetria (15 byte) =======
#define PACKET_MAGIC 0xA1

struct __attribute__((packed)) TelemetryPacket {
  uint8_t  magic;            // sempre PACKET_MAGIC: scarta pacchetti di altri protocolli/versioni
  uint16_t seq;               // contatore incrementale, wrap a 65535
  int32_t  relAltitude_mm;    // altitudine relativa filtrata, millimetri
  int16_t  temperature_c10;   // temperatura BME280, decimi di grado C (es. 235 = 23.5°C), letta senza filtro
  uint16_t batteryMv;         // tensione batteria TX, millivolt (0 = non disponibile)
  uint32_t txUptimeMs;        // millis() del TX al momento dell'invio (diagnostica)
};
// dimensione totale: 1+2+4+2+2+4 = 15 byte
