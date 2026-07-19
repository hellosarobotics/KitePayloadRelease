# KitePayloadRelease

Sistema per il rilascio automatico (o manuale) di un payload agganciato a un aquilone, basato su una scheda **ESP8266**, un sensore di pressione/temperatura/umidità **BME280** e un **servomotore** che sgancia il carico. Il dispositivo espone una pagina web di controllo e monitoraggio via Wi-Fi.

## Struttura del repository

| Cartella | Contenuto |
|---|---|
| `kitepayloadrelease/` | Firmware principale per **ESP8266** (Arduino, `.ino`) |
| `kitepayloadrelease-ESP32-C3/` | Variante del firmware per **ESP32-C3** |
| `kiteAltitude/` | Sistema di telemetria altimetrica via **LoRa** (nodo TX su aquilone + RX a terra) — vedi `kiteAltitude/README.md` |
| `fritzing/` | Schema elettrico/cablaggio (progetto Fritzing) |
| `3DModel/` | Modello 3D del meccanismo di sgancio a servo |
| `cardboardBox/` | Disegno (SVG) della scatola che racchiude l'elettronica |

## Cosa fa il firmware (`kitepayloadrelease/kitepayloadrelease.ino`)

Il microcontrollore, una volta acceso:

1. **Crea un access point Wi-Fi** proprio (SSID `KiteRelease`, password `12345678`) e avvia un server web sulla porta 80: non serve una rete esterna, ci si collega direttamente al dispositivo con smartphone/PC.
2. **Legge periodicamente il sensore BME280** (1 Hz) per temperatura, umidità, pressione e altitudine assoluta (calcolata dalla pressione tramite la pressione di riferimento al livello del mare).
3. **Calcola un'altitudine relativa** rispetto a una quota di base (rilevata all'avvio o al reset), che è il valore usato per decidere lo sgancio.
4. **Filtra il segnale di altitudine** per eliminare rumore e letture anomale del sensore, con una catena a più stadi:
   - oversampling e filtro IIR hardware del BME280;
   - *spike-guard*: se un campione si allontana troppo (>5 m) dal valore filtrato precedente, il salto viene limitato a un passo massimo (2 m/campione);
   - **mediana** sugli ultimi 3 campioni, per scartare outlier isolati;
   - **media esponenziale (EMA)** finale per uno smoothing morbido.
5. **Sgancia automaticamente il payload** (comandando il servo) quando l'altitudine relativa filtrata supera una soglia impostabile, applicando un'**isteresi temporale** di 1 secondo (la soglia deve restare superata per almeno 1 s consecutivo) per evitare falsi positivi dovuti a spike momentanei.
6. **Permette lo sgancio manuale** in qualsiasi momento tramite un pulsante sull'interfaccia web.
7. **Salva in EEPROM** la pressione di riferimento al livello del mare e la quota di sgancio impostate, così i parametri sopravvivono a un riavvio/spegnimento.

### Interfaccia web

La pagina principale (`/`) mostra in tempo reale (aggiornamento ogni 2 s via `fetch` su `/data`):

- Temperatura, umidità, pressione, altitudine assoluta e relativa, altitudine massima raggiunta;
- Stato del carico (**rilasciato** / **non rilasciato**), con indicatore colorato (verde/rosso);
- Quota relativa a cui è avvenuto l'ultimo sgancio (manuale o automatico);
- Avviso se il sensore BME280 non viene rilevato all'avvio.

Dalla stessa pagina è possibile:

- impostare la **pressione al livello del mare** (hPa), usata per calibrare l'altitudine assoluta;
- impostare la **quota di sgancio automatico** (m) sopra la quota di base;
- forzare lo **sgancio manuale** (`RILASCIA`);
- **resettare** quota di base, altitudine massima e stato di sgancio (`Reset Altitudine`), ricalibrando lo zero sulla posizione corrente.

### Endpoint HTTP esposti

| Endpoint | Metodo | Funzione |
|---|---|---|
| `/` | GET | Pagina di controllo/monitoraggio |
| `/data` | GET | Dati correnti in formato JSON (temperatura, umidità, pressione, altitudini, stato sgancio) |
| `/release` | GET | Sgancio manuale immediato |
| `/setPressure` | POST | Imposta e salva la pressione di riferimento al livello del mare |
| `/setReleaseAltitude` | POST | Imposta e salva la quota di sgancio automatico |
| `/resetRelease` | GET | Ricalibra la quota di base e azzera stato/filtri |

### Meccanismo di sgancio

Il rilascio è puramente meccanico: il servo (collegato al pin `D5`) viene portato a 180°, mantenuto per 500 ms e poi riportato a 0°, azionando il meccanismo di sgancio descritto nel modello 3D (`3DModel/`).

### Comportamento in assenza del sensore

Se il BME280 non viene rilevato all'avvio (indirizzi I²C `0x76`/`0x77`), il firmware continua a funzionare: l'access point e il server web restano attivi e lo sgancio manuale resta disponibile, ma la logica di sgancio automatico basata sull'altitudine è disabilitata e l'interfaccia mostra un avviso.

> La variante in `kitepayloadrelease-ESP32-C3/` adatta lo stesso firmware alla scheda ESP32-C3.

## Telemetria altimetrica via LoRa (`kiteAltitude/`)

Progetto separato dal rilascio: due nodi **ESP32-C3 Super Mini** con modulo radio **SX1276 (LoRa 868 MHz)** che si scambiano solo l'altitudine dell'aquilone, senza alcun meccanismo di sgancio.

- **`kiteAltitude/kiteAltitudeTX`** (sull'aquilone, a batteria LiPo, nessun Wi-Fi/BLE per risparmiare peso e consumo): legge il BME280, applica la stessa catena di filtro del progetto di rilascio (spike-guard + mediana-3 + EMA) all'altitudine e trasmette un pacchetto di 15 byte (quota relativa, temperatura, batteria, sequenza) a un intervallo parametrico (default 1 volta al secondo).
- **`kiteAltitude/kiteAltitudeRX`** (stazione di terra, alimentata via USB): riceve i pacchetti via interrupt, calcola la velocità verticale e rileva quando l'aquilone, dopo essere stato stabile a una quota (plateau), inizia a scendere oltre una soglia impostabile. Espone un access point Wi-Fi proprio (`KiteAltitudeRX`) con una pagina che mostra altitudine, temperatura, velocità verticale, stato del link e batteria del TX, e che riproduce un **tono stile variometro** (pitch e ripetizione del beep modulati dal rateo di discesa) mentre l'allarme è attivo.

Documentazione hardware completa (BOM, piedinatura, cablaggio, parametri radio): `kiteAltitude/README.md`.

Librerie richieste (Arduino Library Manager): `RadioLib`, `Adafruit BME280 Library` (+ `Adafruit Unified Sensor`). Parametri radio e piedinatura SPI/I2C sono definiti in `PacketFormat.h`, presente identico in entrambe le sottocartelle: se lo si modifica va aggiornato in entrambe.

⚠️ La frequenza (869.525 MHz, sotto-banda EU868 "h1.5") e la potenza sono scelte per rientrare nei limiti di duty-cycle tipici della normativa ERC/ETSI, ma è responsabilità di chi trasmette verificarne l'applicabilità attuale in Italia prima dell'uso.
