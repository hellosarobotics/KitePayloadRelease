# Kite Altitude — Telemetria via LoRa

Sistema a due nodi, separato dal progetto di rilascio: un nodo **TX** sull'aquilone misura l'altitudine e la trasmette via **LoRa (SX1276, 868 MHz)**; un nodo **RX** a terra la riceve, calcola velocità verticale e allarme di discesa, e la mostra su una pagina web servita dal proprio access point Wi-Fi. Nessuna rete esterna necessaria su nessuno dei due nodi.

## 1. Panoramica

- **`kiteAltitudeTX/`**: nodo headless (nessun Wi-Fi/BLE/WebServer) a batteria, montato sull'aquilone. Legge il BME280 a intervallo parametrico (1 pacchetto ogni 2s di default, vedi §5bis), applica la stessa catena di filtro del progetto di rilascio esistente (spike-guard → mediana-3 → EMA) all'altitudine, e trasmette un pacchetto di 15 byte con altitudine relativa + temperatura via LoRa.
- **`kiteAltitudeRX/`**: stazione di terra, alimentata via USB. Riceve i pacchetti a interrupt, calcola la velocità verticale, rileva quando l'aquilone — dopo essere stato stabile a una quota (plateau) — inizia a scendere oltre una soglia, ed espone tutto su una pagina web (AP Wi-Fi `KiteAltitudeRX`, altitudine + temperatura + velocità verticale) con un tono stile variometro generato via Web Audio API.

## 2. Bill of Materials

**Nodo TX (aquilone):**
- ESP32-C3 "Super Mini"
- Breakout SX1276 868 MHz nudo (antenna via pad ANT, tipico pinout HopeRF RFM95/RA-01), collegato a **PA_BOOST** (verificare che il breakout non usi il pin RFO scollegato)
- BME280 (I2C)
- Antenna elicoidale 868 MHz a saldare sul pad ANT
- LiPo 1S 3.7V, 150–300 mAh
- Modulo di carica/protezione stile TP4056 **con protezione integrata** (DW01A+FS8205A) — non un TP4056 nudo, per non rischiare la sovra-scarica della cella

**Nodo RX (stazione di terra):**
- ESP32-C3 "Super Mini"
- Stesso breakout SX1276 868 MHz
- Alimentazione via USB/power bank — nessun vincolo di peso o batteria

⚠️ Il modulo di carica/protezione è materiale da BOM, non viene progettato qui (nessun PCB): sono indicati solo i collegamenti.

## 3. Piedinatura (identica su TX e RX per SPI/LoRa)

I pin di strapping dell'ESP32-C3 sono **GPIO2, GPIO8, GPIO9**. L'I2C eredita lo schema già validato nel progetto di rilascio esistente (`Wire.begin(8,9)`): sicuro perché le pull-up I2C tengono le linee alte già in fase di boot. **GPIO2 viene lasciato libero** deliberatamente: a differenza dell'I2C, un `RST`/`DIO0` di un modulo radio esterno potrebbe plausibilmente andare basso proprio all'accensione, prima che il firmware lo configuri — non è un rischio da correre per un pin di strapping.

| Segnale | GPIO | Nodo | Colore cavo (convenzione) |
|---|---|---|---|
| SX1276 SCK | 4 | TX + RX | Bianco/Verde |
| SX1276 MISO | 5 | TX + RX | Verde |
| SX1276 MOSI | 6 | TX + RX | Blu |
| SX1276 NSS/CS | 7 | TX + RX | Arancione |
| SX1276 RST | 10 | TX + RX | Bianco/Arancione |
| SX1276 DIO0 (IRQ) | 1 | TX + RX | Bianco/Blu |
| BME280 SDA | 8 | solo TX | Blu |
| BME280 SCL | 9 | solo TX | Giallo |
| Partitore batteria (ADC) | 3 | solo TX | Grigio |
| VCC (3V3) | — | TX + RX | Marrone |
| GND | — | TX + RX | Bianco/Marrone |

⚠️ Colori indicati solo come convenzione di cablaggio interna a questo progetto, per facilitare l'assemblaggio: non esiste uno standard elettronico universale per SCK/MISO/MOSI/CS/RST/IRQ (a differenza di rosso=VCC e nero=GND, quelli sì diffusi quasi ovunque). Il riferimento univoco resta il numero di GPIO in tabella, non il colore del filo usato.

Definiti come `#define` in `kiteAltitudeTX/PacketFormat.h` e `kiteAltitudeRX/PacketFormat.h` (file identico nelle due cartelle).

## 4. Cablaggio per nodo

**TX:**
- SX1276: `VCC`→3V3, `GND`→GND (mai da 5V/VBUS: il chip SX1276 è solo 3.3V), poi SCK/MISO/MOSI/NSS/RST/DIO0 come da tabella
- BME280: `VCC`→3V3, `GND`→GND, `SDA`→GPIO8, `SCL`→GPIO9
- Antenna elicoidale: saldata direttamente sul pad `ANT` del breakout
- Batteria: `LiPo+` → `BAT+` del modulo di carica, `LiPo−` → `BAT−`; `OUT+/OUT−` del modulo di carica → ingresso di alimentazione della board. ⚠️ Verificare col multimetro se quel pin della specifica board è a valle del regolatore di bordo prima di collegare: alimentare il chip direttamente a 4.2V (LiPo carica) supera il VDD massimo dell'ESP32-C3 (3.6V) se il pin bypassa il regolatore
- Partitore batteria: due resistenze 100kΩ in serie tra `LiPo+` e GND, punto centrale → GPIO3

**RX:**
- SX1276: stesso cablaggio del TX (3V3/GND + SCK/MISO/MOSI/NSS/RST/DIO0)
- Alimentazione board: USB-C

## 5. Parametri radio LoRa

Devono essere **identici, testualmente**, nelle chiamate `radio.begin(...)` di TX e RX (definiti una sola volta in `PacketFormat.h`):

| Parametro | Valore |
|---|---|
| Frequenza | 869.525 MHz (sotto-banda EU868 "h1.5", 869.40–869.65 MHz) |
| Banda | 125 kHz |
| Spreading factor | SF7 |
| Coding rate | 4/5 |
| Sync word | `0x12` (privata, diversa dalla `0x34` pubblica LoRaWAN) |
| Potenza | +14 dBm |
| Preambolo | 8 simboli |
| Cadenza TX | parametrica, default 1 pacchetto ogni 2s (vedi §5bis) — a 1/2s: ~46 ms di airtime → duty cycle ~2.3%, sotto il limite ~10% della sotto-banda h1.5 |

⚠️ Verificare la normativa italiana/ERC attuale per questa sotto-banda prima di trasmettere davvero — è responsabilità di chi usa il sistema, non qualcosa dato per assodato da questa documentazione.

## 5bis. Intervallo di campionamento/trasmissione (parametrico)

`SAMPLE_INTERVAL_MS` in cima a `kiteAltitudeTX.ino` è l'unica costante da cambiare per campionare/trasmettere ogni 1s, 3s, ecc. invece che ogni 2s (default 2000). È una costante di compilazione — per cambiarla serve riflashare il TX, non è regolabile a runtime: il link radio è solo TX→RX, e costruire un canale di configurazione bidirezionale via LoRa solo per questo non sarebbe giustificato.

Cosa succede cambiandolo:

- **Vantaggio**: meno duty-cycle radio usato (più margine sotto il limite normativo) e consumo batteria leggermente più basso sul TX — beneficio comunque marginale, dato che a 1 Hz l'autonomia stimata (5–10+ ore) già eccede una sessione di volo tipica.
- **Svantaggio principale — allarme più lento**: l'RX calcola il rateo di discesa sul tempo reale tra i pacchetti (non assume 1 Hz), quindi non si "rompe" nulla, ma con un intervallo più lungo la stima del rateo arriva con più ritardo, in proporzione. Per un aquilone che scende rapidamente, un allarme più lento è l'esatto contrario di quello che serve.
- **Filtro TX auto-scalato**: lo spike-guard e il passo massimo per campione sono espressi come rateo (m/s, `MAX_CLIMB_RATE_MPS`/`SPIKE_RATE_MPS`) e moltiplicati per l'intervallo, quindi restano corretti a qualunque valore si scelga. Anche l'EMA è derivata da una costante di tempo fissa (`ALT_TIME_CONSTANT_S`) invece che da un alpha fisso, così la sua reattività reale (in secondi) non cambia cambiando l'intervallo — a 1000ms il comportamento è identico alla versione originale a 1 Hz.
- **Da ritarare manualmente sull'RX**: `plateauHoldMs` e `linkTimeoutMs` (impostabili dal form web, non richiedono reflash) presuppongono per default ~1 pacchetto/s. Se si allunga l'intervallo sul TX, vanno aumentati proporzionalmente sull'RX — altrimenti, ad esempio, il plateau potrebbe confermarsi dopo un solo pacchetto invece che dopo diversi secondi di stabilità reale, o il link potrebbe sembrare "perso" dopo la sola normale attesa tra due pacchetti.
- **Mediana più lenta in termini reali**: la finestra a 3 campioni (`MEDIAN_N`) dura `3 × SAMPLE_INTERVAL_MS` in tempo reale — a 2s sono 6s (attuale), a 3s diventano 9s di finestra, che si somma al ritardo dell'EMA. Non è "sbagliato", ma aggiunge latenza percepita al valore di altitudine mostrato.

## 6. Alimentazione/potenza (TX)

Il TX resta sempre acceso con campionamento continuo a intervallo fisso (default ogni 2s, Wi-Fi/BLE mai inizializzati, radio LoRa messa in `sleep()` tra una trasmissione e la successiva) invece di usare il deep-sleep dell'ESP32-C3. Motivo: il deep-sleep interromperebbe la continuità dei campioni di cui l'RX ha bisogno per calcolare vario/plateau, e richiederebbe salvare lo stato del filtro in RTC memory — complessità non necessaria per la v1, dato che il consumo stimato (~28–30 mA medi) dà già 5–10+ ore su una LiPo 150–300 mAh, ben oltre una sessione di volo tipica. Da rivalutare come ottimizzazione futura solo se servirà più autonomia.

## 7. Librerie richieste (Arduino Library Manager)

- **RadioLib** (driver SX1276)
- **Adafruit BME280 Library** + **Adafruit Unified Sensor** (solo per il TX)
- Board package **ESP32** (con supporto ESP32-C3) installato in Arduino IDE

## 8. Verifica end-to-end (senza dover volare l'aquilone)

1. **Test banco base**: accendere entrambe le schede vicine, caricare la pagina RX su `http://192.168.4.1/` (IP di default dell'AP), verificare che `/data` si popoli, `seq` cresca e `linkOk` sia `true`.
2. **Test con `SIMULATE_ALTITUDE` attivo** (il più utile): decommentare `#define SIMULATE_ALTITUDE` in `kiteAltitudeTX.ino` e riflashare il TX — genera una rampa ciclica (salita 0→20m/20s, plateau 10s, discesa ~3m/s/7s, plateau 5s). Verificare che il plateau si confermi nei tempi previsti, l'allarme scatti alla soglia di discesa attesa, la velocità verticale segua il rateo scriptato, il tono cambi pitch/beep-rate di conseguenza, e l'allarme si azzeri al recupero. Ricordarsi di ricompilare senza la macro prima del volo.
3. **Test fisico opzionale**: portare il nodo TX (senza `SIMULATE_ALTITUDE`) su/giù per una scala; il BME280 registra davvero qualche hPa di variazione, per una verifica end-to-end indipendente dalla simulazione.
4. **Test perdita link**: spegnere o allontanare il TX; verificare che dopo `linkTimeoutMs` (default 10s) la UI passi allo stato rosso "LINK PERSO" e un eventuale tono attivo si interrompa subito.
5. **Sblocco audio**: ricaricare la pagina con un allarme già attivo lato server; verificare che non parta alcun suono finché non si clicca "Attiva audio" (policy autoplay del browser).
6. **Persistenza impostazioni**: modificare le soglie dal form "Impostazioni allarme discesa", riavviare l'RX, verificare che i valori restino salvati (EEPROM) e che il comportamento le rispetti.
7. **Controllo airtime/duty-cycle**: leggere sul Serial del TX i timestamp di trasmissione; confermare cadenza ~1 pacchetto ogni 2s e airtime coerente con la stima (~46ms), per validare il calcolo di duty-cycle del §5.
8. **Pre-check di portata all'aperto** (senza `SIMULATE_ALTITUDE`): allontanare il TX dall'RX osservando RSSI/SNR nella card diagnostica della pagina, prima di fidarsi in volo.
