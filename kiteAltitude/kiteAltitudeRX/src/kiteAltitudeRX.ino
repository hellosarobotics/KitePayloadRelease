// kiteAltitudeRX.ino
// Stazione di terra: riceve via LoRa (SX1276) altitudine e temperatura trasmesse da kiteAltitudeTX,
// crea un access point Wi-Fi proprio e mostra una pagina web con altitudine, temperatura, velocità
// verticale e un allarme sonoro "stile variometro" quando l'aquilone, dopo essere stato
// stabile a una certa quota, inizia a scendere.
//
// Tutta la logica di plateau/discesa/allarme vive qui (server-autoritativo, come lo stato
// "released" nel progetto di rilascio esistente): il browser si limita a mostrare lo stato
// e a generare il tono via Web Audio API.

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <EEPROM.h>
#include <SPI.h>
#include <RadioLib.h>
#include "PacketFormat.h"

// ======= Wi-Fi AP =======
const char* ssid = "KiteAltitudeRX";
const char* password = "12345678";
const byte DNS_PORT = 53;

WebServer server(80);
DNSServer dnsServer;
SX1276 radio = new Module(LORA_CS_PIN, LORA_DIO0_PIN, LORA_RST_PIN, RADIOLIB_NC);

// ======= EEPROM: 5 float, 20 byte =======
#define EEPROM_SIZE 20
float plateauToleranceM = 1.0f; // [0.1, 10.0]  m di tolleranza per considerare "stabile" la quota
float descentTriggerM   = 3.0f; // [0.5, 50.0]  m di calo dal plateau che fa scattare l'allarme
float plateauHoldMs     = 8000;  // [500, 30000] ms di stabilità richiesti per confermare il plateau
float linkTimeoutMs     = 10000; // [1000, 60000] ms senza pacchetti prima di dichiarare il link perso
float historySampleMs   = 10000; // [1000, 60000] ms tra un campione e l'altro nei grafici storici

// ======= Stato ricezione LoRa =======
volatile bool packetFlag = false;
void setFlag() { packetFlag = true; }

bool lastSeqValid = false;
uint16_t lastSeq = 0;
uint32_t lastTxUptimeMs = 0;
uint32_t packetsReceived = 0;
uint32_t packetsLostEst = 0;
uint32_t lastPacketMs = 0;
float g_altitude = 0;
// Quota massima della sessione di volo corrente: vive in RAM come lo storico, quindi un
// reload della pagina non la perde, ma un riavvio dell'RX sì (nuova sessione di volo).
float g_maxAltitude = 0;
float g_temperature = 0;
float g_humidity = 0;
float g_batteryMv = 0;
float g_rssi = 0, g_snr = 0;

// ======= Temperatura CPU RX: campionata 1x/s + EMA (il sensore interno ha step di ~1°C) =======
float g_cpuTempRX = 0;
bool cpuTempRXInit = false;
uint32_t lastCpuTempMs = 0;

// ======= Velocità verticale: derivata su finestra di 3 campioni =======
#define VSPEED_WINDOW 3
uint32_t vsTimeMs[VSPEED_WINDOW];
float vsAlt[VSPEED_WINDOW];
uint8_t vsIdx = 0;
uint8_t vsCount = 0;
float g_vSpeed = 0;

// ======= Storico per i grafici web: buffer circolare lato RX, così un reload della pagina
// (che azzererebbe uno storico tenuto solo nel browser) non perde i dati dall'accensione
// dell'RX. Tempi in millis() dell'RX (non del TX): un riavvio della stazione di terra azzera
// sia il buffer sia il tempo mostrato nei grafici, a prescindere da quanto è acceso il TX.
// Campionato a parte dal rateo di invio del TX, all'intervallo configurabile historySampleMs:
// con l'impostazione di default (10s) 1800 campioni coprono circa 5 ore (costano ~28KB fissi
// di RAM, ampiamente sostenibili sull'ESP32-C3). =======
#define HISTORY_CAPACITY 1800
uint32_t histT[HISTORY_CAPACITY];
float histAlt[HISTORY_CAPACITY];
float histTemp[HISTORY_CAPACITY];
float histHum[HISTORY_CAPACITY];
uint16_t histCount = 0;
uint16_t histHead = 0;
uint32_t lastHistPushMs = 0;
bool historyStarted = false;

void pushHistorySample(uint32_t t, float alt, float temp, float hum) {
  histT[histHead] = t;
  histAlt[histHead] = alt;
  histTemp[histHead] = temp;
  histHum[histHead] = hum;
  histHead = (histHead + 1) % HISTORY_CAPACITY;
  if (histCount < HISTORY_CAPACITY) histCount++;
}

// ======= Stato plateau / allarme discesa =======
float plateauAltitude = 0;
uint32_t plateauStableSinceMs = 0;
bool plateauConfirmed = false;
bool sinkAlarm = false;
uint32_t alarmRecoverSinceMs = 0;
// 0 e non un valore negativo: l'allarme deve suonare per tutta la durata della discesa,
// quindi la "ripresa" richiede che l'aquilone non stia più scendendo affatto (velocità
// verticale non negativa), non solo che scenda più lentamente di una soglia arbitraria.
const float ALARM_RECOVER_VSPEED = 0.0f;
const uint32_t RECOVER_HOLD_MS = 2000;

// ---------------------- EEPROM: helper generici (stesso pattern raw-byte del progetto esistente) ----------------------
void saveFloatAt(int addr, float value) {
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < 4; i++) EEPROM.write(addr + i, *p++);
  EEPROM.commit();
}
float loadFloatAt(int addr, float lo, float hi, float fallback) {
  float value;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < 4; i++) *p++ = EEPROM.read(addr + i);
  if (isnan(value) || value < lo || value > hi) return fallback;
  return value;
}

// ---------------------- Velocità verticale ----------------------
void pushVSpeedSample(uint32_t t, float alt) {
  vsTimeMs[vsIdx] = t;
  vsAlt[vsIdx] = alt;
  vsIdx = (vsIdx + 1) % VSPEED_WINDOW;
  if (vsCount < VSPEED_WINDOW) vsCount++;

  if (vsCount < 2) return;
  uint8_t oldestIdx = (vsIdx + VSPEED_WINDOW - vsCount) % VSPEED_WINDOW;
  float dt = (t - vsTimeMs[oldestIdx]) / 1000.0f;
  if (dt <= 0) return;
  // Nessuna EMA aggiuntiva qui sopra rawSpeed: la finestra di 3 campioni (~4-6s col rateo di
  // invio del TX) già fa da smoothing, e sommarci un'altra EMA (alpha 0.3, costante di tempo
  // ~5-6s) raddoppiava il ritardo con cui una variazione reale di velocità arrivava a schermo.
  g_vSpeed = (alt - vsAlt[oldestIdx]) / dt;
}

// ---------------------- Macchina a stati: plateau + allarme discesa ----------------------
void updatePlateauAndAlarm(uint32_t t, float alt) {
  if (!plateauConfirmed) {
    // Fase di ricerca: la quota candidata insegue l'altitudine con una EMA finché resta
    // entro tolleranza, e viene confermata dopo plateauHoldMs di stabilità continua.
    if (fabs(alt - plateauAltitude) <= plateauToleranceM) {
      plateauAltitude = plateauAltitude * 0.8f + alt * 0.2f;
      if (plateauStableSinceMs == 0) plateauStableSinceMs = t;
      if (t - plateauStableSinceMs >= (uint32_t)plateauHoldMs) plateauConfirmed = true;
    } else {
      // si è mosso troppo per restare candidato: si ricomincia da qui
      plateauAltitude = alt; plateauStableSinceMs = t;
    }
  } else if (alt > plateauAltitude + plateauToleranceM) {
    // salita oltre tolleranza rispetto al plateau confermato: nuovo candidato più in alto
    plateauAltitude = alt; plateauStableSinceMs = t; plateauConfirmed = false;
  } else {
    // Plateau confermato: da qui in poi resta fisso (non insegue più l'altitudine) finché
    // non risale abbastanza da ridiventare candidato, o l'allarme scatta e poi recupera
    // (vedi sotto). Così anche una discesa lenta, un campione alla volta entro tolleranza,
    // accumula comunque lo scarto dal riferimento invece di "trascinarselo dietro" verso
    // il basso senza mai far scattare l'allarme.
    if ((plateauAltitude - alt) > descentTriggerM) {
      sinkAlarm = true;
    }
  }

  if (sinkAlarm) {
    if (g_vSpeed > ALARM_RECOVER_VSPEED) {
      if (alarmRecoverSinceMs == 0) alarmRecoverSinceMs = t;
      if (t - alarmRecoverSinceMs >= RECOVER_HOLD_MS) {
        sinkAlarm = false; alarmRecoverSinceMs = 0;
        plateauAltitude = alt; plateauStableSinceMs = t; plateauConfirmed = false;
      }
    } else {
      alarmRecoverSinceMs = 0;
    }
  }
}

void resetVarioState() {
  plateauAltitude = g_altitude;
  plateauStableSinceMs = 0;
  plateauConfirmed = false;
  sinkAlarm = false;
  alarmRecoverSinceMs = 0;
  vsIdx = 0; vsCount = 0; g_vSpeed = 0;
}

// ---------------------- WEB UI ----------------------
// La pagina dipende solo dalle impostazioni correnti (tolleranza, hold, ecc.), non dalle
// letture live (quelle arrivano via /data): la si ricostruisce una volta e si tiene in cache,
// invece di rigenerare/ritrasmettere ~15-20KB di HTML a ogni GET. Senza questo, ogni probe di
// rilevamento captive-portal del telefono (periodico, oltre al primo caricamento pagina) pagava
// il costo pieno di concatenazione stringhe + trasmissione Wi-Fi dell'intera pagina.
String cachedHtml;
bool htmlDirty = true;

void buildHtml() {
  String html;
  html.reserve(24000); // tema più ricco (annunciatore, pannelli, orologio): stima larga per evitare troppe riallocazioni
  html += "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<meta name='theme-color' content='#060911' id='themeColor'>";
  html += "<style>";
  // Palette "mission control": nero spaziale, pannelli con bordo blu Insignia, verde/ambra/rosso
  // riservati esclusivamente allo stato del link/allarme (mai usati come colori decorativi altrove).
  // Tema scuro fisso: un pannello di controllo non ha una versione "chiara", quindi niente
  // prefers-color-scheme qui, solo i token in :root.
  html += ":root{--void:#060911;--panel:#0D1524;--panel-border:#1E3A6E;--accent:#4C8DFF;--good:#2FE6A0;--warn:#FFB020;--critical:#FF4433;--text:#E7ECF5;--text-dim:#7E8AA3;--text-faint:#3E4A63;--data-1:#4C8DFF;--data-2:#FF9F45;--data-3:#38D9F5;--mono:ui-monospace,'Cascadia Code','SFMono-Regular',Consolas,'Liberation Mono',monospace;--sans:-apple-system,'Segoe UI',Roboto,Ubuntu,Arial,sans-serif;}";
  html += "*{box-sizing:border-box}";
  html += "body{margin:0;background:var(--void);color:var(--text);font-family:var(--sans);}";
  html += ".container{max-width:480px;margin:24px auto 60px;padding:0 14px;}";
  html += ".titlebar{display:flex;align-items:center;justify-content:space-between;gap:10px;margin-bottom:14px;flex-wrap:wrap}";
  html += ".titlebar-actions{display:flex;gap:8px;flex-wrap:wrap}";
  html += ".title{font-family:var(--mono);font-size:16px;font-weight:700;letter-spacing:.04em}";
  html += ".title .sub{display:block;font-size:9.5px;font-weight:400;letter-spacing:.14em;color:var(--text-dim);margin-top:2px}";
  html += ".annunciator{display:flex;flex-direction:column;gap:10px;padding:14px 16px;border:1px solid var(--panel-border);border-radius:8px;background:var(--panel);margin-bottom:14px}";
  html += ".annunciator-top{display:flex;align-items:center;justify-content:space-between;gap:12px}";
  html += ".annunciator-vitals{display:flex;gap:22px;padding-top:10px;border-top:1px solid var(--panel-border)}";
  html += ".vital{display:flex;align-items:center;gap:8px}";
  html += ".vital .label{font-family:var(--mono);font-size:9.5px;letter-spacing:.08em;text-transform:uppercase;color:var(--text-dim)}";
  html += ".vital .val{font-family:var(--mono);font-weight:700;font-size:13px;color:var(--text)}";
  html += ".vital .val .unit{font-family:var(--sans);font-size:11px;font-weight:600;color:var(--text-dim);margin-left:2px}";
  html += ".lamp-group{display:flex;align-items:center;gap:10px}";
  // .lamp di base è "spento" (grigio, niente glow): niente falso verde prima che arrivi la prima risposta da /data.
  html += ".lamp{width:14px;height:14px;border-radius:50%;background:var(--text-faint);flex:none;transition:background .2s,box-shadow .2s}";
  html += ".lamp-text{font-family:var(--mono);font-weight:700;font-size:13px;letter-spacing:.06em}";
  html += ".lamp-text .sub{display:block;font-family:var(--sans);font-weight:400;font-size:10.5px;letter-spacing:0;color:var(--text-dim);margin-top:2px}";
  html += ".clock{text-align:right;font-family:var(--mono)}";
  html += ".clock .t{font-size:16px;font-weight:700;font-variant-numeric:tabular-nums}";
  html += ".clock .label{display:block;font-size:9px;color:var(--text-dim);letter-spacing:.1em;margin-top:2px}";
  html += ".annunciator.state-ok .lamp{background:var(--good);box-shadow:0 0 3px var(--good),0 0 14px 3px var(--good)}";
  html += ".annunciator.state-warn{border-color:#5b4212}.annunciator.state-warn .lamp{background:var(--warn);box-shadow:0 0 3px var(--warn),0 0 14px 3px var(--warn)}";
  html += ".annunciator.state-bad{border-color:#5a1c14}.annunciator.state-bad .lamp{background:var(--critical);box-shadow:0 0 3px var(--critical),0 0 14px 3px var(--critical);animation:pulse 1s infinite}";
  html += "@keyframes pulse{0%,100%{opacity:1}50%{opacity:.45}}";
  html += "@media (prefers-reduced-motion: reduce){.annunciator.state-bad .lamp{animation:none}}";
  html += ".panel{position:relative;border:1px solid var(--panel-border);border-radius:6px;background:var(--panel);padding:16px 18px 18px;margin-bottom:12px}";
  html += ".panel:before,.panel:after,.rivet-l:before,.rivet-l:after{content:'';position:absolute;width:3px;height:3px;border-radius:50%;background:var(--text-faint)}";
  html += ".panel:before{top:6px;left:6px}.panel:after{top:6px;right:6px}";
  html += ".rivet-l:before{bottom:6px;left:6px}.rivet-l:after{bottom:6px;right:6px}";
  html += ".panel-title{font-family:var(--mono);font-size:10.5px;font-weight:700;letter-spacing:.12em;text-transform:uppercase;color:var(--text-dim);margin:0 0 12px;padding-bottom:8px;border-bottom:1px solid var(--panel-border)}";
  html += ".metrics{display:flex;gap:18px;flex-wrap:wrap}";
  html += ".metric{flex:1 1 110px;min-width:100px}";
  html += ".metric .label{font-family:var(--mono);font-size:9.5px;letter-spacing:.08em;text-transform:uppercase;color:var(--text-dim);margin-bottom:6px}";
  html += ".metric .value{font-family:var(--mono);font-weight:700;font-size:clamp(22px,7vw,28px);font-variant-numeric:tabular-nums;line-height:1}";
  html += ".metric .value.accent-good{color:var(--good)}";
  html += ".metric .unit{font-family:var(--sans);font-size:12px;font-weight:600;color:var(--text-dim);margin-left:4px}";
  html += ".help{font-size:12.5px;line-height:1.55;color:var(--text-dim);margin-top:10px}";
  html += ".qual{display:inline-block;font-family:var(--mono);font-size:11px;font-weight:700;letter-spacing:.05em;padding:2px 8px;border-radius:4px}";
  html += ".qual.ottimo{background:rgba(47,230,160,.12);color:var(--good);border:1px solid rgba(47,230,160,.35)}";
  html += ".qual.buono{background:rgba(76,141,255,.12);color:var(--accent);border:1px solid rgba(76,141,255,.35)}";
  html += ".qual.scarso{background:rgba(255,68,51,.12);color:var(--critical);border:1px solid rgba(255,68,51,.35)}";
  html += "details.link-details{margin-top:10px}";
  html += "details.link-details summary{cursor:pointer;color:var(--accent);font-size:11.5px;font-weight:700;list-style:none;font-family:var(--mono);letter-spacing:.04em}";
  html += "details.link-details summary::-webkit-details-marker{display:none}";
  html += "details.link-details summary:before{content:'▸ ';}";
  html += "details.link-details[open] summary:before{content:'▾ ';}";
  html += "details.link-details .help{margin-top:6px}";
  html += ".panel-collapse summary{cursor:pointer;list-style:none;display:block}";
  html += ".panel-collapse summary::-webkit-details-marker{display:none}";
  html += ".panel-collapse summary:before{content:'▸ '}";
  html += ".panel-collapse[open] summary:before{content:'▾ '}";
  html += ".subhead{font-family:var(--mono);font-size:9.5px;font-weight:700;letter-spacing:.1em;text-transform:uppercase;color:var(--text-faint);margin:18px 0 10px}";
  html += ".subhead:first-of-type{margin-top:4px}";
  html += "form .setting{display:flex;flex-direction:column;gap:5px;margin-bottom:14px}";
  html += ".setting .name{font-weight:600;font-size:13px}";
  html += ".setting .sub{font-size:12px;color:var(--text-dim);line-height:1.5}";
  html += "input[type=number]{background:var(--void);border:1px solid var(--panel-border);color:var(--text);font-family:var(--mono);border-radius:5px;padding:9px 10px;width:100%;font-size:14px}";
  html += "input[type=number]:focus{outline:2px solid var(--accent);outline-offset:1px;border-color:var(--accent)}";
  html += ".btn-row{display:flex;gap:10px;margin-top:6px;flex-wrap:wrap}";
  html += ".btn,.btn-ghost{appearance:none;cursor:pointer;border-radius:6px;padding:11px 16px;font-family:var(--mono);font-weight:700;font-size:12px;letter-spacing:.06em;text-transform:uppercase}";
  html += ".btn{border:none;background:var(--accent);color:var(--void)} .btn:hover{background:#6ba0ff}";
  html += ".btn-ghost{background:transparent;border:1px solid var(--panel-border);color:var(--text-dim)} .btn-ghost:hover{border-color:var(--accent);color:var(--accent)}";
  // Stesso linguaggio grafico del badge .qual (pillola con sfondo/bordo/testo dello stesso
  // colore): ambra finché l'audio non è attivo (va notato e premuto), verde una volta attivato.
  html += ".btn-audio{appearance:none;cursor:pointer;border-radius:6px;padding:11px 16px;font-family:var(--mono);font-weight:700;font-size:12px;letter-spacing:.06em;text-transform:uppercase;background:rgba(255,176,32,.12);color:var(--warn);border:1px solid rgba(255,176,32,.35)}";
  html += ".btn-audio:hover{background:rgba(255,176,32,.2)}";
  html += ".btn-audio.on{background:rgba(47,230,160,.12);color:var(--good);border:1px solid rgba(47,230,160,.35)}";
  html += ".btn-audio.on:hover{background:rgba(47,230,160,.2)}";
  html += ".btn-audio:disabled{opacity:1;cursor:default}";
  html += "button:focus-visible,a:focus-visible{outline:2px solid var(--text);outline-offset:2px}";
  html += "canvas.chart{width:100%;height:140px;display:block;touch-action:pan-y}";
  html += ".chart-hover{font-family:var(--mono);font-size:11px;color:var(--text-dim);margin-top:6px;min-height:14px}";
  html += "</style>";

  html += "<script>";
  html += "function setThemeColor(hex){var m=document.getElementById('themeColor'); if(m){m.setAttribute('content',hex);} }";
  html += "var LINK_TIMEOUT_S=" + String(linkTimeoutMs / 1000.0f, 0) + ";";

  // Schermo intero: su Chrome Android è l'unico modo per nascondere davvero la barra degli
  // indirizzi (che altrimenti resta visibile, colorata, in cima alla pagina). Richiede un gesto
  // dell'utente (il tap sul pulsante), non può partire da sola al caricamento.
  html += "function toggleFullscreen(){";
  html += "if(!document.documentElement.requestFullscreen)return;";
  html += "if(!document.fullscreenElement){document.documentElement.requestFullscreen().catch(function(){});}";
  html += "else{document.exitFullscreen();}}";
  html += "document.addEventListener('fullscreenchange',function(){";
  html += "var b=document.getElementById('fsBtn');if(!b)return;";
  html += "b.textContent=document.fullscreenElement?'Esci da schermo intero':'Schermo intero';});";

  // --- Web Audio: tono continuo di allarme (niente più beep intermittente: l'RX è già
  // l'autorità su quando l'allarme deve suonare/spegnersi, incluso il minimo di 10s
  // all'attivazione — qui ci si limita a riprodurre lo stato che arriva da /data). ---
  html += "let audioCtx=null,osc=null,gainNode=null,audioEnabled=false;";
  html += "function enableAudio(){";
  html += "if(!audioCtx){audioCtx=new (window.AudioContext||window.webkitAudioContext)();";
  html += "osc=audioCtx.createOscillator();gainNode=audioCtx.createGain();gainNode.gain.value=0;osc.frequency.value=600;";
  html += "osc.connect(gainNode);gainNode.connect(audioCtx.destination);osc.start();}";
  html += "audioCtx.resume();audioEnabled=true;";
  html += "var b=document.getElementById('audioBtn');b.textContent='Audio attivo';b.disabled=true;b.classList.add('on');}";

  html += "function updateTone(sinking,vspeed){";
  html += "if(!audioEnabled)return;";
  html += "if(!sinking){gainNode.gain.setTargetAtTime(0,audioCtx.currentTime,0.1);return;}";
  html += "var absV=Math.min(Math.abs(vspeed),6);";
  html += "var freq=Math.max(150,Math.min(600,600-80*absV));";
  html += "osc.frequency.setTargetAtTime(freq,audioCtx.currentTime,0.05);";
  html += "gainNode.gain.setTargetAtTime(0.15,audioCtx.currentTime,0.05);}";

  // --- Doppio beep di conferma al raggiungimento della quota di stabilizzazione: oscillatori
  // "usa e getta" separati dal tono continuo dell'allarme discesa (niente interferenza), tono
  // acuto/positivo (880Hz di default) ben distinto dal tono grave e continuo dell'allarme
  // (150-600Hz). Frequenza parametrizzata: la riusa anche il pip di link ripristinato, con un
  // timbro più acuto per restare distinguibile. ---
  html += "function beepTone(startTime,dur,freq){";
  html += "var o=audioCtx.createOscillator(),g=audioCtx.createGain();o.frequency.value=freq||880;";
  html += "g.gain.setValueAtTime(0,startTime);g.gain.linearRampToValueAtTime(0.2,startTime+0.01);g.gain.linearRampToValueAtTime(0,startTime+dur);";
  html += "o.connect(g);g.connect(audioCtx.destination);o.start(startTime);o.stop(startTime+dur+0.02);}";
  html += "function playPlateauBeep(){if(!audioEnabled)return;var t0=audioCtx.currentTime;beepTone(t0,0.12);beepTone(t0+0.16,0.12);}";

  // --- Segnali di link perso/ripristinato, timbro sintetico "da drone" (onda a dente di sega +
  // tremolo veloce sovrapposto al guadagno, come uno scanner elettronico): sweep discendente e
  // cupo per il link perso, sweep ascendente + doppio pip acuto (1200Hz) per il ripristino, così
  // i due eventi restano intuitivamente opposti anche solo ad orecchio. ---
  // Filtro passa-basso agganciato allo sweep (taglia gli armonici alti del dente di sega,
  // lasciando passare fondamentale + primo armonico): stesso sweep di prima ma timbro più
  // cupo/ovattato invece che squillante.
  html += "function droneSweep(t0,fFrom,fTo,dur,peakGain){";
  html += "var o=audioCtx.createOscillator();o.type='sawtooth';";
  html += "var filt=audioCtx.createBiquadFilter();filt.type='lowpass';filt.Q.value=1;";
  html += "filt.frequency.setValueAtTime(fFrom*1.8,t0);filt.frequency.exponentialRampToValueAtTime(fTo*1.8,t0+dur);";
  html += "var g=audioCtx.createGain();";
  html += "var lfo=audioCtx.createOscillator();lfo.type='sine';lfo.frequency.value=16;";
  html += "var lfoGain=audioCtx.createGain();lfoGain.gain.value=peakGain*0.35;";
  html += "lfo.connect(lfoGain);lfoGain.connect(g.gain);";
  html += "o.frequency.setValueAtTime(fFrom,t0);o.frequency.exponentialRampToValueAtTime(fTo,t0+dur);";
  html += "g.gain.setValueAtTime(0,t0);g.gain.linearRampToValueAtTime(peakGain,t0+0.03);g.gain.linearRampToValueAtTime(0,t0+dur+0.05);";
  html += "o.connect(filt);filt.connect(g);g.connect(audioCtx.destination);";
  html += "o.start(t0);lfo.start(t0);o.stop(t0+dur+0.08);lfo.stop(t0+dur+0.08);}";
  html += "function playLinkLostAlert(){if(!audioEnabled)return;droneSweep(audioCtx.currentTime,900,140,0.5,0.22);}";
  html += "function playLinkRestored(){if(!audioEnabled)return;var t0=audioCtx.currentTime;droneSweep(t0,220,700,0.22,0.2);beepTone(t0+0.30,0.06,1200);beepTone(t0+0.42,0.06,1200);}";

  html += "var plateauStateInited=false,lastPlateauConfirmed=false;";
  html += "function checkPlateauReached(d){";
  html += "if(!plateauStateInited){plateauStateInited=true;lastPlateauConfirmed=d.plateauConfirmed;return;}";
  html += "if(d.linkOk&&d.plateauConfirmed&&!lastPlateauConfirmed)playPlateauBeep();";
  html += "lastPlateauConfirmed=d.plateauConfirmed;}";

  html += "var linkStateInited=false,lastLinkOk=false;";
  html += "function checkLinkTransition(d){";
  html += "if(!linkStateInited){linkStateInited=true;lastLinkOk=d.linkOk;return;}";
  html += "if(!d.linkOk&&lastLinkOk)playLinkLostAlert();";
  html += "else if(d.linkOk&&!lastLinkOk)playLinkRestored();";
  html += "lastLinkOk=d.linkOk;}";

  // --- Qualità link da RSSI/SNR: prende il peggiore dei due giudizi (un solo numero buono
  // non basta se l'altro è scarso) ---
  html += "function linkQuality(rssi,snr){";
  html += "var r=rssi>=-80?3:rssi>=-100?2:rssi>=-115?1:0;";
  html += "var s=snr>=5?3:snr>=0?2:snr>=-10?1:0;";
  html += "var lvl=Math.min(r,s);";
  html += "return [['Scarso','scarso'],['Scarso','scarso'],['Buono','buono'],['Ottimo','ottimo']][lvl];}";

  // --- Grafici storico (altitudine/temperatura/umidità nel tempo dall'accensione dell'RX) ---
  // hist viene ripopolato da /history al caricamento pagina (buffer autoritativo tenuto
  // dall'RX) e poi accodato qui coi nuovi punti dal polling /data: stesso rateo di
  // campionamento di 10s dell'RX, così il grafico resta uniforme dallo storico alla coda live.
  // L'asse dei tempi è legato all'uptime dell'RX (non del TX): un riavvio della stazione di
  // terra azzera sia il buffer sia il tempo mostrato, indipendentemente da quanto è acceso il TX.
  html += "var CH_MAX=1800;";
  html += "var HIST_SAMPLE_MS=" + String(historySampleMs, 0) + ";";
  html += "var hist={t:[],alt:[],temp:[],hum:[]};";
  html += "var lastHistT=-1;";
  html += "function pushHistory(d){";
  html += "if(lastHistT>=0&&d.elapsedMs-lastHistT<HIST_SAMPLE_MS)return;";
  html += "lastHistT=d.elapsedMs;";
  html += "hist.t.push(d.elapsedMs/1000);hist.alt.push(d.altitude);hist.temp.push(d.temperature);hist.hum.push(d.humidity);";
  html += "if(hist.t.length>CH_MAX){hist.t.shift();hist.alt.shift();hist.temp.shift();hist.hum.shift();}}";

  html += "function fmtElapsed(s){s=Math.max(0,Math.round(s));";
  html += "var h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sec=s%60;";
  html += "var mm=(h>0?String(m).padStart(2,'0'):String(m));";
  html += "return (h>0?h+':':'')+mm+':'+String(sec).padStart(2,'0');}";

  html += "function fmtClock(s){s=Math.max(0,Math.round(s));";
  html += "var h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sec=s%60;";
  html += "return String(h).padStart(2,'0')+':'+String(m).padStart(2,'0')+':'+String(sec).padStart(2,'0');}";

  html += "function drawChart(canvas,xs,ys,color,unit,decimals){";
  html += "var dpr=window.devicePixelRatio||1;var w=canvas.clientWidth,h=canvas.clientHeight;";
  html += "if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);}";
  html += "var ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);";
  html += "var padL=38,padR=8,padT=10,padB=20;var plotW=w-padL-padR,plotH=h-padT-padB;";
  html += "if(xs.length<2){ctx.fillStyle='#7E8AA3';ctx.font='12px ui-monospace,Consolas,monospace';ctx.textAlign='center';ctx.fillText('In attesa di dati...',w/2,h/2);canvas._chartData=null;return;}";
  html += "var minY=Math.min.apply(null,ys),maxY=Math.max.apply(null,ys);if(minY===maxY){minY-=1;maxY+=1;}";
  html += "var pad=(maxY-minY)*0.1;minY-=pad;maxY+=pad;";
  html += "var minX=xs[0],maxX=xs[xs.length-1];";
  html += "function px(x){return padL+(x-minX)/((maxX-minX)||1)*plotW;}";
  html += "function py(y){return padT+plotH-(y-minY)/(maxY-minY)*plotH;}";
  html += "ctx.strokeStyle='rgba(62,74,99,0.4)';ctx.lineWidth=1;ctx.fillStyle='#7E8AA3';ctx.font='10px ui-monospace,Consolas,monospace';ctx.textAlign='right';ctx.textBaseline='middle';";
  html += "for(var i=0;i<=2;i++){var v=minY+(maxY-minY)*i/2;var yy=py(v);ctx.beginPath();ctx.moveTo(padL,yy);ctx.lineTo(w-padR,yy);ctx.stroke();ctx.fillText(v.toFixed(decimals),padL-6,yy);}";
  html += "ctx.fillStyle='#7E8AA3';ctx.textBaseline='alphabetic';ctx.textAlign='left';ctx.fillText(fmtElapsed(minX),padL,h-4);ctx.textAlign='right';ctx.fillText(fmtElapsed(maxX),w-padR,h-4);";
  html += "ctx.strokeStyle=color;ctx.lineWidth=2;ctx.lineJoin='round';ctx.lineCap='round';ctx.beginPath();";
  html += "for(var j=0;j<xs.length;j++){var X=px(xs[j]),Y=py(ys[j]);if(j===0)ctx.moveTo(X,Y);else ctx.lineTo(X,Y);}ctx.stroke();";
  html += "var lx=px(xs[xs.length-1]),ly=py(ys[ys.length-1]);ctx.fillStyle=color;ctx.beginPath();ctx.arc(lx,ly,3,0,2*Math.PI);ctx.fill();";
  html += "canvas._chartData={xs:xs,ys:ys,minX:minX,maxX:maxX,padL:padL,plotW:plotW,unit:unit,decimals:decimals};}";

  html += "function chartHoverHandler(canvas,hoverEl){";
  html += "function handle(evt){var d=canvas._chartData;if(!d)return;";
  html += "var rect=canvas.getBoundingClientRect();var clientX=evt.touches?evt.touches[0].clientX:evt.clientX;";
  html += "var frac=(clientX-rect.left-d.padL)/d.plotW;frac=Math.max(0,Math.min(1,frac));";
  html += "var xVal=d.minX+frac*(d.maxX-d.minX);var idx=0,best=Infinity;";
  html += "for(var i=0;i<d.xs.length;i++){var diff=Math.abs(d.xs[i]-xVal);if(diff<best){best=diff;idx=i;}}";
  html += "hoverEl.textContent='T+'+fmtElapsed(d.xs[idx])+' -> '+d.ys[idx].toFixed(d.decimals)+' '+d.unit;}";
  html += "canvas.addEventListener('mousemove',handle);";
  html += "canvas.addEventListener('touchmove',function(e){handle(e);e.preventDefault();},{passive:false});";
  html += "canvas.addEventListener('mouseleave',function(){hoverEl.innerHTML='&nbsp;';});}";

  html += "var chartsInited=false;";
  html += "function initCharts(){if(chartsInited)return;chartsInited=true;";
  html += "chartHoverHandler(document.getElementById('chartAlt'),document.getElementById('chartAltHover'));";
  html += "chartHoverHandler(document.getElementById('chartTemp'),document.getElementById('chartTempHover'));";
  html += "chartHoverHandler(document.getElementById('chartHum'),document.getElementById('chartHumHover'));}";

  // Storico caricato una volta da /history (tenuto dall'RX): così un reload della pagina
  // non fa ripartire i grafici da vuoto, a differenza di uno storico tenuto solo nel browser.
  html += "function loadHistory(){return fetch('/history').then(r=>r.json()).then(data=>{";
  html += "hist.t=data.t.map(function(v){return v/1000;});hist.alt=data.alt;hist.temp=data.temp;hist.hum=data.hum;";
  html += "lastHistT=data.t.length?data.t[data.t.length-1]:-1;";
  html += "}).catch(()=>{});}";

  // --- Polling dati ---
  html += "function aggiorna(){fetch('/data').then(r=>r.json()).then(d=>{";
  html += "document.getElementById('relAlt').textContent=d.altitude.toFixed(1);";
  html += "document.getElementById('vspeed').textContent=(d.vSpeed>=0?'+':'')+d.vSpeed.toFixed(2);";
  html += "document.getElementById('maxAlt').textContent=d.maxAltitude.toFixed(1);";
  html += "document.getElementById('plateauAlt').textContent=d.plateauConfirmed?d.plateauAltitude.toFixed(1):'--';";
  html += "document.getElementById('temp').textContent=d.temperature.toFixed(1);";
  html += "document.getElementById('humidity').textContent=d.humidity.toFixed(1);";
  html += "document.getElementById('battVTop').textContent=(d.batteryMv/1000).toFixed(2);";
  html += "document.getElementById('rssi').textContent=d.rssi.toFixed(1);";
  html += "document.getElementById('snr').textContent=d.snr.toFixed(1);";
  html += "var q=linkQuality(d.rssi,d.snr);var qs=document.getElementById('sigQual');qs.textContent=q[0];qs.className='qual '+q[1];";
  html += "document.getElementById('pktRecv').textContent=d.packetsReceived;";
  html += "document.getElementById('lost').textContent=d.packetsLostEst;";
  html += "document.getElementById('age').textContent=(d.msSinceLastPacket/1000).toFixed(1);";
  html += "document.getElementById('cpuTempRX').textContent=d.cpuTempRX.toFixed(1);";
  html += "document.getElementById('clockT').textContent='T+'+fmtClock(d.elapsedMs/1000);";
  html += "var s=document.getElementById('status');var st=document.getElementById('statusText');var sub=document.getElementById('statusSub');";
  html += "if(!d.linkOk){s.className='annunciator state-bad';st.textContent='LINK PERSO';sub.textContent='Nessun pacchetto da oltre '+LINK_TIMEOUT_S+'s';setThemeColor('#FF4433');}";
  html += "else if(d.sinkAlarm){s.className='annunciator state-warn';st.textContent='ALLARME DISCESA';sub.textContent='Quota sotto la soglia di stabilizzazione';setThemeColor('#FFB020');}";
  html += "else{s.className='annunciator state-ok';st.textContent='LINK OK';sub.textContent='Telemetria in ricezione';setThemeColor('#2FE6A0');}";
  html += "updateTone(d.linkOk&&d.sinkAlarm,d.vSpeed);";
  html += "checkPlateauReached(d);";
  html += "checkLinkTransition(d);";
  html += "if(d.linkOk)pushHistory(d);";
  html += "drawChart(document.getElementById('chartAlt'),hist.t,hist.alt,'#4C8DFF','m',1);";
  html += "drawChart(document.getElementById('chartTemp'),hist.t,hist.temp,'#FF9F45','°C',1);";
  html += "drawChart(document.getElementById('chartHum'),hist.t,hist.hum,'#38D9F5','%',1);";
  html += "}).catch(()=>{});}";

  // 2s invece di 1s: il TX manda comunque ~1 pacchetto ogni 2s (vedi PacketFormat/TX), quindi
  // interrogare /data più spesso non porta dati più freschi ma raddoppia inutilmente i burst
  // di trasmissione Wi-Fi dell'RX, con relativo consumo (telefono già a un passo dall'AP).
  html += "setInterval(aggiorna,2000); window.onload=function(){initCharts();loadHistory().then(aggiorna);};";
  html += "</script></head><body>";

  html += "<div class='container'>";
  html += "  <div class='titlebar'>";
  html += "    <div class='title'>Kite Altitude<span class='sub'>Mission Control</span></div>";
  html += "    <div class='titlebar-actions'>";
  html += "      <button id='fsBtn' class='btn-ghost' onclick='toggleFullscreen()'>Schermo intero</button>";
  html += "      <button id='audioBtn' class='btn-audio' onclick='enableAudio()'>Attiva audio</button>";
  html += "    </div>";
  html += "  </div>";

  html += "  <div class='annunciator' id='status'>";
  html += "    <div class='annunciator-top'>";
  html += "      <div class='lamp-group'><div class='lamp' id='lamp'></div><div class='lamp-text'><span id='statusText'>--</span><span class='sub' id='statusSub'>In attesa di dati...</span></div></div>";
  html += "      <div class='clock'><div class='t' id='clockT'>T+00:00:00</div><div class='label'>SESSIONE RX</div></div>";
  html += "    </div>";
  html += "    <div class='annunciator-vitals'>";
  html += "      <div class='vital'><span class='label'>Segnale</span><span id='sigQual' class='qual'>--</span></div>";
  html += "      <div class='vital'><span class='label'>Batteria TX</span><span class='val'><span id='battVTop'>--</span><span class='unit'>V</span></span></div>";
  html += "    </div>";
  html += "    <details class='link-details'><summary>Dettagli RSSI/SNR</summary><p class='help'>RSSI: <span id='rssi'>--</span> dBm &middot; SNR: <span id='snr'>--</span> dB<br>Ottimo &ge;-80dBm e &ge;5dB &middot; Buono &ge;-100dBm e &ge;0dB &middot; sotto: Scarso</p></details>";
  html += "  </div>";

  html += "  <div class='panel'><div class='rivet-l'></div>";
  html += "    <div class='panel-title'>Telemetria di volo</div>";
  html += "    <div class='metrics'>";
  html += "      <div class='metric'><div class='label'>Altitudine relativa</div><div class='value'><span id='relAlt'>--</span><span class='unit'>m</span></div></div>";
  html += "      <div class='metric'><div class='label'>Velocità verticale</div><div class='value'><span id='vspeed'>--</span><span class='unit'>m/s</span></div></div>";
  html += "      <div class='metric'><div class='label'>Max sessione</div><div class='value accent-good'><span id='maxAlt'>--</span><span class='unit'>m</span></div></div>";
  html += "      <div class='metric'><div class='label'>Quota di stabilizzazione</div><div class='value'><span id='plateauAlt'>--</span><span class='unit'>m</span></div></div>";
  html += "      <div class='metric'><div class='label'>Temperatura</div><div class='value'><span id='temp'>--</span><span class='unit'>&deg;C</span></div></div>";
  html += "      <div class='metric'><div class='label'>Umidità</div><div class='value'><span id='humidity'>--</span><span class='unit'>%</span></div></div>";
  html += "    </div></div>";

  html += "  <div class='panel'><div class='rivet-l'></div><div class='panel-title'>Altitudine nel tempo</div>";
  html += "    <canvas id='chartAlt' class='chart'></canvas><div class='chart-hover' id='chartAltHover'>&nbsp;</div></div>";
  html += "  <div class='panel'><div class='rivet-l'></div><div class='panel-title'>Temperatura nel tempo</div>";
  html += "    <canvas id='chartTemp' class='chart'></canvas><div class='chart-hover' id='chartTempHover'>&nbsp;</div></div>";
  html += "  <div class='panel'><div class='rivet-l'></div><div class='panel-title'>Umidità nel tempo</div>";
  html += "    <canvas id='chartHum' class='chart'></canvas><div class='chart-hover' id='chartHumHover'>&nbsp;</div></div>";

  html += "  <details class='panel panel-collapse'>";
  html += "    <summary class='panel-title'>Diagnostica &amp; impostazioni</summary>";
  html += "    <div class='rivet-l'></div>";
  html += "    <div class='subhead'>Pacchetti</div>";
  html += "    <div class='metrics'>";
  html += "      <div class='metric'><div class='label'>Ricevuti / persi</div><div class='value' style='font-size:20px'><span id='pktRecv'>--</span> / <span id='lost'>--</span></div></div>";
  html += "      <div class='metric'><div class='label'>Ultimo pacchetto</div><div class='value' style='font-size:20px'><span id='age'>--</span><span class='unit'>s fa</span></div></div>";
  html += "    </div>";
  html += "    <div class='subhead'>Temperatura CPU RX</div>";
  html += "    <div class='metric'><div class='value'><span id='cpuTempRX'>--</span><span class='unit'>&deg;C</span></div></div>";
  html += "    <div class='subhead'>Impostazioni allarme discesa</div>";
  html += "    <form action='/setSettings' method='POST'>";
  html += "      <div class='setting'>";
  html += "        <span class='name'>Tolleranza quota di stabilizzazione (m)</span>";
  html += "        <span class='sub'>Quanto può oscillare la quota restando comunque considerata \"stabile\". Più bassa = quota di stabilizzazione più sensibile alle piccole variazioni.</span>";
  html += "        <input type='number' step='0.1' name='tol' value='" + String(plateauToleranceM, 1) + "'>";
  html += "      </div>";
  html += "      <div class='setting'>";
  html += "        <span class='name'>Tempo di stabilità (s)</span>";
  html += "        <span class='sub'>Per quanto tempo la quota deve restare dentro la tolleranza prima che la quota di stabilizzazione venga confermata come riferimento.</span>";
  html += "        <input type='number' step='1' name='hold' value='" + String(plateauHoldMs / 1000.0f, 0) + "'>";
  html += "      </div>";
  html += "      <div class='setting'>";
  html += "        <span class='name'>Soglia di discesa (m)</span>";
  html += "        <span class='sub'>Di quanti metri sotto la quota di stabilizzazione confermata l'aquilone deve scendere prima che scatti l'allarme.</span>";
  html += "        <input type='number' step='0.1' name='trig' value='" + String(descentTriggerM, 1) + "'>";
  html += "      </div>";
  html += "      <div class='setting'>";
  html += "        <span class='name'>Timeout link (s)</span>";
  html += "        <span class='sub'>Se non arriva nessun pacchetto dal TX per più di questo tempo, il link è considerato perso (stato rosso) e l'allarme si azzera.</span>";
  html += "        <input type='number' step='1' name='linktimeout' value='" + String(linkTimeoutMs / 1000.0f, 0) + "'>";
  html += "      </div>";
  html += "      <div class='setting'>";
  html += "        <span class='name'>Campionamento grafici (s)</span>";
  html += "        <span class='sub'>Ogni quanto viene salvato un punto nei grafici storici (altitudine/temperatura/umidità). Più basso = grafici più dettagliati ma storico più corto a parità di memoria.</span>";
  html += "        <input type='number' step='1' name='histsample' value='" + String(historySampleMs / 1000.0f, 0) + "'>";
  html += "      </div>";
  html += "      <div class='btn-row'><input type='submit' class='btn' value='Salva'><a href='/resetAlarm'><button type='button' class='btn-ghost'>Reset allarme</button></a></div>";
  html += "    </form>";
  html += "  </details>";

  html += "</div></body></html>";
  cachedHtml = html;
}

void handleRoot() {
  if (htmlDirty) { buildHtml(); htmlDirty = false; }
  server.send(200, "text/html", cachedHtml);
}

void handleData() {
  uint32_t now = millis();
  bool linkOk = (lastPacketMs != 0) && (now - lastPacketMs < (uint32_t)linkTimeoutMs);
  uint32_t msSince = lastPacketMs != 0 ? (now - lastPacketMs) : 0;

  String json = "{";
  json += "\"altitude\":" + String(g_altitude, 2) + ",";
  json += "\"maxAltitude\":" + String(g_maxAltitude, 2) + ",";
  json += "\"temperature\":" + String(g_temperature, 1) + ",";
  json += "\"humidity\":" + String(g_humidity, 1) + ",";
  json += "\"vSpeed\":" + String(g_vSpeed, 2) + ",";
  json += "\"seq\":" + String(lastSeq) + ",";
  json += "\"rssi\":" + String(g_rssi, 1) + ",";
  json += "\"snr\":" + String(g_snr, 1) + ",";
  json += "\"batteryMv\":" + String((int)g_batteryMv) + ",";
  json += "\"linkOk\":" + String(linkOk ? "true" : "false") + ",";
  json += "\"msSinceLastPacket\":" + String(msSince) + ",";
  json += "\"packetsReceived\":" + String(packetsReceived) + ",";
  json += "\"packetsLostEst\":" + String(packetsLostEst) + ",";
  json += "\"plateauAltitude\":" + String(plateauAltitude, 2) + ",";
  json += "\"plateauConfirmed\":" + String(plateauConfirmed ? "true" : "false") + ",";
  json += "\"sinkAlarm\":" + String((sinkAlarm && linkOk) ? "true" : "false") + ",";
  json += "\"cpuTempRX\":" + String(g_cpuTempRX, 1) + ",";
  json += "\"elapsedMs\":" + String(now);
  json += "}";
  server.send(200, "application/json", json);
}

// Storico completo per i grafici (chiamato una volta al caricamento pagina/reload, non in polling):
// il browser lo usa per ripopolare i grafici, poi continua ad accodare i nuovi punti da /data.
void handleHistory() {
  uint16_t start = (histCount < HISTORY_CAPACITY) ? 0 : histHead;

  String json;
  json.reserve(24 + (size_t)histCount * 34);
  json += "{\"t\":[";
  for (uint16_t i = 0; i < histCount; i++) {
    if (i) json += ",";
    json += String(histT[(start + i) % HISTORY_CAPACITY]);
  }
  json += "],\"alt\":[";
  for (uint16_t i = 0; i < histCount; i++) {
    if (i) json += ",";
    json += String(histAlt[(start + i) % HISTORY_CAPACITY], 2);
  }
  json += "],\"temp\":[";
  for (uint16_t i = 0; i < histCount; i++) {
    if (i) json += ",";
    json += String(histTemp[(start + i) % HISTORY_CAPACITY], 1);
  }
  json += "],\"hum\":[";
  for (uint16_t i = 0; i < histCount; i++) {
    if (i) json += ",";
    json += String(histHum[(start + i) % HISTORY_CAPACITY], 1);
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleSetSettings() {
  if (server.hasArg("tol")) {
    float v = server.arg("tol").toFloat();
    if (v >= 0.1f && v <= 10.0f) { plateauToleranceM = v; saveFloatAt(0, v); }
  }
  if (server.hasArg("trig")) {
    float v = server.arg("trig").toFloat();
    if (v >= 0.5f && v <= 50.0f) { descentTriggerM = v; saveFloatAt(4, v); }
  }
  if (server.hasArg("hold")) {
    float ms = server.arg("hold").toFloat() * 1000.0f;
    if (ms >= 500.0f && ms <= 30000.0f) { plateauHoldMs = ms; saveFloatAt(8, ms); }
  }
  if (server.hasArg("linktimeout")) {
    float ms = server.arg("linktimeout").toFloat() * 1000.0f;
    if (ms >= 1000.0f && ms <= 60000.0f) { linkTimeoutMs = ms; saveFloatAt(12, ms); }
  }
  if (server.hasArg("histsample")) {
    float ms = server.arg("histsample").toFloat() * 1000.0f;
    if (ms >= 1000.0f && ms <= 60000.0f) { historySampleMs = ms; saveFloatAt(16, ms); }
  }
  htmlDirty = true; // le impostazioni appena salvate sono incorporate nella pagina in cache
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleResetAlarm() {
  resetVarioState();
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Riduce il riscaldamento del SoC in AP mode continuo: 80MHz è il minimo supportato
  // col Wi-Fi attivo, ma è ampiamente sufficiente per il carico di questo sketch.
  setCpuFrequencyMhz(80);

  EEPROM.begin(EEPROM_SIZE);
  plateauToleranceM = loadFloatAt(0, 0.1f, 10.0f, 1.0f);
  descentTriggerM   = loadFloatAt(4, 0.5f, 50.0f, 3.0f);
  plateauHoldMs     = loadFloatAt(8, 500.0f, 30000.0f, 8000.0f);
  linkTimeoutMs     = loadFloatAt(12, 1000.0f, 60000.0f, 10000.0f);
  historySampleMs   = loadFloatAt(16, 1000.0f, 60000.0f, 10000.0f);

  SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_CS_PIN);
  int state = radio.begin(LORA_FREQUENCY_MHZ, LORA_BANDWIDTH_KHZ, LORA_SPREADING_FACTOR,
                           LORA_CODING_RATE, LORA_SYNC_WORD, LORA_TX_POWER_DBM, LORA_PREAMBLE_LENGTH);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("Errore inizializzazione radio SX1276: %d\n", state);
  }
  radio.setCRC(true);
  radio.setDio0Action(setFlag, RISING);
  radio.startReceive();

  WiFi.softAP(ssid, password);
  // Potenza TX ridotta al minimo utile: l'AP serve un telefono praticamente attaccato all'RX,
  // non serve la potenza RF di default né i 5dBm usati finché il consumo non era un vincolo
  // stringente (RX ora spesso a batteria, non solo USB/power bank).
  WiFi.setTxPower(WIFI_POWER_2dBm);
  Serial.print("Access Point creato. IP: ");
  Serial.println(WiFi.softAPIP());

  // Captive portal: redirige tutte le risoluzioni DNS all'IP dell'AP, così i sistemi operativi
  // (Android/iOS/Windows) rilevano una rete "con login" e aprono la pagina da soli.
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/history", handleHistory);
  server.on("/setSettings", HTTP_POST, handleSetSettings);
  server.on("/resetAlarm", handleResetAlarm);
  server.onNotFound(handleRoot); // qualunque URL di probe del captive portal serve comunque la pagina
  server.begin();
  Serial.println("Server web avviato");
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();

  if (packetFlag) {
    packetFlag = false;
    size_t len = radio.getPacketLength();
    if (len == sizeof(TelemetryPacket)) {
      TelemetryPacket pkt;
      int state = radio.readData((uint8_t*)&pkt, len);
      if (state == RADIOLIB_ERR_NONE && pkt.magic == PACKET_MAGIC) {
        uint32_t now = millis();
        // Se il TX si riavvia, il suo millis() (txUptimeMs) torna a un valore basso: è un
        // segnale inequivocabile di riavvio, a differenza del solo seq (che potrebbe anche
        // sembrare un wraparound normale). In quel caso non contiamo il gap come persi:
        // sono pacchetti mai esistiti, non pacchetti persi in aria.
        bool txRestarted = lastSeqValid && (pkt.txUptimeMs < lastTxUptimeMs);
        if (lastSeqValid && !txRestarted) {
          uint16_t gap = (uint16_t)(pkt.seq - lastSeq - 1);
          packetsLostEst += gap;
        }
        lastSeq = pkt.seq; lastSeqValid = true;
        lastTxUptimeMs = pkt.txUptimeMs;
        packetsReceived++;
        lastPacketMs = now;
        g_rssi = radio.getRSSI();
        g_snr = radio.getSNR();
        g_batteryMv = pkt.batteryMv;
        g_altitude = pkt.relAltitude_mm / 1000.0f;
        if (g_altitude > g_maxAltitude) g_maxAltitude = g_altitude;
        g_temperature = pkt.temperature_c10 / 10.0f;
        g_humidity = pkt.humidity_pct10 / 10.0f;

        pushVSpeedSample(now, g_altitude);
        updatePlateauAndAlarm(now, g_altitude);
        if (!historyStarted || (now - lastHistPushMs) >= (uint32_t)historySampleMs) {
          pushHistorySample(now, g_altitude, g_temperature, g_humidity);
          lastHistPushMs = now;
          historyStarted = true;
        }

        Serial.printf("Temperatura: %.1f C\n", g_temperature);
      }
    }
    radio.startReceive();
  }

  bool linkOk = (lastPacketMs != 0) && (millis() - lastPacketMs < (uint32_t)linkTimeoutMs);
  if (!linkOk) sinkAlarm = false; // dato stantio non deve mai far suonare l'allarme

  uint32_t nowMs = millis();
  if (nowMs - lastCpuTempMs >= 1000) {
    lastCpuTempMs = nowMs;
    float raw = temperatureRead();
    g_cpuTempRX = cpuTempRXInit ? (g_cpuTempRX * 0.7f + raw * 0.3f) : raw;
    cpuTempRXInit = true;
  }

  // Il TX manda ~1 pacchetto/s e la pagina web fa polling ogni 1s: un loop a 250Hz (delay 2ms)
  // è molto più veloce del necessario e tiene la CPU sveglia inutilmente, scaldando il SoC
  // anche senza nessun client Wi-Fi connesso. 20ms (50Hz) lascia comunque ampio margine sia
  // per la ricezione LoRa (il pacchetto resta nel buffer finché non lo si legge) sia per la
  // reattività percepita dell'AP/captive portal.
  delay(20);
}
