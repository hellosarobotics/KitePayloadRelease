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
float g_temperature = 0;
float g_humidity = 0;
float g_batteryMv = 0;
float g_rssi = 0, g_snr = 0;

// ======= Temperatura CPU RX: campionata 1x/s + EMA (il sensore interno ha step di ~1°C) =======
float g_cpuTempRX = 0;
bool cpuTempRXInit = false;
uint32_t lastCpuTempMs = 0;

// ======= Velocità verticale: finestra di 4 campioni + EMA =======
#define VSPEED_WINDOW 4
uint32_t vsTimeMs[VSPEED_WINDOW];
float vsAlt[VSPEED_WINDOW];
uint8_t vsIdx = 0;
uint8_t vsCount = 0;
float g_vSpeed = 0;

// ======= Storico per i grafici web: buffer circolare lato RX, così un reload della pagina
// (che azzererebbe uno storico tenuto solo nel browser) non perde i dati dall'accensione del TX.
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
uint32_t sinkAlarmSinceMs = 0;
uint32_t alarmRecoverSinceMs = 0;
const float ALARM_RECOVER_VSPEED = -0.3f;
const uint32_t RECOVER_HOLD_MS = 2000;
const uint32_t MIN_ALARM_DURATION_MS = 10000; // una volta scattato, l'allarme suona almeno questo tempo prima di potersi spegnere

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
  float rawSpeed = (alt - vsAlt[oldestIdx]) / dt;
  g_vSpeed = 0.7f * g_vSpeed + 0.3f * rawSpeed;
}

// ---------------------- Macchina a stati: plateau + allarme discesa ----------------------
void updatePlateauAndAlarm(uint32_t t, float alt) {
  if (!plateauConfirmed || fabs(alt - plateauAltitude) <= plateauToleranceM) {
    plateauAltitude = plateauAltitude * 0.8f + alt * 0.2f;
    if (plateauStableSinceMs == 0) plateauStableSinceMs = t;
    if (t - plateauStableSinceMs >= (uint32_t)plateauHoldMs) plateauConfirmed = true;
  } else if (alt > plateauAltitude) {
    // salita: nuovo candidato plateau più in alto
    plateauAltitude = alt; plateauStableSinceMs = t; plateauConfirmed = false;
  } else {
    // sceso oltre la tolleranza rispetto al plateau
    plateauStableSinceMs = t;
    if (plateauConfirmed && (plateauAltitude - alt) > descentTriggerM) {
      if (!sinkAlarm) sinkAlarmSinceMs = t;
      sinkAlarm = true;
    }
  }

  if (sinkAlarm) {
    if (g_vSpeed > ALARM_RECOVER_VSPEED) {
      if (alarmRecoverSinceMs == 0) alarmRecoverSinceMs = t;
      // Oltre alla ripresa sostenuta (sale o si stabilizza), l'allarme deve comunque essere
      // suonato per almeno MIN_ALARM_DURATION_MS dall'attivazione, anche se la ripresa è
      // immediata: altrimenti un rientro rapidissimo lo spegnerebbe prima che si senta.
      bool recoveredLongEnough = (t - alarmRecoverSinceMs) >= RECOVER_HOLD_MS;
      bool minDurationElapsed = (t - sinkAlarmSinceMs) >= MIN_ALARM_DURATION_MS;
      if (recoveredLongEnough && minDurationElapsed) {
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
  sinkAlarmSinceMs = 0;
  alarmRecoverSinceMs = 0;
  vsIdx = 0; vsCount = 0; g_vSpeed = 0;
}

// ---------------------- WEB UI ----------------------
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<meta name='theme-color' content='#22c55e' id='themeColor'>";
  html += "<style>";
  html += "body{margin:0;font-family:system-ui,-apple-system,Segoe UI,Roboto,Ubuntu,Arial;background:#ffffff;color:#111827;}";
  html += ".container{max-width:760px;margin:28px auto;padding:0 16px;}";
  html += ".header{display:flex;align-items:center;justify-content:space-between;margin-bottom:14px;flex-wrap:wrap;gap:10px}";
  html += ".title{font-size:20px;font-weight:700;letter-spacing:.3px}";
  html += ".status{padding:10px 14px;border-radius:10px;color:#fff;font-weight:700;min-width:200px;text-align:center;}";
  html += ".status.ok{background:#22c55e;}";
  html += ".status.warn{background:#f59e0b;}";
  html += ".status.bad{background:#ef4444;}";
  html += ".stack{display:flex;flex-direction:column;gap:14px;margin-top:14px;}";
  html += ".card{background:#f9fafb;border:1px solid #e5e7eb;box-shadow:0 2px 6px rgba(0,0,0,0.08);border-radius:12px;padding:16px 16px 14px}";
  html += ".metrics{display:flex;gap:24px;flex-wrap:wrap}";
  html += ".metric{flex:1 1 140px;min-width:140px}";
  html += ".label{color:#6b7280;font-size:12px;letter-spacing:.5px;text-transform:uppercase}";
  html += ".value{color:#111827;font-weight:800;font-size:26px;line-height:1.1;margin-top:6px}";
  html += ".unit{opacity:.65;font-weight:600;font-size:16px;margin-left:6px}";
  html += ".sub{color:#6b7280;font-size:12px;margin-top:8px}";
  html += "details.link-details{margin-top:8px}";
  html += "details.link-details summary{cursor:pointer;color:#3b82f6;font-size:12px;font-weight:600;list-style:none}";
  html += "details.link-details summary::-webkit-details-marker{display:none}";
  html += "details.link-details summary:before{content:'▸ ';}";
  html += "details.link-details[open] summary:before{content:'▾ ';}";
  html += "details.link-details .sub{margin-top:6px}";
  html += ".row{display:flex;gap:10px;flex-wrap:wrap;margin-top:16px}";
  html += ".btn,.btn-ghost{appearance:none;border:none;cursor:pointer;border-radius:10px;padding:10px 14px;font-weight:700}";
  html += ".btn{color:#fff;background:#3b82f6} .btn:hover{background:#2563eb}";
  html += ".btn-ghost{background:#fff;border:1px solid #d1d5db;color:#111827} .btn-ghost:hover{background:#f3f4f6}";
  html += "form{display:flex;align-items:flex-end;gap:16px;flex-wrap:wrap;margin-top:12px}";
  html += "input[type=number]{background:#fff;border:1px solid #d1d5db;color:#111827;border-radius:8px;padding:8px 10px;min-width:110px}";
  html += "input[type=submit]{border:none;border-radius:8px;padding:10px 14px;background:#3b82f6;color:#fff;font-weight:700;cursor:pointer}";
  html += "input[type=submit]:hover{background:#2563eb}";
  html += ".setting{display:flex;flex-direction:column;gap:4px;flex:1 1 220px;min-width:220px}";
  html += ".setting .name{font-weight:600;font-size:13px}";
  html += ".setting .sub{margin-top:0}";
  html += ".qual{font-size:13px;font-weight:700;padding:2px 9px;border-radius:6px;margin-left:8px;vertical-align:middle}";
  html += ".qual.ottimo{background:#dcfce7;color:#15803d}";
  html += ".qual.buono{background:#dbeafe;color:#1d4ed8}";
  html += ".qual.scarso{background:#fee2e2;color:#b91c1c}";
  html += "canvas.chart{width:100%;height:160px;display:block;touch-action:pan-y}";
  html += ".chart-hover{font-size:12px;color:#6b7280;margin-top:6px;min-height:16px}";
  html += "</style>";

  html += "<script>";
  html += "function setThemeColor(hex){var m=document.getElementById('themeColor'); if(m){m.setAttribute('content',hex);} }";

  // --- Web Audio: tono continuo di allarme (niente più beep intermittente: l'RX è già
  // l'autorità su quando l'allarme deve suonare/spegnersi, incluso il minimo di 10s
  // all'attivazione — qui ci si limita a riprodurre lo stato che arriva da /data). ---
  html += "let audioCtx=null,osc=null,gainNode=null,audioEnabled=false;";
  html += "function enableAudio(){";
  html += "if(!audioCtx){audioCtx=new (window.AudioContext||window.webkitAudioContext)();";
  html += "osc=audioCtx.createOscillator();gainNode=audioCtx.createGain();gainNode.gain.value=0;osc.frequency.value=600;";
  html += "osc.connect(gainNode);gainNode.connect(audioCtx.destination);osc.start();}";
  html += "audioCtx.resume();audioEnabled=true;";
  html += "var b=document.getElementById('audioBtn');b.textContent='Audio attivo';b.disabled=true;}";

  html += "function updateTone(sinking,vspeed){";
  html += "if(!audioEnabled)return;";
  html += "if(!sinking){gainNode.gain.setTargetAtTime(0,audioCtx.currentTime,0.1);return;}";
  html += "var absV=Math.min(Math.abs(vspeed),6);";
  html += "var freq=Math.max(150,Math.min(600,600-80*absV));";
  html += "osc.frequency.setTargetAtTime(freq,audioCtx.currentTime,0.05);";
  html += "gainNode.gain.setTargetAtTime(0.15,audioCtx.currentTime,0.05);}";

  // --- Qualità link da RSSI/SNR: prende il peggiore dei due giudizi (un solo numero buono
  // non basta se l'altro è scarso) ---
  html += "function linkQuality(rssi,snr){";
  html += "var r=rssi>=-80?3:rssi>=-100?2:rssi>=-115?1:0;";
  html += "var s=snr>=5?3:snr>=0?2:snr>=-10?1:0;";
  html += "var lvl=Math.min(r,s);";
  html += "return [['Scarso','scarso'],['Scarso','scarso'],['Buono','buono'],['Ottimo','ottimo']][lvl];}";

  // --- Grafici storico (altitudine/temperatura/umidità nel tempo dall'accensione del TX) ---
  // hist viene ripopolato da /history al caricamento pagina (buffer autoritativo tenuto
  // dall'RX) e poi accodato qui coi nuovi punti dal polling /data: stesso rateo di
  // campionamento di 10s dell'RX, così il grafico resta uniforme dallo storico alla coda live.
  html += "var CH_MAX=1800;";
  html += "var HIST_SAMPLE_MS=" + String(historySampleMs, 0) + ";";
  html += "var hist={t:[],alt:[],temp:[],hum:[]};";
  html += "var lastHistT=-1;";
  html += "function pushHistory(d){";
  html += "if(lastHistT>=0&&d.txUptimeMs-lastHistT<HIST_SAMPLE_MS)return;";
  html += "lastHistT=d.txUptimeMs;";
  html += "hist.t.push(d.txUptimeMs/1000);hist.alt.push(d.altitude);hist.temp.push(d.temperature);hist.hum.push(d.humidity);";
  html += "if(hist.t.length>CH_MAX){hist.t.shift();hist.alt.shift();hist.temp.shift();hist.hum.shift();}}";

  html += "function fmtElapsed(s){s=Math.max(0,Math.round(s));";
  html += "var h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sec=s%60;";
  html += "var mm=(h>0?String(m).padStart(2,'0'):String(m));";
  html += "return (h>0?h+':':'')+mm+':'+String(sec).padStart(2,'0');}";

  html += "function drawChart(canvas,xs,ys,color,unit,decimals){";
  html += "var dpr=window.devicePixelRatio||1;var w=canvas.clientWidth,h=canvas.clientHeight;";
  html += "if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);}";
  html += "var ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);";
  html += "var padL=38,padR=8,padT=10,padB=20;var plotW=w-padL-padR,plotH=h-padT-padB;";
  html += "if(xs.length<2){ctx.fillStyle='#9ca3af';ctx.font='12px system-ui';ctx.textAlign='center';ctx.fillText('In attesa di dati...',w/2,h/2);canvas._chartData=null;return;}";
  html += "var minY=Math.min.apply(null,ys),maxY=Math.max.apply(null,ys);if(minY===maxY){minY-=1;maxY+=1;}";
  html += "var pad=(maxY-minY)*0.1;minY-=pad;maxY+=pad;";
  html += "var minX=xs[0],maxX=xs[xs.length-1];";
  html += "function px(x){return padL+(x-minX)/((maxX-minX)||1)*plotW;}";
  html += "function py(y){return padT+plotH-(y-minY)/(maxY-minY)*plotH;}";
  html += "ctx.strokeStyle='#e5e7eb';ctx.lineWidth=1;ctx.fillStyle='#9ca3af';ctx.font='10px system-ui';ctx.textAlign='right';ctx.textBaseline='middle';";
  html += "for(var i=0;i<=2;i++){var v=minY+(maxY-minY)*i/2;var yy=py(v);ctx.beginPath();ctx.moveTo(padL,yy);ctx.lineTo(w-padR,yy);ctx.stroke();ctx.fillText(v.toFixed(decimals),padL-6,yy);}";
  html += "ctx.fillStyle='#9ca3af';ctx.textBaseline='alphabetic';ctx.textAlign='left';ctx.fillText(fmtElapsed(minX),padL,h-4);ctx.textAlign='right';ctx.fillText(fmtElapsed(maxX),w-padR,h-4);";
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
  html += "document.getElementById('plateauAlt').textContent=d.plateauConfirmed?d.plateauAltitude.toFixed(1):'--';";
  html += "document.getElementById('temp').textContent=d.temperature.toFixed(1);";
  html += "document.getElementById('humidity').textContent=d.humidity.toFixed(1);";
  html += "document.getElementById('battV').textContent=(d.batteryMv/1000).toFixed(2);";
  html += "document.getElementById('rssi').textContent=d.rssi.toFixed(1);";
  html += "document.getElementById('snr').textContent=d.snr.toFixed(1);";
  html += "var q=linkQuality(d.rssi,d.snr);var qe=document.getElementById('linkQual');qe.textContent=q[0];qe.className='qual '+q[1];";
  html += "document.getElementById('pktRecv').textContent=d.packetsReceived;";
  html += "document.getElementById('lost').textContent=d.packetsLostEst;";
  html += "document.getElementById('age').textContent=(d.msSinceLastPacket/1000).toFixed(1);";
  html += "document.getElementById('cpuTempRX').textContent=d.cpuTempRX.toFixed(1);";
  html += "var s=document.getElementById('status');";
  html += "if(!d.linkOk){s.className='status bad';s.textContent='LINK PERSO';setThemeColor('#ef4444');}";
  html += "else if(d.sinkAlarm){s.className='status warn';s.textContent='ALLARME DISCESA';setThemeColor('#f59e0b');}";
  html += "else{s.className='status ok';s.textContent='Link OK';setThemeColor('#22c55e');}";
  html += "updateTone(d.linkOk&&d.sinkAlarm,d.vSpeed);";
  html += "if(d.linkOk)pushHistory(d);";
  html += "drawChart(document.getElementById('chartAlt'),hist.t,hist.alt,'#3b82f6','m',1);";
  html += "drawChart(document.getElementById('chartTemp'),hist.t,hist.temp,'#f97316','°C',1);";
  html += "drawChart(document.getElementById('chartHum'),hist.t,hist.hum,'#06b6d4','%',1);";
  html += "}).catch(()=>{});}";

  html += "setInterval(aggiorna,1000); window.onload=function(){initCharts();loadHistory().then(aggiorna);};";
  html += "</script></head><body>";

  html += "<div class='container'>";
  html += "  <div class='header'>";
  html += "    <div class='title'>Kite Altitude</div>";
  html += "    <button id='audioBtn' class='btn-ghost' onclick='enableAudio()'>Attiva audio</button>";
  html += "    <div id='status' class='status'>--</div>";
  html += "  </div>";

  html += "  <div class='stack'>";

  html += "    <div class='card'><div class='metrics'>";
  html += "      <div class='metric'><div class='label'>Altitudine relativa</div><div class='value'><span id='relAlt'>--</span><span class='unit'>m</span></div></div>";
  html += "      <div class='metric'><div class='label'>Velocità verticale</div><div class='value'><span id='vspeed'>--</span><span class='unit'>m/s</span></div></div>";
  html += "    </div></div>";

  html += "    <div class='card'><div class='label'>Quota di stabilizzazione</div><div class='value'><span id='plateauAlt'>--</span><span class='unit'>m</span></div><div class='sub'>Quota a cui l'aquilone si è stabilizzato abbastanza a lungo da fare da riferimento: se poi scende oltre la soglia impostata rispetto a questo valore, scatta l'allarme di discesa. Resta '--' finché la stabilità richiesta non è confermata.</div></div>";

  html += "    <div class='card'><div class='metrics'>";
  html += "      <div class='metric'><div class='label'>Temperatura</div><div class='value'><span id='temp'>--</span><span class='unit'>&deg;C</span></div></div>";
  html += "      <div class='metric'><div class='label'>Umidità</div><div class='value'><span id='humidity'>--</span><span class='unit'>%</span></div></div>";
  html += "    </div></div>";

  html += "    <div class='card'><div class='metrics'>";
  html += "      <div class='metric'><div class='label'>Batteria TX</div><div class='value'><span id='battV'>--</span><span class='unit'>V</span></div></div>";
  html += "      <div class='metric'><div class='label'>Link</div><div class='value'><span id='linkQual' class='qual'>--</span></div>";
  html += "        <details class='link-details'><summary>Dettagli RSSI/SNR</summary><div class='sub'>RSSI: <span id='rssi'>--</span> dBm &middot; SNR: <span id='snr'>--</span> dB<br>Ottimo &ge;-80dBm e &ge;5dB &middot; Buono &ge;-100dBm e &ge;0dB &middot; sotto: Scarso</div></details>";
  html += "      </div>";
  html += "    </div></div>";

  html += "    <div class='card'><div class='metrics'>";
  html += "      <div class='metric'><div class='label'>Pacchetti (ricevuti / persi)</div><div class='value'><span id='pktRecv'>--</span> / <span id='lost'>--</span></div></div>";
  html += "      <div class='metric'><div class='label'>Ultimo pacchetto ricevuto</div><div class='value'><span id='age'>--</span><span class='unit'>s fa</span></div></div>";
  html += "    </div></div>";

  html += "    <div class='card'><div class='label'>Temperatura CPU RX</div><div class='value'><span id='cpuTempRX'>--</span><span class='unit'>&deg;C</span></div></div>";

  html += "  </div>";

  html += "  <div class='card' style='margin-top:14px'>";
  html += "    <div class='label'>Impostazioni allarme discesa</div>";
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
  html += "      <input type='submit' value='Salva'>";
  html += "    </form>";
  html += "    <div class='row'>";
  html += "      <a href='/resetAlarm'><button class='btn-ghost'>Reset allarme</button></a>";
  html += "    </div>";
  html += "  </div>";

  html += "  <div class='card' style='margin-top:14px'><div class='label'>Altitudine nel tempo</div>";
  html += "    <canvas id='chartAlt' class='chart'></canvas><div class='chart-hover' id='chartAltHover'>&nbsp;</div></div>";
  html += "  <div class='card' style='margin-top:14px'><div class='label'>Temperatura nel tempo</div>";
  html += "    <canvas id='chartTemp' class='chart'></canvas><div class='chart-hover' id='chartTempHover'>&nbsp;</div></div>";
  html += "  <div class='card' style='margin-top:14px'><div class='label'>Umidità nel tempo</div>";
  html += "    <canvas id='chartHum' class='chart'></canvas><div class='chart-hover' id='chartHumHover'>&nbsp;</div></div>";

  html += "</div></body></html>";
  server.send(200, "text/html", html);
}

void handleData() {
  uint32_t now = millis();
  bool linkOk = (lastPacketMs != 0) && (now - lastPacketMs < (uint32_t)linkTimeoutMs);
  uint32_t msSince = lastPacketMs != 0 ? (now - lastPacketMs) : 0;

  String json = "{";
  json += "\"altitude\":" + String(g_altitude, 2) + ",";
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
  json += "\"txUptimeMs\":" + String(lastTxUptimeMs);
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
  // Potenza TX ridotta: l'AP serve un telefono a pochi metri, non serve la potenza RF massima
  // di default (che è una delle cause principali del riscaldamento del SoC in AP continuo).
  WiFi.setTxPower(WIFI_POWER_5dBm);
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
        g_temperature = pkt.temperature_c10 / 10.0f;
        g_humidity = pkt.humidity_pct10 / 10.0f;

        pushVSpeedSample(now, g_altitude);
        updatePlateauAndAlarm(now, g_altitude);
        if (!historyStarted || (now - lastHistPushMs) >= (uint32_t)historySampleMs) {
          pushHistorySample(pkt.txUptimeMs, g_altitude, g_temperature, g_humidity);
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
