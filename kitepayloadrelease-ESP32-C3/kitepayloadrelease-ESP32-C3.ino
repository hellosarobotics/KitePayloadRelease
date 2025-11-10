#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP32Servo.h>
#include <EEPROM.h>

// ======= Wi-Fi AP =======
const char* ssid = "KiteRelease";
const char* password = "12345678";

WebServer server(80);
Servo myservo;
Adafruit_BME280 bme;

// ======= EEPROM =======
#define EEPROM_SIZE 8
float seaLevelPressure_hpa = 1013.25;
float releaseAltitude = 0;

// ======= Stato altitudine =======
float baseAltitude = 0;
bool payloadReleased = false;
bool bmeAvailable = false;

// ======= Cache letture (UI + logica) =======
float g_temperature = 0;
float g_humidity = 0;
float g_pressure = 0;
float g_altitude = 0;
float g_relAltitude = 0;
float g_maxRelAltitude = 0;
float relAltUsedAtRelease = NAN;

// ======= Filtri =======
const float ALT_ALPHA = 0.5f;
bool altFilterInit = false;

const uint32_t BME_INTERVAL_MS = 1000;
uint32_t lastBmeRead = 0;

const float SPIKE_THRESHOLD_M = 5.0f;
const float MAX_STEP_M        = 2.0f;

const uint8_t MEDIAN_N = 3;
float relAltWindow[MEDIAN_N];
uint8_t relAltIdx = 0;
bool relAltFilled = false;

uint32_t aboveThresholdSince = 0;
const uint32_t HYSTERESIS_MS = 1000;

// ======= SERVO =======
#define SERVO_PIN 6

// ---------------------- Utils EEPROM ----------------------
void saveSeaLevelPressure(float value) {
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < 4; i++) EEPROM.write(i, *p++);
  EEPROM.commit();
}

float loadSeaLevelPressure() {
  float value;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < 4; i++) *p++ = EEPROM.read(i);
  if (isnan(value) || value < 300 || value > 1100) return 1013.25;
  return value;
}

void saveReleaseAltitude(float value) {
  byte* p = (byte*)(void*)&value;
  for (int i = 4; i < 8; i++) EEPROM.write(i, *p++);
  EEPROM.commit();
}

float loadReleaseAltitude() {
  float value;
  byte* p = (byte*)(void*)&value;
  for (int i = 4; i < 8; i++) *p++ = EEPROM.read(i);
  if (isnan(value) || value < 0 || value > 10000) return 0;
  return value;
}

// ---------------------- Utils filtro ----------------------
static float clamp(float x, float lo, float hi){ return x < lo ? lo : (x > hi ? hi : x); }

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
  if (n & 1) return tmp[n/2];
  return 0.5f * (tmp[n/2 - 1] + tmp[n/2]);
}

void pushWindow(float v) {
  relAltWindow[relAltIdx++] = v;
  if (relAltIdx >= MEDIAN_N) { relAltIdx = 0; relAltFilled = true; }
}

// ---------------------- SERVO ----------------------
void RELEASE() {
  Serial.println("Rilascio (RELEASE)");
  myservo.write(180);
  delay(500);
  myservo.write(0);
}

// ---------------------- WEB UI ----------------------
void handleRoot();
void handleData();
void handleRelease();
void handleSetPressure();
void handleSetReleaseAltitude();
void handleResetRelease();

// ====================== SETUP ======================
void setup() {
  Serial.begin(115200);
  myservo.attach(SERVO_PIN, 500, 2400);
  myservo.write(0);

  EEPROM.begin(EEPROM_SIZE);
  seaLevelPressure_hpa = loadSeaLevelPressure();
  releaseAltitude = loadReleaseAltitude();

  Serial.print("Pressione livello mare iniziale: ");
  Serial.println(seaLevelPressure_hpa);
  Serial.print("Quota sgancio iniziale: ");
  Serial.println(releaseAltitude);

  Wire.begin(8, 9);
  if (bme.begin(0x76, &Wire) || bme.begin(0x77, &Wire)) {
    bmeAvailable = true;
    Serial.println("BME280 trovato!");
  } else {
    Serial.println("ATTENZIONE: BME280 non trovato!");
  }

  baseAltitude = bmeAvailable ? bme.readAltitude(seaLevelPressure_hpa) : 0;

  if (bmeAvailable) {
    g_temperature = bme.readTemperature();
    g_humidity    = bme.readHumidity();
    g_pressure    = bme.readPressure() / 100.0F;
    g_altitude    = bme.readAltitude(seaLevelPressure_hpa);
    g_relAltitude = g_altitude - baseAltitude;
    g_maxRelAltitude = g_relAltitude;
    altFilterInit = true;
    for (uint8_t i=0;i<MEDIAN_N;i++) relAltWindow[i] = g_relAltitude;
    relAltIdx = 0; relAltFilled = true;
  }

  WiFi.softAP(ssid, password);
  Serial.print("Access Point creato. IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/release", handleRelease);
  server.on("/data", handleData);
  server.on("/setPressure", HTTP_POST, handleSetPressure);
  server.on("/setReleaseAltitude", HTTP_POST, handleSetReleaseAltitude);
  server.on("/resetRelease", handleResetRelease);
  server.begin();
  Serial.println("Server web avviato");
}

// ====================== LOOP ======================
void loop() {
  server.handleClient();

  if (bmeAvailable) {
    uint32_t now = millis();
    if (now - lastBmeRead >= BME_INTERVAL_MS) {
      lastBmeRead = now;

      float temperature = bme.readTemperature();
      float humidity    = bme.readHumidity();
      float pressure    = bme.readPressure() / 100.0F;
      float altitude    = bme.readAltitude(seaLevelPressure_hpa);

      float relAltRaw = altitude - baseAltitude;
      float candidate = relAltRaw;

      if (altFilterInit) {
        float delta = candidate - g_relAltitude;
        if (fabs(delta) > SPIKE_THRESHOLD_M)
          candidate = g_relAltitude + (delta > 0 ? MAX_STEP_M : -MAX_STEP_M);
      }

      pushWindow(candidate);
      float med = medianOfWindow();

      if (!altFilterInit) {
        g_relAltitude = med;
        altFilterInit = true;
      } else {
        g_relAltitude = (1.0f - ALT_ALPHA) * g_relAltitude + ALT_ALPHA * med;
      }

      g_temperature = temperature;
      g_humidity    = humidity;
      g_pressure    = pressure;
      g_altitude    = altitude;
      if (g_relAltitude > g_maxRelAltitude) g_maxRelAltitude = g_relAltitude;

      if (!payloadReleased && releaseAltitude > 0) {
        if (g_relAltitude >= releaseAltitude) {
          if (aboveThresholdSince == 0) aboveThresholdSince = now;
          else if (now - aboveThresholdSince >= HYSTERESIS_MS) {
            Serial.printf("Quota sgancio confermata (relAlt=%.2f m). Rilascio...\n", g_relAltitude);
            relAltUsedAtRelease = g_relAltitude;
            RELEASE();
            payloadReleased = true;
          }
        } else {
          aboveThresholdSince = 0;
        }
      }
    }
  }

  delay(1);
}

// ====================== HANDLER ======================
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<meta name='theme-color' content='#22c55e' id='themeColor'>";
  html += "<style>";
  html += "body{margin:0;font-family:system-ui,-apple-system,Segoe UI,Roboto,Ubuntu,Arial;background:#fff;color:#111827;}";
  html += ".container{max-width:760px;margin:28px auto;padding:0 16px;}";
  html += ".header{display:flex;align-items:center;justify-content:space-between;margin-bottom:14px;}";
  html += ".title{font-size:20px;font-weight:700;letter-spacing:.3px}";
  html += ".status{padding:10px 14px;border-radius:10px;color:#fff;font-weight:700;min-width:220px;text-align:center;}";
  html += ".status.ok{background:#22c55e;} .status.bad{background:#ef4444;}";
  html += ".grid{display:grid;grid-template-columns:repeat(2,1fr);gap:14px;margin-top:14px;}";
  html += ".card{background:#f9fafb;border:1px solid #e5e7eb;box-shadow:0 2px 6px rgba(0,0,0,0.08);border-radius:12px;padding:16px 16px 14px}";
  html += ".label{color:#6b7280;font-size:12px;text-transform:uppercase}";
  html += ".value{color:#111827;font-weight:800;font-size:26px;margin-top:6px}";
  html += ".unit{opacity:.65;font-weight:600;font-size:16px;margin-left:6px}";
  html += ".sub{color:#6b7280;font-size:12px;margin-top:8px}";
  html += ".row{display:flex;gap:10px;flex-wrap:wrap;margin-top:16px}";
  html += ".btn,.btn-ghost{appearance:none;border:none;cursor:pointer;border-radius:10px;padding:10px 14px;font-weight:700}";
  html += ".btn{color:#fff;background:#3b82f6} .btn:hover{background:#2563eb}";
  html += ".btn-ghost{background:#fff;border:1px solid #d1d5db;color:#111827} .btn-ghost:hover{background:#f3f4f6}";
  html += "form{display:flex;align-items:center;gap:10px;flex-wrap:wrap;margin-top:12px}";
  html += "input[type=number]{background:#fff;border:1px solid #d1d5db;color:#111827;border-radius:8px;padding:8px 10px;min-width:140px}";
  html += "input[type=submit]{border:none;border-radius:8px;padding:10px 14px;background:#3b82f6;color:#fff;font-weight:700;cursor:pointer}";
  html += "input[type=submit]:hover{background:#2563eb}";
  html += ".warn{color:#f59e0b}";
  html += "</style>";

  html += "<script>";
  html += "function setThemeColor(hex){var m=document.getElementById('themeColor'); if(m){m.setAttribute('content',hex);} }";
  html += "function aggiorna(){fetch('/data').then(r=>r.json()).then(d=>{";
  html += "document.getElementById('temp').textContent=d.temperatura.toFixed(1);";
  html += "document.getElementById('hum').textContent=d.umidita.toFixed(1);";
  html += "document.getElementById('press').textContent=d.pressione.toFixed(2);";
  html += "document.getElementById('alt').textContent=d.altitudine.toFixed(1);";
  html += "document.getElementById('relAlt').textContent=d.altRel.toFixed(1);";
  html += "document.getElementById('maxAlt').textContent=d.altMax.toFixed(1);";
  html += "if(d.relAltAtRelease!==null){document.getElementById('altAtRelease').textContent=d.relAltAtRelease.toFixed(2);}else{document.getElementById('altAtRelease').textContent='--';}";
  html += "var s=document.getElementById('status');";
  html += "if(d.released){s.className='status bad';s.textContent='Carico RILASCIATO';setThemeColor('#ef4444');}";
  html += "else{s.className='status ok';s.textContent='Carico NON rilasciato';setThemeColor('#22c55e');}";
  html += "}).catch(()=>{});} setInterval(aggiorna,2000);window.onload=aggiorna;";
  html += "</script></head><body>";

  html += "<div class='container'><div class='header'><div class='title'>Kite Payload Release</div><div id='status' class='status'>--</div></div>";

  if (!bmeAvailable) {
    html += "<div class='card' style='grid-column:1/-1'><div class='label'>Sensore</div><div class='sub warn'>ATTENZIONE: BME280 non trovato!</div></div>";
  }

  html += "<div class='grid'>";
  html += "<div class='card'><div class='label'>Temperatura</div><div class='value'><span id='temp'>--</span><span class='unit'>&deg;C</span></div></div>";
  html += "<div class='card'><div class='label'>Umidità</div><div class='value'><span id='hum'>--</span><span class='unit'>%</span></div></div>";
  html += "<div class='card'><div class='label'>Pressione</div><div class='value'><span id='press'>--</span><span class='unit'>hPa</span></div></div>";
  html += "<div class='card'><div class='label'>Altitudine assoluta</div><div class='value'><span id='alt'>--</span><span class='unit'>m</span></div></div>";
  html += "<div class='card'><div class='label'>Altitudine relativa</div><div class='value'><span id='relAlt'>--</span><span class='unit'>m</span></div></div>";
  html += "<div class='card'><div class='label'>Altitudine massima</div><div class='value'><span id='maxAlt'>--</span><span class='unit'>m</span></div></div>";
  html += "<div class='card' style='grid-column:1/-1'><div class='label'>Quota usata per lo sgancio</div><div class='value'><span id='altAtRelease'>--</span><span class='unit'>m</span></div></div></div>";

  html += "<div class='card' style='margin-top:14px'><div class='label'>Impostazioni</div>";
  html += "<form action='/setPressure' method='POST'><span class='sub'>Pressione livello mare (hPa)</span><input type='number' step='0.01' name='slp' value='" + String(seaLevelPressure_hpa, 2) + "'><input type='submit' value='Salva'></form>";
  html += "<form action='/setReleaseAltitude' method='POST'><span class='sub'>Quota sgancio automatica (m)</span><input type='number' step='0.1' name='relAlt' value='" + String(releaseAltitude, 1) + "'><input type='submit' value='Imposta'></form>";
  html += "<div class='row'><a href='/release'><button class='btn'>RILASCIA</button></a><a href='/resetRelease'><button class='btn-ghost'>Reset Altitudine</button></a></div></div></div></body></html>";

  server.send(200, "text/html", html);
}

void handleData() {
  String json = "{";
  json += "\"temperatura\":" + String(g_temperature, 1) + ",";
  json += "\"umidita\":" + String(g_humidity, 1) + ",";
  json += "\"pressione\":" + String(g_pressure, 2) + ",";
  json += "\"altitudine\":" + String(g_altitude, 1) + ",";
  json += "\"altRel\":" + String(g_relAltitude, 1) + ",";
  json += "\"altMax\":" + String(g_maxRelAltitude, 1) + ",";
  json += "\"released\":" + String(payloadReleased ? "true" : "false") + ",";
  json += "\"relAltAtRelease\":" + (isnan(relAltUsedAtRelease) ? String("null") : String(relAltUsedAtRelease, 2));
  json += "}";
  server.send(200, "application/json", json);
}

void handleRelease() {
  relAltUsedAtRelease = g_relAltitude;
  RELEASE();
  payloadReleased = true;
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleSetPressure() {
  if (server.hasArg("slp")) {
    float newPressure = server.arg("slp").toFloat();
    if (newPressure >= 300 && newPressure <= 1100) {
      seaLevelPressure_hpa = newPressure;
      saveSeaLevelPressure(newPressure);
      Serial.print("Nuova pressione livello mare: ");
      Serial.println(newPressure);
      if (bmeAvailable) baseAltitude = bme.readAltitude(seaLevelPressure_hpa);
      altFilterInit = false;
      aboveThresholdSince = 0;
    }
  }
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleSetReleaseAltitude() {
  if (server.hasArg("relAlt")) {
    releaseAltitude = server.arg("relAlt").toFloat();
    saveReleaseAltitude(releaseAltitude);
    payloadReleased = false;
    relAltUsedAtRelease = NAN;
    aboveThresholdSince = 0;
    Serial.print("Quota sgancio impostata: ");
    Serial.println(releaseAltitude);
  }
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleResetRelease() {
  baseAltitude = bmeAvailable ? bme.readAltitude(seaLevelPressure_hpa) : 0;
  g_maxRelAltitude = 0;
  payloadReleased = false;
  relAltUsedAtRelease = NAN;
  altFilterInit = false;
  aboveThresholdSince = 0;
  relAltIdx = 0; relAltFilled = false;
  Serial.println("Reset altitudine e stato sgancio");
  myservo.write(0);
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}
