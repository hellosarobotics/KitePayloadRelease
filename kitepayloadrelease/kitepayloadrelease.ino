#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Servo.h>
#include <EEPROM.h>

// ======= Wi-Fi AP =======
const char* ssid = "KiteRelease";
const char* password = "12345678";

ESP8266WebServer server(80);
Servo myservo;
Adafruit_BME280 bme;

// ======= EEPROM =======
// 4 byte per pressione + 4 byte per quota rilascio
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
float g_relAltitude = 0;      // usata per decidere lo sgancio
float g_maxRelAltitude = 0;
float relAltUsedAtRelease = NAN; // quota usata al momento del rilascio

// Filtro esponenziale (stabilizza l'altitudine relativa)
const float ALT_ALPHA = 0.3f; // 0<alpha<=1; 0.3 ~ buon compromesso
bool altFilterInit = false;

// ======= Isteresi sgancio =======
uint32_t aboveThresholdSince = 0;     // millis quando superiamo la soglia
const uint32_t HYSTERESIS_MS = 1000;  // 1 secondo di conferma

// ====================== EEPROM UTILS ======================
void saveSeaLevelPressure(float value) {
  EEPROM.begin(EEPROM_SIZE);
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < 4; i++) EEPROM.write(i, *p++);
  EEPROM.commit();
  EEPROM.end();
}
float loadSeaLevelPressure() {
  float value;
  EEPROM.begin(EEPROM_SIZE);
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < 4; i++) *p++ = EEPROM.read(i);
  EEPROM.end();
  if (isnan(value) || value < 300 || value > 1100) return 1013.25; // fallback
  return value;
}
void saveReleaseAltitude(float value) {
  EEPROM.begin(EEPROM_SIZE);
  byte* p = (byte*)(void*)&value;
  for (int i = 4; i < 8; i++) EEPROM.write(i, *p++);
  EEPROM.commit();
  EEPROM.end();
}
float loadReleaseAltitude() {
  float value;
  EEPROM.begin(EEPROM_SIZE);
  byte* p = (byte*)(void*)&value;
  for (int i = 4; i < 8; i++) *p++ = EEPROM.read(i);
  EEPROM.end();
  if (isnan(value) || value < 0 || value > 10000) return 0; // fallback
  return value;
}

// ====================== SERVO ======================
void RELEASE() {
  Serial.println("Rilascio (RELEASE)");
  myservo.write(180);
  delay(500);
  myservo.write(0);
}

// ====================== WEB UI ======================
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  // Colore barra di stato su Android/Chrome (aggiornato via JS)
  html += "<meta name='theme-color' content='#22c55e' id='themeColor'>";
  html += "<style>";
  html += "body{margin:0;font-family:system-ui,-apple-system,Segoe UI,Roboto,Ubuntu,Arial;background:#ffffff;color:#111827;}";
  html += ".container{max-width:760px;margin:28px auto;padding:0 16px;}";
  html += ".header{display:flex;align-items:center;justify-content:space-between;margin-bottom:14px;}";
  html += ".title{font-size:20px;font-weight:700;letter-spacing:.3px}";
  html += ".status{padding:10px 14px;border-radius:10px;color:#fff;font-weight:700;min-width:220px;text-align:center;}";
  html += ".status.ok{background:#22c55e;}";      // verde
  html += ".status.bad{background:#ef4444;}";    // rosso
  html += ".grid{display:grid;grid-template-columns:repeat(2,1fr);gap:14px;margin-top:14px;}"; // 2 colonne SEMPRE
  html += ".card{background:#f9fafb;border:1px solid #e5e7eb;box-shadow:0 2px 6px rgba(0,0,0,0.08);border-radius:12px;padding:16px 16px 14px}";
  html += ".label{color:#6b7280;font-size:12px;letter-spacing:.5px;text-transform:uppercase}";
  html += ".value{color:#111827;font-weight:800;font-size:26px;line-height:1.1;margin-top:6px}";
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
  html += "document.getElementById('press').textContent=d.pressione.toFixed(1);";
  html += "document.getElementById('alt').textContent=d.altitudine.toFixed(1);";
  html += "document.getElementById('relAlt').textContent=d.altRel.toFixed(1);";
  html += "document.getElementById('maxAlt').textContent=d.altMax.toFixed(1);";
  html += "if(d.relAltAtRelease!==null){document.getElementById('altAtRelease').textContent=d.relAltAtRelease.toFixed(2);}else{document.getElementById('altAtRelease').textContent='--';}";
  html += "var s=document.getElementById('status');";
  html += "if(d.released){s.className='status bad'; s.textContent='Carico RILASCIATO'; setThemeColor('#ef4444');}";
  html += "else{ s.className='status ok'; s.textContent='Carico NON rilasciato'; setThemeColor('#22c55e');}";
  html += "}).catch(()=>{});}";

  html += "setInterval(aggiorna,2000); window.onload=aggiorna;";
  html += "</script></head><body>";

  html += "<div class='container'>";
  html += "  <div class='header'>";
  html += "    <div class='title'>Kite Payload Release</div>";
  html += "    <div id='status' class='status'>--</div>";
  html += "  </div>";

  if (!bmeAvailable) {
    html += "<div class='card' style='grid-column:1/-1'><div class='label'>Sensore</div>";
    html += "<div class='sub warn'>ATTENZIONE: BME280 non trovato!</div></div>";
  }

  html += "  <div class='grid'>";
  html += "    <div class='card'><div class='label'>Temperatura</div><div class='value'><span id='temp'>--</span><span class='unit'>&deg;C</span></div></div>";
  html += "    <div class='card'><div class='label'>Umidità</div><div class='value'><span id='hum'>--</span><span class='unit'>%</span></div></div>";
  html += "    <div class='card'><div class='label'>Pressione</div><div class='value'><span id='press'>--</span><span class='unit'>hPa</span></div></div>";
  html += "    <div class='card'><div class='label'>Altitudine assoluta</div><div class='value'><span id='alt'>--</span><span class='unit'>m</span></div></div>";
  html += "    <div class='card'><div class='label'>Altitudine relativa</div><div class='value'><span id='relAlt'>--</span><span class='unit'>m</span></div></div>";
  html += "    <div class='card'><div class='label'>Altitudine massima</div><div class='value'><span id='maxAlt'>--</span><span class='unit'>m</span></div></div>";
  html += "    <div class='card' style='grid-column:1/-1'><div class='label'>Quota usata per lo sgancio</div>";
  html += "      <div class='value'><span id='altAtRelease'>--</span><span class='unit'>m</span></div>";
  html += "    </div>";
  html += "  </div>";

  html += "  <div class='card' style='margin-top:14px'>";
  html += "    <div class='label'>Impostazioni</div>";
  html += "    <form action='/setPressure' method='POST'>";
  html += "      <span class='sub' style='min-width:170px;display:inline-block'>Pressione livello mare (hPa)</span>";
  html += "      <input type='number' step='0.01' name='slp' value='" + String(seaLevelPressure_hpa, 2) + "'>";
  html += "      <input type='submit' value='Salva'>";
  html += "    </form>";
  html += "    <form action='/setReleaseAltitude' method='POST'>";
  html += "      <span class='sub' style='min-width:170px;display:inline-block'>Quota sgancio automatica (m)</span>";
  html += "      <input type='number' step='0.1' name='relAlt' value='" + String(releaseAltitude, 1) + "'>";
  html += "      <input type='submit' value='Imposta'>";
  html += "    </form>";
  html += "    <div class='row'>";
  html += "      <a href='/release'><button class='btn'>RILASCIA</button></a>";
  html += "      <a href='/resetRelease'><button class='btn-ghost'>Reset Altitudine</button></a>";
  html += "    </div>";
  html += "  </div>";

  html += "</div></body></html>";
  server.send(200, "text/html", html);
}

void handleData() {
  // Usa SOLO la cache aggiornata dalla loop()
  String json = "{";
  json += "\"temperatura\":" + String(g_temperature, 1) + ",";
  json += "\"umidita\":" + String(g_humidity, 1) + ",";
  json += "\"pressione\":" + String(g_pressure, 1) + ",";
  json += "\"altitudine\":" + String(g_altitude, 1) + ",";
  json += "\"altRel\":" + String(g_relAltitude, 1) + ",";
  json += "\"altMax\":" + String(g_maxRelAltitude, 1) + ",";
  json += "\"released\":" + String(payloadReleased ? "true" : "false") + ",";
  json += "\"relAltAtRelease\":" + (isnan(relAltUsedAtRelease) ? String("null") : String(relAltUsedAtRelease, 2));
  json += "}";
  server.send(200, "application/json", json);
}

void handleRelease() {
  relAltUsedAtRelease = g_relAltitude; // rilascio manuale: salva quota corrente
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
      Serial.print("Nuova pressione livello mare salvata: ");
      Serial.println(newPressure);
      // Ricalibra base e filtro
      if (bmeAvailable) baseAltitude = bme.readAltitude(seaLevelPressure_hpa);
      altFilterInit = false; // riavvia il filtro con la nuova base
      aboveThresholdSince = 0; // prudenza
    }
  }
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleSetReleaseAltitude() {
  if (server.hasArg("relAlt")) {
    releaseAltitude = server.arg("relAlt").toFloat();
    saveReleaseAltitude(releaseAltitude);
    payloadReleased = false;      // reset stato
    relAltUsedAtRelease = NAN;    // azzera quota usata al rilascio
    aboveThresholdSince = 0;      // azzera isteresi
    Serial.print("Quota sgancio impostata e salvata: ");
    Serial.println(releaseAltitude);
  }
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleResetRelease() {
  if (bmeAvailable) baseAltitude = bme.readAltitude(seaLevelPressure_hpa);
  else baseAltitude = 0;

  g_maxRelAltitude = 0;
  payloadReleased = false;
  relAltUsedAtRelease = NAN;
  altFilterInit = false;      // il filtro riparte dal prossimo campione
  aboveThresholdSince = 0;    // azzera isteresi

  Serial.println("Altitudine relativa, massima e stato sgancio resettati");
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
  myservo.write(0);
}

// ====================== SETUP & LOOP ======================
void setup() {
  Serial.begin(115200);
  myservo.attach(D5);
  myservo.write(0);

  seaLevelPressure_hpa = loadSeaLevelPressure();
  releaseAltitude = loadReleaseAltitude();
  Serial.print("Pressione livello mare iniziale: ");
  Serial.println(seaLevelPressure_hpa);
  Serial.print("Quota sgancio iniziale: ");
  Serial.println(releaseAltitude);

  if (bme.begin(0x76) || bme.begin(0x77)) {
    bmeAvailable = true;
    Serial.println("BME280 trovato!");
  } else {
    Serial.println("ATTENZIONE: BME280 non trovato!");
    bmeAvailable = false;
  }

  baseAltitude = bmeAvailable ? bme.readAltitude(seaLevelPressure_hpa) : 0;

  // Prime letture per popolare la cache (UI senza zeri all'avvio)
  if (bmeAvailable) {
    g_temperature = bme.readTemperature();
    g_humidity    = bme.readHumidity();
    g_pressure    = bme.readPressure() / 100.0F;
    g_altitude    = bme.readAltitude(seaLevelPressure_hpa);
    g_relAltitude = g_altitude - baseAltitude;
    g_maxRelAltitude = g_relAltitude;
    altFilterInit = true; // inizializza filtro
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

void loop() {
  server.handleClient();

  if (bmeAvailable) {
    // Letture BME
    float temperature = bme.readTemperature();
    float humidity    = bme.readHumidity();
    float pressure    = bme.readPressure() / 100.0F;
    float altitude    = bme.readAltitude(seaLevelPressure_hpa);

    // Altitudine relativa grezza
    float relAltRaw = altitude - baseAltitude;

    // Filtro per stabilizzare
    if (!altFilterInit) {
      g_relAltitude = relAltRaw;
      altFilterInit = true;
    } else {
      g_relAltitude = (1.0f - ALT_ALPHA) * g_relAltitude + ALT_ALPHA * relAltRaw;
    }

    // Aggiorna cache condivisa
    g_temperature = temperature;
    g_humidity    = humidity;
    g_pressure    = pressure;
    g_altitude    = altitude;

    if (g_relAltitude > g_maxRelAltitude) g_maxRelAltitude = g_relAltitude;

    // ===== Logica di sgancio con isteresi =====
    if (!payloadReleased && releaseAltitude > 0) {
      if (g_relAltitude >= releaseAltitude) {
        if (aboveThresholdSince == 0) {
          aboveThresholdSince = millis(); // appena superata la soglia
        } else if (millis() - aboveThresholdSince >= HYSTERESIS_MS) {
          Serial.printf("Quota di sgancio confermata (relAlt = %.2f m per >= %u ms). Rilascio...\n",
                        g_relAltitude, HYSTERESIS_MS);
          relAltUsedAtRelease = g_relAltitude;
          RELEASE();
          payloadReleased = true;
        }
      } else {
        // scesi sotto soglia → annulla finestra di conferma
        aboveThresholdSince = 0;
      }
    }
  }

  // Se vuoi ridurre il calore: puoi portare a delay(100..1000).
  // delay(1) mantiene l'interfaccia più reattiva.
  delay(1);
}
