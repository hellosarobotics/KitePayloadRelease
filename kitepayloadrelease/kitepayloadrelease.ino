#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Servo.h>
#include <EEPROM.h>

const char* ssid = "KiteRelease";
const char* password = "12345678";

ESP8266WebServer server(80);
Servo myservo;
Adafruit_BME280 bme;

// --- EEPROM settings ---
// 4 byte per pressione + 4 byte per quota rilascio
#define EEPROM_SIZE 8
float seaLevelPressure_hpa = 1013.25;
float releaseAltitude = 0;

// --- Altitudine relativa ---
float baseAltitude = 0;

// --- Sgancio automatico ---
bool payloadReleased = false;

// --- Stato sensore ---
bool bmeAvailable = false;

// --- Cache letture (usate SIA per la logica che per la UI) ---
float g_temperature = 0;
float g_humidity = 0;
float g_pressure = 0;
float g_altitude = 0;
float g_relAltitude = 0;    // QUELLA usata per decidere lo sgancio
float g_maxRelAltitude = 0;

float relAltUsedAtRelease = NAN; // memorizza la quota relativa usata al momento del rilascio

// (opzionale) Filtro esponenziale per stabilizzare l'altitudine relativa
const float ALT_ALPHA = 0.3f;   // 0 < ALT_ALPHA <= 1; 0.3 ≈ buon compromesso
bool altFilterInit = false;


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

// ====================== SERVO RELEASE ======================
void RELEASE() {
  Serial.println("Rilascio (RELEASE)");
  myservo.write(180);
  delay(500);
  myservo.write(0);
}

// ====================== WEB PAGES ======================
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<style>";
  html += "body{font-family:Arial,sans-serif;margin:0;padding:0;background:#f1f5f9;}";
  html += ".container{max-width:420px;margin:40px auto;background:#fff;padding:20px 22px;border-radius:12px;box-shadow:0 6px 20px rgba(0,0,0,0.08);text-align:center;}";
  html += "h1{font-size:22px;margin-bottom:10px;color:#111827;}";
  html += "h2{font-size:18px;margin:16px 0 10px;color:#374151;}";
  html += "p{font-size:16px;margin:8px 0;color:#111827;}";
  html += "#status{margin:15px auto;padding:10px;color:#fff;font-weight:bold;width:85%;border-radius:8px;}";
  html += "form{margin:15px 0;}";
  html += "input[type=number]{padding:8px 10px;width:140px;border:1px solid #d1d5db;border-radius:8px;}";
  html += "input[type=submit],button{padding:10px 16px;font-size:16px;border:none;border-radius:10px;background:#3b82f6;color:white;cursor:pointer;}";
  html += "input[type=submit]:hover,button:hover{background:#2563eb;}";
  html += "a{text-decoration:none;}";
  html += "</style>";
  html += "<script>";
  html += "function aggiorna(){fetch('/data').then(r=>r.json()).then(d=>{";
  html += "document.getElementById('temp').innerHTML=d.temperatura.toFixed(1)+' &deg;C';";
  html += "document.getElementById('hum').innerHTML=d.umidita.toFixed(1)+' %';";
  html += "document.getElementById('press').innerHTML=d.pressione.toFixed(1)+' hPa';";
  html += "document.getElementById('alt').innerHTML=d.altitudine.toFixed(1)+' m';";
  html += "document.getElementById('relAlt').innerHTML=d.altRel.toFixed(1)+' m';";
  html += "document.getElementById('maxAlt').innerHTML=d.altMax.toFixed(1)+' m';";
  html += "if(d.relAltAtRelease!==null){document.getElementById('altAtRelease').innerHTML=d.relAltAtRelease.toFixed(2)+' m';}else{document.getElementById('altAtRelease').innerHTML='--';}";
  html += "if(d.released){document.getElementById('status').style.background='red';document.getElementById('status').innerHTML='Carico RILASCIATO';}";
  html += "else{document.getElementById('status').style.background='green';document.getElementById('status').innerHTML='Carico NON rilasciato';}";
  html += "});}";
  html += "setInterval(aggiorna,2000);window.onload=aggiorna;";
  html += "</script></head><body>";
  html += "<div class='container'>";
  html += "<h1>Kite Payload Release</h1>";
  html += "<div id='status'>--</div>";
  html += "<h2>Dati BME280</h2>";
  if (!bmeAvailable) html += "<p style='color:red;'>ATTENZIONE: BME280 non trovato!</p>";
  html += "<p>Temperatura: <span id='temp'>--</span></p>";
  html += "<p>Umidità: <span id='hum'>--</span></p>";
  html += "<p>Pressione: <span id='press'>--</span></p>";
  html += "<p>Altitudine: <span id='alt'>--</span></p>";
  html += "<p>Altitudine relativa: <span id='relAlt'>--</span></p>";
  html += "<p>Altitudine massima: <span id='maxAlt'>--</span></p>";
  html += "<p>Altitudine usata per lo sgancio: <span id='altAtRelease'>--</span></p>";
  html += "<form action='/setPressure' method='POST'>";
  html += "<p>Pressione livello mare: <input type='number' step='0.01' name='slp' value='" + String(seaLevelPressure_hpa, 2) + "'> hPa ";
  html += "<input type='submit' value='Salva'></p></form>";
  html += "<form action='/setReleaseAltitude' method='POST'>";
  html += "<p>Quota sgancio automatica: <input type='number' step='0.1' name='relAlt' value='" + String(releaseAltitude, 1) + "'> m ";
  html += "<input type='submit' value='Imposta'></p></form>";
  html += "<p><a href='/release'><button>RILASCIA</button></a></p>";
  html += "<p><a href='/resetRelease'><button>Reset Altitudine</button></a></p>";
  html += "</div></body></html>";
  server.send(200, "text/html", html);
}

void handleData() {
  // Usa SOLO i valori in cache aggiornati dalla loop()
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
  relAltUsedAtRelease = g_relAltitude; // se rilasci manualmente, annota la quota corrente
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
      altFilterInit = false; // rilancia il filtro con la nuova base
    }
  }
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleSetReleaseAltitude() {
  if (server.hasArg("relAlt")) {
    releaseAltitude = server.arg("relAlt").toFloat();
    saveReleaseAltitude(releaseAltitude);
    payloadReleased = false; // resetta stato quando si imposta nuova quota
    relAltUsedAtRelease = NAN;
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
  altFilterInit = false;      // ri-inizializza il filtro dalla prossima lettura

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

  // Prime letture per popolare la cache (così la UI non mostra zeri all'avvio)
  if (bmeAvailable) {
    g_temperature = bme.readTemperature();
    g_humidity    = bme.readHumidity();
    g_pressure    = bme.readPressure() / 100.0F;
    g_altitude    = bme.readAltitude(seaLevelPressure_hpa);
    g_relAltitude = g_altitude - baseAltitude;
    g_maxRelAltitude = g_relAltitude;
    altFilterInit = true; // filtro inizializzato con il primo valore
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
    // Letture grezze BME
    float temperature = bme.readTemperature();
    float humidity    = bme.readHumidity();
    float pressure    = bme.readPressure() / 100.0F;
    float altitude    = bme.readAltitude(seaLevelPressure_hpa);

    // Altitudine relativa grezza
    float relAltRaw = altitude - baseAltitude;

    // Filtro (facoltativo ma consigliato per stabilità e coerenza UI/logica)
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

    // Logica di sgancio: USA la stessa g_relAltitude che mostriamo in UI
    if (!payloadReleased && releaseAltitude > 0 && g_relAltitude >= releaseAltitude) {
      Serial.print("Quota di sgancio raggiunta (relAlt = ");
      Serial.print(g_relAltitude, 2);
      Serial.println(" m). Rilascio carico...");
      relAltUsedAtRelease = g_relAltitude; // memorizza la quota reale usata
      RELEASE();
      payloadReleased = true;
    }
  }
}
