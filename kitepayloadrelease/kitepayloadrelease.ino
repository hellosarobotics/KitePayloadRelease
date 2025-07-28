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
#define EEPROM_SIZE 4  // un float = 4 byte
float seaLevelPressure_hpa = 1013.25; 

// --- Altitudine relativa ---
float baseAltitude = 0;
float maxAltitude = 0;

// --- Sgancio automatico ---
float releaseAltitude = 0;     
bool payloadReleased = false;

// --- Stato sensore ---
bool bmeAvailable = false;

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

void RELEASE() {
  Serial.println("Rilascio (RELEASE)");
  myservo.write(0);
  delay(1000);
  myservo.write(180);
}

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<script>";
  html += "function aggiorna(){";
  html += " fetch('/data').then(response => response.json()).then(dati => {";
  html += "   document.getElementById('temp').innerHTML = dati.temperatura.toFixed(1) + ' &deg;C';";
  html += "   document.getElementById('hum').innerHTML = dati.umidita.toFixed(1) + ' %';";
  html += "   document.getElementById('press').innerHTML = dati.pressione.toFixed(1) + ' hPa';";
  html += "   document.getElementById('alt').innerHTML = dati.altitudine.toFixed(1) + ' m';";
  html += "   document.getElementById('relAlt').innerHTML = dati.altRel.toFixed(1) + ' m';";
  html += "   document.getElementById('maxAlt').innerHTML = dati.altMax.toFixed(1) + ' m';";
  html += " });";
  html += "}";
  html += "setInterval(aggiorna, 2000);";
  html += "window.onload = aggiorna;";
  html += "</script></head><body style='text-align:center;font-family:sans-serif;'>";
  html += "<h1>Kite Payload Release</h1>";
  html += "<h2>Dati BME280</h2>";
  if (!bmeAvailable) html += "<p style='color:red;'>ATTENZIONE: BME280 non trovato!</p>";
  html += "<p>Temperatura: <span id='temp'>--</span></p>";
  html += "<p>Umidità: <span id='hum'>--</span></p>";
  html += "<p>Pressione: <span id='press'>--</span></p>";
  html += "<p>Altitudine: <span id='alt'>--</span></p>";
  html += "<p>Altitudine relativa: <span id='relAlt'>--</span></p>";
  html += "<p>Altitudine massima: <span id='maxAlt'>--</span></p>";
  html += "<form action='/setPressure' method='POST'>";
  html += "<p>Pressione livello mare: <input type='number' step='0.01' name='slp' value='" + String(seaLevelPressure_hpa, 2) + "'> hPa ";
  html += "<input type='submit' value='Salva'></p></form>";
  html += "<form action='/setReleaseAltitude' method='POST'>";
  html += "<p>Quota sgancio automatica: <input type='number' step='0.1' name='relAlt' value='" + String(releaseAltitude, 1) + "'> m ";
  html += "<input type='submit' value='Imposta'></p></form>";
  html += "<p><a href='/release'><button style='font-size:24px;'>RILASCIA</button></a></p>";
  html += "<p><a href='/resetRelease'><button style='font-size:16px;'>Reset Altitudine</button></a></p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleData() {
  float temperature = 0;
  float humidity = 0;
  float pressure = 0;
  float altitude = 0;

  if (bmeAvailable) {
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = bme.readPressure() / 100.0F;
    altitude = bme.readAltitude(seaLevelPressure_hpa);
  }

  float relativeAltitude = altitude - baseAltitude;
  if (relativeAltitude > maxAltitude) maxAltitude = relativeAltitude;

  String json = "{";
  json += "\"temperatura\":" + String(temperature, 1) + ",";
  json += "\"umidita\":" + String(humidity, 1) + ",";
  json += "\"pressione\":" + String(pressure, 1) + ",";
  json += "\"altitudine\":" + String(altitude, 1) + ",";
  json += "\"altRel\":" + String(relativeAltitude, 1) + ",";
  json += "\"altMax\":" + String(maxAltitude, 1);
  json += "}";
  server.send(200, "application/json", json);
}

void handleRelease() {
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
    }
  }
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleSetReleaseAltitude() {
  if (server.hasArg("relAlt")) {
    releaseAltitude = server.arg("relAlt").toFloat();
    payloadReleased = false; // permette un nuovo rilascio se quota cambiata
    Serial.print("Quota sgancio impostata: ");
    Serial.println(releaseAltitude);
  }
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleResetRelease() {
  // Reset dell'altitudine relativa e massima e stato sgancio
  if (bmeAvailable) baseAltitude = bme.readAltitude(seaLevelPressure_hpa);
  else baseAltitude = 0;
  maxAltitude = 0;
  payloadReleased = false;
  Serial.println("Altitudine relativa, massima e stato sgancio resettati");
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void setup() {
  Serial.begin(115200);
  myservo.attach(D5);
  myservo.write(180);

  seaLevelPressure_hpa = loadSeaLevelPressure();
  Serial.print("Pressione livello mare iniziale: ");
  Serial.println(seaLevelPressure_hpa);

  // Prova sia indirizzo 0x76 che 0x77
  if (bme.begin(0x76) || bme.begin(0x77)) {
    bmeAvailable = true;
    Serial.println("BME280 trovato!");
  } else {
    Serial.println("ATTENZIONE: BME280 non trovato!");
    bmeAvailable = false;
  }

  // Altitudine base
  baseAltitude = bmeAvailable ? bme.readAltitude(seaLevelPressure_hpa) : 0;
  maxAltitude = 0;

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
    float altitude = bme.readAltitude(seaLevelPressure_hpa);
    float relativeAltitude = altitude - baseAltitude;

    // Rilascio automatico solo una volta
    if (!payloadReleased && releaseAltitude > 0 && relativeAltitude >= releaseAltitude) {
      Serial.println("Quota di sgancio raggiunta! Rilascio carico...");
      RELEASE();
      payloadReleased = true;
    }
  }
}
