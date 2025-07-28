#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Servo.h>

// === CONFIGURAZIONE WIFI ===
const char* ssid = "KiteRelease";
const char* password = "12345678";

ESP8266WebServer server(80);
Servo myservo;
Adafruit_BME280 bme;

const float SEALEVELPRESSURE_HPA = 1013.25; // pressione standard a livello del mare

// === FUNZIONE DI RILASCIO ===
void RELEASE() {
  Serial.println("Rilascio (RELEASE)");
  myservo.write(0);
  delay(1000);
  myservo.write(180);
}

// === PAGINA WEB PRINCIPALE ===
void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<script>";
  html += "function aggiorna(){";
  html += " fetch('/data').then(response => response.json()).then(dati => {";
  html += "   document.getElementById('temp').innerHTML = dati.temperatura.toFixed(1) + ' °C';";
  html += "   document.getElementById('hum').innerHTML = dati.umidita.toFixed(1) + ' %';";
  html += "   document.getElementById('press').innerHTML = dati.pressione.toFixed(1) + ' hPa';";
  html += "   document.getElementById('alt').innerHTML = dati.altitudine.toFixed(1) + ' m';";
  html += " });";
  html += "}";
  html += "setInterval(aggiorna, 2000);"; // ogni 2 secondi
  html += "window.onload = aggiorna;";
  html += "</script></head>";
  html += "<body style='text-align:center;font-family:sans-serif;'>";
  html += "<h1>Kite Payload Release</h1>";
  html += "<h2>Dati BME280</h2>";
  html += "<p>Temperatura: <span id='temp'>--</span></p>";
  html += "<p>Umidità: <span id='hum'>--</span></p>";
  html += "<p>Pressione: <span id='press'>--</span></p>";
  html += "<p>Altitudine: <span id='alt'>--</span></p>";
  html += "<p><a href='/release'><button style='font-size:24px;'>RILASCIA</button></a></p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// === ENDPOINT JSON DATI SENSORI ===
void handleData() {
  float temperature = bme.readTemperature();
  float humidity = bme.readHumidity();
  float pressure = bme.readPressure() / 100.0F; // hPa
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA); // m

  String json = "{";
  json += "\"temperatura\":" + String(temperature, 1) + ",";
  json += "\"umidita\":" + String(humidity, 1) + ",";
  json += "\"pressione\":" + String(pressure, 1) + ",";
  json += "\"altitudine\":" + String(altitude, 1);
  json += "}";

  server.send(200, "application/json", json);
}

// === HANDLER RILASCIO ===
void handleRelease() {
  RELEASE();
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void setup() {
  Serial.begin(115200);
  myservo.attach(D5);
  myservo.write(180);

  if (!bme.begin(0x76)) { // alcuni moduli hanno 0x77
    Serial.println("Errore BME280! Controllare connessioni e indirizzo I2C.");
    while (1);
  }

  WiFi.softAP(ssid, password);
  Serial.print("Access Point creato. IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/release", handleRelease);
  server.on("/data", handleData);
  server.begin();
  Serial.println("Server web avviato");
}

void loop() {
  server.handleClient();
}
