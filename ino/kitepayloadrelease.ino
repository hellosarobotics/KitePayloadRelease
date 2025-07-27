#include <Servo.h>

Servo myservo; 

void setup() {
  Serial.begin(115200);       // avvia la seriale per debug
  myservo.attach(D5);         // collega il servo su D5 (GPIO2)
  Serial.println("Setup completato");
  myservo.write(180); // inizializzazione a 180 perché quando poi passa a 0 rilascia immediatamente
}



// Funzione per rilascio
void RELEASE() {
  Serial.println("Rilascio (RELEASE)");
  myservo.write(0);    // posizione di rilascio
  delay(1000);
  myservo.write(180);
}

void loop() {
 Serial.println("Conto alla rovescia di 10 secondi:");
  
  for (int i = 10; i >= 1; i--) {
    Serial.print("Mancano: ");
    Serial.print(i);
    Serial.println(" secondi");
    delay(1000); // 1 secondo di attesa
  }

  RELEASE();
  
}
