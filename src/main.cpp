#include <Arduino.h>

// Código de teste: piscar o LED interno (LED L) do Arduino
// Usa LED_BUILTIN para funcionar tanto em UNO quanto MEGA.

const unsigned long INTERVALO_MS = 500; // meio segundo
unsigned long ultimoToggle = 0;
bool estadoLed = false;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    Serial.begin(9600);
    while(!Serial) { /* aguarda porta serial em placas que precisam */ }
    Serial.println("Teste LED iniciado. LED deve piscar a cada 500 ms.");
}

void loop() {
    unsigned long agora = millis();
    if (agora - ultimoToggle >= INTERVALO_MS) {
        ultimoToggle = agora;
        estadoLed = !estadoLed;
        digitalWrite(LED_BUILTIN, estadoLed ? HIGH : LOW);
        Serial.print("LED: ");
        Serial.println(estadoLed ? "ON" : "OFF");
    }
    // Outras lógicas de teste podem ser adicionadas aqui sem bloquear
}