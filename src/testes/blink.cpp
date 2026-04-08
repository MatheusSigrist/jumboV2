// Teste simples de LED.
// Funcao: verificar se a placa inicializa e consegue alternar um pino digital.
// Resultado esperado: LED acende por 200 ms e apaga por 10 s em loop.
#include <Arduino.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 48
#endif

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);

  digitalWrite(LED_BUILTIN, LOW);
  delay(10000);
}
