#include <Arduino.h>

#define RX_MUSCULO 44
#define TX_MUSCULO 43

HardwareSerial SerialMusculo(0);
String bufferEntrada = "";
unsigned long ultimoByteMs = 0;

void processarMensagem(String msg) {
  msg.trim();
  msg.toLowerCase();

  if (msg == "oi") {
    SerialMusculo.println("oi");
    Serial.println("Recebi 'oi' do musculo e respondi 'oi'");
  } else if (msg.length() > 0) {
    Serial.print("Recebido (ignorado): ");
    Serial.println(msg);
  }
}

void setup() {
  Serial.begin(115200);
  SerialMusculo.begin(9600, SERIAL_8N1, RX_MUSCULO, TX_MUSCULO);
  Serial.println("Teste comunicacao cabeca: pronto");
  Serial.println("Se receber 'oi' no RX, responde 'oi' no TX");
}

void loop() {
  while (SerialMusculo.available() > 0) {
    char c = (char)SerialMusculo.read();
    ultimoByteMs = millis();

    if (c == '\n' || c == '\r') {
      processarMensagem(bufferEntrada);
      bufferEntrada = "";
      continue;
    }

    if (bufferEntrada.length() < 32) {
      bufferEntrada += c;
    }
  }

  if (bufferEntrada.length() > 0 && (millis() - ultimoByteMs) > 80) {
    processarMensagem(bufferEntrada);
    bufferEntrada = "";
  }

  delay(5);
}
