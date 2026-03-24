#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define RX_CABECA 17
#define TX_CABECA 18

#define SDA_PIN 8
#define SCL_PIN 9
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

bool comunicacaoCabecaOK = false;
String bufferSerial = "";
String mensagemBotao = "NENHUM";
unsigned long ultimoEnvioOi = 0;
unsigned long ultimoRxCabeca = 0;

const unsigned long INTERVALO_OI_MS = 1000;
const unsigned long TIMEOUT_COM_MS = 3000;

void mostrarTelaTeste() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("=== TESTE COM/BTN ===");
  display.println();
  display.print("COM CABECA: ");
  display.println(comunicacaoCabecaOK ? "OK" : "FALHA");
  display.println("EVENTO BOTAO:");
  display.println(mensagemBotao);
  display.println();
  if (comunicacaoCabecaOK) {
    display.println("Aguardando botoes...");
  } else {
    display.println("Verifique serial 17/18");
  }
  display.display();
}

void processarMensagemCabeca(String msg) {
  msg.trim();
  msg.toUpperCase();

  if (msg == "OI") {
    comunicacaoCabecaOK = true;
    ultimoRxCabeca = millis();
    return;
  }

  if (msg.startsWith("BTN:")) {
    String valor = msg.substring(4);
    valor.trim();
    if (valor == "1" || valor == "2" || valor == "3") {
      mensagemBotao = "BOTAO " + valor + " APERTADO";
      comunicacaoCabecaOK = true;
      ultimoRxCabeca = millis();
    }
  }
}

void lerSerialCabeca() {
  while (Serial1.available() > 0) {
    char c = (char)Serial1.read();
    if (c == '\n' || c == '\r') {
      if (bufferSerial.length() > 0) {
        processarMensagemCabeca(bufferSerial);
        bufferSerial = "";
      }
      continue;
    }

    if (bufferSerial.length() < 32) {
      bufferSerial += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RX_CABECA, TX_CABECA);

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    while (1) {
      delay(100);
    }
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 25);
  display.println("TESTANDO...");
  display.display();

  // janela inicial de handshake
  unsigned long inicio = millis();
  while ((millis() - inicio) < 3000) {
    if (millis() - ultimoEnvioOi >= 300) {
      Serial1.println("oi");
      ultimoEnvioOi = millis();
    }
    lerSerialCabeca();
    if (comunicacaoCabecaOK) {
      break;
    }
    delay(10);
  }

  mostrarTelaTeste();
}

void loop() {
  if (millis() - ultimoEnvioOi >= INTERVALO_OI_MS) {
    Serial1.println("oi");
    ultimoEnvioOi = millis();
  }

  lerSerialCabeca();

  if ((millis() - ultimoRxCabeca) > TIMEOUT_COM_MS) {
    comunicacaoCabecaOK = false;
  }

  static unsigned long ultimaTela = 0;
  if ((millis() - ultimaTela) > 120) {
    mostrarTelaTeste();
    ultimaTela = millis();
  }

  delay(5);
}
