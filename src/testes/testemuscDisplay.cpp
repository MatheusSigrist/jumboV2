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

void mostrarTelaTeste() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("===== TESTE ======");
  display.println();
  display.print("MUSC<->CABECA: ");
  display.println(comunicacaoCabecaOK ? "OK" : "FALHA");
  display.println();
  if (comunicacaoCabecaOK) {
    display.println("TUDO OK!");
  } else {
    display.println("Verifique conexoes!");
  }
  display.display();
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

  unsigned long inicio = millis();
  unsigned long ultimoEnvio = 0;

  while ((millis() - inicio) < 5000) {
    if (millis() - ultimoEnvio >= 500) {
      Serial1.println("oi");
      ultimoEnvio = millis();
    }

    if (Serial1.available() > 0) {
      String msg = Serial1.readStringUntil('\n');
      msg.trim();
      msg.toLowerCase();
      if (msg == "oi") {
        comunicacaoCabecaOK = true;
        break;
      }
    }

    delay(20);
  }

  mostrarTelaTeste();
}

void loop() {
  delay(200);
}
