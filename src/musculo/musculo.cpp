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

#define IN1_1_A 5
#define IN2_1_A 6
#define PWM_1_A 4

#define IN1_2_A 3
#define IN2_2_A 46
#define PWM_2_A 7

#define IN1_1_B 11
#define IN2_1_B 12
#define PWM_1_B 10

#define IN1_2_B 13
#define IN2_2_B 14
#define PWM_2_B 47

#define PWM_CH1 0
#define PWM_CH2 1
#define PWM_CH3 2
#define PWM_CH4 3

#define PWM_FREQ 20000
#define PWM_RES 8

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

bool comunicacaoCabecaOK = false;
String bufferSerial = "";
String mensagemBotao = "NENHUM";
unsigned long ultimoEnvioOi = 0;
unsigned long ultimoRxCabeca = 0;
unsigned long mostrarStatusAte = 0;
bool olhoOK = false;
bool peOK = false;

bool corGolAzul = false;
int headingBussolaTeste = 0;
float anguloIr = -1.0;
bool irDetectado = false;

enum Estado { MENU, CALIBRACAO, INICIAR };
Estado estadoAtual = MENU;
int itemSelecionado = 0;

enum SubMenuCalibracao { SUBMENU_PRINCIPAL, SUBMENU_GOL, SUBMENU_BUSSOLA };
SubMenuCalibracao subMenuCalibracao = SUBMENU_PRINCIPAL;
int itemSubMenu = 0;

const unsigned long INTERVALO_OI_MS = 1000;
const unsigned long TIMEOUT_COM_MS = 3000;
const int velocidade_maxima = 255;

void Motor_1(int vel1) {
  int pwm1 = constrain(abs(vel1), 0, 255);
  ledcWrite(PWM_CH1, pwm1);
  if (vel1 <= 0) {
    digitalWrite(IN1_1_A, HIGH);
    digitalWrite(IN2_1_A, LOW);
  } else {
    digitalWrite(IN1_1_A, LOW);
    digitalWrite(IN2_1_A, HIGH);
  }
}

void Motor_2(int vel2) {
  int pwm2 = constrain(abs(vel2), 0, 255);
  ledcWrite(PWM_CH2, pwm2);
  if (vel2 <= 0) {
    digitalWrite(IN1_2_A, HIGH);
    digitalWrite(IN2_2_A, LOW);
  } else {
    digitalWrite(IN1_2_A, LOW);
    digitalWrite(IN2_2_A, HIGH);
  }
}

void Motor_4(int vel3) {
  int pwm3 = constrain(abs(vel3), 0, 255);
  ledcWrite(PWM_CH3, pwm3);
  if (vel3 <= 0) {
    digitalWrite(IN1_1_B, HIGH);
    digitalWrite(IN2_1_B, LOW);
  } else {
    digitalWrite(IN1_1_B, LOW);
    digitalWrite(IN2_1_B, HIGH);
  }
}

void Motor_3(int vel4) {
  int pwm4 = constrain(abs(vel4), 0, 255);
  ledcWrite(PWM_CH4, pwm4);
  if (vel4 <= 0) {
    digitalWrite(IN1_2_B, HIGH);
    digitalWrite(IN2_2_B, LOW);
  } else {
    digitalWrite(IN1_2_B, LOW);
    digitalWrite(IN2_2_B, HIGH);
  }
}

void seguirDirecaoPorAngulo(float anguloGraus, int velocidade) {
  int velocidadeAlvo = constrain(velocidade, 0, velocidade_maxima);
  float theta = anguloGraus * PI / 180.0;
  float vx = velocidadeAlvo * sin(theta);
  float vy = velocidadeAlvo * cos(theta);

  float theta1 = 45.0 * PI / 180.0;
  float theta2 = 135.0 * PI / 180.0;
  float theta3 = 225.0 * PI / 180.0;
  float theta4 = 315.0 * PI / 180.0;

  float v1 = vx * cos(theta1) + vy * sin(theta1);
  float v2 = vx * cos(theta2) + vy * sin(theta2);
  float v3 = vx * cos(theta3) + vy * sin(theta3);
  float v4 = vx * cos(theta4) + vy * sin(theta4);

  float maxVel = max(max(abs(v1), abs(v2)), max(abs(v3), abs(v4)));
  if (maxVel > velocidade_maxima) {
    float escala = (float)velocidade_maxima / maxVel;
    v1 *= escala;
    v2 *= escala;
    v3 *= escala;
    v4 *= escala;
  }

  Motor_1((int)v1);
  Motor_2((int)v2);
  Motor_3((int)v3);
  Motor_4((int)v4);
}

void pararMotores() {
  ledcWrite(PWM_CH1, 0);
  ledcWrite(PWM_CH2, 0);
  ledcWrite(PWM_CH3, 0);
  ledcWrite(PWM_CH4, 0);

  digitalWrite(IN1_1_A, LOW);
  digitalWrite(IN2_1_A, LOW);
  digitalWrite(IN1_2_A, LOW);
  digitalWrite(IN2_2_A, LOW);
  digitalWrite(IN1_1_B, LOW);
  digitalWrite(IN2_1_B, LOW);
  digitalWrite(IN1_2_B, LOW);
  digitalWrite(IN2_2_B, LOW);
}

void desenharMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("==== MENU ====");
  display.println();

  if (itemSelecionado == 0) {
    display.fillRect(0, 16, 128, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(4, 18);
    display.println("CALIBRACAO");
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.setCursor(4, 18);
    display.println("CALIBRACAO");
  }

  if (itemSelecionado == 1) {
    display.fillRect(0, 32, 128, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(4, 34);
    display.println("INICIAR");
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.setCursor(4, 34);
    display.println("INICIAR");
  }

  display.setCursor(0, 54);
  display.print("COM:");
  display.print(comunicacaoCabecaOK ? "OK" : "FALHA");
  display.print("  ");
  display.print(mensagemBotao);
  display.display();
}

void desenharSubmenuCalibracao() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("=== CALIBRACAO ===");
  display.println();

  if (subMenuCalibracao == SUBMENU_PRINCIPAL) {
    if (itemSubMenu == 0) {
      display.fillRect(0, 16, 128, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 18);
      display.println("GOL");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 18);
      display.println("GOL");
    }

    if (itemSubMenu == 1) {
      display.fillRect(0, 32, 128, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 34);
      display.println("BUSSOLA");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 34);
      display.println("BUSSOLA");
    }

    if (itemSubMenu == 2) {
      display.fillRect(0, 48, 128, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 50);
      display.println("VOLTAR");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 50);
      display.println("VOLTAR");
    }
  } else if (subMenuCalibracao == SUBMENU_GOL) {
    display.println("Selecione cor do gol");

    if (itemSubMenu == 0) {
      display.fillRect(0, 24, 128, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 26);
      display.println("AMARELO");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 26);
      display.println("AMARELO");
    }

    if (itemSubMenu == 1) {
      display.fillRect(0, 40, 128, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 42);
      display.println("AZUL");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 42);
      display.println("AZUL");
    }

    if (itemSubMenu == 2) {
      display.fillRect(0, 54, 128, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 56);
      display.println("VOLTAR");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 56);
      display.println("VOLTAR");
    }
  } else {
    display.println("CAL BUSSOLA");
    display.println();
    display.print("Heading: ");
    display.print(headingBussolaTeste);
    display.println(" graus");
    display.println();
    display.println("BTN1/BTN2 alterna");
    display.println("BTN3 confirma");
  }

  display.display();
}

void desenharOperacao() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("=== OPERACAO TESTE ===");
  display.println();
  display.print("COM CABECA: ");
  display.println(comunicacaoCabecaOK ? "OK" : "FALHA");
  display.print("IR ANG: ");
  if (irDetectado) {
    display.print(anguloIr, 1);
    display.println(" deg");
  } else {
    display.println("NAO DETECTADO");
  }
  display.println();
  display.println("BTN3 volta MENU");
  display.print("Ultimo: ");
  display.println(mensagemBotao);
  display.display();
}

void desenharStatusPlacas() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("=== TESTE PLACAS ===");
  display.println();
  display.print("MUSC<->CAB: ");
  display.println(comunicacaoCabecaOK ? "OK" : "FALHA");
  display.print("OLHO: ");
  display.println(olhoOK ? "OK" : "FALHA");
  display.print("PE: ");
  display.println(peOK ? "OK" : "FALHA");
  display.display();
}

void mostrarTelaFalhaComunicacao() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("=== TESTE COM/BTN ===");
  display.println();
  display.println("COM CABECA: FALHA");
  display.println("Verifique serial 17/18");
  display.println();
  display.print("Ultimo: ");
  display.println(mensagemBotao);
  display.display();
}

void desenharTelaAtual() {
  if (millis() < mostrarStatusAte) {
    desenharStatusPlacas();
    return;
  }

  if (!comunicacaoCabecaOK) {
    mostrarTelaFalhaComunicacao();
    return;
  }

  if (estadoAtual == MENU) {
    desenharMenu();
  } else if (estadoAtual == CALIBRACAO) {
    desenharSubmenuCalibracao();
  } else {
    desenharOperacao();
  }
}

void processarEventoBotao(uint8_t botao) {
  if (estadoAtual == MENU) {
    if (botao == 1) {
      itemSelecionado--;
      if (itemSelecionado < 0) itemSelecionado = 1;
    } else if (botao == 2) {
      itemSelecionado++;
      if (itemSelecionado > 1) itemSelecionado = 0;
    } else if (botao == 3) {
      if (itemSelecionado == 0) {
        estadoAtual = CALIBRACAO;
        subMenuCalibracao = SUBMENU_PRINCIPAL;
        itemSubMenu = 0;
      } else {
        estadoAtual = INICIAR;
      }
    }
    return;
  }

  if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_PRINCIPAL) {
    if (botao == 1) {
      itemSubMenu--;
      if (itemSubMenu < 0) itemSubMenu = 2;
    } else if (botao == 2) {
      itemSubMenu++;
      if (itemSubMenu > 2) itemSubMenu = 0;
    } else if (botao == 3) {
      if (itemSubMenu == 0) {
        subMenuCalibracao = SUBMENU_GOL;
        itemSubMenu = corGolAzul ? 1 : 0;
      } else if (itemSubMenu == 1) {
        subMenuCalibracao = SUBMENU_BUSSOLA;
        itemSubMenu = 0;
      } else {
        estadoAtual = MENU;
        itemSelecionado = 0;
      }
    }
    return;
  }

  if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_GOL) {
    if (botao == 1) {
      itemSubMenu--;
      if (itemSubMenu < 0) itemSubMenu = 2;
    } else if (botao == 2) {
      itemSubMenu++;
      if (itemSubMenu > 2) itemSubMenu = 0;
    } else if (botao == 3) {
      if (itemSubMenu == 0 || itemSubMenu == 1) {
        corGolAzul = (itemSubMenu == 1);
      }
      subMenuCalibracao = SUBMENU_PRINCIPAL;
      itemSubMenu = 0;
    }
    return;
  }

  if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_BUSSOLA) {
    if (botao == 1 || botao == 2) {
      itemSubMenu = (itemSubMenu == 0) ? 1 : 0;
    } else if (botao == 3) {
      mensagemBotao = "BUSSOLA GRAVADA";
      subMenuCalibracao = SUBMENU_PRINCIPAL;
      itemSubMenu = 0;
    }
    return;
  }

  if (estadoAtual == INICIAR && botao == 3) {
    estadoAtual = MENU;
  }
}

void processarMensagemCabeca(String msg) {
  msg.trim();
  msg.toUpperCase();

  if (msg == "OI") {
    comunicacaoCabecaOK = true;
    ultimoRxCabeca = millis();
    return;
  }

  if (msg.startsWith("STS:")) {
    int separador = msg.indexOf(',');
    if (separador > 4) {
      String olho = msg.substring(4, separador);
      String pe = msg.substring(separador + 1);
      olho.trim();
      pe.trim();
      olhoOK = (olho == "1");
      peOK = (pe == "1");
      comunicacaoCabecaOK = true;
      ultimoRxCabeca = millis();
      mostrarStatusAte = millis() + 3000;
    }
    return;
  }

  if (msg.startsWith("BTN:")) {
    String valor = msg.substring(4);
    valor.trim();
    if (valor == "1" || valor == "2" || valor == "3") {
      mensagemBotao = "BOTAO " + valor + " APERTADO";
      comunicacaoCabecaOK = true;
      ultimoRxCabeca = millis();
      processarEventoBotao((uint8_t)valor.toInt());
    }
    return;
  }

  if (msg.startsWith("IR:")) {
    String valorIr = msg.substring(3);
    valorIr.trim();
    float novoAngulo = valorIr.toFloat();
    irDetectado = (novoAngulo >= 0.0);
    anguloIr = novoAngulo;
    comunicacaoCabecaOK = true;
    ultimoRxCabeca = millis();
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

  pinMode(IN1_1_A, OUTPUT);
  pinMode(IN2_1_A, OUTPUT);
  pinMode(IN1_2_A, OUTPUT);
  pinMode(IN2_2_A, OUTPUT);
  pinMode(IN1_1_B, OUTPUT);
  pinMode(IN2_1_B, OUTPUT);
  pinMode(IN1_2_B, OUTPUT);
  pinMode(IN2_2_B, OUTPUT);

  ledcSetup(PWM_CH1, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_1_A, PWM_CH1);
  ledcSetup(PWM_CH2, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_2_A, PWM_CH2);
  ledcSetup(PWM_CH3, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_1_B, PWM_CH3);
  ledcSetup(PWM_CH4, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_2_B, PWM_CH4);
  pararMotores();

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

  desenharTelaAtual();
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

  headingBussolaTeste = (headingBussolaTeste + 2) % 360;
  if (estadoAtual == INICIAR && comunicacaoCabecaOK && irDetectado) {
    seguirDirecaoPorAngulo(anguloIr, velocidade_maxima);
  } else {
    pararMotores();
  }

  static unsigned long ultimaTela = 0;
  if ((millis() - ultimaTela) > 120) {
    desenharTelaAtual();
    ultimaTela = millis();
  }

  delay(5);
}
