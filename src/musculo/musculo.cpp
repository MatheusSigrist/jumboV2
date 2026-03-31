#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===================== GIRO ALINHAMENTO - ALTERE AQUI =====================
#define VELOCIDADE_GIRO  140
#define SINAL_GIRO       -1
// ==========================================================================

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
bool cameraOK = false;  // camera se comunicando com o olho

enum Estado { MENU, CALIBRACAO, INICIAR };
Estado estadoAtual = MENU;
int itemSelecionado = 0;

enum SubMenuCalibracao { SUBMENU_PRINCIPAL, SUBMENU_GOL, SUBMENU_BUSSOLA, SUBMENU_CAMERA, SUBMENU_INTENSIDADE };
SubMenuCalibracao subMenuCalibracao = SUBMENU_PRINCIPAL;
int itemSubMenu = 0;

const unsigned long INTERVALO_OI_MS = 1000;
const unsigned long TIMEOUT_COM_MS = 3000;
const int velocidade_maxima = 200;
const int EEPROM_SIZE = 64;
const int EEPROM_ADDR_BUSSOLA = 0;
const float TOLERANCIA_ALINHAMENTO_GRAUS = 20.0f;
const int VELOCIDADE_GIRO_ALINHAMENTO = VELOCIDADE_GIRO;
const int SINAL_GIRO_PID = SINAL_GIRO;
const float ALPHA_FILTRO_BUSSOLA = 0.12f;
const float PID_BUS_KP = 1.0f;
const float PID_BUS_KI = 0.00f;
const float PID_BUS_KD = 0.8;
const float PID_BUS_INTEGRAL_MAX = 120.0f;
const int PID_BUS_SAIDA_MIN = 25;
const int PID_BUS_SAIDA_MAX = 180;
const float GANHO_GIRO_MISTO = 1.0f;
const int VELOCIDADE_MIN_BOLA = 60;

int headingBussolaSalvo = 0;
float erroAlinhamentoGraus = 0.0f;
bool alinhandoAgora = false;
bool corGolPendenteEnvio = true;
unsigned long ultimoEnvioCorGolMs = 0;
const unsigned long INTERVALO_ENVIO_COR_GOL_MS = 500;
float pidBusIntegral = 0.0f;
float pidBusErroAnterior = 0.0f;
unsigned long pidBusUltimoMs = 0;

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

void seguirDirecaoComGiro(float anguloGraus, int velocidade, int cmdGiro) {
  int velocidadeAlvo = constrain(velocidade, 0, velocidade_maxima);
  float theta = anguloGraus * PI / 180.0f;
  float vx = velocidadeAlvo * sinf(theta);
  float vy = velocidadeAlvo * cosf(theta);

  float theta1 = 45.0f * PI / 180.0f;
  float theta2 = 135.0f * PI / 180.0f;
  float theta3 = 225.0f * PI / 180.0f;
  float theta4 = 315.0f * PI / 180.0f;

  float v1 = vx * cosf(theta1) + vy * sinf(theta1);
  float v2 = vx * cosf(theta2) + vy * sinf(theta2);
  float v3 = vx * cosf(theta3) + vy * sinf(theta3);
  float v4 = vx * cosf(theta4) + vy * sinf(theta4);

  // Mesmo sentido de giro da funcao girarNoEixo: Motor_X(-vel)
  float termoGiro = -GANHO_GIRO_MISTO * (float)cmdGiro;
  v1 += termoGiro;
  v2 += termoGiro;
  v3 += termoGiro;
  v4 += termoGiro;

  float maxVel = max(max(fabsf(v1), fabsf(v2)), max(fabsf(v3), fabsf(v4)));
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

void girarNoEixo(int velocidade) {
  int vel = constrain(velocidade, -velocidade_maxima, velocidade_maxima);

  // Mapeamento de giro da base:
  // horario: M1/M2 frente e M3/M4 tras
  // anti-horario: inverso
  Motor_1(-vel);
  Motor_2(-vel);
  Motor_3(-vel);
  Motor_4(-vel);
}

float normalizarErro180(float erro) {
  while (erro > 180.0f) erro -= 360.0f;
  while (erro < -180.0f) erro += 360.0f;
  return erro;
}

float normalizarAngulo360(float ang) {
  while (ang >= 360.0f) ang -= 360.0f;
  while (ang < 0.0f) ang += 360.0f;
  return ang;
}

void resetPidBussola() {
  pidBusIntegral = 0.0f;
  pidBusErroAnterior = 0.0f;
  pidBusUltimoMs = 0;
}

int calcularSaidaPidBussola(float erroGraus) {
  unsigned long agora = millis();
  float dt = 0.02f;
  if (pidBusUltimoMs != 0) {
    dt = (agora - pidBusUltimoMs) / 1000.0f;
    if (dt < 0.005f) dt = 0.005f;
    if (dt > 0.2f) dt = 0.2f;
  }
  pidBusUltimoMs = agora;

  pidBusIntegral += erroGraus * dt;
  if (pidBusIntegral > PID_BUS_INTEGRAL_MAX) pidBusIntegral = PID_BUS_INTEGRAL_MAX;
  if (pidBusIntegral < -PID_BUS_INTEGRAL_MAX) pidBusIntegral = -PID_BUS_INTEGRAL_MAX;

  float derivada = (erroGraus - pidBusErroAnterior) / dt;
  pidBusErroAnterior = erroGraus;

  float u = PID_BUS_KP * erroGraus + PID_BUS_KI * pidBusIntegral + PID_BUS_KD * derivada;
  int saida = (int)fabsf(u);
  if (saida < PID_BUS_SAIDA_MIN) saida = PID_BUS_SAIDA_MIN;
  if (saida > PID_BUS_SAIDA_MAX) saida = PID_BUS_SAIDA_MAX;
  if (saida > VELOCIDADE_GIRO_ALINHAMENTO) saida = VELOCIDADE_GIRO_ALINHAMENTO;

  return (u >= 0.0f) ? saida : -saida;
}

bool filtroBussolaInicializado = false;
float bussolaFiltroX = 1.0f;
float bussolaFiltroY = 0.0f;
float erroGolGraus = 0.0f;
bool golDetectado = false;
uint16_t golPixels = 0;
float anguloLinhaPe = -1.0f;
bool linhaDetectada = false;
float intensidadeIrAtual = 0.0f;
float distanciaBolaD = 0.0f;
const float INTENSIDADE_MIN_BOLA = 16.0f;
const float INTENSIDADE_MAX_BOLA = 45.0f;
const int VELOCIDADE_FUGA_LINHA = 170;
bool fugindoLinhaAgora = false;
float anguloFugaLinhaCmd = 0.0f;

float mapearDistanciaBola(float intensidadeIr) {
  float intensidade = intensidadeIr;
  if (intensidade < INTENSIDADE_MIN_BOLA) intensidade = INTENSIDADE_MIN_BOLA;
  if (intensidade > INTENSIDADE_MAX_BOLA) intensidade = INTENSIDADE_MAX_BOLA;

  // Intensidade maior significa bola mais proxima: distancia D deve diminuir.
  return ((INTENSIDADE_MAX_BOLA - intensidade) * 100.0f) / (INTENSIDADE_MAX_BOLA - INTENSIDADE_MIN_BOLA);
}

void enviarCorGolParaCabeca() {
  if (!corGolPendenteEnvio) {
    return;
  }

  if ((millis() - ultimoEnvioCorGolMs) < INTERVALO_ENVIO_COR_GOL_MS) {
    return;
  }

  Serial1.print("CFG:GOL:");
  Serial1.println(corGolAzul ? "1" : "0");
  ultimoEnvioCorGolMs = millis();
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

  display.setCursor(0, 44);
  display.print("GOL ERR:");
  if (golDetectado) {
    display.print(erroGolGraus, 1);
    display.print("deg");
  } else {
    display.print("SEM GOL");
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
      display.fillRect(0, 40, 128, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 42);
      display.println("TESTE CAM");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 42);
      display.println("TESTE CAM");
    }

    if (itemSubMenu == 3) {
      display.fillRect(0, 48, 128, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 50);
      display.println("INTENSIDADE");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 50);
      display.println("INTENSIDADE");
    }

    if (itemSubMenu == 4) {
      display.fillRect(0, 54, 128, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 56);
      display.println("VOLTAR");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 56);
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
  } else if (subMenuCalibracao == SUBMENU_CAMERA) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("=== TESTE CAMERA ===");
    display.println();
    display.print("MUSC<->CAB: ");
    display.println(comunicacaoCabecaOK ? "OK" : "FALHA");
    display.print("CAM->OLHO:  ");
    display.println(cameraOK ? "OK" : "SEM SINAL");
    display.print("GOL: ");
    display.println(golDetectado ? "DETECTADO" : "NAO VE");
    display.print("ERRO: ");
    if (golDetectado) {
      display.print(erroGolGraus, 1);
      display.println(" deg");
    } else {
      display.println("---");
    }
    display.print("PIXELS: ");
    display.println(golPixels);
    display.println("BTN1/2 VOLTAR");
  } else if (subMenuCalibracao == SUBMENU_INTENSIDADE) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("=== INTENSIDADE IR ===");
    display.println();
    display.setTextSize(3);
    display.setCursor(8, 22);
    display.print(intensidadeIrAtual, 1);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("D MAP: ");
    display.print(distanciaBolaD, 1);
    display.setTextSize(1);
    display.setCursor(0, 56);
    display.println("BTN1/2/3 VOLTAR");
  } else {
    display.setCursor(0, 0);
    display.println("BUSSOLA AGORA");
    display.println();
    display.setTextSize(3);
    display.setCursor(8, 20);
    display.print(headingBussolaTeste);
    display.print((char)247);
    display.setTextSize(1);
    display.setCursor(0, 54);
    display.print("SALVO:");
    display.print(headingBussolaSalvo);
    display.print((char)247);
    display.print("  BTN3 SALVA");
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
  display.print("D MAP: ");
  display.println(distanciaBolaD, 1);
  display.print("ERRO GOL: ");
  if (golDetectado) {
    display.print(erroGolGraus, 1);
    display.println(" deg");
  } else {
    display.println("SEM GOL");
  }
  display.print("LINHA ANG: ");
  if (linhaDetectada) {
    display.print(anguloLinhaPe, 1);
    display.println(" deg");
    display.print("FUGA CMD: ");
    display.print(anguloFugaLinhaCmd, 1);
    display.println(" deg");
  } else {
    display.println("SEM LINHA");
  }
  if (golDetectado) {
    display.print("ERRO: ");
    display.print(erroAlinhamentoGraus, 1);
    display.println(" deg");
    if (fugindoLinhaAgora) {
      display.println("MODO: FUGINDO LINHA");
    } else {
      display.println(alinhandoAgora ? "MODO: ALINHANDO GOL" : "MODO: SEGUINDO BOLA");
    }
    display.print("PIX: ");
    display.println(golPixels);
  } else if (fugindoLinhaAgora) {
    display.println("MODO: FUGINDO LINHA");
  }
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
      if (itemSubMenu < 0) itemSubMenu = 4;
    } else if (botao == 2) {
      itemSubMenu++;
      if (itemSubMenu > 4) itemSubMenu = 0;
    } else if (botao == 3) {
      if (itemSubMenu == 0) {
        subMenuCalibracao = SUBMENU_GOL;
        itemSubMenu = corGolAzul ? 1 : 0;
      } else if (itemSubMenu == 1) {
        subMenuCalibracao = SUBMENU_BUSSOLA;
        itemSubMenu = 0;
      } else if (itemSubMenu == 2) {
        subMenuCalibracao = SUBMENU_CAMERA;
        itemSubMenu = 0;
      } else if (itemSubMenu == 3) {
        subMenuCalibracao = SUBMENU_INTENSIDADE;
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
        corGolPendenteEnvio = true;
        mensagemBotao = corGolAzul ? "ENVIA GOL AZUL" : "ENVIA GOL AMARELO";
      }
      subMenuCalibracao = SUBMENU_PRINCIPAL;
      itemSubMenu = 0;
    }
    return;
  }

  if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_BUSSOLA) {
    if (botao == 3) {
      headingBussolaSalvo = headingBussolaTeste;
      EEPROM.put(EEPROM_ADDR_BUSSOLA, headingBussolaSalvo);
      bool ok = EEPROM.commit();
      mensagemBotao = ok ? "BUSSOLA GRAVADA" : "ERRO EEPROM";
      subMenuCalibracao = SUBMENU_PRINCIPAL;
      itemSubMenu = 0;
    } else if (botao == 1 || botao == 2) {
      subMenuCalibracao = SUBMENU_PRINCIPAL;
      itemSubMenu = 1;
    }
    return;
  }

  if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_CAMERA) {
    if (botao == 1 || botao == 2 || botao == 3) {
      subMenuCalibracao = SUBMENU_PRINCIPAL;
      itemSubMenu = 2;
    }
    return;
  }

  if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_INTENSIDADE) {
    if (botao == 1 || botao == 2 || botao == 3) {
      subMenuCalibracao = SUBMENU_PRINCIPAL;
      itemSubMenu = 3;
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
    return;
  }

  if (msg.startsWith("INT:")) {
    String valorInt = msg.substring(4);
    valorInt.trim();
    intensidadeIrAtual = valorInt.toFloat();
    if (intensidadeIrAtual < 0.0f) intensidadeIrAtual = 0.0f;
    distanciaBolaD = mapearDistanciaBola(intensidadeIrAtual);
    comunicacaoCabecaOK = true;
    ultimoRxCabeca = millis();
    return;
  }

  if (msg == "CFG:GOL:OK") {
    corGolPendenteEnvio = false;
    mensagemBotao = corGolAzul ? "GOL AZUL OK" : "GOL AMARELO OK";
    comunicacaoCabecaOK = true;
    ultimoRxCabeca = millis();
    return;
  }

  if (msg.startsWith("BUS:")) {
    String valorBus = msg.substring(4);
    valorBus.trim();
    float angBus = normalizarAngulo360(valorBus.toFloat());

    float rad = angBus * PI / 180.0f;
    float xNovo = cosf(rad);
    float yNovo = sinf(rad);

    if (!filtroBussolaInicializado) {
      bussolaFiltroX = xNovo;
      bussolaFiltroY = yNovo;
      filtroBussolaInicializado = true;
    } else {
      bussolaFiltroX = (1.0f - ALPHA_FILTRO_BUSSOLA) * bussolaFiltroX + ALPHA_FILTRO_BUSSOLA * xNovo;
      bussolaFiltroY = (1.0f - ALPHA_FILTRO_BUSSOLA) * bussolaFiltroY + ALPHA_FILTRO_BUSSOLA * yNovo;
    }

    float angFiltrado = atan2f(bussolaFiltroY, bussolaFiltroX) * 180.0f / PI;
    angFiltrado = normalizarAngulo360(angFiltrado);

    headingBussolaTeste = (int)(angFiltrado + 0.5f);
    if (headingBussolaTeste >= 360) {
      headingBussolaTeste = 0;
    }
    comunicacaoCabecaOK = true;
    ultimoRxCabeca = millis();
    return;
  }

  if (msg.startsWith("GOL:")) {
    String payload = msg.substring(4);
    int p1 = payload.indexOf(',');
    int p2 = payload.indexOf(',', p1 + 1);
    int p3 = payload.indexOf(',', p2 + 1);
    if (p1 > 0 && p2 > p1) {
      String sErro = payload.substring(0, p1);
      String sDet = payload.substring(p1 + 1, p2);
      String sPix = payload.substring(p2 + 1, (p3 > p2) ? p3 : payload.length());
      sErro.trim();
      sDet.trim();
      sPix.trim();

      erroGolGraus = sErro.toFloat();
      golDetectado = (sDet == "1");
      int pix = sPix.toInt();
      if (pix < 0) pix = 0;
      if (pix > 65535) pix = 65535;
      golPixels = (uint16_t)pix;

      if (p3 > p2) {
        String sCam = payload.substring(p3 + 1);
        sCam.trim();
        cameraOK = (sCam == "1");
      }

      comunicacaoCabecaOK = true;
      ultimoRxCabeca = millis();
    }
    return;
  }

  if (msg.startsWith("LIN:")) {
    String sLinha = msg.substring(4);
    sLinha.trim();
    float novoAng = sLinha.toFloat();
    linhaDetectada = (novoAng >= 0.0f);
    anguloLinhaPe = novoAng;
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
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_ADDR_BUSSOLA, headingBussolaSalvo);
  if (headingBussolaSalvo < 0 || headingBussolaSalvo >= 360) {
    headingBussolaSalvo = 0;
  }

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
  enviarCorGolParaCabeca();

  if ((millis() - ultimoRxCabeca) > TIMEOUT_COM_MS) {
    comunicacaoCabecaOK = false;
  }

  if (estadoAtual == INICIAR && comunicacaoCabecaOK) {
    // Alinhamento usa erro do gol (camera), igual ao teste PIDBussola.
    erroAlinhamentoGraus = erroGolGraus;
    fugindoLinhaAgora = false;

    // Prioridade de movimento: LINHA > BOLA > ALINHAR NO EIXO
    if (linhaDetectada) {
      fugindoLinhaAgora = true;
      alinhandoAgora = false;
      resetPidBussola();
      anguloFugaLinhaCmd = normalizarErro180(anguloLinhaPe - 180.0f);
      seguirDirecaoPorAngulo(anguloFugaLinhaCmd, VELOCIDADE_FUGA_LINHA);
    } else {
      int cmdPidAssinado = 0;
      bool precisaAlinhar = golDetectado && (fabsf(erroAlinhamentoGraus) > TOLERANCIA_ALINHAMENTO_GRAUS);
      if (precisaAlinhar) {
        alinhandoAgora = true;
        int cmdPid = calcularSaidaPidBussola(erroAlinhamentoGraus);
        cmdPidAssinado = -SINAL_GIRO_PID * cmdPid;
      } else {
        alinhandoAgora = false;
        resetPidBussola();
      }

      if (irDetectado) {
        int velocidadeBola = constrain((int)(distanciaBolaD * 2.0f), VELOCIDADE_MIN_BOLA, velocidade_maxima);
        seguirDirecaoComGiro(anguloIr, velocidadeBola, cmdPidAssinado);
      } else if (precisaAlinhar) {
        girarNoEixo(cmdPidAssinado);
      } else {
        pararMotores();
      }
    }
  } else {
    alinhandoAgora = false;
    fugindoLinhaAgora = false;
    resetPidBussola();
    pararMotores();
  }

  static unsigned long ultimaTela = 0;
  if ((millis() - ultimaTela) > 120) {
    desenharTelaAtual();
    ultimaTela = millis();
  }

  delay(5);
}
