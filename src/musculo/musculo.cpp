// Arquivo principal da placa Musculo.
// Funcao: controle de movimento (motores), menu no display OLED,
// calibracao (gol/bussola), alinhamento com PID e logica do kicker.
// Entrada: mensagens da Cabeca (serial), IR/linha/gol e botoes.
// Saida: comandos PWM para motores, pulso do kicker e telas de status.
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===================== GIRO ALINHAMENTO - ALTERE AQUI =====================
#define VELOCIDADE_GIRO  180
#define SINAL_GIRO       -1
// ==========================================================================

#define RX_CABECA 17
#define TX_CABECA 18

// Pinos do barramento I2C e dimensoes do display OLED.
#define SDA_PIN 8
#define SCL_PIN 9
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C

// Ponte H do conjunto A.
#define IN1_1_A 5
#define IN2_1_A 6
#define PWM_1_A 4

#define IN1_2_A 3
#define IN2_2_A 46
#define PWM_2_A 7

// Ponte H do conjunto B.
#define IN1_1_B 11
#define IN2_1_B 12
#define PWM_1_B 10

#define IN1_2_B 13
#define IN2_2_B 14
#define PWM_2_B 47

// Canais PWM usados pelo ESP32 para cada motor.
#define PWM_CH1 0
#define PWM_CH2 1
#define PWM_CH3 2
#define PWM_CH4 3

// Configuracao global do PWM dos motores.
#define PWM_FREQ 20000
#define PWM_RES 8

// Pino e temporizacao do solenoide/kicker.
constexpr uint8_t KICKER_PIN = 21;
constexpr unsigned long KICK_PULSE_MS = 100;
constexpr unsigned long KICK_INTERVAL_MS = 1000;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Estado da comunicacao e ultimo comando de interface recebido da Cabeca.
bool comunicacaoCabecaOK = false;
String bufferSerial = "";
String mensagemBotao = "NENHUM";
unsigned long ultimoEnvioOi = 0;
unsigned long ultimoRxCabeca = 0;
unsigned long mostrarStatusAte = 0;
bool olhoOK = false;
bool peOK = false;

// Sensores/telemetria vindos da Cabeca.
bool corGolAzul = false;
int headingBussolaTeste = 0;
float anguloIr = -1.0;
bool irDetectado = false;
bool cameraOK = false;  // camera se comunicando com o olho
float ultraDcm = -1.0f;
float ultraEcm = -1.0f;
float ultraFcm = -1.0f;
float ultraTcm = -1.0f;
bool ultrasValidos = false;
unsigned long ultimoRxUltraMs = 0;
bool kickerRecebido = false;
bool kickerAtivado = false;  // true quando chave acionada (valor 0 vindo da cabeca)
bool pulsoKickerAtivo = false;
unsigned long inicioPulsoKickerMs = 0;
unsigned long ultimoDisparoKickerMs = 0;

// Estados principais da interface/operacao.
enum Estado { MENU, CALIBRACAO, INICIAR };
Estado estadoAtual = MENU;
int itemSelecionado = 0;

// Submenus disponiveis na tela de calibracao.
enum SubMenuCalibracao { SUBMENU_PRINCIPAL, SUBMENU_GOL, SUBMENU_BUSSOLA, SUBMENU_IR, SUBMENU_ULTRA, SUBMENU_CAMERA };
SubMenuCalibracao subMenuCalibracao = SUBMENU_PRINCIPAL;
int itemSubMenu = 0;

// Temporizacao da comunicacao e limites gerais de velocidade.
const unsigned long INTERVALO_OI_MS = 1000;
const unsigned long TIMEOUT_COM_MS = 3000;
const unsigned long TIMEOUT_LINHA_MS = 150;
const unsigned long TIMEOUT_ULTRA_MS = 1000;
const int velocidade_maxima = 160;

// Endereco e tamanho usados para persistir a bussola na EEPROM.
const int EEPROM_SIZE = 64;
const int EEPROM_ADDR_BUSSOLA = 0;

// Parametros do alinhamento por camera + bussola.
const float TOLERANCIA_ALINHAMENTO_GRAUS = 20.0f;
const int VELOCIDADE_GIRO_ALINHAMENTO = VELOCIDADE_GIRO;
const int SINAL_GIRO_PID = SINAL_GIRO;
const float ALPHA_FILTRO_BUSSOLA = 0.12f;

// Ganhos e saturacoes do PID usado para girar rumo ao gol.
const float PID_BUS_KP = 1.0f;
const float PID_BUS_KI = 0.00f;
const float PID_BUS_KD = 0.8;
const float PID_BUS_INTEGRAL_MAX = 120.0f;
const int PID_BUS_SAIDA_MIN = 25;
const int PID_BUS_SAIDA_MAX = 180;
const float GANHO_GIRO_MISTO = 1.0f;



// Velocidade dedicada para ataque frontal quando a bola estiver entre 330° e 30°.
const int VELOCIDADE_IR_FRONTAL_PWM = 180;

// Referencia salva da bussola e estados auxiliares do controle.
int headingBussolaSalvo = 0;
float erroAlinhamentoGraus = 0.0f;
bool alinhandoAgora = false;
bool corGolPendenteEnvio = true;
unsigned long ultimoEnvioCorGolMs = 0;
const unsigned long INTERVALO_ENVIO_COR_GOL_MS = 500;

// Estado interno do PID entre iteracoes do loop.
float pidBusIntegral = 0.0f;
float pidBusErroAnterior = 0.0f;
unsigned long pidBusUltimoMs = 0;

// Controla o pulso do kicker com tempo minimo e intervalo entre disparos.
void atualizarKicker() {
  unsigned long agora = millis();

  // Finaliza o pulso depois do tempo minimo energizado.
  if (pulsoKickerAtivo && (agora - inicioPulsoKickerMs) >= KICK_PULSE_MS) {
    digitalWrite(KICKER_PIN, LOW);
    pulsoKickerAtivo = false;
  }

  // So permite chute durante o jogo, com comunicacao valida e chave acionada.
  bool podeChutar = (estadoAtual == INICIAR) && comunicacaoCabecaOK && kickerRecebido && kickerAtivado;
  if (!podeChutar) {
    // Garante desligamento imediato se alguma pre-condicao deixar de valer.
    if (pulsoKickerAtivo) {
      digitalWrite(KICKER_PIN, LOW);
      pulsoKickerAtivo = false;
    }
    return;
  }

  // Dispara um novo pulso respeitando o intervalo minimo entre chutes.
  if (!pulsoKickerAtivo && (agora - ultimoDisparoKickerMs) >= KICK_INTERVAL_MS) {
    digitalWrite(KICKER_PIN, HIGH);
    inicioPulsoKickerMs = agora;
    ultimoDisparoKickerMs = agora;
    pulsoKickerAtivo = true;
    Serial.println("Chutei");
  }
}

// Aciona o motor 1 com sentido e PWM conforme a velocidade assinada.
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

// Aciona o motor 2 com sentido e PWM conforme a velocidade assinada.
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

// Aciona o motor 4 com sentido e PWM conforme a velocidade assinada.
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

// Aciona o motor 3 com sentido e PWM conforme a velocidade assinada.
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

// Converte um angulo de translacao em velocidades individuais de roda.
void seguirDirecaoPorAngulo(float anguloGraus, int velocidade) {
  int velocidadeAlvo = constrain(velocidade, 0, velocidade_maxima);

  // Converte o comando polar em vetor translacional no referencial do robo.
  float theta = anguloGraus * PI / 180.0;
  float vx = velocidadeAlvo * sin(theta);
  float vy = velocidadeAlvo * cos(theta);

  // Angulos fisicos das rodas omnidirecionais/mecanum.
  float theta1 = 45.0 * PI / 180.0;
  float theta2 = 135.0 * PI / 180.0;
  float theta3 = 225.0 * PI / 180.0;
  float theta4 = 315.0 * PI / 180.0;

  // Projeta o vetor de translacao em cada roda.
  float v1 = vx * cos(theta1) + vy * sin(theta1);
  float v2 = vx * cos(theta2) + vy * sin(theta2);
  float v3 = vx * cos(theta3) + vy * sin(theta3);
  float v4 = vx * cos(theta4) + vy * sin(theta4);

  // Renormaliza para manter a maior roda dentro do limite configurado.
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

// Move por angulo e soma um termo de giro para alinhar durante o deslocamento.
void seguirDirecaoComGiro(float anguloGraus, int velocidade, int cmdGiro) {
  int velocidadeAlvo = constrain(velocidade, 0, velocidade_maxima);

  // Mantem a mesma decomposicao de translacao da funcao base.
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
  // O termo e somado em todas as rodas para sobrepor rotacao ao deslocamento.
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

// Para todos os motores desligando PWM e deixando pontes em estado neutro.
void pararMotores() {
  ledcWrite(PWM_CH1, 0);
  ledcWrite(PWM_CH2, 0);
  ledcWrite(PWM_CH3, 0);
  ledcWrite(PWM_CH4, 0);

  // Coloca todas as entradas da ponte em neutro para evitar arrasto eletrico.
  digitalWrite(IN1_1_A, LOW);
  digitalWrite(IN2_1_A, LOW);
  digitalWrite(IN1_2_A, LOW);
  digitalWrite(IN2_2_A, LOW);
  digitalWrite(IN1_1_B, LOW);
  digitalWrite(IN2_1_B, LOW);
  digitalWrite(IN1_2_B, LOW);
  digitalWrite(IN2_2_B, LOW);
}

// Gira o robo no proprio eixo usando o mesmo comando de giro para as rodas.
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

// Normaliza um erro angular para a faixa [-180, 180].
float normalizarErro180(float erro) {
  while (erro > 180.0f) erro -= 360.0f;
  while (erro < -180.0f) erro += 360.0f;
  return erro;
}

// Normaliza um angulo absoluto para a faixa [0, 360).
float normalizarAngulo360(float ang) {
  while (ang >= 360.0f) ang -= 360.0f;
  while (ang < 0.0f) ang += 360.0f;
  return ang;
}

// Ajusta o angulo da bola para um angulo de comando mais estavel de movimento.
float mapearAnguloBolaParaMovimento(float anguloBolaGraus) {
  float ang = normalizarAngulo360(anguloBolaGraus);
  if (ang >= 20.0f && ang < 45.0f) return 90.0f;
  if (ang >= 45.0f && ang < 90.0f) return 135.0f;
  if (ang >= 90.0f && ang < 135.0f) return 180.0f;
  if (ang >= 315.0f && ang < 340.0f) return 270.0f;
  if (ang >= 270.0f && ang < 315.0f) return 180.0f;
  if (ang >= 135.0f && ang < 180.0f) return 220.0f;
  if (ang >= 180.0f && ang < 270.0f) return 140.0f;
  
  return ang;
}

// Detecta a faixa frontal do IR em torno de 0°, tratando a transicao 360° -> 0°.
bool irNaFaixaFrontal(float anguloBolaGraus) {
  float ang = normalizarAngulo360(anguloBolaGraus);
  return (ang >= 335.0f || ang <= 25.0f);
}

// Move para frente com PWM fixo nas rodas, mantendo a correcao de giro do alinhamento.
void moverFrenteComGiro(int velocidadePwm, int cmdGiro) {
  int pwmBase = constrain(velocidadePwm, 0, 255);

  // Padrao de rodas equivalente ao avancar para frente no referencial atual da base.
  float v1 = (float)pwmBase;
  float v2 = (float)pwmBase;
  float v3 = -(float)pwmBase;
  float v4 = -(float)pwmBase;

  // Mantem o mesmo sentido de compensacao de giro usado no restante da locomocao.
  float termoGiro = -GANHO_GIRO_MISTO * (float)cmdGiro;
  v1 += termoGiro;
  v2 += termoGiro;
  v3 += termoGiro;
  v4 += termoGiro;

  Motor_1((int)v1);
  Motor_2((int)v2);
  Motor_3((int)v3);
  Motor_4((int)v4);
}

// Zera os estados internos do PID de alinhamento.
void resetPidBussola() {
  pidBusIntegral = 0.0f;
  pidBusErroAnterior = 0.0f;
  pidBusUltimoMs = 0;
}

// Calcula a saida assinada do PID de alinhamento, com limites e anti-windup.
int calcularSaidaPidBussola(float erroGraus) {
  unsigned long agora = millis();
  float dt = 0.02f;

  // Calcula o passo de tempo real do controle e limita extremos para robustez.
  if (pidBusUltimoMs != 0) {
    dt = (agora - pidBusUltimoMs) / 1000.0f;
    if (dt < 0.005f) dt = 0.005f;
    if (dt > 0.2f) dt = 0.2f;
  }
  pidBusUltimoMs = agora;

  // Termo integral com anti-windup por saturacao simples.
  pidBusIntegral += erroGraus * dt;
  if (pidBusIntegral > PID_BUS_INTEGRAL_MAX) pidBusIntegral = PID_BUS_INTEGRAL_MAX;
  if (pidBusIntegral < -PID_BUS_INTEGRAL_MAX) pidBusIntegral = -PID_BUS_INTEGRAL_MAX;

  // Derivada discreta baseada no erro anterior.
  float derivada = (erroGraus - pidBusErroAnterior) / dt;
  pidBusErroAnterior = erroGraus;

  // Monta a acao de controle, converte para modulo e aplica saturacoes.
  float u = PID_BUS_KP * erroGraus + PID_BUS_KI * pidBusIntegral + PID_BUS_KD * derivada;
  int saida = (int)fabsf(u);
  if (saida < PID_BUS_SAIDA_MIN) saida = PID_BUS_SAIDA_MIN;
  if (saida > PID_BUS_SAIDA_MAX) saida = PID_BUS_SAIDA_MAX;
  if (saida > VELOCIDADE_GIRO_ALINHAMENTO) saida = VELOCIDADE_GIRO_ALINHAMENTO;

  // O sinal da saida preserva o sentido do erro angular.
  return (u >= 0.0f) ? saida : -saida;
}

// Estado do filtro vetorial da bussola e telemetria de gol/linha.
bool filtroBussolaInicializado = false;
float bussolaFiltroX = 1.0f;
float bussolaFiltroY = 0.0f;
float erroGolGraus = 0.0f;
bool golDetectado = false;
uint16_t golPixels = 0;
float anguloLinhaPe = -1.0f;
bool linhaDetectada = false;
unsigned long ultimoRxLinhaMs = 0;
const int VELOCIDADE_FUGA_LINHA = 170;
bool fugindoLinhaAgora = false;
float anguloFugaLinhaCmd = 0.0f;

// Descarta leituras antigas de linha para nao manter a fuga ativa com dado obsoleto.
void atualizarValidadeLinha() {
  if (linhaDetectada && (millis() - ultimoRxLinhaMs) > TIMEOUT_LINHA_MS) {
    linhaDetectada = false;
    anguloLinhaPe = -1.0f;
  }
}

// Envia periodicamente para a Cabeca a cor de gol selecionada no menu.
void enviarCorGolParaCabeca() {
  if (!corGolPendenteEnvio) {
    return;
  }

  // Limita a taxa de reenvio para nao poluir a serial.
  if ((millis() - ultimoEnvioCorGolMs) < INTERVALO_ENVIO_COR_GOL_MS) {
    return;
  }

  // A Cabeca confirma com "CFG:GOL:OK" quando assumir a configuracao.
  Serial1.print("CFG:GOL:");
  Serial1.println(corGolAzul ? "1" : "0");
  ultimoEnvioCorGolMs = millis();
}

// Desenha a tela principal de menu no display OLED.
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

// Desenha as telas de calibracao (principal, gol, bussola e teste de camera).
void desenharSubmenuCalibracao() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("=== CALIBRACAO ===");
  display.println();

  // Primeiro nivel: escolhe entre gol, bussola, teste da camera ou voltar.
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
      display.fillRect(0, 26, 128, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 28);
      display.println("BUSSOLA");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 28);
      display.println("BUSSOLA");
    }

    if (itemSubMenu == 2) {
      display.fillRect(0, 36, 128, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 38);
      display.println("IR");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 38);
      display.println("IR");
    }

    if (itemSubMenu == 3) {
      display.fillRect(0, 46, 128, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 48);
      display.println("ULTRA");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 48);
      display.println("ULTRA");
    }

    if (itemSubMenu == 4) {
      display.fillRect(0, 56, 128, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 58);
      display.println("TESTE CAM");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 58);
      display.println("TESTE CAM");
    }

    if (itemSubMenu == 5) {
      display.fillRect(96, 0, 32, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(98, 2);
      display.println("VOLTAR");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(98, 2);
      display.println("VOLTAR");
    }

  // Segundo nivel: define se o gol de referencia sera amarelo ou azul.
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

  // Tela de diagnostico do IR vindo da Cabeca.
  } else if (subMenuCalibracao == SUBMENU_IR) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("=== TESTE IR ===");
    display.println();
    display.print("COM CABECA: ");
    display.println(comunicacaoCabecaOK ? "OK" : "FALHA");
    display.print("ANGULO IR: ");
    if (irDetectado) {
      display.print(anguloIr, 1);
      display.println(" deg");
    } else {
      display.println("SEM BOLA");
    }
    display.println();
    display.println("BTN1/2/3 VOLTAR");

  // Tela de diagnostico dos ultrassonicos vindos da Cabeca.
  } else if (subMenuCalibracao == SUBMENU_ULTRA) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("=== TESTE ULTRA ===");
    display.println();
    if (ultrasValidos && ((millis() - ultimoRxUltraMs) < TIMEOUT_ULTRA_MS)) {
      display.print("D: ");
      display.print(ultraDcm, 1);
      display.println(" cm");
      display.print("E: ");
      display.print(ultraEcm, 1);
      display.println(" cm");
      display.print("F: ");
      display.print(ultraFcm, 1);
      display.println(" cm");
      display.print("T: ");
      display.print(ultraTcm, 1);
      display.println(" cm");
    } else {
      display.println("SEM DADOS");
      display.println("ULTRA DA CABECA");
      display.println("AGUARDANDO...");
    }
    display.println("BTN1/2/3 VOLTAR");

  // Tela de diagnostico da camera, sem alteracao de parametros.
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

  // Tela de calibracao da bussola: mostra leitura atual e valor salvo.
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

// Desenha a tela de operacao com telemetria de jogo e modos ativos.
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

    // Quando houver linha, mostra tambem o angulo efetivo de fuga aplicado.
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

// Mostra uma tela curta com status de comunicacao entre placas.
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

// Mostra tela de falha quando o handshake com a Cabeca cai.
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

// Escolhe qual tela deve ser renderizada conforme estado e comunicacao.
void desenharTelaAtual() {
  if (millis() < mostrarStatusAte) {
    desenharStatusPlacas();
    return;
  }

  // Sem handshake valido, a tela de falha tem prioridade absoluta.
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

// Trata eventos de botao e navega entre menu, calibracoes e operacao.
void processarEventoBotao(uint8_t botao) {
  if (estadoAtual == MENU) {
    // BTN1 sobe, BTN2 desce, BTN3 confirma.
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
    // Navegacao circular entre as opcoes do submenu principal.
    if (botao == 1) {
      itemSubMenu--;
      if (itemSubMenu < 0) itemSubMenu = 5;
    } else if (botao == 2) {
      itemSubMenu++;
      if (itemSubMenu > 5) itemSubMenu = 0;
    } else if (botao == 3) {
      if (itemSubMenu == 0) {
        subMenuCalibracao = SUBMENU_GOL;
        itemSubMenu = corGolAzul ? 1 : 0;
      } else if (itemSubMenu == 1) {
        subMenuCalibracao = SUBMENU_BUSSOLA;
        itemSubMenu = 0;
      } else if (itemSubMenu == 2) {
        subMenuCalibracao = SUBMENU_IR;
        itemSubMenu = 0;
      } else if (itemSubMenu == 3) {
        subMenuCalibracao = SUBMENU_ULTRA;
        itemSubMenu = 0;
      } else if (itemSubMenu == 4) {
        subMenuCalibracao = SUBMENU_CAMERA;
        itemSubMenu = 0;
      } else {
        estadoAtual = MENU;
        itemSelecionado = 0;
      }
    }
    return;
  }

  if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_GOL) {
    // Seleciona a cor do gol e marca envio pendente para a Cabeca.
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
      // Persistencia local da referencia de heading calibrada.
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

  if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_IR) {
    if (botao == 1 || botao == 2 || botao == 3) {
      subMenuCalibracao = SUBMENU_PRINCIPAL;
      itemSubMenu = 2;
    }
    return;
  }

  if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_ULTRA) {
    if (botao == 1 || botao == 2 || botao == 3) {
      subMenuCalibracao = SUBMENU_PRINCIPAL;
      itemSubMenu = 3;
    }
    return;
  }

  if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_CAMERA) {
    if (botao == 1 || botao == 2 || botao == 3) {
      subMenuCalibracao = SUBMENU_PRINCIPAL;
      itemSubMenu = 4;
    }
    return;
  }

  if (estadoAtual == INICIAR && botao == 3) {
    estadoAtual = MENU;
  }
}

// Interpreta mensagens recebidas da Cabeca e atualiza estados locais.
void processarMensagemCabeca(String msg) {
  // Normaliza o frame para simplificar comparacoes de protocolo.
  msg.trim();
  msg.toUpperCase();

  // Handshake simples para indicar que a serial esta viva.
  if (msg == "OI") {
    comunicacaoCabecaOK = true;
    ultimoRxCabeca = millis();
    return;
  }

  // Status agregado das outras placas encaminhado pela Cabeca.
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

  // Eventos de botoes chegam como BTN:1 / BTN:2 / BTN:3.
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

  // Angulo IR negativo indica ausencia de bola detectada.
  if (msg.startsWith("IR:")) {
    String valorIr = msg.substring(3);
    valorIr.trim();
    float novoAngulo = valorIr.toFloat();

    if (novoAngulo < 0.0f) {
      irDetectado = false;
      anguloIr = -1.0f;
    } else {
      // Teste agressivo: rejeitar qualquer 30 exato ou muito proximo
      bool eh30Graus = fabsf(novoAngulo - 30.0f) <= 1.0f;
      
      if (eh30Graus) {
        // Descartar imediatamente como sem sinal de bola
        irDetectado = false;
        anguloIr = -1.0f;
      } else {
        irDetectado = true;
        anguloIr = novoAngulo;
      }
    }

    comunicacaoCabecaOK = true;
    ultimoRxCabeca = millis();
    return;
  }

  // Confirmacao do recebimento da configuracao da cor do gol.
  if (msg == "CFG:GOL:OK") {
    corGolPendenteEnvio = false;
    mensagemBotao = corGolAzul ? "GOL AZUL OK" : "GOL AMARELO OK";
    comunicacaoCabecaOK = true;
    ultimoRxCabeca = millis();
    return;
  }

  // Leitura de bussola filtrada em coordenadas cartesianas para evitar saltos em 0/360.
  if (msg.startsWith("BUS:")) {
    String valorBus = msg.substring(4);
    valorBus.trim();
    float angBus = normalizarAngulo360(valorBus.toFloat());

    // Converte heading em vetor unitario.
    float rad = angBus * PI / 180.0f;
    float xNovo = cosf(rad);
    float yNovo = sinf(rad);

    // Aplica filtro exponencial no vetor e depois reconverte para angulo.
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

  // Telemetria de gol: erro angular, flag de deteccao, pixels e opcionalmente estado da camera.
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

      // Protocolos mais novos podem incluir um quarto campo indicando camera OK.
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

  // Linha segue a mesma convencao do IR: angulo negativo significa sem leitura valida.
  if (msg.startsWith("LIN:")) {
    String sLinha = msg.substring(4);
    sLinha.trim();
    float novoAng = sLinha.toFloat();
    linhaDetectada = (novoAng >= 0.0f);
    anguloLinhaPe = novoAng;
    ultimoRxLinhaMs = millis();
    comunicacaoCabecaOK = true;
    ultimoRxCabeca = millis();
    return;
  }

  // Distancias dos ultrassonicos vindas da Cabeca no formato ULT:D,E,F,T.
  if (msg.startsWith("ULT:")) {
    String payload = msg.substring(4);
    int p1 = payload.indexOf(',');
    int p2 = payload.indexOf(',', p1 + 1);
    int p3 = payload.indexOf(',', p2 + 1);
    if (p1 > 0 && p2 > p1 && p3 > p2) {
      String sD = payload.substring(0, p1);
      String sE = payload.substring(p1 + 1, p2);
      String sF = payload.substring(p2 + 1, p3);
      String sT = payload.substring(p3 + 1);
      sD.trim();
      sE.trim();
      sF.trim();
      sT.trim();

      ultraDcm = sD.toFloat();
      ultraEcm = sE.toFloat();
      ultraFcm = sF.toFloat();
      ultraTcm = sT.toFloat();
      ultrasValidos = (ultraDcm >= 0.0f && ultraEcm >= 0.0f && ultraFcm >= 0.0f && ultraTcm >= 0.0f);
      ultimoRxUltraMs = millis();
      comunicacaoCabecaOK = true;
      ultimoRxCabeca = millis();
    }
    return;
  }

  // Estado da chave do kicker reportado pela Cabeca.
  if (msg.startsWith("KIK:")) {
    String sKik = msg.substring(4);
    sKik.trim();
    if (sKik == "0" || sKik == "1") {
      // Protocolo: 0 = chave acionada, 1 = chave nao acionada
      kickerAtivado = (sKik == "0");
      kickerRecebido = true;
      comunicacaoCabecaOK = true;
      ultimoRxCabeca = millis();
    }
  }
}

// Acumula bytes da serial da Cabeca e despacha mensagens completas por linha.
void lerSerialCabeca() {
  while (Serial1.available() > 0) {
    char c = (char)Serial1.read();

    // O protocolo e orientado a linha; \r e \n encerram o frame atual.
    if (c == '\n' || c == '\r') {
      if (bufferSerial.length() > 0) {
        processarMensagemCabeca(bufferSerial);
        bufferSerial = "";
      }
      continue;
    }

    // Limita o tamanho do buffer para evitar crescer indefinidamente com ruido serial.
    if (bufferSerial.length() < 32) {
      bufferSerial += c;
    }
  }
}

// Inicializa perifericos, faz handshake inicial e prepara o estado de operacao.
void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RX_CABECA, TX_CABECA);

  // Recupera da EEPROM o heading salvo como referencia de calibracao.
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_ADDR_BUSSOLA, headingBussolaSalvo);
  if (headingBussolaSalvo < 0 || headingBussolaSalvo >= 360) {
    headingBussolaSalvo = 0;
  }

  // Configura todas as saidas da ponte H e o pino do kicker.
  pinMode(IN1_1_A, OUTPUT);
  pinMode(IN2_1_A, OUTPUT);
  pinMode(IN1_2_A, OUTPUT);
  pinMode(IN2_2_A, OUTPUT);
  pinMode(IN1_1_B, OUTPUT);
  pinMode(IN2_1_B, OUTPUT);
  pinMode(IN1_2_B, OUTPUT);
  pinMode(IN2_2_B, OUTPUT);
  pinMode(KICKER_PIN, OUTPUT);
  digitalWrite(KICKER_PIN, LOW);

  // Associa cada pino PWM ao seu respectivo canal do ESP32.
  ledcSetup(PWM_CH1, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_1_A, PWM_CH1);
  ledcSetup(PWM_CH2, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_2_A, PWM_CH2);
  ledcSetup(PWM_CH3, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_1_B, PWM_CH3);
  ledcSetup(PWM_CH4, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_2_B, PWM_CH4);
  pararMotores();

  // Inicializa I2C e display antes de mostrar qualquer feedback ao operador.
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

  // Durante alguns segundos, tenta fazer handshake inicial com a Cabeca.
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

// Laco principal: comunica, atualiza controle de movimento e redesenha interface.
void loop() {
  // Mantem o heartbeat da serial para detectar se a Cabeca continua ativa.
  if (millis() - ultimoEnvioOi >= INTERVALO_OI_MS) {
    Serial1.println("oi");
    ultimoEnvioOi = millis();
  }

  // Atualiza entradas vindas da Cabeca e envia configuracoes pendentes.
  lerSerialCabeca();
  atualizarValidadeLinha();
  enviarCorGolParaCabeca();
  atualizarKicker();

  // Se a Cabeca ficar silenciosa alem do timeout, derruba o estado de comunicacao.
  if ((millis() - ultimoRxCabeca) > TIMEOUT_COM_MS) {
    comunicacaoCabecaOK = false;
  }

  if (estadoAtual == INICIAR && comunicacaoCabecaOK) {
    // Alinhamento usa erro do gol (camera), igual ao teste PIDBussola.
    erroAlinhamentoGraus = erroGolGraus;
    fugindoLinhaAgora = false;

    // Prioridade de movimento: LINHA > BOLA > ALINHAR NO EIXO
    if (linhaDetectada) {
      // Ao ver a linha, a resposta imediata e fugir na direcao oposta.
      fugindoLinhaAgora = true;
      alinhandoAgora = false;
      resetPidBussola();
      anguloFugaLinhaCmd = normalizarErro180(anguloLinhaPe - 180.0f);
      seguirDirecaoPorAngulo(anguloFugaLinhaCmd, VELOCIDADE_FUGA_LINHA);
    } else {
      int cmdPidAssinado = 0;

      // So gira para alinhar quando o gol estiver visivel e fora da tolerancia.
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
        // Na faixa frontal do IR, avanca diretamente com 200 PWM mantendo a compensacao de giro.
        if (irNaFaixaFrontal(anguloIr)) {
          moverFrenteComGiro(VELOCIDADE_IR_FRONTAL_PWM, cmdPidAssinado);
        } else {
          // Fora da faixa frontal, segue usando o mapeamento angular ja existente.
          float anguloMovimento = mapearAnguloBolaParaMovimento(anguloIr);
          seguirDirecaoComGiro(anguloMovimento, velocidade_maxima, cmdPidAssinado);
        }
      } else if (precisaAlinhar) {
        // Sem bola, mas com gol fora do centro, faz alinhamento puro no eixo.
        girarNoEixo(cmdPidAssinado);
      } else {
        // Sem linha, sem bola e sem necessidade de alinhar: permanece parado.
        pararMotores();
      }
    }
  } else {
    // Fora do modo de jogo, zera controle e mantem a base parada.
    alinhandoAgora = false;
    fugindoLinhaAgora = false;
    resetPidBussola();
    pararMotores();
  }

  // Redesenha o OLED com taxa limitada para evitar flicker excessivo.
  static unsigned long ultimaTela = 0;
  if ((millis() - ultimaTela) > 120) {
    desenharTelaAtual();
    ultimaTela = millis();
  }

  delay(5);
}
