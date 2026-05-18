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
// Papel recebido da Cabeca: true = atacante, false = defensor.
bool papelAtacante = true;
bool papelAtacanteAnterior = true;  // Para detectar mudancas

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
float ultraRemotoDcm = -1.0f;
float ultraRemotoEcm = -1.0f;
float ultraRemotoFcm = -1.0f;
float ultraRemotoTcm = -1.0f;
bool ultrasValidos = false;
bool ultrasRemotosValidos = false;
unsigned long ultimoRxUltraMs = 0;
unsigned long ultimoRxUltraRemotoMs = 0;
bool kickerRecebido = false;
bool kickerAtivado = false;  // true quando chave acionada (valor 0 vindo da cabeca)
bool pulsoKickerAtivo = false;
unsigned long inicioPulsoKickerMs = 0;
unsigned long ultimoDisparoKickerMs = 0;

// ===== NOVOS: dados de camera (bola + 2 gols) =====
int16_t cameraBallAngle = -999;
uint16_t cameraBallDist = 0;
int16_t cameraBlueAngle = -999;
uint16_t cameraBlueDist = 0;
int16_t cameraYellowAngle = -999;
uint16_t cameraYellowDist = 0;
bool cameraDadosValidos = false;
unsigned long ultimoRxCameraMs = 0;
// ===== FIM novos dados camera =====

// Status da comunicacao ESP-NOW entre as Cabecas.
bool espnowOK = false;
bool sozinho = true;
unsigned long ultimoRxEspnowMs = 0;

// Estados principais da interface/operacao.
enum Estado { MENU, CALIBRACAO, INICIAR };
Estado estadoAtual = MENU;
int itemSelecionado = 0;

// Submenus disponiveis na tela de calibracao.
enum SubMenuCalibracao { SUBMENU_PRINCIPAL, SUBMENU_GOL, SUBMENU_BUSSOLA, SUBMENU_IR, SUBMENU_ULTRA, SUBMENU_CAMERA, SUBMENU_ESPNOW };
SubMenuCalibracao subMenuCalibracao = SUBMENU_PRINCIPAL;
int itemSubMenu = 0;

// Temporizacao da comunicacao e limites gerais de velocidade.
const unsigned long INTERVALO_OI_MS = 1000;
const unsigned long TIMEOUT_COM_MS = 3000;
const unsigned long TIMEOUT_BUSSOLA_MS = 800;
const unsigned long TIMEOUT_LINHA_MS = 150;
const unsigned long TIMEOUT_ULTRA_MS = 1000;
const unsigned long TIMEOUT_CAMERA_MS = 1000;
// Velocidade Maxima do robô - Vamos alterar aqui!
const int velocidade_maxima = 220;
const bool MOVIMENTO_BOLA_HABILITADO = false;



// Endereco e tamanho usados para persistir a bussola na EEPROM.
const int EEPROM_SIZE = 64;
const int EEPROM_ADDR_BUSSOLA = 0;

// Parametros do alinhamento por camera + bussola.
const float TOLERANCIA_ALINHAMENTO_GRAUS = 10.0f;
const float JANELA_FUZZY_ALINHAMENTO_GRAUS = 30.0f;
const int VELOCIDADE_GIRO_ALINHAMENTO = VELOCIDADE_GIRO;
const int SINAL_GIRO_PID = SINAL_GIRO;

// Ganhos e saturacoes do PID usado para girar rumo ao gol.
const float PID_BUS_KP = 0.8f;
const float PID_BUS_KI = 0.01f;
const float PID_BUS_KD = 0.5f;
const float PID_BUS_INTEGRAL_MAX = 120.0f;
const int PID_BUS_SAIDA_MIN = 30;
const int PID_BUS_SAIDA_MAX = 180;
const float GANHO_GIRO_MISTO = 0.7f;

// PID dedicado ao giro do goleiro usando apenas a linha (zonas A e B) como referencia.
const float PID_LINHA_GOL_KP = 0.9f;
const float PID_LINHA_GOL_KI = 0.00f;
const float PID_LINHA_GOL_KD = 0.55f;
const float PID_LINHA_GOL_INTEGRAL_MAX = 90.0f;
const int PID_LINHA_GOL_SAIDA_MIN = 30;
const int PID_LINHA_GOL_SAIDA_MAX = 220;

// Velocidade dedicada para ataque frontal quando a bola estiver entre 330° e 30°.
const int VELOCIDADE_IR_FRONTAL_PWM = 200;
const int VELOCIDADE_IR_FAIXA_REDUZIDA_PWM = 140;
const int PASSO_RAMPA_PWM = 16;
const unsigned long TRANSICAO_ANGULO_IR_MIN_MS = 50;
const unsigned long TRANSICAO_ANGULO_IR_MAX_MS = 100;
const float TRANSICAO_ANGULO_IR_MS_POR_GRAU = 2.0f;
const float PASSO_ANGULO_IR_GRAUS = 5.0f;

// Referencia salva da bussola e estados auxiliares do controle.
int headingBussolaSalvo = 0;
bool bussolaValida = false;
unsigned long ultimoRxBussolaMs = 0;
float erroAlinhamentoGraus = 0.0f;
bool alinhandoAgora = false;
bool corGolPendenteEnvio = true;
unsigned long ultimoEnvioCorGolMs = 0;
const unsigned long INTERVALO_ENVIO_COR_GOL_MS = 500;
const unsigned long TEMPO_CAMERA_SEM_IR_PARA_IGNORAR_LINHA_MS = 1000;
const unsigned long RETENCAO_FUGA_LINHA_MS = 250;

// Estado interno do PID entre iteracoes do loop.
float pidBusIntegral = 0.0f;
float pidBusErroAnterior = 0.0f;
unsigned long pidBusUltimoMs = 0;
float pidLinhaGolIntegral = 0.0f;
float pidLinhaGolErroAnterior = 0.0f;
unsigned long pidLinhaGolUltimoMs = 0;
unsigned long inicioCameraSemIrMs = 0;

// Estado da rampa angular para evitar saltos bruscos entre faixas do IR.
float anguloIrSuaveAtual = 0.0f;
float anguloIrSuaveInicio = 0.0f;
float anguloIrSuaveAlvo = 0.0f;
unsigned long inicioTransicaoIrMs = 0;
unsigned long duracaoTransicaoIrMs = TRANSICAO_ANGULO_IR_MIN_MS;
bool anguloIrSuaveInicializado = false;

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

// Suaviza variacoes de comando para reduzir tranco e excesso de inercia.
int aplicarRampaPwm(int alvo, int atual, int passoMaximo) {
  int delta = alvo - atual;
  if (delta > passoMaximo) return atual + passoMaximo;
  if (delta < -passoMaximo) return atual - passoMaximo;
  return alvo;
}

void aplicarComandoMotoresComRampa(int v1Alvo, int v2Alvo, int v3Alvo, int v4Alvo) {
  static int v1Atual = 0;
  static int v2Atual = 0;
  static int v3Atual = 0;
  static int v4Atual = 0;

  int alvo1 = constrain(v1Alvo, -255, 255);
  int alvo2 = constrain(v2Alvo, -255, 255);
  int alvo3 = constrain(v3Alvo, -255, 255);
  int alvo4 = constrain(v4Alvo, -255, 255);

  v1Atual = aplicarRampaPwm(alvo1, v1Atual, PASSO_RAMPA_PWM);
  v2Atual = aplicarRampaPwm(alvo2, v2Atual, PASSO_RAMPA_PWM);
  v3Atual = aplicarRampaPwm(alvo3, v3Atual, PASSO_RAMPA_PWM);
  v4Atual = aplicarRampaPwm(alvo4, v4Atual, PASSO_RAMPA_PWM);

  Motor_1(v1Atual);
  Motor_2(v2Atual);
  Motor_3(v3Atual);
  Motor_4(v4Atual);
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

  aplicarComandoMotoresComRampa((int)v1, (int)v2, (int)v3, (int)v4);
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

  aplicarComandoMotoresComRampa((int)v1, (int)v2, (int)v3, (int)v4);
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
  aplicarComandoMotoresComRampa(-vel, -vel, -vel, -vel);
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

// Converte o angulo de gol para o referencial do defensor (inversao de 180 graus).
// Aplica duas etapas:
// 1) inverter para tras (+180)
// 2) espelhar o angulo resultante
// Ex.: 30 -> 150, 200 -> 340.
float converterAnguloGolParaDefensor(float anguloGolGraus) {
  float invertido = normalizarAngulo360(anguloGolGraus + 180.0f);
  return normalizarAngulo360(360.0f - invertido);
}

// Calcula o erro angular atual do robo em relacao a referencia salva da bussola.
float calcularErroAngularCampo() {
  // A correção do campo vem do mesmo alinhamento usado pelo gol invertido.
  return erroAlinhamentoGraus;
}

// Corrige um angulo relativo ao robo para o referencial do campo.
float corrigirAnguloParaCampo(float anguloLocalGraus, float erroAngularGraus) {
  return normalizarAngulo360(anguloLocalGraus + erroAngularGraus);
}

// Regras fixas de deslocamento lateral baseadas no angulo da bola corrigido no campo.
// Direita real (20..120)  -> comando 90 + erroAngular
// Esquerda real (240..340)-> comando 270 - erroAngular
bool calcularComandoLateralPorBolaCorrigida(float anguloBolaCorrigido,
                                            float erroAngular,
                                            float &anguloComando) {
  float ang = normalizarAngulo360(anguloBolaCorrigido);

  if (ang >= 20.0f && ang <= 120.0f) {
    anguloComando = normalizarAngulo360(90.0f - erroAngular);
    return true;
  }

  if (ang >= 240.0f && ang <= 340.0f) {
    anguloComando = normalizarAngulo360(270.0f + erroAngular);
    return true;
  }

  return false;
}

// Aceita apenas payload numerico simples para evitar toFloat() cair silenciosamente em 0.
bool payloadNumericoValido(const String &texto) {
  if (texto.length() == 0) {
    return false;
  }

  bool encontrouDigito = false;
  bool encontrouPonto = false;
  for (size_t i = 0; i < texto.length(); i++) {
    char c = texto.charAt(i);
    if (c >= '0' && c <= '9') {
      encontrouDigito = true;
      continue;
    }
    if (c == '.' && !encontrouPonto) {
      encontrouPonto = true;
      continue;
    }
    if ((c == '+' || c == '-') && i == 0) {
      continue;
    }
    return false;
  }

  return encontrouDigito;
}

void atualizarValidadeBussola() {
  if (bussolaValida && (millis() - ultimoRxBussolaMs) > TIMEOUT_BUSSOLA_MS) {
    bussolaValida = false;
  }
}

// Mantem um snapshot universal da camera: a serial atualiza os campos e este helper
// decide apenas se o pacote ainda esta recente para qualquer tela ou logica.
bool cameraPacoteRecente() {
  return (ultimoRxCameraMs > 0) && ((millis() - ultimoRxCameraMs) <= TIMEOUT_CAMERA_MS);
}

void atualizarValidadeCamera() {
  cameraDadosValidos = cameraPacoteRecente();
}

bool cameraTemBolaValida() {
  return cameraPacoteRecente() && (cameraBallAngle != -999);
}

// Atacante usa o gol calibrado; defensor usa o gol oposto como referencia.
bool golReferenciaAzulEfetiva() {
  return papelAtacante ? corGolAzul : !corGolAzul;
}

// No defensor, espelha o alinhamento para usar o gol de tras como referencia:
// 180 -> 0, 170 -> -10, 10 -> -170, etc.
float calcularErroGolEspelhadoDefensor(float anguloGolGraus) {
  // Aqui o angulo ja deve chegar no referencial do defensor.
  return normalizarErro180(anguloGolGraus - 180.0f);
}

// Retorna o erro de alinhamento do gol conforme o papel atual.
float calcularErroGolPorPapel(float anguloGolGraus) {
  if (papelAtacante) {
    return normalizarErro180(anguloGolGraus);
  }
  return calcularErroGolEspelhadoDefensor(anguloGolGraus);
}

bool cameraTemGolSelecionadoValido(int16_t &anguloGol) {
  bool usarAzul = golReferenciaAzulEfetiva();
  uint16_t distGol = usarAzul ? cameraBlueDist : cameraYellowDist;
  anguloGol = usarAzul ? cameraBlueAngle : cameraYellowAngle;

  // No defensor, o angulo lido passa a ser o angulo invertido (+180 mod 360).
  if (!papelAtacante && anguloGol != -999) {
    float convertido = converterAnguloGolParaDefensor((float)anguloGol);
    anguloGol = (int16_t)roundf(convertido);
    if (anguloGol >= 360) {
      anguloGol = 0;
    }
  }

  // Considera valido apenas quando houver pacote recente, angulo valido e distancia positiva.
  // Isso evita falso positivo de angulo 0 sem deteccao real, que no defensor vira erro -180.
  return cameraPacoteRecente() && (anguloGol != -999) && (distGol > 0);
}

// Declaracoes antecipadas para uso no teste de centro da linha.
extern bool linhaDetectada;
extern float anguloLinhaPe;

// Ajusta o angulo da bola para um angulo de comando mais estavel de movimento.
float mapearAnguloBolaParaMovimento(float anguloBolaGraus) {
  float ang = normalizarAngulo360(anguloBolaGraus);

  if(ang >= 15.0f && ang < 45.0f) return 100.0f; 
  if(ang >= 315.0f && ang < 345.0f) return 260.0f; 




  if(ang >= 45.0f && ang < 90.0f) return 135.0f;//ok
  if(ang >= 90.0f && ang < 135.0f) return 180.0f;
  if(ang >= 270.0f && ang < 315.0f) return 225.0f;
  if(ang >= 225.0f && ang < 270.0f) return 180.0f;
  if(ang >= 180.0f && ang < 225.0f) return 135.0f;
  if(ang >= 135.0f && ang < 180.0f) return 225.0f;



  return ang;
}

// Reduz a velocidade em faixas proximas do frontal para melhorar controle lateral.
int calcularVelocidadeIrPorAngulo(float anguloBolaGraus) {
  float ang = normalizarAngulo360(anguloBolaGraus);

  if ((ang >= 15.0f && ang < 45.0f) ||
      (ang >= 315.0f && ang < 345.0f)) {
    return VELOCIDADE_IR_FAIXA_REDUZIDA_PWM;
  }

  return velocidade_maxima;
}

// Detecta a faixa frontal do IR em torno de 0°, tratando a transicao 360° -> 0°.
bool irNaFaixaFrontal(float anguloBolaGraus) {
  float ang = normalizarAngulo360(anguloBolaGraus);
  return (ang >= 340.0f || ang <= 20.0f);
}

// Para teste de centralizacao: indica para onde mover para aproximar o centro do robo ao centro da linha.
float calcularDirecaoCentroLinhaTeste() {
  if (!linhaDetectada || anguloLinhaPe < 0.0f) {
    return -1.0f;
  }

  // Exibe o comando puro da linha, sem inversao de repulsao.
  return normalizarAngulo360(anguloLinhaPe);
}

// Quantiza o comando em passos fixos para transicoes curtas e previsiveis.
float quantizarAnguloPasso(float anguloGraus, float passoGraus) {
  if (passoGraus <= 0.0f) {
    return normalizarAngulo360(anguloGraus);
  }
  float ang = normalizarAngulo360(anguloGraus);
  float quantizado = roundf(ang / passoGraus) * passoGraus;
  return normalizarAngulo360(quantizado);
}

// Faz rampa curta (100-300 ms) entre angulos IR para nao pular seco entre faixas.
float obterAnguloIrSuavizado(float anguloAlvoGraus) {
  float alvo = normalizarAngulo360(anguloAlvoGraus);
  unsigned long agora = millis();

  if (!anguloIrSuaveInicializado) {
    anguloIrSuaveAtual = alvo;
    anguloIrSuaveInicio = alvo;
    anguloIrSuaveAlvo = alvo;
    inicioTransicaoIrMs = agora;
    duracaoTransicaoIrMs = TRANSICAO_ANGULO_IR_MIN_MS;
    anguloIrSuaveInicializado = true;
    return quantizarAnguloPasso(anguloIrSuaveAtual, PASSO_ANGULO_IR_GRAUS);
  }

  float erroNovoAlvo = fabsf(normalizarErro180(alvo - anguloIrSuaveAlvo));
  if (erroNovoAlvo >= 1.0f) {
    anguloIrSuaveInicio = anguloIrSuaveAtual;
    anguloIrSuaveAlvo = alvo;
    inicioTransicaoIrMs = agora;

    float delta = fabsf(normalizarErro180(anguloIrSuaveAlvo - anguloIrSuaveInicio));
    unsigned long duracaoCalculada = (unsigned long)(delta * TRANSICAO_ANGULO_IR_MS_POR_GRAU);
    if (duracaoCalculada < TRANSICAO_ANGULO_IR_MIN_MS) {
      duracaoCalculada = TRANSICAO_ANGULO_IR_MIN_MS;
    }
    if (duracaoCalculada > TRANSICAO_ANGULO_IR_MAX_MS) {
      duracaoCalculada = TRANSICAO_ANGULO_IR_MAX_MS;
    }
    duracaoTransicaoIrMs = duracaoCalculada;
  }

  unsigned long decorridoMs = agora - inicioTransicaoIrMs;
  if (decorridoMs >= duracaoTransicaoIrMs) {
    anguloIrSuaveAtual = anguloIrSuaveAlvo;
  } else {
    float progresso = (float)decorridoMs / (float)duracaoTransicaoIrMs;
    float delta = normalizarErro180(anguloIrSuaveAlvo - anguloIrSuaveInicio);
    anguloIrSuaveAtual = normalizarAngulo360(anguloIrSuaveInicio + delta * progresso);
  }

  return quantizarAnguloPasso(anguloIrSuaveAtual, PASSO_ANGULO_IR_GRAUS);
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

  aplicarComandoMotoresComRampa((int)v1, (int)v2, (int)v3, (int)v4);
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

// Zera os estados internos do PID do goleiro por linha.
void resetPidLinhaGoleiro() {
  pidLinhaGolIntegral = 0.0f;
  pidLinhaGolErroAnterior = 0.0f;
  pidLinhaGolUltimoMs = 0;
}

// PID do giro do goleiro usando somente erro angular da linha (zonas A e B).
int calcularSaidaPidLinhaGoleiro(float erroGraus) {
  unsigned long agora = millis();
  float dt = 0.02f;

  if (pidLinhaGolUltimoMs != 0) {
    dt = (agora - pidLinhaGolUltimoMs) / 1000.0f;
    if (dt < 0.005f) dt = 0.005f;
    if (dt > 0.2f) dt = 0.2f;
  }
  pidLinhaGolUltimoMs = agora;

  pidLinhaGolIntegral += erroGraus * dt;
  if (pidLinhaGolIntegral > PID_LINHA_GOL_INTEGRAL_MAX) pidLinhaGolIntegral = PID_LINHA_GOL_INTEGRAL_MAX;
  if (pidLinhaGolIntegral < -PID_LINHA_GOL_INTEGRAL_MAX) pidLinhaGolIntegral = -PID_LINHA_GOL_INTEGRAL_MAX;

  float derivada = (erroGraus - pidLinhaGolErroAnterior) / dt;
  pidLinhaGolErroAnterior = erroGraus;

  float u = PID_LINHA_GOL_KP * erroGraus + PID_LINHA_GOL_KI * pidLinhaGolIntegral + PID_LINHA_GOL_KD * derivada;
  int saida = (int)fabsf(u);
  if (saida < PID_LINHA_GOL_SAIDA_MIN) saida = PID_LINHA_GOL_SAIDA_MIN;
  if (saida > PID_LINHA_GOL_SAIDA_MAX) saida = PID_LINHA_GOL_SAIDA_MAX;
  if (saida > VELOCIDADE_GIRO_ALINHAMENTO) saida = VELOCIDADE_GIRO_ALINHAMENTO;

  return (u >= 0.0f) ? saida : -saida;
}

// Telemetria de gol/linha.
float erroGolGraus = 0.0f;
bool golDetectado = false;
uint16_t golPixels = 0;
float anguloLinhaPe = -1.0f;
bool linhaDetectada = false;
float anguloLinhaZonaA = -1.0f;
float anguloLinhaZonaB = -1.0f;
bool linhaZonaAValida = false;
bool linhaZonaBValida = false;
unsigned long ultimoRxLinhaMs = 0;
float ultimoAnguloLinhaValido = -1.0f;
unsigned long ultimoComandoLinhaMs = 0;
const int VELOCIDADE_FUGA_LINHA = 255;
bool fugindoLinhaAgora = false;
float anguloFugaLinhaCmd = 0.0f;

// Descarta leituras antigas de linha para nao manter a fuga ativa com dado obsoleto.
void atualizarValidadeLinha() {
  if (linhaDetectada && (millis() - ultimoRxLinhaMs) > TIMEOUT_LINHA_MS) {
    linhaDetectada = false;
    anguloLinhaPe = -1.0f;
  }

  if ((millis() - ultimoRxLinhaMs) > TIMEOUT_LINHA_MS) {
    linhaZonaAValida = false;
    linhaZonaBValida = false;
    anguloLinhaZonaA = -1.0f;
    anguloLinhaZonaB = -1.0f;
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
  int16_t anguloGolMenu = -999;
  bool golVisivelMenu = cameraTemGolSelecionadoValido(anguloGolMenu);

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
  if (golVisivelMenu) {
    display.print((float)anguloGolMenu, 1);
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
  display.println("== CALIBRACAO ==");
  display.println();

  // Primeiro nivel: escolhe entre gol, bussola, camera, espnow ou voltar.
  if (subMenuCalibracao == SUBMENU_PRINCIPAL) {
    if (itemSubMenu == 0) {
      display.fillRect(0, 16, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 16);
      display.println("GOL");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 16);
      display.println("GOL");
    }

    if (itemSubMenu == 1) {
      display.fillRect(0, 24, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 24);
      display.println("BUSSOLA");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 24);
      display.println("BUSSOLA");
    }

    if (itemSubMenu == 2) {
      display.fillRect(0, 32, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 32);
      display.println("IR");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 32);
      display.println("IR");
    }

    if (itemSubMenu == 3) {
      display.fillRect(0, 40, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 40);
      display.println("ULTRA");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 40);
      display.println("ULTRA");
    }

    if (itemSubMenu == 4) {
      display.fillRect(0, 48, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 48);
      display.println("CAM");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 48);
      display.println("CAM");
    }

    if (itemSubMenu == 5) {
      display.fillRect(0, 56, 128, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 56);
      display.println("LINHA CTR");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 56);
      display.println("LINHA CTR");
    }

    if (itemSubMenu == 6) {
      display.fillRect(96, 0, 32, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(98, 2);
      display.println("VOLTA");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(98, 2);
      display.println("VOLTA");
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
      display.println("VOLTA");
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.setCursor(4, 56);
      display.println("VOLTA");
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

  // Tela de teste de centralizacao da linha.
  } else if (subMenuCalibracao == SUBMENU_ESPNOW) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("=== TESTE LINHA ===");

    display.setCursor(0, 12);
    display.print("PAPEL: ");
    display.println(papelAtacante ? "ATC" : "DEF");

    if (papelAtacante) {
      display.setCursor(0, 26);
      display.print("ANG: ");
      if (linhaDetectada && anguloLinhaPe >= 0.0f) {
        display.print(anguloLinhaPe, 1);
        display.println(" deg");
      } else {
        display.println("SEM LEITURA");
      }
    } else {
      display.setCursor(0, 24);
      display.print("A: ");
      if (linhaZonaAValida && anguloLinhaZonaA >= 0.0f) {
        display.print(anguloLinhaZonaA, 1);
        display.println(" deg");
      } else {
        display.println("SEM LEITURA");
      }

      display.setCursor(0, 36);
      display.print("B: ");
      if (linhaZonaBValida && anguloLinhaZonaB >= 0.0f) {
        display.print(anguloLinhaZonaB, 1);
        display.println(" deg");
      } else {
        display.println("SEM LEITURA");
      }
    }

    display.setCursor(0, 48);
    display.println("AUTO: PAPEL CABECA");

    display.setCursor(0, 56);
    display.println("BTN1/2/3 VOLTAR");

  // Tela de diagnostico da camera, mostrando os 6 dados (bola + 2 gols: angulo e distancia).
  } else if (subMenuCalibracao == SUBMENU_CAMERA) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("TESTE CAM");
    
    // Verifica timeout dos dados
    bool dataTimeout = !cameraPacoteRecente();

    display.setTextSize(2);
    display.setCursor(0, 16);
    
    if (!dataTimeout) {
      // Mostra os 6 dados em formato compacto para caber com fonte maior.
      display.print("B");
      display.print(cameraBallAngle);
      display.print("/");
      display.println(cameraBallDist);
      
      display.print("A");
      display.print(cameraBlueAngle);
      display.print("/");
      display.println(cameraBlueDist);
      
      display.print("M");
      display.print(cameraYellowAngle);
      display.print("/");
      display.println(cameraYellowDist);
    } else {
      display.println("SEM");
      display.println("SINAL");
    }

  // Tela de calibracao da bussola: mostra leitura atual e valor salvo.
  } else {
    display.setCursor(0, 0);
    display.println("BUSSOLA AGORA");
    display.println();
    display.print("COM: ");
    display.println((comunicacaoCabecaOK && bussolaValida) ? "OK" : "SEM DADO");
    display.setTextSize(3);
    display.setCursor(8, 20);
    if (bussolaValida) {
      display.print(headingBussolaTeste);
      display.print((char)247);
    } else {
      display.setTextSize(2);
      display.setCursor(8, 24);
      display.print("---");
    }
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
  int16_t anguloGolOperacao = -999;
  bool golVisivelOperacao = cameraTemGolSelecionadoValido(anguloGolOperacao);
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
  if (golVisivelOperacao) {
    float erroGolMostrado = calcularErroGolPorPapel((float)anguloGolOperacao);
    display.print(erroGolMostrado, 1);
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
  if (golVisivelOperacao) {
    display.print("ERRO: ");
    display.print(erroAlinhamentoGraus, 1);
    display.println(" deg");
    if (fugindoLinhaAgora) {
      display.println("MODO: FUGINDO LINHA");
    } else {
      display.println(alinhandoAgora ? "MODO: ALINHANDO GOL" : "MODO: SEGUINDO BOLA");
    }
    display.print("PIX: ");
    bool usarAzul = golReferenciaAzulEfetiva();
    display.println(usarAzul ? cameraBlueDist : cameraYellowDist);
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
      if (itemSubMenu < 0) itemSubMenu = 6;
    } else if (botao == 2) {
      itemSubMenu++;
      if (itemSubMenu > 6) itemSubMenu = 0;
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
      } else if (itemSubMenu == 5) {
        subMenuCalibracao = SUBMENU_ESPNOW;
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

  if (estadoAtual == CALIBRACAO && subMenuCalibracao == SUBMENU_ESPNOW) {
    if (botao == 1 || botao == 2 || botao == 3) {
      subMenuCalibracao = SUBMENU_PRINCIPAL;
      itemSubMenu = 5;
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

  // Leitura de bussola direta da cabeca (sem filtro, mesmo valor que a cabeca le).
  if (msg.startsWith("BUS:")) {
    String valorBus = msg.substring(4);
    valorBus.trim();

    if (!payloadNumericoValido(valorBus)) {
      bussolaValida = false;
      return;
    }

    float angBus = valorBus.toFloat();
    
    // Normaliza para faixa 0-360 e converte para inteiro.
    angBus = normalizarAngulo360(angBus);
    headingBussolaTeste = (int)(angBus + 0.5f);
    if (headingBussolaTeste >= 360) {
      headingBussolaTeste = 0;
    }

    bussolaValida = true;
    ultimoRxBussolaMs = millis();
    
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
    if (linhaDetectada) {
      ultimoAnguloLinhaValido = anguloLinhaPe;
      ultimoComandoLinhaMs = ultimoRxLinhaMs;
    }
    comunicacaoCabecaOK = true;
    ultimoRxCabeca = millis();
    return;
  }

  // Angulo de linha da zona A (0-180) no modo defensor.
  if (msg.startsWith("LINA:")) {
    String sLinhaA = msg.substring(5);
    sLinhaA.trim();
    float novoAngA = sLinhaA.toFloat();
    linhaZonaAValida = (novoAngA >= 0.0f);
    anguloLinhaZonaA = linhaZonaAValida ? novoAngA : -1.0f;
    ultimoRxLinhaMs = millis();
    comunicacaoCabecaOK = true;
    ultimoRxCabeca = millis();
    return;
  }

  // Angulo de linha da zona B (180-360) no modo defensor.
  if (msg.startsWith("LINB:")) {
    String sLinhaB = msg.substring(5);
    sLinhaB.trim();
    float novoAngB = sLinhaB.toFloat();
    linhaZonaBValida = (novoAngB >= 0.0f);
    anguloLinhaZonaB = linhaZonaBValida ? novoAngB : -1.0f;
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

  // Distancias dos ultrassonicos recebidas do outro robo via Cabeca no formato ULR:D,E,F,T.
  if (msg.startsWith("ULR:")) {
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

      ultraRemotoDcm = sD.toFloat();
      ultraRemotoEcm = sE.toFloat();
      ultraRemotoFcm = sF.toFloat();
      ultraRemotoTcm = sT.toFloat();
      ultrasRemotosValidos = (ultraRemotoDcm >= 0.0f && ultraRemotoEcm >= 0.0f && ultraRemotoFcm >= 0.0f && ultraRemotoTcm >= 0.0f);
      ultimoRxUltraRemotoMs = millis();
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
    return;
  }

  // Papel do robo definido pela Cabeca: 1 = atacante, 0 = defensor.
  if (msg.startsWith("ATC:")) {
    String sAtc = msg.substring(4);
    sAtc.trim();
    if (sAtc == "0" || sAtc == "1") {
      bool novoValor = (sAtc == "1");
      if (novoValor != papelAtacante) {
        // Detectou mudanca, envia feedback para Cabeca resincronizar Pe
        Serial0.print("ATCFB:");
        Serial0.println(novoValor ? 1 : 0);
      }
      papelAtacante = novoValor;
      papelAtacanteAnterior = papelAtacante;
      comunicacaoCabecaOK = true;
      ultimoRxCabeca = millis();
    }
    return;
  }

  // ===== NOVO: dados de camera (bola + 2 gols) =====
  // CAM:ballAngle,ballDist,blueAngle,blueDist,yellowAngle,yellowDist,cameraOK
  if (msg.startsWith("CAM:")) {
    String payload = msg.substring(4);
    int p1 = payload.indexOf(',');
    int p2 = payload.indexOf(',', p1 + 1);
    int p3 = payload.indexOf(',', p2 + 1);
    int p4 = payload.indexOf(',', p3 + 1);
    int p5 = payload.indexOf(',', p4 + 1);
    int p6 = payload.indexOf(',', p5 + 1);
    
    if (p1 > 0 && p2 > p1 && p3 > p2 && p4 > p3 && p5 > p4 && p6 > p5) {
      String sBallA = payload.substring(0, p1);
      String sBallD = payload.substring(p1 + 1, p2);
      String sBlueA = payload.substring(p2 + 1, p3);
      String sBlueD = payload.substring(p3 + 1, p4);
      String sYellA = payload.substring(p4 + 1, p5);
      String sYellD = payload.substring(p5 + 1, p6);
      String sCamOK = payload.substring(p6 + 1);
      
      sBallA.trim();
      sBallD.trim();
      sBlueA.trim();
      sBlueD.trim();
      sYellA.trim();
      sYellD.trim();
      sCamOK.trim();

      cameraBallAngle = (int16_t)sBallA.toInt();
      cameraBallDist = (uint16_t)sBallD.toInt();
      cameraBlueAngle = (int16_t)sBlueA.toInt();
      cameraBlueDist = (uint16_t)sBlueD.toInt();
      cameraYellowAngle = (int16_t)sYellA.toInt();
      cameraYellowDist = (uint16_t)sYellD.toInt();
      cameraDadosValidos = (sCamOK == "1");
      ultimoRxCameraMs = millis();

      comunicacaoCabecaOK = true;
      ultimoRxCabeca = millis();
    }
    return;
  }
  // ===== FIM dados camera =====

  // Status da comunicacao ESP-NOW entre as Cabecas reportado pela Cabeca local.
  if (msg.startsWith("ESN:")) {
    String sEsn = msg.substring(4);
    sEsn.trim();
    espnowOK = (sEsn == "1");
    sozinho = !espnowOK;
    ultimoRxEspnowMs = millis();
    comunicacaoCabecaOK = true;
    ultimoRxCabeca = millis();
    return;
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
  Serial1.begin(115200, SERIAL_8N1, RX_CABECA, TX_CABECA);

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

// Funcao de controle para o robo atacante: alinhamento com gol, movimentacao em torno da bola e logica de chute.
void atacante() {
  // O pacote da camera e mantido universalmente; no INICIAR apenas consumimos o ultimo snapshot.
  int16_t anguloGolCamera = -999;
  bool golVisivelCamera = cameraTemGolSelecionadoValido(anguloGolCamera);
  erroAlinhamentoGraus = golVisivelCamera ? calcularErroGolPorPapel((float)anguloGolCamera) : 0.0f;
  fugindoLinhaAgora = false;
  anguloFugaLinhaCmd = 0.0f;

  int cmdPidAssinado = 0;
  bool cameraBolaVisivel = cameraTemBolaValida();
  float erroAngularCampo = calcularErroAngularCampo();

  // So gira para alinhar quando o gol estiver visivel na camera e fora da tolerancia.
  bool precisaAlinhar = golVisivelCamera && (fabsf(erroAlinhamentoGraus) > TOLERANCIA_ALINHAMENTO_GRAUS);
  bool erroGrande = golVisivelCamera && (fabsf(erroAlinhamentoGraus) > 40.0f);
  if (precisaAlinhar) {
    alinhandoAgora = true;
    int cmdPid = calcularSaidaPidBussola(erroAlinhamentoGraus);
    cmdPidAssinado = -SINAL_GIRO_PID * cmdPid;
  } else {
    alinhandoAgora = false;
    resetPidBussola();
  }

  // Se ficar sem IR e com bola na camera por tempo suficiente,
  // a camera assume e a linha e ignorada temporariamente.
  if (!irDetectado && cameraBolaVisivel) {
    if (inicioCameraSemIrMs == 0) {
      inicioCameraSemIrMs = millis();
    }
  } else {
    inicioCameraSemIrMs = 0;
  }

  bool ignorarLinhaPorCamera = (inicioCameraSemIrMs != 0) &&
                               ((millis() - inicioCameraSemIrMs) >= TEMPO_CAMERA_SEM_IR_PARA_IGNORAR_LINHA_MS);

  // Forca uma janela curta de fuga para evitar perder a acao por oscilacao de frame.
  bool linhaRecenteForcada = (ultimoComandoLinhaMs > 0) &&
                             ((millis() - ultimoComandoLinhaMs) <= RETENCAO_FUGA_LINHA_MS);

  float anguloLinhaParaFuga = (linhaDetectada && anguloLinhaPe >= 0.0f) ? anguloLinhaPe : ultimoAnguloLinhaValido;

  // Erro grande: para tudo e alinha de vez.
  // Fora disso, segue bola com transicao angular curta em passos de 5 graus
  // (100-300 ms) para evitar salto seco entre direcoes.
  bool linhaValida = ((linhaDetectada && (anguloLinhaPe >= 0.0f) && !ignorarLinhaPorCamera) ||
                      (linhaRecenteForcada && (anguloLinhaParaFuga >= 0.0f)));
  if (linhaValida) {
    // Prioridade maxima: ao detectar linha, foge no sentido oposto.
    // No NEXUS, o Pe ja envia angulo em modo repulsao quando atacante=true.
    fugindoLinhaAgora = true;
    anguloFugaLinhaCmd = normalizarAngulo360(anguloLinhaParaFuga);
    seguirDirecaoPorAngulo(anguloFugaLinhaCmd, VELOCIDADE_FUGA_LINHA);
    
  } else if (erroGrande) {
    girarNoEixo(cmdPidAssinado);
  } else if (irDetectado) {
    float anguloIrCorrigido = corrigirAnguloParaCampo(anguloIr, erroAngularCampo);

    if (irNaFaixaFrontal(anguloIrCorrigido)) {
      // Na faixa frontal aplica PWM direto e mantem correcao de alinhamento do gol.
      moverFrenteComGiro(VELOCIDADE_IR_FRONTAL_PWM, cmdPidAssinado);
    } else {
      float anguloIrAlvo = mapearAnguloBolaParaMovimento(anguloIrCorrigido);
      float anguloIrComRampa = obterAnguloIrSuavizado(anguloIrAlvo);
      int velocidadeIr = calcularVelocidadeIrPorAngulo(anguloIrCorrigido);
      seguirDirecaoPorAngulo(anguloIrComRampa, velocidadeIr);
    }
  } else if (cameraBolaVisivel) {
    float anguloCameraVetorial = corrigirAnguloParaCampo((float)cameraBallAngle, erroAngularCampo);
    float anguloCameraComRampa = obterAnguloIrSuavizado(anguloCameraVetorial);
    seguirDirecaoComGiro(anguloCameraComRampa, velocidade_maxima, cmdPidAssinado);
  } else if (precisaAlinhar) {
    girarNoEixo(cmdPidAssinado);
  } else {
    pararMotores();
    delay(10);
  }
}

// Calcula o vetor de atracao da linha usando media circular vetorial dos dois pontos.
// Evita problemas de wraparound no calculo do ponto medio entre zonas A e B.
float calcularVetorAtracaoLinha(float anguloA, float anguloB) {
  float aRad = anguloA * PI / 180.0f;
  float bRad = anguloB * PI / 180.0f;
  float mx = cosf(aRad) + cosf(bRad);
  float my = sinf(aRad) + sinf(bRad);
  return normalizarAngulo360(atan2f(my, mx) * 180.0f / PI);
}

void defensor() {
  const float REFERENCIA_ZONA_A = 90.0f;
  const float REFERENCIA_ZONA_B = 270.0f;
  const float TOLERANCIA_GIRO_LINHA_GRAUS = 10.0f;

  bool temZonaA = linhaZonaAValida && (anguloLinhaZonaA >= 0.0f);
  bool temZonaB = linhaZonaBValida && (anguloLinhaZonaB >= 0.0f);

  alinhandoAgora = false;
  fugindoLinhaAgora = false;

  if (!temZonaA && !temZonaB) {
    erroAlinhamentoGraus = 0.0f;
    resetPidLinhaGoleiro();
    pararMotores();
    return;
  }

  float somaErro = 0.0f;
  int quantidadeErros = 0;

  if (temZonaA) {
    float erroA = normalizarErro180(anguloLinhaZonaA - REFERENCIA_ZONA_A);
    somaErro += erroA;
    quantidadeErros++;
  }

  if (temZonaB) {
    float erroB = normalizarErro180(anguloLinhaZonaB - REFERENCIA_ZONA_B);
    somaErro += erroB;
    quantidadeErros++;
  }

  float erroAngularLinha = (quantidadeErros > 0) ? (somaErro / (float)quantidadeErros) : 0.0f;
  erroAlinhamentoGraus = erroAngularLinha;

  // Regra principal do defensor:
  // PRECISA ALINHAR? se sim, alinha. se nao, segue a bola.
  bool precisaAlinhar = (fabsf(erroAngularLinha) >= TOLERANCIA_GIRO_LINHA_GRAUS);
  if (!precisaAlinhar) {
    alinhandoAgora = false;
    resetPidLinhaGoleiro();
    if (irDetectado) {
      if (anguloIr > 20.0f && anguloIr < 120.0f) {
        seguirDirecaoPorAngulo(90.0f, map(anguloIr, 20.0f, 120.0f, 80, 200));
        return;
      } else if (anguloIr > 240.0f && anguloIr < 360.0f) {
        seguirDirecaoPorAngulo(270.0f, map(anguloIr, 240.0f, 360.0f, 80, 200));
        return;
      }
    }
    pararMotores();
    return;
  }

  int cmdPid = calcularSaidaPidLinhaGoleiro(erroAngularLinha);
  if (fabsf(erroAngularLinha) > 20.0f) {
    cmdPid *= 2; // Aumenta agressividade se o erro for muito grande
  }
  alinhandoAgora = true;
  int cmdGiro = SINAL_GIRO_PID * cmdPid;
  girarNoEixo(cmdGiro);

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
  atualizarValidadeBussola();
  atualizarValidadeLinha();
  atualizarValidadeCamera();
  enviarCorGolParaCabeca();
  atualizarKicker();

  // Se a Cabeca ficar silenciosa alem do timeout, derruba o estado de comunicacao.
  if ((millis() - ultimoRxCabeca) > TIMEOUT_COM_MS) {
    comunicacaoCabecaOK = false;
    bussolaValida = false;
  }

  if (estadoAtual == INICIAR && comunicacaoCabecaOK) {
    if (papelAtacante) {
      atacante();
    } else {
      defensor();
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

 // delay(5);
}