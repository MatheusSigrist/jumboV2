#include "motores_movimentacao.hpp"

namespace {
MotoresMovimentacaoConfig g_cfg;
bool g_inicializado = false;
uint8_t g_confirmacoesLinha = 0;

int g_v1Atual = 0;
int g_v2Atual = 0;
int g_v3Atual = 0;
int g_v4Atual = 0;

void motor1(int vel) {
  int pwm = constrain(abs(vel), 0, 255);
  ledcWrite(g_cfg.pwm_ch1, pwm);
  if (vel <= 0) {
    digitalWrite(g_cfg.in1_1_a, HIGH);
    digitalWrite(g_cfg.in2_1_a, LOW);
  } else {
    digitalWrite(g_cfg.in1_1_a, LOW);
    digitalWrite(g_cfg.in2_1_a, HIGH);
  }
}

void motor2(int vel) {
  int pwm = constrain(abs(vel), 0, 255);
  ledcWrite(g_cfg.pwm_ch2, pwm);
  if (vel <= 0) {
    digitalWrite(g_cfg.in1_2_a, HIGH);
    digitalWrite(g_cfg.in2_2_a, LOW);
  } else {
    digitalWrite(g_cfg.in1_2_a, LOW);
    digitalWrite(g_cfg.in2_2_a, HIGH);
  }
}

void motor4(int vel) {
  int pwm = constrain(abs(vel), 0, 255);
  ledcWrite(g_cfg.pwm_ch3, pwm);
  if (vel <= 0) {
    digitalWrite(g_cfg.in1_1_b, HIGH);
    digitalWrite(g_cfg.in2_1_b, LOW);
  } else {
    digitalWrite(g_cfg.in1_1_b, LOW);
    digitalWrite(g_cfg.in2_1_b, HIGH);
  }
}

void motor3(int vel) {
  int pwm = constrain(abs(vel), 0, 255);
  ledcWrite(g_cfg.pwm_ch4, pwm);
  if (vel <= 0) {
    digitalWrite(g_cfg.in1_2_b, HIGH);
    digitalWrite(g_cfg.in2_2_b, LOW);
  } else {
    digitalWrite(g_cfg.in1_2_b, LOW);
    digitalWrite(g_cfg.in2_2_b, HIGH);
  }
}

int aplicarRampaPwm(int alvo, int atual, int passoMaximo) {
  int delta = alvo - atual;
  if (delta > passoMaximo) return atual + passoMaximo;
  if (delta < -passoMaximo) return atual - passoMaximo;
  return alvo;
}

void aplicarComandoMotoresComRampa(int v1Alvo, int v2Alvo, int v3Alvo, int v4Alvo) {
  int alvo1 = constrain(v1Alvo, -255, 255);
  int alvo2 = constrain(v2Alvo, -255, 255);
  int alvo3 = constrain(v3Alvo, -255, 255);
  int alvo4 = constrain(v4Alvo, -255, 255);

  g_v1Atual = aplicarRampaPwm(alvo1, g_v1Atual, g_cfg.passoRampaPwm);
  g_v2Atual = aplicarRampaPwm(alvo2, g_v2Atual, g_cfg.passoRampaPwm);
  g_v3Atual = aplicarRampaPwm(alvo3, g_v3Atual, g_cfg.passoRampaPwm);
  g_v4Atual = aplicarRampaPwm(alvo4, g_v4Atual, g_cfg.passoRampaPwm);

  motor1(g_v1Atual);
  motor2(g_v2Atual);
  motor3(g_v3Atual);
  motor4(g_v4Atual);
}
}

void inicializarMotoresMovimentacao(const MotoresMovimentacaoConfig& config) {
  g_cfg = config;

  pinMode(g_cfg.in1_1_a, OUTPUT);
  pinMode(g_cfg.in2_1_a, OUTPUT);
  pinMode(g_cfg.in1_2_a, OUTPUT);
  pinMode(g_cfg.in2_2_a, OUTPUT);
  pinMode(g_cfg.in1_1_b, OUTPUT);
  pinMode(g_cfg.in2_1_b, OUTPUT);
  pinMode(g_cfg.in1_2_b, OUTPUT);
  pinMode(g_cfg.in2_2_b, OUTPUT);

  ledcSetup(g_cfg.pwm_ch1, g_cfg.pwm_freq, g_cfg.pwm_res);
  ledcAttachPin(g_cfg.pwm_1_a, g_cfg.pwm_ch1);
  ledcSetup(g_cfg.pwm_ch2, g_cfg.pwm_freq, g_cfg.pwm_res);
  ledcAttachPin(g_cfg.pwm_2_a, g_cfg.pwm_ch2);
  ledcSetup(g_cfg.pwm_ch3, g_cfg.pwm_freq, g_cfg.pwm_res);
  ledcAttachPin(g_cfg.pwm_1_b, g_cfg.pwm_ch3);
  ledcSetup(g_cfg.pwm_ch4, g_cfg.pwm_freq, g_cfg.pwm_res);
  ledcAttachPin(g_cfg.pwm_2_b, g_cfg.pwm_ch4);

  g_inicializado = true;
  pararMotores();
}

void pararMotores() {
  if (!g_inicializado) {
    return;
  }

  ledcWrite(g_cfg.pwm_ch1, 0);
  ledcWrite(g_cfg.pwm_ch2, 0);
  ledcWrite(g_cfg.pwm_ch3, 0);
  ledcWrite(g_cfg.pwm_ch4, 0);

  digitalWrite(g_cfg.in1_1_a, LOW);
  digitalWrite(g_cfg.in2_1_a, LOW);
  digitalWrite(g_cfg.in1_2_a, LOW);
  digitalWrite(g_cfg.in2_2_a, LOW);
  digitalWrite(g_cfg.in1_1_b, LOW);
  digitalWrite(g_cfg.in2_1_b, LOW);
  digitalWrite(g_cfg.in1_2_b, LOW);
  digitalWrite(g_cfg.in2_2_b, LOW);

  g_v1Atual = 0;
  g_v2Atual = 0;
  g_v3Atual = 0;
  g_v4Atual = 0;
}

void girarNoEixo(int velocidade) {
  if (!g_inicializado) {
    return;
  }

  int vel = constrain(velocidade, -g_cfg.velocidadeMaxima, g_cfg.velocidadeMaxima);
  aplicarComandoMotoresComRampa(-vel, -vel, -vel, -vel);
}

void seguirDirecaoPorAngulo(float anguloGraus, int velocidade) {
  if (!g_inicializado) {
    return;
  }

  int velocidadeAlvo = constrain(velocidade, 0, g_cfg.velocidadeMaxima);

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

  float maxVel = max(max(fabsf(v1), fabsf(v2)), max(fabsf(v3), fabsf(v4)));
  if (maxVel > g_cfg.velocidadeMaxima) {
    float escala = (float)g_cfg.velocidadeMaxima / maxVel;
    v1 *= escala;
    v2 *= escala;
    v3 *= escala;
    v4 *= escala;
  }

  aplicarComandoMotoresComRampa((int)v1, (int)v2, (int)v3, (int)v4);
}

void seguirDirecaoComGiro(float anguloGraus, int velocidade, int cmdGiro) {
  if (!g_inicializado) {
    return;
  }

  int velocidadeAlvo = constrain(velocidade, 0, g_cfg.velocidadeMaxima);
  int cmdGiroLimitado = constrain(cmdGiro, -255, 255);
  float prioridadeGiro = (float)abs(cmdGiroLimitado) / 255.0f;
  float prioridadeTranslacao = 1.0f - prioridadeGiro;
  int sinalGiro = (cmdGiroLimitado > 0) ? 1 : ((cmdGiroLimitado < 0) ? -1 : 0);

  float theta = anguloGraus * PI / 180.0f;
  float vx = ((float)velocidadeAlvo * prioridadeTranslacao) * sinf(theta);
  float vy = ((float)velocidadeAlvo * prioridadeTranslacao) * cosf(theta);

  float theta1 = 45.0f * PI / 180.0f;
  float theta2 = 135.0f * PI / 180.0f;
  float theta3 = 225.0f * PI / 180.0f;
  float theta4 = 315.0f * PI / 180.0f;

  float v1 = vx * cosf(theta1) + vy * sinf(theta1);
  float v2 = vx * cosf(theta2) + vy * sinf(theta2);
  float v3 = vx * cosf(theta3) + vy * sinf(theta3);
  float v4 = vx * cosf(theta4) + vy * sinf(theta4);

  float termoGiro = -g_cfg.ganhoGiroMisto * (float)velocidadeAlvo * prioridadeGiro * (float)sinalGiro;
  v1 += termoGiro;
  v2 += termoGiro;
  v3 += termoGiro;
  v4 += termoGiro;

  float maxVel = max(max(fabsf(v1), fabsf(v2)), max(fabsf(v3), fabsf(v4)));
  if (maxVel > g_cfg.velocidadeMaxima) {
    float escala = (float)g_cfg.velocidadeMaxima / maxVel;
    v1 *= escala;
    v2 *= escala;
    v3 *= escala;
    v4 *= escala;
  }

  aplicarComandoMotoresComRampa((int)v1, (int)v2, (int)v3, (int)v4);
}


void seguirDirecaoComGiroLaterais(float anguloGraus, int velocidade, int cmdGiro) {
  if (!g_inicializado) {
    return;
  }

  int velocidadeAlvo = constrain(velocidade, 0, g_cfg.velocidadeMaxima);
  int cmdGiroLimitado = constrain(cmdGiro, -255, 255);
  float prioridadeGiro = (float)abs(cmdGiroLimitado) / 255.0f;
  float prioridadeTranslacao = 1.0f - prioridadeGiro;
  int sinalGiro = (cmdGiroLimitado > 0) ? 1 : ((cmdGiroLimitado < 0) ? -1 : 0);

  float theta = anguloGraus * PI / 180.0f;
  float vx = ((float)velocidadeAlvo * prioridadeTranslacao) * sinf(theta);
  float vy = ((float)velocidadeAlvo * prioridadeTranslacao) * cosf(theta);

  float theta1 = 45.0f * PI / 180.0f;
  float theta2 = 135.0f * PI / 180.0f;
  float theta3 = 225.0f * PI / 180.0f;
  float theta4 = 315.0f * PI / 180.0f;

  float v1 = vx * cosf(theta1) + vy * sinf(theta1);
  float v2 = vx * cosf(theta2) + vy * sinf(theta2);
  float v3 = vx * cosf(theta3) + vy * sinf(theta3);
  float v4 = vx * cosf(theta4) + vy * sinf(theta4);

  float termoGiro = (-g_cfg.ganhoGiroMisto * (float)velocidadeAlvo * prioridadeGiro * (float)sinalGiro) + 10;
  
  
  
  if((anguloGraus > 0) && (anguloGraus < 180)){
  v1 += termoGiro - 50; // 315
  v2 += termoGiro; // 225
  v3 += termoGiro; // 135
  v4 += termoGiro - 50; //45
  }else{
  v1 += termoGiro + 15; // 315
  v2 += termoGiro; // 225
  v3 += termoGiro; // 135
  v4 += termoGiro + 15; //45
  }


  float maxVel = max(max(fabsf(v1), fabsf(v2)), max(fabsf(v3), fabsf(v4)));
  if (maxVel > g_cfg.velocidadeMaxima) {
    float escala = (float)g_cfg.velocidadeMaxima / maxVel;
    v1 *= escala;
    v2 *= escala;
    v3 *= escala;
    v4 *= escala;
  }

 // corrige certo para teste
 // aplicarComandoMotoresComRampa((int)termoGiro, (int)termoGiro, (int)termoGiro, (int)termoGiro);
    aplicarComandoMotoresComRampa((int)v1, (int)v2, (int)v3, (int)v4);
}







void seguirDirecaoComGiroLateraisDefensor(float anguloGraus, int velocidade, int cmdGiro) {
  if (!g_inicializado) {
    return;
  }

  int velocidadeAlvo = constrain(velocidade, 0, g_cfg.velocidadeMaxima);
  int cmdGiroLimitado = constrain(cmdGiro, -255, 255);
  float prioridadeGiro = (float)abs(cmdGiroLimitado) / 255.0f;
  float prioridadeTranslacao = 1.0f - prioridadeGiro;
  int sinalGiro = (cmdGiroLimitado > 0) ? 1 : ((cmdGiroLimitado < 0) ? -1 : 0);

  float theta = anguloGraus * PI / 180.0f;
  float vx = ((float)velocidadeAlvo * prioridadeTranslacao) * sinf(theta);
  float vy = ((float)velocidadeAlvo * prioridadeTranslacao) * cosf(theta);

  float theta1 = 45.0f * PI / 180.0f;
  float theta2 = 135.0f * PI / 180.0f;
  float theta3 = 225.0f * PI / 180.0f;
  float theta4 = 315.0f * PI / 180.0f;

  float v1 = vx * cosf(theta1) + vy * sinf(theta1);
  float v2 = vx * cosf(theta2) + vy * sinf(theta2);
  float v3 = vx * cosf(theta3) + vy * sinf(theta3);
  float v4 = vx * cosf(theta4) + vy * sinf(theta4);

  float termoGiro = (-g_cfg.ganhoGiroMisto * (float)velocidadeAlvo * prioridadeGiro * (float)sinalGiro) + 10;
  
  
  
  if((anguloGraus > 0) && (anguloGraus < 180)){
  v1 += termoGiro - 50; // 315
  v2 += termoGiro; // 225
  v3 += termoGiro; // 135
  v4 += termoGiro - 50; //45
  }else{
  v1 += termoGiro + 15; // 315
  v2 += termoGiro; // 225
  v3 += termoGiro; // 135
  v4 += termoGiro + 15; //45
  }


  float maxVel = max(max(fabsf(v1), fabsf(v2)), max(fabsf(v3), fabsf(v4)));
  if (maxVel > g_cfg.velocidadeMaxima) {
    float escala = (float)g_cfg.velocidadeMaxima / maxVel;
    v1 *= escala;
    v2 *= escala;
    v3 *= escala;
    v4 *= escala;
  }

 // corrige certo para teste
 // aplicarComandoMotoresComRampa((int)termoGiro, (int)termoGiro, (int)termoGiro, (int)termoGiro);
    aplicarComandoMotoresComRampa((int)termoGiro, (int)termoGiro, (int)termoGiro, (int)termoGiro);
}










void moverFrenteComGiro(int velocidadePwm, int cmdGiro) {
  moverFrenteComGiro(velocidadePwm, cmdGiro, g_cfg.ganhoGiroMisto);
}

void moverFrenteComGiro(int velocidadePwm, int cmdGiro, float ganhoGiroMisto) {
  if (!g_inicializado) {
    return;
  }

  int pwmBase = constrain(velocidadePwm, 0, 255);

  float v1 = (float)pwmBase;
  float v2 = (float)pwmBase;
  float v3 = -(float)pwmBase;
  float v4 = -(float)pwmBase;

  float termoGiro = -ganhoGiroMisto * (float)cmdGiro;
  v1 += termoGiro;
  v2 += termoGiro;
  v3 += termoGiro;
  v4 += termoGiro;

  aplicarComandoMotoresComRampa((int)v1, (int)v2, (int)v3, (int)v4);
}

bool g_fugindoDaLinha = false;
float g_anguloFugaTravado = 0.0f;
unsigned long g_ultimaLinhaDetectadaMs = 0;

const unsigned long TEMPO_PERDA_LINHA_MS = 100;


bool sairDaLinha(bool linhaDetectada, float anguloLinhaGraus, int velocidadePwm,
                 float* anguloComandoSaida) {

  if (!g_inicializado) {
    return false;
  }

  bool linhaAtiva = linhaDetectada && (anguloLinhaGraus >= 0.0f);

  // Normalização prévia do ângulo
  float anguloNormalizado = anguloLinhaGraus;
  if (linhaAtiva) {
    while (anguloNormalizado >= 360.0f) anguloNormalizado -= 360.0f;
    while (anguloNormalizado < 0.0f) anguloNormalizado += 360.0f;
  }

  // Verifica se a detecção é no setor frontal (315° a 45°)
  bool eZonaFrontal = (anguloNormalizado > 315.0f || anguloNormalizado < 45.0f);

  // ============================================================
  // LÓGICA NOVA: FUGA FRONTAL (315° a 45°)
  // ============================================================
  if (g_fugindoDaLinha || (linhaAtiva && eZonaFrontal)) {

    // Primeira detecção frontal: Inverte o ângulo em +180° para RECUAR
    if (!g_fugindoDaLinha) {
      float anguloRecuo = anguloNormalizado + 180.0f;
      while (anguloRecuo >= 360.0f) anguloRecuo -= 360.0f;

      g_anguloFugaTravado = anguloRecuo;
      g_fugindoDaLinha = true;
      g_ultimaLinhaDetectadaMs = millis();

      if (anguloComandoSaida != nullptr) {
        *anguloComandoSaida = g_anguloFugaTravado;
      }

      seguirDirecaoPorAngulo(g_anguloFugaTravado, velocidadePwm);
      return true;
    }

    // Mantém o recuo enquanto a linha estiver ativa
    if (linhaAtiva) {
      g_ultimaLinhaDetectadaMs = millis();

      if (anguloComandoSaida != nullptr) {
        *anguloComandoSaida = g_anguloFugaTravado;
      }

      seguirDirecaoPorAngulo(g_anguloFugaTravado, velocidadePwm);
      return true;
    }

    // Tolerância de tempo após perder a linha
    unsigned long tempoSemLinha = millis() - g_ultimaLinhaDetectadaMs;
    if (tempoSemLinha < TEMPO_PERDA_LINHA_MS) {

      if (anguloComandoSaida != nullptr) {
        *anguloComandoSaida = g_anguloFugaTravado;
      }

      seguirDirecaoPorAngulo(g_anguloFugaTravado, velocidadePwm);
      return true;
    }

    // Fim da fuga frontal
    g_fugindoDaLinha = false;
    g_anguloFugaTravado = 0.0f;
    g_ultimaLinhaDetectadaMs = 0;

    if (anguloComandoSaida != nullptr) {
      *anguloComandoSaida = 0.0f;
    }

    return false;
  }

  // ============================================================
  // LÓGICA ANTIGA: LATERAIS E TRASEIRA (45° a 315°)
  // ============================================================
  if (linhaAtiva) {
    if (anguloComandoSaida != nullptr) {
      *anguloComandoSaida = anguloNormalizado;
    }

    seguirDirecaoPorAngulo(anguloNormalizado, velocidadePwm);
    return true;
  }

  return false;
}

void resetSairDaLinha() {
  g_confirmacoesLinha = 0;
}

namespace {
MotoresPosicionamentoConfig g_posCfg;
MotoresPosicionamentoOrientacaoHooks g_posHooks;

float g_ultraEcm = -1.0f;
float g_ultraDcm = -1.0f;
float g_ultraFcm = -1.0f;
float g_ultraTcm = -1.0f;
bool g_ultrasPosValidos = false;

float g_posicaoXcm = 91.0f;
float g_posicaoYcm = 121.5f;
float g_confiancaPos = 0.0f;
float g_anguloAlvoGraus = 0.0f;
unsigned long g_ultimaEstimativaPosMs = 0;
bool g_posicaoAtualValida = false;

float normalizarAngulo360Pos(float ang) {
  while (ang >= 360.0f) ang -= 360.0f;
  while (ang < 0.0f) ang += 360.0f;
  return ang;
}

float mapearFaixaClampedPos(float valor,
                            float entradaMin,
                            float entradaMax,
                            float saidaMin,
                            float saidaMax) {
  float denominador = entradaMax - entradaMin;
  if (fabsf(denominador) < 0.0001f) {
    return saidaMin;
  }
  float proporcao = (valor - entradaMin) / denominador;
  if (proporcao < 0.0f) proporcao = 0.0f;
  if (proporcao > 1.0f) proporcao = 1.0f;
  return saidaMin + ((saidaMax - saidaMin) * proporcao);
}

bool ultraValidoParaPosicao(float v) {
  return isfinite(v) && v > 1.0f && v < 350.0f;
}

float filtroComplementarPos(float anterior, float medicao, float confianca, float dtSec) {
  float tau = (confianca >= 0.8f) ? g_posCfg.filtroTauRapido : g_posCfg.filtroTauLento;
  if (tau < 0.02f) tau = 0.02f;
  float alpha = expf(-dtSec / tau);
  alpha = constrain(alpha + ((1.0f - confianca) * 0.18f), 0.08f, 0.96f);
  return (alpha * anterior) + ((1.0f - alpha) * medicao);
}

struct EixoPosResultado {
  float valor;
  float confianca;
};

EixoPosResultado estimarEixoPosicao(float leituraA,
                                    float leituraB,
                                    float campo,
                                    float ultimo,
                                    float dtSec) {
  const bool temA = ultraValidoParaPosicao(leituraA);
  const bool temB = ultraValidoParaPosicao(leituraB);
  const float minimo = g_posCfg.roboRaioCm;
  const float maximo = campo - g_posCfg.roboRaioCm;
  const float utilizavel = campo - (2.0f * g_posCfg.roboRaioCm);

  float medida = ultimo;
  float confianca = 0.05f;

  if (temA && temB) {
    float diretoA = constrain(leituraA + g_posCfg.roboRaioCm, minimo, maximo);
    float diretoB = constrain(campo - (leituraB + g_posCfg.roboRaioCm), minimo, maximo);
    float soma = leituraA + leituraB;
    if (soma < 1.0f) soma = 1.0f;
    float residual = fabsf((leituraA + leituraB + (2.0f * g_posCfg.roboRaioCm)) - campo);

    if (residual <= g_posCfg.compToleranciaCm) {
      medida = 0.5f * (diretoA + diretoB);
      confianca = 0.95f;
    } else {
      float fracA = constrain(leituraA / soma, 0.0f, 1.0f);
      float fracB = constrain(leituraB / soma, 0.0f, 1.0f);
      float mapaA = minimo + (fracA * utilizavel);
      float mapaB = minimo + ((1.0f - fracB) * utilizavel);
      medida = 0.5f * (mapaA + mapaB);
      confianca = 0.56f;
    }
  } else if (temA) {
    medida = constrain(leituraA + g_posCfg.roboRaioCm, minimo, maximo);
    confianca = 0.62f;
  } else if (temB) { // Teste para o commit
    medida = constrain(campo - (leituraB + g_posCfg.roboRaioCm), minimo, maximo);
    confianca = 0.62f;
  }

  float salto = medida - ultimo;
  float medidaLimitada = ultimo + constrain(salto, -g_posCfg.saltoMaximoCm, g_posCfg.saltoMaximoCm);
  float filtrada = filtroComplementarPos(ultimo, medidaLimitada, confianca, dtSec);

  EixoPosResultado out;
  out.valor = constrain(filtrada, minimo, maximo);
  out.confianca = confianca;
  return out;
}

int calcularVelocidadePosicionamento(float distanciaCm) {
  float velMapeada = mapearFaixaClampedPos(distanciaCm,
                                           0.0f,
                                           g_posCfg.distanciaRampaCm,
                                           (float)g_posCfg.velocidadePwmMin,
                                           (float)g_posCfg.velocidadePwmMax);
  return constrain((int)roundf(velMapeada), g_posCfg.velocidadePwmMin, g_posCfg.velocidadePwmMax);
}

bool usarFaixaNormalMovimento(float anguloMov) {
  return (anguloMov >= 315.0f || anguloMov <= 45.0f ||
          (anguloMov >= 135.0f && anguloMov <= 225.0f));
}

bool hooksOrientacaoValidos() {
  return g_posHooks.temReferenciaOrientacao != nullptr &&
         g_posHooks.obterErroOrientacaoGraus != nullptr &&
         g_posHooks.calcularComandoGiro != nullptr;
}

bool preencherVetorParaAlvo(float alvoX,
                            float alvoY,
                            float& erroX,
                            float& erroY,
                            float& distanciaCm,
                            float& anguloGraus) {
  if (!g_posicaoAtualValida && !atualizarPosicaoAtual()) {
    return false;
  }

  const float atualX = obterPosicaoX();
  const float atualY = obterPosicaoY();
  erroX = alvoX - atualX;
  erroY = alvoY - atualY;
  distanciaCm = sqrtf((erroX * erroX) + (erroY * erroY));
  anguloGraus = normalizarAngulo360Pos(atan2f(erroX, -erroY) * 180.0f / PI);
  g_anguloAlvoGraus = anguloGraus;
  return true;
}
}

void inicializarMotoresPosicionamento(const MotoresPosicionamentoConfig& config) {
  g_posCfg = config;
  g_posHooks.temReferenciaOrientacao = config.hookTemReferenciaOrientacao;
  g_posHooks.obterErroOrientacaoGraus = config.hookObterErroOrientacaoGraus;
  g_posHooks.calcularComandoGiro = config.hookCalcularComandoGiro;
  g_posicaoXcm = 0.5f * g_posCfg.campoLarguraCm;
  g_posicaoYcm = 0.5f * g_posCfg.campoAlturaCm;
  g_confiancaPos = 0.0f;
  g_anguloAlvoGraus = 0.0f;
  g_ultimaEstimativaPosMs = 0;
  g_posicaoAtualValida = false;
}

void configurarHooksOrientacaoPosicionamento(const MotoresPosicionamentoOrientacaoHooks& hooks) {
  g_posHooks = hooks;
}

void atualizarLeiturasPosicionamento(float ultraEsquerdaCm,
                                     float ultraDireitaCm,
                                     float ultraFrenteCm,
                                     float ultraTrasCm,
                                     bool leiturasValidas) {
  g_ultraEcm = ultraEsquerdaCm;
  g_ultraDcm = ultraDireitaCm;
  g_ultraFcm = ultraFrenteCm;
  g_ultraTcm = ultraTrasCm;
  g_ultrasPosValidos = leiturasValidas;
  g_posicaoAtualValida = false;
}

bool atualizarPosicaoAtual() {
  if (!g_ultrasPosValidos) {
    g_posicaoAtualValida = false;
    return false;
  }

  unsigned long agora = millis();
  float dtSec = 0.08f;
  if (g_ultimaEstimativaPosMs > 0 && agora > g_ultimaEstimativaPosMs) {
    dtSec = (agora - g_ultimaEstimativaPosMs) / 1000.0f;
    if (dtSec < 0.05f) dtSec = 0.05f;
    if (dtSec > 0.8f) dtSec = 0.8f;
  }
  g_ultimaEstimativaPosMs = agora;

  EixoPosResultado eixoX = estimarEixoPosicao(g_ultraEcm, g_ultraDcm,
                                              g_posCfg.campoLarguraCm,
                                              g_posicaoXcm,
                                              dtSec);
  EixoPosResultado eixoY = estimarEixoPosicao(g_ultraFcm, g_ultraTcm,
                                              g_posCfg.campoAlturaCm,
                                              g_posicaoYcm,
                                              dtSec);

  g_posicaoXcm = eixoX.valor;
  g_posicaoYcm = eixoY.valor;
  g_confiancaPos = 0.5f * (eixoX.confianca + eixoY.confianca);
  g_posicaoAtualValida = (eixoX.confianca > 0.1f) && (eixoY.confianca > 0.1f);
  return g_posicaoAtualValida;
}

float obterPosicaoX() {
  return g_posicaoXcm;
}

float obterPosicaoY() {
  return g_posicaoYcm;
}

float obterConfiancaPosicao() {
  return g_confiancaPos;
}

float obterAnguloAlvoPosicionamentoGraus() {
  return g_anguloAlvoGraus;
}

bool moverParaSemGiro(float xCm, float yCm) {
  float erroX = 0.0f;
  float erroY = 0.0f;
  float distanciaCm = 0.0f;
  float anguloMov = 0.0f;
  if (!preencherVetorParaAlvo(xCm, yCm, erroX, erroY, distanciaCm, anguloMov)) {
    return false;
  }

  int velocidade = calcularVelocidadePosicionamento(distanciaCm);
  seguirDirecaoPorAngulo(anguloMov, velocidade);
  return true;
}

bool moverParaComGiro(float xCm, float yCm) {
  float erroX = 0.0f;
  float erroY = 0.0f;
  float distanciaCm = 0.0f;
  float anguloMov = 0.0f;
  if (!preencherVetorParaAlvo(xCm, yCm, erroX, erroY, distanciaCm, anguloMov)) {
    return false;
  }

  int velocidade = calcularVelocidadePosicionamento(distanciaCm);
  if (!hooksOrientacaoValidos() || !g_posHooks.temReferenciaOrientacao()) {
    seguirDirecaoPorAngulo(anguloMov, velocidade);
    return true;
  }

  float erroOrientacao = g_posHooks.obterErroOrientacaoGraus();
  int cmdGiro = g_posCfg.sinalGiro * g_posHooks.calcularComandoGiro(erroOrientacao);

  if (fabsf(erroOrientacao) > g_posCfg.giroSomenteEixoGraus) {
    int cmdEixo = constrain(cmdGiro, -g_posCfg.velocidadeGiroEixoPwm, g_posCfg.velocidadeGiroEixoPwm);
    girarNoEixo(cmdEixo);
    return true;
  }

  bool habilitarGiro = fabsf(erroOrientacao) > g_posCfg.erroMinimoAtivarGiroGraus;
  if (!habilitarGiro) {
    seguirDirecaoPorAngulo(anguloMov, velocidade);
    return true;
  }

  if (usarFaixaNormalMovimento(anguloMov)) {
    seguirDirecaoComGiro(anguloMov, velocidade, cmdGiro);
  } else {
    seguirDirecaoComGiroLaterais(anguloMov, velocidade, cmdGiro);
  }
  return true;
}