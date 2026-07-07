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

  float termoGiro = -g_cfg.ganhoGiroMisto * (float)cmdGiro;
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

  float termoGiro = -g_cfg.ganhoGiroMisto * (float)cmdGiro;
  v1 += termoGiro;
  v2 -= termoGiro;
  v3 -= termoGiro;
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

void moverFrenteComGiro(int velocidadePwm, int cmdGiro) {
  if (!g_inicializado) {
    return;
  }

  int pwmBase = constrain(velocidadePwm, 0, 255);

  float v1 = (float)pwmBase;
  float v2 = (float)pwmBase;
  float v3 = -(float)pwmBase;
  float v4 = -(float)pwmBase;

  float termoGiro = -g_cfg.ganhoGiroMisto * (float)cmdGiro;
  v1 += termoGiro;
  v2 += termoGiro;
  v3 += termoGiro;
  v4 += termoGiro;

  aplicarComandoMotoresComRampa((int)v1, (int)v2, (int)v3, (int)v4);
}

bool sairDaLinha(bool linhaDetectada, float anguloLinhaGraus, int velocidadePwm,
                 float* anguloComandoSaida) {
  if (!g_inicializado) {
    return false;
  }

  bool linhaAtiva = linhaDetectada && (anguloLinhaGraus >= 0.0f);
  if (linhaAtiva) {
    if (g_confirmacoesLinha < 3) {
      g_confirmacoesLinha++;
    }
  } else {
    g_confirmacoesLinha = 0;
  }

  // Requer confirmacao minima para reduzir falso positivo de frame unico.
  if (g_confirmacoesLinha < 2) {
    return false;
  }

  float anguloFuga = anguloLinhaGraus;
  while (anguloFuga >= 360.0f) anguloFuga -= 360.0f;
  while (anguloFuga < 0.0f) anguloFuga += 360.0f;
  if (anguloComandoSaida != nullptr) {
    *anguloComandoSaida = anguloFuga;
  }

  seguirDirecaoPorAngulo(anguloFuga, velocidadePwm);
  return true;
}

void resetSairDaLinha() {
  g_confirmacoesLinha = 0;
}
