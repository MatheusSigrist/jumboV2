#include <Arduino.h>
#include "defensor.hpp"
#include "motores_movimentacao.hpp"

// =============================================================================
// PARÂMETROS E CONSTANTES DO DEFENSOR
// =============================================================================
constexpr float DEFENSOR_REFERENCIA_ZONA_A = 90.0f;
constexpr float DEFENSOR_REFERENCIA_ZONA_B = 270.0f;
constexpr float DEFENSOR_TOLERANCIA_ALINHAMENTO_BOLA_GRAUS = 3.0f;
constexpr float DEFENSOR_DEADZONE_VETOR = 6.0f;
constexpr float DEFENSOR_VELOCIDADE_MIN_PWM = 200.0f;
constexpr float DEFENSOR_SUAVIZACAO_VETOR = 0.45f;

const float DEFENSOR_TOLERANCIA_IR_FRONTAL_GRAUS = 45.0f;
const int DEFENSOR_VELOCIDADE_AVANCO_IR_FRONTAL_PWM = 220;
const unsigned long DEFENSOR_TEMPO_GATILHO_IR_FRONTAL_MS = 3000;
const unsigned long DEFENSOR_TEMPO_AVANCO_IR_FRONTAL_MS = 1500;

constexpr float DEFENSOR_MAGNITUDE_MINIMA_PARAR      = 0.12f;
constexpr int   DEFENSOR_VELOCIDADE_MIN_CORRECAO_PWM = 70;
constexpr int   DEFENSOR_VELOCIDADE_MAX_CORRECAO_PWM = 220;

// =============================================================================
// GANHOS DO PID DA LINHA (MAGNITUDE)
// =============================================================================
constexpr float KP_DEFENSOR_LINHA = 200.0f;
constexpr float KI_DEFENSOR_LINHA = 0.1f;
constexpr float KD_DEFENSOR_LINHA = 20.0f;

// =============================================================================
// GANHOS DO PID DA BÚSSOLA
// =============================================================================
constexpr float KP_BUSSOLA_NOVINHA = 2.5f;
constexpr float KI_BUSSOLA_NOVINHA = 0.0f;
constexpr float KD_BUSSOLA_NOVINHA = 0.5f;

// =============================================================================
// ESTRUTURA E FUNÇÕES DO PID DE MAGNITUDE DA LINHA
// =============================================================================
static float erroAnteriorLinha = 0.0f;
static float integralLinha = 0.0f;
static unsigned long tempoAnteriorLinhaMs = 0;

void resetPidLinha() {
  integralLinha = 0.0f;
  erroAnteriorLinha = 0.0f;
  tempoAnteriorLinhaMs = 0;
}

float calcularSaidaPidLinha(float erroMagnitude, unsigned long agora) {
  float dt = (tempoAnteriorLinhaMs > 0) ? (agora - tempoAnteriorLinhaMs) / 1000.0f : 0.02f;
  if (dt <= 0.0f) dt = 0.02f;

  float pTerm = KP_DEFENSOR_LINHA * erroMagnitude;
  integralLinha += erroMagnitude * dt;
  float iTerm = KI_DEFENSOR_LINHA * integralLinha;
  float dTerm = KD_DEFENSOR_LINHA * ((erroMagnitude - erroAnteriorLinha) / dt);

  erroAnteriorLinha = erroMagnitude;
  tempoAnteriorLinhaMs = agora;

  return pTerm + iTerm + dTerm;
}

// =============================================================================
// ESTRUTURA E FUNÇÕES DO PID DA BÚSSOLA
// =============================================================================
static float erroAnteriorBussola = 0.0f;
static float integralBussola = 0.0f;
static unsigned long tempoAnteriorBussolaMs = 0;

void resetPidZimBussola() {
  integralBussola = 0.0f;
  erroAnteriorBussola = 0.0f;
  tempoAnteriorBussolaMs = 0;
}

float PIDZIMBUSSOLANOVINHA_DEFENSOR(float erroBussola) {
  unsigned long agora = millis();
  float dt = (tempoAnteriorBussolaMs > 0) ? (agora - tempoAnteriorBussolaMs) / 1000.0f : 0.02f;
  if (dt <= 0.0f) dt = 0.02f;

  float pTerm = KP_BUSSOLA_NOVINHA * erroBussola;
  integralBussola += erroBussola * dt;
  float iTerm = KI_BUSSOLA_NOVINHA * integralBussola;
  float dTerm = KD_BUSSOLA_NOVINHA * ((erroBussola - erroAnteriorBussola) / dt);

  erroAnteriorBussola = erroBussola;
  tempoAnteriorBussolaMs = agora;

  return pTerm + iTerm + dTerm;
}

// =============================================================================
// FUNÇÕES AUXILIARES E MATEMÁTICAS
// =============================================================================

float suavizarDefensor(float atual, float alvo, float fator) {
  float fatorClamped = constrain(fator, 0.0f, 1.0f);
  return atual + ((alvo - atual) * fatorClamped);
}

float aplicarDeadzoneDefensor(float valor, float deadzone) {
  return (fabsf(valor) < deadzone) ? 0.0f : valor;
}

float calcularMagnitudeVetorDefensor(float vetorX, float vetorY) {
  return sqrtf((vetorX * vetorX) + (vetorY * vetorY));
}

float calcularAnguloVetorDefensor(float vetorX, float vetorY) {
  return normalizarAngulo360(atan2f(vetorX, vetorY) * 180.0f / PI);
}

void calcularVetorPontoMedioLinha(float anguloA, float anguloB, float &anguloResultante, float &magnitudeResultante) {
  float radA = anguloA * PI / 180.0f;
  float radB = anguloB * PI / 180.0f;

  float xA = sinf(radA);
  float yA = cosf(radA);
  float xB = sinf(radB);
  float yB = cosf(radB);

  float xM = (xA + xB) / 2.0f;
  float yM = (yA + yB) / 2.0f;

  anguloResultante = calcularAnguloVetorDefensor(xM, yM);
  magnitudeResultante = calcularMagnitudeVetorDefensor(xM, yM);
}

bool executarAvancoFrontalTemporizadoDefensor(unsigned long agora,
                                              float &vetorXSuave,
                                              float &vetorYSuave,
                                              float &cmdGiroSuave) {
  static unsigned long inicioDeteccaoIrFrontalMs = 0;
  static unsigned long inicioAvancoIrFrontalMs = 0;
  static bool avancoIrFrontalAtivo = false;
  static bool alinhamentoIrFrontalAtivo = false;
  static bool aguardarSaidaJanelaIrFrontal = false;
  static float ultimoAnguloAvancoIrFrontal = 0.0f;

  bool irFrontalAtivo = irDetectado &&
                        (fabsf(normalizarErro180(anguloIr)) <= DEFENSOR_TOLERANCIA_IR_FRONTAL_GRAUS);

  if (avancoIrFrontalAtivo) {
    if ((agora - inicioAvancoIrFrontalMs) < DEFENSOR_TEMPO_AVANCO_IR_FRONTAL_MS) {
      if (irDetectado) ultimoAnguloAvancoIrFrontal = normalizarAngulo360(anguloIr);
      alinhandoAgora = false; erroAlinhamentoGraus = 0.0f;
      resetPidBussola();
      vetorXSuave = vetorYSuave = cmdGiroSuave = 0.0f;
      seguirDirecaoPorAngulo(ultimoAnguloAvancoIrFrontal, DEFENSOR_VELOCIDADE_AVANCO_IR_FRONTAL_PWM);
      return true;
    }
    avancoIrFrontalAtivo = false; inicioAvancoIrFrontalMs = inicioDeteccaoIrFrontalMs = 0;
  }

  if (alinhamentoIrFrontalAtivo) {
    if (!irDetectado) {
      alinhamentoIrFrontalAtivo = false; inicioDeteccaoIrFrontalMs = 0;
      aguardarSaidaJanelaIrFrontal = false; resetPidBussola(); return false;
    }
    float erroAlinhamentoBola = normalizarErro180(anguloIr);
    ultimoAnguloAvancoIrFrontal = normalizarAngulo360(anguloIr);
    erroAlinhamentoGraus = erroAlinhamentoBola;
    if (fabsf(erroAlinhamentoBola) > DEFENSOR_TOLERANCIA_ALINHAMENTO_BOLA_GRAUS) {
      alinhandoAgora = true;
      vetorXSuave = vetorYSuave = cmdGiroSuave = 0.0f;
      int cmdPidBola = calcularSaidaPidBussola(erroAlinhamentoBola);
      int cmdGiroBola = -SINAL_GIRO_PID * cmdPidBola;
      girarNoEixo(-cmdGiroBola); return true;
    }
    alinhamentoIrFrontalAtivo = false; avancoIrFrontalAtivo = true;
    inicioAvancoIrFrontalMs = agora;
    alinhandoAgora = false; erroAlinhamentoGraus = 0.0f;
    resetPidBussola();
    vetorXSuave = vetorYSuave = cmdGiroSuave = 0.0f;
    seguirDirecaoPorAngulo(ultimoAnguloAvancoIrFrontal, DEFENSOR_VELOCIDADE_AVANCO_IR_FRONTAL_PWM);
    return true;
  }

  if (!irFrontalAtivo) { inicioDeteccaoIrFrontalMs = 0; aguardarSaidaJanelaIrFrontal = false; return false; }
  if (aguardarSaidaJanelaIrFrontal) return false;

  if (inicioDeteccaoIrFrontalMs == 0) { inicioDeteccaoIrFrontalMs = agora; return false; }
  if ((agora - inicioDeteccaoIrFrontalMs) < DEFENSOR_TEMPO_GATILHO_IR_FRONTAL_MS) return false;

  alinhamentoIrFrontalAtivo = true;
  aguardarSaidaJanelaIrFrontal = true;
  ultimoAnguloAvancoIrFrontal = normalizarAngulo360(anguloIr);
  alinhandoAgora = false; erroAlinhamentoGraus = 0.0f;
  resetPidBussola();
  vetorXSuave = vetorYSuave = cmdGiroSuave = 0.0f;
  return false;
}

// =============================================================================
// ESTRATÉGIA DO DEFENSOR (GOLEIRO)
// =============================================================================

void defensor() {
  static float vetorXSuave = 0.0f;
  static float vetorYSuave = 0.0f;
  static float cmdGiroSuave = 0.0f;

  bool temZonaAAtual = linhaZonaAValida && (anguloLinhaZonaA >= 0.0f);
  bool temZonaBAtual = linhaZonaBValida && (anguloLinhaZonaB >= 0.0f);
  float anguloZonaAUsado = temZonaAAtual ? anguloLinhaZonaA : -1.0f;
  float anguloZonaBUsado = temZonaBAtual ? anguloLinhaZonaB : -1.0f;

  unsigned long agora = millis();
  bool temZonaARetida = (!temZonaAAtual) && (ultimoAnguloLinhaZonaAValido >= 0.0f) &&
                        ((agora - ultimoRxLinhaZonaAMs) <= RETENCAO_ZONA_LINHA_DEFENSOR_MS);
  bool temZonaBRetida = (!temZonaBAtual) && (ultimoAnguloLinhaZonaBValido >= 0.0f) &&
                        ((agora - ultimoRxLinhaZonaBMs) <= RETENCAO_ZONA_LINHA_DEFENSOR_MS);

  if (temZonaARetida) anguloZonaAUsado = ultimoAnguloLinhaZonaAValido;
  if (temZonaBRetida) anguloZonaBUsado = ultimoAnguloLinhaZonaBValido;

  bool temZonaA = temZonaAAtual || temZonaARetida;
  bool temZonaB = temZonaBAtual || temZonaBRetida;

  alinhandoAgora = false;
  fugindoLinhaAgora = false;
  erroAlinhamentoGraus = 0.0f;

  const bool centroLinhaValido = temZonaA && temZonaB;

  // 1. Avanço frontal temporizado ao detectar a bola
  if (executarAvancoFrontalTemporizadoDefensor(agora, vetorXSuave, vetorYSuave, cmdGiroSuave)) {
    return;
  }

  // 2. Cálculo contínuo do giro pela bússola
  float erroBussola = calcularErroReferenciaBussola();
  int cmdGiroBussola = constrain(
    (int)roundf(-PIDZIMBUSSOLANOVINHA_DEFENSOR(erroBussola)),
    -255,
    255
  );

  // 3. Estratégia do Ponto Médio com PID de Magnitude
  if (centroLinhaValido) {
    float anguloCorrecao = 0.0f;
    float magnitudeErro = 0.0f;

    calcularVetorPontoMedioLinha(anguloZonaAUsado, anguloZonaBUsado, anguloCorrecao, magnitudeErro);

    if (magnitudeErro <= DEFENSOR_MAGNITUDE_MINIMA_PARAR) {
      resetPidLinha();
      seguirDirecaoComGiroLaterais(0.0f, 0, cmdGiroBussola);
      return;
    }

    float saidaPidLinha = calcularSaidaPidLinha(magnitudeErro, agora);

    int velocidadeCalculada = (int)constrain(
      DEFENSOR_VELOCIDADE_MIN_CORRECAO_PWM + saidaPidLinha,
      DEFENSOR_VELOCIDADE_MIN_CORRECAO_PWM,
      DEFENSOR_VELOCIDADE_MAX_CORRECAO_PWM
    );

    seguirDirecaoComGiroLaterais(anguloCorrecao, velocidadeCalculada, cmdGiroBussola);
    return;
  } else {
    resetPidLinha();
    seguirDirecaoComGiroLaterais(0.0f, 0, cmdGiroBussola);
  }
}