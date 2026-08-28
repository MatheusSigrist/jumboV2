#include <Arduino.h>
#include "defensor.hpp"
#include "motores_movimentacao.hpp"

// =============================================================================
// PARÂMETROS DE TESTE DO DEFENSOR
// =============================================================================
constexpr int DEFENSOR_VELOCIDADE_TESTE_PWM = 200; // Velocidade fixa para o teste

// =============================================================================
// GANHOS DO PID DA BÚSSOLA
// =============================================================================
constexpr float KP_BUSSOLA_NOVINHA = 2.5f;
constexpr float KI_BUSSOLA_NOVINHA = 0.0f;
constexpr float KD_BUSSOLA_NOVINHA = 0.5f;

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
// ESTRATÉGIA DO DEFENSOR (TESTE DE MOVIMENTO LATERAL PURO)
// =============================================================================

void defensor() {
  // 1. Correção contínua da bússola para manter a frente travada (0°)
    float anguloBussolaAlvo =
        calcularErroReferenciaBussola();

    float erroBussola =
        normalizarErro180(-anguloBussolaAlvo);
  int cmdGiroBussola = constrain(
    (int)roundf(-PIDZIMBUSSOLANOVINHA_DEFENSOR(erroBussola)),
    -255,
    255
  );

  // 2. Leitura do IR da bola
  float anguloBola = -1.0f;
  bool bolaDisponivel = obterAnguloIrDisponivel(anguloBola);

  // 3. Deslocamento lateral puro com velocidade fixa (sem lógica de linha)
  if (bolaDisponivel) {
    // Bola à direita (15° a 115°): Força movimento fixo a 90°
    if (anguloBola >= 15.0f && anguloBola <= 115.0f) {
      seguirDirecaoComGiroLaterais(90.0f, DEFENSOR_VELOCIDADE_TESTE_PWM, cmdGiroBussola);
      return;
    } 
    // Bola à esquerda (245° a 345°): Força movimento fixo a 270°
    else if (anguloBola >= 245.0f && anguloBola <= 345.0f) {
      seguirDirecaoComGiroLaterais(270.0f, DEFENSOR_VELOCIDADE_TESTE_PWM, cmdGiroBussola);
      return;
    }
  }

  // 4. Sem bola nas faixas de teste: Parado, mantendo apenas o giro da bússola
  girarNoEixo(cmdGiroBussola);
}
