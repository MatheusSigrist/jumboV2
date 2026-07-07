#pragma once

#include <Arduino.h>

/*
 Biblioteca: motores_movimentacao

 Guia rapido de uso (API de movimentacao):
 - inicializarMotoresMovimentacao(config):
   Inicializa pinos da ponte H, canais PWM e parametros globais da locomocao.
   Parametros principais em config:
   - velocidadeMaxima: limite global de velocidade das rodas (0..255).
   - passoRampaPwm: passo maximo por ciclo na rampa de aceleracao/frenagem.
   - ganhoGiroMisto: ganho de mistura entre translacao e giro (cmdGiro).

 - pararMotores():
   Zera PWM e coloca as pontes em estado neutro.

 - girarNoEixo(velocidade):
   Gira o robo no proprio eixo.
   Parametros:
   - velocidade: comando assinado de giro. Sinal define o sentido.

 - seguirDirecaoPorAngulo(anguloGraus, velocidade):
   Realiza translacao no angulo desejado (base mecanum/omni).
   Parametros:
   - anguloGraus: direcao de movimento no referencial local (0..360).
   - velocidade: modulo da translacao (0..velocidadeMaxima).

 - seguirDirecaoComGiro(anguloGraus, velocidade, cmdGiro):
   Translacao com correcao de orientacao durante o deslocamento.
   Parametros:
   - anguloGraus: direcao de translacao (0..360).
   - velocidade: modulo da translacao (0..velocidadeMaxima).
   - cmdGiro: comando de giro assinado (tipicamente saida de PID).

 - seguirDirecaoComGiroLaterais(anguloGraus, velocidade, cmdGiro):
   Variante para deslocamentos laterais, com distribuicao de giro otimizada.
   Parametros:
   - anguloGraus: direcao de translacao (0..360).
   - velocidade: modulo da translacao (0..velocidadeMaxima).
   - cmdGiro: comando de giro assinado (tipicamente saida de PID).

 - moverFrenteComGiro(velocidadePwm, cmdGiro):
   Comando direto de avancar com compensacao de giro.
   Parametros:
   - velocidadePwm: PWM base de avancar (0..255).
   - cmdGiro: comando de giro assinado para correcao angular.

//==========================================================================//
    float anguloFuga = 0.0f;
    if (sairDaLinha(linhaDetectada, anguloLinhaPe,
            aplicarFreioUltrassonicoAtacante(VELOCIDADE_FUGA_LINHA),
            &anguloFuga)) {
    fugindoLinhaAgora = true;
    anguloFugaLinhaCmd = anguloFuga;
    return;
    }
//==========================================================================//
*/

struct MotoresMovimentacaoConfig {
  uint8_t in1_1_a = 5;
  uint8_t in2_1_a = 6;
  uint8_t pwm_1_a = 4;

  uint8_t in1_2_a = 3;
  uint8_t in2_2_a = 46;
  uint8_t pwm_2_a = 7;

  uint8_t in1_1_b = 11;
  uint8_t in2_1_b = 12;
  uint8_t pwm_1_b = 10;

  uint8_t in1_2_b = 13;
  uint8_t in2_2_b = 14;
  uint8_t pwm_2_b = 47;

  uint8_t pwm_ch1 = 0;
  uint8_t pwm_ch2 = 1;
  uint8_t pwm_ch3 = 2;
  uint8_t pwm_ch4 = 3;

  uint32_t pwm_freq = 20000;
  uint8_t pwm_res = 8;

  int velocidadeMaxima = 255;
  int passoRampaPwm = 16;
  float ganhoGiroMisto = 0.7f;
};

void inicializarMotoresMovimentacao(const MotoresMovimentacaoConfig& config);
void pararMotores();
void girarNoEixo(int velocidade);
void seguirDirecaoPorAngulo(float anguloGraus, int velocidade);
void seguirDirecaoComGiro(float anguloGraus, int velocidade, int cmdGiro);
void seguirDirecaoComGiroLaterais(float anguloGraus, int velocidade, int cmdGiro);
void moverFrenteComGiro(int velocidadePwm, int cmdGiro);

// Detecta linha e executa fuga imediatamente quando confirmada.
// Retorna true quando o comando de fuga foi aplicado.
bool sairDaLinha(bool linhaDetectada, float anguloLinhaGraus, int velocidadePwm,
                 float* anguloComandoSaida = nullptr);

// Reseta estado interno de confirmacao da rotina sairDaLinha.
void resetSairDaLinha();
