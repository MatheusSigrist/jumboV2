#include <Arduino.h>
#include "defensor.hpp"
#include "motores_movimentacao.hpp"

// =============================================================================
// ESTRATEGIA DO DEFENSOR (GOLEIRO)
// =============================================================================
//
// Responsabilidades:
//   • Manter posição na frente do gol usando linha como referência (zonas A e B)
//   • Acompanhar a bola lateralmente via IR / câmera
//   • Conter a bola com ultrassônicos laterais e de profundidade
//   • Retornar ao gol pela bússola quando sem linha
//   • Avançar sobre a bola quando ela fica frontal por tempo suficiente
//

void defensor() {

  int16_t anguloGolSelecionadoMenu = -999;
  uint16_t distanciaGolSelecionadoMenu = 0;
  bool golSelecionadoMenuVisivel = cameraLerGolSelecionadoMenu(anguloGolSelecionadoMenu, distanciaGolSelecionadoMenu);
  bool retornoDefensorPorBussolaValido = bussolaTemReferenciaValida();
  float anguloRetornoDefensor = retornoDefensorPorBussolaValido ? calcularAnguloRetornoGolPorBussola() : 0.0f;
  static unsigned long ultimoPrintGolSelecionadoMs = 0;

  if ((millis() - ultimoPrintGolSelecionadoMs) >= 200) {
    Serial.print("DEF GOL MENU: ");
    Serial.print(corGolAzul ? "AZUL" : "AMARELO");
    Serial.print(" ANG=");
    Serial.print(anguloGolSelecionadoMenu);
    Serial.print(" DIST=");
    Serial.print(distanciaGolSelecionadoMenu);
    Serial.print(" VIS=");
    Serial.println(golSelecionadoMenuVisivel ? 1 : 0);
    ultimoPrintGolSelecionadoMs = millis();
  }



  static float vetorXSuave = 0.0f;
  static float vetorYSuave = 0.0f;
  static float cmdGiroSuave = 0.0f;
  static bool retornoAoGolPorBussolaAtivo = false;
  static bool alinhamentoAntesRetornoBussolaPendente = false;
  static bool alinhamentoUnicoBussolaPendente = false;
  static unsigned long inicioAlinhamentoAntesRetornoBussolaMs = 0;
  static unsigned long inicioAlinhamentoUnicoBussolaMs = 0;

  bool temZonaAAtual = linhaZonaAValida && (anguloLinhaZonaA >= 0.0f);
  bool temZonaBAtual = linhaZonaBValida && (anguloLinhaZonaB >= 0.0f);
  float anguloZonaAUsado = temZonaAAtual ? anguloLinhaZonaA : -1.0f;
  float anguloZonaBUsado = temZonaBAtual ? anguloLinhaZonaB : -1.0f;

  unsigned long agora = millis();
  bool temZonaARetida = (!temZonaAAtual) && (ultimoAnguloLinhaZonaAValido >= 0.0f) &&
                        ((agora - ultimoRxLinhaZonaAMs) <= RETENCAO_ZONA_LINHA_DEFENSOR_MS);
  bool temZonaBRetida = (!temZonaBAtual) && (ultimoAnguloLinhaZonaBValido >= 0.0f) &&
                        ((agora - ultimoRxLinhaZonaBMs) <= RETENCAO_ZONA_LINHA_DEFENSOR_MS);

  if (temZonaARetida) {
    anguloZonaAUsado = ultimoAnguloLinhaZonaAValido;
  }
  if (temZonaBRetida) {
    anguloZonaBUsado = ultimoAnguloLinhaZonaBValido;
  }

  bool temZonaA = temZonaAAtual || temZonaARetida;
  bool temZonaB = temZonaBAtual || temZonaBRetida;
  bool linhaDefensorDisponivel = temZonaA || temZonaB;

  alinhandoAgora = false;
  fugindoLinhaAgora = false;

  float erroAngularLinha = 0.0f;
  int quantidadeErros = 0;

  if (temZonaA) {
    erroAngularLinha += normalizarErro180(anguloZonaAUsado - DEFENSOR_REFERENCIA_ZONA_A);
    quantidadeErros++;
  }

  if (temZonaB) {
    erroAngularLinha += normalizarErro180(anguloZonaBUsado - DEFENSOR_REFERENCIA_ZONA_B);
    quantidadeErros++;
  }

  if (quantidadeErros > 0) {
    erroAngularLinha /= (float)quantidadeErros;
  } else {
    resetPidLinhaGoleiro();
  }

  erroAngularLinha = aplicarDeadzoneDefensor(erroAngularLinha, 0.5f);
  erroAlinhamentoGraus = erroAngularLinha;

  int cmdPid = calcularSaidaPidLinhaGoleiro(erroAngularLinha);
  if (fabsf(erroAngularLinha) < DEFENSOR_TOLERANCIA_GIRO_GRAUS) {
    cmdPid = 0;
  }
  cmdPid = constrain(cmdPid, -180, 180);

  float cmdGiroAlvo = (float)(SINAL_GIRO_PID * cmdPid);
  if (fabsf(cmdGiroAlvo) < DEFENSOR_DEADZONE_GIRO) {
    cmdGiroAlvo = 0.0f;
  }
  cmdGiroSuave = suavizarDefensor(cmdGiroSuave, cmdGiroAlvo, DEFENSOR_SUAVIZACAO_GIRO);
  int cmdGiro = (int)roundf(cmdGiroSuave);
  alinhandoAgora = (fabsf(erroAngularLinha) >= DEFENSOR_TOLERANCIA_GIRO_GRAUS);

  float vetorX = 0.0f;
  float vetorY = 0.0f;
  const bool centroLinhaValido = temZonaA && temZonaB;
  const float pesoAtracaoLinha = 35.0f;
  const float fatorBolaComLinha = centroLinhaValido ? 0.9f : 1.78f;
  const float fatorUltra = centroLinhaValido ? 0.75f : 1.0f;
  const float bolaDireitaMin = 15.0f;
  const float bolaDireitaMax = 115.0f;
  const float bolaEsquerdaMin = 245.0f;
  const float bolaEsquerdaMax = 345.0f;
  const unsigned long TEMPO_MAX_ALINHAMENTO_UNICO_BUSSOLA_MS = 2000;
  bool ultrasRecentes = ultrasValidos && (ultimoRxUltraMs > 0) && ((agora - ultimoRxUltraMs) <= TIMEOUT_ULTRA_MS);

  if (executarAvancoFrontalTemporizadoDefensor(agora, vetorXSuave, vetorYSuave, cmdGiroSuave)) {
    return;
  }

  if (!linhaDefensorDisponivel && retornoDefensorPorBussolaValido) {
    if (!retornoAoGolPorBussolaAtivo) {
      retornoAoGolPorBussolaAtivo = true;
      alinhamentoAntesRetornoBussolaPendente = true;
      alinhamentoUnicoBussolaPendente = true;
      inicioAlinhamentoAntesRetornoBussolaMs = 0;
      inicioAlinhamentoUnicoBussolaMs = 0;
      erroAlinhamentoGraus = 0.0f;
      alinhandoAgora = false;
      resetPidLinhaGoleiro();
      resetPidBussola();
      vetorXSuave = 0.0f;
      vetorYSuave = 0.0f;
      cmdGiroSuave = 0.0f;
    }

    if (alinhamentoAntesRetornoBussolaPendente) {
      if (inicioAlinhamentoAntesRetornoBussolaMs == 0) {
        inicioAlinhamentoAntesRetornoBussolaMs = agora;
      }

      float erroBussolaRetorno = calcularErroReferenciaBussola();
      erroAlinhamentoGraus = erroBussolaRetorno;
      bool tempoAlinhamentoAtivo = (agora - inicioAlinhamentoAntesRetornoBussolaMs) < TEMPO_MAX_ALINHAMENTO_UNICO_BUSSOLA_MS;

      if (tempoAlinhamentoAtivo && (fabsf(erroBussolaRetorno) > TOLERANCIA_ALINHAMENTO_GRAUS)) {
        alinhandoAgora = true;
        resetPidLinhaGoleiro();
        vetorXSuave = 0.0f;
        vetorYSuave = 0.0f;
        cmdGiroSuave = 0.0f;

        int cmdPidBussola = calcularSaidaPidBussola(erroBussolaRetorno);
        int cmdGiroBussola = SINAL_GIRO_PID * cmdPidBussola;
        girarNoEixo(cmdGiroBussola);
        return;
      }

      alinhamentoAntesRetornoBussolaPendente = false;
      inicioAlinhamentoAntesRetornoBussolaMs = 0;
      resetPidBussola();
    }

    erroAlinhamentoGraus = 0.0f;
    alinhandoAgora = false;
    float anguloRetornoComUltra = comporAnguloRetornoBussolaComUltraLaterais(anguloRetornoDefensor, ultrasRecentes);
    seguirDirecaoPorAngulo(anguloRetornoComUltra,
                           (int)DEFENSOR_VELOCIDADE_RETORNO_GOL_PWM);
    return;
  }

  if (linhaDefensorDisponivel && alinhamentoUnicoBussolaPendente) {
    if (retornoDefensorPorBussolaValido) {
      if (inicioAlinhamentoUnicoBussolaMs == 0) {
        inicioAlinhamentoUnicoBussolaMs = agora;
      }

      float erroBussolaRetorno = calcularErroReferenciaBussola();
      erroAlinhamentoGraus = erroBussolaRetorno;
      bool tempoAlinhamentoAtivo = (agora - inicioAlinhamentoUnicoBussolaMs) < TEMPO_MAX_ALINHAMENTO_UNICO_BUSSOLA_MS;

      if (tempoAlinhamentoAtivo && (fabsf(erroBussolaRetorno) > TOLERANCIA_ALINHAMENTO_GRAUS)) {
        alinhandoAgora = true;
        resetPidLinhaGoleiro();
        vetorXSuave = 0.0f;
        vetorYSuave = 0.0f;
        cmdGiroSuave = 0.0f;

        int cmdPidBussola = calcularSaidaPidBussola(erroBussolaRetorno);
        int cmdGiroBussola = SINAL_GIRO_PID * cmdPidBussola;
        girarNoEixo(cmdGiroBussola);
        return;
      }
    }

    alinhamentoUnicoBussolaPendente = false;
    retornoAoGolPorBussolaAtivo = false;
    alinhamentoAntesRetornoBussolaPendente = false;
    inicioAlinhamentoAntesRetornoBussolaMs = 0;
    inicioAlinhamentoUnicoBussolaMs = 0;
    resetPidBussola();
  }

  if (!retornoAoGolPorBussolaAtivo) {
    alinhamentoAntesRetornoBussolaPendente = false;
    alinhamentoUnicoBussolaPendente = false;
    inicioAlinhamentoAntesRetornoBussolaMs = 0;
    inicioAlinhamentoUnicoBussolaMs = 0;
  }

  if (centroLinhaValido) {
    float anguloCentroLinha = calcularVetorAtracaoLinha(anguloZonaAUsado, anguloZonaBUsado);
    float anguloCentroLinhaRad = anguloCentroLinha * PI / 180.0f;

    // O centro entre A e B passa a ser a prioridade maxima de translacao do defensor.
    vetorX += sinf(anguloCentroLinhaRad) * pesoAtracaoLinha;
    vetorY += cosf(anguloCentroLinhaRad) * pesoAtracaoLinha;
  }

  // Bola: mantem a mesma logica de peso, mas usa a camera quando o IR nao estiver vendo.
  float anguloBola = -1.0f;
  bool bolaDisponivel = false;



  if (obterAnguloIrDisponivel(anguloBola)) {
    bolaDisponivel = true;
  }

  if (bolaDisponivel) {
    if (anguloBola >= bolaDireitaMin && anguloBola <= bolaDireitaMax) {
      vetorX += mapearFaixaClamped(anguloBola, bolaDireitaMin, bolaDireitaMax, DEFENSOR_PESO_MIN_BOLA, DEFENSOR_PESO_MAX_BOLA) * fatorBolaComLinha;
    }
    if (anguloBola >= bolaEsquerdaMin && anguloBola <= bolaEsquerdaMax) {
      vetorX -= mapearFaixaClamped(anguloBola, bolaEsquerdaMax, bolaEsquerdaMin, DEFENSOR_PESO_MIN_BOLA, DEFENSOR_PESO_MAX_BOLA) * fatorBolaComLinha;
    }
  }else{
        if ((ultraDcm >= 0.0f) && (ultraDcm < 80) && (ultraEcm > 40)) {
      vetorX -= mapearFaixaClamped(ultraDcm, DEFENSOR_ULTRA_LATERAL_ATIVO_CM, DEFENSOR_ULTRA_LATERAL_CRITICO_CM, 0.0f, DEFENSOR_PESO_MAX_ULTRA) * fatorUltra;
    }else{
              if ((ultraEcm >= 0.0f) && (ultraEcm < 80) && (ultraDcm > 40)) {
vetorX += mapearFaixaClamped(ultraEcm, DEFENSOR_ULTRA_LATERAL_ATIVO_CM, DEFENSOR_ULTRA_LATERAL_CRITICO_CM, 0.0f, DEFENSOR_PESO_MAX_ULTRA) * fatorUltra;
    }
    }
  }

  // Ultrassons: vetores de contencao, nunca mais como prioridade bloqueante.
  if (ultrasRecentes) {
    if ((ultraDcm >= 0.0f) && (ultraDcm < DEFENSOR_ULTRA_LATERAL_ATIVO_CM) && (ultraEcm > DEFENSOR_ULTRA_LATERAL_CONFIRMADOR_CM)) {
      vetorX -= mapearFaixaClamped(ultraDcm, DEFENSOR_ULTRA_LATERAL_ATIVO_CM, DEFENSOR_ULTRA_LATERAL_CRITICO_CM, 0.0f, DEFENSOR_PESO_MAX_ULTRA) * fatorUltra;
    }
    if ((ultraEcm >= 0.0f) && (ultraEcm < DEFENSOR_ULTRA_LATERAL_ATIVO_CM) && (ultraDcm > DEFENSOR_ULTRA_LATERAL_CONFIRMADOR_CM)) {
      vetorX += mapearFaixaClamped(ultraEcm, DEFENSOR_ULTRA_LATERAL_ATIVO_CM, DEFENSOR_ULTRA_LATERAL_CRITICO_CM, 0.0f, DEFENSOR_PESO_MAX_ULTRA) * fatorUltra;
    }
    if ((ultraTcm > DEFENSOR_ULTRA_FRENTE_LIMITE_CM) && (ultraFcm > DEFENSOR_ULTRA_FRONTAL_CONFIRMADOR_CM)) {
      vetorY -= mapearFaixaClamped(ultraTcm, DEFENSOR_ULTRA_FRENTE_LIMITE_CM, DEFENSOR_ULTRA_PROFUNDIDADE_MAX_CM, 0.0f, DEFENSOR_PESO_MAX_ULTRA_PROFUNDIDADE) * fatorUltra;
    }
    if ((ultraTcm >= 0.0f) && (ultraTcm < DEFENSOR_ULTRA_TRAS_LIMITE_CM) && (ultraFcm > DEFENSOR_ULTRA_FRONTAL_CONFIRMADOR_CM)) {
      vetorY += mapearFaixaClamped(ultraTcm, DEFENSOR_ULTRA_TRAS_LIMITE_CM, DEFENSOR_ULTRA_PROFUNDIDADE_MIN_CM, 0.0f, DEFENSOR_PESO_MAX_ULTRA_PROFUNDIDADE) * fatorUltra;
    }
  }

  vetorX = aplicarDeadzoneDefensor(vetorX, DEFENSOR_DEADZONE_VETOR);
  vetorY = aplicarDeadzoneDefensor(vetorY, DEFENSOR_DEADZONE_VETOR);

  // Suavizacao exponencial para reduzir jitter entre frames.
  vetorXSuave = suavizarDefensor(vetorXSuave, vetorX, DEFENSOR_SUAVIZACAO_VETOR);
  vetorYSuave = suavizarDefensor(vetorYSuave, vetorY, DEFENSOR_SUAVIZACAO_VETOR);
  vetorXSuave = aplicarDeadzoneDefensor(vetorXSuave, DEFENSOR_DEADZONE_VETOR * 0.5f);
  vetorYSuave = aplicarDeadzoneDefensor(vetorYSuave, DEFENSOR_DEADZONE_VETOR * 0.5f);

  // Resultado final: soma de linha, bola e ultras em um unico comando de translacao.
  float magnitudeVetor = calcularMagnitudeVetorDefensor(vetorXSuave, vetorYSuave);
  int velocidadeFinal = (int)roundf(constrain(magnitudeVetor, 0.0f, (float)velocidade_maxima));
  if (velocidadeFinal > 0 && velocidadeFinal < DEFENSOR_VELOCIDADE_MIN_PWM) {
    velocidadeFinal = (int)DEFENSOR_VELOCIDADE_MIN_PWM;
  }

  float anguloFinal = calcularAnguloVetorDefensor(vetorXSuave, vetorYSuave);
  seguirDirecaoComGiro(anguloFinal, velocidadeFinal, cmdGiro);
}


// =============================================================================
// FUNÇÕES COMPLEMENTARES DO DEFENSOR (GOLEIRO)
// =============================================================================
//

// Parâmetros de ajuste — *** ALTERE APENAS AQUI para tunar o defensor ***
// =============================================================================

// --- Referências de zona (ângulos esperados das linhas A e B no referencial do defensor) ---
constexpr float DEFENSOR_REFERENCIA_ZONA_A = 90.0f;
constexpr float DEFENSOR_REFERENCIA_ZONA_B = 270.0f;

// --- Tolerâncias angulares do defensor ---
constexpr float DEFENSOR_TOLERANCIA_GIRO_GRAUS     = 5.0f;   // Deadzone de giro
constexpr float DEFENSOR_TOLERANCIA_ALINHAMENTO_BOLA_GRAUS = 3.0f;  // Alinhamento antes do avanço frontal

// --- Pesos de atração lateral por bola ---
constexpr float DEFENSOR_PESO_MIN_BOLA = 75.0f;
constexpr float DEFENSOR_PESO_MAX_BOLA = 200.0f;

// --- Limites dos ultrassônicos do defensor ---
constexpr float DEFENSOR_ULTRA_LATERAL_ATIVO_CM       = 65.0f;   // Distância onde começa a repulsão lateral
constexpr float DEFENSOR_ULTRA_LATERAL_CRITICO_CM     = 45.0f;   // Distância de repulsão máxima
constexpr float DEFENSOR_ULTRA_LATERAL_CONFIRMADOR_CM = 60.0f;   // Confirma a parede oposta livre
constexpr float DEFENSOR_ULTRA_FRENTE_LIMITE_CM       = 40.0f;   // Limite para repulsão de profundidade (frente)
constexpr float DEFENSOR_ULTRA_TRAS_LIMITE_CM         = 35.0f;   // Limite para repulsão de profundidade (trás)
constexpr float DEFENSOR_ULTRA_FRONTAL_CONFIRMADOR_CM = 100.0f;  // Confirmador de campo frontal livre
constexpr float DEFENSOR_ULTRA_PROFUNDIDADE_MAX_CM    = 60.0f;
constexpr float DEFENSOR_ULTRA_PROFUNDIDADE_MIN_CM    = 10.0f;
constexpr float DEFENSOR_PESO_MAX_ULTRA               = 100.0f;
constexpr float DEFENSOR_PESO_MAX_ULTRA_PROFUNDIDADE  = 38.0f;

// --- Deadzones do defensor ---
constexpr float DEFENSOR_DEADZONE_VETOR = 6.0f;
constexpr float DEFENSOR_DEADZONE_GIRO  = 5.0f;

// --- Velocidades do defensor ---
constexpr float DEFENSOR_VELOCIDADE_MIN_PWM        = 200.0f;
constexpr float DEFENSOR_VELOCIDADE_RETORNO_GOL_PWM = 165.0f;

// --- Suavização do defensor (fator exponencial) ---
constexpr float DEFENSOR_SUAVIZACAO_VETOR = 0.45f;
constexpr float DEFENSOR_SUAVIZACAO_GIRO  = 0.35f;

// --- Parâmetros do avanço frontal temporizado do defensor ---
const float         DEFENSOR_TOLERANCIA_IR_FRONTAL_GRAUS      = 45.0f;
const int           DEFENSOR_VELOCIDADE_AVANCO_IR_FRONTAL_PWM = 220;
const unsigned long DEFENSOR_TEMPO_GATILHO_IR_FRONTAL_MS      = 3000;
const unsigned long DEFENSOR_TEMPO_AVANCO_IR_FRONTAL_MS       = 1500;

// --- Controle proporcional lateral do defensor por IR ---
int calcularVelocidadeLateralDefensorPorIr(float anguloBolaGraus) {
  const float ANG_DIREITA_MIN  = 20.0f;
  const float ANG_DIREITA_MAX  = 160.0f;  /// 
  const float ANG_ESQUERDA_MIN = 200.0f;  /// 
  const float ANG_ESQUERDA_MAX = 340.0f;
  const int   VEL_MIN = 200;
  const int   VEL_MAX = 255;

  float ang = normalizarAngulo360(anguloBolaGraus);
  if ((ang >= 0.0f && ang <= ANG_DIREITA_MIN) || (ang >= ANG_ESQUERDA_MAX && ang <= 360.0f)) return 0;
  if (ang > ANG_DIREITA_MIN && ang <= ANG_DIREITA_MAX) {
    float ganho = (float)(VEL_MAX - VEL_MIN) / (ANG_DIREITA_MAX - ANG_DIREITA_MIN);
    return constrain((int)(VEL_MIN + ganho * (ang - ANG_DIREITA_MIN)), VEL_MIN, VEL_MAX);
  }
  if (ang >= ANG_ESQUERDA_MIN && ang < ANG_ESQUERDA_MAX) {
    float ganho = (float)(VEL_MAX - VEL_MIN) / (ANG_ESQUERDA_MAX - ANG_ESQUERDA_MIN);
    return constrain((int)(VEL_MIN + ganho * (ANG_ESQUERDA_MAX - ang)), VEL_MIN, VEL_MAX);
  }
  return 0;
}

// --- PID do giro do goleiro usando zonas A e B da linha ---
// *** AJUSTE AQUI para tunar o giro do defensor ***
const float PID_LINHA_GOL_KP           = 0.9f;
const float PID_LINHA_GOL_KI           = 0.01f;
const float PID_LINHA_GOL_KD           = 0.55f;
const float PID_LINHA_GOL_INTEGRAL_MAX = 90.0f;
const int   PID_LINHA_GOL_SAIDA_MIN    = 60;
const int   PID_LINHA_GOL_SAIDA_MAX    = 220;






// Estado interno do PID de linha do goleiro
float        pidLinhaGolIntegral      = 0.0f;
float        pidLinhaGolErroAnterior  = 0.0f;
unsigned long pidLinhaGolUltimoMs     = 0;

// Zera o PID do goleiro por linha
void resetPidLinhaGoleiro() {
  pidLinhaGolIntegral     = 0.0f;
  pidLinhaGolErroAnterior = 0.0f;
  pidLinhaGolUltimoMs     = 0;
}

// Calcula saída do PID do goleiro para o giro por linha
int calcularSaidaPidLinhaGoleiro(float erroGraus) {
  unsigned long agora = millis();
  float dt = 0.02f;
  if (pidLinhaGolUltimoMs != 0) {
    dt = (agora - pidLinhaGolUltimoMs) / 1000.0f;
    if (dt < 0.005f) dt = 0.005f;
    if (dt > 0.2f)   dt = 0.2f;
  }
  pidLinhaGolUltimoMs = agora;

  pidLinhaGolIntegral += erroGraus * dt;
  if (pidLinhaGolIntegral >  PID_LINHA_GOL_INTEGRAL_MAX) pidLinhaGolIntegral =  PID_LINHA_GOL_INTEGRAL_MAX;
  if (pidLinhaGolIntegral < -PID_LINHA_GOL_INTEGRAL_MAX) pidLinhaGolIntegral = -PID_LINHA_GOL_INTEGRAL_MAX;

  float derivada = (erroGraus - pidLinhaGolErroAnterior) / dt;
  pidLinhaGolErroAnterior = erroGraus;

  float u    = PID_LINHA_GOL_KP * erroGraus + PID_LINHA_GOL_KI * pidLinhaGolIntegral + PID_LINHA_GOL_KD * derivada;
  int   saida = (int)fabsf(u);
  if (saida < PID_LINHA_GOL_SAIDA_MIN)           saida = PID_LINHA_GOL_SAIDA_MIN;
  if (saida > PID_LINHA_GOL_SAIDA_MAX)           saida = PID_LINHA_GOL_SAIDA_MAX;
  if (saida > VELOCIDADE_GIRO_ALINHAMENTO)       saida = VELOCIDADE_GIRO_ALINHAMENTO;
  return (u >= 0.0f) ? saida : -saida;
}

// --- Funções auxiliares do defensor ---

// Aplica suavização exponencial a um valor do defensor
float suavizarDefensor(float atual, float alvo, float fator) {
  float fatorClamped = constrain(fator, 0.0f, 1.0f);
  return atual + ((alvo - atual) * fatorClamped);
}

// Aplica deadzone a um valor do defensor
float aplicarDeadzoneDefensor(float valor, float deadzone) {
  return (fabsf(valor) < deadzone) ? 0.0f : valor;
}

// Calcula a magnitude do vetor de movimento do defensor
float calcularMagnitudeVetorDefensor(float vetorX, float vetorY) {
  return sqrtf((vetorX * vetorX) + (vetorY * vetorY));
}

// Calcula o ângulo do vetor de movimento do defensor (0 = frente, 90 = direita)
float calcularAnguloVetorDefensor(float vetorX, float vetorY) {
  return normalizarAngulo360(atan2f(vetorX, vetorY) * 180.0f / PI);
}

// Retorna o sinal do erro angular do defensor com deadzone
int sinalErroDefensor(float erro, float toleranciaZero) {
  if (erro >  toleranciaZero) return  1;
  if (erro < -toleranciaZero) return -1;
  return 0;
}

// Calcula comando de giro do defensor com base no desequilíbrio entre zonas A e B
int calcularGiroDefensor(float erroA, float erroB, float toleranciaIgual, int giroMaximo) {
  float deltaMag = fabsf(erroA) - fabsf(erroB);
  if (fabsf(deltaMag) <= toleranciaIgual) return 0;
  float ganho  = 2.0f;
  int   cmdGiro = (int)(fabsf(deltaMag) * ganho);
  if (cmdGiro < 35)          cmdGiro = 35;
  if (cmdGiro > giroMaximo)  cmdGiro = giroMaximo;
  return (deltaMag >= 0.0f) ? cmdGiro : -cmdGiro;
}

// Calcula velocidade proporcional de translação (frente/trás) pelo erro de linha
int calcularVelocidadeLinhaDefensor(float erroA, float erroB, bool temZonaA, bool temZonaB,
                                    float toleranciaZero, float erroMaxRef,
                                    int velocidadeMinima, int velocidadeMaxima) {
  float intensidade = 0.0f;
  if (temZonaA) { float magA = fabsf(erroA) - toleranciaZero; if (magA > intensidade) intensidade = magA; }
  if (temZonaB) { float magB = fabsf(erroB) - toleranciaZero; if (magB > intensidade) intensidade = magB; }
  if (intensidade <= 0.0f) return 0;
  if (intensidade > erroMaxRef) intensidade = erroMaxRef;
  float t = intensidade / erroMaxRef;
  return constrain((int)(velocidadeMinima + t * (float)(velocidadeMaxima - velocidadeMinima)), velocidadeMinima, velocidadeMaxima);
}

// Calcula vetor de atração média entre duas leituras de linha
float calcularVetorAtracaoLinha(float anguloA, float anguloB) {
  float aRad = anguloA * PI / 180.0f;
  float bRad = anguloB * PI / 180.0f;
  float mx   = cosf(aRad) + cosf(bRad);
  float my   = sinf(aRad) + sinf(bRad);
  return normalizarAngulo360(atan2f(my, mx) * 180.0f / PI);
}

// Compõe ângulo de retorno pela bússola com vetor de repulsão dos ultrassônicos laterais
float comporAnguloRetornoBussolaComUltraLaterais(float anguloRetornoBase, bool ultrasRecentes) {
  float anguloBaseRad = anguloRetornoBase * PI / 180.0f;
  float vetorX = sinf(anguloBaseRad) * DEFENSOR_VELOCIDADE_RETORNO_GOL_PWM;
  float vetorY = cosf(anguloBaseRad) * DEFENSOR_VELOCIDADE_RETORNO_GOL_PWM;

  if (ultrasRecentes) {
    if ((ultraDcm >= 0.0f) && (ultraDcm < DEFENSOR_ULTRA_LATERAL_ATIVO_CM) && (ultraEcm > DEFENSOR_ULTRA_LATERAL_CONFIRMADOR_CM))
      vetorX -= mapearFaixaClamped(ultraDcm, DEFENSOR_ULTRA_LATERAL_ATIVO_CM, DEFENSOR_ULTRA_LATERAL_CRITICO_CM, 0.0f, DEFENSOR_PESO_MAX_ULTRA);
    if ((ultraEcm >= 0.0f) && (ultraEcm < DEFENSOR_ULTRA_LATERAL_ATIVO_CM) && (ultraDcm > DEFENSOR_ULTRA_LATERAL_CONFIRMADOR_CM))
      vetorX += mapearFaixaClamped(ultraEcm, DEFENSOR_ULTRA_LATERAL_ATIVO_CM, DEFENSOR_ULTRA_LATERAL_CRITICO_CM, 0.0f, DEFENSOR_PESO_MAX_ULTRA);
  }
  return calcularAnguloVetorDefensor(vetorX, vetorY);
}

// Executa avanço frontal temporizado quando a bola fica na frente por tempo suficiente
bool executarAvancoFrontalTemporizadoDefensor(unsigned long agora,
                                              float &vetorXSuave,
                                              float &vetorYSuave,
                                              float &cmdGiroSuave) {
  static unsigned long inicioDeteccaoIrFrontalMs  = 0;
  static unsigned long inicioAvancoIrFrontalMs    = 0;
  static bool avancoIrFrontalAtivo                = false;
  static bool alinhamentoIrFrontalAtivo           = false;
  static bool aguardarSaidaJanelaIrFrontal        = false;
  static float ultimoAnguloAvancoIrFrontal        = 0.0f;

  bool irFrontalAtivo = irDetectado &&
                        (fabsf(normalizarErro180(anguloIr)) <= DEFENSOR_TOLERANCIA_IR_FRONTAL_GRAUS);

  // Executa avanço enquanto dentro do tempo
  if (avancoIrFrontalAtivo) {
    if ((agora - inicioAvancoIrFrontalMs) < DEFENSOR_TEMPO_AVANCO_IR_FRONTAL_MS) {
      if (irDetectado) ultimoAnguloAvancoIrFrontal = normalizarAngulo360(anguloIr);
      alinhandoAgora = false; erroAlinhamentoGraus = 0.0f;
      resetPidBussola(); resetPidLinhaGoleiro();
      vetorXSuave = vetorYSuave = cmdGiroSuave = 0.0f;
      seguirDirecaoPorAngulo(ultimoAnguloAvancoIrFrontal, DEFENSOR_VELOCIDADE_AVANCO_IR_FRONTAL_PWM);
      return true;
    }
    avancoIrFrontalAtivo = false; inicioAvancoIrFrontalMs = inicioDeteccaoIrFrontalMs = 0;
  }

  // Alinha ao ângulo da bola antes de avançar
  if (alinhamentoIrFrontalAtivo) {
    if (!irDetectado) {
      alinhamentoIrFrontalAtivo = false; inicioDeteccaoIrFrontalMs = 0;
      aguardarSaidaJanelaIrFrontal = false; resetPidBussola(); return false;
    }
    float erroAlinhamentoBola    = normalizarErro180(anguloIr);
    ultimoAnguloAvancoIrFrontal  = normalizarAngulo360(anguloIr);
    erroAlinhamentoGraus         = erroAlinhamentoBola;
    if (fabsf(erroAlinhamentoBola) > DEFENSOR_TOLERANCIA_ALINHAMENTO_BOLA_GRAUS) {
      alinhandoAgora = true; resetPidLinhaGoleiro();
      vetorXSuave = vetorYSuave = cmdGiroSuave = 0.0f;
      int cmdPidBola  = calcularSaidaPidBussola(erroAlinhamentoBola);
      int cmdGiroBola = -SINAL_GIRO_PID * cmdPidBola;
      girarNoEixo(-cmdGiroBola); return true;
    }
    alinhamentoIrFrontalAtivo = false; avancoIrFrontalAtivo = true;
    inicioAvancoIrFrontalMs   = agora;
    alinhandoAgora = false; erroAlinhamentoGraus = 0.0f;
    resetPidBussola(); resetPidLinhaGoleiro();
    vetorXSuave = vetorYSuave = cmdGiroSuave = 0.0f;
    seguirDirecaoPorAngulo(ultimoAnguloAvancoIrFrontal, DEFENSOR_VELOCIDADE_AVANCO_IR_FRONTAL_PWM);
    return true;
  }

  // Aguarda bola sair da janela frontal antes de nova detecção
  if (!irFrontalAtivo) { inicioDeteccaoIrFrontalMs = 0; aguardarSaidaJanelaIrFrontal = false; return false; }
  if (aguardarSaidaJanelaIrFrontal) return false;

  // Cronometra tempo com bola frontal
  if (inicioDeteccaoIrFrontalMs == 0) { inicioDeteccaoIrFrontalMs = agora; return false; }
  if ((agora - inicioDeteccaoIrFrontalMs) < DEFENSOR_TEMPO_GATILHO_IR_FRONTAL_MS) return false;

  // Gatilho atingido: inicia alinhamento antes do avanço
  alinhamentoIrFrontalAtivo    = true;
  aguardarSaidaJanelaIrFrontal = true;
  ultimoAnguloAvancoIrFrontal  = normalizarAngulo360(anguloIr);
  alinhandoAgora = false; erroAlinhamentoGraus = 0.0f;
  resetPidBussola(); resetPidLinhaGoleiro();
  vetorXSuave = vetorYSuave = cmdGiroSuave = 0.0f;
  return false;
}

// Placeholder para nova lógica de ultrassônico específica do defensor
bool ultrassonico_defensor() {
  return false;
}