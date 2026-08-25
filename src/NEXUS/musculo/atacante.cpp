#include <Arduino.h>
#include "atacante.hpp"
#include "motores_movimentacao.hpp"

// =============================================================================
// ESTRATEGIA DO ATACANTE
// =============================================================================
//
// Responsabilidades:
//   • Alinhar o robô ao gol via câmera (PID de bússola)
//   • Seguir a bola por IR com rampa angular suave
//   • Usar câmera como fallback quando IR está ausente
//   • Fugir da linha branca com prioridade máxima
//   • Frear ao se aproximar de paredes (ultrassônicos)
//   • Chutar ao se alinhar (kicker automático via atualizarKicker)
//

void atacante() {
    int velo = 255;
    int veloFrente = 220;

    float anguloIrAtual = -1.0f;

    float anguloBussolaAlvo =
        calcularErroReferenciaBussola();

    float erroBussola =
        normalizarErro180(-anguloBussolaAlvo);

    int cmdGiro =
        constrain(
            (int)roundf(
                -PIDZIMBUSSOLANOVINHA(erroBussola)
            ),
            -255,
            255
        );

    float anguloFuga = 0.0f;

    // -------------------------------------------------------------------------
    // FAILSAFE DA FUGA DE LINHA
    // -------------------------------------------------------------------------
    // Se a linha permanecer detectada continuamente por 2000 ms, consideramos
    // que o robô ficou preso na condição de fuga. Nesse caso, a fuga é
    // interrompida e permanece bloqueada até a linha deixar de ser detectada.
    //
    // As variáveis são static para manter o estado entre chamadas de atacante(),
    // mas continuam privadas a esta função (não precisam ir para o .hpp).
    static constexpr unsigned long TEMPO_MAX_FUGA_LINHA_MS = 2000UL;
    static unsigned long inicioFugaLinhaMs = 0;
    static bool fugaLinhaBloqueadaPorTimeout = false;

    const unsigned long agoraFugaLinhaMs = millis();

    // Saiu da linha: rearma completamente o failsafe para a próxima ocorrência.
    if (!linhaDetectada) {
        inicioFugaLinhaMs = 0;
        fugaLinhaBloqueadaPorTimeout = false;
    }

    // Enquanto a linha estiver sendo detectada e o failsafe não estiver
    // bloqueado, mede há quanto tempo a condição permanece ativa.
    if (linhaDetectada && !fugaLinhaBloqueadaPorTimeout) {

        if (inicioFugaLinhaMs == 0) {
            inicioFugaLinhaMs = agoraFugaLinhaMs;
        }

        // Ficou 2 segundos sem conseguir abandonar a linha: corta a fuga.
        if ((agoraFugaLinhaMs - inicioFugaLinhaMs) >= TEMPO_MAX_FUGA_LINHA_MS) {
            fugaLinhaBloqueadaPorTimeout = true;
            fugindoLinhaAgora = false;
            inicioFugaLinhaMs = 0;

            // IMPORTANTE: não damos return aqui.
            // A estratégia normal do atacante continua neste mesmo ciclo e
            // passa a comandar os motores, retirando o robô da fuga travada.
        }
    }

    // Preserva o comportamento original de sairDaLinha(): ela continua sendo
    // chamada normalmente enquanto o failsafe não estiver bloqueado, inclusive
    // caso a própria função possua algum estado interno de finalização da fuga.
    if (!fugaLinhaBloqueadaPorTimeout &&
        sairDaLinha(
            linhaDetectada,
            anguloLinhaPe,
            VELOCIDADE_FUGA_LINHA,
            &anguloFuga)) {

        fugindoLinhaAgora = true;
        anguloFugaLinhaCmd = anguloFuga;
        return;
    }

    // Se houve timeout, garante que nenhum outro trecho interprete o robô como
    // ainda estando em fuga enquanto a linha continuar detectada.
    if (fugaLinhaBloqueadaPorTimeout) {
        fugindoLinhaAgora = false;
    }

    // -------------------------------------------------------------------------
    // ESTRATÉGIA NORMAL DO ATACANTE
    // -------------------------------------------------------------------------
    if (obterAnguloIrDisponivel(anguloIrAtual)) {

            if (irNaFaixaFrontal(anguloIrAtual)) {

                moverFrenteComGiroParaGol(veloFrente);
                

            } else {
            /*     
                if (sairDaLinha(
                    linhaDetectada,
                       anguloLinhaPe,
                       VELOCIDADE_FUGA_LINHA,
                           &anguloFuga)) {

                        fugindoLinhaAgora = true;
                       anguloFugaLinhaCmd = anguloFuga;
                         return;

                          }*/
                resetControleGolCamera();
                        
          float anguloMovimento = mapearAnguloBolaParaMovimento(anguloIrAtual);

            seguirDirecaoComGiroLaterais(
                anguloMovimento,
                velo,
                cmdGiro
            );
            }

    } else {

        girarNoEixo(cmdGiro);
        return;
    }

    return;
}

// =============================================================================
// FUNÇÕES COMPLEMENTARES DO ATACANTE
// =============================================================================


// Parâmetros de ajuste — *** ALTERE APENAS AQUI para tunar o atacante ***
// =============================================================================

// --- Velocidades do atacante ---
const int VELOCIDADE_IR_FRONTAL_PWM       = 200;   // PWM na faixa frontal do IR (±32°)
const int VELOCIDADE_IR_FAIXA_REDUZIDA_PWM = 140;  // PWM em faixas laterais do IR

// --- Freio ultrassônico do atacante (laterais) ---
const float ATACANTE_ULTRA_FREIO_INICIO_CM    = 70.0f;   // Distância onde o freio começa
const float ATACANTE_ULTRA_FREIO_CRITICO_CM   = 50.0f;   // Distância de freio máximo
const int   ATACANTE_ULTRA_FREIO_VELOCIDADE_MIN = 80;   // Velocidade mínima com freio
const int   ATACANTE_ULTRA_FREIO_PWM_POR_CM   = 3;       // Incremento de PWM por cm

// --- Confirmação de linha + parede (evita falso positivo único) ---
const uint8_t ATACANTE_LINHA_PAREDE_CONFIRMACAO = 3;

// --- Tempo mínimo sem bola na câmera para iniciar busca ---
const unsigned long ATACANTE_ESPERA_SEM_BOLA_CAMERA_MS = 2000UL;

// --- PID de suavização angular do atacante (transição entre ângulos de movimento) ---
// *** AJUSTE AQUI para suavizar ou tornar mais responsiva a transição de direção ***
const float PID_MOVIMENTO_KP           = 2.0f;
const float PID_MOVIMENTO_KI           = 0.01f;
const float PID_MOVIMENTO_KD           = 0.8f;
const float PID_MOVIMENTO_INTEGRAL_MAX = 90.0f;
const float PID_MOVIMENTO_SAIDA_MAX    = 15.0f;
const float ALPHA_MOVIMENTO            = 0.15f;   // Suavização exponencial do ângulo atual
const float ALPHA_MOVIMENTO_ALVO       = 0.11f;   // Suavização exponencial do ângulo alvo
const float PASSO_MAX_MOVIMENTO_ALVO_GRAUS = 18.0f; // Passo máximo por ciclo no alvo filtrado

// Estado interno do PID de movimento do atacante
float        pidMovimentoIntegral              = 0.0f;
float        pidMovimento                      = 0.0f;
float        erroMovimento                     = 0.0f;
float        erroAnteriorMovimento             = 0.0f;
float        anguloMovimentoAtual              = -1.0f;
float        anguloMovimentoSuavizado          = -1.0f;
float        anguloMovimentoDesejadoFiltrado   = -1.0f;
unsigned long ultimoTempoPidMovimento          = 0;
unsigned long inicioCameraSemIrMs              = 0;

// Zera o controlador angular do atacante
void resetControleMovimentoAtacante() {
  pidMovimentoIntegral            = 0.0f;
  pidMovimento                    = 0.0f;
  erroMovimento                   = 0.0f;
  erroAnteriorMovimento           = 0.0f;
  anguloMovimentoAtual            = -1.0f;
  anguloMovimentoSuavizado        = -1.0f;
  anguloMovimentoDesejadoFiltrado = -1.0f;
  ultimoTempoPidMovimento         = 0;
}



// PID de transição angular do atacante (suaviza ângulo de movimento)
float calcularPidMovimento(float erro) {
  unsigned long agora = millis();
  float dt = 0.02f;
  if (ultimoTempoPidMovimento != 0) {
    dt = (agora - ultimoTempoPidMovimento) / 1000.0f;
    if (dt < 0.005f) dt = 0.005f;
    if (dt > 0.2f)   dt = 0.2f;
  }
  ultimoTempoPidMovimento = agora;

  pidMovimentoIntegral += erro * dt;
  if (pidMovimentoIntegral >  PID_MOVIMENTO_INTEGRAL_MAX) pidMovimentoIntegral =  PID_MOVIMENTO_INTEGRAL_MAX;
  if (pidMovimentoIntegral < -PID_MOVIMENTO_INTEGRAL_MAX) pidMovimentoIntegral = -PID_MOVIMENTO_INTEGRAL_MAX;

  float derivada = (erro - erroAnteriorMovimento) / dt;
  erroAnteriorMovimento = erro;

  pidMovimento = PID_MOVIMENTO_KP * erro + PID_MOVIMENTO_KI * pidMovimentoIntegral + PID_MOVIMENTO_KD * derivada;
  if (pidMovimento >  PID_MOVIMENTO_SAIDA_MAX) pidMovimento =  PID_MOVIMENTO_SAIDA_MAX;
  if (pidMovimento < -PID_MOVIMENTO_SAIDA_MAX) pidMovimento = -PID_MOVIMENTO_SAIDA_MAX;

  // Evita microcorreções perto do alvo
  if (fabsf(erro) < 2.0f) pidMovimento = 0.0f;

  return -pidMovimento;
}

// Aplica suavização exponencial circular ao ângulo de movimento do atacante
float suavizarAnguloMovimentoAtacante(float anguloMovimentoDesejado) {
  // Referencial deslocado 180°: o que era 0 passa a ser 180 e vice-versa
  float alvoBruto = normalizarAngulo360(anguloMovimentoDesejado + 180.0f);

  if ((anguloMovimentoAtual < 0.0f) || (anguloMovimentoSuavizado < 0.0f)) {
    anguloMovimentoAtual            = alvoBruto;
    anguloMovimentoSuavizado        = alvoBruto;
    anguloMovimentoDesejadoFiltrado = alvoBruto;
    erroMovimento = erroAnteriorMovimento = pidMovimentoIntegral = pidMovimento = 0.0f;
    ultimoTempoPidMovimento = 0;
    return anguloMovimentoSuavizado;
  }

  // Estabiliza o alvo em modo circular para evitar saltos (ex.: 90° para 270°)
  float deltaAlvo = normalizarErro180(alvoBruto - anguloMovimentoDesejadoFiltrado);
  deltaAlvo = constrain(deltaAlvo, -PASSO_MAX_MOVIMENTO_ALVO_GRAUS, PASSO_MAX_MOVIMENTO_ALVO_GRAUS);
  anguloMovimentoDesejadoFiltrado = normalizarAngulo360(anguloMovimentoDesejadoFiltrado + deltaAlvo);
  anguloMovimentoDesejadoFiltrado = normalizarAngulo360(
      anguloMovimentoDesejadoFiltrado +
      ALPHA_MOVIMENTO_ALVO * normalizarErro180(alvoBruto - anguloMovimentoDesejadoFiltrado));

  // Erro: última direção realmente comandada como realimentação
  erroMovimento = normalizarErro180(anguloMovimentoDesejadoFiltrado - anguloMovimentoAtual);
  pidMovimento  = calcularPidMovimento(erroMovimento);

  anguloMovimentoAtual = normalizarAngulo360(anguloMovimentoAtual + pidMovimento);
  anguloMovimentoSuavizado = anguloMovimentoSuavizado +
                             ALPHA_MOVIMENTO * normalizarErro180(anguloMovimentoAtual - anguloMovimentoSuavizado);
  anguloMovimentoSuavizado = normalizarAngulo360(anguloMovimentoSuavizado);
  return anguloMovimentoSuavizado;
}

/*

//////////// NÃO VAMOS USAR///////////////////

// Faz rampa angular curta (100–300 ms) entre faixas do IR
float obterAnguloIrSuavizado(float anguloAlvoGraus) {
  float alvo  = normalizarAngulo360(anguloAlvoGraus);
  unsigned long agora = millis();

  if (!anguloIrSuaveInicializado) {
    anguloIrSuaveAtual = anguloIrSuaveInicio = anguloIrSuaveAlvo = alvo;
    inicioTransicaoIrMs = agora; duracaoTransicaoIrMs = TRANSICAO_ANGULO_IR_MIN_MS;
    anguloIrSuaveInicializado = true;
    return quantizarAnguloPasso(anguloIrSuaveAtual, PASSO_ANGULO_IR_GRAUS);
  }

  float erroNovoAlvo = fabsf(normalizarErro180(alvo - anguloIrSuaveAlvo));
  if (erroNovoAlvo >= 1.0f) {
    anguloIrSuaveInicio = anguloIrSuaveAtual;
    anguloIrSuaveAlvo   = alvo;
    inicioTransicaoIrMs = agora;
    float delta             = fabsf(normalizarErro180(anguloIrSuaveAlvo - anguloIrSuaveInicio));
    unsigned long duracaoCalculada = (unsigned long)(delta * TRANSICAO_ANGULO_IR_MS_POR_GRAU);
    if (duracaoCalculada < TRANSICAO_ANGULO_IR_MIN_MS) duracaoCalculada = TRANSICAO_ANGULO_IR_MIN_MS;
    if (duracaoCalculada > TRANSICAO_ANGULO_IR_MAX_MS) duracaoCalculada = TRANSICAO_ANGULO_IR_MAX_MS;
    duracaoTransicaoIrMs = duracaoCalculada;
  }

  unsigned long decorridoMs = agora - inicioTransicaoIrMs;
  if (decorridoMs >= duracaoTransicaoIrMs) {
    anguloIrSuaveAtual = anguloIrSuaveAlvo;
  } else {
    float progresso = (float)decorridoMs / (float)duracaoTransicaoIrMs;
    float delta     = normalizarErro180(anguloIrSuaveAlvo - anguloIrSuaveInicio);
    anguloIrSuaveAtual = normalizarAngulo360(anguloIrSuaveInicio + delta * progresso);
  }

  return quantizarAnguloPasso(anguloIrSuaveAtual, PASSO_ANGULO_IR_GRAUS);
}
  */

// Detecta faixa frontal do IR em torno de 0° (±32°), tratando wrap 360°->0°
bool irNaFaixaFrontal(float anguloBolaGraus) {
  float ang = normalizarAngulo360(anguloBolaGraus);
  return (ang > 328.0f || ang < 32.0f);
}

// Retém por poucos milissegundos o último ângulo IR válido para evitar parada em falhas curtas.
bool obterAnguloIrDisponivel(float &anguloBolaGraus) {
  if (irDetectado && anguloIr >= 0.0f) {
    anguloBolaGraus = normalizarAngulo360(anguloIr);
    return true;
  }

  if (ultimoAnguloIrValido >= 0.0f && (millis() - ultimoRxIrValidoMs) <= RETENCAO_IR_VALIDO_MS) {
    anguloBolaGraus = normalizarAngulo360(ultimoAnguloIrValido);
    return true;
  }

  return false;
}

// Retorna true se algum ultrassônico lateral do atacante está em nível crítico
bool ultraLateralCriticoAtacante() {
  bool ultraDireitoCritico  = (ultraDcm >= 0.0f) && (ultraDcm <= ATACANTE_ULTRA_FREIO_CRITICO_CM);
  bool ultraEsquerdoCritico = (ultraEcm >= 0.0f) && (ultraEcm <= ATACANTE_ULTRA_FREIO_CRITICO_CM);
  return ultraDireitoCritico || ultraEsquerdoCritico;
}

// Limita velocidade por freio ultrassônico frontal (frente do robô)
int aplicarFreioUltrassonicoAtacanteFrente(int velocidadeDesejada) {
  int velocidadeBase = constrain(velocidadeDesejada, 0, 255);
  bool ultrasRecentes = (ultimoRxUltraMs > 0) && ((millis() - ultimoRxUltraMs) <= TIMEOUT_ULTRA_MS);
  if (!ultrasRecentes) return velocidadeBase;

  float menorUltraCm = -1.0f;
  // Usa apenas ultraF para o freio frontal; ultraT é ignorado se estiver próximo
  float leituras[] = { ultraFcm };
  for (float leitura : leituras) {
    if (leitura < 0.0f) continue;
    if (ultraTcm > 150) {
      if ((menorUltraCm < 0.0f) || (leitura < menorUltraCm)) menorUltraCm = leitura;
    }
    if ((menorUltraCm < 0.0f) || (menorUltraCm > ATACANTE_ULTRA_FREIO_INICIO_CM)) return velocidadeBase;
    int velocidadeLimite = ATACANTE_ULTRA_FREIO_VELOCIDADE_MIN;
    if (menorUltraCm > ATACANTE_ULTRA_FREIO_CRITICO_CM)
      velocidadeLimite += (int)((menorUltraCm - ATACANTE_ULTRA_FREIO_CRITICO_CM) * ATACANTE_ULTRA_FREIO_PWM_POR_CM);
    if (velocidadeLimite > velocidade_maxima) velocidadeLimite = velocidade_maxima;
    return min(velocidadeBase, velocidadeLimite);
  }
  return velocidadeBase;
}

// Limita velocidade por freio ultrassônico lateral do atacante (D e E)
int aplicarFreioUltrassonicoAtacante(int velocidadeDesejada) {
  int velocidadeBase = constrain(velocidadeDesejada, 0, 255);
  bool ultrasRecentes = (ultimoRxUltraMs > 0) && ((millis() - ultimoRxUltraMs) <= TIMEOUT_ULTRA_MS);
  if (!ultrasRecentes) return velocidadeBase;

  float menorUltraCm = -1.0f;
  float leituras[] = { ultraDcm, ultraEcm };
  for (float leitura : leituras) {
    if (leitura < 0.0f) continue;
    if ((menorUltraCm < 0.0f) || (leitura < menorUltraCm)) menorUltraCm = leitura;
  }
  if ((menorUltraCm < 0.0f) || (menorUltraCm > ATACANTE_ULTRA_FREIO_INICIO_CM)) return velocidadeBase;

  int velocidadeLimite = ATACANTE_ULTRA_FREIO_VELOCIDADE_MIN;
  if (menorUltraCm > ATACANTE_ULTRA_FREIO_CRITICO_CM)
    velocidadeLimite += (int)((menorUltraCm - ATACANTE_ULTRA_FREIO_CRITICO_CM) * ATACANTE_ULTRA_FREIO_PWM_POR_CM);
  if (velocidadeLimite > velocidade_maxima) velocidadeLimite = velocidade_maxima;
  return min(velocidadeBase, velocidadeLimite);
}

// Ajusta o ângulo da bola para o ângulo de comando de movimento (mapeamento por faixas)
// *** ALTERE AQUI para modificar o comportamento de contorno da bola pelo atacante ***
float mapearAnguloBolaParaMovimento(float anguloBolaGraus)
{
  float ang = normalizarAngulo360(anguloBolaGraus);
  


  if (ang >= 32.0f  && ang <= 60.0f)  return 100.0f;
  if (ang >  60.0f  && ang <  90.0f)  return 90.0f;
  if (ang >= 90.0f  && ang < 135.0f)  return 180.0f;
  if (ang >= 135.0f && ang < 180.0f)  return 225.0f;
  if (ang >= 180.0f && ang < 225.0f)  return 135.0f;
  if (ang >= 225.0f && ang < 270.0f)  return 180.0f;
  if (ang >= 270.0f && ang < 300.0f)  return 270.0f;
  if (ang >= 300.0f && ang <= 328.0f) return 260.0f;

  return ang;
}

// Reduz velocidade em faixas próximas do frontal para melhorar controle lateral
int calcularVelocidadeIrPorAngulo(float anguloBolaGraus) {
  float ang = normalizarAngulo360(anguloBolaGraus);
  if ((ang >= 33.0f && ang <= 60.0f) || (ang >= 300.0f && ang <= 328.0f)) return VELOCIDADE_IR_FAIXA_REDUZIDA_PWM;
  if (ang >= 140.0f && ang < 220.0f) return VELOCIDADE_IR_FAIXA_REDUZIDA_PWM;
  return velocidade_maxima;
}

// Calcula ângulo de busca quando a câmera perdeu a bola (usa ultrassônicos laterais)
float calcularAnguloBuscaSemBolaCameraAtacante() {
  bool ultrasRecentes = ultrasValidos && (ultimoRxUltraMs > 0) && ((millis() - ultimoRxUltraMs) <= TIMEOUT_ULTRA_MS);
  if (ultrasRecentes) {
    bool esquerdaPerto = (ultraEcm >= 0.0f) && (ultraEcm < 60.0f);
    bool direitaPerto  = (ultraDcm >= 0.0f) && (ultraDcm < 60.0f);
    bool esquerdaLivre = ultraEcm > 50.0f;
    bool direitaLivre  = ultraDcm > 50.0f;
    if (esquerdaPerto && direitaLivre)  return  90.0f;
    if (direitaPerto  && esquerdaLivre) return 270.0f;
  }
  return 0.0f;
}






float suavizadorMegaAnguloMovimento(float anguloNovo)
{
    static float anguloFiltrado = 0.0f;
    const float ALFA = 0.18f; // quanto menor, mais suave

    // normalização básica de salto de ângulo (evita pulo 359->0)
    float diff = anguloNovo - anguloFiltrado;

    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    anguloFiltrado += ALFA * diff;

    // normaliza 0–360
    if (anguloFiltrado < 0) anguloFiltrado += 360.0f;
    if (anguloFiltrado >= 360.0f) anguloFiltrado -= 360.0f;

    return anguloFiltrado;
}


float PIDZIMBUSSOLANOVINHA(float erro)
{
  static float erroAnterior = 0.0f;
  static float integral = 0.0f;
  static unsigned long ultimoMs = 0;

  const float Kp = 1.2f;
  const float Ki = 0.01f;
  const float Kd = 0.8f;

  unsigned long agora = millis();
  float dt = 0.02f;
  if (ultimoMs != 0) {
    dt = (agora - ultimoMs) / 1000.0f;
    if (dt < 0.005f) dt = 0.005f;
    if (dt > 0.2f) dt = 0.2f;
  }
  ultimoMs = agora;

  integral += erro * dt;

  // Anti-windup
  integral = constrain(integral, -100.0f, 100.0f);

  float derivada = (erro - erroAnterior) / dt;

  float saida =
    (Kp * erro) +
    (Ki * integral) +
    (Kd * derivada);

  erroAnterior = erro;

  // Corrige o sentido da sua bússola
  return -saida;
}



// =============================================================================
// CONTROLE DE GIRO PARA O GOL DURANTE ATAQUE FRONTAL
// =============================================================================
// Quando a bola está na faixa frontal do IR, o robô continua avançando para
// frente, mas o comando de rotação passa a ser calculado pelo ângulo do gol
// fornecido pela câmera.
//
// Referência:
//   cameraGolSelecionadoAngle = 0°  -> gol alinhado com a frente do robô
//   valor positivo              -> gol de um lado
//   valor negativo              -> gol do outro lado
//
// A bússola continua sendo usada normalmente fora do ataque frontal.
// =============================================================================

const float PID_GOL_CAMERA_KP = 1.2f;
const float PID_GOL_CAMERA_KI = 0.0f;
const float PID_GOL_CAMERA_KD = 0.5f;

const float PID_GOL_CAMERA_INTEGRAL_MAX = 100.0f;

const int PID_GOL_CAMERA_SAIDA_MIN = 30;
const int PID_GOL_CAMERA_SAIDA_MAX = 180;

// Dentro desta faixa o robô considera o gol alinhado.
const float TOLERANCIA_GOL_CAMERA_GRAUS = 2.0f;

// Mantém a última leitura válida por este período se a câmera perder
// o gol momentaneamente.
const unsigned long RETENCAO_GOL_CAMERA_ATAQUE_MS = 100;

// Limita saltos muito grandes entre duas leituras da câmera.
const float SALTO_MAX_GOL_CAMERA_GRAUS = 20.0f;

// Estado do PID específico da câmera.
// NÃO compartilha integral/derivada com o PID da bússola.
float pidGolCameraIntegral = 0.0f;
float pidGolCameraErroAnterior = 0.0f;
unsigned long pidGolCameraUltimoMs = 0;

// Estado do filtro do ângulo do gol.
float cameraGolAnguloFiltrado = 0.0f;
bool cameraGolFiltroInicializado = false;
unsigned long cameraGolUltimaLeituraValidaMs = 0;


// Zera completamente o controlador de giro para o gol.
void resetPidGolCamera()
{
  pidGolCameraIntegral = 0.0f;
  pidGolCameraErroAnterior = 0.0f;
  pidGolCameraUltimoMs = 0;
}


// Zera filtro e PID do gol.
void resetControleGolCamera()
{
  cameraGolAnguloFiltrado = 0.0f;
  cameraGolFiltroInicializado = false;
  cameraGolUltimaLeituraValidaMs = 0;

  resetPidGolCamera();
}


// Obtém o ângulo do gol selecionado e aplica filtro circular.
// Se a câmera perder o gol por poucos milissegundos, mantém a última
// leitura válida durante RETENCAO_GOL_CAMERA_ATAQUE_MS.
bool obterAnguloGolCameraAtaque(float &anguloGol)
{
  const unsigned long agora = millis();

  int16_t leituraGol = -999;

  if (cameraTemGolSelecionadoValido(leituraGol))
  {
    float leitura = normalizarErro180((float)leituraGol);

    if (!cameraGolFiltroInicializado)
    {
      cameraGolAnguloFiltrado = leitura;
      cameraGolFiltroInicializado = true;
    }
    else
    {
      float delta = normalizarErro180(
        leitura - cameraGolAnguloFiltrado
      );

      // Rejeita mudanças instantâneas muito grandes.
      delta = constrain(
        delta,
        -SALTO_MAX_GOL_CAMERA_GRAUS,
         SALTO_MAX_GOL_CAMERA_GRAUS
      );

      // Filtro exponencial circular.
      const float ALPHA_GOL_CAMERA = 0.5f;

      cameraGolAnguloFiltrado = normalizarErro180(
        cameraGolAnguloFiltrado +
        (ALPHA_GOL_CAMERA * delta)
      );
    }

    cameraGolUltimaLeituraValidaMs = agora;
    anguloGol = cameraGolAnguloFiltrado;

    return true;
  }

  // Câmera perdeu o gol momentaneamente:
  // mantém a última leitura por um curto período.
  if (cameraGolFiltroInicializado &&
      cameraGolUltimaLeituraValidaMs > 0 &&
      (agora - cameraGolUltimaLeituraValidaMs) <= RETENCAO_GOL_CAMERA_ATAQUE_MS)
  {
    anguloGol = cameraGolAnguloFiltrado;
    return true;
  }

  return false;
}


// Calcula o comando de rotação usando SOMENTE o erro angular do gol.
// O objetivo é fazer:
//       anguloGol = 0°
//
// Portanto:
//       erro = -anguloGol
//
// SINAL_GIRO_PID é utilizado para manter a mesma convenção de sentido
// de rotação já utilizada no restante do robô.
int calcularCmdGiroGolCamera(float anguloGol)
{
  const unsigned long agora = millis();

  float dt = 0.02f;

  if (pidGolCameraUltimoMs != 0)
  {
    dt = (agora - pidGolCameraUltimoMs) / 1000.0f;

    if (dt < 0.005f) dt = 0.005f;
    if (dt > 0.2f)   dt = 0.2f;
  }

  pidGolCameraUltimoMs = agora;

  // Queremos que o ângulo do gol chegue a 0°.
  float erroGol = normalizarErro180(-anguloGol);

  // Zona morta para evitar oscilação quando já estiver alinhado.
  if (fabsf(erroGol) <= TOLERANCIA_GOL_CAMERA_GRAUS)
  {
    pidGolCameraIntegral = 0.0f;
    pidGolCameraErroAnterior = erroGol;

    return 0;
  }

  // Integral.
  pidGolCameraIntegral += erroGol * dt;

  pidGolCameraIntegral = constrain(
    pidGolCameraIntegral,
    -PID_GOL_CAMERA_INTEGRAL_MAX,
     PID_GOL_CAMERA_INTEGRAL_MAX
  );

  // Derivada.
  float derivada =
    (erroGol - pidGolCameraErroAnterior) / dt;

  pidGolCameraErroAnterior = erroGol;

  // PID.
  float saida =
      PID_GOL_CAMERA_KP * erroGol
    + PID_GOL_CAMERA_KI * pidGolCameraIntegral
    + PID_GOL_CAMERA_KD * derivada;

  // Magnitude.
  int magnitude = (int)fabsf(saida);

  magnitude = constrain(
    magnitude,
    PID_GOL_CAMERA_SAIDA_MIN,
    PID_GOL_CAMERA_SAIDA_MAX
  );

  // Mantém o mesmo limite geral utilizado no alinhamento.
  magnitude = min(magnitude, VELOCIDADE_GIRO_ALINHAMENTO);

  // Convenção de sentido do robô.
  int cmdGiro = (saida >= 0.0f)
              ? magnitude
              : -magnitude;

  cmdGiro *= SINAL_GIRO_PID;

  return constrain(cmdGiro, -255, 255);
}


// Avança para frente enquanto gira o chassi para alinhar com o gol.
// Se a câmera perder o gol definitivamente, volta temporariamente para
// a referência da bússola, evitando deixar o robô sem controle de rotação.
void moverFrenteComGiroParaGol(int velocidade)
{
  float anguloGol = 0.0f;

  if (obterAnguloGolCameraAtaque(anguloGol))
  {
    int cmdGiroGol =
      calcularCmdGiroGolCamera(anguloGol);

    // TRANSLADA PARA FRENTE + ROTACIONA PARA O GOL.
    moverFrenteComGiro(
      velocidade,
      cmdGiroGol
    );

    return;
  }

  // Sem gol válido na câmera: fallback seguro para a bússola.
  resetPidGolCamera();

  float erroBussola =
    normalizarErro180(
      -calcularErroReferenciaBussola()
    );

  int cmdGiroBussola =
    constrain(
      (int)roundf(
        -PIDZIMBUSSOLANOVINHA(erroBussola)
      ),
      -255,
      255
    );

  moverFrenteComGiro(
    velocidade,
    cmdGiroBussola
  );
}