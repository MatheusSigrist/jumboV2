#include <Arduino.h>
#include "atacante.hpp"
#include "motores_movimentacao.hpp"

// O ESP32-S3 DevKitC-1 usa o NeoPixel embutido no GPIO 48. O framework
// normalmente fornece RGB_BUILTIN; o fallback mantém o código explícito.
#ifndef RGB_BUILTIN
#define RGB_BUILTIN 48
#endif

void controlarMovimentoAtacante(
    float anguloReferencia,
    int velocidadeReferencia,
    int giroReferencia,
    float &anguloControlado,
    int &velocidadeControlada,
    int &giroControlado
);

void atualizarLedLinhaAtacante(bool linhaAtiva) {
    static int8_t ultimoEstadoLinha = -1;
    const int8_t estadoAtualLinha = linhaAtiva ? 1 : 0;

    if (estadoAtualLinha == ultimoEstadoLinha) {
        return;
    }

    if (linhaAtiva) {
        neopixelWrite(RGB_BUILTIN, 25, 25, 25);  // Branco durante a linha.
    } else {
        neopixelWrite(RGB_BUILTIN, 0, 25, 0);    // Verde fora da linha.
    }

    ultimoEstadoLinha = estadoAtualLinha;
}

// =============================================================================
// REPOSICIONAMENTO POR ZONA (A/B/C) RECEBIDA DO DEFENSOR VIA ESP-NOW
// =============================================================================
//
// A referencia bussola (headingReferenciaBussola) e sempre calibrada apontando
// para o gol adversario, e o controlador de giro do posicionamento mantem o
// robo alinhado a ela. Por isso, no referencial X/Y do campo (ultrassonicos
// F/T = eixo Y, E/D = eixo X), Y pequeno = lado de ataque (gol adversario) e
// Y grande = lado de tras/defesa — sempre, independente de qual lado fisico
// da sala esta sendo defendido no jogo atual.
//
// Zona A (direita, tras) e Zona B (esquerda, tras) ficam a 1/4 do campo de
// distancia da parede de tras, e a 1/4 do campo de distancia da lateral.
// =============================================================================

bool obterAlvoReposicionamentoPorZona(float &xCm, float &yCm) {
    const bool zonaRecente =
        (ultimoRxZonaDefensorMs > 0) &&
        ((millis() - ultimoRxZonaDefensorMs) <= TIMEOUT_ZONA_DEFENSOR_MS);

    if (!zonaRecente) {
        return false;
    }

    // Zona C = meio do campo (sem lado preferencial informado pelo defensor).
    if (zonaDefensorRecebida == 'C') {
        xCm = 89.0f;
        yCm = 120.0f;
        return true;
    }

    // Zona A = canto esquerdo inferior
    else if (zonaDefensorRecebida == 'A') {
        xCm = 40.0f;
        yCm = 181.0f;
        return true;
    }

    // Zona B = canto direito inferior
    else if (zonaDefensorRecebida == 'B') {
      xCm = 140.0f;
      yCm = 181.0f;
      return true;
    }

    yCm = CAMPO_ALTURA_CM - (CAMPO_ALTURA_CM * 0.25f);
    xCm = (zonaDefensorRecebida == 'A')
              ? (CAMPO_LARGURA_CM - (CAMPO_LARGURA_CM * 0.25f))
              : (CAMPO_LARGURA_CM * 0.25f);
    return true;
}

// =============================================================================
// ESTRATEGIA DO ATACANTE
// =============================================================================
//
// Responsabilidades:
//   • Alinhar o robô ao gol via câmera (PID de bússola)
//   • Seguir a bola por IR com controle progressivo de movimento
//   • Usar câmera como fallback quando IR está ausente
//   • Fugir da linha branca com prioridade máxima
//   • Frear ao se aproximar de paredes (ultrassônicos)
//   • Chutar ao se alinhar (kicker automático via atualizarKicker)
//

void atacante() {
    atualizarLedLinhaAtacante(linhaDetectada);

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

    // =========================================================================
    // FAILSAFE FINAL DA LINHA
    // =========================================================================
    static constexpr unsigned long TEMPO_TIMEOUT_FINAL_LINHA_MS = 1500UL;
    static unsigned long inicioTimeoutFinalLinhaMs = 0UL;
    static bool fugaLinhaBloqueadaPorTimeout = false;

    const unsigned long agoraLinhaMs = millis();

    // =========================================================================
    // TEMPO CONTÍNUO INDO DE FRENTE PARA O GOL
    // =========================================================================
    // Quanto mais tempo o robô fica correndo de frente para o gol, maior a
    // inércia acumulada — por isso o retorno ao pegar a linha precisa ser
    // proporcionalmente mais longo (só nesse movimento, não nas laterais).
    static constexpr unsigned long LIMIAR_FRENTE_GOL_PARA_EXTRA_MS = 500UL;
    static constexpr unsigned long DIVISOR_FRENTE_GOL_PARA_EXTRA   = 2UL;
    static unsigned long inicioMovimentoFrenteGolMs = 0UL;
    static unsigned long tempoContinuoFrenteGolMs = 0UL;
    static bool indoDeFrenteParaGolAgora = false;

    static float anguloFugaTravadoExtra = 0.0f;
    static unsigned long duracaoExtraRetornoMs = 0UL;
    static unsigned long inicioExtraRetornoMs = 0UL;

    // Tempo seguido sem enxergar a bola (IR nem câmera) antes de reposicionar por zona.
    static constexpr unsigned long TEMPO_SEM_BOLA_PARA_REPOSICIONAR_MS = 1500UL;
    static unsigned long inicioSemBolaMs = 0UL;

    if (!linhaDetectada) {
        inicioTimeoutFinalLinhaMs = 0UL;
        fugaLinhaBloqueadaPorTimeout = false;
    }

    if (linhaDetectada && !fugaLinhaBloqueadaPorTimeout) {

        if (inicioTimeoutFinalLinhaMs == 0UL) {
            inicioTimeoutFinalLinhaMs = agoraLinhaMs;
        }

        if ((agoraLinhaMs - inicioTimeoutFinalLinhaMs) >=
            TEMPO_TIMEOUT_FINAL_LINHA_MS) {

            fugaLinhaBloqueadaPorTimeout = true;
            fugindoLinhaAgora = false;
            anguloFugaLinhaCmd = 0.0f;

            girarNoEixo(0);
            return;
        }
    }

    // -------------------------------------------------------------------------
    // FUGA NORMAL — sairDaLinha() CONTINUA SENDO A PRINCIPAL
    // -------------------------------------------------------------------------
    if (!fugaLinhaBloqueadaPorTimeout) {

        // Linha acabou de ser pega: define o extra de retorno com base em
        // quanto tempo o robô já vinha correndo de frente para o gol.
        if (linhaDetectada && duracaoExtraRetornoMs == 0UL && inicioExtraRetornoMs == 0UL) {
            if (tempoContinuoFrenteGolMs > LIMIAR_FRENTE_GOL_PARA_EXTRA_MS) {
                duracaoExtraRetornoMs =
                    (tempoContinuoFrenteGolMs - LIMIAR_FRENTE_GOL_PARA_EXTRA_MS) /
                    DIVISOR_FRENTE_GOL_PARA_EXTRA;
            }
        }

        if (sairDaLinha(
                linhaDetectada,
                anguloLinhaPe,
                VELOCIDADE_FUGA_LINHA,
                &anguloFuga)) {

            fugindoLinhaAgora = true;
            anguloFugaLinhaCmd = anguloFuga;
            anguloFugaTravadoExtra = anguloFuga;
//            inicioExtraRetornoMs = 0UL;

            // sairDaLinha() já comandou a fuga normal neste ciclo.
            return;
        }

        // sairDaLinha() encerrou a fuga normal: aplica o extra de retorno
        // acumulado (apenas quando a corrida de frente para o gol foi longa).
        if (duracaoExtraRetornoMs > 0UL) {

            if (inicioExtraRetornoMs == 0UL) {
                inicioExtraRetornoMs = agoraLinhaMs;
            }

            if ((agoraLinhaMs - inicioExtraRetornoMs) < duracaoExtraRetornoMs) {
                fugindoLinhaAgora = true;
                anguloFugaLinhaCmd = anguloFugaTravadoExtra;

                seguirDirecaoPorAngulo(
                    anguloFugaTravadoExtra,
                    VELOCIDADE_FUGA_LINHA
                );

                return;
            }

            duracaoExtraRetornoMs = 0UL;
            inicioExtraRetornoMs = 0UL;
        }
    }
        
    else {
        fugindoLinhaAgora = false;
        anguloFugaLinhaCmd = 0.0f;
        duracaoExtraRetornoMs = 0UL;
        inicioExtraRetornoMs = 0UL;
    }

    // -------------------------------------------------------------------------
    // ESTRATÉGIA NORMAL DO ATACANTE
    // -------------------------------------------------------------------------
    if (obterAnguloIrDisponivel(anguloIrAtual)) {

        inicioSemBolaMs = 0UL;

        if (irNaFaixaFrontal(anguloIrAtual)) {

            if (!indoDeFrenteParaGolAgora) {
                indoDeFrenteParaGolAgora = true;
                inicioMovimentoFrenteGolMs = agoraLinhaMs;
            }
            tempoContinuoFrenteGolMs = agoraLinhaMs - inicioMovimentoFrenteGolMs;

            moverFrenteComGiroParaGol(veloFrente);

        } else {

            indoDeFrenteParaGolAgora = false;
            tempoContinuoFrenteGolMs = 0UL;

            resetControleGolCamera();

            float anguloMovimento =
                mapearAnguloBolaParaMovimento(anguloIrAtual);

            // ================================================================
            // NOVO CONTROLE PROGRESSIVO DOS 3 COMANDOS
            // ================================================================
            //
            // anguloMovimento = referência do ângulo
            // velo            = referência da velocidade
            // cmdGiro         = referência do giro
            //
            // O controlador compara cada referência com o último valor
            // realmente comandado e aproxima a saída gradualmente.
            //
            float anguloMovimentoControlado = 0.0f;
            int velocidadeControlada = 0;
            int cmdGiroControlado = 0;

            controlarMovimentoAtacante(
                anguloMovimento,
                velo,
                cmdGiro,
                anguloMovimentoControlado,
                velocidadeControlada,
                cmdGiroControlado
            );

            seguirDirecaoComGiro(
                anguloMovimentoControlado,
                velocidadeControlada,
                cmdGiroControlado
            );
        }

    } 
    
    // Se IR não estiver disponível (SEM BOLA!!)

    else {
        indoDeFrenteParaGolAgora = false;
        tempoContinuoFrenteGolMs = 0UL;

        // So reposiciona pela zona apos 3s seguidos sem enxergar a bola.
        if (inicioSemBolaMs == 0UL) {
            inicioSemBolaMs = agoraLinhaMs;
        }

        const bool semBolaHaTempoSuficiente =
            (agoraLinhaMs - inicioSemBolaMs) >= TEMPO_SEM_BOLA_PARA_REPOSICIONAR_MS;

        float alvoX = 0.0f;
        float alvoY = 0.0f;
        if (semBolaHaTempoSuficiente &&
            obterAlvoReposicionamentoPorZona(alvoX, alvoY))
           {
            // moverParaComGiro() depende da posicao estimada por PosicaoCampo,
            // que so e atualizada se alimentarmos as leituras a cada ciclo aqui.
            atualizarLeiturasPosicionamento(ultraEcm, ultraDcm, ultraFcm, ultraTcm, ultrasValidos);
            if (moverParaComGiro(alvoX, alvoY)) {
                return;
            }
        }

      else {
        girarNoEixo(cmdGiro);
        return;
    }
  }
    return;
}

// =============================================================================
// FUNÇÕES COMPLEMENTARES DO ATACANTE
// =============================================================================

// Parâmetros de ajuste — *** ALTERE APENAS AQUI para tunar o atacante ***
// =============================================================================

// --- Velocidades do atacante ---
const int VELOCIDADE_IR_FRONTAL_PWM         = 200;
const int VELOCIDADE_IR_FAIXA_REDUZIDA_PWM = 140;

// --- Freio ultrassônico do atacante (laterais) ---
const float ATACANTE_ULTRA_FREIO_INICIO_CM       = 70.0f;
const float ATACANTE_ULTRA_FREIO_CRITICO_CM      = 50.0f;
const int   ATACANTE_ULTRA_FREIO_VELOCIDADE_MIN  = 80;
const int   ATACANTE_ULTRA_FREIO_PWM_POR_CM      = 3;

// --- Confirmação de linha + parede (evita falso positivo único) ---
const uint8_t ATACANTE_LINHA_PAREDE_CONFIRMACAO = 3;

// --- Tempo mínimo sem bola na câmera para iniciar busca ---
const unsigned long ATACANTE_ESPERA_SEM_BOLA_CAMERA_MS = 2000UL;


// =============================================================================
// NOVO CONTROLADOR DOS 3 FATORES DE MOVIMENTO
// =============================================================================
//
// O controlador trabalha com:
//
//   REFERÊNCIA:
//      - anguloMovimento
//      - velocidade
//      - cmdGiro
//
//   ÚLTIMO VALOR REALMENTE COMANDADO:
//      - ultimoAnguloMovimentoControle
//      - ultimaVelocidadeMovimentoControle
//      - ultimoCmdGiroMovimentoControle
//
// A saída é:
//
//      novoValor = ultimoValor + correçãoPID
//
// Exemplo:
//
//      último ângulo = 90°
//      referência    = 180°
//
//      erro = 180 - 90
//           = 90°
//
//      PID = +40°
//
//      saída = 90 + 40
//            = 130°
//
// No próximo ciclo:
//
//      último ângulo = 130°
//      referência    = 180°
//
//      erro = 180 - 130
//           = 50°
//
// E assim sucessivamente.
//
// =============================================================================


// -----------------------------------------------------------------------------
// GANHOS DO CONTROLADOR DE ÂNGULO
// -----------------------------------------------------------------------------
//
// O Kp é o principal responsável pela aproximação.
//
// Exemplo com Kp = 0.45:
//
//      erro = 90°
//      correção = 90 × 0.45
//               = 40.5°
//
//      90 + 40.5 = 130.5°
//
// Portanto, esse valor produz aproximadamente o comportamento desejado
// no exemplo discutido.
// -----------------------------------------------------------------------------

const float CONTROLE_MOVIMENTO_ANGULO_KP = 0.75f;
const float CONTROLE_MOVIMENTO_ANGULO_KI = 0.0f;
const float CONTROLE_MOVIMENTO_ANGULO_KD = 0.0f;

const float CONTROLE_MOVIMENTO_ANGULO_INTEGRAL_MAX = 180.0f;
const float CONTROLE_MOVIMENTO_ANGULO_CORRECAO_MAX = 45.0f;


// -----------------------------------------------------------------------------
// GANHOS DO CONTROLADOR DE VELOCIDADE
// -----------------------------------------------------------------------------

const float CONTROLE_MOVIMENTO_VELOCIDADE_KP = 0.35f;
const float CONTROLE_MOVIMENTO_VELOCIDADE_KI = 0.0f;
const float CONTROLE_MOVIMENTO_VELOCIDADE_KD = 0.0f;

const float CONTROLE_MOVIMENTO_VELOCIDADE_INTEGRAL_MAX = 255.0f;
const float CONTROLE_MOVIMENTO_VELOCIDADE_CORRECAO_MAX = 60.0f;


// -----------------------------------------------------------------------------
// GANHOS DO CONTROLADOR DE GIRO
// -----------------------------------------------------------------------------

const float CONTROLE_MOVIMENTO_GIRO_KP = 0.45f;
const float CONTROLE_MOVIMENTO_GIRO_KI = 0.0f;
const float CONTROLE_MOVIMENTO_GIRO_KD = 0.0f;

const float CONTROLE_MOVIMENTO_GIRO_INTEGRAL_MAX = 255.0f;
const float CONTROLE_MOVIMENTO_GIRO_CORRECAO_MAX = 60.0f;


// -----------------------------------------------------------------------------
// ÚLTIMOS VALORES REALMENTE COMANDADOS
// -----------------------------------------------------------------------------

float ultimoAnguloMovimentoControle = 0.0f;
float ultimaVelocidadeMovimentoControle = 0.0f;
float ultimoCmdGiroMovimentoControle = 0.0f;


// -----------------------------------------------------------------------------
// ESTADOS DOS 3 CONTROLADORES
// -----------------------------------------------------------------------------

float integralAnguloMovimentoControle = 0.0f;
float erroAnteriorAnguloMovimentoControle = 0.0f;

float integralVelocidadeMovimentoControle = 0.0f;
float erroAnteriorVelocidadeMovimentoControle = 0.0f;

float integralGiroMovimentoControle = 0.0f;
float erroAnteriorGiroMovimentoControle = 0.0f;

unsigned long ultimoTempoControleMovimento = 0;

bool controleMovimentoAtacanteInicializado = false;


// =============================================================================
// RESET DO CONTROLADOR DOS 3 MOVIMENTOS
// =============================================================================

void resetControleMovimentoAtacante3Fatores()
{
    ultimoAnguloMovimentoControle = 0.0f;
    ultimaVelocidadeMovimentoControle = 0.0f;
    ultimoCmdGiroMovimentoControle = 0.0f;

    integralAnguloMovimentoControle = 0.0f;
    erroAnteriorAnguloMovimentoControle = 0.0f;

    integralVelocidadeMovimentoControle = 0.0f;
    erroAnteriorVelocidadeMovimentoControle = 0.0f;

    integralGiroMovimentoControle = 0.0f;
    erroAnteriorGiroMovimentoControle = 0.0f;

    ultimoTempoControleMovimento = 0;

    controleMovimentoAtacanteInicializado = false;
}


// =============================================================================
// CONTROLADOR PID GENÉRICO
// =============================================================================
//
// Retorna apenas a CORREÇÃO que será aplicada ao último valor.
//
// A saída final será:
//
//     último valor + correção
//
// =============================================================================

float calcularControlePIDMovimento(
    float erro,
    float &integral,
    float &erroAnterior,
    float Kp,
    float Ki,
    float Kd,
    float integralMax,
    float correcaoMax,
    float dt
)
{
    integral += erro * dt;

    integral = constrain(
        integral,
        -integralMax,
        integralMax
    );

    float derivada =
        (erro - erroAnterior) / dt;

    erroAnterior = erro;

    float correcao =
          (Kp * erro)
        + (Ki * integral)
        + (Kd * derivada);

    correcao = constrain(
        correcao,
        -correcaoMax,
        correcaoMax
    );

    return correcao;
}


// =============================================================================
// CONTROLE DOS 3 FATORES DE MOVIMENTO
// =============================================================================
//
// Entradas:
//      anguloReferencia
//      velocidadeReferencia
//      giroReferencia
//
// Saídas:
//      anguloControlado
//      velocidadeControlada
//      giroControlado
//
// =============================================================================

void controlarMovimentoAtacante(
    float anguloReferencia,
    int velocidadeReferencia,
    int giroReferencia,
    float &anguloControlado,
    int &velocidadeControlada,
    int &giroControlado
)
{
    unsigned long agora = millis();

    float dt = 0.02f;

    if (ultimoTempoControleMovimento != 0)
    {
        dt =
            (agora - ultimoTempoControleMovimento)
            / 1000.0f;

        if (dt < 0.005f)
            dt = 0.005f;

        if (dt > 0.2f)
            dt = 0.2f;
    }

    ultimoTempoControleMovimento = agora;


    // =========================================================================
    // PRIMEIRA EXECUÇÃO
    // =========================================================================
    //
    // Evita o robô começar partindo de 0.
    //
    // A primeira referência simplesmente vira o primeiro estado conhecido.
    //
    if (!controleMovimentoAtacanteInicializado)
    {
        ultimoAnguloMovimentoControle =
            normalizarAngulo360(anguloReferencia);

        ultimaVelocidadeMovimentoControle =
            constrain(
                (float)velocidadeReferencia,
                0.0f,
                255.0f
            );

        ultimoCmdGiroMovimentoControle =
            constrain(
                (float)giroReferencia,
                -255.0f,
                255.0f
            );

        erroAnteriorAnguloMovimentoControle = 0.0f;
        erroAnteriorVelocidadeMovimentoControle = 0.0f;
        erroAnteriorGiroMovimentoControle = 0.0f;

        integralAnguloMovimentoControle = 0.0f;
        integralVelocidadeMovimentoControle = 0.0f;
        integralGiroMovimentoControle = 0.0f;

        controleMovimentoAtacanteInicializado = true;

        anguloControlado =
            ultimoAnguloMovimentoControle;

        velocidadeControlada =
            (int)roundf(
                ultimaVelocidadeMovimentoControle
            );

        giroControlado =
            (int)roundf(
                ultimoCmdGiroMovimentoControle
            );

        return;
    }


    // =========================================================================
    // ERRO DO ÂNGULO
    // =========================================================================
    //
    // Aqui NÃO usamos simplesmente:
    //
    //     referencia - atual
    //
    // porque o ângulo é circular.
    //
    // Exemplo:
    //
    //     atual = 350°
    //     alvo  = 10°
    //
    // O erro correto é +20°, e não -340°.
    //
    float erroAngulo =
        normalizarErro180(
            normalizarAngulo360(anguloReferencia)
            - ultimoAnguloMovimentoControle
        );


    // =========================================================================
    // ERRO DA VELOCIDADE
    // =========================================================================

    float erroVelocidade =
        (float)velocidadeReferencia
        - ultimaVelocidadeMovimentoControle;


    // =========================================================================
    // ERRO DO GIRO
    // =========================================================================

    float erroGiro =
        (float)giroReferencia
        - ultimoCmdGiroMovimentoControle;


    // =========================================================================
    // PID / CONTROLE PROPORCIONAL DO ÂNGULO
    // =========================================================================

    float correcaoAngulo =
        calcularControlePIDMovimento(
            erroAngulo,
            integralAnguloMovimentoControle,
            erroAnteriorAnguloMovimentoControle,
            CONTROLE_MOVIMENTO_ANGULO_KP,
            CONTROLE_MOVIMENTO_ANGULO_KI,
            CONTROLE_MOVIMENTO_ANGULO_KD,
            CONTROLE_MOVIMENTO_ANGULO_INTEGRAL_MAX,
            CONTROLE_MOVIMENTO_ANGULO_CORRECAO_MAX,
            dt
        );


    // =========================================================================
    // PID / CONTROLE PROPORCIONAL DA VELOCIDADE
    // =========================================================================

    float correcaoVelocidade =
        calcularControlePIDMovimento(
            erroVelocidade,
            integralVelocidadeMovimentoControle,
            erroAnteriorVelocidadeMovimentoControle,
            CONTROLE_MOVIMENTO_VELOCIDADE_KP,
            CONTROLE_MOVIMENTO_VELOCIDADE_KI,
            CONTROLE_MOVIMENTO_VELOCIDADE_KD,
            CONTROLE_MOVIMENTO_VELOCIDADE_INTEGRAL_MAX,
            CONTROLE_MOVIMENTO_VELOCIDADE_CORRECAO_MAX,
            dt
        );


    // =========================================================================
    // PID / CONTROLE PROPORCIONAL DO GIRO
    // =========================================================================

    float correcaoGiro =
        calcularControlePIDMovimento(
            erroGiro,
            integralGiroMovimentoControle,
            erroAnteriorGiroMovimentoControle,
            CONTROLE_MOVIMENTO_GIRO_KP,
            CONTROLE_MOVIMENTO_GIRO_KI,
            CONTROLE_MOVIMENTO_GIRO_KD,
            CONTROLE_MOVIMENTO_GIRO_INTEGRAL_MAX,
            CONTROLE_MOVIMENTO_GIRO_CORRECAO_MAX,
            dt
        );


    // =========================================================================
    // APLICA A CORREÇÃO SOBRE O ÚLTIMO VALOR
    // =========================================================================
    //
    // ESTA É A PARTE PRINCIPAL DA NOVA LÓGICA.
    //
    // Não fazemos:
    //
    //     saída = PID
    //
    // Fazemos:
    //
    //     saída = último valor + PID
    //
    // =========================================================================

    ultimoAnguloMovimentoControle =
        normalizarAngulo360(
            ultimoAnguloMovimentoControle
            + correcaoAngulo
        );


    ultimaVelocidadeMovimentoControle +=
        correcaoVelocidade;

    ultimaVelocidadeMovimentoControle =
        constrain(
            ultimaVelocidadeMovimentoControle,
            0.0f,
            255.0f
        );


    ultimoCmdGiroMovimentoControle +=
        correcaoGiro;

    ultimoCmdGiroMovimentoControle =
        constrain(
            ultimoCmdGiroMovimentoControle,
            -255.0f,
            255.0f
        );


    // =========================================================================
    // ENTREGA AS NOVAS SAÍDAS
    // =========================================================================

    anguloControlado =
        ultimoAnguloMovimentoControle;

    velocidadeControlada =
        (int)roundf(
            ultimaVelocidadeMovimentoControle
        );

    giroControlado =
        (int)roundf(
            ultimoCmdGiroMovimentoControle
        );
}


// =============================================================================
// PID DE SUAVIZAÇÃO ANGULAR ANTIGO DO ATACANTE
// =============================================================================
// Mantido no código original.
// =============================================================================

const float PID_MOVIMENTO_KP           = 2.0f;
const float PID_MOVIMENTO_KI           = 0.01f;
const float PID_MOVIMENTO_KD           = 0.8f;
const float PID_MOVIMENTO_INTEGRAL_MAX = 90.0f;
const float PID_MOVIMENTO_SAIDA_MAX    = 15.0f;
const float ALPHA_MOVIMENTO            = 0.15f;
const float ALPHA_MOVIMENTO_ALVO       = 0.11f;
const float PASSO_MAX_MOVIMENTO_ALVO_GRAUS = 18.0f;

float        pidMovimentoIntegral              = 0.0f;
float        pidMovimento                      = 0.0f;
float        erroMovimento                     = 0.0f;
float        erroAnteriorMovimento             = 0.0f;
float        anguloMovimentoAtual              = -1.0f;
float        anguloMovimentoSuavizado          = -1.0f;
float        anguloMovimentoDesejadoFiltrado   = -1.0f;
unsigned long ultimoTempoPidMovimento          = 0;
unsigned long inicioCameraSemIrMs              = 0;


// Zera o controlador angular antigo do atacante
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


// PID de transição angular antigo
float calcularPidMovimento(float erro) {
  unsigned long agora = millis();
  float dt = 0.02f;

  if (ultimoTempoPidMovimento != 0) {
    dt = (agora - ultimoTempoPidMovimento) / 1000.0f;

    if (dt < 0.005f)
        dt = 0.005f;

    if (dt > 0.2f)
        dt = 0.2f;
  }

  ultimoTempoPidMovimento = agora;

  pidMovimentoIntegral += erro * dt;

  if (pidMovimentoIntegral > PID_MOVIMENTO_INTEGRAL_MAX)
      pidMovimentoIntegral = PID_MOVIMENTO_INTEGRAL_MAX;

  if (pidMovimentoIntegral < -PID_MOVIMENTO_INTEGRAL_MAX)
      pidMovimentoIntegral = -PID_MOVIMENTO_INTEGRAL_MAX;

  float derivada =
      (erro - erroAnteriorMovimento) / dt;

  erroAnteriorMovimento = erro;

  pidMovimento =
      PID_MOVIMENTO_KP * erro
      + PID_MOVIMENTO_KI * pidMovimentoIntegral
      + PID_MOVIMENTO_KD * derivada;

  if (pidMovimento > PID_MOVIMENTO_SAIDA_MAX)
      pidMovimento = PID_MOVIMENTO_SAIDA_MAX;

  if (pidMovimento < -PID_MOVIMENTO_SAIDA_MAX)
      pidMovimento = -PID_MOVIMENTO_SAIDA_MAX;

  if (fabsf(erro) < 2.0f)
      pidMovimento = 0.0f;

  return -pidMovimento;
}


// Aplica suavização exponencial circular ao ângulo de movimento do atacante
float suavizarAnguloMovimentoAtacante(float anguloMovimentoDesejado) {

  float alvoBruto =
      normalizarAngulo360(
          anguloMovimentoDesejado + 180.0f
      );

  if ((anguloMovimentoAtual < 0.0f) ||
      (anguloMovimentoSuavizado < 0.0f)) {

    anguloMovimentoAtual            = alvoBruto;
    anguloMovimentoSuavizado        = alvoBruto;
    anguloMovimentoDesejadoFiltrado = alvoBruto;

    erroMovimento =
        erroAnteriorMovimento =
        pidMovimentoIntegral =
        pidMovimento = 0.0f;

    ultimoTempoPidMovimento = 0;

    return anguloMovimentoSuavizado;
  }

  float deltaAlvo =
      normalizarErro180(
          alvoBruto
          - anguloMovimentoDesejadoFiltrado
      );

  deltaAlvo =
      constrain(
          deltaAlvo,
          -PASSO_MAX_MOVIMENTO_ALVO_GRAUS,
          PASSO_MAX_MOVIMENTO_ALVO_GRAUS
      );

  anguloMovimentoDesejadoFiltrado =
      normalizarAngulo360(
          anguloMovimentoDesejadoFiltrado
          + deltaAlvo
      );

  anguloMovimentoDesejadoFiltrado =
      normalizarAngulo360(
          anguloMovimentoDesejadoFiltrado +
          ALPHA_MOVIMENTO_ALVO *
          normalizarErro180(
              alvoBruto
              - anguloMovimentoDesejadoFiltrado
          )
      );

  erroMovimento =
      normalizarErro180(
          anguloMovimentoDesejadoFiltrado
          - anguloMovimentoAtual
      );

  pidMovimento =
      calcularPidMovimento(erroMovimento);

  anguloMovimentoAtual =
      normalizarAngulo360(
          anguloMovimentoAtual
          + pidMovimento
      );

  anguloMovimentoSuavizado =
      anguloMovimentoSuavizado +
      ALPHA_MOVIMENTO *
      normalizarErro180(
          anguloMovimentoAtual
          - anguloMovimentoSuavizado
      );

  anguloMovimentoSuavizado =
      normalizarAngulo360(
          anguloMovimentoSuavizado
      );

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
    inicioTransicaoIrMs = agora;
    duracaoTransicaoIrMs = TRANSICAO_ANGULO_IR_MIN_MS;
    anguloIrSuaveInicializado = true;
    return quantizarAnguloPasso(
        anguloIrSuaveAtual,
        PASSO_ANGULO_IR_GRAUS
    );
  }

  float erroNovoAlvo =
      fabsf(
          normalizarErro180(
              alvo - anguloIrSuaveAlvo
          )
      );

  if (erroNovoAlvo >= 1.0f) {
    anguloIrSuaveInicio = anguloIrSuaveAtual;
    anguloIrSuaveAlvo   = alvo;
    inicioTransicaoIrMs = agora;

    float delta =
        fabsf(
            normalizarErro180(
                anguloIrSuaveAlvo
                - anguloIrSuaveInicio
            )
        );

    unsigned long duracaoCalculada =
        (unsigned long)(
            delta *
            TRANSICAO_ANGULO_IR_MS_POR_GRAU
        );

    if (duracaoCalculada < TRANSICAO_ANGULO_IR_MIN_MS)
        duracaoCalculada = TRANSICAO_ANGULO_IR_MIN_MS;

    if (duracaoCalculada > TRANSICAO_ANGULO_IR_MAX_MS)
        duracaoCalculada = TRANSICAO_ANGULO_IR_MAX_MS;

    duracaoTransicaoIrMs = duracaoCalculada;
  }

  unsigned long decorridoMs =
      agora - inicioTransicaoIrMs;

  if (decorridoMs >= duracaoTransicaoIrMs) {
    anguloIrSuaveAtual =
        anguloIrSuaveAlvo;
  } else {
    float progresso =
        (float)decorridoMs /
        (float)duracaoTransicaoIrMs;

    float delta =
        normalizarErro180(
            anguloIrSuaveAlvo
            - anguloIrSuaveInicio
        );

    anguloIrSuaveAtual =
        normalizarAngulo360(
            anguloIrSuaveInicio
            + delta * progresso
        );
  }

  return quantizarAnguloPasso(
      anguloIrSuaveAtual,
      PASSO_ANGULO_IR_GRAUS
  );
}
*/


// =============================================================================
// IR
// =============================================================================

bool irNaFaixaFrontal(float anguloBolaGraus) {
  float ang = normalizarAngulo360(anguloBolaGraus);

  return (ang > 328.0f || ang < 32.0f);
}


// Retém por poucos milissegundos o último ângulo IR válido
bool obterAnguloIrDisponivel(float &anguloBolaGraus) {

  if (irDetectado && anguloIr >= 0.0f) {

    anguloBolaGraus =
        normalizarAngulo360(anguloIr);

    return true;
  }

  if (ultimoAnguloIrValido >= 0.0f &&
      (millis() - ultimoRxIrValidoMs) <=
      RETENCAO_IR_VALIDO_MS) {

    anguloBolaGraus =
        normalizarAngulo360(
            ultimoAnguloIrValido
        );

    return true;
  }

  return false;
}


// =============================================================================
// ULTRASSÔNICOS
// =============================================================================

bool ultraLateralCriticoAtacante() {

  bool ultraDireitoCritico =
      (ultraDcm >= 0.0f) &&
      (ultraDcm <=
       ATACANTE_ULTRA_FREIO_CRITICO_CM);

  bool ultraEsquerdoCritico =
      (ultraEcm >= 0.0f) &&
      (ultraEcm <=
       ATACANTE_ULTRA_FREIO_CRITICO_CM);

  return ultraDireitoCritico ||
         ultraEsquerdoCritico;
}


// Limita velocidade por freio ultrassônico frontal
int aplicarFreioUltrassonicoAtacanteFrente(
    int velocidadeDesejada
) {

  int velocidadeBase =
      constrain(
          velocidadeDesejada,
          0,
          255
      );

  bool ultrasRecentes =
      (ultimoRxUltraMs > 0) &&
      ((millis() - ultimoRxUltraMs) <=
       TIMEOUT_ULTRA_MS);

  if (!ultrasRecentes)
      return velocidadeBase;

  float menorUltraCm = -1.0f;

  float leituras[] = {
      ultraFcm
  };

  for (float leitura : leituras) {

    if (leitura < 0.0f)
        continue;

    if (ultraTcm > 150) {

      if ((menorUltraCm < 0.0f) ||
          (leitura < menorUltraCm)) {

        menorUltraCm = leitura;
      }
    }

    if ((menorUltraCm < 0.0f) ||
        (menorUltraCm >
         ATACANTE_ULTRA_FREIO_INICIO_CM)) {

      return velocidadeBase;
    }

    int velocidadeLimite =
        ATACANTE_ULTRA_FREIO_VELOCIDADE_MIN;

    if (menorUltraCm >
        ATACANTE_ULTRA_FREIO_CRITICO_CM) {

      velocidadeLimite +=
          (int)(
              (menorUltraCm -
               ATACANTE_ULTRA_FREIO_CRITICO_CM)
              *
              ATACANTE_ULTRA_FREIO_PWM_POR_CM
          );
    }

    if (velocidadeLimite >
        velocidade_maxima) {

      velocidadeLimite =
          velocidade_maxima;
    }

    return min(
        velocidadeBase,
        velocidadeLimite
    );
  }

  return velocidadeBase;
}


// Limita velocidade por freio ultrassônico lateral
int aplicarFreioUltrassonicoAtacante(
    int velocidadeDesejada
) {

  int velocidadeBase =
      constrain(
          velocidadeDesejada,
          0,
          255
      );

  bool ultrasRecentes =
      (ultimoRxUltraMs > 0) &&
      ((millis() - ultimoRxUltraMs) <=
       TIMEOUT_ULTRA_MS);

  if (!ultrasRecentes)
      return velocidadeBase;

  float menorUltraCm = -1.0f;

  float leituras[] = {
      ultraDcm,
      ultraEcm
  };

  for (float leitura : leituras) {

    if (leitura < 0.0f)
        continue;

    if ((menorUltraCm < 0.0f) ||
        (leitura < menorUltraCm)) {

      menorUltraCm = leitura;
    }
  }

  if ((menorUltraCm < 0.0f) ||
      (menorUltraCm >
       ATACANTE_ULTRA_FREIO_INICIO_CM)) {

    return velocidadeBase;
  }

  int velocidadeLimite =
      ATACANTE_ULTRA_FREIO_VELOCIDADE_MIN;

  if (menorUltraCm >
      ATACANTE_ULTRA_FREIO_CRITICO_CM) {

    velocidadeLimite +=
        (int)(
            (menorUltraCm -
             ATACANTE_ULTRA_FREIO_CRITICO_CM)
            *
            ATACANTE_ULTRA_FREIO_PWM_POR_CM
        );
  }

  if (velocidadeLimite >
      velocidade_maxima) {

    velocidadeLimite =
        velocidade_maxima;
  }

  return min(
      velocidadeBase,
      velocidadeLimite
  );
}


// =============================================================================
// MAPEAMENTO DA BOLA
// =============================================================================

float mapearAnguloBolaParaMovimento(
    float anguloBolaGraus
)
{
  float ang =
      normalizarAngulo360(
          anguloBolaGraus
      );

  if (ang >= 32.0f && ang <= 60.0f)
      return 100.0f;

  if (ang > 60.0f && ang < 90.0f)
      return 90.0f;

  if (ang >= 90.0f && ang < 135.0f)
      return 180.0f;

  if (ang >= 135.0f && ang < 180.0f)
      return 225.0f;

  if (ang >= 180.0f && ang < 225.0f)
      return 135.0f;

  if (ang >= 225.0f && ang < 270.0f)
      return 180.0f;

  if (ang >= 270.0f && ang < 300.0f)
      return 270.0f;

  if (ang >= 300.0f && ang <= 328.0f)
      return 260.0f;

  return ang;
}


// Reduz velocidade em faixas próximas do frontal
int calcularVelocidadeIrPorAngulo(
    float anguloBolaGraus
) {

  float ang =
      normalizarAngulo360(
          anguloBolaGraus
      );

  if ((ang >= 33.0f && ang <= 60.0f) ||
      (ang >= 300.0f && ang <= 328.0f)) {

    return VELOCIDADE_IR_FAIXA_REDUZIDA_PWM;
  }

  if (ang >= 140.0f && ang < 220.0f)
      return VELOCIDADE_IR_FAIXA_REDUZIDA_PWM;

  return velocidade_maxima;
}


// =============================================================================
// BUSCA SEM BOLA
// =============================================================================

float calcularAnguloBuscaSemBolaCameraAtacante() {

  bool ultrasRecentes =
      ultrasValidos &&
      (ultimoRxUltraMs > 0) &&
      ((millis() - ultimoRxUltraMs) <=
       TIMEOUT_ULTRA_MS);

  if (ultrasRecentes) {

    bool esquerdaPerto =
        (ultraEcm >= 0.0f) &&
        (ultraEcm < 60.0f);

    bool direitaPerto =
        (ultraDcm >= 0.0f) &&
        (ultraDcm < 60.0f);

    bool esquerdaLivre =
        ultraEcm > 50.0f;

    bool direitaLivre =
        ultraDcm > 50.0f;

    if (esquerdaPerto && direitaLivre)
        return 90.0f;

    if (direitaPerto && esquerdaLivre)
        return 270.0f;
  }

  return 0.0f;
}


// =============================================================================
// FILTRO MEGA ANGULO
// =============================================================================

float suavizadorMegaAnguloMovimento(
    float anguloNovo
)
{
    static float anguloFiltrado = 0.0f;

    const float ALFA = 0.18f;

    float diff =
        anguloNovo -
        anguloFiltrado;

    if (diff > 180.0f)
        diff -= 360.0f;

    if (diff < -180.0f)
        diff += 360.0f;

    anguloFiltrado +=
        ALFA * diff;

    if (anguloFiltrado < 0)
        anguloFiltrado += 360.0f;

    if (anguloFiltrado >= 360.0f)
        anguloFiltrado -= 360.0f;

    return anguloFiltrado;
}


// =============================================================================
// PID DA BÚSSOLA
// =============================================================================

float PIDZIMBUSSOLANOVINHA(
    float erro
)
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

    dt =
        (agora - ultimoMs)
        / 1000.0f;

    if (dt < 0.005f)
        dt = 0.005f;

    if (dt > 0.2f)
        dt = 0.2f;
  }

  ultimoMs = agora;

  integral +=
      erro * dt;

  integral =
      constrain(
          integral,
          -100.0f,
          100.0f
      );

  float derivada =
      (erro - erroAnterior)
      / dt;

  float saida =
      (Kp * erro)
    + (Ki * integral)
    + (Kd * derivada);

  erroAnterior = erro;

  return -saida;
}


// =============================================================================
// CONTROLE DE GIRO PARA O GOL DURANTE ATAQUE FRONTAL
// =============================================================================

const float PID_GOL_CAMERA_KP = 1.2f;
const float PID_GOL_CAMERA_KI = 0.0f;
const float PID_GOL_CAMERA_KD = 0.5f;

const float PID_GOL_CAMERA_INTEGRAL_MAX = 100.0f;

const int PID_GOL_CAMERA_SAIDA_MIN = 30;
const int PID_GOL_CAMERA_SAIDA_MAX = 180;

const float TOLERANCIA_GOL_CAMERA_GRAUS = 2.0f;

const unsigned long RETENCAO_GOL_CAMERA_ATAQUE_MS = 100;

const float SALTO_MAX_GOL_CAMERA_GRAUS = 20.0f;


// Estado do PID específico da câmera.
float pidGolCameraIntegral = 0.0f;
float pidGolCameraErroAnterior = 0.0f;
unsigned long pidGolCameraUltimoMs = 0;


// Estado do filtro do ângulo do gol.
float cameraGolAnguloFiltrado = 0.0f;
bool cameraGolFiltroInicializado = false;
unsigned long cameraGolUltimaLeituraValidaMs = 0;


// =============================================================================
// RESET PID DO GOL
// =============================================================================

void resetPidGolCamera()
{
  pidGolCameraIntegral = 0.0f;
  pidGolCameraErroAnterior = 0.0f;
  pidGolCameraUltimoMs = 0;
}


// =============================================================================
// RESET FILTRO E PID DO GOL
// =============================================================================

void resetControleGolCamera()
{
  cameraGolAnguloFiltrado = 0.0f;
  cameraGolFiltroInicializado = false;
  cameraGolUltimaLeituraValidaMs = 0;

  resetPidGolCamera();
}


// =============================================================================
// OBTÉM ÂNGULO DO GOL
// =============================================================================

bool obterAnguloGolCameraAtaque(
    float &anguloGol
)
{
  const unsigned long agora =
      millis();

  int16_t leituraGol = -999;

  if (cameraTemGolSelecionadoValido(
          leituraGol))
  {
    float leitura =
        normalizarErro180(
            (float)leituraGol
        );

    if (!cameraGolFiltroInicializado)
    {
      cameraGolAnguloFiltrado =
          leitura;

      cameraGolFiltroInicializado =
          true;
    }
    else
    {
      float delta =
          normalizarErro180(
              leitura -
              cameraGolAnguloFiltrado
          );

      delta =
          constrain(
              delta,
              -SALTO_MAX_GOL_CAMERA_GRAUS,
               SALTO_MAX_GOL_CAMERA_GRAUS
          );

      const float ALPHA_GOL_CAMERA =
          0.5f;

      cameraGolAnguloFiltrado =
          normalizarErro180(
              cameraGolAnguloFiltrado +
              (ALPHA_GOL_CAMERA * delta)
          );
    }

    cameraGolUltimaLeituraValidaMs =
        agora;

    anguloGol =
        cameraGolAnguloFiltrado;

    return true;
  }

  if (cameraGolFiltroInicializado &&
      cameraGolUltimaLeituraValidaMs > 0 &&
      (agora -
       cameraGolUltimaLeituraValidaMs) <=
      RETENCAO_GOL_CAMERA_ATAQUE_MS)
  {
    anguloGol =
        cameraGolAnguloFiltrado;

    return true;
  }

  return false;
}


// =============================================================================
// CALCULA CMD GIRO DO GOL PELA CÂMERA
// =============================================================================

int calcularCmdGiroGolCamera(
    float anguloGol
)
{
  const unsigned long agora =
      millis();

  float dt = 0.02f;

  if (pidGolCameraUltimoMs != 0)
  {
    dt =
        (agora - pidGolCameraUltimoMs)
        / 1000.0f;

    if (dt < 0.005f)
        dt = 0.005f;

    if (dt > 0.2f)
        dt = 0.2f;
  }

  pidGolCameraUltimoMs =
      agora;

  float erroGol =
      normalizarErro180(
          -anguloGol
      );

  if (fabsf(erroGol) <=
      TOLERANCIA_GOL_CAMERA_GRAUS)
  {
    pidGolCameraIntegral = 0.0f;
    pidGolCameraErroAnterior =
        erroGol;

    return 0;
  }

  pidGolCameraIntegral +=
      erroGol * dt;

  pidGolCameraIntegral =
      constrain(
          pidGolCameraIntegral,
          -PID_GOL_CAMERA_INTEGRAL_MAX,
           PID_GOL_CAMERA_INTEGRAL_MAX
      );

  float derivada =
      (erroGol -
       pidGolCameraErroAnterior)
      / dt;

  pidGolCameraErroAnterior =
      erroGol;

  float saida =
      PID_GOL_CAMERA_KP * erroGol
    + PID_GOL_CAMERA_KI *
      pidGolCameraIntegral
    + PID_GOL_CAMERA_KD *
      derivada;

  int magnitude =
      (int)fabsf(saida);

  magnitude =
      constrain(
          magnitude,
          PID_GOL_CAMERA_SAIDA_MIN,
          PID_GOL_CAMERA_SAIDA_MAX
      );

  magnitude =
      min(
          magnitude,
          VELOCIDADE_GIRO_ALINHAMENTO
      );

  int cmdGiro =
      (saida >= 0.0f)
      ? magnitude
      : -magnitude;

  cmdGiro *=
      SINAL_GIRO_PID;

  return constrain(
      cmdGiro,
      -255,
      255
  );
}


// =============================================================================
// FRENTE + GIRO PARA O GOL
// =============================================================================

void moverFrenteComGiroParaGol(
    int velocidade
)
{
  float anguloGol = 0.0f;

  if (obterAnguloGolCameraAtaque(
          anguloGol))
  {
    int cmdGiroGol =
        calcularCmdGiroGolCamera(
            anguloGol
        );

    moverFrenteComGiro(
        velocidade,
        cmdGiroGol
    );

    return;
  }

  // Sem gol válido na câmera:
  // fallback seguro para a bússola.
  resetPidGolCamera();

  float erroBussola =
      normalizarErro180(
          -calcularErroReferenciaBussola()
      );

  int cmdGiroBussola =
      constrain(
          (int)roundf(
              -PIDZIMBUSSOLANOVINHA(
                  erroBussola
              )
          ),
          -255,
          255
      );

  moverFrenteComGiro(
      velocidade,
      cmdGiroBussola
  );
}