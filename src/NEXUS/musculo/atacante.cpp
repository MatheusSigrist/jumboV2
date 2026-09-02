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

    // =========================================================================
    // PROTEÇÃO ROBUSTA DA LINHA — V2
    // =========================================================================
    // PRINCÍPIOS:
    //   1) Linha tem prioridade sobre bola/câmera/bússola durante a fuga normal.
    //   2) Entrada é instantânea; somente a SAÍDA possui confirmação temporal.
    //   3) O freio frontal aprovado nos testes foi preservado.
    //   4) Pequenas perdas de leitura NÃO reiniciam o watchdog da ocorrência V2.
    //   5) A recuperação continua consultando sairDaLinha() e acompanha a
    //      posição ATUAL da linha, em vez de usar apenas uma direção antiga.
    //   6) Reentradas repetidas na linha antecipam a recuperação.
    //   7) FAILSAFE FINAL: 2000 ms contínuos de linha bloqueiam a fuga e
    //      devolvem o controle à estratégia normal, como na versão antiga.
    // =========================================================================

    enum EstadoProtecaoLinhaAtacante {
        LINHA_LIVRE,
        LINHA_FREIO_FRONTAL,
        LINHA_FREIO_TRASEIRO,
        LINHA_ESCAPE_TRASEIRO_MINIMO,
        LINHA_FUGA_NORMAL,
        LINHA_RECUPERACAO,
        LINHA_CONFIRMAR_SAIDA
    };

    static EstadoProtecaoLinhaAtacante estadoLinha = LINHA_LIVRE;

    // ------------------------- AJUSTES DA PROTEÇÃO ----------------------------
    static constexpr unsigned long TEMPO_FREIO_FRONTAL_MS         = 40UL;

    // Proteção específica quando o robô encontra a linha ATRÁS enquanto estava
    // realmente se deslocando para trás. O freio corta a inércia e o escape
    // mínimo impede que o desaparecimento da faixa branca seja confundido com
    // uma saída segura depois de o robô atravessar completamente a linha.
    static constexpr unsigned long TEMPO_FREIO_TRASEIRO_MS         = 40UL;
    static constexpr unsigned long TEMPO_ESCAPE_TRASEIRO_MIN_MS    = 250UL;

    // Menor que na V1: se a fuga normal não resolveu em ~0,9 s, não vale a pena
    // continuar insistindo exatamente na mesma condição.
    static constexpr unsigned long TEMPO_MAX_OCORRENCIA_NORMAL_MS = 900UL;

    static constexpr unsigned long TEMPO_CONFIRMAR_SAIDA_MS       = 120UL;
    static constexpr unsigned long PERIODO_RECUPERACAO_MS         = 180UL;
    static constexpr unsigned long TEMPO_RECUPERACAO_FORTE_MS     = 900UL;

    // Se a leitura some e reaparece repetidamente sem confirmar a saída,
    // considera que o robô está "raspando"/oscilando sobre a mesma linha.
    static constexpr uint8_t MAX_REENTRADAS_ANTES_RECUPERAR = 3;

    static constexpr float FAIXA_LINHA_FRONTAL_GRAUS = 50.0f;

    // Zona traseira ampla. No referencial de movimento do atacante:
    //   0°   = frente
    //   180° = trás
    // A proteção especial só entra se a linha estiver nesta zona E o último
    // movimento estratégico também tiver componente real para trás.
    static constexpr float LINHA_TRASEIRA_MIN_GRAUS = 120.0f;
    static constexpr float LINHA_TRASEIRA_MAX_GRAUS = 240.0f;
    static constexpr float LIMIAR_COMPONENTE_RECUO   = -0.20f;

    // Recuperação lateral/traseira precisa de desvios maiores que a V1.
    static constexpr float DESVIO_RECUPERACAO_MEDIO_GRAUS = 35.0f;
    static constexpr float DESVIO_RECUPERACAO_FORTE_GRAUS = 60.0f;

    // Cronômetro da OCORRÊNCIA inteira. Ele só é zerado após a linha ficar
    // realmente ausente pelo TEMPO_CONFIRMAR_SAIDA_MS.
    static unsigned long inicioOcorrenciaLinhaMs = 0UL;

    // Cronômetro do estado atual (freio/recuperação).
    static unsigned long inicioEstadoLinhaMs = 0UL;
    static unsigned long inicioSemLinhaMs = 0UL;

    static uint8_t reentradasLinha = 0;

    static float ultimaDirecaoFugaLinha = 0.0f;
    static bool  temDirecaoFugaLinha = false;

    // Último movimento ESTRATÉGICO realmente comandado pelo atacante.
    // É separado dos comandos de fuga para sabermos em que direção o robô
    // estava indo ANTES de tocar a linha.
    static float ultimoAnguloMovimentoAtaqueCmd = 0.0f;
    static bool  ultimoMovimentoAtaqueCmdValido = false;

    // Estado específico da proteção traseira.
    static float direcaoEscapeLinhaTraseira = 0.0f;
    static unsigned long inicioEscapeLinhaTraseiraMs = 0UL;

    // -------------------------------------------------------------------------
    // FAILSAFE FINAL DE 2000 ms — MESMO PRINCÍPIO DA VERSÃO ANTIGA
    // -------------------------------------------------------------------------
    // Se linhaDetectada permanecer TRUE continuamente por 2000 ms, toda a
    // estratégia de fuga é encerrada e atacante() retorna imediatamente ao loop
    // principal. A fuga permanece bloqueada nas próximas chamadas enquanto a
    // linha continuar ativa e só é rearmada quando linhaDetectada voltar a FALSE.
    static constexpr unsigned long TEMPO_TIMEOUT_FINAL_LINHA_MS = 2000UL;
    static unsigned long inicioTimeoutFinalLinhaMs = 0UL;
    static bool fugaLinhaBloqueadaPorTimeout = false;

    const unsigned long agoraLinhaMs = millis();

    // Saiu da linha: rearma o timeout final para uma próxima ocorrência.
    if (!linhaDetectada) {
        inicioTimeoutFinalLinhaMs = 0UL;
        fugaLinhaBloqueadaPorTimeout = false;
    }

    // Enquanto a linha permanecer continuamente detectada, conta os 2000 ms.
    if (linhaDetectada && !fugaLinhaBloqueadaPorTimeout) {
        if (inicioTimeoutFinalLinhaMs == 0UL) {
            inicioTimeoutFinalLinhaMs = agoraLinhaMs;
        }

        if ((agoraLinhaMs - inicioTimeoutFinalLinhaMs) >=
            TEMPO_TIMEOUT_FINAL_LINHA_MS) {

            // ================================================================
            // TIMEOUT FINAL: QUEBRA REAL DA AÇÃO DE LINHA
            // ================================================================
            // Ao completar 2000 ms, não continuamos descendo dentro de atacante()
            // neste mesmo ciclo. Isso evita que um estado antigo de fuga permaneça
            // comandando os motores quando, por exemplo, o IR também foi perdido.
            //
            // O bloqueio permanece armado até linhaDetectada voltar a FALSE.
            // No próximo ciclo do loop principal, atacante() será chamado de novo
            // já com a rotina de linha bloqueada pelo timeout.
            fugaLinhaBloqueadaPorTimeout = true;
            fugindoLinhaAgora = false;
            anguloFugaLinhaCmd = 0.0f;

            // Limpa completamente a máquina de estados externa da linha.
            estadoLinha = LINHA_LIVRE;
            inicioOcorrenciaLinhaMs = 0UL;
            inicioEstadoLinhaMs = 0UL;
            inicioSemLinhaMs = 0UL;
            reentradasLinha = 0;
            temDirecaoFugaLinha = false;
            ultimaDirecaoFugaLinha = 0.0f;
            inicioEscapeLinhaTraseiraMs = 0UL;
            direcaoEscapeLinhaTraseira = 0.0f;

            // Não conserva uma direção estratégica antiga depois do timeout.
            ultimoMovimentoAtaqueCmdValido = false;

            // Zera explicitamente o último comando de movimento antes de sair.
            // Assim o último vetor de fuga não continua ativo caso o IR esteja ausente.
            girarNoEixo(0);

            // Retorna imediatamente ao loop principal.
            return;
        }
    }

    // Enquanto NÃO houve timeout, executa toda a proteção robusta da V2.
    // Se o timeout disparar, atacante() já terá retornado ao loop principal.
    // Nas próximas chamadas, este bloco fica ignorado até a linha desaparecer.
    if (!fugaLinhaBloqueadaPorTimeout) {

    // -------------------------------------------------------------------------
    // 1. ENTRADA IMEDIATA NA PROTEÇÃO
    // -------------------------------------------------------------------------
    if (estadoLinha == LINHA_LIVRE && linhaDetectada) {

        inicioOcorrenciaLinhaMs = agoraLinhaMs;
        inicioEstadoLinhaMs = agoraLinhaMs;
        inicioSemLinhaMs = 0UL;
        reentradasLinha = 0;
        temDirecaoFugaLinha = false;

        bool linhaNaZonaFrontal = false;
        bool linhaNaZonaTraseira = false;
        float anguloLinhaNormalizado = -1.0f;

        if (anguloLinhaPe >= 0.0f) {
            anguloLinhaNormalizado =
                normalizarAngulo360(anguloLinhaPe);

            linhaNaZonaFrontal =
                (anguloLinhaNormalizado <= FAIXA_LINHA_FRONTAL_GRAUS) ||
                (anguloLinhaNormalizado >= (360.0f - FAIXA_LINHA_FRONTAL_GRAUS));

            linhaNaZonaTraseira =
                (anguloLinhaNormalizado >= LINHA_TRASEIRA_MIN_GRAUS) &&
                (anguloLinhaNormalizado <= LINHA_TRASEIRA_MAX_GRAUS);
        }

        // Verifica a direção que estava sendo comandada ANTES da linha aparecer.
        // cos(ângulo) < 0 significa componente para trás. O limiar -0,20 evita
        // ativar a proteção especial em movimentos quase puramente laterais.
        bool estavaMovendoParaTras = false;

        if (ultimoMovimentoAtaqueCmdValido) {
            const float anguloMovimentoRad =
                normalizarAngulo360(ultimoAnguloMovimentoAtaqueCmd) * PI / 180.0f;

            estavaMovendoParaTras =
                cosf(anguloMovimentoRad) < LIMIAR_COMPONENTE_RECUO;
        }

        const bool emergenciaLinhaTraseira =
            linhaNaZonaTraseira &&
            estavaMovendoParaTras &&
            (anguloLinhaNormalizado >= 0.0f);

        if (emergenciaLinhaTraseira) {
            // Memoriza imediatamente uma direção para DENTRO do campo:
            // oposta ao ponto onde a linha foi vista. Para uma linha traseira
            // (120°..240°), o resultado sempre cai no hemisfério frontal.
            direcaoEscapeLinhaTraseira =
                normalizarAngulo360(anguloLinhaNormalizado + 180.0f);

            ultimaDirecaoFugaLinha = direcaoEscapeLinhaTraseira;
            temDirecaoFugaLinha = true;

            estadoLinha = LINHA_FREIO_TRASEIRO;
        }
        else {
            // Preserva exatamente a ideia que funcionou bem nos testes frontais.
            estadoLinha = linhaNaZonaFrontal
                ? LINHA_FREIO_FRONTAL
                : LINHA_FUGA_NORMAL;
        }
    }

    // -------------------------------------------------------------------------
    // 2. FREIO CURTO FRONTAL — PRESERVADO
    // -------------------------------------------------------------------------
    if (estadoLinha == LINHA_FREIO_FRONTAL) {

        fugindoLinhaAgora = true;

        if (!linhaDetectada) {
            estadoLinha = LINHA_CONFIRMAR_SAIDA;
            inicioSemLinhaMs = agoraLinhaMs;
        }
        else if ((agoraLinhaMs - inicioEstadoLinhaMs) < TEMPO_FREIO_FRONTAL_MS) {
            girarNoEixo(0);
            return;
        }
        else {
            estadoLinha = LINHA_FUGA_NORMAL;
            inicioEstadoLinhaMs = agoraLinhaMs;
            // IMPORTANTE: inicioOcorrenciaLinhaMs NÃO é zerado aqui.
        }
    }

    // -------------------------------------------------------------------------
    // 2B. FREIO TRASEIRO + ESCAPE MÍNIMO CONTEXTUAL
    // -------------------------------------------------------------------------
    // Só é usado quando: linha atrás + último comando de ataque indo para trás.
    // Diferente da fuga genérica, o desaparecimento instantâneo da linha NÃO
    // encerra esta proteção: isso evita considerar como "saída" o caso em que
    // o robô atravessou completamente a faixa branca por inércia.
    if (estadoLinha == LINHA_FREIO_TRASEIRO) {

        fugindoLinhaAgora = true;
        anguloFugaLinhaCmd = direcaoEscapeLinhaTraseira;

        if ((agoraLinhaMs - inicioEstadoLinhaMs) < TEMPO_FREIO_TRASEIRO_MS) {
            girarNoEixo(0);
            return;
        }

        estadoLinha = LINHA_ESCAPE_TRASEIRO_MINIMO;
        inicioEscapeLinhaTraseiraMs = agoraLinhaMs;
        inicioEstadoLinhaMs = agoraLinhaMs;
    }

    if (estadoLinha == LINHA_ESCAPE_TRASEIRO_MINIMO) {

        fugindoLinhaAgora = true;

        // Enquanto ainda enxerga a linha na região traseira, atualiza a direção
        // oposta usando a leitura ATUAL. Isso melhora a precisão se o robô girar
        // ou tocar a faixa em diagonal durante a retirada.
        if (linhaDetectada && anguloLinhaPe >= 0.0f) {
            const float anguloLinhaAtual =
                normalizarAngulo360(anguloLinhaPe);

            if (anguloLinhaAtual >= LINHA_TRASEIRA_MIN_GRAUS &&
                anguloLinhaAtual <= LINHA_TRASEIRA_MAX_GRAUS) {

                direcaoEscapeLinhaTraseira =
                    normalizarAngulo360(anguloLinhaAtual + 180.0f);

                ultimaDirecaoFugaLinha = direcaoEscapeLinhaTraseira;
                temDirecaoFugaLinha = true;
            }
        }

        anguloFugaLinhaCmd = direcaoEscapeLinhaTraseira;

        const unsigned long tempoEscapeTraseiro =
            agoraLinhaMs - inicioEscapeLinhaTraseiraMs;

        if (tempoEscapeTraseiro < TEMPO_ESCAPE_TRASEIRO_MIN_MS) {
            // Mantém o afastamento mesmo se linhaDetectada já tiver ficado FALSE.
            seguirDirecaoComGiro(
                direcaoEscapeLinhaTraseira,
                VELOCIDADE_FUGA_LINHA,
                0
            );
            return;
        }

        // Após cumprir o deslocamento mínimo, volta para a máquina de estados
        // normal. Se ainda há linha, sairDaLinha() continua trabalhando; se não
        // há, começamos a confirmação temporal de saída já existente.
        if (linhaDetectada) {
            estadoLinha = LINHA_FUGA_NORMAL;
            inicioEstadoLinhaMs = agoraLinhaMs;
        }
        else {
            estadoLinha = LINHA_CONFIRMAR_SAIDA;
            inicioSemLinhaMs = agoraLinhaMs;
        }
    }

    // -------------------------------------------------------------------------
    // 3. CONFIRMAÇÃO DE SAÍDA
    // -------------------------------------------------------------------------
    if (estadoLinha == LINHA_CONFIRMAR_SAIDA) {

        if (linhaDetectada) {
            // A linha reapareceu: era apenas uma perda momentânea de leitura.
            // NÃO reinicia o cronômetro da ocorrência.
            if (reentradasLinha < 255) {
                reentradasLinha++;
            }

            inicioSemLinhaMs = 0UL;

            const bool ocorrenciaLonga =
                (inicioOcorrenciaLinhaMs > 0UL) &&
                ((agoraLinhaMs - inicioOcorrenciaLinhaMs) >=
                 TEMPO_MAX_OCORRENCIA_NORMAL_MS);

            const bool muitasReentradas =
                reentradasLinha >= MAX_REENTRADAS_ANTES_RECUPERAR;

            if (ocorrenciaLonga || muitasReentradas) {
                estadoLinha = LINHA_RECUPERACAO;
                inicioEstadoLinhaMs = agoraLinhaMs;
            } else {
                estadoLinha = LINHA_FUGA_NORMAL;
            }
        }
        else {
            // Permite que sairDaLinha() conclua eventual estado interno.
            float anguloFinalizacao = 0.0f;
            const bool fugaOriginalFinalizando =
                sairDaLinha(
                    false,
                    anguloLinhaPe,
                    VELOCIDADE_FUGA_LINHA,
                    &anguloFinalizacao
                );

            if (fugaOriginalFinalizando) {
                ultimaDirecaoFugaLinha = anguloFinalizacao;
                temDirecaoFugaLinha = true;
                fugindoLinhaAgora = true;
                anguloFugaLinhaCmd = anguloFinalizacao;
            }

            if ((agoraLinhaMs - inicioSemLinhaMs) < TEMPO_CONFIRMAR_SAIDA_MS) {
                // Mantém afastamento somente enquanto confirma a saída.
                if (!fugaOriginalFinalizando) {
                    if (temDirecaoFugaLinha) {
                        fugindoLinhaAgora = true;
                        anguloFugaLinhaCmd = ultimaDirecaoFugaLinha;
                        seguirDirecaoComGiro(
                            ultimaDirecaoFugaLinha,
                            VELOCIDADE_FUGA_LINHA,
                            0
                        );
                    } else {
                        girarNoEixo(0);
                    }
                }
                return;
            }

            // SAÍDA REALMENTE CONFIRMADA: somente aqui rearma tudo.
            estadoLinha = LINHA_LIVRE;
            inicioOcorrenciaLinhaMs = 0UL;
            inicioEstadoLinhaMs = 0UL;
            inicioSemLinhaMs = 0UL;
            reentradasLinha = 0;
            fugindoLinhaAgora = false;
            temDirecaoFugaLinha = false;
            inicioEscapeLinhaTraseiraMs = 0UL;
        }
    }

    // -------------------------------------------------------------------------
    // 4. FUGA NORMAL — sairDaLinha() CONTINUA SENDO A PRINCIPAL
    // -------------------------------------------------------------------------
    if (estadoLinha == LINHA_FUGA_NORMAL) {

        const bool fugaOriginalAtiva =
            sairDaLinha(
                linhaDetectada,
                anguloLinhaPe,
                VELOCIDADE_FUGA_LINHA,
                &anguloFuga
            );

        if (fugaOriginalAtiva) {
            // Atualiza SEMPRE. Não congela a primeira direção recebida.
            ultimaDirecaoFugaLinha = anguloFuga;
            temDirecaoFugaLinha = true;

            fugindoLinhaAgora = true;
            anguloFugaLinhaCmd = anguloFuga;
        }

        if (!linhaDetectada) {
            estadoLinha = LINHA_CONFIRMAR_SAIDA;
            inicioSemLinhaMs = agoraLinhaMs;

            if (!fugaOriginalAtiva) {
                if (temDirecaoFugaLinha) {
                    fugindoLinhaAgora = true;
                    anguloFugaLinhaCmd = ultimaDirecaoFugaLinha;
                    seguirDirecaoComGiro(
                        ultimaDirecaoFugaLinha,
                        VELOCIDADE_FUGA_LINHA,
                        0
                    );
                } else {
                    girarNoEixo(0);
                }
            }
            return;
        }

        // O watchdog agora mede a ocorrência INTEIRA, inclusive pequenos falses.
        const bool ocorrenciaLonga =
            (inicioOcorrenciaLinhaMs > 0UL) &&
            ((agoraLinhaMs - inicioOcorrenciaLinhaMs) >=
             TEMPO_MAX_OCORRENCIA_NORMAL_MS);

        if (ocorrenciaLonga) {
            estadoLinha = LINHA_RECUPERACAO;
            inicioEstadoLinhaMs = agoraLinhaMs;
            // Não retorna ainda: já executa o primeiro ciclo de recuperação.
        }
        else {
            if (fugaOriginalAtiva) {
                return;
            }

            // Se a função original não comandar neste ciclo, usa a leitura ATUAL
            // da linha. Diferente da V1, esse fallback também pode ser atualizado.
            if (anguloLinhaPe >= 0.0f) {
                ultimaDirecaoFugaLinha =
                    normalizarAngulo360(anguloLinhaPe + 180.0f);
                temDirecaoFugaLinha = true;
            }

            if (temDirecaoFugaLinha) {
                fugindoLinhaAgora = true;
                anguloFugaLinhaCmd = ultimaDirecaoFugaLinha;
                seguirDirecaoComGiro(
                    ultimaDirecaoFugaLinha,
                    VELOCIDADE_FUGA_LINHA,
                    0
                );
            } else {
                fugindoLinhaAgora = true;
                girarNoEixo(0);
            }
            return;
        }
    }

    // -------------------------------------------------------------------------
    // 5. RECUPERAÇÃO DINÂMICA DE DESTRAVAMENTO
    // -------------------------------------------------------------------------
    // DIFERENÇA PRINCIPAL DA V2:
    //   - sairDaLinha() continua sendo consultada em TODOS os ciclos;
    //   - a direção-base acompanha anguloLinhaPe atual;
    //   - os desvios são maiores para liberar laterais/traseira;
    //   - o timer não é reiniciado por flicker da linha.
    // -------------------------------------------------------------------------
    if (estadoLinha == LINHA_RECUPERACAO) {

        fugindoLinhaAgora = true;

        if (!linhaDetectada) {
            estadoLinha = LINHA_CONFIRMAR_SAIDA;
            inicioSemLinhaMs = agoraLinhaMs;

            if (temDirecaoFugaLinha) {
                anguloFugaLinhaCmd = ultimaDirecaoFugaLinha;
                seguirDirecaoComGiro(
                    ultimaDirecaoFugaLinha,
                    VELOCIDADE_FUGA_LINHA,
                    0
                );
            } else {
                girarNoEixo(0);
            }
            return;
        }

        // 5.1 — Tenta obter a direção que a lógica original considera segura AGORA.
        float direcaoOriginalAtual = 0.0f;
        const bool fugaOriginalRecuperacao =
            sairDaLinha(
                true,
                anguloLinhaPe,
                VELOCIDADE_FUGA_LINHA,
                &direcaoOriginalAtual
            );

        if (fugaOriginalRecuperacao) {
            ultimaDirecaoFugaLinha = direcaoOriginalAtual;
            temDirecaoFugaLinha = true;
        }
        else if (anguloLinhaPe >= 0.0f) {
            // 5.2 — Se sairDaLinha() estiver em um estado que não devolve comando,
            // deriva uma direção usando a posição ATUAL da linha.
            ultimaDirecaoFugaLinha =
                normalizarAngulo360(anguloLinhaPe + 180.0f);
            temDirecaoFugaLinha = true;
        }

        if (temDirecaoFugaLinha) {

            const unsigned long tempoRecuperando =
                agoraLinhaMs - inicioEstadoLinhaMs;

            const float amplitudeDesvio =
                (tempoRecuperando >= TEMPO_RECUPERACAO_FORTE_MS)
                ? DESVIO_RECUPERACAO_FORTE_GRAUS
                : DESVIO_RECUPERACAO_MEDIO_GRAUS;

            // Fases:
            //   0 = direção segura atual
            //   1 = diagonal +
            //   2 = direção segura atual
            //   3 = diagonal -
            // Isso quebra travamentos tangenciais sem inverter para o lado da linha.
            const unsigned long fase =
                (tempoRecuperando / PERIODO_RECUPERACAO_MS) % 4UL;

            float desvio = 0.0f;
            if (fase == 1UL) desvio =  amplitudeDesvio;
            if (fase == 3UL) desvio = -amplitudeDesvio;

            const float anguloRecuperacao =
                normalizarAngulo360(ultimaDirecaoFugaLinha + desvio);

            // Um pequeno reforço de PWM durante recuperação ajuda a vencer atrito
            // estático/roda encostada, sem ultrapassar o limite do driver.
            const int velocidadeRecuperacao =
                constrain((int)VELOCIDADE_FUGA_LINHA + 20, 0, 255);

            anguloFugaLinhaCmd = anguloRecuperacao;

            seguirDirecaoComGiro(
                anguloRecuperacao,
                velocidadeRecuperacao,
                0
            );
        }
        else {
            // Sem nenhuma referência angular confiável, não libera a bola.
            girarNoEixo(0);
        }

        return;
    }

    } // fim: if (!fugaLinhaBloqueadaPorTimeout)

    // Enquanto o bloqueio do timeout permanecer armado nas chamadas seguintes,
    // garante que nenhum trecho interprete o robô como ainda em fuga da linha.
    if (fugaLinhaBloqueadaPorTimeout) {
        fugindoLinhaAgora = false;
    }

    // -------------------------------------------------------------------------
    // ESTRATÉGIA NORMAL DO ATACANTE
    // -------------------------------------------------------------------------
    if (obterAnguloIrDisponivel(anguloIrAtual)) {

            if (irNaFaixaFrontal(anguloIrAtual)) {

                // Memoriza o movimento estratégico para a próxima leitura da linha.
                ultimoAnguloMovimentoAtaqueCmd = 0.0f;
                ultimoMovimentoAtaqueCmdValido = true;

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

            // Guarda a direção REAL pedida pela estratégia de bola. Na próxima
            // detecção de linha isso permite saber se o robô estava avançando
            // contra a região traseira quando tocou a faixa.
            ultimoAnguloMovimentoAtaqueCmd = anguloMovimento;
            ultimoMovimentoAtaqueCmdValido = true;

// Alterada a função de movimento do atacante de seguirDirecaoComGiroLaterais para seguirDirecaoComGiro
            seguirDirecaoComGiro(
                anguloMovimento,
                velo,
                cmdGiro
            );
            }

    } else {

        // Sem translação de ataque neste ciclo; não usa uma direção antiga para
        // classificar uma futura detecção traseira como movimento para trás.
        ultimoMovimentoAtaqueCmdValido = false;

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