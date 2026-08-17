#include <Arduino.h>
#include "atacante.hpp"
#include "motores_movimentacao.hpp"












void atacante() {
    int velo = 255;

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

    if (sairDaLinha(
            linhaDetectada,
            anguloLinhaPe,
            VELOCIDADE_FUGA_LINHA,
            &anguloFuga)) {

        fugindoLinhaAgora = true;
        anguloFugaLinhaCmd = anguloFuga;
        return;

    } else {

        if (obterAnguloIrDisponivel(anguloIrAtual)) {

            if (irNaFaixaFrontal(anguloIrAtual)) {

                moverFrenteComGiroParaGol(velo);

            } else {
                 

                resetControleGolCamera();

                float anguloMovimento =
                    mapearAnguloBolaParaMovimento(anguloIrAtual);

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
    }

    return;
}