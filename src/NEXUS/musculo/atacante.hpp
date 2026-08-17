#ifndef ATACANTE_HPP
#define ATACANTE_HPP


// ============================================================
// FUNÇÃO PRINCIPAL
// ============================================================

void atacante();


// ============================================================
// FUNÇÕES UTILIZADAS DIRETAMENTE PELO ATACANTE
// ============================================================

float calcularErroReferenciaBussola();

float normalizarErro180(float erro);

float PIDZIMBUSSOLANOVINHA(float erro);

float mapearAnguloBolaParaMovimento(float anguloBolaGraus);

void resetControleGolCamera();

void moverFrenteComGiroParaGol(int velocidade);

bool obterAnguloIrDisponivel(float &anguloBolaGraus);

bool irNaFaixaFrontal(float anguloBolaGraus);

int aplicarFreioUltrassonicoAtacante(int velocidadeDesejada);



// ============================================================
// VARIÁVEIS GLOBAIS
// DEFINIDAS NO musculo.cpp
// ============================================================

extern bool fugindoLinhaAgora;

extern float anguloFugaLinhaCmd;

extern bool linhaDetectada;

extern float anguloLinhaPe;

extern const int VELOCIDADE_FUGA_LINHA;


#endif