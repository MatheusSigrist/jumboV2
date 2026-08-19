#ifndef DEFENSOR_HPP
#define DEFENSOR_HPP

#include <Arduino.h>

// ============================================================
// FUNÇÃO PRINCIPAL
// ============================================================

void defensor();


// ============================================================
// CONTROLE PRINCIPAL DO DEFENSOR
// ============================================================

bool executarAvancoFrontalTemporizadoDefensor(
    unsigned long agora,
    float &vetorXSuave,
    float &vetorYSuave,
    float &cmdGiroSuave
);


// ============================================================
// CONTROLE DE VELOCIDADE LATERAL POR IR
// ============================================================

int calcularVelocidadeLateralDefensorPorIr(
    float anguloBolaGraus
);


// ============================================================
// PID DE GIRO DO GOLEIRO PELAS LINHAS
// ============================================================

void resetPidLinhaGoleiro();

int calcularSaidaPidLinhaGoleiro(
    float erroGraus
);


// ============================================================
// FUNÇÕES AUXILIARES DE MOVIMENTO DO DEFENSOR
// ============================================================

float suavizarDefensor(
    float atual,
    float alvo,
    float fator
);

float aplicarDeadzoneDefensor(
    float valor,
    float deadzone
);

float calcularMagnitudeVetorDefensor(
    float vetorX,
    float vetorY
);

float calcularAnguloVetorDefensor(
    float vetorX,
    float vetorY
);

int sinalErroDefensor(
    float erro,
    float toleranciaZero
);

int calcularGiroDefensor(
    float erroA,
    float erroB,
    float toleranciaIgual,
    int giroMaximo
);

int calcularVelocidadeLinhaDefensor(
    float erroA,
    float erroB,
    bool temZonaA,
    bool temZonaB,
    float toleranciaZero,
    float erroMaxRef,
    int velocidadeMinima,
    int velocidadeMaxima
);


// ============================================================
// CONTROLE DAS ZONAS DE LINHA
// ============================================================

float calcularVetorAtracaoLinha(
    float anguloA,
    float anguloB
);


// ============================================================
// RETORNO AO GOL PELA BÚSSOLA
// ============================================================

float comporAnguloRetornoBussolaComUltraLaterais(
    float anguloRetornoBase,
    bool ultrasRecentes
);


// ============================================================
// CONTROLE ULTRASSÔNICO DO DEFENSOR
// ============================================================

// Placeholder para futura lógica específica
// de ultrassônicos do defensor.
bool ultrassonico_defensor();


// ============================================================
// VARIÁVEIS GLOBAIS
// DEFINIDAS EM OUTROS MÓDULOS
// ============================================================

// ------------------------------------------------------------
// Estado geral do alinhamento / fuga
// ------------------------------------------------------------

extern bool alinhandoAgora;
extern bool fugindoLinhaAgora;
extern float erroAlinhamentoGraus;


// ------------------------------------------------------------
// Linha - Zona A
// ------------------------------------------------------------

extern bool linhaZonaAValida;
extern float anguloLinhaZonaA;

extern float ultimoAnguloLinhaZonaAValido;
extern unsigned long ultimoRxLinhaZonaAMs;


// ------------------------------------------------------------
// Linha - Zona B
// ------------------------------------------------------------

extern bool linhaZonaBValida;
extern float anguloLinhaZonaB;

extern float ultimoAnguloLinhaZonaBValido;
extern unsigned long ultimoRxLinhaZonaBMs;


// ------------------------------------------------------------
// Bússola
// ------------------------------------------------------------

extern bool bussolaTemReferenciaValida();

extern float calcularAnguloRetornoGolPorBussola();

extern float calcularErroReferenciaBussola();

extern void resetPidBussola();

extern int calcularSaidaPidBussola(
    float erroGraus
);


// ------------------------------------------------------------
// IR
// ------------------------------------------------------------

extern bool irDetectado;
extern float anguloIr;

extern bool obterAnguloIrDisponivel(
    float &anguloBolaGraus
);


// ------------------------------------------------------------
// Câmera
// ------------------------------------------------------------

extern bool cameraLerGolSelecionadoMenu(
    int16_t &anguloGol,
    uint16_t &distanciaGol
);

extern bool cameraTemGolSelecionadoValido(
    int16_t &anguloGol
);

extern bool cameraTemBolaValida();

// ------------------------------------------------------------
// Ultrassônicos
// ------------------------------------------------------------

extern bool ultrasValidos;

extern unsigned long ultimoRxUltraMs;

extern float ultraDcm;
extern float ultraEcm;
extern float ultraFcm;
extern float ultraTcm;


// ------------------------------------------------------------
// Configuração de gol
// ------------------------------------------------------------

extern bool corGolAzul;


// ============================================================
// CONSTANTES / VARIÁVEIS EXTERNAS DE CONTROLE
// ============================================================

extern const int VELOCIDADE_GIRO_ALINHAMENTO;
extern const int SINAL_GIRO_PID;
extern const float TOLERANCIA_ALINHAMENTO_GRAUS;

extern const unsigned long TIMEOUT_ULTRA_MS;

extern const unsigned long RETENCAO_ZONA_LINHA_DEFENSOR_MS;

extern int velocidade_maxima;


// ============================================================
// FUNÇÕES GERAIS UTILIZADAS PELO DEFENSOR
// DEFINIDAS EM OUTROS MÓDULOS
// ============================================================

extern float normalizarAngulo360(float angulo);

extern float normalizarErro180(float erro);

extern float mapearFaixaClamped(
    float valor,
    float entradaMin,
    float entradaMax,
    float saidaMin,
    float saidaMax
);


// ============================================================
// FUNÇÕES DE MOVIMENTAÇÃO
// DEFINIDAS EM motores_movimentacao.cpp
// ============================================================

void girarNoEixo(int velocidade);

void seguirDirecaoPorAngulo(
    float angulo,
    int velocidade
);

void seguirDirecaoComGiro(
    float angulo,
    int velocidade,
    int cmdGiro
);


// ============================================================
// PARÂMETROS DO DEFENSOR
// ============================================================

// ------------------------------------------------------------
// Referências das zonas de linha
// ------------------------------------------------------------

extern const float DEFENSOR_REFERENCIA_ZONA_A;
extern const float DEFENSOR_REFERENCIA_ZONA_B;


// ------------------------------------------------------------
// Tolerâncias angulares
// ------------------------------------------------------------

extern const float DEFENSOR_TOLERANCIA_GIRO_GRAUS;
extern const float DEFENSOR_TOLERANCIA_ALINHAMENTO_BOLA_GRAUS;


// ------------------------------------------------------------
// Pesos da bola
// ------------------------------------------------------------

extern const float DEFENSOR_PESO_MIN_BOLA;
extern const float DEFENSOR_PESO_MAX_BOLA;


// ------------------------------------------------------------
// Ultrassônicos
// ------------------------------------------------------------

extern const float DEFENSOR_ULTRA_LATERAL_ATIVO_CM;
extern const float DEFENSOR_ULTRA_LATERAL_CRITICO_CM;
extern const float DEFENSOR_ULTRA_LATERAL_CONFIRMADOR_CM;

extern const float DEFENSOR_ULTRA_FRENTE_LIMITE_CM;
extern const float DEFENSOR_ULTRA_TRAS_LIMITE_CM;

extern const float DEFENSOR_ULTRA_FRONTAL_CONFIRMADOR_CM;

extern const float DEFENSOR_ULTRA_PROFUNDIDADE_MAX_CM;
extern const float DEFENSOR_ULTRA_PROFUNDIDADE_MIN_CM;

extern const float DEFENSOR_PESO_MAX_ULTRA;
extern const float DEFENSOR_PESO_MAX_ULTRA_PROFUNDIDADE;


// ------------------------------------------------------------
// Deadzones
// ------------------------------------------------------------

extern const float DEFENSOR_DEADZONE_VETOR;
extern const float DEFENSOR_DEADZONE_GIRO;


// ------------------------------------------------------------
// Velocidades
// ------------------------------------------------------------

extern const float DEFENSOR_VELOCIDADE_MIN_PWM;
extern const float DEFENSOR_VELOCIDADE_RETORNO_GOL_PWM;


// ------------------------------------------------------------
// Suavização
// ------------------------------------------------------------

extern const float DEFENSOR_SUAVIZACAO_VETOR;
extern const float DEFENSOR_SUAVIZACAO_GIRO;


// ------------------------------------------------------------
// Avanço frontal temporizado
// ------------------------------------------------------------

extern const float DEFENSOR_TOLERANCIA_IR_FRONTAL_GRAUS;

extern const int DEFENSOR_VELOCIDADE_AVANCO_IR_FRONTAL_PWM;

extern const unsigned long DEFENSOR_TEMPO_GATILHO_IR_FRONTAL_MS;

extern const unsigned long DEFENSOR_TEMPO_AVANCO_IR_FRONTAL_MS;


#endif