


// =============================================================================
// MUSCULO.CPP — Placa de Atuadores e Estratégias do Cronos
// =============================================================================
// Responsabilidades:
//   • Controle de movimento (motores DC via ponte H)
//   • Menu de navegação no display OLED
//   • Calibração de gol (cor), bússola e IR
//   • Alinhamento e correção angular via PID
//   • Lógica do kicker (solenoide)
//   • Estratégias de ATACANTE e DEFENSOR
//
// Entradas:
//   • Mensagens da Cabeça via Serial1 (IR, bússola, linha, ultrasson, câmera, botões)
//   • Sensores de linha/gol e botões físicos
//
// Saídas:
//   • Comandos PWM para 4 motores (rodas omnidirecionais/mecanum)
//   • Pulso do kicker (solenoide)
//   • Telas de status no display OLED 128x64
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "motores_movimentacao.hpp"
#include "display/ihm_display.hpp"


// =============================================================================
// SECAO 1 — DEFINICOES GERAIS DE HARDWARE
// =============================================================================

// --- Papel atual do robô recebido da Cabeça ---
// true = atacante | false = defensor
bool papelAtacante = false;
bool papelAtacanteAnterior = false;  // Detecta mudanças de papel entre ciclos

// --- Pinos da Serial com a Cabeça ---
#define RX_CABECA 17
#define TX_CABECA 18

// --- Pinos do barramento I2C ---
#define SDA_PIN 8
#define SCL_PIN 9

// --- Pino e temporização do solenoide (kicker) ---
constexpr uint8_t  KICKER_PIN = 21;
constexpr unsigned long KICK_PULSE_MS = 100;   // Duração do pulso de chute (ms)
constexpr unsigned long KICK_INTERVAL_MS = 1000;  // Intervalo mínimo entre chutes (ms)

// =============================================================================
// SECAO 2 — VARIAVEIS DE COMUNICACAO E ESTADO GERAL
// =============================================================================

// --- Estado da comunicação com a Cabeça ---
bool   comunicacaoCabecaOK  = false;
String bufferSerial         = "";
String mensagemBotao        = "NENHUM";

// --- Temporização de handshake e timeout ---
unsigned long ultimoEnvioOi    = 0;
unsigned long ultimoRxCabeca   = 0;
unsigned long mostrarStatusAte = 0;

// --- Status das placas auxiliares (Olho e Pé) ---
bool olhoOK = false;
bool peOK   = false;

// --- Estado do jogo enviado à Cabeça ---
bool          estadoJogoCabecaEnviado         = false;
unsigned long ultimoEnvioEstadoJogoCabecaMs   = 0;

// --- Temporização e limites gerais de comunicação ---
const unsigned long INTERVALO_OI_MS               = 1000;
const unsigned long TIMEOUT_COM_MS                = 5000;
const unsigned long INTERVALO_ENVIO_ESTADO_JOGO_MS = 500;

// --- Botão de ação longa (pino físico) ---
const uint8_t BOTAO_MEIO_LONGO = 23;


// =============================================================================
// SECAO 3 — VARIAVEIS DE SENSORES E TELEMETRIA (recebidos da Cabeça)
// =============================================================================

// --- Bússola ---
bool   bussolaValida       = false;
int    headingBussolaTeste = 0;    // Leitura atual da bússola (°)
int    headingBussolaSalvo = 0;    // Referência calibrada salva em EEPROM
unsigned long ultimoRxBussolaMs = 0;
const unsigned long TIMEOUT_BUSSOLA_MS = 800;

// --- Sensor IR (detecção de bola) ---
float  anguloIr              = -1.0;    // Ângulo atual do IR (° ou -1 se sem bola)
bool   irDetectado           = false;
float  ultimoAnguloIrValido  = -1.0f;
unsigned long ultimoIrValidoMs = 0;

// --- Câmera (bola + 2 gols: azul e amarelo) ---
bool   cameraOK          = false;   // Câmera se comunicando com o Olho
bool   cameraDadosValidos = false;
unsigned long ultimoRxCameraMs              = 0;
unsigned long cameraUltimaVezBolaDetetadaMs = 0;
unsigned long cameraSemBolaBrutaInicioMs    = 0;
const unsigned long TIMEOUT_CAMERA_MS       = 1000;

int16_t  cameraBallAngle   = -999;
uint16_t cameraBallDist    = 0;
int16_t  cameraBlueAngle   = -999;
uint16_t cameraBlueDist    = 0;
int16_t  cameraYellowAngle = -999;
uint16_t cameraYellowDist  = 0;

// Gol selecionado (calculado a partir de corGolAzul)
int16_t  cameraGolSelecionadoAngle = -999;
uint16_t cameraGolSelecionadoDist  = 0;
bool     cameraGolSelecionadoValido = false;
bool     cameraGolSelecionadoAzul   = false;

// Buffer circular de ângulos da câmera para filtragem
const uint8_t CAMERA_BOLA_BUFFER_TAM                  = 3;
const float   CAMERA_BOLA_PESO_PREVISAO                = 0.65f;
const float   CAMERA_BOLA_DELTA_PREVISAO_MAX_GRAUS     = 35.0f;
float    cameraBallBufferAngulos[CAMERA_BOLA_BUFFER_TAM] = {0.0f, 0.0f, 0.0f};
uint8_t  cameraBallBufferIndice    = 0;
uint8_t  cameraBallBufferQuantidade = 0;

// --- Ultrassônicos locais (D=direita, E=esquerda, F=frente, T=trás) ---
float ultraDcm = -1.0f;
float ultraEcm = -1.0f;
float ultraFcm = -1.0f;
float ultraTcm = -1.0f;
bool  ultrasValidos    = false;
unsigned long ultimoRxUltraMs = 0;
const unsigned long TIMEOUT_ULTRA_MS = 1000;

// --- Ultrassônicos remotos (do outro robô, via ESP-NOW) ---
float ultraRemotoDcm = -1.0f;
float ultraRemotoEcm = -1.0f;
float ultraRemotoFcm = -1.0f;
float ultraRemotoTcm = -1.0f;
bool  ultrasRemotosValidos    = false;
unsigned long ultimoRxUltraRemotoMs = 0;

// --- Sensores brutos de linha (LDR do Pé) ---
int sensorBruto1  = -1;
int sensorBruto9  = -1;
int sensorBruto17 = -1;
int sensorBruto25 = -1;
unsigned long ultimoRxSensoresBrutosMs   = 0;
unsigned long ultimoReqSensoresBrutosMs  = 0;
const unsigned long TIMEOUT_SENSORES_BRUTOS_MS        = 1200;
const unsigned long INTERVALO_REQ_SENSORES_BRUTOS_MS  = 250;

// --- Limiar de linha (compartilhado entre Músculo e Pé) ---
int  limiarLinhaEditado          = 2500;
bool limiarLinhaSincronizado     = false;
unsigned long ultimoReqLimiarLinhaMs     = 0;
unsigned long entradaTelaLimiarMs        = 0;
const int  LIMIAR_LINHA_MIN              = 100;
const int  LIMIAR_LINHA_MAX              = 4000;
const int  LIMIAR_LINHA_PASSO            = 100;
const unsigned long INTERVALO_REQ_LIMIAR_LINHA_MS = 400;

// --- Linha (atacante: ângulo único; defensor: zonas A e B) ---
float  anguloLinhaPe        = -1.0f;
bool   linhaDetectada       = false;
float  anguloLinhaZonaA     = -1.0f;
float  anguloLinhaZonaB     = -1.0f;
bool   linhaZonaAValida     = false;
bool   linhaZonaBValida     = false;
unsigned long ultimoRxLinhaMs             = 0;
unsigned long ultimoComandoLinhaMs        = 0;
float  ultimoAnguloLinhaValido            = -1.0f;
float  ultimoAnguloLinhaZonaAValido       = -1.0f;
float  ultimoAnguloLinhaZonaBValido       = -1.0f;
unsigned long ultimoRxLinhaZonaAMs        = 0;
unsigned long ultimoRxLinhaZonaBMs        = 0;
const unsigned long TIMEOUT_LINHA_MS      = 50;

// --- Cor do gol de referência e envio pendente à Cabeça ---
bool corGolAzul             = false;
bool corGolPendenteEnvio    = true;
unsigned long ultimoEnvioCorGolMs         = 0;
const unsigned long INTERVALO_ENVIO_COR_GOL_MS = 500;
unsigned long ultimoEnvioRefBussolaMs = 0;
const unsigned long INTERVALO_ENVIO_REF_BUSSOLA_MS = 500;

// --- Kicker ---
bool kickerRecebido         = false;
bool kickerAtivado          = false;   // true quando chave acionada (valor 0 vindo da Cabeça)
bool pulsoKickerAtivo       = false;
bool pulsoKickerManualAtivo = false;
bool pedidoChuteManual      = false;
unsigned long inicioPulsoKickerMs   = 0;
unsigned long ultimoDisparoKickerMs = 0;

// --- ESP-NOW (comunicação entre as Cabeças) ---
bool espnowOK = false;
bool sozinho  = true;
unsigned long ultimoRxEspnowMs = 0;

// Papel automático: quem está mais perto da bola assume o ataque
constexpr bool  PAPEL_AUTO_DESEMPATE_ATACANTE = true;
constexpr float PAPEL_AUTO_JANELA_EMPATE_CM   = 0.5f;


// =============================================================================
// SECAO 4 — EEPROM: ENDERECOS E FUNCOES DE PERSISTENCIA
// =============================================================================

const int EEPROM_SIZE             = 64;
const int EEPROM_ADDR_BUSSOLA     = 0;
const int EEPROM_ADDR_PAPEL_CONFIG = EEPROM_ADDR_BUSSOLA + (int)sizeof(int);
const int EEPROM_ADDR_COR_GOL     = EEPROM_ADDR_PAPEL_CONFIG + (int)sizeof(uint8_t);

// Enums de configuração de papel e cor de gol (persistidos em EEPROM)
enum PapelConfigurado { PAPEL_CONFIG_ATACANTE, PAPEL_CONFIG_DEFENSOR, PAPEL_CONFIG_AUTO };
int papelConfiguradoMenu = PAPEL_CONFIG_AUTO;

// Aplica o papel configurado localmente (ignora a Cabeça quando fixo)
void aplicarPapelConfiguradoLocal() {
  if (papelConfiguradoMenu == PAPEL_CONFIG_ATACANTE) {
    papelAtacante = true;
    papelAtacanteAnterior = true;
  } else if (papelConfiguradoMenu == PAPEL_CONFIG_DEFENSOR) {
    papelAtacante = false;
    papelAtacanteAnterior = false;
  }
}

// Salva papel configurado na EEPROM; retorna true se OK
bool salvarPapelConfiguradoEEPROM() {
  uint8_t papelSalvo = (uint8_t)papelConfiguradoMenu;
  EEPROM.put(EEPROM_ADDR_PAPEL_CONFIG, papelSalvo);
  return EEPROM.commit();
}

// Salva cor do gol na EEPROM; retorna true se OK
bool salvarCorGolEEPROM() {
  uint8_t corGolSalva = corGolAzul ? 1 : 0;
  EEPROM.put(EEPROM_ADDR_COR_GOL, corGolSalva);
  return EEPROM.commit();
}

// Carrega papel configurado da EEPROM e aplica localmente
void carregarPapelConfiguradoEEPROM() {
  uint8_t papelSalvo = (uint8_t)PAPEL_CONFIG_AUTO;
  EEPROM.get(EEPROM_ADDR_PAPEL_CONFIG, papelSalvo);
  if (papelSalvo > (uint8_t)PAPEL_CONFIG_AUTO) {
    papelSalvo = (uint8_t)PAPEL_CONFIG_AUTO;
  }
  papelConfiguradoMenu = (PapelConfigurado)papelSalvo;
  aplicarPapelConfiguradoLocal();
}

// Carrega cor do gol da EEPROM
void carregarCorGolEEPROM() {
  uint8_t corGolSalva = 0;
  EEPROM.get(EEPROM_ADDR_COR_GOL, corGolSalva);
  if (corGolSalva > 1) {
    corGolSalva = 0;
  }
  corGolAzul = (corGolSalva == 1);
}


// =============================================================================
// SECAO 5 — ESTADOS DE INTERFACE (MENU, CALIBRACAO, FUNCAO, INICIAR)
// =============================================================================

enum Estado { MENU, CALIBRACAO, FUNCAO, INICIAR };
int estadoAtual   = MENU;
int    itemSelecionado = 0;

// Submenus da calibração
enum SubMenuCalibracao {
  SUBMENU_PRINCIPAL,
  SUBMENU_GOL,
  SUBMENU_BUSSOLA,
  SUBMENU_IR,
  SUBMENU_ULTRA,
  SUBMENU_CAMERA,
  SUBMENU_ESPNOW
};
int subMenuCalibracao = SUBMENU_PRINCIPAL;
int itemSubMenu = 0;

// Submenus de função
enum SubMenuFuncao {
  SUBFUNCAO_PRINCIPAL,
  SUBFUNCAO_PAPEIS,
  SUBFUNCAO_POSICIONAMENTO,
  SUBFUNCAO_SENSORES,
  SUBFUNCAO_LIMIAR_LINHA,
  SUBFUNCAO_KICKER
};
int subMenuFuncao     = SUBFUNCAO_PRINCIPAL;
int           itemSubMenuFuncao = 0;

bool bussolaTemReferenciaValida();
float calcularErroReferenciaBussola();

// Posicionamento por coordenadas enviado via HTTPS (repasse da Cabeca)
bool posicionamentoAlvoAtivo = false;
float posicionamentoAlvoXcm = 91.0f;
float posicionamentoAlvoYcm = 121.5f;
float posicionamentoAtualXcm = 91.0f;
float posicionamentoAtualYcm = 121.5f;
float posicionamentoConfianca = 0.0f;
float posicionamentoAnguloAlvoGraus = 0.0f;
unsigned long posicionamentoUltimaEstimativaMs = 0;


// =============================================================================
// SECAO 6 — CONTROLE DE MOVIMENTO: PARAMETROS GERAIS
// =============================================================================

// --- Velocidade máxima global dos motores (0–255) ---
// *** AJUSTE AQUI para alterar a velocidade máxima do robô ***
const int velocidade_maxima = 255;

// --- Habilita/desabilita o movimento baseado em bola (teste) ---
const bool MOVIMENTO_BOLA_HABILITADO = false;

// --- Rampa de aceleração PWM (suaviza variações bruscas de comando) ---
const int PASSO_RAMPA_PWM = 16;

// --- Configuração de giro no eixo (sentido e velocidade padrão) ---
// *** AJUSTE AQUI para corrigir o sentido ou velocidade de giro de alinhamento ***
#define VELOCIDADE_GIRO  180
#define SINAL_GIRO       -1

// --- Ganho de mistura translação + rotação ---
const float GANHO_GIRO_MISTO = 0.7f;

// --- Temporização de transição angular do IR (rampa suave entre faixas) ---
const unsigned long TRANSICAO_ANGULO_IR_MIN_MS    = 50;
const unsigned long TRANSICAO_ANGULO_IR_MAX_MS    = 100;
const float         TRANSICAO_ANGULO_IR_MS_POR_GRAU = 2.0f;
const float         PASSO_ANGULO_IR_GRAUS          = 5.0f;

// Estado interno da rampa angular do IR
float        anguloIrSuaveAtual        = 0.0f;
float        anguloIrSuaveInicio       = 0.0f;
float        anguloIrSuaveAlvo         = 0.0f;
unsigned long inicioTransicaoIrMs       = 0;
unsigned long duracaoTransicaoIrMs      = TRANSICAO_ANGULO_IR_MIN_MS;
bool         anguloIrSuaveInicializado  = false;

// --- Variáveis de estado de fuga de linha e alinhamento ---
float  erroGolGraus       = 0.0f;   // Erro angular atual em relação ao gol
bool   golDetectado       = false;
uint16_t golPixels        = 0;
float  erroAlinhamentoGraus = 0.0f; // Erro de alinhamento com o gol (°)
bool   alinhandoAgora     = false;  // true quando executando giro de alinhamento
bool   fugindoLinhaAgora  = false;  // true quando executando fuga de linha
float  anguloFugaLinhaCmd = 0.0f;   // Ângulo do comando de fuga enviado aos motores
const int VELOCIDADE_FUGA_LINHA = 255;

const float CAMPO_LARGURA_CM = 182.0f;
const float CAMPO_ALTURA_CM = 243.0f;
const float ROBO_DIAMETRO_CAMPO_CM = 21.0f;
const float ROBO_RAIO_CAMPO_CM = ROBO_DIAMETRO_CAMPO_CM * 0.5f;
const float POS_COMP_TOL_CM = 22.0f;
const float POS_TAU_FAST = 0.26f;
const float POS_TAU_SLOW = 0.48f;
const float POS_TOLERANCIA_CM = 10.0f;
const float POS_TOLERANCIA_STOP_BRUTA_CM = 13.0f;
const float POS_JUMP_MAX_CM = 35.0f;
const int POS_VELOCIDADE_PWM = 150;
const int POS_VELOCIDADE_PWM_MIN = 100;
const float POS_DIST_RAMP_CM = 80.0f;
const unsigned long POS_STOP_CONFIRM_MS = 180;
const float POS_TOLERANCIA_GIRO_GRAUS = 6.0f;
const float POS_GIRO_SO_EIXO_GRAUS = 35.0f;

// --- Tolerâncias de alinhamento ---
// *** AJUSTE AQUI para afinar a janela de alinhamento com o gol ***
const float TOLERANCIA_ALINHAMENTO_GRAUS   = 8.0f;
const float JANELA_FUZZY_ALINHAMENTO_GRAUS = 30.0f;
const int   VELOCIDADE_GIRO_ALINHAMENTO    = VELOCIDADE_GIRO;
const int   SINAL_GIRO_PID                 = SINAL_GIRO;

// --- Tempo de retenção de dados de linha e zona (evita perda por frame único) ---
const unsigned long RETENCAO_FUGA_LINHA_MS          = 250;
const unsigned long RETENCAO_ZONA_LINHA_DEFENSOR_MS  = 300;
const unsigned long TEMPO_CAMERA_SEM_IR_PARA_IGNORAR_LINHA_MS = 1000;


// =============================================================================
// SECAO 7 — PID GERAL DE ALINHAMENTO (BUSSOLA)
//            Usado por ATACANTE e DEFENSOR para alinhar com o gol via câmera
// =============================================================================

// Ganhos e saturações do PID de alinhamento bússola/câmera
// *** AJUSTE AQUI para afinação do giro de alinhamento com o gol ***
const float PID_BUS_KP            = 0.8f;
const float PID_BUS_KI            = 0.01f;
const float PID_BUS_KD            = 0.5f;
const float PID_BUS_INTEGRAL_MAX  = 120.0f;
const int   PID_BUS_SAIDA_MIN     = 30;
const int   PID_BUS_SAIDA_MAX     = 180;

// Estado interno do PID de bússola (entre iterações do loop)
float        pidBusIntegral      = 0.0f;
float        pidBusErroAnterior  = 0.0f;
unsigned long pidBusUltimoMs     = 0;

// --- Zera o PID de bússola ---
void resetPidBussola() {
  pidBusIntegral     = 0.0f;
  pidBusErroAnterior = 0.0f;
  pidBusUltimoMs     = 0;
}

// --- Calcula saída assinada do PID de bússola com anti-windup ---
int calcularSaidaPidBussola(float erroGraus) {
  unsigned long agora = millis();
  float dt = 0.02f;

  // Calcula dt real e limita extremos para robustez
  if (pidBusUltimoMs != 0) {
    dt = (agora - pidBusUltimoMs) / 1000.0f;
    if (dt < 0.005f) dt = 0.005f;
    if (dt > 0.2f)   dt = 0.2f;
  }
  pidBusUltimoMs = agora;

  // Termo integral com anti-windup por saturação simples
  pidBusIntegral += erroGraus * dt;
  if (pidBusIntegral >  PID_BUS_INTEGRAL_MAX) pidBusIntegral =  PID_BUS_INTEGRAL_MAX;
  if (pidBusIntegral < -PID_BUS_INTEGRAL_MAX) pidBusIntegral = -PID_BUS_INTEGRAL_MAX;

  // Derivada discreta
  float derivada = (erroGraus - pidBusErroAnterior) / dt;
  pidBusErroAnterior = erroGraus;

  // Ação de controle, módulo e saturações
  float u    = PID_BUS_KP * erroGraus + PID_BUS_KI * pidBusIntegral + PID_BUS_KD * derivada;
  int   saida = (int)fabsf(u);
  if (saida < PID_BUS_SAIDA_MIN)            saida = PID_BUS_SAIDA_MIN;
  if (saida > PID_BUS_SAIDA_MAX)            saida = PID_BUS_SAIDA_MAX;
  if (saida > VELOCIDADE_GIRO_ALINHAMENTO)  saida = VELOCIDADE_GIRO_ALINHAMENTO;

  // Sinal preserva o sentido do erro angular
  return (u >= 0.0f) ? saida : -saida;
}


// =============================================================================
// SECAO 8 — FUNCOES MATEMATICAS UTILITARIAS
// =============================================================================

// Normaliza erro angular para a faixa [-180, 180]
float normalizarErro180(float erro) {
  while (erro >  180.0f) erro -= 360.0f;
  while (erro < -180.0f) erro += 360.0f;
  return erro;
}

// Normaliza ângulo absoluto para a faixa [0, 360)
float normalizarAngulo360(float ang) {
  while (ang >= 360.0f) ang -= 360.0f;
  while (ang <    0.0f) ang += 360.0f;
  return ang;
}

// Quantiza ângulo em passos fixos para transições curtas e previsíveis
float quantizarAnguloPasso(float anguloGraus, float passoGraus) {
  if (passoGraus <= 0.0f) {
    return normalizarAngulo360(anguloGraus);
  }
  float ang       = normalizarAngulo360(anguloGraus);
  float quantizado = roundf(ang / passoGraus) * passoGraus;
  return normalizarAngulo360(quantizado);
}

// Valida payload numérico (evita toFloat() silencioso em 0)
bool payloadNumericoValido(const String &texto) {
  if (texto.length() == 0) {
    return false;
  }
  bool encontrouDigito = false;
  bool encontrouPonto  = false;
  for (size_t i = 0; i < texto.length(); i++) {
    char c = texto.charAt(i);
    if (c >= '0' && c <= '9')             { encontrouDigito = true; continue; }
    if (c == '.' && !encontrouPonto)       { encontrouPonto  = true; continue; }
    if ((c == '+' || c == '-') && i == 0) { continue; }
    return false;
  }
  return encontrouDigito;
}

// Mapeia um valor de entrada para saída, com clamping nas bordas
float mapearFaixaClamped(float valor, float entradaMin, float entradaMax,
                         float saidaMin, float saidaMax) {
  float denominador = entradaMax - entradaMin;
  if (fabsf(denominador) < 0.0001f) {
    return saidaMin;
  }
  float proporcao = (valor - entradaMin) / denominador;
  if (proporcao < 0.0f) proporcao = 0.0f;
  if (proporcao > 1.0f) proporcao = 1.0f;
  return saidaMin + ((saidaMax - saidaMin) * proporcao);
}

bool posicionamentoModuloArmado() {
  return (estadoAtual == FUNCAO) && (subMenuFuncao == SUBFUNCAO_POSICIONAMENTO);
}

void cancelarPosicionamentoAlvo(bool manterMensagem = false) {
  posicionamentoAlvoAtivo = false;
  resetPidBussola();
  pararMotores();
  if (!manterMensagem) {
    mensagemBotao = "POS CANCELADO";
    mostrarStatusAte = millis() + 1800;
  }
}

bool ultraValidoParaPosicao(float v) {
  return isfinite(v) && v > 1.0f && v < 350.0f;
}

float filtroComplementarPos(float anterior, float medicao, float confianca, float dtSec) {
  float tau = (confianca >= 0.8f) ? POS_TAU_FAST : POS_TAU_SLOW;
  if (tau < 0.02f) tau = 0.02f;
  float alpha = expf(-dtSec / tau);
  alpha = constrain(alpha + ((1.0f - confianca) * 0.18f), 0.08f, 0.96f);
  return (alpha * anterior) + ((1.0f - alpha) * medicao);
}

struct EixoPosResultado {
  float valor;
  float medida;
  float confianca;
  float residual;
};

EixoPosResultado estimarEixoPosicao(float leituraA,
                                    float leituraB,
                                    float campo,
                                    float ultimo,
                                    float dtSec) {
  const bool temA = ultraValidoParaPosicao(leituraA);
  const bool temB = ultraValidoParaPosicao(leituraB);
  const float minimo = ROBO_RAIO_CAMPO_CM;
  const float maximo = campo - ROBO_RAIO_CAMPO_CM;
  const float utilizavel = campo - (2.0f * ROBO_RAIO_CAMPO_CM);

  float medida = ultimo;
  float confianca = 0.05f;
  float residual = 999.0f;

  if (temA && temB) {
    float diretoA = constrain(leituraA + ROBO_RAIO_CAMPO_CM, minimo, maximo);
    float diretoB = constrain(campo - (leituraB + ROBO_RAIO_CAMPO_CM), minimo, maximo);
    float soma = leituraA + leituraB;
    if (soma < 1.0f) soma = 1.0f;
    residual = fabsf((leituraA + leituraB + (2.0f * ROBO_RAIO_CAMPO_CM)) - campo);
    if (residual <= POS_COMP_TOL_CM) {
      medida = 0.5f * (diretoA + diretoB);
      confianca = 0.95f;
    } else {
      float fracA = constrain(leituraA / soma, 0.0f, 1.0f);
      float fracB = constrain(leituraB / soma, 0.0f, 1.0f);
      float mapaA = minimo + (fracA * utilizavel);
      float mapaB = minimo + ((1.0f - fracB) * utilizavel);
      medida = 0.5f * (mapaA + mapaB);
      confianca = 0.56f;
    }
  } else if (temA) {
    medida = constrain(leituraA + ROBO_RAIO_CAMPO_CM, minimo, maximo);
    confianca = 0.62f;
  } else if (temB) {
    medida = constrain(campo - (leituraB + ROBO_RAIO_CAMPO_CM), minimo, maximo);
    confianca = 0.62f;
  }

  float salto = medida - ultimo;
  float medidaLimitada = ultimo + constrain(salto, -POS_JUMP_MAX_CM, POS_JUMP_MAX_CM);
  float filtrada = filtroComplementarPos(ultimo, medidaLimitada, confianca, dtSec);

  EixoPosResultado out;
  out.valor = constrain(filtrada, minimo, maximo);
  out.medida = constrain(medida, minimo, maximo);
  out.confianca = confianca;
  out.residual = residual;
  return out;
}

bool estimarPosicaoCampoAtual(float &xCm, float &yCm, float &confianca) {
  unsigned long agora = millis();
  float dtSec = 0.08f;
  if (posicionamentoUltimaEstimativaMs > 0 && agora > posicionamentoUltimaEstimativaMs) {
    dtSec = (agora - posicionamentoUltimaEstimativaMs) / 1000.0f;
    if (dtSec < 0.05f) dtSec = 0.05f;
    if (dtSec > 0.8f) dtSec = 0.8f;
  }
  posicionamentoUltimaEstimativaMs = agora;

  EixoPosResultado eixoX = estimarEixoPosicao(ultraEcm, ultraDcm, CAMPO_LARGURA_CM,
                                              posicionamentoAtualXcm, dtSec);
  EixoPosResultado eixoY = estimarEixoPosicao(ultraFcm, ultraTcm, CAMPO_ALTURA_CM,
                                              posicionamentoAtualYcm, dtSec);

  xCm = eixoX.valor;
  yCm = eixoY.valor;
  confianca = 0.5f * (eixoX.confianca + eixoY.confianca);

  posicionamentoAtualXcm = xCm;
  posicionamentoAtualYcm = yCm;
  posicionamentoConfianca = confianca;
  return (eixoX.confianca > 0.1f) && (eixoY.confianca > 0.1f);
}

bool dentroToleranciaPos(float erroX, float erroY, float tolerancia) {
  return (fabsf(erroX) <= tolerancia) && (fabsf(erroY) <= tolerancia);
}

bool executarPosicionamentoAlvo() {
  static unsigned long dentroTolDesdeMs = 0;

  if (!posicionamentoModuloArmado()) {
    dentroTolDesdeMs = 0;
    if (posicionamentoAlvoAtivo) {
      cancelarPosicionamentoAlvo(true);
      mensagemBotao = "POS SAIU MODULO";
      mostrarStatusAte = millis() + 1800;
    }
    return false;
  }

  if (!posicionamentoAlvoAtivo) {
    dentroTolDesdeMs = 0;
    resetPidBussola();
    pararMotores();
    return true;
  }

  if (!ultrasValidos) {
    pararMotores();
    mensagemBotao = "POS AGUARDA SENS";
    mostrarStatusAte = millis() + 1200;
    return true;
  }

  float atualX = posicionamentoAtualXcm;
  float atualY = posicionamentoAtualYcm;
  float confianca = 0.0f;
  if (!estimarPosicaoCampoAtual(atualX, atualY, confianca)) {
    pararMotores();
    mensagemBotao = "POS SEM POSICAO";
    mostrarStatusAte = millis() + 1200;
    return true;
  }

  const float erroX = posicionamentoAlvoXcm - atualX;
  const float erroY = posicionamentoAlvoYcm - atualY;
  const float erroXBruto = posicionamentoAlvoXcm - constrain((ultraValidoParaPosicao(ultraEcm) ? (ultraEcm + ROBO_RAIO_CAMPO_CM) : atualX), ROBO_RAIO_CAMPO_CM, CAMPO_LARGURA_CM - ROBO_RAIO_CAMPO_CM);
  const float erroYBruto = posicionamentoAlvoYcm - constrain((ultraValidoParaPosicao(ultraFcm) ? (ultraFcm + ROBO_RAIO_CAMPO_CM) : atualY), ROBO_RAIO_CAMPO_CM, CAMPO_ALTURA_CM - ROBO_RAIO_CAMPO_CM);
  bool emTolerancia = dentroToleranciaPos(erroX, erroY, POS_TOLERANCIA_CM) ||
                      dentroToleranciaPos(erroXBruto, erroYBruto, POS_TOLERANCIA_STOP_BRUTA_CM);

  if (emTolerancia) {
    if (dentroTolDesdeMs == 0) {
      dentroTolDesdeMs = millis();
    }
    if ((millis() - dentroTolDesdeMs) >= POS_STOP_CONFIRM_MS) {
      cancelarPosicionamentoAlvo(true);
      mensagemBotao = "POSICAO ATINGIDA";
      mostrarStatusAte = millis() + 2200;
      dentroTolDesdeMs = 0;
      return true;
    }
  } else {
    dentroTolDesdeMs = 0;
  }

  posicionamentoAnguloAlvoGraus = normalizarAngulo360(atan2f(erroX, -erroY) * 180.0f / PI);
  float distAlvoCm = sqrtf((erroX * erroX) + (erroY * erroY));
  float velPosMapeada = mapearFaixaClamped(distAlvoCm,
                                           POS_TOLERANCIA_CM,
                                           POS_DIST_RAMP_CM,
                                           (float)POS_VELOCIDADE_PWM_MIN,
                                           (float)POS_VELOCIDADE_PWM);
  int velocidadePosPwm = constrain((int)roundf(velPosMapeada),
                                   POS_VELOCIDADE_PWM_MIN,
                                   POS_VELOCIDADE_PWM);

  int cmdGiroPos = 0;
  bool usarGiroPos = false;
  if (bussolaTemReferenciaValida()) {
    float erroBussolaPos = calcularErroReferenciaBussola();
    if (fabsf(erroBussolaPos) > 5.0f) {
      int cmdPidPos = calcularSaidaPidBussola(erroBussolaPos);
      cmdGiroPos = SINAL_GIRO_PID * cmdPidPos;
      usarGiroPos = true;
    }
  }

  float anguloMov = normalizarAngulo360(posicionamentoAnguloAlvoGraus);
  bool faixaNormal = (anguloMov >= 315.0f || anguloMov <= 45.0f ||
                      (anguloMov >= 135.0f && anguloMov <= 225.0f));

  if (faixaNormal) {
    if (usarGiroPos) {
      seguirDirecaoComGiro(anguloMov, velocidadePosPwm, cmdGiroPos);
    } else {
      seguirDirecaoPorAngulo(anguloMov, velocidadePosPwm);
    }
  } else {
    if (usarGiroPos) {
      seguirDirecaoComGiroLaterais(anguloMov, velocidadePosPwm, cmdGiroPos);
    } else {
      seguirDirecaoPorAngulo(anguloMov, velocidadePosPwm);
    }
  }
  return true;
}


// =============================================================================
// SECAO 10 — KICKER (SOLENOIDE)
// =============================================================================

// Solicita chute manual pelo submenu kicker
void solicitarChuteManualkicker() {
  pedidoChuteManual = true;
}

// Controla o pulso do kicker com tempo mínimo e intervalo entre disparos
void atualizarKicker() {
  unsigned long agora = millis();

  // Finaliza o pulso após o tempo mínimo energizado
  if (pulsoKickerAtivo && (agora - inicioPulsoKickerMs) >= KICK_PULSE_MS) {
    digitalWrite(KICKER_PIN, LOW);
    pulsoKickerAtivo       = false;
    pulsoKickerManualAtivo = false;
  }

  // Disparo manual solicitado pelo submenu kicker
  if (pedidoChuteManual && !pulsoKickerAtivo) {
    digitalWrite(KICKER_PIN, HIGH);
    inicioPulsoKickerMs    = agora;
    ultimoDisparoKickerMs  = agora;
    pulsoKickerAtivo       = true;
    pulsoKickerManualAtivo = true;
    pedidoChuteManual      = false;
    Serial.println("Chute manual");
    return;
  }
  pedidoChuteManual = false;

  // Só permite chute automático durante o jogo, com comunicação válida e chave acionada
  bool podeChutar = (estadoAtual == INICIAR) && comunicacaoCabecaOK && kickerRecebido && kickerAtivado;
  if (!podeChutar) {
    // Desliga imediatamente apenas o pulso automático
    if (pulsoKickerAtivo && !pulsoKickerManualAtivo) {
      digitalWrite(KICKER_PIN, LOW);
      pulsoKickerAtivo = false;
    }
    return;
  }

  // Dispara novo pulso respeitando o intervalo mínimo entre chutes
  if (!pulsoKickerAtivo && (agora - ultimoDisparoKickerMs) >= KICK_INTERVAL_MS) {
    digitalWrite(KICKER_PIN, HIGH);
    inicioPulsoKickerMs   = agora;
    ultimoDisparoKickerMs = agora;
    pulsoKickerAtivo      = true;
    pulsoKickerManualAtivo = false;
    Serial.println("Chutei");
  }
}


// =============================================================================
// SECAO 11 — FUNCOES DE GOL, BUSSOLA E ANGULO DE CAMPO (compartilhadas)
// =============================================================================

// Retorna true se o gol selecionado é o azul
bool golReferenciaAzulEfetiva() {
  return corGolAzul;
}

// Converte ângulo de gol para o referencial do defensor (inversão de 180°)
// Ex.: 30 -> 150, 200 -> 340
float converterAnguloGolParaDefensor(float anguloGolGraus) {
  float invertido = normalizarAngulo360(anguloGolGraus + 180.0f);
  return normalizarAngulo360(360.0f - invertido);
}

// Calcula erro de alinhamento com o gol no referencial do defensor (espelhado)
float calcularErroGolEspelhadoDefensor(float anguloGolGraus) {
  return normalizarErro180(anguloGolGraus - 180.0f);
}

// Retorna o erro de alinhamento com o gol conforme o papel atual
float calcularErroGolPorPapel(float anguloGolGraus) {
  if (papelAtacante) {
    return normalizarErro180(anguloGolGraus);
  }
  return calcularErroGolEspelhadoDefensor(anguloGolGraus);
}

// Calcula o erro angular atual em relação à referência salva da bússola
float calcularErroReferenciaBussola() {
  if (!bussolaValida) {
    return 0.0f;
  }
  return normalizarErro180((float)headingBussolaSalvo - (float)headingBussolaTeste);
}

// Gera ângulo local de translação para voltar ao gol pela bússola
float calcularAnguloRetornoGolPorBussola() {
  return normalizarAngulo360(180.0f + calcularErroReferenciaBussola());
}

// Retorna true se a bússola possui referência válida
bool bussolaTemReferenciaValida() {
  return bussolaValida;
}

// Atualiza validade da bússola por timeout
void atualizarValidadeBussola() {
  if (bussolaValida && (millis() - ultimoRxBussolaMs) > TIMEOUT_BUSSOLA_MS) {
    bussolaValida = false;
  }
}

// Calcula o erro do campo (mesma base do gol invertido)
float calcularErroAngularCampo() {
  return erroAlinhamentoGraus;
}

// Corrige ângulo local para o referencial do campo
float corrigirAnguloParaCampo(float anguloLocalGraus, float erroAngularGraus) {
  return normalizarAngulo360(anguloLocalGraus + erroAngularGraus);
}

// Regras de deslocamento lateral pela bola corrigida no campo
// Direita real (20..120) -> 120 - erroAngular
// Esquerda real (240..340) -> 340 + erroAngular
bool calcularComandoLateralPorBolaCorrigida(float anguloBolaCorrigido,
                                            float erroAngular,
                                            float &anguloComando) {
  float ang = normalizarAngulo360(anguloBolaCorrigido);
  if (ang >= 20.0f && ang <= 120.0f) {
    anguloComando = normalizarAngulo360(120.0f - erroAngular);
    return true;
  }
  if (ang >= 240.0f && ang <= 340.0f) {
    anguloComando = normalizarAngulo360(340.0f + erroAngular);
    return true;
  }
  return false;
}


// =============================================================================
// SECAO 12 — FUNCOES DE CAMERA (compartilhadas)
// =============================================================================

// Retorna true se o pacote de câmera está dentro do timeout
bool cameraPacoteRecente() {
  return (ultimoRxCameraMs > 0) && ((millis() - ultimoRxCameraMs) <= TIMEOUT_CAMERA_MS);
}

// Atualiza flags de validade e seleciona gol conforme corGolAzul
void atualizarValidadeCamera() {
  cameraDadosValidos          = cameraPacoteRecente();
  cameraGolSelecionadoAzul    = corGolAzul;
  cameraGolSelecionadoAngle   = cameraGolSelecionadoAzul ? cameraBlueAngle  : cameraYellowAngle;
  cameraGolSelecionadoDist    = cameraGolSelecionadoAzul ? cameraBlueDist   : cameraYellowDist;
  cameraGolSelecionadoValido  = cameraPacoteRecente() &&
                                (cameraGolSelecionadoAngle != -999) &&
                                (cameraGolSelecionadoDist > 0);
}

// Retorna true se a câmera está vendo a bola com dados válidos
bool cameraTemBolaValida() {
  return cameraPacoteRecente() &&
         (cameraBallAngle != -999) &&
         (cameraBallDist > 0) &&
         !((cameraBallAngle == 0) && (cameraBallDist == 0));
}

// Adiciona leitura de ângulo de bola ao buffer circular
void adicionarLeituraCameraNoBuffer(float anguloGraus) {
  cameraBallBufferAngulos[cameraBallBufferIndice] = normalizarAngulo360(anguloGraus);
  cameraBallBufferIndice = (cameraBallBufferIndice + 1) % CAMERA_BOLA_BUFFER_TAM;
  if (cameraBallBufferQuantidade < CAMERA_BOLA_BUFFER_TAM) {
    cameraBallBufferQuantidade++;
  }
}

// Filtra o ângulo da bola pelo buffer (média circular + predição)
bool obterAnguloCameraBolaFiltrado(float &anguloFiltrado) {
  if (!cameraPacoteRecente() || cameraBallBufferQuantidade == 0) {
    return false;
  }
  float somaSin = 0.0f;
  float somaCos = 0.0f;
  for (uint8_t i = 0; i < cameraBallBufferQuantidade; i++) {
    float rad = cameraBallBufferAngulos[i] * PI / 180.0f;
    somaSin += sinf(rad);
    somaCos += cosf(rad);
  }
  float mediaCircular = normalizarAngulo360(atan2f(somaSin, somaCos) * 180.0f / PI);
  int idxUltimo = (cameraBallBufferIndice + CAMERA_BOLA_BUFFER_TAM - 1) % CAMERA_BOLA_BUFFER_TAM;
  float anguloPrevisto = cameraBallBufferAngulos[idxUltimo];

  // Predição de um passo à frente pelo delta angular mais recente
  if (cameraBallBufferQuantidade >= 2) {
    int idxPenultimo = (idxUltimo + CAMERA_BOLA_BUFFER_TAM - 1) % CAMERA_BOLA_BUFFER_TAM;
    float deltaPrev = normalizarErro180(cameraBallBufferAngulos[idxUltimo] - cameraBallBufferAngulos[idxPenultimo]);
    if (deltaPrev >  CAMERA_BOLA_DELTA_PREVISAO_MAX_GRAUS) deltaPrev =  CAMERA_BOLA_DELTA_PREVISAO_MAX_GRAUS;
    if (deltaPrev < -CAMERA_BOLA_DELTA_PREVISAO_MAX_GRAUS) deltaPrev = -CAMERA_BOLA_DELTA_PREVISAO_MAX_GRAUS;
    anguloPrevisto = normalizarAngulo360(cameraBallBufferAngulos[idxUltimo] + deltaPrev);
  }

  float erroPrevMedia = normalizarErro180(anguloPrevisto - mediaCircular);
  anguloFiltrado = normalizarAngulo360(mediaCircular + (erroPrevMedia * CAMERA_BOLA_PESO_PREVISAO));
  return true;
}

// Lê gol selecionado (para o menu de calibração)
bool cameraLerGolSelecionadoMenu(int16_t &anguloGol, uint16_t &distGol) {
  anguloGol = cameraGolSelecionadoAngle;
  distGol   = cameraGolSelecionadoDist;
  return cameraGolSelecionadoValido;
}

// Retorna true e ângulo do gol selecionado se visível
bool cameraTemGolSelecionadoValido(int16_t &anguloGol) {
  anguloGol = cameraGolSelecionadoAngle;
  return cameraGolSelecionadoValido;
}

// Retorna true e ângulo do gol de retorno do defensor (gol adversário)
bool cameraTemGolRetornoDefensorValido(int16_t &anguloGol) {
  bool     usarGolAzul = !corGolAzul;
  uint16_t distGol     = usarGolAzul ? cameraBlueDist    : cameraYellowDist;
  anguloGol             = usarGolAzul ? cameraBlueAngle   : cameraYellowAngle;
  return cameraPacoteRecente() && (anguloGol != -999) && (distGol > 0);
}


// =============================================================================
// SECAO 13 — FUNCOES DE LINHA (compartilhadas)
// =============================================================================

// Declarações antecipadas para uso interno nas funções de linha
extern bool  linhaDetectada;
extern float anguloLinhaPe;

// Descarta leituras antigas para não manter fuga ativa com dado obsoleto
void atualizarValidadeLinha() {
  if (linhaDetectada && (millis() - ultimoRxLinhaMs) > TIMEOUT_LINHA_MS) {
    linhaDetectada = false;
    anguloLinhaPe  = -1.0f;
  }
  if ((millis() - ultimoRxLinhaMs) > TIMEOUT_LINHA_MS) {
    linhaZonaAValida = false;
    linhaZonaBValida = false;
    anguloLinhaZonaA = -1.0f;
    anguloLinhaZonaB = -1.0f;
  }
}

// Calcula direção de centralização para teste de linha (teste de centro)
float calcularDirecaoCentroLinhaTeste() {
  if (!linhaDetectada || anguloLinhaPe < 0.0f) {
    return -1.0f;
  }
  // Exibe o comando puro da linha, sem inversão de repulsão
  return normalizarAngulo360(anguloLinhaPe);
}


// =============================================================================
// SECAO 14 — FUNCOES ESP-NOW E PAPEL AUTOMATICO
// =============================================================================

// Retorna true se ESP-NOW está conectado e com pacote recente
bool espnowConectadoRecente() {
  return espnowOK && (ultimoRxEspnowMs > 0) && ((millis() - ultimoRxEspnowMs) <= TIMEOUT_COM_MS);
}

// Atualiza flag "sozinho" conforme estado do ESP-NOW durante o jogo
void atualizarSozinhoLocal() {
  if (estadoAtual == INICIAR) {
    sozinho = !espnowConectadoRecente();
  }
}

// Envia estado de jogo (RUN:1/0) para a Cabeça
void enviarEstadoJogoParaCabeca(bool forcar = false) {
  bool emJogo = (estadoAtual == INICIAR);
  unsigned long agora = millis();
  if (!forcar &&
      emJogo == estadoJogoCabecaEnviado &&
      (agora - ultimoEnvioEstadoJogoCabecaMs) < INTERVALO_ENVIO_ESTADO_JOGO_MS) {
    return;
  }
  Serial1.print("RUN:");
  Serial1.println(emJogo ? 1 : 0);
  estadoJogoCabecaEnviado        = emJogo;
  ultimoEnvioEstadoJogoCabecaMs  = agora;
}

// Envia papel atual (ATCFB:1/0) para a Cabeça ressincronizar o Pé
void enviarPapelAtualParaCabeca() {
  Serial1.print("ATCFB:");
  Serial1.println(papelAtacante ? 1 : 0);
}

// Retorna true se ultrassônicos locais estão recentes (para decisão de papel)
bool ultrasLocaisRecentesParaPapel() {
  return ultrasValidos && (ultimoRxUltraMs > 0) && ((millis() - ultimoRxUltraMs) <= TIMEOUT_ULTRA_MS);
}

// Retorna true se ultrassônicos remotos estão recentes (para decisão de papel)
bool ultrasRemotosRecentesParaPapel() {
  return ultrasRemotosValidos && (ultimoRxUltraRemotoMs > 0) && ((millis() - ultimoRxUltraRemotoMs) <= TIMEOUT_ULTRA_MS);
}

// Atualiza o papel automático com base na parceria (ESP-NOW)
void atualizarPapelAutomaticoPorParceria() {
  if (papelConfiguradoMenu != PAPEL_CONFIG_AUTO) {
    return;
  }
  bool novoPapelAtacante = true;
  if (novoPapelAtacante != papelAtacante) {
    papelAtacante        = novoPapelAtacante;
    papelAtacanteAnterior = novoPapelAtacante;
    enviarPapelAtualParaCabeca();
  }
}


// =============================================================================
// SECAO 15 — FUNCOES DE ENVIO PARA A CABECA
// =============================================================================

// Envia periodicamente a cor de gol selecionada para a Cabeça
void enviarCorGolParaCabeca() {
  if (!corGolPendenteEnvio) {
    return;
  }
  if ((millis() - ultimoEnvioCorGolMs) < INTERVALO_ENVIO_COR_GOL_MS) {
    return;
  }
  // A Cabeça confirma com "CFG:GOL:OK" quando assumir a configuração
  Serial1.print("CFG:GOL:");
  Serial1.println(corGolAzul ? "1" : "0");
  ultimoEnvioCorGolMs = millis();
}

// Envia a referencia de heading salva na EEPROM para a Cabeca.
void enviarReferenciaBussolaParaCabeca(bool forcar = false) {
  if (!forcar && (millis() - ultimoEnvioRefBussolaMs) < INTERVALO_ENVIO_REF_BUSSOLA_MS) {
    return;
  }

  int ref = headingBussolaSalvo;
  if (ref < 0) ref = 0;
  if (ref >= 360) ref %= 360;

  Serial1.print("BUSREF:");
  Serial1.println(ref);
  ultimoEnvioRefBussolaMs = millis();
}

// Solicita leituras brutas de sensor ao Pé (somente na tela de sensores)
void solicitarSensoresBrutosPe(bool forcar = false) {
  if (estadoAtual != FUNCAO || subMenuFuncao != SUBFUNCAO_SENSORES) {
    return;
  }
  if (!forcar && (millis() - ultimoReqSensoresBrutosMs) < INTERVALO_REQ_SENSORES_BRUTOS_MS) {
    return;
  }
  Serial1.println("REQ:SENS");
  ultimoReqSensoresBrutosMs = millis();
}

// Solicita limiar de linha ao Pé (somente na tela de ajuste de limiar)
void solicitarLimiarLinhaPe(bool forcar = false) {
  if (estadoAtual != FUNCAO || subMenuFuncao != SUBFUNCAO_LIMIAR_LINHA) {
    return;
  }
  if (!forcar && (millis() - ultimoReqLimiarLinhaMs) < INTERVALO_REQ_LIMIAR_LINHA_MS) {
    return;
  }
  Serial1.println("REQ:LIM");
  ultimoReqLimiarLinhaMs = millis();
}

// Envia limiar de linha editado para o Pé aplicar
void enviarLimiarLinhaParaPe() {
  Serial1.print("SETLIM:");
  Serial1.println(limiarLinhaEditado);
}


// IHM movida para src/NEXUS/musculo/display/ihm_display.cpp


// =============================================================================
// SECAO 18 — PROCESSAMENTO DE MENSAGENS DA CABECA (Serial1)
// =============================================================================

// Interpreta mensagens recebidas da Cabeça e atualiza estados locais
void processarMensagemCabeca(String msg) {
  // Normaliza o frame para simplificar comparações de protocolo
  msg.trim();
  msg.toUpperCase();

  // Handshake: confirma que a serial está viva
  if (msg == "OI") {
    comunicacaoCabecaOK = true; ultimoRxCabeca = millis(); return;
  }

  // Status agregado das placas auxiliares (Olho e Pé)
  if (msg.startsWith("STS:")) {
    int separador = msg.indexOf(',');
    if (separador > 4) {
      String olho = msg.substring(4, separador);
      String pe   = msg.substring(separador + 1);
      olho.trim(); pe.trim();
      olhoOK = (olho == "1"); peOK = (pe == "1");
      comunicacaoCabecaOK = true; ultimoRxCabeca = millis();
      mostrarStatusAte = millis() + 3000;
    }
    return;
  }

  // Eventos de botão: BTN:1, BTN:2, BTN:3 ou BTN:23
  if (msg.startsWith("BTN:")) {
    String valor = msg.substring(4); valor.trim();
    if (valor == "1" || valor == "2" || valor == "3" || valor == "23") {
      mensagemBotao = "BOTAO " + valor + " APERTADO";
      comunicacaoCabecaOK = true; ultimoRxCabeca = millis();
      processarEventoBotao((uint8_t)valor.toInt());
    }
    return;
  }

  // Ângulo IR: negativo = sem bola; 30° exato é descartado como ruído
  if (msg.startsWith("IR:")) {
    String valorIr = msg.substring(3); valorIr.trim();
    float novoAngulo = valorIr.toFloat();
    if (novoAngulo < 0.0f) {
      irDetectado = false; anguloIr = -1.0f;
    } else {
      bool eh30Graus = fabsf(novoAngulo - 30.0f) <= 1.0f;
      if (eh30Graus) {
        // Descarta imediatamente como sem sinal de bola
        irDetectado = false; anguloIr = -1.0f;
      } else {
        irDetectado = true; anguloIr = novoAngulo;
        ultimoAnguloIrValido = normalizarAngulo360(novoAngulo);
        ultimoIrValidoMs = millis();
      }
    }
    comunicacaoCabecaOK = true; ultimoRxCabeca = millis(); return;
  }

  // Confirmação de configuração de cor do gol
  if (msg == "CFG:GOL:OK") {
    corGolPendenteEnvio = false;
    mensagemBotao = corGolAzul ? "GOL AZUL OK" : "GOL AMARELO OK";
    comunicacaoCabecaOK = true; ultimoRxCabeca = millis(); return;
  }

  // Leitura da bússola: normaliza e converte para inteiro [0, 359]
  if (msg.startsWith("BUS:")) {
    String valorBus = msg.substring(4); valorBus.trim();
    if (!payloadNumericoValido(valorBus)) { bussolaValida = false; return; }
    float angBus = valorBus.toFloat();
    angBus = normalizarAngulo360(angBus);
    headingBussolaTeste = (int)(angBus + 0.5f);
    if (headingBussolaTeste >= 360) headingBussolaTeste = 0;
    bussolaValida = true; ultimoRxBussolaMs = millis();
    comunicacaoCabecaOK = true; ultimoRxCabeca = millis(); return;
  }

  if (msg.startsWith("POS:")) {
    String payload = msg.substring(4);
    payload.trim();
    int separador = payload.indexOf('/');
    if (separador > 0) {
      String sx = payload.substring(0, separador);
      String sy = payload.substring(separador + 1);
      sx.trim(); sy.trim();
      if (payloadNumericoValido(sx) && payloadNumericoValido(sy)) {
        float alvoX = sx.toFloat();
        float alvoY = sy.toFloat();
        float minX = ROBO_RAIO_CAMPO_CM;
        float maxX = CAMPO_LARGURA_CM - ROBO_RAIO_CAMPO_CM;
        float minY = ROBO_RAIO_CAMPO_CM;
        float maxY = CAMPO_ALTURA_CM - ROBO_RAIO_CAMPO_CM;
        if (!posicionamentoModuloArmado()) {
          mensagemBotao = "POS BLOQ DISPLAY";
          mostrarStatusAte = millis() + 1800;
          cancelarPosicionamentoAlvo(true);
          comunicacaoCabecaOK = true; ultimoRxCabeca = millis();
          return;
        }
        if (alvoX >= minX && alvoX <= maxX && alvoY >= minY && alvoY <= maxY) {
          posicionamentoAlvoXcm = alvoX;
          posicionamentoAlvoYcm = alvoY;
          posicionamentoAlvoAtivo = true;
          mensagemBotao = "POS ALVO RECEBIDO";
          mostrarStatusAte = millis() + 1800;
        } else {
          mensagemBotao = "POS FORA CAMPO";
          mostrarStatusAte = millis() + 1800;
        }
        comunicacaoCabecaOK = true; ultimoRxCabeca = millis();
      }
    }
    return;
  }

  // Telemetria de gol: erro angular, flag de detecção, pixels e estado da câmera
  if (msg.startsWith("GOL:")) {
    String payload = msg.substring(4);
    int p1 = payload.indexOf(','), p2 = payload.indexOf(',', p1+1), p3 = payload.indexOf(',', p2+1);
    if (p1 > 0 && p2 > p1) {
      String sErro = payload.substring(0, p1);
      String sDet  = payload.substring(p1+1, p2);
      String sPix  = payload.substring(p2+1, (p3 > p2) ? p3 : payload.length());
      sErro.trim(); sDet.trim(); sPix.trim();
      erroGolGraus = sErro.toFloat();
      golDetectado = (sDet == "1");
      int pix = sPix.toInt();
      if (pix < 0) pix = 0; if (pix > 65535) pix = 65535;
      golPixels = (uint16_t)pix;
      if (p3 > p2) { String sCam = payload.substring(p3+1); sCam.trim(); cameraOK = (sCam == "1"); }
      comunicacaoCabecaOK = true; ultimoRxCabeca = millis();
    }
    return;
  }

  // Linha atacante: ângulo único (negativo = sem leitura)
  if (msg.startsWith("LIN:")) {
    String sLinha = msg.substring(4); sLinha.trim();
    float novoAng = sLinha.toFloat();
    linhaDetectada = (novoAng >= 0.0f); anguloLinhaPe = novoAng;
    ultimoRxLinhaMs = millis();
    if (linhaDetectada) { ultimoAnguloLinhaValido = anguloLinhaPe; ultimoComandoLinhaMs = ultimoRxLinhaMs; }
    comunicacaoCabecaOK = true; ultimoRxCabeca = millis(); return;
  }

  // Linha defensor — Zona A (0°–180°)
  if (msg.startsWith("LINA:")) {
    String sLinhaA = msg.substring(5); sLinhaA.trim();
    float novoAngA = sLinhaA.toFloat();
    linhaZonaAValida = (novoAngA >= 0.0f); anguloLinhaZonaA = linhaZonaAValida ? novoAngA : -1.0f;
    ultimoRxLinhaMs = millis();
    if (linhaZonaAValida) { ultimoAnguloLinhaZonaAValido = anguloLinhaZonaA; ultimoRxLinhaZonaAMs = ultimoRxLinhaMs; }
    comunicacaoCabecaOK = true; ultimoRxCabeca = millis(); return;
  }

  // Linha defensor — Zona B (180°–360°)
  if (msg.startsWith("LINB:")) {
    String sLinhaB = msg.substring(5); sLinhaB.trim();
    float novoAngB = sLinhaB.toFloat();
    linhaZonaBValida = (novoAngB >= 0.0f); anguloLinhaZonaB = linhaZonaBValida ? novoAngB : -1.0f;
    ultimoRxLinhaMs = millis();
    if (linhaZonaBValida) { ultimoAnguloLinhaZonaBValido = anguloLinhaZonaB; ultimoRxLinhaZonaBMs = ultimoRxLinhaMs; }
    comunicacaoCabecaOK = true; ultimoRxCabeca = millis(); return;
  }

  // Ultrassônicos locais: ULT:D,E,F,T
  if (msg.startsWith("ULT:")) {
    String payload = msg.substring(4);
    int p1 = payload.indexOf(','), p2 = payload.indexOf(',', p1+1), p3 = payload.indexOf(',', p2+1);
    if (p1 > 0 && p2 > p1 && p3 > p2) {
      String sD = payload.substring(0, p1), sE = payload.substring(p1+1, p2);
      String sF = payload.substring(p2+1, p3), sT = payload.substring(p3+1);
      sD.trim(); sE.trim(); sF.trim(); sT.trim();
      ultraDcm = sD.toFloat(); ultraEcm = sE.toFloat();
      ultraFcm = sF.toFloat(); ultraTcm = sT.toFloat();
      ultrasValidos = (ultraDcm >= 0.0f && ultraEcm >= 0.0f && ultraFcm >= 0.0f && ultraTcm >= 0.0f);
      ultimoRxUltraMs = millis();
      atualizarPapelAutomaticoPorParceria();
      comunicacaoCabecaOK = true; ultimoRxCabeca = millis();
    }
    return;
  }

  // Ultrassônicos remotos (do outro robô via ESP-NOW): ULR:D,E,F,T
  if (msg.startsWith("ULR:")) {
    String payload = msg.substring(4);
    int p1 = payload.indexOf(','), p2 = payload.indexOf(',', p1+1), p3 = payload.indexOf(',', p2+1);
    if (p1 > 0 && p2 > p1 && p3 > p2) {
      String sD = payload.substring(0, p1), sE = payload.substring(p1+1, p2);
      String sF = payload.substring(p2+1, p3), sT = payload.substring(p3+1);
      sD.trim(); sE.trim(); sF.trim(); sT.trim();
      ultraRemotoDcm = sD.toFloat(); ultraRemotoEcm = sE.toFloat();
      ultraRemotoFcm = sF.toFloat(); ultraRemotoTcm = sT.toFloat();
      ultrasRemotosValidos = (ultraRemotoDcm >= 0.0f && ultraRemotoEcm >= 0.0f &&
                              ultraRemotoFcm >= 0.0f && ultraRemotoTcm >= 0.0f);
      ultimoRxUltraRemotoMs = millis();
      atualizarPapelAutomaticoPorParceria();
      comunicacaoCabecaOK = true; ultimoRxCabeca = millis();
    }
    return;
  }

  // Estado da chave do kicker: KIK:0 = acionada, KIK:1 = não acionada
  if (msg.startsWith("KIK:")) {
    String sKik = msg.substring(4); sKik.trim();
    if (sKik == "0" || sKik == "1") {
      kickerAtivado = (sKik == "0"); kickerRecebido = true;
      comunicacaoCabecaOK = true; ultimoRxCabeca = millis();
    }
    return;
  }

  // Papel definido pela Cabeça: ATC:1 = atacante, ATC:0 = defensor
  if (msg.startsWith("ATC:")) {
    String sAtc = msg.substring(4); sAtc.trim();
    if (sAtc == "0" || sAtc == "1") {
      bool novoValor = (sAtc == "1");
      if (papelConfiguradoMenu == PAPEL_CONFIG_AUTO) {
        if (novoValor != papelAtacante) {
          Serial0.print("ATCFB:"); Serial0.println(novoValor ? 1 : 0);
        }
        papelAtacante = novoValor; atualizarPapelAutomaticoPorParceria();
      } else {
        aplicarPapelConfiguradoLocal();
        if (novoValor != papelAtacante) { Serial0.print("ATCFB:"); Serial0.println(papelAtacante ? 1 : 0); }
      }
      papelAtacanteAnterior = papelAtacante;
      comunicacaoCabecaOK = true; ultimoRxCabeca = millis();
    }
    return;
  }

  // Dados de câmera: CAM:ballAngle,ballDist,blueAngle,blueDist,yellowAngle,yellowDist,cameraOK
  if (msg.startsWith("CAM:")) {
    String payload = msg.substring(4);
    int p1=payload.indexOf(','), p2=payload.indexOf(',',p1+1), p3=payload.indexOf(',',p2+1);
    int p4=payload.indexOf(',',p3+1), p5=payload.indexOf(',',p4+1), p6=payload.indexOf(',',p5+1);
    if (p1>0 && p2>p1 && p3>p2 && p4>p3 && p5>p4 && p6>p5) {
      String sBallA=payload.substring(0,p1),      sBallD=payload.substring(p1+1,p2);
      String sBlueA=payload.substring(p2+1,p3),   sBlueD=payload.substring(p3+1,p4);
      String sYellA=payload.substring(p4+1,p5),   sYellD=payload.substring(p5+1,p6);
      String sCamOK=payload.substring(p6+1);
      sBallA.trim(); sBallD.trim(); sBlueA.trim(); sBlueD.trim(); sYellA.trim(); sYellD.trim(); sCamOK.trim();
      cameraBallAngle = (int16_t)sBallA.toInt();
      cameraBallDist  = (uint16_t)sBallD.toInt();
      unsigned long agoraCameraMs = millis();
      bool cameraBolaBrutaValida = (cameraBallAngle != -999) && (cameraBallDist > 0) &&
                                   !((cameraBallAngle == 0) && (cameraBallDist == 0));
      if (cameraBolaBrutaValida) {
        cameraUltimaVezBolaDetetadaMs = agoraCameraMs;
        cameraSemBolaBrutaInicioMs = 0;
        adicionarLeituraCameraNoBuffer((float)cameraBallAngle);
      } else if (cameraSemBolaBrutaInicioMs == 0) {
        cameraSemBolaBrutaInicioMs = agoraCameraMs;
      }
      cameraBlueAngle   = (int16_t)sBlueA.toInt();
      cameraBlueDist    = (uint16_t)sBlueD.toInt();
      cameraYellowAngle = (int16_t)sYellA.toInt();
      cameraYellowDist  = (uint16_t)sYellD.toInt();
      cameraDadosValidos = (sCamOK == "1");
      ultimoRxCameraMs  = millis();
      atualizarValidadeCamera();
      comunicacaoCabecaOK = true; ultimoRxCabeca = millis();
    }
    return;
  }

  // Sensores brutos do Pé: SENS:S1,S9,S17,S25
  if (msg.startsWith("SENS:")) {
    String payload = msg.substring(5);
    int p1=payload.indexOf(','), p2=payload.indexOf(',',p1+1), p3=payload.indexOf(',',p2+1);
    if (p1>0 && p2>p1 && p3>p2) {
      String s1=payload.substring(0,p1), s9=payload.substring(p1+1,p2);
      String s17=payload.substring(p2+1,p3), s25=payload.substring(p3+1);
      s1.trim(); s9.trim(); s17.trim(); s25.trim();
      sensorBruto1 = s1.toInt(); sensorBruto9 = s9.toInt();
      sensorBruto17 = s17.toInt(); sensorBruto25 = s25.toInt();
      ultimoRxSensoresBrutosMs = millis();
      comunicacaoCabecaOK = true; ultimoRxCabeca = millis();
    }
    return;
  }

  // Limiar de linha recebido do Pé: LIM:valor
  if (msg.startsWith("LIM:")) {
    String valor = msg.substring(4); valor.trim();
    int recebido = valor.toInt();
    if (recebido >= LIMIAR_LINHA_MIN && recebido <= LIMIAR_LINHA_MAX) {
      limiarLinhaEditado = recebido; limiarLinhaSincronizado = true;
      comunicacaoCabecaOK = true; ultimoRxCabeca = millis();
    }
    return;
  }

  // Status do ESP-NOW entre as Cabeças: ESN:1 = conectado
  if (msg.startsWith("ESN:")) {
    String sEsn = msg.substring(4); sEsn.trim();
    espnowOK = (sEsn == "1"); ultimoRxEspnowMs = millis();
    atualizarSozinhoLocal(); atualizarPapelAutomaticoPorParceria();
    comunicacaoCabecaOK = true; ultimoRxCabeca = millis(); return;
  }
}

// Acumula bytes da Serial1 e despacha mensagens completas por linha
void lerSerialCabeca() {
  while (Serial1.available() > 0) {
    char c = (char)Serial1.read();
    // Protocolo orientado a linha: \r e \n encerram o frame atual
    if (c == '\n' || c == '\r') {
      if (bufferSerial.length() > 0) {
        processarMensagemCabeca(bufferSerial);
        bufferSerial = "";
      }
      continue;
    }
    // Limita o buffer para evitar crescimento indefinido com ruído serial
    if (bufferSerial.length() < 32) {
      bufferSerial += c;
    }
  }
}


// =============================================================================
// SECAO 19 — ESTRATEGIA DO ATACANTE
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
// Parâmetros de ajuste — *** ALTERE APENAS AQUI para tunar o atacante ***
// =============================================================================

// --- Velocidades do atacante ---
const int VELOCIDADE_IR_FRONTAL_PWM       = 200;   // PWM na faixa frontal do IR (±32°)
const int VELOCIDADE_IR_FAIXA_REDUZIDA_PWM = 140;  // PWM em faixas laterais do IR

// --- Freio ultrassônico do atacante (laterais) ---
const float ATACANTE_ULTRA_FREIO_INICIO_CM    = 55.0f;   // Distância onde o freio começa
const float ATACANTE_ULTRA_FREIO_CRITICO_CM   = 35.0f;   // Distância de freio máximo
const int   ATACANTE_ULTRA_FREIO_VELOCIDADE_MIN = 125;   // Velocidade mínima com freio
const int   ATACANTE_ULTRA_FREIO_PWM_POR_CM   = 3;       // Incremento de PWM por cm

// --- Buffer de perda de sinal IR ---
const unsigned long IR_BUFFER_PERDA_MS = 160; // Tempo que o último ângulo IR é mantido após perda

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

// Detecta faixa frontal do IR em torno de 0° (±32°), tratando wrap 360°->0°
bool irNaFaixaFrontal(float anguloBolaGraus) {
  float ang = normalizarAngulo360(anguloBolaGraus);
  return (ang > 328.0f || ang < 32.0f);
}

// Retorna true e ângulo do IR (com buffer de retenção após perda de sinal)
bool obterAnguloIrComBuffer(float &anguloIrSaida, bool &usandoBuffer)
{
    if (irDetectado && anguloIr >= 0.0f)
    {
        usandoBuffer = false;
        anguloIrSaida = normalizarAngulo360(anguloIr);
        return true;
    }

    if ((ultimoAnguloIrValido >= 0.0f) &&
        (ultimoIrValidoMs > 0) &&
        ((millis() - ultimoIrValidoMs) <= IR_BUFFER_PERDA_MS))
    {
        usandoBuffer = true;
        anguloIrSaida = normalizarAngulo360(ultimoAnguloIrValido);
        return true;
    }

    usandoBuffer = false;
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
  static unsigned long inicioOrbita = 0;
  static bool emOrbitaEspecial = false;

  float ang = normalizarAngulo360(anguloBolaGraus);

  if(ang >= 32.0f && ang <= 60.0f)
  {
    emOrbitaEspecial = false;
    return 100.0f;
  }

  if(ang >= 300.0f && ang <= 328.0f)
  {
    emOrbitaEspecial = false;
    return 260.0f;
  }

  // ==========================================
  // 60 -> 90
  // 135 vai fechando até 90
  // ==========================================
  if(ang > 60.0f && ang < 90.0f)
  {
    if(!emOrbitaEspecial)
    {
      inicioOrbita = millis();
      emOrbitaEspecial = true;
    }

    float t = (millis() - inicioOrbita) / 1000.0f;

    float anguloMov = 135.0f - (t * 20.0f);

    if(anguloMov < 90.0f)
      anguloMov = 90.0f;

    return anguloMov;
  }

  // ==========================================
  // 270 -> 300
  // 225 vai abrindo até 270
  // ==========================================
  if(ang >= 270.0f && ang < 300.0f)
  {
    if(!emOrbitaEspecial)
    {
      inicioOrbita = millis();
      emOrbitaEspecial = true;
    }

    float t = (millis() - inicioOrbita) / 1000.0f;

    float anguloMov = 225.0f + (t * 20.0f);

    if(anguloMov > 270.0f)
      anguloMov = 270.0f;

    return anguloMov;
  }

  // saiu das zonas especiais
  emOrbitaEspecial = false;

  if(ang >= 90.0f && ang < 135.0f)  return 180.0f;
  if(ang >= 225.0f && ang < 270.0f) return 180.0f;
  if(ang >= 180.0f && ang < 225.0f) return 135.0f;
  if(ang >= 135.0f && ang < 180.0f) return 225.0f;

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
    static float erroAnterior = 0;
    static float integral = 0;

    const float Kp = 1.2;
    const float Ki = 0.01;
    const float Kd = 0.8;

    integral += erro;

    // Anti-windup
    integral = constrain(integral, -100, 100);

    float derivada = erro - erroAnterior;

    float saida =
        (Kp * erro) +
        (Ki * integral) +
        (Kd * derivada);

    erroAnterior = erro;

    // Corrige o sentido da sua bússola
    return -saida;
}


// *** FUNCAO PRINCIPAL DO ATACANTE ***
// Alinha ao gol, segue bola por IR/câmera, foge da linha e chuta quando alinhado
void atacante() {

  float erroBussola = calcularErroReferenciaBussola();
  Serial.println(erroBussola);

  int errobussolaCorretoMovi = PIDZIMBUSSOLANOVINHA(erroBussola);






  float anguloIrBufferizado = -1.0f;
  bool usandoBuffer = false;

  bool irDisponivel =
      obterAnguloIrComBuffer(
          anguloIrBufferizado,
          usandoBuffer
      );

  // -------------------------------
  // ALINHAMENTO (SEM TRAVAR FLUXO)
  // -------------------------------
  

  // -------------------------------
  // PRIORIDADE 1: BÚSSOLA FORTE
  // -------------------------------
  if (fabsf(erroBussola) > 80.0f) {
    girarNoEixo(errobussolaCorretoMovi);
    return;
  }

  // -------------------------------
  // PRIORIDADE 2: LINHA (USANDO SUA FUNÇÃO NOVA)
  // -------------------------------
    float anguloFuga = 0.0f;
    if (sairDaLinha(linhaDetectada, anguloLinhaPe,
            aplicarFreioUltrassonicoAtacante(VELOCIDADE_FUGA_LINHA),
            &anguloFuga)) {
    fugindoLinhaAgora = true;
    anguloFugaLinhaCmd = anguloFuga;
    return;
    }

  // -------------------------------
  // PRIORIDADE 3: IR + ATAQUE
  // -------------------------------
  else if (irDisponivel) {

    float ang;

    if (usandoBuffer) {
      ang = anguloIrBufferizado;
    } else {
      ang = mapearAnguloBolaParaMovimento(
          anguloIrBufferizado
      );
    }

    float angSuave =
        suavizadorMegaAnguloMovimento(ang);

    // -----------------------
    // FRONTAL
    // -----------------------
    if ((irNaFaixaFrontal(obterAnguloIrSuavizado(anguloIrBufferizado))) ||
        irNaFaixaFrontal(anguloIrBufferizado)) {

      if (fabs(errobussolaCorretoMovi) > 5) {

        if (ultraDcm < ultraEcm) {
          seguirDirecaoComGiro(353, velocidade_maxima, errobussolaCorretoMovi);
        } else {
          seguirDirecaoComGiro(7, velocidade_maxima, errobussolaCorretoMovi);
        }

      } else {

        if (ultraDcm < ultraEcm) {
          seguirDirecaoPorAngulo(353, velocidade_maxima);
        } else {
          seguirDirecaoPorAngulo(7, velocidade_maxima);
        }
      }
    }

    // -----------------------
    // NÃO FRONTAL
    // -----------------------
    else {

      if (((ang > 32) && (ang < 135)) ||
          ((ang < 328) && (ang > 225))) {

        seguirDirecaoComGiroLaterais(
            angSuave,
            velocidade_maxima,
            errobussolaCorretoMovi
        );

      } else {

        if (fabs(errobussolaCorretoMovi) > 5) {

          seguirDirecaoComGiro(
              angSuave,
              velocidade_maxima,
              errobussolaCorretoMovi
          );

        } else {

          seguirDirecaoPorAngulo(
              angSuave,
              velocidade_maxima
          );
        }
      }
    }
  }

  // -------------------------------
  // SEM INFORMAÇÃO
  // -------------------------------
  else {

    if (fabsf(errobussolaCorretoMovi) > 15.0f) {
      girarNoEixo(errobussolaCorretoMovi);
    }

    pararMotores();
    delay(10);
  }
}
















// =============================================================================
// SECAO 20 — ESTRATEGIA DO DEFENSOR (GOLEIRO)
// =============================================================================
//
// Responsabilidades:
//   • Manter posição na frente do gol usando linha como referência (zonas A e B)
//   • Acompanhar a bola lateralmente via IR / câmera
//   • Conter a bola com ultrassônicos laterais e de profundidade
//   • Retornar ao gol pela bússola quando sem linha
//   • Avançar sobre a bola quando ela fica frontal por tempo suficiente
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
constexpr float DEFENSOR_VELOCIDADE_MIN_PWM        = 145.0f;
constexpr float DEFENSOR_VELOCIDADE_RETORNO_GOL_PWM = 165.0f;

// --- Suavização do defensor (fator exponencial) ---
constexpr float DEFENSOR_SUAVIZACAO_VETOR = 0.45f;
constexpr float DEFENSOR_SUAVIZACAO_GIRO  = 0.35f;

// --- Parâmetros do avanço frontal temporizado do defensor ---
const float         DEFENSOR_TOLERANCIA_IR_FRONTAL_GRAUS      = 45.0f;
const int           DEFENSOR_VELOCIDADE_AVANCO_IR_FRONTAL_PWM = 255;
const unsigned long DEFENSOR_TEMPO_GATILHO_IR_FRONTAL_MS      = 3000;
const unsigned long DEFENSOR_TEMPO_AVANCO_IR_FRONTAL_MS       = 2500;

// --- Controle proporcional lateral do defensor por IR ---
int calcularVelocidadeLateralDefensorPorIr(float anguloBolaGraus) {
  const float ANG_DIREITA_MIN  = 20.0f;
  const float ANG_DIREITA_MAX  = 160.0f;  /// 
  const float ANG_ESQUERDA_MIN = 200.0f;  /// 
  const float ANG_ESQUERDA_MAX = 340.0f;
  const int   VEL_MIN = 130;
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

// *** FUNCAO PRINCIPAL DO DEFENSOR ***
// Mantém posição no gol usando linha, bola e ultrassônicos como entradas do vetor de movimento
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



  float anguloIrBufferizado = -1.0f;
bool usandoBuffer = false;

  if (obterAnguloIrComBuffer(anguloIrBufferizado, usandoBuffer)) {
    anguloBola = anguloIrBufferizado;
    bolaDisponivel = true;
  } else if (cameraTemBolaValida()) {
    anguloBola = normalizarAngulo360((float)cameraBallAngle);
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
// SECAO 21 — SETUP: INICIALIZACAO DO HARDWARE E HANDSHAKE INICIAL
// =============================================================================

void setup() {
  // Inicializa seriais: debug (USB) e comunicação com a Cabeça
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX_CABECA, TX_CABECA);

  // Recupera da EEPROM a referência de bússola, papel e cor de gol
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_ADDR_BUSSOLA, headingBussolaSalvo);
  if (headingBussolaSalvo < 0 || headingBussolaSalvo >= 360) headingBussolaSalvo = 0;
  carregarPapelConfiguradoEEPROM();
  carregarCorGolEEPROM();

  // Configura o pino do kicker
  pinMode(KICKER_PIN, OUTPUT);
  digitalWrite(KICKER_PIN, LOW);

  // Inicializa toda a camada de motores e movimentacao pela biblioteca dedicada
  MotoresMovimentacaoConfig motoresCfg;
  motoresCfg.velocidadeMaxima = velocidade_maxima;
  motoresCfg.passoRampaPwm = PASSO_RAMPA_PWM;
  motoresCfg.ganhoGiroMisto = GANHO_GIRO_MISTO;
  inicializarMotoresMovimentacao(motoresCfg);

  // Inicializa I2C e display OLED
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!iniciarDisplayIHM()) {
    while (1) { delay(100); }
  }
  mostrarBootEtapaIHM("OLED", 20);
  mostrarBootEtapaIHM("I2C", 35);
  mostrarBootEtapaIHM("UART", 55);
  mostrarBootEtapaIHM("EEPROM", 75);

  // Tenta handshake inicial com a Cabeça por TIMEOUT_COM_MS
  unsigned long inicio = millis();
  while ((millis() - inicio) < TIMEOUT_COM_MS) {
    if (millis() - ultimoEnvioOi >= 300) {
      Serial1.println("oi"); ultimoEnvioOi = millis();
    }
    lerSerialCabeca();
    if (comunicacaoCabecaOK) break;
    delay(10);
  }

  mostrarBootEtapaIHM("CABECA", 100);

  enviarEstadoJogoParaCabeca(true);
  enviarReferenciaBussolaParaCabeca(true);
  desenharTelaAtual();
}


// =============================================================================
// SECAO 22 — LOOP PRINCIPAL
// =============================================================================

void loop() {
  // Heartbeat: mantém a serial com a Cabeça viva
  if (millis() - ultimoEnvioOi >= INTERVALO_OI_MS) {
    Serial1.println("oi"); ultimoEnvioOi = millis();
  }

  // Processa todas as entradas e envia configurações pendentes
  lerSerialCabeca();
  atualizarValidadeBussola();
  atualizarValidadeLinha();
  atualizarValidadeCamera();
  atualizarSozinhoLocal();
  atualizarPapelAutomaticoPorParceria();
  enviarEstadoJogoParaCabeca();
  enviarCorGolParaCabeca();
  enviarReferenciaBussolaParaCabeca();
  solicitarSensoresBrutosPe();
  if (!limiarLinhaSincronizado) solicitarLimiarLinhaPe();
  atualizarKicker();

  // Timeout de comunicação: derruba estado se Cabeça ficar silenciosa
  if ((millis() - ultimoRxCabeca) > TIMEOUT_COM_MS) {
    comunicacaoCabecaOK = false; bussolaValida = false;
  }

  if (executarPosicionamentoAlvo()) {
    static unsigned long ultimaTelaPos = 0;
    if ((millis() - ultimaTelaPos) > 120) {
      desenharTelaAtual(); ultimaTelaPos = millis();
    }
    return;
  }

  // Lógica de jogo: executa estratégia conforme papel atual
  if (estadoAtual == INICIAR && comunicacaoCabecaOK) {
    if (papelAtacante != papelAtacanteAnterior) {
      // Reinicia transição angular ao mudar de papel (evita carregar estado antigo)
      resetControleMovimentoAtacante();
      papelAtacanteAnterior = papelAtacante;
    }
    if (papelAtacante) atacante();
    else               defensor();
  } else {
    // Fora do modo de jogo: zera controles e para os motores
    alinhandoAgora = false; fugindoLinhaAgora = false;
    resetControleMovimentoAtacante();
    resetPidBussola();
    pararMotores();
  }

  // Redesenha o OLED com taxa limitada para evitar flicker
  static unsigned long ultimaTela = 0;
  if ((millis() - ultimaTela) > 120) {
    desenharTelaAtual(); ultimaTela = millis();
  }

 // delay(5);
}