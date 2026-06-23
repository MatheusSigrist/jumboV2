// =============================================================================
// LAYER 1 - 4: DATABASE INTEGRATION FOR MUSCULO.CPP
// IR Bruto → IR Previsto → Comando Movimento → Saída PWM
// =============================================================================
// Cada banco mantém 50 amostras em sliding window (FIFO circular)
// Nova entrada empurra a mais antiga para fora
// =============================================================================

#ifndef DATABASES_LAYER_HPP
#define DATABASES_LAYER_HPP

#include <Arduino.h>
#include <cstring>

// ============================================================================
// TEMPLATE: Circular Buffer com Sliding Window
// ============================================================================
template<typename T, uint16_t TAM_BUFFER>
class SlidingWindowBuffer {
private:
  T buffer[TAM_BUFFER];
  uint16_t indiceEscrita;
  uint16_t quantidadeAtual;
  portMUX_TYPE mutex;

public:
  SlidingWindowBuffer() 
    : indiceEscrita(0), quantidadeAtual(0), 
      mutex(portMUX_INITIALIZER_UNLOCKED) {
    memset(buffer, 0, sizeof(buffer));
  }

  void adicionar(const T& valor) {
    portENTER_CRITICAL(&mutex);
    buffer[indiceEscrita] = valor;
    indiceEscrita = (indiceEscrita + 1) % TAM_BUFFER;
    if (quantidadeAtual < TAM_BUFFER) quantidadeAtual++;
    portEXIT_CRITICAL(&mutex);
  }

  bool obterPorIndice(uint16_t idx, T& saida) const {
    if (idx >= quantidadeAtual) return false;
    portENTER_CRITICAL(&mutex);
    uint16_t indiceReal = (indiceEscrita + idx - quantidadeAtual + TAM_BUFFER) % TAM_BUFFER;
    saida = buffer[indiceReal];
    portEXIT_CRITICAL(&mutex);
    return true;
  }

  bool obterUltimo(T& saida) const {
    if (quantidadeAtual == 0) return false;
    portENTER_CRITICAL(&mutex);
    uint16_t indiceUltimo = (indiceEscrita - 1 + TAM_BUFFER) % TAM_BUFFER;
    saida = buffer[indiceUltimo];
    portEXIT_CRITICAL(&mutex);
    return true;
  }

  uint16_t obterQuantidade() const { return quantidadeAtual; }
  bool estaCheia() const { return quantidadeAtual == TAM_BUFFER; }
  bool estaVazio() const { return quantidadeAtual == 0; }

  uint16_t obterSnapshot(T* arr, uint16_t max) const {
    if (arr == nullptr) return 0;
    portENTER_CRITICAL(&mutex);
    uint16_t qtd = (quantidadeAtual < max) ? quantidadeAtual : max;
    for (uint16_t i = 0; i < qtd; i++) {
      uint16_t idx = (indiceEscrita + i - quantidadeAtual + TAM_BUFFER) % TAM_BUFFER;
      arr[i] = buffer[idx];
    }
    portEXIT_CRITICAL(&mutex);
    return qtd;
  }

  void limpar() {
    portENTER_CRITICAL(&mutex);
    indiceEscrita = 0;
    quantidadeAtual = 0;
    memset(buffer, 0, sizeof(buffer));
    portEXIT_CRITICAL(&mutex);
  }
};

// ============================================================================
// CAMADA 1: IR BRUTO (Raw Sensor Data)
// ============================================================================
struct DadosIrBruto {
  uint32_t timestamp_ms;
  int16_t  angulo_graus;
  uint16_t distancia_cm;
  uint8_t  confianca_percent;
  bool     valido;
};

// ============================================================================
// CAMADA 2: IR PREVISTO (Com Predição/Kalman)
// ============================================================================
struct DadosIrPrevisto {
  uint32_t timestamp_ms;
  int16_t  angulo_real_graus;
  int16_t  angulo_previsto_graus;
  int16_t  erro_predicao_graus;
  uint8_t  confianca_previsao;
  uint8_t  modelo_usado;
};

// ============================================================================
// CAMADA 3: COMANDO DE MOVIMENTO (Decision Layer)
// ============================================================================
struct DadosComandoMovimento {
  uint32_t timestamp_ms;
  float    angulo_desejado_graus;
  int16_t  velocidade_pwm;
  uint8_t  tipo_estrategia;        // 0=IR, 1=Linha, 2=Alinhamento, 3=Retorno
  uint8_t  papel_robo;             // 0=Defensor, 1=Atacante
  uint8_t  fonte_principal;        // 0=IR, 1=Câmera, 2=Bússola, 3=Linha
  bool     linha_detectada;
  bool     alinhando;
  int16_t  erro_alinhamento_graus;
};

// ============================================================================
// CAMADA 4: SAÍDA PWM REAL (Motor Output + Feedback)
// ============================================================================
struct DadosSaidaPwm {
  uint32_t timestamp_ms;
  int16_t  pwm_motor[4];
  float    angulo_executado_graus;
  float    velocidade_media_pwm;
  int16_t  erro_posicional_graus;
  uint8_t  qualidade_execucao;
};

// ============================================================================
// INSTÂNCIAS GLOBAIS DOS 4 BANCOS DE DADOS
// ============================================================================
static const uint16_t TAM_BUFFER_DB = 50;

SlidingWindowBuffer<DadosIrBruto, TAM_BUFFER_DB> dbIrBruto;
SlidingWindowBuffer<DadosIrPrevisto, TAM_BUFFER_DB> dbIrPrevisto;
SlidingWindowBuffer<DadosComandoMovimento, TAM_BUFFER_DB> dbComandoMovimento;
SlidingWindowBuffer<DadosSaidaPwm, TAM_BUFFER_DB> dbSaidaPwm;

// ============================================================================
// FUNÇÕES HELPER
// ============================================================================

uint8_t calcularConfiancaPrevisao(int16_t real, int16_t previsto) {
  int16_t erro = abs(real - previsto);
  if (erro < 2)    return 100;
  if (erro < 5)    return 90;
  if (erro < 10)   return 75;
  if (erro < 20)   return 50;
  if (erro < 45)   return 25;
  return 0;
}

uint8_t calcularQualidadeExecucao(int16_t erro_ang) {
  int16_t abs_erro = abs(erro_ang);
  if (abs_erro < 3)    return 0;
  if (abs_erro < 8)    return 20;
  if (abs_erro < 15)   return 40;
  if (abs_erro < 30)   return 60;
  if (abs_erro < 60)   return 80;
  return 100;
}

// ============================================================================
// ADICIONAR AOS BANCOS
// ============================================================================

void adicionarIrBruto(int16_t angulo, uint16_t dist, uint8_t confianca) {
  DadosIrBruto dado = {
    .timestamp_ms = millis(),
    .angulo_graus = angulo,
    .distancia_cm = dist,
    .confianca_percent = confianca,
    .valido = (angulo >= 0)
  };
  dbIrBruto.adicionar(dado);
}

void adicionarIrPrevisto(int16_t real, int16_t previsto, uint8_t modelo) {
  DadosIrPrevisto dado = {
    .timestamp_ms = millis(),
    .angulo_real_graus = real,
    .angulo_previsto_graus = previsto,
    .erro_predicao_graus = (int16_t)(real - previsto),
    .confianca_previsao = calcularConfiancaPrevisao(real, previsto),
    .modelo_usado = modelo
  };
  dbIrPrevisto.adicionar(dado);
}

void adicionarComandoMovimento(float angulo_des, int16_t vel, uint8_t estrat, 
                               bool temp_linha = false, bool temp_alinhando = false,
                               int16_t temp_erro = 0) {
  DadosComandoMovimento dado = {
    .timestamp_ms = millis(),
    .angulo_desejado_graus = normalizarAngulo360(angulo_des),
    .velocidade_pwm = constrain(vel, -255, 255),
    .tipo_estrategia = estrat,
    .papel_robo = (papelAtacante ? 1 : 0),
    .fonte_principal = (irDetectado ? 0 : (cameraTemBolaValida() ? 1 : 2)),
    .linha_detectada = temp_linha,
    .alinhando = temp_alinhando,
    .erro_alinhamento_graus = temp_erro
  };
  dbComandoMovimento.adicionar(dado);
}

void adicionarSaidaPwm(int16_t p1, int16_t p2, int16_t p3, int16_t p4,
                       float ang_exec, float vel_exec, int16_t erro_ang) {
  DadosSaidaPwm dado = {
    .timestamp_ms = millis(),
    .pwm_motor[0] = p1,
    .pwm_motor[1] = p2,
    .pwm_motor[2] = p3,
    .pwm_motor[3] = p4,
    .angulo_executado_graus = ang_exec,
    .velocidade_media_pwm = vel_exec,
    .erro_posicional_graus = erro_ang,
    .qualidade_execucao = calcularQualidadeExecucao(erro_ang)
  };
  dbSaidaPwm.adicionar(dado);
}

// ============================================================================
// FILTRO KALMAN PARA IR
// ============================================================================

struct KalmanFilterIr {
  float x, p, q, r, k;
  
  KalmanFilterIr() : x(0), p(1000), q(0.01f), r(25), k(0) {}
  
  float atualizar(float medicao) {
    if (medicao < 0) return x;
    p = p + q;
    k = p / (p + r);
    x = x + k * (medicao - x);
    p = (1 - k) * p;
    return x;
  }
};

static KalmanFilterIr kalmanIr;

int16_t filtroKalmanIr(float medicao) {
  float previsto = kalmanIr.atualizar(medicao);
  return (int16_t)normalizarAngulo360(previsto);
}

// ============================================================================
// EXPORTAR JSON (via Serial para análise)
// ============================================================================

void exportarDadosJson() {
  Serial.println("{");
  
  Serial.println("  \"ir_bruto\": [");
  DadosIrBruto arr1[TAM_BUFFER_DB];
  uint16_t qtd1 = dbIrBruto.obterSnapshot(arr1, TAM_BUFFER_DB);
  for (uint16_t i = 0; i < qtd1; i++) {
    Serial.printf("    {\"t\":%u,\"ang\":%d,\"conf\":%u}",
      arr1[i].timestamp_ms, arr1[i].angulo_graus, arr1[i].confianca_percent);
    if (i < qtd1 - 1) Serial.println(",");
    else Serial.println();
  }
  Serial.println("  ],");
  
  Serial.println("  \"ir_previsto\": [");
  DadosIrPrevisto arr2[TAM_BUFFER_DB];
  uint16_t qtd2 = dbIrPrevisto.obterSnapshot(arr2, TAM_BUFFER_DB);
  for (uint16_t i = 0; i < qtd2; i++) {
    Serial.printf("    {\"t\":%u,\"real\":%d,\"prev\":%d,\"err\":%d}",
      arr2[i].timestamp_ms, arr2[i].angulo_real_graus,
      arr2[i].angulo_previsto_graus, arr2[i].erro_predicao_graus);
    if (i < qtd2 - 1) Serial.println(",");
    else Serial.println();
  }
  Serial.println("  ],");
  
  Serial.println("  \"comando_movimento\": [");
  DadosComandoMovimento arr3[TAM_BUFFER_DB];
  uint16_t qtd3 = dbComandoMovimento.obterSnapshot(arr3, TAM_BUFFER_DB);
  for (uint16_t i = 0; i < qtd3; i++) {
    Serial.printf("    {\"t\":%u,\"ang\":%.1f,\"vel\":%d,\"est\":%u}",
      arr3[i].timestamp_ms, arr3[i].angulo_desejado_graus,
      arr3[i].velocidade_pwm, arr3[i].tipo_estrategia);
    if (i < qtd3 - 1) Serial.println(",");
    else Serial.println();
  }
  Serial.println("  ],");
  
  Serial.println("  \"saida_pwm\": [");
  DadosSaidaPwm arr4[TAM_BUFFER_DB];
  uint16_t qtd4 = dbSaidaPwm.obterSnapshot(arr4, TAM_BUFFER_DB);
  for (uint16_t i = 0; i < qtd4; i++) {
    Serial.printf("    {\"t\":%u,\"pwm\":[%d,%d,%d,%d],\"ang\":%.1f,\"q\":%u}",
      arr4[i].timestamp_ms,
      arr4[i].pwm_motor[0], arr4[i].pwm_motor[1],
      arr4[i].pwm_motor[2], arr4[i].pwm_motor[3],
      arr4[i].angulo_executado_graus,
      arr4[i].qualidade_execucao);
    if (i < qtd4 - 1) Serial.println(",");
    else Serial.println();
  }
  Serial.println("  ]");
  Serial.println("}");
}

#endif // DATABASES_LAYER_HPP
