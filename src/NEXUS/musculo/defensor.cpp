#include <Arduino.h>
#include "defensor.hpp"
#include "motores_movimentacao.hpp"

// =============================================================================
// TESTE DO DEFENSOR
// V9: LINHA + BUSSOLA + FUSAO W + SIG*D
// =============================================================================
//
// Convencao de angulos usada pelo projeto:
//   0   = frente
//   90  = direita
//   180 = tras
//   270 = esquerda
//
// Neste teste:
// - a linha controla SOMENTE a translacao X/Y;
// - a bussola controla SOMENTE o giro;
// - os dois controles atuam ao mesmo tempo;
// - o vetor da linha e filtrado no referencial FIXO do campo;
// - o giro do robo pela bussola nao desloca artificialmente o filtro da linha;
// - a linha continua usando exatamente seu vetor/direcao e intensidade atuais;
// - a bola vira um vetor lateral: 0..180 -> 90 graus, 181..360 -> 270 graus;
// - a fusao segue F = W + SIG_BOLA_DEFENSOR * D;
// - SIG_BOLA_DEFENSOR e ajustado manualmente: maior = mais bola, menor = mais linha;
// - ultrassons, retorno ao gol e avancos especiais NAO participam.
// =============================================================================

// -----------------------------------------------------------------------------
// AJUSTES DA LINHA
// -----------------------------------------------------------------------------

// Filtro ADAPTATIVO:
// - longe do centro responde quase instantaneamente;
// - perto do centro filtra mais para nao perseguir ruido.
static constexpr float LINHA_ALPHA_PERTO = 0.28f;
static constexpr float LINHA_ALPHA_LONGE = 0.82f;
static constexpr float LINHA_MAG_ALPHA_LONGE = 0.30f;

// Abaixo desta magnitude a linha e considerada suficientemente centralizada
// para nao contribuir na mistura com a bola.
static constexpr float LINHA_MAG_STOP = 0.055f;

// Curva de velocidade SEM piso fixo de PWM.
// Assim a forca realmente tende a zero quando chega ao centro.
static constexpr float LINHA_MAG_PWM_MAX = 0.30f;
static constexpr float LINHA_CURVA_EXP = 1.25f;
static constexpr int LINHA_PWM_MAX = 255;

// Se uma das zonas piscar logo depois de termos A+B, nao podemos interpretar
// imediatamente isso como erro maximo e mandar 255. Seguramos por alguns ms
// o ultimo vetor fino e deixamos ele decair.
static constexpr unsigned long LINHA_GRACA_PERDA_UMA_ZONA_MS = 120;
static constexpr float LINHA_DECAIMENTO_GRACA = 0.82f;

// Uma unica zona mantida alem da janela de graca significa deslocamento real:
// ai sim queremos recuperacao forte. O erro da linha pode chegar a 1.0,
// mas, se houver bola, a fusao V8 ainda preserva o peso minimo da bola.
static constexpr float LINHA_GANHO_UMA_ZONA = 1.00f;

// Retencao curta para uma perda de pacote/leitura isolada nao desmontar o vetor.
static constexpr unsigned long LINHA_RETENCAO_MS = 40;

// -----------------------------------------------------------------------------
// AJUSTES DA BUSSOLA
// -----------------------------------------------------------------------------

static constexpr float BUSSOLA_KP = 1.35f;
static constexpr float BUSSOLA_KD = 0.025f;
static constexpr float BUSSOLA_D_MAX = 25.0f;
static constexpr float BUSSOLA_DEADZONE_GRAUS = 1.5f;
static constexpr int BUSSOLA_SAIDA_MAX = 140;

// Sinal que converte o erro da bussola em orientacao do robo no referencial
// fixo da linha/campo. Comece com +1.0f.
// Se ao girar o robo para a DIREITA a compensacao da linha piorar/inverter,
// troque SOMENTE para -1.0f.
static constexpr float BUSSOLA_SINAL_REFERENCIAL_LINHA = +1.0f;

// -----------------------------------------------------------------------------
// AJUSTES DA BOLA / IR
// -----------------------------------------------------------------------------

// Intensidade base do vetor D da bola.
static constexpr int BOLA_VELOCIDADE_PWM = 200;

// AJUSTE MANUAL PRINCIPAL DA FUSAO:
//   0.00 -> bola nao influencia quando linha + bola existem
//   0.25 -> linha tende a dominar
//   0.50 -> influencia moderada da bola
//   1.00 -> vetor da bola entra com intensidade base completa
//   1.50 / 2.00 -> bola passa a dominar cada vez mais
//
// Pode alterar este valor manualmente durante os testes.
static float SIG_BOLA_DEFENSOR = 1.0f;

// =============================================================================
// ESTADOS INTERNOS
// =============================================================================

struct ErroLinhaVetorial {
  float x;
  float y;
  float magnitude;
  bool valido;
  bool duasZonas;
};

static float linhaErroXCampoFiltrado = 0.0f;
static float linhaErroYCampoFiltrado = 0.0f;
static bool linhaFiltroInicializado = false;
static unsigned long linhaUltimaDuasZonasMs = 0;

static float bussolaErroAnterior = 0.0f;
static unsigned long bussolaUltimoPidMs = 0;
static bool bussolaPidInicializado = false;

// =============================================================================
// AUXILIARES
// =============================================================================

static float normalizar360Defensor(float angulo) {
  while (angulo >= 360.0f) angulo -= 360.0f;
  while (angulo < 0.0f) angulo += 360.0f;
  return angulo;
}

static float calcularMagnitude(float x, float y) {
  return sqrtf((x * x) + (y * y));
}

// Mantem a mesma convencao angular usada no restante do robo:
// atan2(X, Y): 0=frente, 90=direita, 180=tras, 270=esquerda.
static float calcularAnguloDoVetor(float x, float y) {
  if (fabsf(x) < 0.00001f && fabsf(y) < 0.00001f) {
    return 0.0f;
  }

  return normalizar360Defensor(
    atan2f(x, y) * 180.0f / PI
  );
}

// -----------------------------------------------------------------------------
// TROCA DE REFERENCIAL: ROBO <-> CAMPO
// -----------------------------------------------------------------------------
// Convencao: 0=frente, 90=direita, angulos positivos no sentido horario.
//
// orientacaoRoboCampoGraus = quanto o robo esta girado em relacao a referencia
// desejada da bussola. Ao converter para o campo, a rotacao do robo deixa de
// contaminar o filtro temporal da linha.

static void vetorRoboParaCampo(float xRobo,
                               float yRobo,
                               float orientacaoRoboCampoGraus,
                               float &xCampo,
                               float &yCampo) {
  const float psi = orientacaoRoboCampoGraus * PI / 180.0f;
  const float c = cosf(psi);
  const float s = sinf(psi);

  xCampo = (xRobo * c) + (yRobo * s);
  yCampo = (yRobo * c) - (xRobo * s);
}

static void vetorCampoParaRobo(float xCampo,
                               float yCampo,
                               float orientacaoRoboCampoGraus,
                               float &xRobo,
                               float &yRobo) {
  const float psi = orientacaoRoboCampoGraus * PI / 180.0f;
  const float c = cosf(psi);
  const float s = sinf(psi);

  xRobo = (xCampo * c) - (yCampo * s);
  yRobo = (yCampo * c) + (xCampo * s);
}

// =============================================================================
// RESET DOS CONTROLES
// =============================================================================

void resetPidLinha() {
  linhaErroXCampoFiltrado = 0.0f;
  linhaErroYCampoFiltrado = 0.0f;
  linhaFiltroInicializado = false;
  linhaUltimaDuasZonasMs = 0;
}

void resetPidZimBussola() {
  bussolaErroAnterior = 0.0f;
  bussolaUltimoPidMs = 0;
  bussolaPidInicializado = false;
}

// =============================================================================
// AQUISICAO DA LINHA
// =============================================================================

static void obterZonasLinha(bool &zonaAValida,
                            float &anguloA,
                            bool &zonaBValida,
                            float &anguloB,
                            unsigned long agora) {

  const bool zonaAAtual =
    linhaZonaAValida && (anguloLinhaZonaA >= 0.0f);

  const bool zonaBAtual =
    linhaZonaBValida && (anguloLinhaZonaB >= 0.0f);

  const bool zonaARetida =
    !zonaAAtual &&
    (ultimoAnguloLinhaZonaAValido >= 0.0f) &&
    (ultimoRxLinhaZonaAMs > 0) &&
    ((agora - ultimoRxLinhaZonaAMs) <= LINHA_RETENCAO_MS);

  const bool zonaBRetida =
    !zonaBAtual &&
    (ultimoAnguloLinhaZonaBValido >= 0.0f) &&
    (ultimoRxLinhaZonaBMs > 0) &&
    ((agora - ultimoRxLinhaZonaBMs) <= LINHA_RETENCAO_MS);

  zonaAValida = zonaAAtual || zonaARetida;
  zonaBValida = zonaBAtual || zonaBRetida;

  anguloA = zonaAAtual
    ? anguloLinhaZonaA
    : (zonaARetida ? ultimoAnguloLinhaZonaAValido : -1.0f);

  anguloB = zonaBAtual
    ? anguloLinhaZonaB
    : (zonaBRetida ? ultimoAnguloLinhaZonaBValido : -1.0f);
}

// =============================================================================
// GEOMETRIA DA LINHA
// =============================================================================
//
// Cada angulo valido vira um vetor unitario apontando PARA a linha.
//
// Exemplo ideal:
//   A = 90 graus  -> (+1, 0)
//   B = 270 graus -> (-1, 0)
//
// Media:
//   (0, 0) -> robo centralizado.
//
// Se os vetores nao se cancelam, a resultante aponta para o lado para onde
// o centro do robo deve se deslocar.
// =============================================================================

static ErroLinhaVetorial calcularErroLinhaVetorial(bool zonaAValida,
                                                    float anguloA,
                                                    bool zonaBValida,
                                                    float anguloB) {
  float x = 0.0f;
  float y = 0.0f;
  int quantidade = 0;

  if (zonaAValida) {
    const float radA = normalizar360Defensor(anguloA) * PI / 180.0f;
    x += sinf(radA);
    y += cosf(radA);
    quantidade++;
  }

  if (zonaBValida) {
    const float radB = normalizar360Defensor(anguloB) * PI / 180.0f;
    x += sinf(radB);
    y += cosf(radB);
    quantidade++;
  }

  if (quantidade == 0) {
    return {0.0f, 0.0f, 0.0f, false, false};
  }

  x /= (float)quantidade;
  y /= (float)quantidade;

  // Somente uma zona: sabemos para que lado esta a linha, mas nao o erro fino.
  // Para este teste tratamos isso como erro grande e reagimos com forca.
  if (quantidade == 1) {
    x *= LINHA_GANHO_UMA_ZONA;
    y *= LINHA_GANHO_UMA_ZONA;
  }

  return {
    x,
    y,
    calcularMagnitude(x, y),
    true,
    quantidade == 2
  };
}

// =============================================================================
// CONTROLE DA LINHA - DIRECAO VETORIAL + VELOCIDADE NAO LINEAR
// =============================================================================

static int calcularVelocidadeLinhaPorMagnitude(float magnitude) {
  if (magnitude <= LINHA_MAG_STOP) {
    return 0;
  }

  // 0 no limite de parada e 1 na regiao de erro grande.
  float t = (magnitude - LINHA_MAG_STOP) /
            (LINHA_MAG_PWM_MAX - LINHA_MAG_STOP);
  t = constrain(t, 0.0f, 1.0f);

  const float curva = powf(t, LINHA_CURVA_EXP);
  const float pwm = curva * (float)LINHA_PWM_MAX;

  return (int)roundf(constrain(pwm, 0.0f, (float)LINHA_PWM_MAX));
}

// Peso usado na fusao linha + bola:
//   0.00 = linha perfeitamente centralizada -> bola domina
//   0.30 = aproximadamente 70% alinhado     -> 30% linha / 70% bola
//   1.00 = erro maximo                      -> linha domina
static float calcularPesoLinhaPorMagnitude(float magnitude,
                                           bool umaZonaPersistente) {
  if (umaZonaPersistente) {
    return 1.0f;
  }

  if (magnitude <= LINHA_MAG_STOP) {
    return 0.0f;
  }

  return constrain(magnitude, 0.0f, 1.0f);
}

// Calcula o comando da linha MESMO quando o erro e pequeno.
// Isso e necessario porque agora nao existe mais a troca binaria
// "linha OU bola": o erro da linha vira um peso continuo da fusao.
static bool calcularComandoLinha(const ErroLinhaVetorial &erroBruto,
                                 float orientacaoRoboCampoGraus,
                                 unsigned long agora,
                                 float &anguloMovimento,
                                 int &velocidadePwm,
                                 float &pesoLinha) {

  anguloMovimento = 0.0f;
  velocidadePwm = 0;
  pesoLinha = 0.0f;

  if (!erroBruto.valido) {
    resetPidLinha();
    return false;
  }

  // A leitura nasce no referencial do ROBO. Converte para o CAMPO antes
  // de filtrar para que o giro causado pela bussola nao contamine a linha.
  float erroXCampo = 0.0f;
  float erroYCampo = 0.0f;
  vetorRoboParaCampo(
    erroBruto.x,
    erroBruto.y,
    orientacaoRoboCampoGraus,
    erroXCampo,
    erroYCampo
  );

  float alvoX = erroXCampo;
  float alvoY = erroYCampo;
  bool usandoGracaUmaZona = false;

  if (erroBruto.duasZonas) {
    linhaUltimaDuasZonasMs = agora;
  }
  else if (linhaFiltroInicializado &&
           linhaUltimaDuasZonasMs > 0 &&
           (agora - linhaUltimaDuasZonasMs) <= LINHA_GRACA_PERDA_UMA_ZONA_MS) {
    // Uma zona acabou de piscar. Nao transforma um pequeno erro fino em
    // erro maximo instantaneamente.
    alvoX = linhaErroXCampoFiltrado * LINHA_DECAIMENTO_GRACA;
    alvoY = linhaErroYCampoFiltrado * LINHA_DECAIMENTO_GRACA;
    usandoGracaUmaZona = true;
  }

  const float magnitudeAlvo = calcularMagnitude(alvoX, alvoY);

  // Filtro adaptativo: rapido longe, suave perto.
  float tAlpha = magnitudeAlvo / LINHA_MAG_ALPHA_LONGE;
  tAlpha = constrain(tAlpha, 0.0f, 1.0f);
  float alpha = LINHA_ALPHA_PERTO +
                tAlpha * (LINHA_ALPHA_LONGE - LINHA_ALPHA_PERTO);

  // Uma unica zona persistente = deslocamento real. Resposta rapida.
  const bool umaZonaPersistente =
    !erroBruto.duasZonas && !usandoGracaUmaZona;

  if (umaZonaPersistente) {
    alpha = LINHA_ALPHA_LONGE;
  }

  if (!linhaFiltroInicializado) {
    linhaErroXCampoFiltrado = alvoX;
    linhaErroYCampoFiltrado = alvoY;
    linhaFiltroInicializado = true;
  } else {
    linhaErroXCampoFiltrado += alpha * (alvoX - linhaErroXCampoFiltrado);
    linhaErroYCampoFiltrado += alpha * (alvoY - linhaErroYCampoFiltrado);
  }

  const float magnitudeFiltrada =
    calcularMagnitude(linhaErroXCampoFiltrado, linhaErroYCampoFiltrado);

  pesoLinha = calcularPesoLinhaPorMagnitude(
    magnitudeFiltrada,
    umaZonaPersistente
  );

  // Centralizado: a linha deixa de contribuir para a translacao.
  if (pesoLinha <= 0.0f) {
    return false;
  }

  // Volta do referencial do CAMPO para o referencial atual do ROBO.
  float comandoXRobo = 0.0f;
  float comandoYRobo = 0.0f;
  vetorCampoParaRobo(
    linhaErroXCampoFiltrado,
    linhaErroYCampoFiltrado,
    orientacaoRoboCampoGraus,
    comandoXRobo,
    comandoYRobo
  );

  const float magnitudeComandoRobo =
    calcularMagnitude(comandoXRobo, comandoYRobo);

  if (magnitudeComandoRobo < 0.00001f) {
    pesoLinha = 0.0f;
    return false;
  }

  anguloMovimento = calcularAnguloDoVetor(
    comandoXRobo,
    comandoYRobo
  );

  if (umaZonaPersistente) {
    velocidadePwm = LINHA_PWM_MAX;
  } else {
    velocidadePwm = calcularVelocidadeLinhaPorMagnitude(
      magnitudeFiltrada
    );
  }

  return velocidadePwm > 0;
}

// =============================================================================
// CONTROLE DA BOLA / IR
// =============================================================================
//
// Novo algoritmo pedido:
//   bola entre   0 e 180 graus -> vetor D = 90 graus
//   bola entre 181 e 360 graus -> vetor D = 270 graus
//
// O angulo bruto serve apenas para decidir o LADO da bola.
// A fusao posterior usa esse vetor D junto ao vetor W da linha.
// =============================================================================

static bool obterDirecaoMovimentoBola(float &anguloMovimentoBola) {
  float anguloBola = -1.0f;

  if (!obterAnguloIrDisponivel(anguloBola)) {
    return false;
  }

  // Preserva explicitamente o caso 360 graus conforme a regra pedida.
  if (anguloBola >= 0.0f && anguloBola <= 180.0f) {
    anguloMovimentoBola = 90.0f;
    return true;
  }

  if (anguloBola > 180.0f && anguloBola <= 360.0f) {
    anguloMovimentoBola = 270.0f;
    return true;
  }

  // Fallback para qualquer valor fora da faixa nominal.
  const float a = normalizar360Defensor(anguloBola);
  anguloMovimentoBola = (a <= 180.0f) ? 90.0f : 270.0f;
  return true;
}

// =============================================================================
// NOVA FUSAO VETORIAL: F = W + SIG * D
// =============================================================================
//
// W = vetor atual calculado pela LINHA.
//     A direcao e anguloLinha e sua intensidade real e velocidadeLinha.
//
// D = vetor lateral definido pela BOLA.
//     0..180 -> 90 graus
//     181..360 -> 270 graus
//     intensidade base = BOLA_VELOCIDADE_PWM
//
// SIG = SIG_BOLA_DEFENSOR, ajustado manualmente.
//
// IMPORTANTE:
// Nao fazemos "anguloLinha + SIG*anguloBola" porque angulos nao podem ser
// somados diretamente. Somamos as componentes X/Y dos dois vetores:
//
//   Fx = Wx + SIG*Dx
//   Fy = Wy + SIG*Dy
//
// e somente no final convertemos F de volta para um angulo.
// =============================================================================

static bool combinarLinhaEBolaNovo(float anguloLinha,
                                   int velocidadeLinha,
                                   bool linhaDisponivel,
                                   float anguloBola,
                                   int velocidadeBola,
                                   bool bolaDisponivel,
                                   float sigBola,
                                   float &anguloFinal,
                                   int &velocidadeFinal) {
  anguloFinal = 0.0f;
  velocidadeFinal = 0;

  if (!linhaDisponivel && !bolaDisponivel) {
    return false;
  }

  // Sem bola: preserva EXATAMENTE o comando atual da linha.
  if (linhaDisponivel && !bolaDisponivel) {
    anguloFinal = normalizar360Defensor(anguloLinha);
    velocidadeFinal = constrain(velocidadeLinha, 0, 255);
    return velocidadeFinal > 0;
  }

  // Sem linha: segue a bola normalmente. SIG serve como relacao linha/bola,
  // portanto nao reduz a velocidade quando nao existe vetor W para competir.
  if (!linhaDisponivel && bolaDisponivel) {
    anguloFinal = normalizar360Defensor(anguloBola);
    velocidadeFinal = constrain(velocidadeBola, 0, 255);
    return velocidadeFinal > 0;
  }

  // Linha + bola presentes.
  const float sig = (sigBola < 0.0f) ? 0.0f : sigBola;

  const float radLinha =
    normalizar360Defensor(anguloLinha) * PI / 180.0f;

  const float radBola =
    normalizar360Defensor(anguloBola) * PI / 180.0f;

  // W usa a intensidade que o controlador da linha JA calculou.
  const float forcaLinha =
    constrain((float)velocidadeLinha, 0.0f, 255.0f);

  // D recebe a intensidade base da bola multiplicada por SIG.
  // Nao limitamos a 255 antes da soma, pois SIG e justamente a autoridade
  // relativa da bola. Limitamos apenas o vetor FINAL enviado aos motores.
  const float forcaBola =
    constrain((float)velocidadeBola, 0.0f, 255.0f) * sig;

  const float xLinha = sinf(radLinha) * forcaLinha;
  const float yLinha = cosf(radLinha) * forcaLinha;

  const float xBola = sinf(radBola) * forcaBola;
  const float yBola = cosf(radBola) * forcaBola;

  const float xFinal = xLinha + xBola;
  const float yFinal = yLinha + yBola;

  const float magnitudeFinal =
    calcularMagnitude(xFinal, yFinal);

  // Se dois vetores opostos se anularem quase perfeitamente, nao escolhemos
  // uma direcao aleatoria por ruido numerico.
  if (magnitudeFinal < 0.5f) {
    return false;
  }

  anguloFinal =
    calcularAnguloDoVetor(xFinal, yFinal);

  velocidadeFinal =
    (int)roundf(
      constrain(magnitudeFinal, 0.0f, 255.0f)
    );

  return velocidadeFinal > 0;
}

// =============================================================================
// PD DA BUSSOLA
// =============================================================================

float PIDZIMBUSSOLANOVINHA_DEFENSOR(float erroBussola) {
  const unsigned long agora = millis();

  if (fabsf(erroBussola) <= BUSSOLA_DEADZONE_GRAUS) {
    bussolaErroAnterior = erroBussola;
    bussolaUltimoPidMs = agora;
    bussolaPidInicializado = true;
    return 0.0f;
  }

  float derivada = 0.0f;

  if (bussolaPidInicializado && bussolaUltimoPidMs > 0) {
    float dt = (agora - bussolaUltimoPidMs) / 1000.0f;

    if (dt >= 0.002f && dt <= 0.10f) {
      derivada = (erroBussola - bussolaErroAnterior) / dt;
    }
  }

  bussolaErroAnterior = erroBussola;
  bussolaUltimoPidMs = agora;
  bussolaPidInicializado = true;

  const float termoD = constrain(
    BUSSOLA_KD * derivada,
    -BUSSOLA_D_MAX,
    BUSSOLA_D_MAX
  );

  return (BUSSOLA_KP * erroBussola) + termoD;
}

static int calcularComandoGiroBussola(float erroBussola) {
  return constrain(
    (int)roundf(
      -PIDZIMBUSSOLANOVINHA_DEFENSOR(erroBussola)
    ),
    -BUSSOLA_SAIDA_MAX,
    BUSSOLA_SAIDA_MAX
  );
}

// =============================================================================
// LOOP DO DEFENSOR - LINHA + BOLA + BUSSOLA
// =============================================================================

void defensor() {
  const unsigned long agora = millis();

  // ---------------------------------------------------------------------------
  // 1. BUSSOLA: SEMPRE ATIVA
  // ---------------------------------------------------------------------------
  if (!bussolaTemReferenciaValida()) {
    resetPidZimBussola();
    resetPidLinha();
    girarNoEixo(0);
    return;
  }

  const float erroBussola = calcularErroReferenciaBussola();
  const int cmdGiroBussola = calcularComandoGiroBussola(erroBussola);

  const float orientacaoRoboCampoGraus =
    BUSSOLA_SINAL_REFERENCIAL_LINHA * erroBussola;

  // ---------------------------------------------------------------------------
  // 2. LE A LINHA
  // ---------------------------------------------------------------------------
  bool zonaAValida = false;
  bool zonaBValida = false;
  float anguloA = -1.0f;
  float anguloB = -1.0f;

  obterZonasLinha(
    zonaAValida,
    anguloA,
    zonaBValida,
    anguloB,
    agora
  );

  const ErroLinhaVetorial erroLinha =
    calcularErroLinhaVetorial(
      zonaAValida,
      anguloA,
      zonaBValida,
      anguloB
    );

  // ---------------------------------------------------------------------------
  // 3. CALCULA VETOR / PESO DA LINHA
  // ---------------------------------------------------------------------------
  float anguloLinha = 0.0f;
  int velocidadeLinha = 0;
  float pesoLinha = 0.0f; // mantido pelo controlador da linha; nao e o SIG da bola

  const bool linhaDisponivel =
    calcularComandoLinha(
      erroLinha,
      orientacaoRoboCampoGraus,
      agora,
      anguloLinha,
      velocidadeLinha,
      pesoLinha
    );

  // ---------------------------------------------------------------------------
  // 4. CALCULA VETOR DA BOLA
  // ---------------------------------------------------------------------------
  float anguloBola = 0.0f;
  const bool bolaDisponivel =
    obterDirecaoMovimentoBola(anguloBola);

  // ---------------------------------------------------------------------------
  // 5. FUSAO NOVA: F = W + SIG * D
  // ---------------------------------------------------------------------------
  // Quando linha + bola existem:
  //   F = W + SIG_BOLA_DEFENSOR * D
  //
  // Maior SIG -> resultado puxado mais para a bola.
  // Menor SIG -> resultado puxado mais para a linha.
  //
  // Somente linha -> preserva o comando da linha.
  // Somente bola  -> segue a bola.
  // Nenhum        -> somente bussola.
  float anguloFinal = 0.0f;
  int velocidadeFinal = 0;

  const bool temTranslacao =
    combinarLinhaEBolaNovo(
      anguloLinha,
      velocidadeLinha,
      linhaDisponivel,
      anguloBola,
      BOLA_VELOCIDADE_PWM,
      bolaDisponivel,
      SIG_BOLA_DEFENSOR,
      anguloFinal,
      velocidadeFinal
    );

  if (temTranslacao) {
    seguirDirecaoComGiroLaterais(
      anguloFinal,
      velocidadeFinal,
      cmdGiroBussola
    );
    return;
  }

  // Sem vetor translacional: a bussola continua obrigatoriamente alinhando.
  girarNoEixo(cmdGiroBussola);
}
