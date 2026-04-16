// Arquivo principal da placa Cabeca.
// Funcao: concentrador de comunicacao entre Musculo, Olho e Pe,
// leitura de botoes e bussola, e repasse de dados para o Musculo.
// Entrada: serial das placas, botoes fisicos, chave do kicker, I2C bussola.
// Saida: mensagens de estado/dados para Musculo e comandos para Olho.
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_QMC5883P.h>

#define RX_MUSCULO 44
#define TX_MUSCULO 43

#define BYTE_INICIA 0xAA
#define BYTE_PARA 0x55
#define ID_PLACA_OLHO 0x01
#define ID_PLACA_PE 0x02

#define RX_OLHO 6
#define TX_OLHO 7
#define RX_PE 4
#define TX_PE 5
#define BAUD_PE_CABECA 19200

#define BOTAO_1 3
#define BOTAO_2 37
#define BOTAO_3 46
#define KICKER_PIN 45
#define I2C_SDA 8
#define I2C_SCL 9

const unsigned long DEBOUNCE_BOTAO_MS = 180;
const unsigned long INTERVALO_OI_MS = 1000;
const unsigned long INTERVALO_BUSSOLA_MS = 300;

HardwareSerial SerialMusculo(0);
HardwareSerial SerialOlho(1);
HardwareSerial SerialPe(2);
Adafruit_QMC5883P compass;

struct PacoteOlho {
  int16_t uD;
  int16_t uE;
  int16_t uF;
  int16_t uT;
  int16_t angulo;
  int16_t intensidade;
  int16_t erroGol;
  uint16_t pixelsGol;
  uint8_t golDetectado;
  uint8_t cameraOK;  // 1 = camera enviando dados, 0 = sem sinal
};

struct PacotePe {
  int16_t angulo;
};

struct PacoteEstado {
  bool sozinho;
  bool atacante;
  bool corGolAzul;
};

String bufferEntrada = "";
unsigned long ultimoByteMs = 0;
unsigned long ultimoEventoBotaoMs = 0;
unsigned long ultimoEnvioOiMs = 0;
unsigned long ultimoEnvioIrMs = 0;
unsigned long ultimoEnvioBussolaMs = 0;
unsigned long ultimoEnvioBussolaMusculoMs = 0;
unsigned long ultimoEnvioGolMusculoMs = 0;
unsigned long ultimoEnvioLinhaMusculoMs = 0;
unsigned long ultimoEnvioIntMusculoMs = 0;
unsigned long ultimoEnvioUltraMusculoMs = 0;
unsigned long ultimoEnvioEstadoOlhoMs = 0;
unsigned long ultimoEnvioKickerMusculoMs = 0;

bool botao1Anterior = HIGH;
bool botao2Anterior = HIGH;
bool botao3Anterior = HIGH;
bool comunicacaoMusculoOK = false;
bool comunicacaoOlhoOK = false;
bool comunicacaoPeOK = false;
bool corGolAzulCfg = false;

String bufferOlho = "";
String bufferPe = "";
unsigned long ultimoPingOlhoMs = 0;
unsigned long ultimoPingPeMs = 0;
unsigned long ultimoRxOlhoMs = 0;
int16_t ultimoAnguloIrX10 = -10;
int16_t ultimaIntensidadeIrX10 = 0;
int16_t ultimoUltraDX10 = -10;
int16_t ultimoUltraEX10 = -10;
int16_t ultimoUltraFX10 = -10;
int16_t ultimoUltraTX10 = -10;
int16_t ultimoAnguloLinhaX10 = -10;
float ultimoErroGolGraus = 0.0f;
uint16_t ultimoPixelsGol = 0;
bool golDetectadoOlho = false;
bool cameraOlhoOK = false;  // camera esta se comunicando com o olho
bool kickerAtivado = false;
int ultimoKickerEnviado = -1;
float ultimoHeadingBussola = 0.0f;
float ultimoX360 = 0.0f;
float ultimoY360 = 0.0f;
bool bussolaOK = false;
float rawXFiltrado = 0.0f;
float rawYFiltrado = 0.0f;
bool filtroXYInicializado = false;

int16_t minX = 32767;
int16_t maxX = -32768;
int16_t minY = 32767;
int16_t maxY = -32768;

const unsigned long INTERVALO_ENVIO_IR_MS = 120;
const unsigned long INTERVALO_ENVIO_BUSSOLA_MS = 120;
const unsigned long INTERVALO_ENVIO_GOL_MS = 120;
const unsigned long INTERVALO_ENVIO_LINHA_MS = 20;
const unsigned long INTERVALO_ENVIO_INT_MS = 120;
const unsigned long INTERVALO_ENVIO_ULTRA_MS = 120;
const unsigned long INTERVALO_ENVIO_KICKER_MS = 120;
const unsigned long INTERVALO_ENVIO_ESTADO_OLHO_MS = 700;
const float ALPHA_FILTRO_XY = 0.18f;
const int16_t INTENSIDADE_MINIMA_IR_X10 = 80;  // 8.0
const unsigned long TIMEOUT_DADO_OLHO_MS = 500;

void resetLeituraIrOlho() {
  ultimoAnguloIrX10 = -10;
  ultimaIntensidadeIrX10 = 0;
}

// Normaliza qualquer angulo para a faixa [0, 360).
float normalizarAngulo360(float anguloGraus) {
  if (isnan(anguloGraus) || isinf(anguloGraus)) {
    return 0.0f;
  }

  float resultado = fmod(anguloGraus, 360.0f);
  if (resultado < 0.0f) {
    resultado += 360.0f;
  }
  if (resultado >= 360.0f) {
    resultado -= 360.0f;
  }
  return resultado;
}

// Inicializa e configura a bussola QMC5883P para leitura continua.
bool iniciarBussola() {
  if (!compass.begin()) {
    return false;
  }

  compass.setRange(QMC5883P_RANGE_8G);
  compass.setMode(QMC5883P_MODE_CONTINUOUS);
  compass.setODR(QMC5883P_ODR_50HZ);
  compass.setOSR(QMC5883P_OSR_8);
  compass.setDSR(QMC5883P_DSR_1);
  minX = 32767;
  maxX = -32768;
  minY = 32767;
  maxY = -32768;
  filtroXYInicializado = false;
  return true;
}

// Atualiza limites min/max dos eixos para calibracao dinamica.
void atualizarCalibracaoXY(int16_t rawX, int16_t rawY) {
  if (rawX < minX) minX = rawX;
  if (rawX > maxX) maxX = rawX;
  if (rawY < minY) minY = rawY;
  if (rawY > maxY) maxY = rawY;
}

// Mapeia um eixo calibrado para escala normalizada de 0 a 360 graus.
float eixoPara360(int16_t valor, int16_t minValor, int16_t maxValor) {
  int32_t faixa = (int32_t)maxValor - (int32_t)minValor;
  if (faixa < 10) {
    return 0.0f;
  }

  float norm01 = ((float)valor - (float)minValor) / (float)faixa;
  if (norm01 < 0.0f) norm01 = 0.0f;
  if (norm01 > 1.0f) norm01 = 1.0f;
  return norm01 * 360.0f;
}

// Le a bussola, aplica filtro, atualiza heading e publica debug no serial.
void atualizarBussola() {
  if (!bussolaOK) {
    bussolaOK = iniciarBussola();
    if (bussolaOK) {
      Serial.println("QMC5883P conectada na cabeca.");
    }
    return;
  }

  if ((millis() - ultimoEnvioBussolaMs) < INTERVALO_BUSSOLA_MS) {
    return;
  }

  int16_t rawX = 0;
  int16_t rawY = 0;
  int16_t rawZ = 0;
  if (!compass.getRawMagnetic(&rawX, &rawY, &rawZ)) {
    bussolaOK = false;
    return;
  }

  if (!filtroXYInicializado) {
    rawXFiltrado = (float)rawX;
    rawYFiltrado = (float)rawY;
    filtroXYInicializado = true;
  } else {
    rawXFiltrado = (1.0f - ALPHA_FILTRO_XY) * rawXFiltrado + ALPHA_FILTRO_XY * (float)rawX;
    rawYFiltrado = (1.0f - ALPHA_FILTRO_XY) * rawYFiltrado + ALPHA_FILTRO_XY * (float)rawY;
  }

  int16_t rawXSuave = (int16_t)lroundf(rawXFiltrado);
  int16_t rawYSuave = (int16_t)lroundf(rawYFiltrado);

  atualizarCalibracaoXY(rawXSuave, rawYSuave);

  ultimoX360 = eixoPara360(rawXSuave, minX, maxX);
  ultimoY360 = eixoPara360(rawYSuave, minY, maxY);

  float xCentro = ultimoX360 - 180.0f;
  float yCentro = ultimoY360 - 180.0f;
  ultimoHeadingBussola = normalizarAngulo360(atan2(yCentro, xCentro) * 180.0f / PI);

  Serial.print("BUS X360:");
  Serial.print(ultimoX360, 1);
  Serial.print(" Y360:");
  Serial.print(ultimoY360, 1);
  Serial.print(" HDG:");
  Serial.println(ultimoHeadingBussola, 1);

  ultimoEnvioBussolaMs = millis();
}

// Processa comandos textuais vindos do Musculo (handshake e configuracoes).
void processarMensagem(String msg) {
  msg.trim();
  msg.toLowerCase();

  if (msg == "oi") {
    comunicacaoMusculoOK = true;
    SerialMusculo.println("oi");
    Serial.println("Musculo respondeu handshake");
  } else if (msg.startsWith("cfg:gol:")) {
    String v = msg.substring(8);
    v.trim();

    bool novaCorAzul = corGolAzulCfg;
    if (v == "1" || v == "azul") {
      novaCorAzul = true;
    } else if (v == "0" || v == "amarelo") {
      novaCorAzul = false;
    }

    corGolAzulCfg = novaCorAzul;
    Serial.print("Cor do gol configurada via musculo: ");
    Serial.println(corGolAzulCfg ? "AZUL" : "AMARELO");
    SerialMusculo.println("CFG:GOL:OK");
  } else if (msg.length() > 0) {
    Serial.print("Recebido do musculo: ");
    Serial.println(msg);
  }
}

// Envia para a placa Olho o estado do jogo e a configuracao atual.
void enviarEstadoParaOlho(bool forcar = false) {
  if (!forcar && (millis() - ultimoEnvioEstadoOlhoMs) < INTERVALO_ENVIO_ESTADO_OLHO_MS) {
    return;
  }

  PacoteEstado estado;
  estado.sozinho = false;
  estado.atacante = false;
  estado.corGolAzul = corGolAzulCfg;

  SerialOlho.write(BYTE_INICIA);
  SerialOlho.write(ID_PLACA_OLHO);
  SerialOlho.write((uint8_t*)&estado, sizeof(PacoteEstado));
  SerialOlho.write(BYTE_PARA);
  ultimoEnvioEstadoOlhoMs = millis();
}

// Encaminha evento de botao para o Musculo.
void enviarEventoBotao(uint8_t botao) {
  SerialMusculo.print("BTN:");
  SerialMusculo.println(botao);

  Serial.print("Enviado para musculo -> BTN:");
  Serial.println(botao);
}

// Envia resultado do autoteste de comunicacao das placas secundarias.
void enviarStatusPlacasParaMusculo() {
  SerialMusculo.print("STS:");
  SerialMusculo.print(comunicacaoOlhoOK ? 1 : 0);
  SerialMusculo.print(",");
  SerialMusculo.println(comunicacaoPeOK ? 1 : 0);
}

// Publica periodicamente o angulo IR recebido da placa Olho.
void enviarIrParaMusculo() {
  if ((millis() - ultimoEnvioIrMs) < INTERVALO_ENVIO_IR_MS) {
    return;
  }

  bool pacoteRecente = (ultimoRxOlhoMs > 0) && ((millis() - ultimoRxOlhoMs) < TIMEOUT_DADO_OLHO_MS);
  bool anguloValido = (ultimoAnguloIrX10 >= 0);
  bool intensidadeValida = (ultimaIntensidadeIrX10 >= INTENSIDADE_MINIMA_IR_X10);
  bool leituraValida = pacoteRecente && anguloValido && intensidadeValida;

  if (!leituraValida) {
    resetLeituraIrOlho();
  }

  float anguloIr = leituraValida ? (ultimoAnguloIrX10 / 10.0f) : -1.0f;
  SerialMusculo.print("IR:");
  SerialMusculo.println(anguloIr, 1);
  ultimoEnvioIrMs = millis();
}

// Publica periodicamente o heading da bussola para o Musculo.
void enviarBussolaParaMusculo() {
  if (!bussolaOK) {
    return;
  }

  if ((millis() - ultimoEnvioBussolaMusculoMs) < INTERVALO_ENVIO_BUSSOLA_MS) {
    return;
  }

  SerialMusculo.print("BUS:");
  SerialMusculo.println(ultimoHeadingBussola, 1);
  ultimoEnvioBussolaMusculoMs = millis();
}

// Publica erro de gol, deteccao e confianca de camera para o Musculo.
void enviarGolParaMusculo() {
  if ((millis() - ultimoEnvioGolMusculoMs) < INTERVALO_ENVIO_GOL_MS) {
    return;
  }

  SerialMusculo.print("GOL:");
  SerialMusculo.print(ultimoErroGolGraus, 1);
  SerialMusculo.print(",");
  SerialMusculo.print(golDetectadoOlho ? 1 : 0);
  SerialMusculo.print(",");
  SerialMusculo.print(ultimoPixelsGol);
  SerialMusculo.print(",");
  SerialMusculo.println(cameraOlhoOK ? 1 : 0);
  ultimoEnvioGolMusculoMs = millis();
}

// Publica angulo de linha calculado pela placa Pe.
void enviarLinhaParaMusculo() {
  if ((millis() - ultimoEnvioLinhaMusculoMs) < INTERVALO_ENVIO_LINHA_MS) {
    return;
  }

  float angLinha = ultimoAnguloLinhaX10 / 10.0f;
  SerialMusculo.print("LIN:");
  SerialMusculo.println(angLinha, 1);
  ultimoEnvioLinhaMusculoMs = millis();
}

// Publica intensidade IR estimada pela placa Olho.
void enviarIntensidadeParaMusculo() {
  if ((millis() - ultimoEnvioIntMusculoMs) < INTERVALO_ENVIO_INT_MS) {
    return;
  }

  float intensidadeIr = ultimaIntensidadeIrX10 / 10.0f;
  SerialMusculo.print("INT:");
  SerialMusculo.println(intensidadeIr, 1);
  ultimoEnvioIntMusculoMs = millis();
}

// Publica distancias dos 4 ultrassonicos da placa Olho para o Musculo.
void enviarUltrasParaMusculo() {
  if ((millis() - ultimoEnvioUltraMusculoMs) < INTERVALO_ENVIO_ULTRA_MS) {
    return;
  }

  bool pacoteRecente = (ultimoRxOlhoMs > 0) && ((millis() - ultimoRxOlhoMs) < TIMEOUT_DADO_OLHO_MS);
  float uD = pacoteRecente ? (ultimoUltraDX10 / 10.0f) : -1.0f;
  float uE = pacoteRecente ? (ultimoUltraEX10 / 10.0f) : -1.0f;
  float uF = pacoteRecente ? (ultimoUltraFX10 / 10.0f) : -1.0f;
  float uT = pacoteRecente ? (ultimoUltraTX10 / 10.0f) : -1.0f;

  SerialMusculo.print("ULT:");
  SerialMusculo.print(uD, 1);
  SerialMusculo.print(",");
  SerialMusculo.print(uE, 1);
  SerialMusculo.print(",");
  SerialMusculo.print(uF, 1);
  SerialMusculo.print(",");
  SerialMusculo.println(uT, 1);
  ultimoEnvioUltraMusculoMs = millis();
}

// Le chave do kicker e envia estado periodico/por mudanca ao Musculo.
void enviarEstadoKickerParaMusculo() {
  if ((millis() - ultimoEnvioKickerMusculoMs) < INTERVALO_ENVIO_KICKER_MS) {
    return;
  }

  // INPUT_PULLUP: 0 = chave acionada, 1 = chave nao acionada
  int leitura = digitalRead(KICKER_PIN);
  kickerAtivado = (leitura == LOW);
  int valorEnvio = kickerAtivado ? 0 : 1;

  // Envia sempre quando muda e periodicamente para manter sincronismo.
  if (valorEnvio != ultimoKickerEnviado || (millis() - ultimoEnvioKickerMusculoMs) >= 500) {
    SerialMusculo.print("KIK:");
    SerialMusculo.println(valorEnvio);
    ultimoKickerEnviado = valorEnvio;
  }

  ultimoEnvioKickerMusculoMs = millis();
}

// Valida respostas textuais de vida e marca comunicacao ativa.
void processarTextoResposta(String &buffer, bool &flagResposta) {
  buffer.trim();
  buffer.toUpperCase();
  if (buffer == "OK" || buffer == "OI" || buffer == "OI_PE" || buffer == "OI_OLHO") {
    flagResposta = true;
  }
  buffer = "";
}

// Le serial da placa Olho, decodifica pacote binario e fallback textual.
void lerRespostaOlho() {
  while (SerialOlho.available() > 0) {
    if (SerialOlho.peek() == BYTE_INICIA) {
      if (SerialOlho.available() < (int)(sizeof(PacoteOlho) + 3)) {
        return;
      }

      SerialOlho.read();
      byte id = SerialOlho.read();
      if (id == ID_PLACA_OLHO) {
        PacoteOlho pacote;
        SerialOlho.readBytes((uint8_t*)&pacote, sizeof(PacoteOlho));
        byte stop = SerialOlho.read();
        if (stop == BYTE_PARA) {
          comunicacaoOlhoOK = true;
          ultimoRxOlhoMs = millis();
          ultimoUltraDX10 = pacote.uD;
          ultimoUltraEX10 = pacote.uE;
          ultimoUltraFX10 = pacote.uF;
          ultimoUltraTX10 = pacote.uT;
          ultimoAnguloIrX10 = pacote.angulo;
          ultimaIntensidadeIrX10 = pacote.intensidade;
          ultimoErroGolGraus = pacote.erroGol / 10.0f;
          ultimoPixelsGol = pacote.pixelsGol;
          golDetectadoOlho = (pacote.golDetectado != 0);
          cameraOlhoOK = (pacote.cameraOK != 0);
        }
      }
      continue;
    }

    char c = (char)SerialOlho.read();
    if (c == '\n' || c == '\r') {
      if (bufferOlho.length() > 0) {
        processarTextoResposta(bufferOlho, comunicacaoOlhoOK);
      }
      continue;
    }

    if (isPrintable(c) && bufferOlho.length() < 16) {
      bufferOlho += c;
    } else {
      bufferOlho = "";
    }
  }
}

// Le serial da placa Pe, decodifica pacote binario e fallback textual.
void lerRespostaPe() {
  while (SerialPe.available() > 0) {
    if (SerialPe.peek() == BYTE_INICIA) {
      if (SerialPe.available() < (int)(sizeof(PacotePe) + 3)) {
        return;
      }

      SerialPe.read();
      byte id = SerialPe.read();
      if (id == ID_PLACA_PE) {
        PacotePe pacote;
        SerialPe.readBytes((uint8_t*)&pacote, sizeof(PacotePe));
        byte stop = SerialPe.read();
        if (stop == BYTE_PARA) {
          comunicacaoPeOK = true;
          ultimoAnguloLinhaX10 = pacote.angulo;
        }
      }
      continue;
    }

    char c = (char)SerialPe.read();
    if (c == '\n' || c == '\r') {
      if (bufferPe.length() > 0) {
        processarTextoResposta(bufferPe, comunicacaoPeOK);
      }
      continue;
    }

    if (isPrintable(c) && bufferPe.length() < 16) {
      bufferPe += c;
    } else {
      bufferPe = "";
    }
  }
}

// Executa teste rapido de handshake com Olho e Pe no startup.
void testarOlhoPe() {
  comunicacaoOlhoOK = false;
  comunicacaoPeOK = false;

  unsigned long inicioTeste = millis();
  while ((millis() - inicioTeste) < 2000) {
    if (!comunicacaoOlhoOK && (millis() - ultimoPingOlhoMs) >= 250) {
      SerialOlho.println("oi");
      ultimoPingOlhoMs = millis();
    }

    if (!comunicacaoPeOK && (millis() - ultimoPingPeMs) >= 250) {
      SerialPe.println("oi");
      ultimoPingPeMs = millis();
    }

    lerRespostaOlho();
    lerRespostaPe();

    if (comunicacaoOlhoOK && comunicacaoPeOK) {
      break;
    }

    delay(10);
  }

  enviarStatusPlacasParaMusculo();
}

// Detecta borda de pressionamento dos botoes com debounce.
void verificarBotoes() {
  bool botao1Atual = digitalRead(BOTAO_1);
  bool botao2Atual = digitalRead(BOTAO_2);
  bool botao3Atual = digitalRead(BOTAO_3);
  unsigned long agora = millis();

  if (agora - ultimoEventoBotaoMs >= DEBOUNCE_BOTAO_MS) {
    if (botao1Anterior == HIGH && botao1Atual == LOW) {
      ultimoEventoBotaoMs = agora;
      enviarEventoBotao(1);
    } else if (botao2Anterior == HIGH && botao2Atual == LOW) {
      ultimoEventoBotaoMs = agora;
      enviarEventoBotao(2);
    } else if (botao3Anterior == HIGH && botao3Atual == LOW) {
      ultimoEventoBotaoMs = agora;
      enviarEventoBotao(3);
    }
  }

  botao1Anterior = botao1Atual;
  botao2Anterior = botao2Atual;
  botao3Anterior = botao3Atual;
}

// Le serial do Musculo e processa mensagens por terminador de linha.
void lerSerialMusculo() {
  while (SerialMusculo.available() > 0) {
    char c = (char)SerialMusculo.read();
    ultimoByteMs = millis();

    if (c == '\n' || c == '\r') {
      processarMensagem(bufferEntrada);
      bufferEntrada = "";
      continue;
    }

    if (bufferEntrada.length() < 32) {
      bufferEntrada += c;
    }
  }

  if (bufferEntrada.length() > 0 && (millis() - ultimoByteMs) > 80) {
    processarMensagem(bufferEntrada);
    bufferEntrada = "";
  }
}

// Inicializa serials, sensores e handshakes iniciais do sistema.
void setup() {
  Serial.begin(115200);
  SerialMusculo.begin(9600, SERIAL_8N1, RX_MUSCULO, TX_MUSCULO);
  SerialOlho.begin(9600, SERIAL_8N1, RX_OLHO, TX_OLHO);
  SerialPe.begin(BAUD_PE_CABECA, SERIAL_8N1, RX_PE, TX_PE);
  Wire.begin(I2C_SDA, I2C_SCL);

  pinMode(BOTAO_1, INPUT_PULLUP);
  pinMode(BOTAO_2, INPUT_PULLUP);
  pinMode(BOTAO_3, INPUT_PULLUP);
  pinMode(KICKER_PIN, INPUT_PULLUP);

  Serial.println("Cabeca principal em modo minimo de comunicacao");
  bussolaOK = iniciarBussola();
  if (!bussolaOK) {
    Serial.println("Aviso: QMC5883P nao detectada na inicializacao.");
  }

  unsigned long inicioHandshake = millis();
  while ((millis() - inicioHandshake) < 3000 && !comunicacaoMusculoOK) {
    if (millis() - ultimoEnvioOiMs >= 300) {
      SerialMusculo.println("oi");
      ultimoEnvioOiMs = millis();
    }
    lerSerialMusculo();
    delay(10);
  }

  testarOlhoPe();
  enviarEstadoParaOlho(true);
}

// Laco principal da Cabeca: coleta entradas e redistribui dados para o Musculo.
void loop() {
  if (millis() - ultimoEnvioOiMs >= INTERVALO_OI_MS) {
    SerialMusculo.println("oi");
    ultimoEnvioOiMs = millis();
  }

  verificarBotoes();
  lerSerialMusculo();
  lerRespostaOlho();
  lerRespostaPe();
  enviarIrParaMusculo();
  atualizarBussola();
  enviarBussolaParaMusculo();
  enviarGolParaMusculo();
  enviarLinhaParaMusculo();
  enviarIntensidadeParaMusculo();
  enviarUltrasParaMusculo();
  enviarEstadoKickerParaMusculo();
  enviarEstadoParaOlho();

  delay(5);
}
