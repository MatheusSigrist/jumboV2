// Arquivo principal da placa Cabeca.
// Funcao: concentrador de comunicacao entre Musculo, Olho e Pe,
// leitura de botoes e bussola, e repasse de dados para o Musculo.
// Entrada: serial das placas, botoes fisicos, chave do kicker, I2C bussola.
// Saida: mensagens de estado/dados para Musculo e comandos para Olho.
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

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
#define I2C_FREQ 100000

// Inicializacao e configuracao da bussola.
const uint8_t QMC5883P_ADDR = 0x2C;

// Valores de calibracao validados no teste dedicado.
const float xOffset = 682.5000;
const float yOffset = 143.5000;
const float xScale  = 1.005451;
const float yScale  = 0.994608;

int head = 0;

bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(reg);
  Wire.write(value);
  uint8_t err = Wire.endTransmission(true);

  if (err != 0) {
    Serial.print("Erro I2C writeReg reg 0x");
    Serial.print(reg, HEX);
    Serial.print(" -> codigo ");
    Serial.println(err);
    return false;
  }
  return true;
}

bool readReg(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(reg);

  uint8_t err = Wire.endTransmission(false);
  if (err != 0) {
    Serial.print("Erro I2C readReg(endTransmission) reg 0x");
    Serial.print(reg, HEX);
    Serial.print(" -> codigo ");
    Serial.println(err);
    return false;
  }

  size_t n = Wire.requestFrom((int)QMC5883P_ADDR, 1, true);
  if (n != 1) {
    Serial.print("Erro I2C readReg(requestFrom) reg 0x");
    Serial.print(reg, HEX);
    Serial.print(" -> recebidos ");
    Serial.println((int)n);
    return false;
  }

  value = Wire.read();
  return true;
}

void scanI2C() {
  Serial.println("Escaneando barramento I2C...");
  uint8_t found = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
      Serial.print("Dispositivo encontrado em 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      found++;
    }
  }

  if (found == 0) {
    Serial.println("Nenhum dispositivo I2C encontrado.");
  }
}

bool initQMC5883P() {
  delay(20);

  bool ok = true;
  ok &= writeReg(0x29, 0x06);
  ok &= writeReg(0x0B, 0x08);
  ok &= writeReg(0x0A, 0xC3);

  delay(20);
  return ok;
}

bool readQMC5883PData(int16_t &x, int16_t &y, int16_t &z) {
  uint8_t status = 0;

  if (!readReg(0x09, status)) {
    return false;
  }

  if ((status & 0x01) == 0) {
    return false;
  }

  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(0x01);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) {
    Serial.print("Erro I2C leitura bloco -> codigo ");
    Serial.println(err);
    return false;
  }

  size_t n = Wire.requestFrom((int)QMC5883P_ADDR, 6, true);
  if (n != 6) {
    Serial.print("Leitura incompleta: ");
    Serial.println((int)n);
    return false;
  }

  uint8_t x_lsb = Wire.read();
  uint8_t x_msb = Wire.read();
  uint8_t y_lsb = Wire.read();
  uint8_t y_msb = Wire.read();
  uint8_t z_lsb = Wire.read();
  uint8_t z_msb = Wire.read();

  x = (int16_t)((x_msb << 8) | x_lsb);
  y = (int16_t)((y_msb << 8) | y_lsb);
  z = (int16_t)((z_msb << 8) | z_lsb);

  return true;
}

int calcularHead(int16_t xRaw, int16_t yRaw) {
  float xCorr = (xRaw - xOffset) * xScale;
  float yCorr = (yRaw - yOffset) * yScale;

  float ang = atan2(-yCorr, xCorr) * 180.0 / PI;

  if (ang < 0) ang += 360.0;

  return (int)ang;
}

// Fim das configuracoes da bussola.

const unsigned long DEBOUNCE_BOTAO_MS = 180;
const unsigned long INTERVALO_OI_MS = 1000;
const unsigned long INTERVALO_BUSSOLA_MS = 50;

HardwareSerial SerialMusculo(0);
HardwareSerial SerialOlho(1);
HardwareSerial SerialPe(2);

struct PacoteOlho {
  int16_t uD;
  int16_t uE;
  int16_t uF;
  int16_t uT;
  int16_t angulo;
  int16_t intensidade;
  // ===== NOVOS: dados de camera (bola + 2 gols) =====
  int16_t ballAngle;
  uint16_t ballDist;
  int16_t blueAngle;
  uint16_t blueDist;
  int16_t yellowAngle;
  uint16_t yellowDist;
  // ===== FIM novos dados camera =====
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

// ===== NOVOS: dados de camera (bola + 2 gols) =====
int16_t ultimoBallAngle = -999;
uint16_t ultimoBallDist = 0;
int16_t ultimoBlueAngle = -999;
uint16_t ultimoBlueDist = 0;
int16_t ultimoYellowAngle = -999;
uint16_t ultimoYellowDist = 0;
// ===== FIM novos dados camera =====

bool golDetectadoOlho = false;
bool cameraOlhoOK = false;  // camera esta se comunicando com o olho
bool kickerAtivado = false;
int ultimoKickerEnviado = -1;
float ultimoHeadingBussola = 0.0f;
bool bussolaOK = false;

const unsigned long INTERVALO_ENVIO_IR_MS = 120;
const unsigned long INTERVALO_ENVIO_BUSSOLA_MS = 120;
const unsigned long INTERVALO_ENVIO_GOL_MS = 120;
const unsigned long INTERVALO_ENVIO_LINHA_MS = 20;
const unsigned long INTERVALO_ENVIO_INT_MS = 120;
const unsigned long INTERVALO_ENVIO_ULTRA_MS = 120;
const unsigned long INTERVALO_ENVIO_KICKER_MS = 120;
const unsigned long INTERVALO_ENVIO_ESTADO_OLHO_MS = 700;
const int16_t INTENSIDADE_MINIMA_IR_X10 = 80;  // 8.0
const unsigned long TIMEOUT_DADO_OLHO_MS = 500;

void resetLeituraIrOlho() {
  ultimoAnguloIrX10 = -10;
  ultimaIntensidadeIrX10 = 0;
}

// Inicializa e valida a bussola QMC5883P usando o mesmo fluxo do teste.
bool iniciarBussola() {
  if (!initQMC5883P()) {
    Serial.println("Falha ao inicializar QMC5883P.");
    return false;
  }

  uint8_t chipID = 0;
  if (readReg(0x00, chipID)) {
    Serial.print("CHIP ID QMC5883P: 0x");
    Serial.println(chipID, HEX);
  } else {
    Serial.println("Falha ao ler CHIP ID da QMC5883P.");
  }

  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;
  for (uint8_t i = 0; i < 10; i++) {
    if (readQMC5883PData(x, y, z)) {
      return true;
    }
    delay(5);
  }
  return false;
}

// Le a bussola no formato do teste e atualiza o heading.
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
  if (!readQMC5883PData(rawX, rawY, rawZ)) {
    return;
  }

  head = calcularHead(rawX, rawY);
  ultimoHeadingBussola = (float)head;

  Serial.print("BUS X:");
  Serial.print(rawX);
  Serial.print(" Y:");
  Serial.print(rawY);
  Serial.print(" Z:");
  Serial.print(rawZ);
  Serial.print(" H:");
  Serial.println(head);

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

// [REMOVIDO] Envia para a placa Olho - comunicação now é unidirecional (Olho->Cabeca)
// Camera envia seus dados direto para Olho, sem necessidade de feedback

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

// Publica bola e gols (ângulo e distância) detectados pela camera para o Musculo.
void enviarCameraParaMusculo() {
  if ((millis() - ultimoEnvioGolMusculoMs) < INTERVALO_ENVIO_GOL_MS) {
    return;
  }

  SerialMusculo.print("CAM:");
  SerialMusculo.print(ultimoBallAngle);
  SerialMusculo.print(",");
  SerialMusculo.print(ultimoBallDist);
  SerialMusculo.print(",");
  SerialMusculo.print(ultimoBlueAngle);
  SerialMusculo.print(",");
  SerialMusculo.print(ultimoBlueDist);
  SerialMusculo.print(",");
  SerialMusculo.print(ultimoYellowAngle);
  SerialMusculo.print(",");
  SerialMusculo.print(ultimoYellowDist);
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
          
          // ===== NOVOS: dados de camera =====
          ultimoBallAngle = pacote.ballAngle;
          ultimoBallDist = pacote.ballDist;
          ultimoBlueAngle = pacote.blueAngle;
          ultimoBlueDist = pacote.blueDist;
          ultimoYellowAngle = pacote.yellowAngle;
          ultimoYellowDist = pacote.yellowDist;
          // ===== FIM novos dados camera =====
          
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
  Wire.setClock(I2C_FREQ);
  Wire.setTimeOut(20);

  pinMode(BOTAO_1, INPUT_PULLUP);
  pinMode(BOTAO_2, INPUT_PULLUP);
  pinMode(BOTAO_3, INPUT_PULLUP);
  pinMode(KICKER_PIN, INPUT_PULLUP);

  Serial.println("Cabeca principal em modo minimo de comunicacao");
  scanI2C();
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
  // [REMOVIDO] enviarEstadoParaOlho(true) - Olho não recebe mais config de cor
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
  enviarCameraParaMusculo();  // Envia dados de camera (bola + 2 gols)
  enviarLinhaParaMusculo();
  enviarIntensidadeParaMusculo();
  enviarUltrasParaMusculo();
  enviarEstadoKickerParaMusculo();

  delay(5);
}
