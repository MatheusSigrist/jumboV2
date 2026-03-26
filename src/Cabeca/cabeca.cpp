#include <Arduino.h>

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

#define BOTAO_1 3
#define BOTAO_2 37
#define BOTAO_3 46

const unsigned long DEBOUNCE_BOTAO_MS = 180;
const unsigned long INTERVALO_OI_MS = 1000;

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
  int16_t erroGol;
  uint16_t pixelsGol;
  uint8_t golDetectado;
};

struct PacotePe {
  int16_t angulo;
};

String bufferEntrada = "";
unsigned long ultimoByteMs = 0;
unsigned long ultimoEventoBotaoMs = 0;
unsigned long ultimoEnvioOiMs = 0;
unsigned long ultimoEnvioIrMs = 0;

bool botao1Anterior = HIGH;
bool botao2Anterior = HIGH;
bool botao3Anterior = HIGH;
bool comunicacaoMusculoOK = false;
bool comunicacaoOlhoOK = false;
bool comunicacaoPeOK = false;

String bufferOlho = "";
String bufferPe = "";
unsigned long ultimoPingOlhoMs = 0;
unsigned long ultimoPingPeMs = 0;
int16_t ultimoAnguloIrX10 = -10;

const unsigned long INTERVALO_ENVIO_IR_MS = 120;

void processarMensagem(String msg) {
  msg.trim();
  msg.toLowerCase();

  if (msg == "oi") {
    comunicacaoMusculoOK = true;
    SerialMusculo.println("oi");
    Serial.println("Musculo respondeu handshake");
  } else if (msg.length() > 0) {
    Serial.print("Recebido do musculo: ");
    Serial.println(msg);
  }
}

void enviarEventoBotao(uint8_t botao) {
  SerialMusculo.print("BTN:");
  SerialMusculo.println(botao);

  Serial.print("Enviado para musculo -> BTN:");
  Serial.println(botao);
}

void enviarStatusPlacasParaMusculo() {
  SerialMusculo.print("STS:");
  SerialMusculo.print(comunicacaoOlhoOK ? 1 : 0);
  SerialMusculo.print(",");
  SerialMusculo.println(comunicacaoPeOK ? 1 : 0);
}

void enviarIrParaMusculo() {
  if ((millis() - ultimoEnvioIrMs) < INTERVALO_ENVIO_IR_MS) {
    return;
  }

  float anguloIr = ultimoAnguloIrX10 / 10.0;
  SerialMusculo.print("IR:");
  SerialMusculo.println(anguloIr, 1);
  ultimoEnvioIrMs = millis();
}

void processarTextoResposta(String &buffer, bool &flagResposta) {
  buffer.trim();
  buffer.toUpperCase();
  if (buffer == "OK" || buffer == "OI" || buffer == "OI_PE" || buffer == "OI_OLHO") {
    flagResposta = true;
  }
  buffer = "";
}

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
          ultimoAnguloIrX10 = pacote.angulo;
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

void setup() {
  Serial.begin(115200);
  SerialMusculo.begin(9600, SERIAL_8N1, RX_MUSCULO, TX_MUSCULO);
  SerialOlho.begin(9600, SERIAL_8N1, RX_OLHO, TX_OLHO);
  SerialPe.begin(9600, SERIAL_8N1, RX_PE, TX_PE);

  pinMode(BOTAO_1, INPUT_PULLUP);
  pinMode(BOTAO_2, INPUT_PULLUP);
  pinMode(BOTAO_3, INPUT_PULLUP);

  Serial.println("Cabeca principal em modo minimo de comunicacao");

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
}

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

  delay(5);
}
