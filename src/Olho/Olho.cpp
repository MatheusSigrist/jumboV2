#include <Arduino.h>
#include <math.h>
#include <EEPROM.h>
#define BYTE_INICIA 0xAA
#define BYTE_PARA 0x55
#include <HCSR04.h>    // <-- Biblioteca Ultrassonico

bool atacante = false;  // true = atacante, false = defensor
bool jaSaudouCabeca = false;  // Flag para enviar "OK" no startup
bool corGolAzul = false;      // false = amarelo, true = azul

#define ID_PLACA_OLHO 0x01
#define ID_PLACA_PE 0x02
#define ID_PLACA_CAMERA 0x03

#define EEPROM_SIZE 16
#define EEPROM_ADDR_COR_GOL 0



HCSR04 hcT(4, 5);    // <-- Ultrassonico 1 - TRIGG, ECHO
HCSR04 hcF(10, 9);    // <-- Ultrassonico 2 - TRIGG, ECHO
HCSR04 hcD(21, 47);    // <-- Ultrassonico 3 - TRIGG, ECHO
HCSR04 hcE(37, 36);    // <-- Ultrassonico 4 - TRIGG, ECHO

float  ultraT = 0;
float  ultraF = 0;
float  ultraE = 0;
float  ultraD = 0;

void L_Ultra() {     // <-- Função para leitura dos sensores ultrassonicos
  ultraD = hcD.dist();
  ultraE = hcE.dist();
  ultraF = hcF.dist();
  ultraT = hcT.dist();

}


#define RX_CABECA 17
#define TX_CABECA 18
#define RX_CAMERA 8 
#define TX_CAMERA 3




//------------------------------------ IR SEEKER -----------------------------//
const int NUM_SENSORES = 12;
const int sensoresTSOP[NUM_SENSORES] = {
  6, 7, 46, 11, 12, 13, 14, 48,
  45, 35, 38, 39  
};
float angulos[NUM_SENSORES] = {
  0, 30, 60, 90, 120, 150, 180, 210,
  240, 270, 300, 330 
};
const unsigned long JANELA_TEMPO = 10;    // <-- Janela de tempo para contagens de pulso de IR (10ms)
const int LIMIAR_PULSOS = 2;    // <-- Limiar de pulsos para identificar que é a bola
unsigned int pulsos[NUM_SENSORES];
unsigned int intensidade = 0;  // <-- Declarada globalmente para ser usada em enviarDados()



void contarPulsosSensores() {     // <-- Função de contagens de pulsos IR emitidos pela bola (IR SEEKER)
  for (int i = 0; i < NUM_SENSORES; i++) pulsos[i] = 0;
  unsigned long t0 = millis();
  int oldState[NUM_SENSORES];
  for (int i = 0; i < NUM_SENSORES; i++)
    oldState[i] = digitalRead(sensoresTSOP[i]);
  while (millis() - t0 < JANELA_TEMPO) {
    for (int i = 0; i < NUM_SENSORES; i++) {
      int s = digitalRead(sensoresTSOP[i]);
      if (oldState[i] == HIGH && s == LOW) {
        pulsos[i]++;
      }
      oldState[i] = s;
    } 
  }
  // Atualizar intensidade global
  intensidade = 0;
  for (int i = 0; i < NUM_SENSORES; i++) {
    intensidade += pulsos[i];
  }
}

float calculaAnguloBola() {     // <-- Função para cálculo do angulo da bola pelos pulsos de IR lidos (IR SEEKER)
  float x = 0, y = 0, soma_pesos = 0;
  for (int i = 0; i < NUM_SENSORES; i++) {
    if (pulsos[i] >= LIMIAR_PULSOS) {
      float rad = angulos[i] * PI / 180.0;
      x += pulsos[i] * cos(rad);
      y += pulsos[i] * sin(rad);
      soma_pesos += pulsos[i];
    }
  }
  if (soma_pesos == 0) return -1.0;
  float angulo_bola = atan2(y, x) * 180.0 / PI;
  if (angulo_bola < 0) angulo_bola += 360.0;
  return angulo_bola;
}












//---------------jeitim-2------------------//
#include <stdint.h>

struct Pacote {
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

struct PacoteEstado {
  bool sozinho;    // mantido para compatibilidade
  bool atacante;   // true = atacante, false = defensor
  bool corGolAzul; // true = azul, false = amarelo
};

// Dados recebidos da camera:
// [gol_detectado (1B)] [erro_gol_x10 (int16, big-endian)] [pixels (uint16, big-endian)]
bool golDetectadoCamera = false;
float erroGolCamera = 0.0;
uint16_t pixelsGolCamera = 0;
String bufferHandshakeCabeca = "";
unsigned long ultimoByteHandshakeCabeca = 0;

void ProcessarPingCabeca() {
  while (Serial1.available() > 0) {
    if (Serial1.peek() == BYTE_INICIA) {
      return;
    }

    char c = (char)Serial1.read();
    ultimoByteHandshakeCabeca = millis();

    if (c == '\n' || c == '\r') {
      bufferHandshakeCabeca.trim();
      bufferHandshakeCabeca.toLowerCase();
      if (bufferHandshakeCabeca == "oi") {
        Serial1.println("OI");
      }
      bufferHandshakeCabeca = "";
      continue;
    }

    if (isPrintable(c) && bufferHandshakeCabeca.length() < 16) {
      bufferHandshakeCabeca += c;
    } else {
      bufferHandshakeCabeca = "";
    }
  }

  if (bufferHandshakeCabeca.length() > 0 && (millis() - ultimoByteHandshakeCabeca) > 80) {
    bufferHandshakeCabeca.trim();
    bufferHandshakeCabeca.toLowerCase();
    if (bufferHandshakeCabeca == "oi") {
      Serial1.println("OI");
    }
    bufferHandshakeCabeca = "";
  }
}

void EnviarCorParaCamera() {
  Serial2.write(BYTE_INICIA);
  Serial2.write(ID_PLACA_OLHO);
  Serial2.write(corGolAzul ? 1 : 0);
  Serial2.write(BYTE_PARA);
}

void SalvarCorGolEEPROM() {
  EEPROM.writeByte(EEPROM_ADDR_COR_GOL, corGolAzul ? 1 : 0);
  EEPROM.commit();
}

void CarregarCorGolEEPROM() {
  uint8_t val = EEPROM.readByte(EEPROM_ADDR_COR_GOL);
  corGolAzul = (val == 1);
}

void LeituraSerial() {
  while (Serial1.available() >= 2) {
    if (Serial1.read() == BYTE_INICIA) {
      byte id = Serial1.read();
      if (id == ID_PLACA_OLHO || id == ID_PLACA_PE) {
        if (Serial1.available() >= sizeof(PacoteEstado) + 1) {
          PacoteEstado temp;
          Serial1.readBytes((uint8_t*)&temp, sizeof(PacoteEstado));
          byte stop = Serial1.read();
          if (stop == BYTE_PARA) {
            atacante = temp.atacante;  // recebe o papel (atacante/defensor)
            bool novaCorGol = temp.corGolAzul;
            if (novaCorGol != corGolAzul) {
              corGolAzul = novaCorGol;
              SalvarCorGolEEPROM();
              EnviarCorParaCamera();
            }
          }
        }
      }
    }
  }
}

void LeituraCamera() {
  // Protocolo camera: [0xAA][ID=0x03][5 bytes payload][0x55]
  while (Serial2.available() >= 8) {
    if (Serial2.read() != BYTE_INICIA) continue;

    byte id = Serial2.read();
    if (id != ID_PLACA_CAMERA) {
      continue;
    }

    byte payload[5];
    Serial2.readBytes(payload, 5);
    byte stop = Serial2.read();
    if (stop != BYTE_PARA) {
      continue;
    }

    golDetectadoCamera = (payload[0] != 0);

    int16_t erroRaw = (int16_t)((payload[1] << 8) | payload[2]);
    erroGolCamera = erroRaw / 10.0;

    pixelsGolCamera = (uint16_t)((payload[3] << 8) | payload[4]);
  }
}


void enviarDados() { 
  Pacote p;

  p.uD = (int16_t)round(ultraD * 10.0);
  p.uE = (int16_t)round(ultraE * 10.0);
  p.uF = (int16_t)round(ultraF * 10.0);
  p.uT = (int16_t)round(ultraT * 10.0);
  p.angulo = (int16_t)round(calculaAnguloBola() * 10.0);
  p.intensidade = (int16_t)round(intensidade * 10.0);

  int16_t erroGolRaw = (int16_t)round(erroGolCamera * 10.0);
  p.erroGol = erroGolRaw;
  p.pixelsGol = pixelsGolCamera;
  p.golDetectado = golDetectadoCamera ? 1 : 0;

  Serial1.write(BYTE_INICIA);
  Serial1.write(ID_PLACA_OLHO);
  Serial1.write((uint8_t*)&p, sizeof(Pacote));
  Serial1.write(BYTE_PARA);
}

//--------------------------------------//





void setup() {
    for (int i = 0; i < NUM_SENSORES; i++)
      pinMode(sensoresTSOP[i], INPUT);

    EEPROM.begin(EEPROM_SIZE);
    CarregarCorGolEEPROM();

    Serial1.begin(9600, SERIAL_8N1, RX_CABECA, TX_CABECA);
    Serial2.begin(19200, SERIAL_8N1, RX_CAMERA, TX_CAMERA);

    // Ao ligar, envia uma vez a cor salva para camera
    delay(50);
    EnviarCorParaCamera();
}

void loop(){
  ProcessarPingCabeca();

  contarPulsosSensores(); // Leitura IR SEEKER
  L_Ultra(); // Leitura dos ultras

  // ler estado recebido da cabeça (atacante/defensor)
  LeituraSerial();

  // ler dados recebidos da camera (gol detectado/erro/pixels)
  LeituraCamera();

  enviarDados(); 
  delay(50);
}