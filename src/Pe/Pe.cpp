#include <Arduino.h>
#include <math.h>



#define BYTE_INICIA 0xAA
#define BYTE_PARA   0x55

bool atacante = false;  // true = atacante (repulsao), false = defensor (atraçao)
bool jaSaudouCabeca = false;  // Flag para enviar "OK" no startup

#define ID_PLACA_OLHO 0x01
#define ID_PLACA_PE 0x02
#define RX_CABECA 10
#define TX_CABECA 17




#define NUM_SENSORES 32
#define PI 3.14

// Limiar mínimo do sensor para considerar que há linha
// Ajuste conforme seu piso/iluminação (0..4095 no ESP32)
#define LIMIAR_LINHA 400 

// Intervalo do debug serial para não poluir/travar monitor
#define INTERVALO_DEBUG_MS 120

uint8_t mapaSensores[NUM_SENSORES] = {
  0,  1,  2,  3,
  
  4,  5,  6,  7,
  8,  9, 10, 11,
  12, 13, 14, 15,
  16, 17, 18, 19,
  20, 21, 22, 23,
  24, 25, 26, 27,
  28, 29, 30, 31
};

// ===== PINOS MUX 1 =====
const int MUX1_SIG = 2;
const int MUX1_S0  = 21;
const int MUX1_S1  = 47;
const int MUX1_S2  = 48;
const int MUX1_S3  = 45;

// ===== PINOS MUX 2 =====
const int MUX2_SIG = 12;
const int MUX2_S0  = 4;
const int MUX2_S1  = 5;
const int MUX2_S2  = 6;
const int MUX2_S3  = 7;

int ldr[NUM_SENSORES];

// vetores unitários dos sensores
float sensorX[NUM_SENSORES];
float sensorY[NUM_SENSORES];
unsigned long ultimoDebugMs = 0;

void selecionarCanalMUX(int s0, int s1, int s2, int s3, int canal) {
  digitalWrite(s0, bitRead(canal, 0));
  digitalWrite(s1, bitRead(canal, 1));
  digitalWrite(s2, bitRead(canal, 2));
  digitalWrite(s3, bitRead(canal, 3));
}


#include <stdint.h>

struct Pacote {
  int16_t angulo;
};

struct PacoteEstado {
  bool sozinho;    // mantido para compatibilidade
  bool atacante;   // true = atacante (repulsao), false = defensor (atraçao)
  bool corGolAzul; // compatibilidade com pacote enviado pela Cabeça
};

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
          }
        }
      }
    }
  }
}

int16_t calcularAngulo(bool repulsao) {
  // ===== CENTROIDE =====
  float Cx = 0.0, Cy = 0.0, soma = 0.0;
  
  for (int i = 0; i < NUM_SENSORES; i++) {
    int idxFisico = mapaSensores[i];
    float peso = ldr[idxFisico];
    if (peso >= LIMIAR_LINHA) {
      Cx += peso * sensorX[i];
      Cy += peso * sensorY[i];
      soma += peso;
    }
  }

  if (soma == 0) {
    Serial.println("Nenhuma linha detectada");
    return -1;  // erro: nenhuma linha detectada
  }

  Cx /= soma;
  Cy /= soma;

  // ===== REPULSÃO OU ATRAÇÃO =====
  float Vx = repulsao ? -Cx : Cx;
  float Vy = repulsao ? -Cy : Cy;

  float anguloRad = atan2(Vy, Vx);
  float anguloGraus = anguloRad * 180.0 / PI;
  if (anguloGraus < 0) anguloGraus += 360.0;

  return (int16_t)round(anguloGraus * 10.0);
}

void imprimirLeituraSensores() {
  Serial.print("Sensores: ");
  for (int i = 0; i < NUM_SENSORES; i++) {
    Serial.print("S");
    Serial.print(i);
    Serial.print("=");
    Serial.print(ldr[i]);
    if (ldr[i] >= LIMIAR_LINHA) {
      Serial.print("*");
    }
    if (i < NUM_SENSORES - 1) {
      Serial.print(" | ");
    }
  }
  Serial.println();
}




void setup() {

  Serial.begin(9600);
  Serial1.begin(9600, SERIAL_8N1, RX_CABECA, TX_CABECA);
  pinMode(MUX1_S0, OUTPUT);
  pinMode(MUX1_S1, OUTPUT);
  pinMode(MUX1_S2, OUTPUT);
  pinMode(MUX1_S3, OUTPUT);

  pinMode(MUX2_S0, OUTPUT);
  pinMode(MUX2_S1, OUTPUT);
  pinMode(MUX2_S2, OUTPUT);
  pinMode(MUX2_S3, OUTPUT);

  pinMode(MUX1_SIG, INPUT);
  pinMode(MUX2_SIG, INPUT);
  Serial.println("Placa PE inicializada");
  // ===== PRÉ-CÁLCULO DOS ÂNGULOS =====
  for (int i = 0; i < NUM_SENSORES; i++) {
    float angulo = (2.0 * PI / NUM_SENSORES) * i;
    sensorX[i] = cos(angulo);
    sensorY[i] = sin(angulo);
  }
}

void loop() {

  // Na primeira iteração, enviar "OK" para a cabeça (para comunicação teste)
  if (!jaSaudouCabeca) {
    Serial1.print("OK");
    jaSaudouCabeca = true;
    delay(100);
  }

  // ===== LEITURA FÍSICA =====
  for (int canal = 0; canal < 16; canal++) {
    selecionarCanalMUX(MUX1_S0, MUX1_S1, MUX1_S2, MUX1_S3, canal);
    selecionarCanalMUX(MUX2_S0, MUX2_S1, MUX2_S2, MUX2_S3, canal);

    delayMicroseconds(10);

    ldr[canal]      = analogRead(MUX1_SIG);
    ldr[canal + 16] = analogRead(MUX2_SIG);
  }

  // atualiza estado recebido da cabeça
  LeituraSerial();

  // preparar pacote de ângulo
  Pacote p;
  int16_t angulo = calcularAngulo(atacante);  // true=repulsão (atacante), false=atração (defensor)
  p.angulo = angulo;  // envia -1 se nenhuma linha detectada, ou o ângulo

  // enviar pacote
  Serial1.write(BYTE_INICIA);
  Serial1.write(ID_PLACA_PE);
  Serial1.write((uint8_t*)&p, sizeof(Pacote));
  Serial1.write(BYTE_PARA);

  // debug serial com taxa controlada
  unsigned long agora = millis();
  if (agora - ultimoDebugMs >= INTERVALO_DEBUG_MS) {
    ultimoDebugMs = agora;
    imprimirLeituraSensores();
    if (p.angulo == -1) {
      Serial.println("Linha: NAO detectada");
    } else {
      Serial.print("Linha angulo (deg): ");
      Serial.println(p.angulo / 10.0, 1);
    }
  }


delay(50);
}

