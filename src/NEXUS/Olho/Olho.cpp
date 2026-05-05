// Arquivo principal da placa Olho.
// Funcao: ler sensores IR (direcao da bola), ultrassonicos e camera,
// montar pacote com deteccoes e enviar para a Cabeca.
// Entrada: TSOPs, ultrassonicos, camera UART e estado recebido da Cabeca.
// Saida: pacote serial com angulo/intensidade IR, dados do gol e cameraOK.
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



HCSR04 hcF(4, 5);    // <-- Ultrassonico 1 - TRIGG, ECHO
HCSR04 hcD(10, 9);    // <-- Ultrassonico 2 - TRIGG, ECHO
HCSR04 hcT(21, 47);    // <-- Ultrassonico 3 - TRIGG, ECHO
HCSR04 hcE(37, 36);    // <-- Ultrassonico 4 - TRIGG, ECHO

float  ultraT = 0;
float  ultraF = 0;
float  ultraE = 0;
float  ultraD = 0;

// Le os quatro ultrassonicos e atualiza distancias globais em cm.
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

const unsigned long JANELA_TEMPO = 15;    // <-- Janela de tempo para contagens de pulso de IR (10ms)
const int LIMIAR_PULSOS = 8;    // <-- Limiar de pulsos para identificar que é a bola
const int NUM_AMOSTRAS_VOTO = 5;          // <-- Qtd de amostras para votacao
const float TOLERANCIA_VOTO_GRAUS = 45.0f; // <-- Tolerancia para considerar amostras do mesmo grupo
unsigned int pulsos[NUM_SENSORES];
unsigned int nivelBaixo[NUM_SENSORES];
float pesosIr[NUM_SENSORES];
unsigned long amostrasJanela = 1;
float intensidade = 0;  // <-- Declarada globalmente para ser usada em enviarDados()



// Conta pulsos IR por sensor em uma janela curta e calcula pesos por deteccao.
void contarPulsosSensores() {     // <-- Função de contagens de pulsos IR emitidos pela bola (IR SEEKER)
  for (int i = 0; i < NUM_SENSORES; i++) {
    pulsos[i] = 0;
    nivelBaixo[i] = 0;
    pesosIr[i] = 0;
  }
  unsigned long t0 = millis();
  amostrasJanela = 0;
  int oldState[NUM_SENSORES];
  for (int i = 0; i < NUM_SENSORES; i++)
    oldState[i] = digitalRead(sensoresTSOP[i]);

  while (millis() - t0 < JANELA_TEMPO) {
    amostrasJanela++;
    for (int i = 0; i < NUM_SENSORES; i++) {
      int s = digitalRead(sensoresTSOP[i]);
      if (s == LOW) {
        nivelBaixo[i]++;
      }
      if (oldState[i] == HIGH && s == LOW) {
        pulsos[i]++;
      }
      oldState[i] = s;
    } 
  }

  if (amostrasJanela == 0) {
    amostrasJanela = 1;
  }

  // Atualizar intensidade global somente por contagem de pulsos validos.
  intensidade = 0;
  for (int i = 0; i < NUM_SENSORES; i++) {
    bool detectou = (pulsos[i] >= LIMIAR_PULSOS);

    if (detectou) {
      pesosIr[i] = (float)pulsos[i];
      intensidade += pesosIr[i];
    }
  }
}

// Calcula o angulo da bola por media vetorial dos sensores IR ativos.
float calculaAnguloBola() {     // <-- Função para cálculo do angulo da bola pelos pulsos de IR lidos (IR SEEKER)
  float x = 0, y = 0, soma_pesos = 0;
  for (int i = 0; i < NUM_SENSORES; i++) {
    if (pesosIr[i] > 0.0f) {
      float rad = angulos[i] * PI / 180.0;
      x += pesosIr[i] * cos(rad);
      y += pesosIr[i] * sin(rad);
      soma_pesos += pesosIr[i];
    }
  }
  if (soma_pesos == 0) return -1.0;
  float angulo_bola = atan2(y, x) * 180.0 / PI;
  if (angulo_bola < 0) angulo_bola += 360.0;
  return angulo_bola;
}

// Retorna a menor diferenca angular absoluta entre dois angulos.
float diferencaAngularAbsoluta(float a, float b) {
  float d = fabs(a - b);
  if (d > 180.0f) d = 360.0f - d;
  return d;
}

// Filtra o angulo da bola por votacao entre amostras para reduzir ruido.
float filtrarAnguloBola() {
  // Coleta NUM_AMOSTRAS_VOTO amostras
  float amostras[NUM_AMOSTRAS_VOTO];
  int validas = 0;
  for (int i = 0; i < NUM_AMOSTRAS_VOTO; i++) {
    contarPulsosSensores();
    float a = calculaAnguloBola();
    if (a >= 0.0f) {
      amostras[validas++] = a;
    }
  }

  if (validas == 0) return -1.0f;
  if (validas == 1) return amostras[0];

  // Para cada amostra, conta quantas outras estão dentro da TOLERANCIA_VOTO_GRAUS
  int melhorVotos = 0;
  int melhorIdx = 0;
  for (int i = 0; i < validas; i++) {
    int votos = 0;
    for (int j = 0; j < validas; j++) {
      if (diferencaAngularAbsoluta(amostras[i], amostras[j]) <= TOLERANCIA_VOTO_GRAUS) {
        votos++;
      }
    }
    if (votos > melhorVotos) {
      melhorVotos = votos;
      melhorIdx = i;
    }
  }

  // Retorna a média vetorial das amostras vencedoras do grupo
  float sx = 0, sy = 0;
  for (int i = 0; i < validas; i++) {
    if (diferencaAngularAbsoluta(amostras[i], amostras[melhorIdx]) <= TOLERANCIA_VOTO_GRAUS) {
      float rad = amostras[i] * PI / 180.0f;
      sx += cosf(rad);
      sy += sinf(rad);
    }
  }
  float resultado = atan2f(sy, sx) * 180.0f / PI;
  if (resultado < 0) resultado += 360.0f;
  return resultado;
}

// Imprime debug IR no mesmo formato do teste de bancada.
void imprimirDebugSensoresIR(float angulo) {
  Serial.println("===== TESTE PLACA OLHO (IR) =====");
  for (int i = 0; i < NUM_SENSORES; i++) {
    Serial.print("IR[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.println(pulsos[i]);
  }

  if (angulo < 0.0f) {
    Serial.println("Angulo: sem deteccao");
  } else {
    Serial.print("Angulo: ");
    Serial.print(angulo, 1);
    Serial.println(" graus");
  }

  Serial.println();
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

struct PacoteEstado {
  bool sozinho;    // mantido para compatibilidade
  bool atacante;   // true = atacante, false = defensor
  bool corGolAzul; // true = azul, false = amarelo
};

// Dados recebidos da camera:
// [BALL_A(int16)][BALL_D(uint16)][BLUE_A(int16)][BLUE_D(uint16)][YELLOW_A(int16)][YELLOW_D(uint16)]
int16_t ballCameraAngle = 0;
uint16_t ballCameraDist = 0;
int16_t blueCameraAngle = -999;
uint16_t blueCameraDist = 0;
int16_t yellowCameraAngle = -999;
uint16_t yellowCameraDist = 0;
unsigned long ultimoRxCameraMs = 0;  // timestamp do ultimo pacote valido recebido da camera
String bufferHandshakeCabeca = "";
unsigned long ultimoByteHandshakeCabeca = 0;

// Responde handshake textual da Cabeca sem atrapalhar o protocolo binario.
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

// [Removido] Envia para a camera - agora camera envia 6 valores diretos para olho

// Salva a cor de gol na EEPROM para manter configuracao apos reboot.
void SalvarCorGolEEPROM() {
  EEPROM.writeByte(EEPROM_ADDR_COR_GOL, corGolAzul ? 1 : 0);
  EEPROM.commit();
}

// Carrega da EEPROM a cor de gol usada ao iniciar a placa.
void CarregarCorGolEEPROM() {
  uint8_t val = EEPROM.readByte(EEPROM_ADDR_COR_GOL);
  corGolAzul = (val == 1);
}

// Le estado enviado pela Cabeca e aplica mudancas de atacante/cor de gol.
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
            }
          }
        }
      }
    }
  }
}

// Le pacote da camera com dados de BOLA + 2 GOLS (ângulo e distância cada)
void LeituraCamera() {
  // Protocolo camera: [0xAA][ID=0x03][12 bytes payload][0x55]
  // Payload: [BALL_A(2B)][BALL_D(2B)][BLUE_A(2B)][BLUE_D(2B)][YELLOW_A(2B)][YELLOW_D(2B)]
  while (Serial2.available() >= 15) {  // 1 + 1 + 12 + 1 = 15 bytes totais
    if (Serial2.read() != BYTE_INICIA) continue;

    byte id = Serial2.read();
    if (id != ID_PLACA_CAMERA) {
      continue;
    }

    byte payload[12];
    Serial2.readBytes(payload, 12);
    byte stop = Serial2.read();
    if (stop != BYTE_PARA) {
      continue;
    }

    // Desempacota dados em big-endian (como enviado pela camera)
    ballCameraAngle = (int16_t)((payload[0] << 8) | payload[1]);
    ballCameraDist = (uint16_t)((payload[2] << 8) | payload[3]);
    
    blueCameraAngle = (int16_t)((payload[4] << 8) | payload[5]);
    blueCameraDist = (uint16_t)((payload[6] << 8) | payload[7]);
    
    yellowCameraAngle = (int16_t)((payload[8] << 8) | payload[9]);
    yellowCameraDist = (uint16_t)((payload[10] << 8) | payload[11]);
    
    ultimoRxCameraMs = millis();  // marca que camera esta viva
  }
}


// Monta e envia para a Cabeca um pacote com ultras, IR e dados de camera (bola + 2 gols).
void enviarDados() { 
  Pacote p;

  p.uD = (int16_t)round(ultraD * 10.0);
  p.uE = (int16_t)round(ultraE * 10.0);
  p.uF = (int16_t)round(ultraF * 10.0);
  p.uT = (int16_t)round(ultraT * 10.0);
  p.angulo = (int16_t)round(filtrarAnguloBola() * 10.0);
  p.intensidade = (int16_t)round(intensidade * 10.0);
  
  // ===== Dados de camera =====
  p.ballAngle = ballCameraAngle;
  p.ballDist = ballCameraDist;
  p.blueAngle = blueCameraAngle;
  p.blueDist = blueCameraDist;
  p.yellowAngle = yellowCameraAngle;
  p.yellowDist = yellowCameraDist;
  // ===== FIM dados camera =====
  
  p.cameraOK = ((ultimoRxCameraMs > 0) && ((millis() - ultimoRxCameraMs) < 3000)) ? 1 : 0;

  Serial1.write(BYTE_INICIA);
  Serial1.write(ID_PLACA_OLHO);
  Serial1.write((uint8_t*)&p, sizeof(Pacote));
  Serial1.write(BYTE_PARA);
}

//--------------------------------------//





// Inicializa pinos/seriais (sem sincronizar cor com camera)
void setup() {
    for (int i = 0; i < NUM_SENSORES; i++)
      pinMode(sensoresTSOP[i], INPUT);

    Serial.begin(115200);
    unsigned long tSerial = millis();
    while (!Serial && (millis() - tSerial) < 2000) {
      delay(10);
    }
    Serial.println("OLHO boot");

    EEPROM.begin(EEPROM_SIZE);
    CarregarCorGolEEPROM();

    Serial1.begin(115200, SERIAL_8N1, RX_CABECA, TX_CABECA);
    Serial2.begin(115200, SERIAL_8N1, RX_CAMERA, TX_CAMERA);
}

// Laco principal: handshake, leituras de sensores e envio de pacote.
void loop(){
  ProcessarPingCabeca();


  contarPulsosSensores(); // Leitura IR SEEKER
  float angulo = calculaAnguloBola();
  imprimirDebugSensoresIR(angulo);
  L_Ultra(); // Leitura dos ultras

  // ler estado recebido da cabeça (atacante/defensor)
  LeituraSerial();

  // ler dados recebidos da camera (gol detectado/erro/pixels)
  LeituraCamera();

  enviarDados(); 
  delay(50);
}