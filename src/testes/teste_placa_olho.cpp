// Teste da placa Olho focado em IR seeker.
// Funcao: contar pulsos dos 12 sensores TSOP e calcular angulo da bola.
// Saida: logs de pulsos por sensor e angulo estimado no monitor serial.
#include <Arduino.h>
#include <math.h>

const int NUM_SENSORES = 12;
const int sensoresTSOP[NUM_SENSORES] = {
  6, 7, 46, 11, 12, 13, 14, 48,
  45, 35, 38, 39  
};
float angulos[NUM_SENSORES] = {
  0, 30, 60, 90, 120, 150, 180, 210,
  240, 270, 300, 330 
};

const unsigned long JANELA_TEMPO_MS = 15;
const int LIMIAR_PULSOS = 8;

unsigned int pulsos[NUM_SENSORES];

void contarPulsosSensores() {
  for (int i = 0; i < NUM_SENSORES; i++) {
    pulsos[i] = 0;
  }

  int estadoAnterior[NUM_SENSORES];
  for (int i = 0; i < NUM_SENSORES; i++) {
    estadoAnterior[i] = digitalRead(sensoresTSOP[i]);
  }

  unsigned long inicio = millis();
  while (millis() - inicio < JANELA_TEMPO_MS) {
    for (int i = 0; i < NUM_SENSORES; i++) {
      int estadoAtual = digitalRead(sensoresTSOP[i]);
      if (estadoAnterior[i] == HIGH && estadoAtual == LOW) {
        pulsos[i]++;
      }
      estadoAnterior[i] = estadoAtual;
    }
  }
}

float calcularAnguloBola() {
  float x = 0.0;
  float y = 0.0;
  float somaPesos = 0.0;

  for (int i = 0; i < NUM_SENSORES; i++) {
    if (pulsos[i] >= LIMIAR_PULSOS) {
      float rad = angulos[i] * PI / 180.0;
      x += pulsos[i] * cos(rad);
      y += pulsos[i] * sin(rad);
      somaPesos += pulsos[i];
    }
  }

  if (somaPesos == 0.0) {
    return -1.0;
  }

  float angulo = atan2(y, x) * 180.0 / PI;
  if (angulo < 0) {
    angulo += 360.0;
  }

  return angulo;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando teste da placa Olho...");
  for (int i = 0; i < NUM_SENSORES; i++) {
    pinMode(sensoresTSOP[i], INPUT);
  }

  delay(300);
  Serial.println("Iniciando teste da placa Olho...");
}

void loop() {
  contarPulsosSensores();
  float angulo = calcularAnguloBola();

  Serial.println("===== TESTE PLACA OLHO (IR) =====");
  for (int i = 0; i < NUM_SENSORES; i++) {
    Serial.print("IR[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.println(pulsos[i]);
  }

  if (angulo < 0.0) {
    Serial.println("Angulo: sem deteccao");
  } else {
    Serial.print("Angulo: ");
    Serial.print(angulo, 1);
    Serial.println(" graus");
  }

  Serial.println();
  delay(200);
}

