// Teste isolado de alinhamento por PID usando erro de gol.
// Funcao: receber "GOL:erro,det,..." da Cabeca e girar no eixo ate alinhar.
// Entrada: erro angular do gol via serial.
// Saida: comando de giro nos quatro motores para validar KP/KI/KD.
#include <Arduino.h>

#define RX_CABECA 17
#define TX_CABECA 18

#define IN1_1_A 5
#define IN2_1_A 6
#define PWM_1_A 4

#define IN1_2_A 3
#define IN2_2_A 46
#define PWM_2_A 7

#define IN1_1_B 11
#define IN2_1_B 12
#define PWM_1_B 10

#define IN1_2_B 13
#define IN2_2_B 14
#define PWM_2_B 47

#define PWM_CH1 0
#define PWM_CH2 1
#define PWM_CH3 2
#define PWM_CH4 3

#define PWM_FREQ 20000
#define PWM_RES 8

const int VELOCIDADE_MAXIMA = 180;
const float TOLERANCIA_ERRO_GRAUS = 5.0f;

// Ganhos para ajuste manual durante teste
float PID_KP = 1.0;
float PID_KI = 0.0f;
float PID_KD = 0.8;

const float PID_INTEGRAL_MAX = 120.0f;
const int PID_SAIDA_MIN = 25;
const int PID_SAIDA_MAX = 140;

const unsigned long INTERVALO_OI_MS = 500;
const unsigned long TIMEOUT_COM_MS = 2500;

HardwareSerial SerialCabeca(1);

String bufferSerial = "";
unsigned long ultimoEnvioOiMs = 0;
unsigned long ultimoRxMs = 0;

bool comunicacaoCabecaOK = false;
bool golDetectado = false;
float erroGolGraus = 0.0f;

float pidIntegral = 0.0f;
float pidErroAnterior = 0.0f;
unsigned long pidUltimoMs = 0;

void Motor_1(int vel1) {
  int pwm1 = constrain(abs(vel1), 0, 255);
  ledcWrite(PWM_CH1, pwm1);
  if (vel1 <= 0) {
    digitalWrite(IN1_1_A, HIGH);
    digitalWrite(IN2_1_A, LOW);
  } else {
    digitalWrite(IN1_1_A, LOW);
    digitalWrite(IN2_1_A, HIGH);
  }
}

void Motor_2(int vel2) {
  int pwm2 = constrain(abs(vel2), 0, 255);
  ledcWrite(PWM_CH2, pwm2);
  if (vel2 <= 0) {
    digitalWrite(IN1_2_A, HIGH);
    digitalWrite(IN2_2_A, LOW);
  } else {
    digitalWrite(IN1_2_A, LOW);
    digitalWrite(IN2_2_A, HIGH);
  }
}

void Motor_4(int vel3) {
  int pwm3 = constrain(abs(vel3), 0, 255);
  ledcWrite(PWM_CH3, pwm3);
  if (vel3 <= 0) {
    digitalWrite(IN1_1_B, HIGH);
    digitalWrite(IN2_1_B, LOW);
  } else {
    digitalWrite(IN1_1_B, LOW);
    digitalWrite(IN2_1_B, HIGH);
  }
}

void Motor_3(int vel4) {
  int pwm4 = constrain(abs(vel4), 0, 255);
  ledcWrite(PWM_CH4, pwm4);
  if (vel4 <= 0) {
    digitalWrite(IN1_2_B, HIGH);
    digitalWrite(IN2_2_B, LOW);
  } else {
    digitalWrite(IN1_2_B, LOW);
    digitalWrite(IN2_2_B, HIGH);
  }
}

void pararMotores() {
  ledcWrite(PWM_CH1, 0);
  ledcWrite(PWM_CH2, 0);
  ledcWrite(PWM_CH3, 0);
  ledcWrite(PWM_CH4, 0);

  digitalWrite(IN1_1_A, LOW);
  digitalWrite(IN2_1_A, LOW);
  digitalWrite(IN1_2_A, LOW);
  digitalWrite(IN2_2_A, LOW);
  digitalWrite(IN1_1_B, LOW);
  digitalWrite(IN2_1_B, LOW);
  digitalWrite(IN1_2_B, LOW);
  digitalWrite(IN2_2_B, LOW);
}

void girarNoEixo(int velocidade) {
  int vel = constrain(velocidade, -VELOCIDADE_MAXIMA, VELOCIDADE_MAXIMA);
  // horario: M1/M2 frente, M3/M4 tras; anti-horario: inverso
  Motor_1(-vel);
  Motor_2(-vel);
  Motor_3(-vel);
  Motor_4(-vel);
}

void resetPid() {
  pidIntegral = 0.0f;
  pidErroAnterior = 0.0f;
  pidUltimoMs = 0;
}

int calcularSaidaPid(float erro) {
  unsigned long agora = millis();
  float dt = 0.02f;
  if (pidUltimoMs != 0) {
    dt = (agora - pidUltimoMs) / 1000.0f;
    if (dt < 0.005f) dt = 0.005f;
    if (dt > 0.2f) dt = 0.2f;
  }
  pidUltimoMs = agora;

  pidIntegral += erro * dt;
  if (pidIntegral > PID_INTEGRAL_MAX) pidIntegral = PID_INTEGRAL_MAX;
  if (pidIntegral < -PID_INTEGRAL_MAX) pidIntegral = -PID_INTEGRAL_MAX;

  float derivada = (erro - pidErroAnterior) / dt;
  pidErroAnterior = erro;

  float u = PID_KP * erro + PID_KI * pidIntegral + PID_KD * derivada;

  int saida = (int)fabsf(u);
  if (saida < PID_SAIDA_MIN) saida = PID_SAIDA_MIN;
  if (saida > PID_SAIDA_MAX) saida = PID_SAIDA_MAX;

  return (u >= 0.0f) ? saida : -saida;
}

void processarMensagemCabeca(String msg) {
  msg.trim();
  msg.toUpperCase();

  if (msg == "OI") {
    comunicacaoCabecaOK = true;
    ultimoRxMs = millis();
    return;
  }

  if (msg.startsWith("GOL:")) {
    int p1 = msg.indexOf(',');
    int p2 = msg.indexOf(',', p1 + 1);
    if (p1 > 0 && p2 > p1) {
      String sErro = msg.substring(4, p1);
      String sDet = msg.substring(p1 + 1, p2);
      sErro.trim();
      sDet.trim();

      erroGolGraus = sErro.toFloat();
      golDetectado = (sDet == "1");

      comunicacaoCabecaOK = true;
      ultimoRxMs = millis();
    }
  }
}

void lerSerialCabeca() {
  while (SerialCabeca.available() > 0) {
    char c = (char)SerialCabeca.read();
    if (c == '\n' || c == '\r') {
      if (bufferSerial.length() > 0) {
        processarMensagemCabeca(bufferSerial);
        bufferSerial = "";
      }
      continue;
    }

    if (bufferSerial.length() < 64) {
      bufferSerial += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  SerialCabeca.begin(9600, SERIAL_8N1, RX_CABECA, TX_CABECA);

  pinMode(IN1_1_A, OUTPUT);
  pinMode(IN2_1_A, OUTPUT);
  pinMode(IN1_2_A, OUTPUT);
  pinMode(IN2_2_A, OUTPUT);
  pinMode(IN1_1_B, OUTPUT);
  pinMode(IN2_1_B, OUTPUT);
  pinMode(IN1_2_B, OUTPUT);
  pinMode(IN2_2_B, OUTPUT);

  ledcSetup(PWM_CH1, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_1_A, PWM_CH1);
  ledcSetup(PWM_CH2, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_2_A, PWM_CH2);
  ledcSetup(PWM_CH3, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_1_B, PWM_CH3);
  ledcSetup(PWM_CH4, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_2_B, PWM_CH4);

  pararMotores();

  Serial.println("TESTE PIDBussola iniciado");
  Serial.println("Alinhamento por erro da camera (GOL:erro,det,...)");
  Serial.println("Ajuste KP/KI/KD no codigo e reflashe para testar");
}

void loop() {
  if ((millis() - ultimoEnvioOiMs) >= INTERVALO_OI_MS) {
    SerialCabeca.println("oi");
    ultimoEnvioOiMs = millis();
  }

  lerSerialCabeca();

  if ((millis() - ultimoRxMs) > TIMEOUT_COM_MS) {
    comunicacaoCabecaOK = false;
    golDetectado = false;
  }

  if (comunicacaoCabecaOK && golDetectado) {
    if (fabsf(erroGolGraus) <= TOLERANCIA_ERRO_GRAUS) {
      resetPid();
      pararMotores();
    } else {
      int cmd = calcularSaidaPid(erroGolGraus);
      girarNoEixo(cmd);
    }
  } else {
    resetPid();
    pararMotores();
  }

  static unsigned long tPrint = 0;
  if ((millis() - tPrint) > 120) {
    tPrint = millis();
    Serial.print("COM=");
    Serial.print(comunicacaoCabecaOK ? "OK" : "FALHA");
    Serial.print(" GOL=");
    Serial.print(golDetectado ? "1" : "0");
    Serial.print(" ERRO=");
    Serial.print(erroGolGraus, 2);
    Serial.print(" PID[int]=");
    Serial.print(pidIntegral, 2);
    Serial.print(" KP=");
    Serial.print(PID_KP, 2);
    Serial.print(" KI=");
    Serial.print(PID_KI, 3);
    Serial.print(" KD=");
    Serial.println(PID_KD, 2);
  }

  delay(5);
}
