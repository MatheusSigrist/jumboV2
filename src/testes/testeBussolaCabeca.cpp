// Teste dedicado da bussola QMC5883P na placa Cabeca.
// Funcao: iniciar sensor, calibrar min/max XY e calcular heading 0..360.
// Saida: leituras RAW/GAUSS e heading em graus no monitor serial.
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_QMC5883P.h>

// Pinagem informada por voce para a cabeca
#define I2C_SDA 8
#define I2C_SCL 9

Adafruit_QMC5883P compass;
bool compassReady = false;
unsigned long lastRetryMs = 0;

int16_t minX = 32767;
int16_t maxX = -32768;
int16_t minY = 32767;
int16_t maxY = -32768;

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

void resetCalibracaoXY() {
  minX = 32767;
  maxX = -32768;
  minY = 32767;
  maxY = -32768;
}

void atualizarCalibracaoXY(int16_t rawX, int16_t rawY) {
  if (rawX < minX) minX = rawX;
  if (rawX > maxX) maxX = rawX;
  if (rawY < minY) minY = rawY;
  if (rawY > maxY) maxY = rawY;
}

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

void scanI2C() {
  Serial.println("Varredura I2C:");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(" - dispositivo em 0x");
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) {
    Serial.println(" - nenhum dispositivo encontrado");
  }
}

bool iniciarBussola() {
  if (!compass.begin()) {
    return false;
  }

  compass.setRange(QMC5883P_RANGE_8G);
  compass.setMode(QMC5883P_MODE_CONTINUOUS);
  compass.setODR(QMC5883P_ODR_50HZ);
  compass.setOSR(QMC5883P_OSR_8);
  compass.setDSR(QMC5883P_DSR_1);
  return true;
}

void setup() {
  Serial.begin(115200);
  unsigned long inicioSerial = millis();
  while (!Serial && (millis() - inicioSerial) < 4000) {
    delay(10);
  }
  delay(300);

  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println();
  Serial.println("==============================");
  Serial.println("Iniciando teste da bussola QMC5883P...");
  Serial.print("SDA=");
  Serial.print(I2C_SDA);
  Serial.print(" SCL=");
  Serial.println(I2C_SCL);
  Serial.println("Se nao aparecer nada, verifique USB CDC/monitor.");

  compassReady = iniciarBussola();
  if (!compassReady) {
    Serial.println("ERRO: QMC5883P nao encontrada.");
    Serial.println("Verifique alimentacao e fios SDA/SCL.");
    scanI2C();
    Serial.println("Vou continuar tentando sem travar o programa...");
  } else {
    Serial.println("QMC5883P OK.");
  }
}

void loop() {
  if (!compassReady) {
    if (millis() - lastRetryMs >= 1000) {
      lastRetryMs = millis();
      Serial.println("Aguardando QMC5883P... tentando reconectar");
      compassReady = iniciarBussola();
      if (compassReady) {
        Serial.println("QMC5883P conectou agora.");
      }
    }
    delay(50);
    return;
  }

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 'r' || c == 'R') {
      resetCalibracaoXY();
      Serial.println("Calibracao XY resetada.");
      Serial.println("Gire a bussola em todas as direcoes por alguns segundos.");
    }
  }

  int16_t rawX = 0;
  int16_t rawY = 0;
  int16_t rawZ = 0;
  float gaussX = 0.0;
  float gaussY = 0.0;
  float gaussZ = 0.0;

  bool rawOk = compass.getRawMagnetic(&rawX, &rawY, &rawZ);
  bool gaussOk = compass.getGaussField(&gaussX, &gaussY, &gaussZ);

  if (!rawOk || !gaussOk) {
    Serial.println("Falha na leitura da QMC5883P");
    delay(300);
    return;
  }

  atualizarCalibracaoXY(rawX, rawY);

  Serial.println("===== LEITURA BUSSOLA =====");
  Serial.print("RAW X: ");
  Serial.print(rawX);
  Serial.print(" | RAW Y: ");
  Serial.print(rawY);
  Serial.print(" | RAW Z: ");
  Serial.println(rawZ);

  Serial.print("GAUSS X: ");
  Serial.print(gaussX, 4);
  Serial.print(" | GAUSS Y: ");
  Serial.print(gaussY, 4);
  Serial.print(" | GAUSS Z: ");
  Serial.println(gaussZ, 4);

  float x360 = eixoPara360(rawX, minX, maxX);
  float y360 = eixoPara360(rawY, minY, maxY);

  // Heading calculado do zero usando os eixos normalizados para 0..360.
  // Primeiro traz para centro [-180, +180], depois aplica atan2 e normaliza.
  float xCentro = x360 - 180.0f;
  float yCentro = y360 - 180.0f;
  float heading = atan2(yCentro, xCentro) * 180.0f / PI;
  heading = normalizarAngulo360(heading);

  Serial.print("CAL X[min/max]: ");
  Serial.print(minX);
  Serial.print("/");
  Serial.print(maxX);
  Serial.print(" | CAL Y[min/max]: ");
  Serial.print(minY);
  Serial.print("/");
  Serial.println(maxY);

  Serial.print("X em 0-360: ");
  Serial.print(x360, 2);
  Serial.print(" | Y em 0-360: ");
  Serial.println(y360, 2);

  Serial.print("Heading (graus): ");
  Serial.println(heading, 2);
  Serial.println();

  delay(300);
}
