#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace ComunicacaoNexus {

#pragma pack(push, 1)

struct EstadoRoboPayload {
  bool sozinho;
  bool atacante;
  bool corGolAzul;
};

struct LinhaAtacantePayload {
  int16_t angulo;
};

struct LinhaDefensorPayload {
  int16_t anguloZonaA;
  int16_t anguloZonaB;
  uint8_t temLinhaZonaA;
  uint8_t temLinhaZonaB;
};

struct OlhoTelemetriaPayload {
  int16_t uD;
  int16_t uE;
  int16_t uF;
  int16_t uT;
  int16_t angulo;
  int16_t intensidade;
  int16_t ballAngle;
  uint16_t ballDist;
  int16_t blueAngle;
  uint16_t blueDist;
  int16_t yellowAngle;
  uint16_t yellowDist;
  uint8_t cameraOK;
};

struct PeSensoresPayload {
  int16_t sensor1;
  int16_t sensor9;
  int16_t sensor17;
  int16_t sensor25;
};

struct PeLimiarPayload {
  int16_t limiar;
};

#pragma pack(pop)

enum class ComandoPe : uint8_t {
  REQ_SENS = 1,
  REQ_LIM = 2,
  SET_LIM = 3
};

#pragma pack(push, 1)
struct PeComandoPayload {
  uint8_t comando;
  int16_t valor;
};
#pragma pack(pop)

}  // namespace ComunicacaoNexus
