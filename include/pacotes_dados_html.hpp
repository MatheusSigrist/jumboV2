#pragma once

#include <Arduino.h>

namespace PacotesDadosHtml {

static const uint8_t MAP32_START = 0xAA;
static const uint8_t MAP32_STOP = 0x55;
static const uint8_t MAP32_ID = 0x32;
static const uint8_t MAP32_SENSOR_COUNT = 32;

#pragma pack(push, 1)
struct Mapa32Payload {
  uint16_t seq;
  uint16_t limiar;
  uint16_t sensores[MAP32_SENSOR_COUNT];
};
#pragma pack(pop)

inline uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x80) {
        crc = (uint8_t)((crc << 1) ^ 0x07);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

inline uint8_t crcMapa32(const Mapa32Payload& payload) {
  return crc8(reinterpret_cast<const uint8_t*>(&payload), sizeof(Mapa32Payload));
}

}  // namespace PacotesDadosHtml
