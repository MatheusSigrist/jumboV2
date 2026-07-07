#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <string.h>
#include "nexus_config.hpp"

/*
  Biblioteca global de comunicacao serial entre placas.

  Objetivo:
  - Unificar envio e recebimento entre Cabeca, Musculo, Olho e Pe.
  - Permitir evoluir payload sem reescrever parser em todas as placas.
  - Manter API simples: escolher rota + passar struct de dados.

  Modelo de frame:
    [START=0xA6][ROTA][LEN_L][LEN_H][PAYLOAD...][CRC8][STOP=0x55]

  API principal:
  - enviarStruct<T>(serial, rota, dados)
    Envia qualquer struct como payload binario.
    Parametros:
    - serial: Stream de saida (exemplo: Serial1, Serial2).
    - rota: identificador logico do sentido (ex.: PE_PARA_CABECA).
    - dados: struct com os campos que voce quer transmitir.

  - enviarBytes(serial, rota, payload, tamanho)
    Envia payload raw (uint8_t*).
    Parametros:
    - serial: Stream de saida.
    - rota: identificador logico da mensagem.
    - payload: ponteiro para bytes de payload.
    - tamanho: quantidade de bytes do payload.

  - Receptor::poll(serial, frame)
    Consome bytes da serial e retorna true quando um frame completo e valido chegar.
    Parametros:
    - serial: Stream de entrada.
    - frame: estrutura de saida com rota, tamanho e bytes do payload.

  - lerStruct<T>(frame, out)
    Converte payload recebido para struct tipada.
    Parametros:
    - frame: frame validado pelo Receptor.
    - out: struct de destino.

  Como escalar quando adicionar/remover informacao:
  1) Altere apenas o struct do payload daquela rota.
  2) Mantenha o envio via enviarStruct.
  3) Mantenha recepcao via poll + lerStruct.
  4) Opcionalmente use versionamento de struct se precisar compatibilidade retroativa.
*/

namespace ComunicacaoUnificada {

static const uint8_t kStartByte = NexusConfig::Comunicacao::PROTO_START;
static const uint8_t kStopByte = NexusConfig::Comunicacao::PROTO_STOP;
static const uint16_t kMaxPayload = NexusConfig::Comunicacao::PROTO_MAX_PAYLOAD;

// Rotas logicas entre placas.
enum class Rota : uint8_t {
  DESCONHECIDA = 0,
  CABECA_PARA_PE = 1,
  PE_PARA_CABECA = 2,
  CABECA_PARA_OLHO = 3,
  OLHO_PARA_CABECA = 4,
  CABECA_PARA_MUSCULO = 5,
  MUSCULO_PARA_CABECA = 6,
  MUSCULO_PARA_PE = 7,
  PE_PARA_MUSCULO = 8,
  CABECA_PARA_TODOS = 9,
  CABECA_PARA_PE_COMANDO = 10,
  PE_PARA_CABECA_SENSORES = 11,
  PE_PARA_CABECA_LIMIAR = 12
};

struct Frame {
  Rota rota = Rota::DESCONHECIDA;
  uint16_t tamanho = 0;
  uint8_t payload[kMaxPayload] = {0};
};

inline uint8_t crc8(const uint8_t* data, uint16_t len) {
  uint8_t crc = 0;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80) {
        crc = (uint8_t)((crc << 1) ^ 0x07);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

inline bool enviarBytes(Stream& serial, Rota rota, const uint8_t* payload, uint16_t tamanho) {
  if (tamanho > kMaxPayload) {
    return false;
  }

  uint8_t cabecalho[4];
  cabecalho[0] = kStartByte;
  cabecalho[1] = (uint8_t)rota;
  cabecalho[2] = (uint8_t)(tamanho & 0xFF);
  cabecalho[3] = (uint8_t)((tamanho >> 8) & 0xFF);

  uint8_t crcData[1 + 2 + kMaxPayload];
  crcData[0] = cabecalho[1];
  crcData[1] = cabecalho[2];
  crcData[2] = cabecalho[3];
  if (tamanho > 0 && payload != nullptr) {
    memcpy(&crcData[3], payload, tamanho);
  }
  uint8_t crc = crc8(crcData, (uint16_t)(3 + tamanho));

  serial.write(cabecalho, sizeof(cabecalho));
  if (tamanho > 0 && payload != nullptr) {
    serial.write(payload, tamanho);
  }
  serial.write(crc);
  serial.write(kStopByte);
  return true;
}

template <typename T>
inline bool enviarStruct(Stream& serial, Rota rota, const T& dados) {
  return enviarBytes(serial, rota, reinterpret_cast<const uint8_t*>(&dados), (uint16_t)sizeof(T));
}

template <typename T>
inline bool lerStruct(const Frame& frame, T& out) {
  if (frame.tamanho != sizeof(T)) {
    return false;
  }
  memcpy(&out, frame.payload, sizeof(T));
  return true;
}

class Receptor {
 public:
  Receptor() { reset(); }

  void reset() {
    estado_ = Estado::ESPERANDO_START;
    rota_ = 0;
    tam_ = 0;
    idx_ = 0;
    crcRx_ = 0;
  }

  bool poll(Stream& serial, Frame& frameOut) {
    while (serial.available() > 0) {
      // Modo nao-intrusivo quando aguardando START:
      // se o proximo byte nao e deste protocolo, nao consome.
      if (estado_ == Estado::ESPERANDO_START) {
        int proximo = serial.peek();
        if (proximo < 0) {
          return false;
        }
        if ((uint8_t)proximo != kStartByte) {
          return false;
        }
      }

      uint8_t b = (uint8_t)serial.read();
      if (consumirByte(b, frameOut)) {
        return true;
      }
    }
    return false;
  }

 private:
  enum class Estado : uint8_t {
    ESPERANDO_START = 0,
    LENDO_ROTA,
    LENDO_LEN_L,
    LENDO_LEN_H,
    LENDO_PAYLOAD,
    LENDO_CRC,
    LENDO_STOP
  };

  Estado estado_;
  uint8_t rota_;
  uint16_t tam_;
  uint16_t idx_;
  uint8_t crcRx_;
  uint8_t payload_[kMaxPayload];

  bool consumirByte(uint8_t b, Frame& frameOut) {
    switch (estado_) {
      case Estado::ESPERANDO_START:
        if (b == kStartByte) {
          estado_ = Estado::LENDO_ROTA;
        }
        return false;

      case Estado::LENDO_ROTA:
        rota_ = b;
        estado_ = Estado::LENDO_LEN_L;
        return false;

      case Estado::LENDO_LEN_L:
        tam_ = b;
        estado_ = Estado::LENDO_LEN_H;
        return false;

      case Estado::LENDO_LEN_H:
        tam_ |= ((uint16_t)b << 8);
        if (tam_ > kMaxPayload) {
          reset();
          return false;
        }
        idx_ = 0;
        estado_ = (tam_ == 0) ? Estado::LENDO_CRC : Estado::LENDO_PAYLOAD;
        return false;

      case Estado::LENDO_PAYLOAD:
        payload_[idx_++] = b;
        if (idx_ >= tam_) {
          estado_ = Estado::LENDO_CRC;
        }
        return false;

      case Estado::LENDO_CRC:
        crcRx_ = b;
        estado_ = Estado::LENDO_STOP;
        return false;

      case Estado::LENDO_STOP:
        if (b != kStopByte) {
          reset();
          return false;
        }

        if (!validarCrc()) {
          reset();
          return false;
        }

        frameOut.rota = (Rota)rota_;
        frameOut.tamanho = tam_;
        if (tam_ > 0) {
          memcpy(frameOut.payload, payload_, tam_);
        }
        reset();
        return true;
    }

    reset();
    return false;
  }

  bool validarCrc() const {
    uint8_t crcData[1 + 2 + kMaxPayload];
    crcData[0] = rota_;
    crcData[1] = (uint8_t)(tam_ & 0xFF);
    crcData[2] = (uint8_t)((tam_ >> 8) & 0xFF);
    if (tam_ > 0) {
      memcpy(&crcData[3], payload_, tam_);
    }
    uint8_t crcCalc = crc8(crcData, (uint16_t)(3 + tam_));
    return (crcCalc == crcRx_);
  }
};

}  // namespace ComunicacaoUnificada
