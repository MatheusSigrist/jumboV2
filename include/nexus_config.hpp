#pragma once

#include <stdint.h>

namespace NexusConfig {
namespace Comunicacao {

static const uint8_t PROTO_START = 0xA6;
static const uint8_t PROTO_STOP = 0x55;
static const uint16_t PROTO_MAX_PAYLOAD = 192;

}  // namespace Comunicacao
}  // namespace NexusConfig
