#pragma once

#include "utils.hpp"

namespace sophia {

struct Value {
  enum class Type : uint8_t {
    eBlob = 0x00,
    eContract = 0x01,
    eLibrary = 0x02,
    eTopicBootstrap = 0x3,
  } type;

  uint32_t revision;
  u256 id, parent;
  u512 signature;
  std::vector<u8> data;

  void read(cbuff_view_t &pSource);
  void write(buff_view_t &pDest, bool pWithSign = true) const;

  size_t serializedSize() const;
  u512 computeSignature(const sensitive_t<u512> &pKey) const;
  bool signatureValid() const;

  cbuff_view_t view() const;
};

void randValue(Value &pOut);

struct Event {
  u256 topic, source;
  u512 signature;

  u8 height, type;
  u16 extra;
  std::vector<u8> data;

  void read(cbuff_view_t &pSource);
  void write(buff_view_t &pDest, bool pWithSign = true) const;

  size_t serializedSize() const;
  u512 computeSignature(const sensitive_t<u512> &pKey) const;
  bool signatureValid() const;

  cbuff_view_t view() const;
};

} // namespace sophia
