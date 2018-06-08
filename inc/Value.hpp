#pragma once

#include "utils.hpp"

namespace sophia {

struct Value {
  enum Type {
    eBlob = 0x00,
    eContract = 0x01,
    eLibrary = 0x02,
    eNodeProfile = 0x10,
    eTopicProfile = 0x11,
  } type;

  uint32_t revision;
  u256 id;
  u512 signature;
  std::vector<u8> data;

  void read(cbuff_view_t &pSource);
  void write(buff_view_t &pDest, bool pWithSign = true) const;

  size_t serializedSize() const;
  u512 computeSignature(const sensitive_t<u512> &pKey) const;
  bool signatureValid() const;

  cbuff_view_t view() const;
};

void randValue(Value *pOut);

} // namespace sophia
