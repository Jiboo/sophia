#include <boost/endian/buffers.hpp>

#include "utils.hpp"

using namespace sophia;

void buff_view_t::seek(size_t pBytes) {
  data += pBytes;
  size -= pBytes;
}

void cbuff_view_t::seek(size_t pBytes) {
  data += pBytes;
  size -= pBytes;
}

std::ostream &sophia::operator<<(std::ostream &pOS, const buff_view_t &pBuff) {
  size_t lHexLen = pBuff.size * 2 + 1;
  char lHex[lHexLen];
  sodium_bin2hex(lHex, lHexLen, pBuff.data, pBuff.size);
  return pOS << lHex;
}

std::ostream &sophia::operator<<(std::ostream &pOS, const cbuff_view_t &pBuff) {
  return pOS << buff_view_t{(u8 *)pBuff.data, pBuff.size};
}

void sophia::parseHex(const buff_view_t &pDest, const std::string_view &pSource) {
  sodium_hex2bin(pDest.data, pDest.size, pSource.data(), pSource.size(), nullptr, nullptr, nullptr);
}

void sophia::readBuff(const buff_view_t &pDest, cbuff_view_t &pSource) {
  assert(pSource.size >= pDest.size);
  memcpy(pDest.data, pSource.data, pDest.size);
  pSource.seek(pDest.size);
}

void sophia::writeBuff(buff_view_t &pDest, const cbuff_view_t &pSource) {
  assert(pDest.size >= pSource.size);
  memcpy(pDest.data, pSource.data, pSource.size);
  pDest.seek(pSource.size);
}

std::ostream &sophia::operator<<(std::ostream &pOS, const dur_t &pDur) {
  auto lNano = pDur.time.count();
  if (lNano < 1000) {
    return pOS << lNano << "ns";
  }
  if (lNano < 1'000'000) {
    return pOS << lNano / 1000.0 << "µs";
  }
  if (lNano < 1'000'000'000) {
    return pOS << lNano / 1'000'000.0 << "ms";
  }
  return pOS << lNano / 1'000'000'000.0 << "s";
}

u256 sophia::dist256(const u8 *pLeft, const u8 *pRight) {
  u256 lResult;
  const auto lSize = lResult.size();
  for (size_t i = 0; i < lSize; i++) {
    lResult[i] = pLeft[i] ^ pRight[i];
  }
  return lResult;
}

int sophia::closerDist256(const u8 *pRef, const u8 *pLeft, const u8 *pRight) {
  for (size_t i = 0; i < 32; i++) {
    u8 lLeft = pRef[i] ^ pLeft[i];
    u8 lRight = pRef[i] ^ pRight[i];
    if (lLeft != lRight) {
      return lLeft < lRight ? -1 : 1;
    }
  }
  return 0;
}

size_t sophia::clzDist256(const u8 *pLeft, const u8 *pRight) {
  size_t lResult = 0;
  for (size_t i = 0; i < 32; i++) {
    u8 lByte = pLeft[i] ^ pRight[i];
    if (lByte == 0) {
      lResult += 8;
    } else {
      while ((lByte & 0x80u) == 0) {
        lByte <<= 1;
        lResult++;
      }
      break;
    }
  }
  return lResult;
}

u256 sophia::randSuffix(const u8 *pRef, size_t pSamePrefixBits) {
  auto lResult = rand<u256>();

  size_t lByteOffset = 0;
  while (pSamePrefixBits > 8) {
    lResult[lByteOffset] = pRef[lByteOffset];
    pSamePrefixBits -= 8;
    lByteOffset++;
  }

  uint8_t lByteMask = 0xFF;
  size_t lMaskBits = 8 - pSamePrefixBits;
  lByteMask <<= lMaskBits;

  lResult[lByteOffset] = (pRef[lByteOffset] & lByteMask) | (lResult[lByteOffset] & ~lByteMask);

  return lResult;
}
