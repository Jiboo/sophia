#pragma once

#include <cstdint>

#include <array>
#include <chrono>
#include <iostream>

#include <experimental/filesystem>

#include <boost/asio.hpp>
#include <utility>

#include <sodium.h>

#define SOPHIA_TIMEOUT boost::posix_time::seconds(1)
#define SOPHIA_K 20
#define SOPHIA_ALPHA 3

#define SOPHIA_CCALL(expr)                                                                                             \
  if (expr)                                                                                                            \
    throw std::runtime_error(std::string(#expr) + " failed at " __FILE__ ":" + std::to_string(__LINE__));

namespace sophia {

namespace net = boost::asio;
namespace fs = std::experimental::filesystem;

using Clock = std::chrono::steady_clock;
using ErrorCode = boost::system::error_code;

static ErrorCode sNoError = ErrorCode(boost::system::errc::success, boost::system::generic_category());
static ErrorCode sTimeoutError = ErrorCode(boost::system::errc::timed_out, boost::system::generic_category());

class sophia_fatal : public std::runtime_error {
public:
  inline sophia_fatal(ErrorCode pError, const char *pDesc)
      : std::runtime_error(std::string(pDesc) + " (" + std::to_string(pError.value()) + ": " + pError.message() + ")") {
  }
};

using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;

struct buff_view_t {
  u8 *data;
  size_t size;

  void seek(size_t pBytes);
};

struct cbuff_view_t {
  const u8 *data;
  size_t size;

  void seek(size_t pBytes);
};
std::ostream &operator<<(std::ostream &pOS, const buff_view_t &pBuff);
std::ostream &operator<<(std::ostream &pOS, const cbuff_view_t &pBuff);

void parseHex(const buff_view_t &pDest, const std::string_view &pSource);

template <const size_t TSize> struct buff_t : public std::array<u8, TSize> {
  static constexpr size_t sSize = TSize;

  buff_t() = default;
  bool operator==(const buff_t<TSize> &pOther) {
    return memcmp(std::array<u8, TSize>::data(), pOther.data(), TSize) == 0;
  }

  buff_view_t view() { return buff_view_t{std::array<u8, TSize>::data(), std::array<u8, TSize>::size()}; }

  cbuff_view_t view() const { return cbuff_view_t{std::array<u8, TSize>::data(), std::array<u8, TSize>::size()}; }
};

void readBuff(const buff_view_t &pDest, cbuff_view_t &pSource);
void writeBuff(buff_view_t &pDest, const cbuff_view_t &pSource);

template <typename T> void readField(T *pDest, cbuff_view_t &pSource) {
  buff_view_t lDest{(u8 *)pDest, sizeof(T)};
  readBuff(lDest, pSource);
}

template <typename T> void writeField(buff_view_t &pDest, const T &pSource) {
  cbuff_view_t lSource{(u8 *)&pSource, sizeof(T)};
  writeBuff(pDest, lSource);
}

template <const size_t TSize> std::ostream &operator<<(std::ostream &pOS, const buff_t<TSize> &pBuff) {
  size_t lHexLen = pBuff.size() * 2 + 1;
  char lHex[lHexLen];
  sodium_bin2hex(lHex, lHexLen, pBuff.data(), pBuff.size());
  return pOS << lHex;
}

using u128 = buff_t<16>;
using u192 = buff_t<24>;
using u256 = buff_t<32>;
using u512 = buff_t<64>;

struct dur_t {
  std::chrono::nanoseconds time;

  template <typename TRep, typename TPeriod>
  explicit dur_t(std::chrono::duration<TRep, TPeriod> pTime)
      : time(std::chrono::duration_cast<std::chrono::nanoseconds>(pTime)) {}
};
std::ostream &operator<<(std::ostream &pOS, const dur_t &pDur);

template <typename T> struct sensitive_t {
  T contained;

  sensitive_t() { sodium_mlock(contained.data(), contained.size()); }
  explicit sensitive_t(T pInit) : contained(std::move(std::move(std::move(pInit)))) {
    sodium_mlock(contained.data(), contained.size());
  }

  ~sensitive_t() {
    sodium_memzero(contained.data(), contained.size());
    sodium_munlock(contained.data(), contained.size());
  }

  T *operator->() { return &contained; }

  const T *operator->() const { return &contained; }
};

using passphrase_t = sensitive_t<std::string>;

struct keypairs_t {
  u256 publicKey;
  sensitive_t<u256> privateKey;

  u256 hashPublicKey;
  sensitive_t<u512> hashPrivateKey;
};

template <typename T> T rand() {
  T lResult;
  randombytes_buf(lResult.data(), lResult.size());
  return lResult;
}

u256 dist256(const u8 *pLeft, const u8 *pRight);

/** Compares the distance of pLeft and pRight relative to pRef
 * @return 0 if pLeft and pRight are as closer, return <0 if pLeft is closer, >0 if pRight is closer */
int closerDist256(const u8 *pRef, const u8 *pLeft, const u8 *pRight);

size_t clzDist256(const u8 *pLeft, const u8 *pRight);

u256 randSuffix(const u8 *pRef, size_t pSamePrefixBits);

template <size_t TSize> struct array_hasher_t {
  using sType = std::array<u8, TSize>;

  inline std::size_t operator()(const sType &p) const {
    std::string_view pView{(const char *)p.data(), p.size()};
    return std::hash<std::string_view>()(pView);
  }
};

} // namespace sophia
