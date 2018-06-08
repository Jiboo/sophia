#pragma once

#include <list>
#include <memory>

#include <boost/functional/hash.hpp>

#include "Database.hpp"
#include "Table.hpp"

namespace sophia {

class Node;

enum MessageType {
  eError = 0x00,
  ePing = 0x01,
  ePong = 0x02,

  eClosestNodes = 0x10,
  eClosestValues = 0x11,
  eFindValue = 0x12,
  eStoreValue = 0x13,

  eNodesResult = 0x20,
  eValuesResult = 0x21,
  eValueResult = 0x22,

  ePubsubJoin = 0x30,
  ePubsubLeave = 0x31,
  ePubsubClosestNodes = 0x32,
  ePubsubNodesResult = 0x33,
  ePubsubEvent = 0x34,
};

enum SophiaError {
  eNoError = 0x0,
  eUnspecified = 0x1,

  eLocalStoreFull = 0x1300,
  eKeyAlreadyAssigned = 0x1301,
  eInvalidSignature = 0x1302,
  eNotUpToDate = 0x1303,
};

struct inflight_t {
  using Callback = std::function<void(ErrorCode, const Contact &, const u256 &, const cbuff_view_t &)>;
  net::deadline_timer timeout;
  u256 id;
  u256 dest;
  Callback callback;

  inflight_t(net::io_service &pService, const u256 &pDest, const u256 &pID, Callback pCallback);

  inflight_t(inflight_t &&pOther) = default;
};

template <typename TCallbackType> struct IterativeContext {
  Clock::time_point start;
  u256 key;
  TCallbackType callback;
  std::vector<Contact> inflight, searched;
  bool found = false;

  IterativeContext(const u256 &pKey, const TCallbackType &pCallback)
      : start(Clock::now()), key(pKey), callback(pCallback), inflight(SOPHIA_ALPHA) {}

  ~IterativeContext() {}

  bool isKnown(const Contact &pOther) const {
    return std::find(inflight.begin(), inflight.end(), pOther) != inflight.end() ||
           std::find(searched.begin(), searched.end(), pOther) != searched.end();
  }

  void markSearched(const Contact &pInflight) {
    for (auto it = inflight.begin(); it != inflight.end(); it++) {
      if (*it == pInflight) {
        inflight.erase(it);
        break;
      }
    }
    searched.push_back(pInflight);
  }
};

class Node {
  friend struct IterativeClosestNodesFunctor;
  friend struct IterativeFindValueFunctor;

  using udp = net::ip::udp;

  static constexpr size_t sBuffSize = 1280 - 40 - 8; // IPv6 MTU - IPv6 header - UDP header
  static constexpr size_t sHeaderSize = 72;          // ID + Nonce + Mac
  static constexpr size_t sNonceOffset = 32;
  static constexpr size_t sNonceSize = 24;
  static constexpr size_t sMacOffset = sNonceOffset + sNonceSize;
  static constexpr size_t sEntrySize = 32 + 16 + 2; // id, ipv6, port

  net::io_service &service;

  Database db;
  udp::socket sock;
  keypair_t keypair;
  Table routing;

  udp::endpoint lastDist;
  buff_t<sBuffSize> recvBuff;

  std::unordered_map<size_t, inflight_t> inflights;

  void send(const Contact &pDest, const cbuff_view_t &pBuff);
  void recieve();

  size_t prepareCommand(const Contact &pDest, const u256 &pID, MessageType pCommandType, u8 *pBuffer,
                        const inflight_t::Callback &pCallback);

  void prepareReply(const Contact &pSource, MessageType pReplyType, const u8 *pCommandBuffer, u8 *pReplyBuffer);
  void recvResponse(const Contact &pSource, const cbuff_view_t &pBuff);

  size_t hashInflight(const u256 &pDest, uint32_t pToken);

  cbuff_view_t readToken(const u8 *pData);

  using PingCallback = std::function<void(ErrorCode pError, const Contact &pSource)>;
  size_t sendPing(const Contact &pDest, const PingCallback &pCallback);
  void replyPing(const Contact &pSource, const cbuff_view_t &pBuff);

  using ClosestNodesCallback =
      std::function<void(ErrorCode pError, const Contact &pSource, const std::vector<Contact> &)>;
  using IterativeClosestNodesCallback = std::function<void(const std::vector<Contact> &)>;
  struct IterativeClosestNodesFunctor {
    IterativeContext<IterativeClosestNodesCallback> *context;
    Node *node;
    void operator()(ErrorCode pError, const Contact &pSource, const std::vector<Contact> &pResults);
  };

  size_t sendClosestNodes(const Contact &pDest, const u256 &pKey, const ClosestNodesCallback &pCallback);
  void replyClosestNodes(const Contact &pSource, const cbuff_view_t &pBuff);
  void iterativeClosestNodes(const u256 &pKey, const IterativeClosestNodesCallback &pCallback);

  using FindValueCallback = std::function<void(ErrorCode pError, const Contact &pSource, const std::vector<Contact> &,
                                               const std::optional<Value> &)>;
  using IterativeFindValueCallback = std::function<void(const std::optional<Value> &)>;
  struct IterativeFindValueFunctor {
    IterativeContext<IterativeFindValueCallback> *context;
    Node *node;
    void operator()(ErrorCode pError, const Contact &pSource, const std::vector<Contact> &pResults,
                    const std::optional<Value> &pVal);
  };

  size_t sendFindValue(const Contact &pDest, const u256 &pKey, const FindValueCallback &pCallback);
  void replyFindValue(const Contact &pSource, const cbuff_view_t &pBuff);
  void iterativeFindValue(const u256 &pKey, const IterativeFindValueCallback &pCallback);

  using StoreCallback = std::function<void(ErrorCode pError, const Contact &pSource)>;
  size_t sendStore(const Contact &pDest, const Value &pValue, const StoreCallback &pCallback);
  void replyStore(const Contact &pSource, const cbuff_view_t &pBuff);

public:
  Node(net::io_service &pService, const char *pDBPath, const passphrase_t &pPassphrase, const udp::endpoint &pEndpoint);

  Contact self() const;

  void bootstrap(const Contact &pContact);
  void refreshBuckets();

  using PutCallback = std::function<void()>;
  void put(const Value &pValue, const PutCallback &pCallback);

  using GetCallback = std::function<void(const std::optional<Value> &)>;
  void get(const u256 &pKey, const GetCallback &pCallback);

  inline void debugRouteTable() { routing.debug(); }
};

} // namespace sophia