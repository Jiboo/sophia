#pragma once

#include <list>
#include <memory>

#include <boost/functional/hash.hpp>
#include <unordered_set>
#include <utility>

#include "Database.hpp"
#include "Table.hpp"

namespace sophia {

class Node;

enum MessageType {
  eResult = 0x00,

  ePing = 0x10,
  eClosestNodes = 0x11,
  eFindValue = 0x12,
  eStoreValue = 0x13,

  ePong = 0x20,
  eNodesResult = 0x21,
  eValueResult = 0x22,

  ePubsubJoin = 0x30,
  ePubsubClosestNodes = 0x31,
  ePubsubEvent = 0x32,

  ePubsubNodesResult = 0x38,

  eRPC = 0x40,
};

enum class SophiaErrorCode {
  eNoError = 0x0,
  eUnspecified = 0x1,
  eIllformed = 0x2,

  eMTUTooLow = 0x1000,

  eLocalStoreFull = 0x1300,
  eKeyAlreadyAssigned = 0x1301,
  eInvalidSignature = 0x1302,
  eNotUpToDate = 0x1303,

  eNotRegisted = 0x3000,

  eNotSubscribed = 0x4000,
  eInternalError = 0x4001,
};

struct SophiaErrorCategory : boost::system::error_category {
  const char *name() const noexcept override;
  std::string message(int ev) const override;
  static const SophiaErrorCategory &instance();
  static ErrorCode wrap(SophiaErrorCode pCode);
};

struct inflight_t {
  using Callback = std::function<void(ErrorCode, const Contact &, const u256 &, const cbuff_view_t &)>;
  net::deadline_timer timeout;
  MessageType type;
  Contact dest;
  u256 id;
  Callback callback;

  inflight_t(net::io_service &pService, MessageType pType, Contact pDest, const u256 &pID, Callback pCallback);

  inflight_t(inflight_t &&pOther) = default;
};

template <typename TCallbackType> struct IterativeContext {
  Clock::time_point start;
  u256 key;
  TCallbackType callback;
  std::vector<Contact> inflight, searched;
  bool found = false;

  IterativeContext(const u256 &pKey, TCallbackType pCallback)
      : start(Clock::now()), key(pKey), callback(std::move(pCallback)), inflight(SOPHIA_ALPHA) {}

  virtual ~IterativeContext() = default;

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

template <typename TCallbackType> struct IterativePubsubContext : public IterativeContext<TCallbackType> {
  u256 topic;

  IterativePubsubContext(const u256 &pTopic, const u256 &pKey, TCallbackType pCallback)
      : IterativeContext<TCallbackType>(pKey, pCallback), topic(pTopic) {}

  ~IterativePubsubContext() override = default;
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
  keypairs_t keypair;

  Table routing;

  struct TopicContext {
    using EventCallback = std::function<void(const Event &)>;
    Table routing;
    EventCallback callback;
    std::unordered_set<u512, array_hasher_t<u512::sSize>> knownEvents; // TODO Circular buffer
  };
  std::unordered_map<u256, TopicContext, array_hasher_t<u256::sSize>> topics;

  udp::endpoint lastDist;
  buff_t<sBuffSize> recvBuff;

  std::unordered_map<size_t, inflight_t> inflights;

  void send(const Contact &pDest, const cbuff_view_t &pBuff);
  void recieve();

  size_t prepareCommand(const Contact &pDest, const u256 &pID, MessageType pCommandType, u8 *pBuffer,
                        const inflight_t::Callback &pCallback = {});

  void prepareReply(const Contact &pSource, MessageType pReplyType, const u8 *pCommandBuffer, u8 *pReplyBuffer);
  void recvResponse(const Contact &pSource, const cbuff_view_t &pBuff);

  /**
   * Note: Don't reply to an illformed command
   */
  void replyWithResult(const Contact &pSource, const u8 *pCommandBuffer, SophiaErrorCode pError);

  size_t hashInflight(const u256 &pDest, uint32_t pToken);

  cbuff_view_t readToken(const u8 *pData);

  using PingCallback = std::function<void(ErrorCode pError, const Contact &pSource)>;
  size_t sendPing(const Contact &pDest, const PingCallback &pCallback);
  void replyPing(const Contact &pSource, const cbuff_view_t &pBuff);

  using ClosestNodesCallback =
      std::function<void(ErrorCode pError, const Contact &pSource, const std::vector<Contact> &)>;
  using IterativeClosestNodesCallback = std::function<void(const std::vector<Contact> &)>;
  struct IterativeClosestNodesFunctor {
    std::shared_ptr<IterativeContext<IterativeClosestNodesCallback>> context;
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
    std::shared_ptr<IterativeContext<IterativeFindValueCallback>> context;
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

  using PubsubJoinCallback =
      std::function<void(ErrorCode pError, const Contact &pSource, const std::vector<Contact> &)>;
  size_t sendPubsubJoin(const Contact &pDest, const u256 &pTopicID, const PubsubJoinCallback &pCallback);
  void replyPubsubJoin(const Contact &pSource, const cbuff_view_t &pBuff);

  struct IterativePubsubClosestNodesFunctor {
    std::shared_ptr<IterativePubsubContext<IterativeClosestNodesCallback>> context;
    Node *node;
    void operator()(ErrorCode pError, const Contact &pSource, const std::vector<Contact> &pResults);
  };

  size_t sendPubsubClosestNodes(const Contact &pDest, const u256 &pTopic, const u256 &pKey,
                                const ClosestNodesCallback &pCallback);
  void replyPubsubClosestNodes(const Contact &pSource, const cbuff_view_t &pBuff);
  void iterativePubsubClosestNodes(const u256 &pTopic, const u256 &pKey,
                                   const IterativeClosestNodesCallback &pCallback);

  size_t sendPubsubEvent(const Contact &pDest, const Event &pEvent);
  void handlePubsubEvent(const Contact &pSource, const cbuff_view_t &pBuff);
  void broadcastPubsubEvent(const Event &pEvent);

  void handleRPC(const Contact &pSource, const cbuff_view_t &pBuff);

  void subscribeToKnownTopics();

public:
  Node(net::io_service &pService, const char *pDBPath, const passphrase_t &pPassphrase, const udp::endpoint &pEndpoint);

  Contact self() const;

  using BootstrapCallback = std::function<void()>;
  void bootstrap(const Contact &pContact, const BootstrapCallback &pCallback);
  void refreshBuckets();
  void refreshTopicBuckets(const u256 &pTopicID);
  void refreshTopic(const u256 &pTopicID);

  using GetCallback = std::function<void(const std::optional<Value> &)>;
  void get(const u256 &pKey, const GetCallback &pCallback);

  using PutCallback = std::function<void()>;
  void put(const Value &pValue, const PutCallback &pCallback);
  u256 put(Value::Type pType, uint32_t pRevision, const u256 &pParent, const cbuff_view_t &pData,
           const PutCallback &pCallback);
  void update(const u256 &pID, uint32_t pNewRevision, const cbuff_view_t &pData, const PutCallback &pCallback);

  using JoinCallback = std::function<void()>;
  using EventCallback = TopicContext::EventCallback;
  u256 createTopic(const EventCallback &pEventCallback, const JoinCallback &pJoinCallback);
  void subscribe(const u256 &pTopicID, const EventCallback &pEventCallback, const JoinCallback &pJoinCallback);
  void publish(const u256 &pTopicID, u8 pEventType, u16 pEventExtra, const cbuff_view_t &pData);

  using RPCCallback = std::function<void(ErrorCode)>;
  void rpc(const u256 &pTargetNodeID, const u256 &pTargetContract, u32 pParam32, const u256 &pParam256,
           const u512 &pParam512, const cbuff_view_t &pData, const RPCCallback &pCallback);

#ifdef SOPHIA_EXTRA_API
  static std::unordered_map<MessageType, uint64_t> sSentMsg;
  static std::unordered_map<MessageType, Clock::time_point> sLastRecvMsg;
  static Clock::time_point sLastProcessedEvent;
  inline const Table &routingTable() const { return routing; }
  inline Table &topicRoutingTable(const u256 &pTopic) { return topics[pTopic].routing; }
  using SimplePingCallback = std::function<void()>;
  inline void ping(const Contact &pContact, const SimplePingCallback &pCb) {
    sendPing(pContact, [pCb](ErrorCode pError, const Contact &pSource) { pCb(); });
  }
  using SimpleClosestNodesCallback = IterativeClosestNodesCallback;
  inline void closestNodes(const u256 &pID, const SimpleClosestNodesCallback &pCb) {
    iterativeClosestNodes(pID, [pCb](const std::vector<Contact> &pContacts) { pCb(pContacts); });
  }
  inline void pubsubClosestNodes(const u256 &pTopicID, const u256 &pID, const SimpleClosestNodesCallback &pCb) {
    iterativePubsubClosestNodes(pTopicID, pID, [pCb](const std::vector<Contact> &pContacts) { pCb(pContacts); });
  }
#endif
};

} // namespace sophia
