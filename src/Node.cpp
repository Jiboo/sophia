#include <utility>

#include "Node.hpp"

using namespace sophia;

constexpr auto sMaxClosestNodes = 20; // Could push up to 23 without going over 1280 bytes, but dunno if its worth

constexpr bool sDebugIterativeFindValue = false;
constexpr bool sDebugIterativeClosestNodes = false;
constexpr bool sDebugPut = false;

inflight_t::inflight_t(net::io_service &pService, const u256 &pDest, const u256 &pID, Callback pCallback)
    : timeout(pService, SOPHIA_TIMEOUT), id(pID), dest(pDest), callback(std::move(pCallback)) {}

void Node::IterativeClosestNodesFunctor::operator()(ErrorCode pError, const Contact &pSource,
                                                    const std::vector<Contact> &pResults) {
  context->markSearched(pSource);

  if (!pResults.empty()) {
    size_t lClosest = 0;

    for (size_t i = 0; i < pResults.size(); i++) {
      auto lCloser = closerDist256(context->key.data(), pResults[lClosest].id.data(), pResults[i].id.data());
      if (lCloser > 0 && !context->isKnown(pResults[i]))
        lClosest = i;
    }

    if constexpr (sDebugIterativeClosestNodes) {
      std::cout << "Status for iterative on: " << node->self() << " done " << context->searched.size() << " steps in "
                << dur_t{Clock::now() - context->start} << std::endl;
      for (auto &lInflight : context->inflight)
        std::cout << "\tI: " << lInflight << std::endl;
      for (auto &lSearched : context->searched)
        std::cout << "\tS: " << lSearched << std::endl;
      for (auto &lResponse : pResults)
        std::cout << "\tR: " << lResponse << std::endl;
      std::cout << "Best " << pResults[lClosest] << ", known? " << context->isKnown(pResults[lClosest]) << std::endl;
    }

    if (!context->isKnown(pResults[lClosest])) {
      if constexpr (sDebugIterativeClosestNodes)
        std::cout << "Looking for a better node than inflight.." << std::endl;
      bool found = false;
      for (auto it = context->inflight.begin(); it != context->inflight.end(); it++) {
        auto lCloser = closerDist256(context->key.data(), pResults[lClosest].id.data(), it->id.data());
        if (lCloser < 0) {
          if constexpr (sDebugIterativeClosestNodes)
            std::cout << "Got a closest nodes than inflight: " << pResults[lClosest] << std::endl;

          context->inflight.push_back(pResults[lClosest]);
          node->sendClosestNodes(pResults[lClosest], context->key, *this);
          found = true;
          break;
        }
      }
      if (!found) {
        if constexpr (sDebugIterativeClosestNodes)
          std::cout << "Looking for a better node than history.." << std::endl;
        for (const auto &lSearched : context->searched) {
          auto lCloser = closerDist256(context->key.data(), pResults[lClosest].id.data(), lSearched.id.data());
          if (lCloser < 0) {
            if constexpr (sDebugIterativeClosestNodes)
              std::cout << "Got a closest nodes than searched: " << pResults[lClosest] << std::endl;

            context->inflight.push_back(pResults[lClosest]);
            node->sendClosestNodes(pResults[lClosest], context->key, *this);
            break;
          }
        }
      }
    }
  }

  if (context->inflight.empty()) {
    if constexpr (sDebugIterativeClosestNodes)
      std::cout << node->self() << " Got all responses after " << context->searched.size() << " steps in "
                << dur_t{Clock::now() - context->start} << std::endl;

    auto lResults = context->searched;
    for (const auto &lContact : pResults) {
      if (std::find(lResults.begin(), lResults.end(), lContact) == lResults.end()) {
        lResults.push_back(lContact);
      }
    }

    std::sort(lResults.begin(), lResults.end(), [this](const Contact &pLeft, const Contact &pRight) {
      return closerDist256(context->key.data(), pLeft.id.data(), pRight.id.data()) < 0;
    });
    lResults.resize(std::min((size_t)SOPHIA_K, lResults.size()));

    context->callback(lResults);
    delete context;
  }
}

void Node::IterativeFindValueFunctor::operator()(ErrorCode pError, const Contact &pSource,
                                                 const std::vector<Contact> &pResults,
                                                 const std::optional<Value> &pVal) {
  context->markSearched(pSource);

  if constexpr (sDebugIterativeFindValue) {
    std::cout << node->self() << " iterative find node step " << context->searched.size() << " for "
              << dur_t{Clock::now() - context->start} << " from " << pSource << std::endl;
    for (auto &lInflight : context->inflight)
      std::cout << "\tI: " << lInflight << std::endl;
    for (auto &lSearched : context->searched)
      std::cout << "\tS: " << lSearched << std::endl;
    for (auto &lResponse : pResults)
      std::cout << "\tR: " << lResponse << std::endl;
  }

  if (pVal && !context->found) {
    /*std::cout << node->self() << " Got the value, responses after " << context->searched.size() << " steps in "
              << dur_t{Clock::now() - context->start} << std::endl;*/

    context->callback(pVal);
    context->found = true;
  } else if (!pResults.empty()) {
    size_t lClosest = 0;

    for (size_t i = 0; i < pResults.size(); i++) {
      auto lCloser = closerDist256(context->key.data(), pResults[lClosest].id.data(), pResults[i].id.data());
      if (lCloser > 0 && !context->isKnown(pResults[i]))
        lClosest = i;
    }

    if constexpr (sDebugIterativeFindValue) {
      std::cout << "Best " << pResults[lClosest] << ", known? " << context->isKnown(pResults[lClosest]) << std::endl;
    }

    if (!context->isKnown(pResults[lClosest])) {
      bool found = false;
      for (auto it = context->inflight.begin(); it != context->inflight.end(); it++) {
        auto lCloser = closerDist256(context->key.data(), pResults[lClosest].id.data(), it->id.data());
        if (lCloser < 0) {
          // std::cout << "Got a closest nodes than inflight: " << pResults[lClosest] << std::endl;

          context->inflight.push_back(pResults[lClosest]);
          node->sendFindValue(pResults[lClosest], context->key, *this);
          found = true;
          break;
        }
      }
      if (!found) {
        for (const auto &lSearched : context->searched) {
          auto lCloser = closerDist256(context->key.data(), pResults[lClosest].id.data(), lSearched.id.data());
          if (lCloser < 0) {
            // std::cout << "Got a closest nodes than searched: " << pResults[lClosest] << std::endl;

            context->inflight.push_back(pResults[lClosest]);
            node->sendFindValue(pResults[lClosest], context->key, *this);
            break;
          }
        }
      }
    }
  }

  if (context->inflight.empty()) {
    /*std::cout << node->self() << " Got all responses after " << context->searched.size() << " steps in "
              << dur_t{Clock::now() - context->start} << std::endl;*/
    if (!context->found)
      context->callback({});
    delete context;
  }
}

size_t Node::hashInflight(const u256 &pDest, uint32_t pToken) {
  std::size_t seed = 0;
  boost::hash_combine(seed, boost::hash_value(pToken));
  boost::hash_combine(seed, boost::hash_value(pDest));
  return seed;
}

void Node::send(const Contact &pDest, const cbuff_view_t &pBuff) {
  size_t lMessSize = sHeaderSize + pBuff.size;
  auto lBuff = new u8[lMessSize];
  u8 *lNonce = lBuff + sNonceOffset;
  u8 *lMac = lBuff + sMacOffset;
  u8 *lCrypt = lBuff + sHeaderSize;
  memcpy(lBuff, keypair.publicKey.data(), keypair.publicKey.size());
  randombytes_buf(lNonce, sNonceSize);
  if (crypto_box_detached(lCrypt, lMac, pBuff.data, pBuff.size, lNonce, pDest.id.data(),
                          keypair.privateKey.contained.data()) == 0) {
    /*std::cout << "[NET] " << self() << " sending " << pMessLen << " bytes to: " << pDest << std::endl;
    std::cout << "\tdata: " << buff_view_t{pBuffer, pMessLen} << std::endl;*/

    sock.async_send_to(boost::asio::const_buffer(lBuff, lMessSize), pDest.endpoint(),
                       [lMessSize, lBuff](const ErrorCode &pError, size_t pBytesWritten) {
                         if (pError != nullptr) {
                           std::cout << "[NET] Got error on send: " << pError.message() << " (" << pError.value() << ')'
                                     << std::endl;
                         } else if (pBytesWritten != lMessSize) {
                           std::cout << "[NET] Got error on send: sent less bytes than expected" << std::endl;
                         }
                         delete[] lBuff;
                       });
  } else {
    std::cout << "[NET] Error while boxing message, skipping." << std::endl;
    delete[] lBuff;
  }
}

void Node::recieve() {
  sock.async_receive_from(
      boost::asio::buffer(recvBuff.data(), sBuffSize), lastDist, [this](ErrorCode pError, size_t pBytesRecv) {
        if (pError != nullptr) {
          std::cout << "[NET] Got error on recv: " << pError.message() << std::endl;
        } else {
          Contact lSource;
          lSource.address = lastDist.address();
          lSource.port = lastDist.port();
          memcpy(lSource.id.data(), recvBuff.data(), lSource.id.size());
          size_t lMessLen = pBytesRecv - sHeaderSize;
          buff_t<sBuffSize - sHeaderSize> lDecrypted;
          u8 *lNonce = recvBuff.data() + sNonceOffset;
          u8 *lMac = recvBuff.data() + sMacOffset;
          u8 *lCrypt = recvBuff.data() + sHeaderSize;

          if (crypto_box_open_detached(lDecrypted.data(), lCrypt, lMac, lMessLen, lNonce, lSource.id.data(),
                                       keypair.privateKey.contained.data()) == 0) {
            cbuff_view_t lBuff{lDecrypted.data(), lMessLen};
            /*std::cout << "[NET] " << self() << " received " << lMessLen << " bytes from: " << lSource << std::endl;
            std::cout << "\tdata: " << buff_view_t{lBuffer, lMessLen} << std::endl;*/
            auto lCommand = (MessageType)lBuff.data[0];
            switch (lCommand) {
            case ePing:
              replyPing(lSource, lBuff);
              break;
            case eClosestNodes:
              replyClosestNodes(lSource, lBuff);
              break;
            case eFindValue:
              replyFindValue(lSource, lBuff);
              break;
            case eStoreValue:
              replyStore(lSource, lBuff);
              break;
            case eClosestValues:
              break;

            case eNodesResult:
            case ePong:
            case eError:
            case eValuesResult:
            case eValueResult:
              recvResponse(lSource, lBuff);
              break;

            case ePubsubJoin:
              break;
            case ePubsubLeave:
              break;
            case ePubsubEvent:
              break;
            default:
              std::cout << "[NET] unexpected command identifier" << std::endl;
            }
          } else {
            std::cout << "[NET] Error while opening message, ignoring." << std::endl;
          }
        }
        recieve();
      });
}

size_t Node::prepareCommand(const Contact &pDest, const u256 &pID, MessageType pCommandType, u8 *pBuffer,
                            const inflight_t::Callback &pCallback) {
  uint32_t lToken;
  size_t lID;
  do {
    lToken = randombytes_random() & 0xFFFFFFu;
    lID = hashInflight(pDest.id, lToken);
  } while (inflights.find(lID) != inflights.end());

  inflight_t lInflight{service, pID, pDest.id, pCallback};

  lInflight.timeout.async_wait([this, lID, lToken](const ErrorCode &pError) {
    auto lRef = inflights.find(lID);
    if (pError != boost::asio::error::operation_aborted && lRef != inflights.end()) {
      auto lEntry = routing.findEntry(lRef->second.dest);
      if (lEntry != nullptr) {
        lEntry->sentCommands++;
        lRef->second.callback(pError, lEntry->contact, lRef->second.id, {nullptr, 0});
      }
      // std::cout << "[NET] Inflight timeout " << self() << ", to " << lRef->second.dest << ", token: " << std::hex <<
      // lToken << std::dec << std::endl;
      inflights.erase(lRef);
    }
  });

  pBuffer[0] = (u8)pCommandType;
  memcpy(pBuffer + 1, &lToken, 3);

  inflights.emplace(lID, std::move(lInflight));

  // std::cout << "[NET] Sending inflight " << self() << ", to " << pDest << ", token: " << readToken(pBuffer) <<
  // std::endl;

  return lID;
}

void Node::prepareReply(const Contact &pSource, MessageType pReplyType, const u8 *pCommandBuffer, u8 *pReplyBuffer) {
  pReplyBuffer[0] = pReplyType;
  memcpy(pReplyBuffer + 1, pCommandBuffer + 1, 3);
  auto lEntry = routing.mayAddNewContact(pSource);
  if (lEntry != nullptr)
    lEntry->lastContact = Clock::now();
}

void Node::recvResponse(const Contact &pSource, const cbuff_view_t &pBuff) {
  uint32_t lToken = 0;
  memcpy(&lToken, pBuff.data + 1, 3);

  auto lInflight = inflights.find(hashInflight(pSource.id, lToken));
  if (lInflight == inflights.end()) {
    std::cout << "[NET] Non inflight " << self() << ", from " << pSource << ", token: " << readToken(pBuff.data)
              << std::endl;
    // TODO Blacklist?
    return;
  }

  auto lRTT = SOPHIA_TIMEOUT - lInflight->second.timeout.expires_from_now();
  lInflight->second.timeout.cancel();
  routing.addRTT(pSource, lRTT.total_microseconds() / 1000000.f);

  lInflight->second.callback(sNoError, pSource, lInflight->second.id, pBuff);

  inflights.erase(lInflight);
}

cbuff_view_t Node::readToken(const u8 *pData) { return {pData + 1, 3}; }

size_t Node::sendPing(const Contact &pDest, const Node::PingCallback &pCallback) {
  u8 lCommand[4 + 1156];

  std::array<u8, 1156> lRand;
  randombytes_buf(lRand.data(), 1156);

  auto lCallback = [pCallback, lRand](ErrorCode pError, const Contact &pSource, const u256 &pID,
                                      const cbuff_view_t &pBuff) {
    if (pBuff.data[0] != ePong)
      return;

    if (pError != nullptr) {
      if (pCallback)
        pCallback(pError, pSource);
      return;
    }

    if (pBuff.size != 1156 + 4) {
      // TODO Reply with an error
      return;
    }
    if (memcmp(pBuff.data + 4, lRand.data(), 1156) != 0) {
      // TODO Reply with an error
      return;
    }

    if (pCallback)
      pCallback(sNoError, pSource);
  };

  size_t lToken = prepareCommand(pDest, pDest.id, ePing, lCommand, lCallback);
  memcpy(lCommand + 4, lRand.data(), 1156);

  send(pDest, {lCommand, sizeof(lCommand)});

  return lToken;
}

void Node::replyPing(const Contact &pSource, const cbuff_view_t &pBuff) {
  u8 lReply[4 + 1156];

  prepareReply(pSource, ePong, pBuff.data, lReply);

  if (pBuff.size != sizeof(lReply)) {
    // TODO Reply with an error
    return;
  }

  memcpy(lReply + 4, pBuff.data + 4, 1156);
  send(pSource, {lReply, sizeof(lReply)});
}

size_t Node::sendClosestNodes(const Contact &pDest, const u256 &pKey, const Node::ClosestNodesCallback &pCallback) {
  u8 lCommand[4 + 32];

  auto lCallback = [this, pCallback](ErrorCode pError, const Contact &pSource, const u256 &pID,
                                     const cbuff_view_t &pBuff) {
    if (pBuff.data[0] != eNodesResult)
      return;
    // TODO Blacklist?

    if (pError != nullptr) {
      if (pCallback)
        pCallback(pError, pSource, {});
      return;
    }

    u8 lCount = pBuff.data[4];
    if (lCount > sMaxClosestNodes) {
      std::cout << "[NET] Incorrect count in nodes result" << std::endl;
    }
    std::vector<Contact> lResult(lCount);
    size_t lContactsBuffSize = sEntrySize * lCount;
    if (lContactsBuffSize != pBuff.size - 5) {
      // TODO pCallback(malformed error, pSource, {});
      return;
    }
    cbuff_view_t lContactsBuff{pBuff.data + 5, lContactsBuffSize};
    for (u8 i = 0; i < lCount; i++) {
      lResult[i].read(lContactsBuff);
      routing.mayAddNewContact(lResult[i]);
      /*std::cout << "\t\t" << buff_view_t{lContact.publicKey.data(), lContact.publicKey.size()}
                << ", " << buff_view_t{lAddrBytes.data(), lAddrBytes.size()}
                << ", " << lContact.port << std::endl;*/
    }
    /*std::cout << "[DHT] " << self() << " recvNodesResult token " << readToken(pBuffer) << ", with " << (int)lCount
              << " nodes from " << pSource << std::endl;*/
    if (pCallback)
      pCallback(sNoError, pSource, lResult);
  };

  size_t lToken = prepareCommand(pDest, pKey, eClosestNodes, lCommand, lCallback);
  memcpy(lCommand + 4, pKey.data(), 32);

  /*std::cout << "[DHT] " << self() << " sendClosestNodes with token " << readToken(lCommand) << " with ID "
            << pKey.view() << " to " << pDest << std::endl;*/

  send(pDest, {lCommand, sizeof(lCommand)});
  return lToken;
}

void Node::replyClosestNodes(const Contact &pSource, const cbuff_view_t &pBuff) {
  u256 lID;
  memcpy(lID.data(), pBuff.data + 4, lID.size());
  std::array<Contact, sMaxClosestNodes> lContacts;
  u8 lCount = routing.closestNodes(lID, lContacts.data(), lContacts.size(), &pSource.id);
  size_t lContactsBuffSize = sEntrySize * lCount;
  u8 lReply[4 + 1 + lContactsBuffSize];
  prepareReply(pSource, eNodesResult, pBuff.data, lReply);

  lReply[4] = lCount;
  buff_view_t lContactsBuff{lReply + 5, lContactsBuffSize};

  for (size_t i = 0; i < lCount; i++) {
    lContacts[i].write(lContactsBuff);
    /*std::cout << "\t\t" << buff_view_t{lContact.publicKey.data(), lContact.publicKey.size()}
              << ", " << buff_view_t{lAddrBytes.data(), lAddrBytes.size()}
              << ", " << lContact.port << std::endl;*/
  }
  /*std::cout << "[DHT] " << self() << " replyClosestNodes token " << readToken(pData) << ", with "
            << (int)lCount << " nodes to " << pSource << std::endl;*/
  send(pSource, {lReply, sizeof(lReply)});
}

void Node::iterativeClosestNodes(const u256 &pKey, const Node::IterativeClosestNodesCallback &pCallback) {
  auto lContext = new IterativeContext<Node::IterativeClosestNodesCallback>(pKey, pCallback);

  size_t lCount = routing.closestNodes(pKey, lContext->inflight.data(), SOPHIA_ALPHA, nullptr);
  lContext->inflight.resize(lCount);

  for (const auto &lContact : lContext->inflight) {
    if constexpr (sDebugIterativeClosestNodes)
      std::cout << self() << " iterativeClosestNodes to " << lContact << std::endl;
    sendClosestNodes(lContact, pKey, IterativeClosestNodesFunctor{lContext, this});
  }
}

size_t Node::sendFindValue(const Contact &pDest, const u256 &pKey, const Node::FindValueCallback &pCallback) {
  u8 lCommand[4 + 32];

  auto lCallback = [this, pCallback](ErrorCode pError, const Contact &pSource, const u256 &pID,
                                     const cbuff_view_t &pBuff) {
    if (pError != nullptr) {
      if (pCallback)
        pCallback(pError, pSource, {}, {});
      return;
    }

    if (pBuff.data[0] == eNodesResult) {
      u8 lCount = pBuff.data[4];
      if (lCount > sMaxClosestNodes) {
        std::cout << "[NET] Incorrect count in nodes result" << std::endl;
      }
      std::vector<Contact> lResult(lCount);
      size_t lContactsBuffSize = sEntrySize * lCount;
      if (lContactsBuffSize != pBuff.size - 5) {
        // TODO pCallback(malformed error, pSource, {});
        return;
      }
      cbuff_view_t lContactsBuff{pBuff.data + 5, lContactsBuffSize};
      for (u8 i = 0; i < lCount; i++) {
        lResult[i].read(lContactsBuff);
        routing.mayAddNewContact(lResult[i]);
      }
      if (pCallback)
        pCallback(sNoError, pSource, lResult, {});
    } else if (pBuff.data[0] == eValueResult) {
      cbuff_view_t lValueBuff = {pBuff.data + 4, pBuff.size - 4};
      Value lResult;
      lResult.read(lValueBuff);
      if (pCallback)
        pCallback(sNoError, pSource, {}, lResult);
    }
  };

  size_t lToken = prepareCommand(pDest, pKey, eFindValue, lCommand, lCallback);
  memcpy(lCommand + 4, pKey.data(), 32);

  send(pDest, {lCommand, sizeof(lCommand)});

  return lToken;
}

void Node::replyFindValue(const Contact &pSource, const cbuff_view_t &pBuff) {
  auto lValue = db.loadValue(pBuff.data + 4);

  if (!lValue) {
    replyClosestNodes(pSource, pBuff);
  } else {
    size_t lValueSize = lValue->serializedSize();
    u8 lReply[4 + lValueSize];
    prepareReply(pSource, eValueResult, pBuff.data, lReply);
    buff_view_t lValueBuff{lReply + 4, lValueSize};
    lValue->write(lValueBuff);
    send(pSource, {lReply, sizeof(lReply)});
  }
}

void Node::iterativeFindValue(const u256 &pKey, const Node::IterativeFindValueCallback &pCallback) {
  auto lContext = new IterativeContext<Node::IterativeFindValueCallback>(pKey, pCallback);

  size_t lCount = routing.closestNodes(pKey, lContext->inflight.data(), SOPHIA_ALPHA, nullptr);
  lContext->inflight.resize(lCount);

  for (const auto &lContact : lContext->inflight) {
    if constexpr (sDebugIterativeFindValue)
      std::cout << self() << " iterativeFindValue to " << lContact << std::endl;
    sendFindValue(lContact, pKey, IterativeFindValueFunctor{lContext, this});
  }
}

size_t Node::sendStore(const Contact &pDest, const Value &pValue, const Node::StoreCallback &pCallback) {
  auto lCallback = [pCallback](ErrorCode pError, const Contact &pSource, const u256 &pID, const cbuff_view_t &pBuff) {
    if (pError != nullptr) {
      if (pCallback)
        pCallback(pError, pSource);
      return;
    }

    if (pCallback)
      pCallback(sNoError, pSource);
  };

  size_t lSize = 4 + pValue.serializedSize();
  u8 lCommand[lSize];

  size_t lToken = prepareCommand(pDest, pDest.id, eStoreValue, lCommand, lCallback);
  buff_view_t lBuff{lCommand + 4, lSize - 4};
  pValue.write(lBuff, true);

  send(pDest, {lCommand, sizeof(lCommand)});

  return lToken;
}

void Node::replyStore(const Contact &pSource, const cbuff_view_t &pBuff) {
  u8 lReply[4 + 4];

  prepareReply(pSource, eError, pBuff.data, lReply);
  uint32_t lErrorCode = eNoError;

  cbuff_view_t lReadBuff = pBuff;
  lReadBuff.seek(4);
  Value lValue;
  lValue.read(lReadBuff);
  db.mayStoreValue(lValue);
  // FIXME Handle some errors and set lErrorCode

  uint32_t lNErrorCode = htonl(lErrorCode);
  memcpy(lReply + 4, &lNErrorCode, sizeof(lNErrorCode));

  send(pSource, {lReply, sizeof(lReply)});
}

Node::Node(net::io_service &pService, const char *pDBPath, const passphrase_t &pPassphrase,
           const udp::endpoint &pEndpoint)
    : service(pService), db(pDBPath), sock(pService, pEndpoint) {
  keypair = db.loadProfile(pPassphrase);
  routing.myID = keypair.publicKey;
  recieve();
}

Contact Node::self() const {
  return {keypair.publicKey, sock.local_endpoint().address(), sock.local_endpoint().port()};
}

void Node::bootstrap(const Contact &pContact) {
  routing.mayAddNewContact(pContact);

  sendClosestNodes(pContact, keypair.publicKey,
                   [this](ErrorCode pError, const Contact &pSource, std::vector<Contact> pResult) {
                     if (!pError)
                       iterativeClosestNodes(keypair.publicKey, [this](std::vector<Contact> pResult) {
                         // routing.debug();
                         refreshBuckets();
                       });
                   });
}

void Node::put(const Value &pValue, const PutCallback &pCallback) {
  db.mayStoreValue(pValue);
  this->iterativeClosestNodes(pValue.id, [this, pValue, pCallback](const std::vector<Contact> pContacts) {
    assert(!pContacts.empty());
    auto lCount = std::make_shared<size_t>();
    *lCount = pContacts.size();
    if constexpr (sDebugPut)
      std::cout << self() << " storing " << pValue.id.view() << " on " << pContacts.size() << " nodes" << std::endl;
    for (const auto &lContact : pContacts) {
      sendStore(lContact, pValue, [this, pCallback, lCount](const ErrorCode pError, const Contact &pContact) {
        (*lCount)--;
        if constexpr (sDebugPut)
          std::cout << self() << " stored at " << pContact << " still got " << *lCount << std::endl;
        if (*lCount == 0) {
          if (pCallback)
            pCallback();
        }
      });
    }
  });
}

void Node::get(const u256 &pKey, const GetCallback &pCallback) { iterativeFindValue(pKey, pCallback); }

void Node::refreshBuckets() {
  // debugRouteTable();
  for (size_t i = 0; i < routing.buckets.size(); i++) {
    auto lRandID = randSuffix(keypair.publicKey.data(), i);
    assert(clzDist256(lRandID.data(), keypair.publicKey.data()) >= i);
    iterativeClosestNodes(lRandID, [this, i](std::vector<Contact> pResult) {
      // if (i == routing.buckets.size() - 1) debugRouteTable();
    });
  }
}
