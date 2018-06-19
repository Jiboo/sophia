#include <gtest/gtest.h>
#include <unordered_set>

#define SOPHIA_EXTRA_API
#include "Node.hpp"

using namespace sophia;

class KademliaTest : public ::testing::Test {
protected:
  void SetUp() override {
    passphrase = passphrase_t("test");
    constexpr size_t lNodesCount = 1000;
    for (size_t i = 0; i < lNodesCount; i++) {
      std::string lDbName = "file:memdb" + std::to_string(i) + "?mode=memory";
      nodes.emplace_back(
          std::make_unique<Node>(service, lDbName.c_str(), passphrase,
                                 net::ip::udp::endpoint(boost::asio::ip::address::from_string("::1"), 0)));
      if (i > 0)
        nodes[i]->bootstrap(nodes[0]->self(), []() {});
      service.run_for(std::chrono::milliseconds(20));
    }
    // Refresh buckets on first tenth
    for (size_t i = 0; i < lNodesCount / 10; i++) {
      nodes[i]->refreshBuckets();
      service.run_for(std::chrono::milliseconds(20));
    }
  }

  net::io_service service;
  passphrase_t passphrase;
  std::vector<std::unique_ptr<Node>> nodes;
};

TEST_F(KademliaTest, Bootstrap) {
  for (auto &node : nodes) {
    EXPECT_GT(node->routingTable().countNodes(), 1);
  }
}

TEST_F(KademliaTest, Ping) {
  size_t lExpectedPingCount = 500;
  size_t lPingCount = 0;
  for (size_t i = 0; i < lExpectedPingCount; i++) {
    size_t lSourceID = randombytes_uniform(nodes.size());
    size_t lDestID;
    do {
      lDestID = randombytes_uniform(nodes.size());
    } while (lDestID == lSourceID);
    nodes[lSourceID]->ping(nodes[lDestID]->self(), [&lPingCount]() { lPingCount++; });
    service.run_for(std::chrono::milliseconds(5));
  }
  ASSERT_EQ(lExpectedPingCount, lPingCount);
}

TEST_F(KademliaTest, ClosestNodes) {
  std::vector<u256> lNodeIDs;
  for (const auto &lNode : nodes)
    lNodeIDs.emplace_back(lNode->self().id);

  u256 lRef = lNodeIDs[0];
  std::sort(lNodeIDs.begin(), lNodeIDs.end(), [lRef](const u256 &pLeft, const u256 &pRight) {
    return closerDist256(lRef.data(), pLeft.data(), pRight.data()) < 0;
  });
  lNodeIDs.resize(SOPHIA_K);

  for (size_t i = 1; i < nodes.size(); i++) {
    nodes[i]->closestNodes(lRef, [&lNodeIDs](const std::vector<Contact> &pResult) {
      ASSERT_EQ(SOPHIA_K, pResult.size());
      size_t lFoundClosest = 0;
      for (const auto &lResultNode : pResult) {
        if (std::find(lNodeIDs.begin(), lNodeIDs.end(), lResultNode.id) != lNodeIDs.end())
          lFoundClosest++;
      }
      ASSERT_GE(lFoundClosest,
                SOPHIA_ALPHA * 2); // FIXME: Dunno what to expect here, depends on routing tables of contacted nodes?
    });
    service.run_for(std::chrono::milliseconds(20));
  }
}

TEST_F(KademliaTest, PutGet) {
  size_t lExpectedStoreCount = 500;
  size_t lGetCount = 0;
  for (size_t i = 0; i < lExpectedStoreCount; i++) {
    size_t lSourceID = randombytes_uniform(nodes.size());
    Value lValue;
    randValue(lValue);
    const auto lKey = lValue.id;
    nodes[lSourceID]->put(lValue, [this, lKey, &lGetCount]() {
      size_t lReaderID = randombytes_uniform(nodes.size());
      nodes[lReaderID]->get(lKey, [lKey, &lGetCount](const std::optional<Value> &pValue) {
        EXPECT_TRUE(pValue.has_value());
        EXPECT_EQ(lKey, pValue->id);
        lGetCount++;
      });
    });
    service.run_for(std::chrono::milliseconds(50));
  }
  ASSERT_EQ(lExpectedStoreCount, lGetCount);
}
