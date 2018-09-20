#include <gtest/gtest.h>

#include "Network.hpp"

using namespace sophia;

class KademliaTest : public SophiaNetworkTest {
};

TEST_F(KademliaTest, Bootstrap) {
  for (auto &node : nodes) {
    EXPECT_GT(node->routingTable().countNodes(), 1);
  }
}

TEST_F(KademliaTest, Ping) {
  size_t lExpectedPongCount = 500;
  size_t lPongCount = 0;
  for (size_t i = 0; i < lExpectedPongCount; i++) {
    size_t lSourceID = randombytes_uniform(nodes.size());
    size_t lDestID;
    do {
      lDestID = randombytes_uniform(nodes.size());
    } while (lDestID == lSourceID);
    nodes[lSourceID]->ping(nodes[lDestID]->self(), [&lPongCount]() { lPongCount++; });
    process();
  }
  ASSERT_EQ(lExpectedPongCount, lPongCount);
}

TEST_F(KademliaTest, ClosestNodes) {
  for (size_t i = 1; i < nodes.size(); i++) {
    u256 lRef = nodes[randombytes_uniform(nodes.size())]->self().id;
    nodes[i]->closestNodes(lRef, [lRef](const std::vector<Contact> &pResult) { ASSERT_EQ(lRef, pResult[0].id); });
    process();
  }
}

TEST_F(KademliaTest, PutGet) {
  size_t lExpectedStoreCount = 100;
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
        if (pValue)
          lGetCount++;
      });
    });
    process();
  }
  ASSERT_EQ(lExpectedStoreCount, lGetCount);
}
