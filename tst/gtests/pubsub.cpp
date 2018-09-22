#include <gtest/gtest.h>

#include "Network.hpp"

using namespace sophia;

class PubsubTest : public SophiaNetworkTest {};

TEST_F(PubsubTest, Join) {
  u256 lTopicID = nodes[0]->createTopic([](const Event &pEvent) {}, [] {});
  process();

  const size_t lSubs = SOPHIA_K;
  for (size_t i = 1; i <= lSubs; i++) {
    // std::cout << "node " << i << ", " << nodes[i]->self() << " trying to join..." << std::endl;
    nodes[i]->subscribe(lTopicID, [](const Event &pEvent) {},
                        [this, i, lTopicID]() {
                          // std::cout << "node " << i << ", " << nodes[i]->self() << " joined" << std::endl;
                          // nodes[i]->topicRoutingTable(lTopicID).debug();
                          EXPECT_EQ(i, nodes[i]->topicRoutingTable(lTopicID).countNodes());
                        });
    process();
  }
}

TEST_F(PubsubTest, ClosestNodes) {
  u256 lTopicID = nodes[0]->createTopic([](const Event &pEvent) {}, [] {});
  process();

  const size_t lSubs = nodes.size() - 1;
  for (size_t i = 1; i <= lSubs; i++) {
    nodes[i]->subscribe(lTopicID, [](const Event &pEvent) {}, []() {});
    process();
  }

  for (size_t i = 0; i <= lSubs; i++) {
    // std::cout << "refreshing node " << i << ", " << nodes[i]->self() << std::endl;
    nodes[i]->refreshTopic(lTopicID);
    process();
    // nodes[i]->topicRoutingTable(lTopicID).debug();
  }

  const size_t lTests = lSubs / 10;
  for (size_t i = 0; i < lTests; i++) {
    size_t lSourceIndex = randombytes_uniform(lSubs);
    size_t lRefIndex;
    do {
      lRefIndex = randombytes_uniform(lSubs);
    } while (lRefIndex == lSourceIndex);
    u256 lRef = nodes[lRefIndex]->self().id;
    nodes[lSourceIndex]->pubsubClosestNodes(
        lTopicID, lRef, [lRef](const std::vector<Contact> &pResult) { EXPECT_EQ(lRef, pResult[0].id); });
    process();
  }
}

TEST_F(PubsubTest, Broadcast) {
  size_t lHandled = 0;

  u256 lTopicID = nodes[0]->createTopic(
      [&lHandled /*, this*/](const Event &pEvent) {
        lHandled++;
        // std::cout << "node 0, " << nodes[0]->self() << " got event " << cbuff_view_t{pEvent.signature.data(), 4} <<
        // std::endl;
      },
      [/*this*/]() {
        // std::cout << "node 0, " << nodes[0]->self() << " joined " << std::endl;
        // nodes[0]->topicRoutingTable(lTopicID).debug();
      });
  process();

  const size_t lSubs = nodes.size() - 1;
  for (size_t i = 1; i <= lSubs; i++) {
    nodes[i]->subscribe(lTopicID,
                        [/*this, i,*/ &lHandled](const Event &pEvent) {
                          lHandled++;
                          // std::cout << "node " << i << ", " << nodes[i]->self() << " got event " <<
                          // cbuff_view_t{pEvent.signature.data(), 4} << std::endl;
                        },
                        [/*this, i, lTopicID*/]() {
                          // std::cout << "node " << i << ", " << nodes[i]->self() << " joined " << lTopicID <<
                          // std::endl;
                        });
    process();
  }

  for (size_t i = 0; i <= lSubs; i++) {
    // std::cout << "refreshing node " << i << ", " << nodes[i]->self() << std::endl;
    nodes[i]->refreshTopic(lTopicID);
    process();
    // nodes[i]->topicRoutingTable(lTopicID).debug();
  }

  float lTotalCover = 0;
  const size_t lTests = lSubs / 10;
  for (size_t i = 0; i < lTests; i++) {
    lHandled = 0;
    Node::sSentMsg.clear();
    size_t lSourceIndex = randombytes_uniform(lSubs);
    std::vector<uint8_t> lData;
    lData.resize(1024);
    randombytes_buf(lData.data(), lData.size());
    auto lStart = Clock::now();
    nodes[lSourceIndex]->publish(lTopicID, 0, 0, cbuff_view_t{lData.data(), lData.size()});
    process();
    float lCover = lHandled / float(lSubs);
    lTotalCover += lCover;
    auto lDelay = Node::sLastProcessedEvent - lStart;
    std::cout << (lCover * 100) << "% nodes received event in " << Node::sSentMsg[MessageType::ePubsubEvent]
              << " messages in " << sophia::dur_t{lDelay} << std::endl;
  }
  EXPECT_GE(lTotalCover / lTests, 0.9);
}
