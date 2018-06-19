#include <gtest/gtest.h>

#define SOPHIA_EXTRA_API
#include "Node.hpp"

using namespace sophia;

class PubsubTest : public ::testing::Test {
protected:
  void SetUp() override {
    passphrase = passphrase_t("test");
    constexpr size_t lNodesCount = 100;
    for (size_t i = 0; i < lNodesCount; i++) {
      std::string lDbName = "file:memdb" + std::to_string(i) + "?mode=memory";
      nodes.emplace_back(
          std::make_unique<Node>(service, lDbName.c_str(), passphrase,
                                 net::ip::udp::endpoint(boost::asio::ip::address::from_string("::1"), 0)));
      if (i > 0)
        nodes[i]->bootstrap(nodes[0]->self(), []() {});
      service.run_for(std::chrono::milliseconds(20));
    }
  }

  net::io_service service;
  passphrase_t passphrase;
  std::vector<std::unique_ptr<Node>> nodes;
};

TEST_F(PubsubTest, Join) {
  u256 lTopicID = nodes[0]->createTopic(
      [this]() {
        // std::cout << "node 0, " << nodes[0]->self() << " joined " << std::endl;
        // nodes[0]->topicRoutingTable(lTopicID).debug();
      },
      [](const Event &pEvent) {

      });
  service.run_for(std::chrono::milliseconds(20));

  const size_t lSubs = nodes.size() / 10;
  for (size_t i = 1; i <= lSubs; i++) {
    // std::cout << "node " << i << ", " << nodes[i]->self() << " trying to join..." << std::endl;
    nodes[i]->subscribe(lTopicID,
                        [this, i, lTopicID]() {
                          // std::cout << "node " << i << ", " << nodes[i]->self() << " joined" << std::endl;
                          // nodes[i]->topicRoutingTable(lTopicID).debug();
                          EXPECT_EQ(i, nodes[i]->topicRoutingTable(lTopicID).countNodes());
                        },
                        [](const Event &pEvent) {});
    service.run_for(std::chrono::milliseconds(20));
  }
}

TEST_F(PubsubTest, ClosestNodes) {
  // TODO
}

TEST_F(PubsubTest, Broadcast) {
  // TODO
}
