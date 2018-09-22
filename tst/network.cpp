#include "Node.hpp"
using namespace sophia;

inline void process(net::io_service &pService) {
  while (pService.run_for(std::chrono::milliseconds(10)) > 0);
}

int main(int argc, char **argv) {
  passphrase_t lPassphrase("test");

  net::io_service lService;
  size_t lNodesCount = argc > 1 ? std::stoul(argv[1]) : 100;
  size_t lTestsCount = argc > 2 ? std::stoul(argv[2]) : 10;

  std::unique_ptr<Node> lNodes[lNodesCount];

  auto lInitStart = Clock::now();
  for (size_t i = 0; i < lNodesCount; i++) {
    std::string lDbName = "file:memdb" + std::to_string(i) + "?mode=memory";

    lNodes[i] = std::make_unique<Node>(lService, lDbName.c_str(), lPassphrase,
                                       net::ip::udp::endpoint(boost::asio::ip::address::from_string("::1"), 0));

    std::cout << "[TST] Node " << i << ": " << lNodes[i]->self() << std::endl;

    if (i > 0)
      lNodes[i]->bootstrap(lNodes[0]->self(), []() {});

    process(lService);
  }

  std::cout << "[TST] Initialized " << lNodesCount << " nodes in " << dur_t{Clock::now() - lInitStart} << std::endl;

  auto lRefreshStart = Clock::now();
  for (size_t i = 0; i < lNodesCount; i++) {
    auto lRefreshNodeStart = Clock::now();
    Node::sSentMsg.clear();
    lNodes[i]->refreshBuckets();
    process(lService);
    std::cout << "[TST] Node " << i << " refreshed in " << dur_t{Clock::now() - lRefreshNodeStart}
              << ", closest_nodes: " << Node::sSentMsg[MessageType::eClosestNodes]
              << ", nodes_result: " << Node::sSentMsg[MessageType::eNodesResult] << std::endl;
  }

  std::cout << "[TST] Refreshed buckets in " << dur_t{Clock::now() - lRefreshStart} << std::endl;

  Value lTemp;
  for (size_t lValIndex = 0; lValIndex < lTestsCount; lValIndex++) {
    randValue(lTemp);

    Node *lWriter = lNodes[randombytes_uniform(lNodesCount)].get();
    auto lPutStart = Clock::now();
    Node::sSentMsg.clear();
    lWriter->put(lTemp, [&lNodes, lNodesCount, lTemp, lPutStart] {
      std::cout << "[TST] Put in " << dur_t{Clock::now() - lPutStart}
                << ", closest_nodes: " << Node::sSentMsg[MessageType::eClosestNodes]
                << ", nodes_result: " << Node::sSentMsg[MessageType::eNodesResult]
                << ", store: " << Node::sSentMsg[MessageType::eStoreValue]
                << ", result: " << Node::sSentMsg[MessageType::eResult] << std::endl;

      Node *lReader = lNodes[randombytes_uniform(lNodesCount)].get();
      auto lGetStart = Clock::now();
      Node::sSentMsg.clear();
      lReader->get(lTemp.id, [lGetStart, lTemp](const std::optional<Value> &pValue) {
        if (pValue) {
          std::cout << "[TST] Found in ";
        } else {
          // lReader->debugRouteTable();
          std::cout << "[TST] Failed in ";
        }
        std::cout << dur_t{Clock::now() - lGetStart} << ", find_value: " << Node::sSentMsg[MessageType::eFindValue]
                  << ", nodes_result: " << Node::sSentMsg[MessageType::eNodesResult]
                  << ", value_result: " << Node::sSentMsg[MessageType::eValueResult] << std::endl;
      });
    });
    process(lService);
  }

  auto lSubscribeStart = Clock::now();
  uint64_t lReceived = 0;
  u256 lTopicID = lNodes[0]->createTopic([&lReceived](const Event &pEvent) { lReceived++; }, {});
  process(lService);

  for (size_t lNodeIndex = 1; lNodeIndex < lNodesCount; lNodeIndex++) {
    Node::sSentMsg.clear();
    auto lSubscribeNodeStart = Clock::now();
    lNodes[lNodeIndex]->subscribe(lTopicID, [&lReceived](const Event &pEvent) { lReceived++; }, {});
    process(lService);
    std::cout << "[TST] Node " << lNodeIndex << " joined topic in " << dur_t{Clock::now() - lSubscribeNodeStart}
              << ", closest_nodes: " << Node::sSentMsg[MessageType::eClosestNodes]
              << ", nodes_result: " << Node::sSentMsg[MessageType::eNodesResult]
              << ", pubsub_join: " << Node::sSentMsg[MessageType::ePubsubJoin]
              << ", pubsub_closest_nodes: " << Node::sSentMsg[MessageType::ePubsubNodesResult] << std::endl;
  }

  std::cout << "[TST] Created and joined topic in " << dur_t{Clock::now() - lSubscribeStart} << std::endl;

  auto lRefreshTopicStart = Clock::now();
  for (size_t i = 0; i < lNodesCount; i++) {
    auto lRefreshTopicNodeStart = Clock::now();
    Node::sSentMsg.clear();
    lNodes[i]->refreshTopicBuckets(lTopicID);
    process(lService);
    std::cout << "[TST] Node " << i << " refreshed topic buckets in " << dur_t{Clock::now() - lRefreshTopicNodeStart}
              << ", closest_nodes: " << Node::sSentMsg[MessageType::eClosestNodes]
              << ", nodes_result: " << Node::sSentMsg[MessageType::eNodesResult]
              << ", pubsub_join: " << Node::sSentMsg[MessageType::ePubsubJoin]
              << ", pubsub_closest_nodes: " << Node::sSentMsg[MessageType::ePubsubNodesResult] << std::endl;
  }

  std::cout << "[TST] Refreshed topic buckets in " << dur_t{Clock::now() - lRefreshTopicStart} << std::endl;

  for (size_t lValIndex = 0; lValIndex < lTestsCount; lValIndex++) {
    lReceived = 0;
    Node::sSentMsg.clear();
    size_t lSourceIndex = randombytes_uniform(lNodesCount);
    std::vector<uint8_t> lData;
    lData.resize(1024);
    randombytes_buf(lData.data(), lData.size());
    auto lPublishStart = Clock::now();
    lNodes[lSourceIndex]->publish(lTopicID, 0, 0, cbuff_view_t{lData.data(), lData.size()});
    process(lService);
    float lCover = (lReceived + 1) / float(lNodesCount);
    auto lDelay = Node::sLastProcessedEvent - lPublishStart;
    std::cout << "[TST] publish stats: spread in " << sophia::dur_t{lDelay} << ", cover: " << (lCover * 100)
              << "%, with " << Node::sSentMsg[MessageType::ePubsubEvent] / float(lNodesCount) << " messages/node"
              << std::endl;
  }

  std::cout << "[TST] Done tests" << std::endl;

  lService.run_for(std::chrono::seconds(1));

  return EXIT_SUCCESS;
}
