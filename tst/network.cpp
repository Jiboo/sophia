#include "Node.hpp"
using namespace sophia;

int main(int argc, char **argv) {
  passphrase_t lPassphrase("test");

  net::io_service lService;
  size_t lNodesCount = 400; // std::stoul(argv[1]);

  std::unique_ptr<Node> lNodes[lNodesCount];

  auto lInitStart = Clock::now();
  for (size_t i = 0; i < lNodesCount; i++) {
    std::string lDbName = "file:memdb" + std::to_string(i) + "?mode=memory";

    lNodes[i] = std::make_unique<Node>(lService, lDbName.c_str(), lPassphrase,
                                       net::ip::udp::endpoint(boost::asio::ip::address::from_string("::1"), 0));

    std::cout << "[TST] Node " << i << ": " << lNodes[i]->self() << std::endl;

    if (i > 0)
      lNodes[i]->bootstrap(lNodes[0]->self(), []() {});

    lService.run_for(std::chrono::milliseconds(250));
  }

  std::cout << "[TST] Initialized " << lNodesCount << " nodes in " << dur_t{Clock::now() - lInitStart} << std::endl;

  lService.run_for(std::chrono::seconds(1));

  /*for (size_t i = 1; i < lNodesCount; i++) {
    lNodes[i]->bootstrap(lNodes[0]->self());
    lService.run_for(std::chrono::milliseconds(5));
  }*/

  Value lTemp;
  for (size_t lValIndex = 0; lValIndex < lNodesCount * 72; lValIndex++) {
    randValue(lTemp);

    // std::cout << "[TST] Generated value " << lTemp.id.view() << std::endl;

    Node *lWriter = lNodes[randombytes_uniform(lNodesCount)].get();
    auto lPutStart = Clock::now();
    lWriter->put(lTemp, [&lNodes, lNodesCount, lTemp, lPutStart] {
      // std::cout << "[TST] Put " << lTemp.id.view() << " in " << dur_t{Clock::now() - lPutStart} << std::endl;

      Node *lReader = lNodes[randombytes_uniform(lNodesCount)].get();
      auto lGetStart = Clock::now();
      lReader->get(lTemp.id, [lGetStart, lReader, lTemp](const std::optional<Value> &pValue) {
        if (pValue) {
          // std::cout << "[TST] Found " << lTemp.id.view() << " in " << dur_t{Clock::now() - lGetStart} << std::endl;
        } else {
          // lReader->debugRouteTable();
          std::cout << "[TST] Failed " << lTemp.id.view() << " in " << dur_t{Clock::now() - lGetStart} << std::endl;
        }
      });
    });
    lService.run_for(std::chrono::milliseconds(250));
  }

  /*u256 lTopicID = lNodes[0]->createTopic([](const Event &pEvent){

  });
  for (size_t lNodeIndex = 1; lNodeIndex < lNodesCount; lNodeIndex++) {
    lNodes[lNodeIndex]->subscribe(lTopicID, [](const Event &pEvent){

    });
  }*/
  // TODO Send events and check the percentage of nodes that got it, spread time, etc..

  std::cerr << "[TST] Done tests" << std::endl;

  lService.run_for(std::chrono::seconds(1));

  return EXIT_SUCCESS;
}
