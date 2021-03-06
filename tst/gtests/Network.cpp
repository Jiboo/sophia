#include "Network.hpp"

void SophiaNetworkTest::process() {
  while (service.run_for(std::chrono::milliseconds(10)) > 0);
}

void SophiaNetworkTest::SetUp() {
  auto lStart = sophia::Clock::now();
  sophia::passphrase_t passphrase("test");
  auto lPassGenerated = sophia::Clock::now();
  constexpr size_t lNodesCount = 1000;
  for (size_t i = 0; i < lNodesCount; i++) {
    std::string lDbName = "file:memdb" + std::to_string(i) + "?mode=memory";
    // std::cout << "Creating node " << i << std::endl;
    nodes.emplace_back(std::make_unique<sophia::Node>(
        service, lDbName.c_str(), passphrase,
        sophia::net::ip::udp::endpoint(sophia::net::ip::address::from_string("::1"), 0)));
    if (i > 0)
      nodes[i]->bootstrap(nodes[0]->self(), []() {});
    process();
  }
  auto lNodesCreated = sophia::Clock::now();
  // Refresh buckets on first tenth
  for (size_t i = 0; i < lNodesCount; i++) {
    // std::cout << "Refreshing node " << i << std::endl;
    nodes[i]->refreshBuckets();
    process();
  }
  auto lDone = sophia::Clock::now();
  std::cout << "Initialized in " << sophia::dur_t{lDone - lStart}
            << "ms (pass: " << sophia::dur_t{lPassGenerated - lStart}
            << "ms, nodes: " << sophia::dur_t{lNodesCreated - lPassGenerated}
            << "ms, refresh: " << sophia::dur_t{lDone - lNodesCreated} << "ms)" << std::endl;
}

sophia::Node *SophiaNetworkTest::findNode(const sophia::u256 &pID) {
  for (const auto &lNode : nodes) {
    if (lNode->self().id == pID)
      return lNode.get();
  }
  return nullptr;
}
