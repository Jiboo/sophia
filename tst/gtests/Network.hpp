#pragma once

#include "gtest/gtest.h"

#include "Node.hpp"

class SophiaNetworkTest : public ::testing::Test {
private:
  sophia::net::io_service service;

protected:
  std::vector<std::unique_ptr<sophia::Node>> nodes;
  void SetUp() override;

  void process();
  sophia::Node *findNode(const sophia::u256 &pID);

public:
  ~SophiaNetworkTest() override = default;
};
