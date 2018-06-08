#include <gtest/gtest.h>
#include <unordered_set>

#include "Database.hpp"

using namespace sophia;

TEST(Database, Values) {
  Database lDb("file:memdb_tst?mode=memory");
  lDb.loadProfile(passphrase_t(std::string("test")));

  Value lTemp = randValue();

  EXPECT_TRUE(lTemp.signatureValid());

  EXPECT_LT(lDb.getRevision(lTemp.id.data()), 0);
  lDb.mayStoreValue(lTemp);
  EXPECT_GE(lDb.getRevision(lTemp.id.data()), 0);

  // std::unordered_set<u256, array_hasher_t<32>> lStored;

  SOPHIA_CCALL(sqlite3_exec(lDb.getSQLite(), "BEGIN;", nullptr, nullptr, nullptr));
  for (size_t i = 0; i < 10000; i++) {
    lTemp = randValue();
    lDb.mayStoreValue(lTemp);
    // lStored.emplace(lTemp.id);
    EXPECT_GE(lDb.getRevision(lTemp.id.data()), 0);
  }
  SOPHIA_CCALL(sqlite3_exec(lDb.getSQLite(), "COMMIT;", nullptr, nullptr, nullptr));

  /*uint32_t lRandIndex = randombytes_uniform((uint32_t)lStored.size());
  auto lIterator = lStored.begin();
  while (lRandIndex--) lIterator++;

  auto lNow = Clock::now();
  auto lRetreived = lDb.loadValue(lIterator->data());
  std::cout << "Retreived random key in " << dur_t{Clock::now() - lNow} << std::endl;
  EXPECT_TRUE(lRetreived.has_value());*/
}
