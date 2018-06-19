#include <gtest/gtest.h>
#include <unordered_set>

#include "Database.hpp"

using namespace sophia;

TEST(Database, Values) {
  Database lDb("file:memdb_tst?mode=memory");
  lDb.loadProfile(passphrase_t(std::string("test")));

  Value lTemp;
  randValue(lTemp);

  EXPECT_TRUE(lTemp.signatureValid());

  EXPECT_EQ(-1, lDb.getRevision(lTemp.id.data()));
  lDb.mayStoreValue(lTemp);
  EXPECT_EQ(0, lDb.getRevision(lTemp.id.data()));

  // std::unordered_set<u256, array_hasher_t<32>> lStored;

  SOPHIA_CCALL(sqlite3_exec(lDb.getSQLite(), "BEGIN;", nullptr, nullptr, nullptr));
  for (size_t i = 0; i < 10000; i++) {
    randValue(lTemp);
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

TEST(Database, PrivK) {
  Database lDb("file:memdb_tst?mode=memory");
  lDb.loadProfile(passphrase_t(std::string("test")));

  u256 lID;
  sensitive_t<u512> lPK;
  assert(lPK->size() == crypto_sign_SECRETKEYBYTES);
  static_assert(lID.size() == crypto_sign_PUBLICKEYBYTES);
  SOPHIA_CCALL(crypto_sign_keypair(lID.data(), lPK->data()))

  lDb.storePrivKey(lID, lPK);

  auto lRetreivedK = lDb.loadPrivKey(lID);
  EXPECT_TRUE(lRetreivedK.has_value());
  EXPECT_EQ(lPK->size(), (*lRetreivedK)->size());
  EXPECT_EQ(0, memcmp(lPK->data(), (*lRetreivedK)->data(), lPK->size()));
}
