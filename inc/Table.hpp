#pragma once

#include "Bucket.hpp"
#include "utils.hpp"

namespace sophia {

struct Table {
  u256 myID;
  std::vector<Bucket> buckets;

  Table();

  int firstNonEmptyBucket() const;
  size_t index(const u256 &pOtherID) const;
  Entry *findEntry(const u256 &pNodeID);

  void splitLast();

  Entry *mayAddNewContact(const Contact &pContact);
  u8 closestNodes(const u256 &pTarget, Contact *pArray, u8 pMax, const u256 *pIgnore);
  const Entry *randomNodeInKBucket(u8 pBucket) const;

  void addRTT(const Contact &pContact, float pTime);

#ifdef SOPHIA_EXTRA_API
  inline void debug() const {
    std::cout << "Routing table for " << myID << std::endl;
    for (size_t i = 0; i < buckets.size(); i++) {
      for (const auto &lEntry : buckets[i])
        std::cout << "\t" << i << ": " << lEntry.contact << ", " << dist256(myID.data(), lEntry.contact.id.data())
                  << ", " << clzDist256(myID.data(), lEntry.contact.id.data()) << std::endl;
    }
  }
  inline size_t countNodes() const {
    size_t lResult = 0;
    for (const auto &bucket : buckets) {
      lResult += bucket.size();
    }
    return lResult;
  }
#endif
};

} // namespace sophia
