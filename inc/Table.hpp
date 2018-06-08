#pragma once

#include "Bucket.hpp"
#include "utils.hpp"

namespace sophia {

struct Table {
  u256 myID;
  std::vector<Bucket> buckets;

  Table();

  int firstNonEmptyBucket();
  size_t index(const u256 &pOtherID) const;
  Entry *findEntry(const u256 &pNodeID);

  void splitLast();

  Entry *mayAddNewContact(const Contact &pContact);
  u8 closestNodes(const u256 &pTarget, Contact *pArray, u8 pMax, const u256 *pIgnore);

  void addRTT(const Contact &pContact, float pTime);

  void debug() const;
};

} // namespace sophia
