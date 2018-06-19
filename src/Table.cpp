#include "Table.hpp"

using namespace sophia;

Table::Table() : buckets(1) {}

size_t Table::index(const u256 &pOtherID) const { return clzDist256(myID.data(), pOtherID.data()); }

void Table::splitLast() {
  /*std::cout << "before split" << std::endl;
  debug();*/
  auto lLastBucketID = buckets.size() - 1;
  buckets.resize(buckets.size() + 1);
  auto &lLastBucket = buckets[lLastBucketID];
  auto &lNewBucket = buckets[lLastBucketID + 1];
  for (auto it = lLastBucket.begin(); it != lLastBucket.end();) {
    if (index(it->contact.id) > lLastBucketID) {
      lNewBucket.push(*it);
      it = lLastBucket.erase(it);
    } else {
      it++;
    }
  }
  /*std::cout << "after split" << std::endl;
  debug();*/
}

Entry *Table::findEntry(const u256 &pNodeID) {
  auto lBucketID = std::min(index(pNodeID), buckets.size() - 1);
  auto &lBucket = buckets[lBucketID];
  auto lIterator = lBucket.find(pNodeID);
  if (lIterator == lBucket.end())
    return nullptr;
  return &*lIterator; // FIXME UB?
}

u8 Table::closestNodes(const u256 &pTarget, Contact *pArray, u8 pMax, const u256 *pIgnore) {
  auto lNonEmpty = firstNonEmptyBucket();
  if (lNonEmpty == -1)
    return 0; // All buckets are empty

  auto lBucketID = std::max(size_t(lNonEmpty), std::min(index(pTarget), buckets.size() - 1));

  std::array<Entry, SOPHIA_K> lBuff;
  u8 lCount = 0;
  while (lCount < pMax) {
    auto &lBucket = buckets[lBucketID];

    std::partial_sort_copy(lBucket.begin(), lBucket.end(), lBuff.begin(), lBuff.end(),
                           [this, &pTarget](const Entry &pLeft, const Entry &pRight) {
                             return closerDist256(pTarget.data(), pLeft.contact.id.data(), pRight.contact.id.data()) <
                                    0;
                           });

    /*std::cout << "closestNodes for " << pTarget.view() << std::endl;
    for (const auto &lItem : lBucket) {
      std::cout << "\tB: " << lItem.contact << std::endl;
    }
    for (size_t i = 0; i < lBucket.size(); i++) {
      std::cout << "\tS: " << lBuff[i].contact << ", " << dist256(pTarget.data(), lBuff[i].contact.id.data()) << ", "
                           << clzDist256(pTarget.data(), lBuff[i].contact.id.data()) << std::endl;
    }*/

    for (size_t i = 0; i < lBucket.size() && lCount < pMax; i++) {
      auto &lEntry = lBuff[i];
      if ((pIgnore != nullptr) && (*pIgnore == lEntry.contact.id))
        continue;

      pArray[lCount++] = lEntry.contact;
    }

    if (lBucketID == 0)
      break;
    lBucketID--;
  }
  return lCount;
}

Entry *Table::mayAddNewContact(const Contact &pContact) {
  assert(pContact.id != myID);

  if (pContact.port <= 1024)
    return nullptr;

  auto lBestBucketID = index(pContact.id);
  auto lBucketID = std::min(lBestBucketID, buckets.size() - 1);
  auto lSearch = buckets[lBucketID].find(pContact.id);
  if (lSearch != buckets[lBucketID].end())
    return &*lSearch;

  while (buckets[lBucketID].full() && lBestBucketID > lBucketID) {
    // std::cout << "need split for " << pContact.publicKey.view() << ", " << lBucketID << ", " << lBestBucketID <<
    // std::endl;
    splitLast();
    lBucketID++;
  }

  auto &lBucket = buckets[lBucketID];
  if (lBucket.full()) {
    lBucket.sort();
    lBucket.pop();
  }
  lBucket.push(pContact);
  assert(lBucket.size() <= SOPHIA_K);
  return &lBucket.last();
}

void Table::addRTT(const Contact &pContact, float pTime) {
  auto lEntry = findEntry(pContact.id);
  if (lEntry != nullptr) {
    lEntry->lastContact = Clock::now();
    lEntry->recvReplies++;
    lEntry->totalRTT += pTime;
  }
}

int Table::firstNonEmptyBucket() {
  int lResult = -1;
  for (int lIndex = 0; lIndex < int(buckets.size()); lIndex++) {
    if (!buckets[lIndex].empty()) {
      lResult = lIndex;
      break;
    }
  }
  return lResult;
}
