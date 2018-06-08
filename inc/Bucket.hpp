#pragma once

#include "Contact.hpp"
#include "utils.hpp"

namespace sophia {

struct Entry {
  Contact contact;
  uint64_t sentCommands = 0;
  uint64_t recvReplies = 0;
  float totalRTT = 0;
  Clock::time_point lastContact;

  Entry();
  explicit Entry(Contact pContact);
  Entry(const Entry &pCopy) = default;

  bool operator<(const Entry &pOther) const;

  float packetLoss() const;
  float roundTrip() const;
  float score() const;
};

struct Bucket {
  using container = std::vector<Entry>;
  using iterator = container::iterator;
  using citerator = container::const_iterator;

  container contacts;

  size_t size() const;
  bool empty() const;
  bool full() const;

  iterator begin();
  iterator end();
  citerator begin() const;
  citerator end() const;
  Entry &last();

  iterator find(const u256 &pNodeID);

  void sort();
  iterator erase(iterator it);
  void pop();
  void push(const Contact &pContact);
  void push(const Entry &pEntry);
};

} // namespace sophia
