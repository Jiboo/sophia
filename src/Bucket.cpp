#include "Bucket.hpp"

#include <utility>

using namespace sophia;

Entry::Entry() : lastContact(Clock::time_point::min()) {}

Entry::Entry(Contact pContact) : contact(std::move(pContact)), lastContact(Clock::time_point::min()) {}

float Entry::packetLoss() const {
  if (sentCommands == 0u)
    return 1;
  return (float)recvReplies / sentCommands;
}

float Entry::roundTrip() const {
  if (recvReplies == 0u)
    return 1000;
  return totalRTT / recvReplies;
}

float Entry::score() const {
  auto timeUnseen = std::chrono::duration_cast<std::chrono::seconds>(Clock::now() - lastContact);
  return timeUnseen.count() + packetLoss() * 100 + roundTrip() * 100;
}

bool Entry::operator<(const Entry &pOther) const { return score() < pOther.score(); }

size_t Bucket::size() const { return contacts.size(); }
bool Bucket::empty() const { return contacts.empty(); }
bool Bucket::full() const { return contacts.size() >= SOPHIA_K; }

Bucket::iterator Bucket::begin() { return contacts.begin(); }
Bucket::iterator Bucket::end() { return contacts.end(); }
Bucket::citerator Bucket::begin() const { return contacts.cbegin(); }
Bucket::citerator Bucket::end() const { return contacts.cend(); }

Entry &Bucket::last() {
  assert(!empty());
  return contacts.back();
}

Bucket::iterator Bucket::find(const u256 &pNodeID) {
  for (auto it = begin(); it != end(); it++) {
    if (it->contact.id == pNodeID)
      return it;
  }
  return end();
}

void Bucket::sort() { std::sort(contacts.begin(), contacts.end()); }

Bucket::iterator Bucket::erase(iterator it) { return contacts.erase(it); }

void Bucket::pop() { contacts.pop_back(); }

void Bucket::push(const Contact &pContact) { contacts.emplace_back(pContact); }

void Bucket::push(const Entry &pEntry) { contacts.emplace_back(pEntry); }
