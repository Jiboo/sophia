#pragma once

#include "utils.hpp"

namespace sophia {

struct Contact {
  u256 id;
  net::ip::address address;
  u16 port;

  bool operator==(const Contact &pOther) const;

  net::ip::udp::endpoint endpoint() const;

  void read(cbuff_view_t &pSource);
  void write(buff_view_t &pDest) const;
};

std::ostream &operator<<(std::ostream &pOS, const Contact &pContact);

} // namespace sophia
