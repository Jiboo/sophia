#include "Contact.hpp"

using namespace sophia;

bool Contact::operator==(const Contact &pOther) const {
  return id == pOther.id && address == pOther.address && port == pOther.port;
}

net::ip::udp::endpoint Contact::endpoint() const { return {address, port}; }

void Contact::read(cbuff_view_t &pSource) {
  readBuff(id.view(), pSource);
  net::ip::address_v6::bytes_type lAddrBytes;
  readBuff({lAddrBytes.data(), lAddrBytes.size()}, pSource);
  address = net::ip::make_address_v6(lAddrBytes);
  uint16_t lPort;
  readBuff({(u8 *)&lPort, sizeof(lPort)}, pSource);
  port = ntohs(lPort);
}

void Contact::write(buff_view_t &pDest) const {
  writeBuff(pDest, id.view());
  auto lAddrBytes = address.to_v6().to_bytes();
  writeBuff(pDest, {lAddrBytes.data(), lAddrBytes.size()});
  uint16_t lPort = htons(port);
  writeBuff(pDest, {(u8 *)&lPort, sizeof(lPort)});
}

std::ostream &sophia::operator<<(std::ostream &pOS, const Contact &pContact) {
  return pOS << '{' << pContact.id << ", " << pContact.address.to_string() << ", " << pContact.port << '}';
}
