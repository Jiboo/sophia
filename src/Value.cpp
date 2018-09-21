#include "Value.hpp"

using namespace sophia;

void Value::read(cbuff_view_t &pSource) {
  readBuff(id.view(), pSource);
  readBuff(parent.view(), pSource);
  readBuff(signature.view(), pSource);

  readField<u8>((u8 *)&type, pSource);
  uint32_t lRevision = 0;
  readBuff({(u8 *)&lRevision, 3}, pSource);
  revision = ntohl(lRevision);

  data.resize(pSource.size);
  readBuff({data.data(), data.size()}, pSource);
}

void Value::write(buff_view_t &pDest, bool pForSign) const {
  writeBuff(pDest, id.view());
  writeBuff(pDest, parent.view());
  if (pForSign)
    writeBuff(pDest, signature.view());

  writeField(pDest, (u8)type);
  uint32_t lRevision = htonl(revision & 0xFFFFFFu);
  writeBuff(pDest, {((u8 *)&lRevision) + 1, 3});

  writeBuff(pDest, {data.data(), data.size()});
}

size_t Value::serializedSize(bool pForSign) const {
  return 4 + id.size() + parent.size() + data.size() + (pForSign ? 0 : signature.size());
}

bool Value::signatureValid() const {
  auto lSize = serializedSize(true);
  u8 lData[lSize];
  buff_view_t lBuff{lData, lSize};
  write(lBuff, false);
  assert(lBuff.size == 0);
  /*std::cout << "V: " << id << ", " << signature << std::endl;
  std::cout << "V view: " << view() << std::endl;
  std::cout << "V buff: " << buff_view_t{lData, lSize} << std::endl;*/
  return crypto_sign_verify_detached(signature.data(), lData, lSize, id.data()) == 0;
}

u512 Value::computeSignature(const sensitive_t<u512> &pKey) const {
  u512 lResult;
  auto lSize = serializedSize(true);
  u8 lData[lSize];
  buff_view_t lBuff{lData, lSize};
  write(lBuff, false);
  assert(lBuff.size == 0);
  assert(pKey->size() == crypto_sign_SECRETKEYBYTES);
  assert(id.size() == crypto_sign_PUBLICKEYBYTES);
  SOPHIA_CCALL(crypto_sign_detached(lResult.data(), nullptr, lData, lSize, pKey->data()));
  /*std::cout << "C: " << id << ", " << signature << std::endl;
  std::cout << "C view: " << view() << std::endl;
  std::cout << "C pkey: " << pKey.contained << std::endl;
  std::cout << "C buff: " << buff_view_t{lData, lSize} << std::endl;*/
  return lResult;
}

cbuff_view_t Value::view() const { return cbuff_view_t{data.data(), data.size()}; }

void sophia::randValue(Value &pOut) {
  sensitive_t<u512> privateKey;
  assert(privateKey->size() == crypto_sign_SECRETKEYBYTES);
  assert(pOut.id.size() == crypto_sign_PUBLICKEYBYTES);
  SOPHIA_CCALL(crypto_sign_keypair(pOut.id.data(), privateKey->data()))

  pOut.revision = 0;
  pOut.type = Value::Type::eBlob;

  pOut.data.resize(randombytes_uniform(1024));
  randombytes_buf(pOut.data.data(), pOut.data.size());
  randombytes_buf(pOut.parent.data(), pOut.parent.size());

  pOut.signature = pOut.computeSignature(privateKey);
}

void Event::read(cbuff_view_t &pSource) {
  readBuff(topic.view(), pSource);
  readBuff(source.view(), pSource);
  readBuff(signature.view(), pSource);

  readField<u8>(&height, pSource);
  readField<u8>(&type, pSource);
  readField<u16>(&extra, pSource);
  extra = ntohs(extra);

  data.resize(pSource.size);
  readBuff({data.data(), data.size()}, pSource);
}

void Event::write(buff_view_t &pDest, bool pForSign) const {
  writeBuff(pDest, topic.view());
  writeBuff(pDest, source.view());
  if (pForSign) {
    writeBuff(pDest, signature.view());
    writeField(pDest, height);
  }
  writeField(pDest, type);
  writeField(pDest, htons(extra));

  writeBuff(pDest, {data.data(), data.size()});
}

size_t Event::serializedSize(bool pForSign) const {
  return (pForSign ? 3 : 4) + topic.size() + source.size() + data.size() + (pForSign ? 0 : signature.size());
}

bool Event::signatureValid() const {
  assert(crypto_sign_BYTES == signature.sSize);
  auto lSize = serializedSize(true);
  u8 lData[lSize];
  buff_view_t lBuff{lData, lSize};
  write(lBuff, false);
  assert(lBuff.size == 0);
  /*std::cout << "V: " << source << ", " << signature << std::endl;
  std::cout << "V buff: " << buff_view_t{lData, lSize} << std::endl;
  std::cout << "V result: " << crypto_sign_verify_detached(signature.data(), lData, lSize, source.data()) <<
  std::endl;*/
  return crypto_sign_verify_detached(signature.data(), lData, lSize, source.data()) == 0;
}

u512 Event::computeSignature(const sensitive_t<u512> &pKey) const {
  u512 lResult;
  auto lSize = serializedSize(true);
  u8 lData[lSize];
  buff_view_t lBuff{lData, lSize};
  write(lBuff, false);
  assert(lBuff.size == 0);
  assert(pKey->size() == crypto_sign_SECRETKEYBYTES);
  assert(source.size() == crypto_sign_PUBLICKEYBYTES);
  SOPHIA_CCALL(crypto_sign_detached(lResult.data(), nullptr, lData, lSize, pKey->data()));
  /*std::cout << "C: " << source << ", " << signature << std::endl;
  std::cout << "C pkey: " << pKey.contained << std::endl;
  std::cout << "C buff: " << buff_view_t{lData, lSize} << std::endl;
  std::cout << "C result: " << lResult.view() << std::endl;*/
  return lResult;
}

cbuff_view_t Event::view() const { return cbuff_view_t{data.data(), data.size()}; }
