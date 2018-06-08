#include <iostream>

#include "Database.hpp"

#ifdef SOPHIA_PWHASH_FAST
#define SOPHIA_PWHASH_OPSLIMIT crypto_pwhash_OPSLIMIT_MIN
#define SOPHIA_PWHASH_MEMLIMIT crypto_pwhash_MEMLIMIT_MIN
#else
#define SOPHIA_PWHASH_OPSLIMIT crypto_pwhash_OPSLIMIT_SENSITIVE
#define SOPHIA_PWHASH_MEMLIMIT crypto_pwhash_MEMLIMIT_SENSITIVE
#endif

using namespace sophia;

const char *sInitSQL = "BEGIN;"
                       "CREATE TABLE IF NOT EXISTS profile("
                       "key TEXT PRIMARY KEY,"
                       "value BLOB"
                       ");"
                       "CREATE TABLE IF NOT EXISTS store("
                       "id BLOB PRIMARY KEY,"
                       "type INT,"
                       "revision INT,"
                       "signature BLOB,"
                       "last_sent INT DEFAULT CURRENT_TIMESTAMP,"
                       "last_recv INT DEFAULT CURRENT_TIMESTAMP,"
                       "data BLOB"
                       ");"
                       "COMMIT;";

const char *sVerifyToken = "verified";

int Database::sqlTrace(unsigned pMask, void *pContext, void *pParamP, void *pParamX) {
  switch (pMask) {
  /*case SQLITE_TRACE_STMT: {
    auto lStmt = (sqlite3_stmt*)pParamP;
    char * lSQL = sqlite3_expanded_sql(lStmt);
    std::cout << "[SQL-S] " << (char*) pParamX << "\n\t" << lSQL << std::endl;
    sqlite3_free(lSQL);
  } break;*/
  case SQLITE_TRACE_PROFILE: {
    auto lStmt = (sqlite3_stmt *)pParamP;
    char *lSQL = sqlite3_expanded_sql(lStmt);
    std::chrono::nanoseconds lRawNS(*((int64_t *)pParamX));
    std::cout << "[SQL] " << dur_t{lRawNS} << " " << lSQL << std::endl;
    sqlite3_free(lSQL);
  } break;
  case SQLITE_TRACE_ROW: {
    auto lStmt = (sqlite3_stmt *)pParamP;
    const int lColCount = sqlite3_column_count(lStmt);
    if (lColCount > 0) {
      std::cout << "[ROW] ";
      for (int i = 0; i < lColCount; i++) {
        std::cout << sqlite3_column_name(lStmt, i) << " = ";
        switch (sqlite3_column_type(lStmt, i)) {
        case SQLITE_INTEGER:
          std::cout << sqlite3_column_int64(lStmt, i) << ", ";
          break;
        case SQLITE_FLOAT:
          std::cout << sqlite3_column_double(lStmt, i) << ", ";
          break;
        case SQLITE_BLOB: {
          auto lSize = sqlite3_column_bytes(lStmt, i);
          auto lBuffSize = lSize * 2 + 1;
          char lBuff[lBuffSize];
          sodium_bin2hex(lBuff, lBuffSize, (const u8 *)sqlite3_column_blob(lStmt, i), lSize);
          std::cout << lBuff << ", ";
        } break;
        case SQLITE_NULL:
          std::cout << "null, ";
          break;
        case SQLITE_TEXT:
          std::cout << sqlite3_column_text(lStmt, i) << ", ";
          break;
        default:
          break;
        }
      }
      std::cout << std::endl;
    }
  } break;

  default:
    break;
  }
  return 0;
}

void Database::sqlClzDist(sqlite3_context *pContext, int pArgc, sqlite3_value **pValues) {
  assert(pArgc == 2);
  sqlite3_value *lLValue = pValues[0];
  sqlite3_value *lRValue = pValues[1];
  assert(sqlite3_value_type(lLValue) == SQLITE_BLOB);
  assert(sqlite3_value_type(lRValue) == SQLITE_BLOB);
  assert(sqlite3_value_bytes(lLValue) == 32);
  assert(sqlite3_value_bytes(lRValue) == 32);
  auto lResult = clzDist256((const u8 *)sqlite3_value_blob(lLValue), (const u8 *)sqlite3_value_blob(lRValue));
  sqlite3_result_int(pContext, (int)lResult);
}

bool Database::hasProfileEntry(const std::string_view &pKey) {
  SOPHIA_CCALL(sqlite3_bind_text(selProfileStmt, 1, pKey.data(), (int)pKey.size(), nullptr));
  bool lResult = sqlite3_step(selProfileStmt) == SQLITE_ROW;
  sqlite3_reset(selProfileStmt);
  return lResult;
}

void Database::insertProfileEntry(const std::string_view &pKey, const buff_view_t &pVal) {
  SOPHIA_CCALL(sqlite3_bind_text(insProfileStmt, 1, pKey.data(), (int)pKey.size(), nullptr));
  SOPHIA_CCALL(sqlite3_bind_blob(insProfileStmt, 2, pVal.data, (int)pVal.size, nullptr));
  sqlite3_step(insProfileStmt);
  SOPHIA_CCALL(sqlite3_reset(insProfileStmt));
}

void Database::getProfileEntry(const std::string_view &pKey, buff_view_t pVal) {
  SOPHIA_CCALL(sqlite3_bind_text(selProfileStmt, 1, pKey.data(), (int)pKey.size(), nullptr));
  sqlite3_step(selProfileStmt);

  auto lSize = sqlite3_column_bytes(selProfileStmt, 1);
  assert(lSize <= (int)pVal.size);
  auto lValue = (void *)sqlite3_column_blob(selProfileStmt, 1);
  memcpy(pVal.data, lValue, (size_t)lSize);

  sqlite3_reset(selProfileStmt);
}

keypair_t Database::initialize(const passphrase_t &pPassphrase) {
  // auto lStart = Clock::now();

  auto lPassNonce = rand<u128>();
  privatekey_t lSecretKey;
  static_assert(lPassNonce.size() == crypto_pwhash_SALTBYTES);
  assert(lSecretKey->size() >= crypto_pwhash_BYTES_MIN);
  assert(lSecretKey->size() <= crypto_pwhash_BYTES_MAX);
  SOPHIA_CCALL(crypto_pwhash(lSecretKey->data(), lSecretKey->size(), pPassphrase->data(), pPassphrase->size(),
                             lPassNonce.data(), SOPHIA_PWHASH_OPSLIMIT, SOPHIA_PWHASH_MEMLIMIT,
                             crypto_pwhash_ALG_DEFAULT));

  auto lVerify = rand<u128>();
  memcpy(lVerify.data(), sVerifyToken, strlen(sVerifyToken));
  auto lVerifyNonce = rand<u192>();
  u128 lVerifyCrypt;
  u128 lVerifyMac;
  SOPHIA_CCALL(crypto_secretbox_detached(lVerifyCrypt.data(), lVerifyMac.data(), lVerify.data(), lVerify.size(),
                                         lVerifyNonce.data(), lSecretKey->data()));

  u256 lPubK;
  sensitive_t<u256> lPrivK;
  static_assert(lPubK.size() == crypto_box_PUBLICKEYBYTES);
  assert(lPrivK->size() == crypto_box_SECRETKEYBYTES);
  SOPHIA_CCALL(crypto_box_keypair(lPubK.data(), lPrivK->data()));

  auto lSKNonce = rand<u192>();
  u128 lSKMac;
  u256 lSKCrypt;
  static_assert(lSKNonce.size() == crypto_secretbox_NONCEBYTES);
  static_assert(lSKMac.size() == crypto_secretbox_MACBYTES);
  assert(lSKCrypt.size() == lPrivK->size());
  SOPHIA_CCALL(crypto_secretbox_detached(lSKCrypt.data(), lSKMac.data(), lPrivK->data(), lPrivK->size(),
                                         lSKNonce.data(), lSecretKey->data()));

  SOPHIA_CCALL(sqlite3_exec(db, "BEGIN;", nullptr, nullptr, nullptr));
  insertProfileEntry("publickey", lPubK.view());
  insertProfileEntry("pass_nonce", lPassNonce.view());

  insertProfileEntry("verify_mac", lVerifyMac.view());
  insertProfileEntry("verify_nonce", lVerifyNonce.view());
  insertProfileEntry("verify_crypt", lVerifyCrypt.view());

  insertProfileEntry("privatekey_mac", lSKMac.view());
  insertProfileEntry("privatekey_nonce", lSKNonce.view());
  insertProfileEntry("privatekey_crypt", lSKCrypt.view());
  SOPHIA_CCALL(sqlite3_exec(db, "COMMIT;", nullptr, nullptr, nullptr));

  // std::cout << "[SDB] Initialized in " << dur_t{Clock::now() - lStart} << std::endl;

  return {lPubK, lPrivK};
}

Database::Database(const char *pPath) {
  uint32_t lFlags = SQLITE_OPEN_NOMUTEX | SQLITE_OPEN_URI | SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE;
  SOPHIA_CCALL(sqlite3_open_v2(pPath, &db, lFlags, nullptr));

  // SOPHIA_CCALL(sqlite3_trace_v2(db, 0xFF, &Database::sqlTrace, this));
  SOPHIA_CCALL(sqlite3_exec(db, sInitSQL, nullptr, nullptr, nullptr));

  SOPHIA_CCALL(sqlite3_prepare_v2(db, "INSERT INTO profile (key, value) VALUES (?, ?)", -1, &insProfileStmt, nullptr));
  SOPHIA_CCALL(sqlite3_prepare_v2(db, "SELECT value FROM profile WHERE key = ?;", -1, &selProfileStmt, nullptr));

  SOPHIA_CCALL(sqlite3_prepare_v2(db,
                                  "INSERT INTO store "
                                  "(id, type, revision, signature, data) "
                                  "VALUES (?, ?, ?, ?, ?)",
                                  -1, &insValueStmt, nullptr));
  SOPHIA_CCALL(sqlite3_prepare_v2(db,
                                  "SELECT id, type, revision, signature, data "
                                  "FROM store WHERE id = ?;",
                                  -1, &selValueStmt, nullptr));
  SOPHIA_CCALL(sqlite3_prepare_v2(db, "SELECT revision FROM store WHERE id = ?;", -1, &hasValueStmt, nullptr));

  SOPHIA_CCALL(sqlite3_create_function(db, "clzDist", 2, SQLITE_UTF8, this, sqlClzDist, nullptr, nullptr));
}

Database::~Database() {
  sqlite3_finalize(selProfileStmt);
  sqlite3_finalize(insProfileStmt);
  sqlite3_finalize(selValueStmt);
  sqlite3_finalize(insValueStmt);
  sqlite3_close(db);
}

keypair_t Database::loadProfile(const passphrase_t &pPassphrase) {
  if (!hasProfileEntry("publickey")) {
    return initialize(pPassphrase);
  }

  auto lPublicKey = retreiveProfileEntry<u256>("publickey");
  auto lPassNonce = retreiveProfileEntry<u128>("pass_nonce");

  auto lVerifyMac = retreiveProfileEntry<u128>("verify_mac");
  auto lVerifyNonce = retreiveProfileEntry<u192>("verify_nonce");
  auto lVerifyCrypt = retreiveProfileEntry<u128>("verify_crypt");

  auto lSKMac = retreiveProfileEntry<u128>("privatekey_mac");
  auto lSKNonce = retreiveProfileEntry<u192>("privatekey_nonce");
  auto lSKCrypt = retreiveProfileEntry<u256>("privatekey_crypt");

  static_assert(lPassNonce.size() == crypto_pwhash_SALTBYTES);
  assert(secretKey->size() >= crypto_pwhash_BYTES_MIN);
  assert(secretKey->size() <= crypto_pwhash_BYTES_MAX);
  SOPHIA_CCALL(crypto_pwhash(secretKey->data(), secretKey->size(), pPassphrase->data(), pPassphrase->size(),
                             lPassNonce.data(), SOPHIA_PWHASH_OPSLIMIT, SOPHIA_PWHASH_MEMLIMIT,
                             crypto_pwhash_ALG_DEFAULT));

  u128 lVerify;
  SOPHIA_CCALL(crypto_secretbox_open_detached(lVerify.data(), lVerifyCrypt.data(), lVerifyMac.data(),
                                              lVerifyCrypt.size(), lVerifyNonce.data(), secretKey->data()));

  if (memcmp(lVerify.data(), sVerifyToken, strlen(sVerifyToken)) != 0)
    throw std::runtime_error("wrong passphrase");

  privatekey_t lPrivateKey;
  SOPHIA_CCALL(crypto_secretbox_open_detached(lPrivateKey->data(), lSKCrypt.data(), lSKMac.data(), lSKCrypt.size(),
                                              lSKNonce.data(), secretKey->data()));

  return {lPublicKey, lPrivateKey};
}

int32_t Database::getRevision(const u8 *pKey) {
  SOPHIA_CCALL(sqlite3_bind_blob(hasValueStmt, 1, pKey, 32, nullptr));
  int32_t lRevision = -1;
  int lResult = sqlite3_step(hasValueStmt);
  if (lResult == SQLITE_ROW) {
    lRevision = sqlite3_column_int(hasValueStmt, 1);
  }
  sqlite3_reset(hasValueStmt);
  return lRevision;
}

std::optional<Value> Database::loadValue(const u8 *pKey) {
  SOPHIA_CCALL(sqlite3_bind_blob(selValueStmt, 1, pKey, 32, nullptr));
  int lResult = sqlite3_step(selValueStmt);
  if (lResult != SQLITE_ROW) {
    SOPHIA_CCALL(sqlite3_reset(selValueStmt));
    return {};
  }

  Value lValue;

  assert(sqlite3_column_bytes(selValueStmt, 0) == 32);
  memcpy(lValue.id.data(), sqlite3_column_blob(selValueStmt, 0), 32);

  lValue.type = (Value::Type)sqlite3_column_int(selValueStmt, 1);
  lValue.revision = (Value::Type)sqlite3_column_int(selValueStmt, 2);

  assert(sqlite3_column_bytes(selValueStmt, 3) == 64);
  memcpy(lValue.signature.data(), sqlite3_column_blob(selValueStmt, 3), 64);

  auto lDataSize = (uint32_t)sqlite3_column_bytes(selValueStmt, 4);
  lValue.data.resize(lDataSize);
  memcpy(lValue.data.data(), sqlite3_column_blob(selValueStmt, 4), lDataSize);

  SOPHIA_CCALL(sqlite3_reset(selValueStmt));

  return lValue;
}

void Database::mayStoreValue(const Value &pValue) {
  auto lLocalRevision = getRevision(pValue.id.data());
  if ((int32_t)pValue.revision > lLocalRevision) {
    if (!pValue.signatureValid()) {
      std::cout << "invalid signature!" << std::endl;
      return;
    }
    if (pValue.data.size() > 1024) {
      return;
    }

    SOPHIA_CCALL(sqlite3_bind_blob(insValueStmt, 1, pValue.id.data(), 32, nullptr));
    SOPHIA_CCALL(sqlite3_bind_int(insValueStmt, 2, pValue.type));
    SOPHIA_CCALL(sqlite3_bind_int(insValueStmt, 3, pValue.revision));
    SOPHIA_CCALL(sqlite3_bind_blob(insValueStmt, 4, pValue.signature.data(), 64, nullptr));
    SOPHIA_CCALL(sqlite3_bind_blob(insValueStmt, 5, pValue.data.data(), (int)pValue.data.size(), nullptr));
    sqlite3_step(insValueStmt);

    SOPHIA_CCALL(sqlite3_reset(insValueStmt));
  }
}

sqlite3 *Database::getSQLite() { return db; }
