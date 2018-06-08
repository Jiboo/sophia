#pragma once

#include <sqlite3.h>

#include "Value.hpp"
#include "utils.hpp"

namespace sophia {

class Database {
  sqlite3 *db = nullptr;
  sqlite3_stmt *insProfileStmt = nullptr;
  sqlite3_stmt *selProfileStmt = nullptr;
  sqlite3_stmt *hasValueStmt = nullptr;
  sqlite3_stmt *insValueStmt = nullptr;
  sqlite3_stmt *selValueStmt = nullptr;

  privatekey_t secretKey;

  static int sqlTrace(unsigned pMask, void *pContext, void *pParamP, void *pParamX);
  static void sqlClzDist(sqlite3_context *pContext, int pArgc, sqlite3_value **pValues);

  bool hasProfileEntry(const std::string_view &pKey);

  void insertProfileEntry(const std::string_view &pKey, const buff_view_t &pVal);
  void getProfileEntry(const std::string_view &pKey, buff_view_t pVal);

  template <typename TConainer> TConainer retreiveProfileEntry(const std::string_view &pKey) {
    TConainer lResult;
    getProfileEntry(pKey, buff_view_t{lResult.data(), lResult.size()});
    return lResult;
  }

  keypair_t initialize(const passphrase_t &pPassphrase);

public:
  explicit Database(const char *pPath);
  ~Database();

  keypair_t loadProfile(const passphrase_t &pPassphrase);

  int32_t getRevision(const u8 *pKey);
  std::optional<Value> loadValue(const u8 *pKey);
  void mayStoreValue(const Value &pValue);

  sqlite3 *getSQLite();
};

} // namespace sophia
