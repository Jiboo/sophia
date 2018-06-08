#include <iostream>

#include <miniupnpc/miniupnpc.h>
#include <miniupnpc/upnpcommands.h>

#include "Node.hpp"

using namespace sophia;

sensitive_t<std::string> getPassphrase() {
  // https://www.gnu.org/savannah-checkouts/gnu/libc/manual/html_node/getpass.html
  sensitive_t<std::string> lResult;

  /* disabling echo */
  termios lOld, lNoEcho;
  tcgetattr(fileno(stdin), &lOld);
  lNoEcho = lOld;
  lNoEcho.c_lflag &= ~ECHO;
  lNoEcho.c_lflag |= ECHONL;

  SOPHIA_CCALL(tcsetattr(fileno(stdin), TCSANOW, &lNoEcho));

  std::cout << "Passphrase: ";
  std::cin >> lResult.contained;

  /* restore terminal */
  SOPHIA_CCALL(tcsetattr(fileno(stdin), TCSANOW, &lOld));

  return lResult;
}

int upnpConfigure(const u16 pPort, const boost::posix_time::seconds &pLeaseTime) {
  // Credits: https://gist.github.com/fsmv/389de500ddac60c52a7d
  int error = 0;
  auto lPortStr = std::to_string(pPort);
  auto lLeaseTimeStr = std::to_string(pLeaseTime.total_seconds());
  std::cout << "[UPnP] Trying to lease port " << lPortStr << std::endl;

  UPNPDev *lDevice = upnpDiscover(100, nullptr, nullptr, 0, 1, 2, &error);
  if (lDevice == nullptr || error != 0) {
    std::cout << "[UPnP] Couldn't find a IPv6 UPnP device, fallback to IPv4" << std::endl;

    lDevice = upnpDiscover(100, nullptr, nullptr, 0, 0, 2, &error);
    if (lDevice == nullptr || error != 0) {
      std::cout << "[UPnP] Couldn't find a UPnP device (" << error << ")" << std::endl;
      freeUPNPDevlist(lDevice);
      return 1;
    }
  }

  char lLocalAddress[INET6_ADDRSTRLEN];
  UPNPUrls lUrls;
  IGDdatas lIGDData;
  error = UPNP_GetValidIGD(lDevice, &lUrls, &lIGDData, lLocalAddress, sizeof(lLocalAddress));
  if (error != 1) {
    std::cout << "[UPnP] Couldn't find an IGD device (" << error << ")" << std::endl;
    FreeUPNPUrls(&lUrls);
    freeUPNPDevlist(lDevice);
    return -1;
  }
  std::cout << "[UPnP] Local address: " << lLocalAddress << std::endl;

  char lExternalAddress[INET6_ADDRSTRLEN];
  error = UPNP_GetExternalIPAddress(lUrls.controlURL, lIGDData.first.servicetype, lExternalAddress);
  if (error != 0) {
    std::cout << "[UPnP] Couldn't find an IGD device (" << error << ")" << std::endl;
    FreeUPNPUrls(&lUrls);
    freeUPNPDevlist(lDevice);
    return -1;
  }
  std::cout << "[UPnP] External address: " << lExternalAddress << std::endl;

  error = UPNP_AddPortMapping(lUrls.controlURL, lIGDData.first.servicetype, lPortStr.c_str(), lPortStr.c_str(),
                              lLocalAddress, "Sophia DHT", "UDP", nullptr, lLeaseTimeStr.c_str());
  if (error != 0) {
    std::cout << "[UPnP] Couldn't map requested port (" << error << ")" << std::endl;
  } else {
    std::cout << "[UPnP] Successfully leased port " << lPortStr << std::endl;
  }

  FreeUPNPUrls(&lUrls);
  freeUPNPDevlist(lDevice);

  return error;
}

void helpMsg() { std::cout << "usage: sophia --db <dbpath>" << std::endl; }

int main(int argc, char **argv) {
  fs::path lDbPath;

  for (int i = 1; i < argc; i++) {
    if (strcmp("--help", argv[i]) == 0) {
      helpMsg();
      return EXIT_SUCCESS;
    }
    if (strcmp("--db", argv[i]) == 0 && argc >= (i + 1)) {
      lDbPath = argv[++i];
    } else {
      std::cout << "unknown argument: " << argv[i] << std::endl;
    }
  }

  if (lDbPath.empty()) {
    helpMsg();
    return EXIT_FAILURE;
  }

  std::cout << "Boost: " << BOOST_VERSION / 100000 << "." // maj. version
            << BOOST_VERSION / 100 % 1000 << "."          // min. version
            << BOOST_VERSION % 100                        // patch version
            << "\nSQLite: " << sqlite3_version << "\nsodium: " << sodium_version_string()
            << "\ndatabase: " << lDbPath.c_str() << std::endl;

  SOPHIA_CCALL(sodium_init());

#if 1
  fs::remove(lDbPath);
  passphrase_t lPassphrase("passphrase");
#else
  passphrase lPassphrase = getPassphrase();
#endif

  try {
    net::io_service lService;
    Node node(lService, lDbPath.c_str(), lPassphrase, net::ip::udp::endpoint(net::ip::udp::v6(), 0));

    auto lSelfContact = node.self();
    std::cout << "[NET] Listening on [" << lSelfContact.address.to_string() << "]:" << lSelfContact.port << std::endl;

    boost::posix_time::seconds lUPnPLeaseTime(60 * 60);
    net::deadline_timer lLeaseRefresh(lService, lUPnPLeaseTime);

    std::function<void(boost::system::error_code)> lLeaseRefresher;
    lLeaseRefresher = [lUPnPLeaseTime, &lLeaseRefresh, &lLeaseRefresher,
                       lSelfContact](const boost::system::error_code &pError) {
      upnpConfigure(lSelfContact.port, lUPnPLeaseTime);
      lLeaseRefresh.expires_at(lLeaseRefresh.expires_at() + lUPnPLeaseTime);
      lLeaseRefresh.async_wait(lLeaseRefresher);
    };
    upnpConfigure(lSelfContact.port, lUPnPLeaseTime);
    lLeaseRefresh.async_wait(lLeaseRefresher);

    lService.run();
  } catch (std::exception &pException) {
    std::cout << "Caught exception: " << pException.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
