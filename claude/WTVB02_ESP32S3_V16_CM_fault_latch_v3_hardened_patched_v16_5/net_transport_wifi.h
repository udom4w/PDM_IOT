#pragma once
// ============================================================================
// [PHASE1.5-S5/S6] WiFiTransport -- Wi-Fi + WiFiClientSecure + SNTP
// ----------------------------------------------------------------------------
// Spec: docs/engineering/PHASE1_5_DUAL_NETWORK_CHANGE_SPEC.md §3, §6.3
//
// ACTIVE SINCE STEP 7: NETWORK_WIFI_ONLY is the runtime default, so this is
// normally the selected transport. Nothing here touches the radio until
// begin() is called, and begin() is only reached via
// VibNetworkManager::begin(NETWORK_WIFI_ONLY).
//
// Credentials and time-validity bounds are INJECTED (setCredentials/
// setTimeBounds/setWifiCredentials) so that the single definition of the mTLS
// PEMs and of FL_TS_MIN_VALID / MAX_VALID_YEAR stays in the sketch -- this
// class never duplicates a constant that already exists.
// ============================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "net_transport.h"

class WiFiTransport : public INetTransport {
public:
  WiFiTransport();

  // -- injection (call before begin(); no radio activity) --------------------
  void setWifiCredentials(const char* ssid, const char* passphrase);
  void setCredentials(const char* ca, const char* clientCert, const char* privateKey);
  void setTimeBounds(uint32_t minEpoch, uint16_t maxYear);
  void setNtpServers(const char* primary, const char* secondary);
  void setAssocTimeoutMs(uint32_t ms);

  // -- INetTransport ---------------------------------------------------------
  bool        begin() override;
  bool        connect() override;
  void        disconnect() override;
  bool        linkUp() override;
  Client&     client() override { return _client; }
  bool        applyCredentials() override;
  void        resetSecureSession() override;
  bool        fetchNetworkUTC(DateTime& out) override;
  int         signalPercent() override;
  const char* name() const override { return "WIFI"; }
  const char* localIP() override;

  // Association-window helper for the future non-blocking connect ladder.
  bool associationTimedOut() const;

private:
  WiFiClientSecure _client;

  const char* _ssid;
  const char* _pass;
  const char* _ca;
  const char* _crt;
  const char* _key;
  const char* _ntp1;
  const char* _ntp2;

  uint32_t _minEpoch;      // lower sanity bound (injected: FL_TS_MIN_VALID)
  uint16_t _maxYear;       // upper sanity bound (injected: MAX_VALID_YEAR)
  uint32_t _assocTimeoutMs;
  uint32_t _assocStartMs;
  bool     _sntpStarted;
  char     _ipStr[20];
};
