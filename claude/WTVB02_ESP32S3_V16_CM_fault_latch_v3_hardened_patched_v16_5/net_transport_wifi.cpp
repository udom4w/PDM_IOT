// ============================================================================
// [PHASE1.5-S5/S6] WiFiTransport implementation
// Spec: docs/engineering/PHASE1_5_DUAL_NETWORK_CHANGE_SPEC.md §3, §6.3
//
// ACTIVE SINCE STEP 7 -- see net_transport_wifi.h header note.
// ============================================================================
#include "net_transport_wifi.h"
#include <time.h>

// Association window (§3.2): 15 s, well inside the 30 s task WDT and about
// half the modem's per-attempt budget. Overridable via setAssocTimeoutMs().
static const uint32_t WIFI_ASSOC_TIMEOUT_MS_DEFAULT = 15000UL;

// Bounded SNTP wait (§6.3). 5 s ceiling, also well inside the 30 s task WDT.
static const uint32_t WIFI_SNTP_WAIT_MS = 5000UL;

// RSSI -> percent presentation mapping (§3.4). Display only, never published.
static const int WIFI_RSSI_MIN_DBM = -90;   //   0 %
static const int WIFI_RSSI_MAX_DBM = -30;   // 100 %

WiFiTransport::WiFiTransport()
  : _ssid(nullptr), _pass(nullptr),
    _ca(nullptr), _crt(nullptr), _key(nullptr),
    _ntp1("pool.ntp.org"), _ntp2("time.nist.gov"),
    _minEpoch(0), _maxYear(0),
    _assocTimeoutMs(WIFI_ASSOC_TIMEOUT_MS_DEFAULT),
    _assocStartMs(0), _sntpStarted(false) {
  _ipStr[0] = '\0';
}

void WiFiTransport::setWifiCredentials(const char* ssid, const char* passphrase) {
  _ssid = ssid;
  _pass = passphrase;
}

void WiFiTransport::setCredentials(const char* ca, const char* clientCert, const char* privateKey) {
  _ca  = ca;
  _crt = clientCert;
  _key = privateKey;
}

void WiFiTransport::setTimeBounds(uint32_t minEpoch, uint16_t maxYear) {
  _minEpoch = minEpoch;
  _maxYear  = maxYear;
}

void WiFiTransport::setNtpServers(const char* primary, const char* secondary) {
  if (primary)   _ntp1 = primary;
  if (secondary) _ntp2 = secondary;
}

void WiFiTransport::setAssocTimeoutMs(uint32_t ms) {
  _assocTimeoutMs = ms;
}

bool WiFiTransport::begin() {
  if (_ssid == nullptr || _ssid[0] == '\0') {
    // §7.2: absent SSID must NOT fall back to any cellular path -- it is a
    // hard configuration error that retries on the caller's backoff ladder.
    Serial.println("[WIFI] No SSID configured -- cannot start Wi-Fi");
    return false;
  }
  WiFi.persistent(false);          // do not wear flash with credential writes
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  Serial.printf("[WIFI] Station mode ready (ssid=%s)\n", _ssid);
  return true;
}

bool WiFiTransport::connect() {
  if (_ssid == nullptr || _ssid[0] == '\0') return false;

  // Non-blocking by design (§3.2): issue the join and return. The caller polls
  // linkUp() on its existing loop cadence. No delay() is used here -- the
  // blocking style of the modem path must not be inherited.
  _assocStartMs = millis();
  _sntpStarted  = false;           // re-arm SNTP for the new association
  WiFi.begin(_ssid, _pass);
  Serial.printf("[WIFI] Association started (timeout=%lums)\n",
                (unsigned long)_assocTimeoutMs);
  return true;
}

void WiFiTransport::disconnect() {
  WiFi.disconnect(true);
  _sntpStarted = false;
  Serial.println("[WIFI] Disconnected");
}

bool WiFiTransport::linkUp() {
  // LIVE query -- never a latched flag (§4.3 of the spec).
  if (WiFi.status() != WL_CONNECTED) return false;
  return (uint32_t)WiFi.localIP() != 0;
}

bool WiFiTransport::associationTimedOut() const {
  if (_assocStartMs == 0) return false;
  return (millis() - _assocStartMs) > _assocTimeoutMs;
}

// [PHASE1.5-MQTTWORKER][DEFENSE-IN-DEPTH] TLS handshake timeout bound.
// *** PROVISIONAL -- UNKNOWN, REQUIRES HARDWARE VALIDATION. *** Not a
// field-measured figure: derived only from the existing in-repo GPRS
// TLS-handshake comment ("TLS handshake ~2s", the FIX-WDT block in the
// .ino) scaled up with margin for unmeasured Wi-Fi/broker RTT variability
// at the pump01 site. Must be corrected against real network timing before
// being treated as final. See
// docs/engineering/PHASE1_5_MQTT_WORKER_IMPLEMENTATION_PLAN.md §4.
//
// This bounds ONLY the TLS handshake phase (NetworkClientSecure's
// handshake_timeout). It does NOT bound the separate TCP/socket-connect
// phase (socket_timeout, default 30000ms): exhaustive inspection of
// NetworkClientSecure.h/.cpp found no public setter for that field that
// does not also perform a real, side-effecting connection attempt (the only
// reachable overload, connect(host,port,timeout), both sets the field AND
// immediately runs a full connect+handshake). That TCP-phase exposure is
// therefore left at its unmodified default -- it is not fixable from
// application code. See
// docs/engineering/PHASE1_5_WIFI_TLS_WATCHDOG_IMPLEMENTATION_DECISION.md §2.2.
static const unsigned long WIFI_TLS_HANDSHAKE_TIMEOUT_S_PROVISIONAL = 8;

bool WiFiTransport::applyCredentials() {
  if (_ca == nullptr || _crt == nullptr || _key == nullptr) {
    Serial.println("[WIFI] mTLS material not injected -- cannot apply credentials");
    return false;
  }
  // Identical API to GsmTLSClient (setCACert/setCertificate/setPrivateKey),
  // using the same three PEM constants defined once in the sketch.
  _client.setCACert(_ca);
  _client.setCertificate(_crt);
  _client.setPrivateKey(_key);
  // Durable across every future _client.stop()/reconnect -- confirmed:
  // stop_ssl_socket() explicitly preserves handshake_timeout across its
  // teardown (ssl_client.cpp:402-414) -- so this one-time call is
  // sufficient for the process lifetime; it does not need to be reissued
  // per connect attempt (including the MQTT worker's own attempts).
  _client.setHandshakeTimeout(WIFI_TLS_HANDSHAKE_TIMEOUT_S_PROVISIONAL);
  Serial.printf("[WIFI] mTLS certificates applied to WiFiClientSecure (handshake timeout=%lus PROVISIONAL)\n",
                WIFI_TLS_HANDSHAKE_TIMEOUT_S_PROVISIONAL);
  return true;
}

void WiFiTransport::resetSecureSession() {
  // Equivalent of GsmTLSClient::resetTLS() for this transport: drop stale
  // session state before a reconnect attempt.
  _client.stop();
}

bool WiFiTransport::fetchNetworkUTC(DateTime& out) {
  if (!linkUp()) {
    Serial.println("[NTP] Wi-Fi link down, skip sync");
    return false;
  }

  if (!_sntpStarted) {
    // UTC only (offset 0, no DST): the firmware's time contract is UTC --
    // applyNetworkTime() and every published timestamp expect UTC.
    configTime(0, 0, _ntp1, _ntp2);
    _sntpStarted = true;
    Serial.printf("[NTP] SNTP started (%s, %s)\n", _ntp1, _ntp2);
  }

  // Bounded wait for SNTP to deliver a first sample.
  struct tm tmInfo;
  if (!getLocalTime(&tmInfo, WIFI_SNTP_WAIT_MS)) {
    Serial.println("[NTP] SNTP has no valid time yet");
    return false;
  }

  time_t nowEpoch = time(nullptr);

  // Sanity bounds REUSE the sketch's existing constants (injected) -- no new
  // definition of "plausible time" is introduced by this transport.
  if (_minEpoch != 0 && (uint32_t)nowEpoch < _minEpoch) {
    Serial.printf("[NTP] SNTP epoch %lu below minimum %lu -- rejected\n",
                  (unsigned long)nowEpoch, (unsigned long)_minEpoch);
    return false;
  }

  DateTime utc((uint32_t)nowEpoch);
  if (_maxYear != 0 && utc.year() > _maxYear) {
    Serial.printf("[NTP] SNTP year %d above maximum %u -- rejected\n",
                  utc.year(), (unsigned)_maxYear);
    return false;
  }

  Serial.printf("[NTP] UTC  : %04d-%02d-%02dT%02d:%02d:%02dZ (SNTP)\n",
                utc.year(), utc.month(), utc.day(),
                utc.hour(), utc.minute(), utc.second());

  out = utc;
  return true;
}

int WiFiTransport::signalPercent() {
  if (WiFi.status() != WL_CONNECTED) return 0;
  int rssi = (int)WiFi.RSSI();
  if (rssi <= WIFI_RSSI_MIN_DBM) return 0;
  if (rssi >= WIFI_RSSI_MAX_DBM) return 100;
  return (int)(((long)(rssi - WIFI_RSSI_MIN_DBM) * 100L) /
               (long)(WIFI_RSSI_MAX_DBM - WIFI_RSSI_MIN_DBM));
}

const char* WiFiTransport::localIP() {
  if (WiFi.status() != WL_CONNECTED) {
    _ipStr[0] = '\0';
    return _ipStr;
  }
  IPAddress ip = WiFi.localIP();
  snprintf(_ipStr, sizeof(_ipStr), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  return _ipStr;
}
