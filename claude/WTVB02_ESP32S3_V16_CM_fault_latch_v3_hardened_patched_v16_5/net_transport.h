#pragma once
// ============================================================================
// [PHASE1.5-S1] INetTransport -- network transport abstraction
// ----------------------------------------------------------------------------
// Spec: docs/engineering/PHASE1_5_DUAL_NETWORK_CHANGE_SPEC.md §2
//
// Every method below exists because a concrete call site in the existing
// firmware requires it -- this interface was derived from the call sites, not
// from convention. The "replaces" note on each method is the binding
// constraint; do not add methods without a call site that needs one.
//
// Deliberately NOT in this interface: imei, operatorName, csq, apn, rssi.
// Those are transport-specific and are exposed (if at all) only through
// signalPercent()/name(). That exclusion is what keeps cellular vocabulary
// from leaking back into shared code.
// ============================================================================
#include <Arduino.h>
#include <Client.h>
#include <RTClib.h>   // DateTime -- already the firmware's time type (rtc.adjust)

class INetTransport {
public:
  virtual ~INetTransport() {}

  // Link-layer bring-up. Replaces modemInit() / WiFi.mode()+begin().
  virtual bool begin() = 0;

  // Establish the data link (GPRS attach / association + DHCP).
  // Replaces modemConnectGPRS().
  virtual bool connect() = 0;

  // Clean teardown. Used before a transport switch (FUTURE) and on shutdown.
  virtual void disconnect() = 0;

  // LIVE link query -- must NOT return a latched flag.
  // Replaces modem.isGprsConnected() / WiFi.status()==WL_CONNECTED.
  // (g_network.modemReady is latched true and never cleared; that defect must
  //  not be inherited here.)
  virtual bool linkUp() = 0;

  // The Client handed to MQTTClient::begin(host, port, Client&).
  // Both GsmTLSClient and WiFiClientSecure derive from Client, so the MQTT
  // layer above is transport-agnostic.
  virtual Client& client() = 0;

  // Install the mTLS material (CA + client cert + private key).
  // Replaces the body of setupTLS().
  virtual bool applyCredentials() = 0;

  // Clear stale TLS session state before a reconnect attempt.
  // Replaces the unconditional gsmClient.resetTLS() call.
  virtual void resetSecureSession() = 0;

  // Acquire network wall-clock as UTC. Replaces modem.getGSMDateTime(DATE_FULL)
  // on GPRS and SNTP on Wi-Fi. Returns false if no valid time is available;
  // the caller then applies nothing (see applyNetworkTime()).
  virtual bool fetchNetworkUTC(DateTime& out) = 0;

  // Signal strength as a PERCENTAGE (0..100) -- the unit the OLED already
  // consumes. Display only; never published.
  virtual int signalPercent() = 0;

  // Short transport name for logs/OLED/banner, e.g. "GPRS" / "WIFI".
  virtual const char* name() const = 0;

  // Local address for diagnostics. May return "" when not applicable.
  virtual const char* localIP() = 0;
};
