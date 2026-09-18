#pragma once
// ============================================================================
// [PHASE1.5-S3] VibNetworkManager -- transport selection and link-state ownership
// ----------------------------------------------------------------------------
// Spec: docs/engineering/PHASE1_5_DUAL_NETWORK_CHANGE_SPEC.md §1
//
// SCOPE NOTE: this manager owns LINK state only.
// MQTT state already has an owner (g_systemState.mqttConnected, written by the
// network task and read via getMqttConnectedCached() under mutexSystemState)
// and time state already has an owner (g_timeSync). Folding either into
// NetworkState_t would create a second source of truth for state that already
// has one, so NetworkState_t deliberately describes the link and nothing else.
//
// Transports are INJECTED via registerTransport() rather than constructed here:
// the GPRS transport necessarily lives in the .ino (it wraps modem/gsmClient
// objects defined there), so the manager must not depend on its concrete type.
// ============================================================================
#include <Arduino.h>
#include "net_transport.h"

// Numeric values are PERSISTED in NVS (key cfg_netmode) and are therefore
// part of the device's stored contract -- do not renumber them.
//
// FAIL-SAFE RULE (taskNetwork): GPRS is selected ONLY when the stored value is
// exactly NETWORK_GPRS_ONLY (1). Every other value -- 0, 2, negative, garbage
// from a corrupt NVS record -- resolves to NETWORK_WIFI_ONLY. This is a
// whitelist, not a blacklist, because the A7670 fitted to this prototype is
// physically damaged and must never be energised by an unrecognised value.
typedef enum {
  NETWORK_WIFI_ONLY = 0,   // DEFAULT since Step 7, and the fail-safe fallback
  NETWORK_GPRS_ONLY = 1,   // reproduces v16.5.0 behaviour; opt-in only
  NETWORK_AUTO      = 2    // FUTURE -- declared only; refused at runtime (§1.6)
} NetworkMode_t;

typedef enum {
  NET_DOWN       = 0,   // no link, not attempting
  NET_CONNECTING = 1,   // association / attach in progress
  NET_UP         = 2,   // link usable
  NET_FAULT      = 3    // selection/bring-up failed
} NetworkState_t;

class VibNetworkManager {
public:
  VibNetworkManager();

  // Inject a concrete transport for a mode. Safe to call before begin().
  void registerTransport(NetworkMode_t mode, INetTransport* transport);

  // Select + bring up the transport for `mode`.
  // NETWORK_AUTO is refused in this build (see §1.6) -- it does not fall
  // through to any cellular path.
  bool begin(NetworkMode_t mode);

  // Establish the data link on the active transport.
  bool connect();

  // Tear down the active transport's link.
  void disconnect();

  // LIVE link query, delegated to the active transport. This is a 1:1
  // replacement for a direct transport query -- it issues exactly one
  // underlying query per call, so it can replace an existing call site
  // without changing how often the transport is polled.
  bool isConnected();

  // Evaluate and return link state. NOTE: this calls isConnected() and
  // therefore polls the transport -- do not add calls to it inside a loop
  // that is already polling, or the poll rate changes.
  NetworkState_t health();

  bool            hasActive() const { return _active != nullptr; }
  INetTransport&  active()          { return *_active; }   // precondition: hasActive()
  Client&         client()          { return _active->client(); }

  NetworkMode_t   mode()  const { return _mode; }
  NetworkState_t  state() const { return _state; }
  const char*     modeName() const;

private:
  INetTransport* _transportForMode(NetworkMode_t mode) const;

  INetTransport* _wifi;
  INetTransport* _gprs;
  INetTransport* _active;
  NetworkMode_t  _mode;
  NetworkState_t _state;
};
