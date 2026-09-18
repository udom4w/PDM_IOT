// ============================================================================
// [PHASE1.5-S3] VibNetworkManager implementation
// Spec: docs/engineering/PHASE1_5_DUAL_NETWORK_CHANGE_SPEC.md §1
// ============================================================================
#include "network_manager.h"

VibNetworkManager::VibNetworkManager()
  : _wifi(nullptr), _gprs(nullptr), _active(nullptr),
    _mode(NETWORK_GPRS_ONLY), _state(NET_DOWN) {}

void VibNetworkManager::registerTransport(NetworkMode_t mode, INetTransport* transport) {
  switch (mode) {
    case NETWORK_WIFI_ONLY: _wifi = transport; break;
    case NETWORK_GPRS_ONLY: _gprs = transport; break;
    default: /* NETWORK_AUTO has no transport of its own */ break;
  }
}

INetTransport* VibNetworkManager::_transportForMode(NetworkMode_t mode) const {
  switch (mode) {
    case NETWORK_WIFI_ONLY: return _wifi;
    case NETWORK_GPRS_ONLY: return _gprs;
    default:                return nullptr;
  }
}

const char* VibNetworkManager::modeName() const {
  switch (_mode) {
    case NETWORK_WIFI_ONLY: return "WIFI_ONLY";
    case NETWORK_GPRS_ONLY: return "GPRS_ONLY";
    case NETWORK_AUTO:      return "AUTO";
    default:                return "?";
  }
}

bool VibNetworkManager::begin(NetworkMode_t mode) {
  // [§1.6] NETWORK_AUTO is architecture-ready but deliberately NOT implemented.
  // Refuse it explicitly rather than silently selecting a transport: a silent
  // fallback is exactly how a damaged modem could get energised by accident.
  if (mode == NETWORK_AUTO) {
    Serial.println("[NET] NETWORK_AUTO is not implemented in this build -- refused");
    _state = NET_FAULT;
    return false;
  }

  INetTransport* t = _transportForMode(mode);
  if (t == nullptr) {
    Serial.printf("[NET] No transport registered for mode %d -- cannot begin\n", (int)mode);
    _mode   = mode;
    _active = nullptr;
    _state  = NET_FAULT;
    return false;
  }

  _mode   = mode;
  _active = t;                 // set before begin() so active() is valid even on failure
  _state  = NET_CONNECTING;

  Serial.printf("[NET] Selected transport: %s (mode=%s)\n", t->name(), modeName());

  if (!_active->begin()) {
    Serial.printf("[NET] Transport %s begin() failed\n", _active->name());
    _state = NET_FAULT;
    return false;
  }
  return true;
}

bool VibNetworkManager::connect() {
  if (_active == nullptr) {
    _state = NET_FAULT;
    return false;
  }
  _state = NET_CONNECTING;
  bool ok = _active->connect();
  _state = ok ? NET_UP : NET_DOWN;
  return ok;
}

void VibNetworkManager::disconnect() {
  if (_active == nullptr) return;
  _active->disconnect();
  _state = NET_DOWN;
}

bool VibNetworkManager::isConnected() {
  if (_active == nullptr) return false;
  return _active->linkUp();
}

NetworkState_t VibNetworkManager::health() {
  if (_active == nullptr) {
    _state = NET_DOWN;
    return _state;
  }
  if (isConnected()) {
    _state = NET_UP;
  } else if (_state == NET_UP) {
    _state = NET_DOWN;   // link was up and has been lost
  }
  return _state;
}
