# `ATTraceStream` — Raw AT/TLS Modem Diagnostic — Preserved Evidence

**Classification: EXPERIMENTAL / DIAGNOSTIC — NOT PRODUCTION**
**Extraction date:** 2026-08-27

---

## Provenance

| Field | Value |
|---|---|
| Original source path | `claude/firmware/Commit8_CurrentEvidenceFix_RC/WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino` |
| Original `.ino` SHA256 (working copy, unchanged) | `363c84492516d1e0f407697dc396407f017ae01ee10d2df11fa27e39df51ce0c` |
| Last real commit touching this file | `6b3cad6` — "RC: TLS handshake investigation candidate" |
| Local-diff status | **Uncommitted.** 232 insertions / 8 deletions on top of `6b3cad6`, sitting in the working tree since at least 2026-08-06 (file mtime), never staged or committed |
| Extraction method | `git diff` against the file's own last commit — content below is copied **verbatim** from the `+` lines of that diff; no code was rewritten, reformatted, or paraphrased |

**This is diagnostic/experimental code only. It has never been built into a flashed production binary in this project's provenance chain (`c55853a` and every ancestor lack it entirely — confirmed absent) and must not be treated as a production firmware source.**

---

## What problem this was built to investigate

The RC file's own commit message ("TLS handshake investigation candidate") and this diagnostic's own comments identify the target precisely: **raw AT command/response traffic exchanged with the SIMCom modem, across three distinct code paths, at the moment of TCP connect / TLS handshake / MQTT connect** —

1. **TinyGSM's own AT engine** (GPRS connect, signal quality, `modemGetConnected()`/`modemGetAvailable()`, etc.) — captured by wrapping the modem's serial port in `ATTraceStream`, which intercepts every byte TinyGSM itself reads/writes.
2. **This firmware's hand-rolled TLS payload fetch path** — `_ciprxget()`/`_atReadInt()`/`_drainOK()` — which talks to `SerialAT` *directly*, bypassing TinyGSM's engine entirely, and is therefore invisible to the `ATTraceStream` wrapper. This path is separately instrumented at its own call sites, in the same `[millis()] >>>/<<<` format, so no AT traffic is silently missed regardless of which path it travels.
3. **MQTT connect**, logged as a bounding `=== MQTT CONNECT === / === MQTT CONNECT RESULT ===` pair around `mqttClient.connect()`.

The RC file also contains a second, **explicitly independent** diagnostic — `DEBUG_MQTT_TIMING`/`dbgLogMqttPublish()`, a "[Phase 11] MQTT PUBLISH TIMING" tool wired into 8 separate `mqttClient.publish()` call sites. Its own comment states it is *"Independent of every earlier debug flag (DEBUG_TLS/DEBUG_MODEM/DEBUG_UART_HEX/DEBUG_AT_RAW)."* **It is out of scope for this record and deliberately excluded below** — this document preserves only the `ATTraceStream`/`DEBUG_AT_RAW` material specifically requested.

---

## Configuration comment (module scope, `DEBUG_AT_RAW` toggle)

```cpp
// ============================================================================
// [AT trace] Raw AT command/response capture -- OFF by default, zero behavior
// change. Independent of DEBUG_TLS/DEBUG_MODEM above (different, complementary
// tool: this shows the literal bytes exchanged with the modem; DEBUG_TLS/
// DEBUG_MODEM show this firmware's own higher-level interpretation of them).
//   DEBUG_AT_RAW -- logs every ">>> " (sent) / "<<< " (received) line, each
//                   prefixed "[millis()] ", plus "=== STAGE ===" markers
//                   around TCP connect / TLS handshake / MQTT connect.
// NOTE ON SCOPE: TinyGSM's own AT engine (GPRS connect, signal quality,
// modemGetConnected()/modemGetAvailable(), etc.) is captured via the
// ATTraceStream wrapper below, which intercepts every byte TinyGSM itself
// reads/writes. This firmware's OWN hand-rolled _ciprxget()/_atReadInt()/
// _drainOK() (the TLS payload fetch path) talk to SerialAT DIRECTLY,
// bypassing TinyGSM's engine entirely (see V2.1/code_review_v2.md history)
// -- those are separately logged at their own call sites below, in the same
// format, so no AT traffic is silently missed.
// ============================================================================
// #define DEBUG_AT_RAW
```

## Class definition

```cpp
#ifdef DEBUG_AT_RAW
// Minimal Stream pass-through wrapper. Deliberately NOT the "StreamDebugger"
// Arduino library: that library is not vendored anywhere in this project (a
// grep of the whole repo confirms it), so depending on it would add an
// unvetted new build dependency; it also does not timestamp its own output,
// which requirement 3 (millis() on every line) needs. This class is a pure,
// transparent pass-through -- every call forwards to the real stream and
// changes nothing about what is sent/received or when; it only observes.
class ATTraceStream : public Stream {
public:
  explicit ATTraceStream(Stream& real) : _real(real) {}

  int available() override { return _real.available(); }
  int peek() override { return _real.peek(); }
  void flush() override { _real.flush(); }

  int read() override {
    int c = _real.read();
    if (c >= 0) _logByte(_rxLine, _rxLen, (char)c, "<<<");
    return c;
  }
  size_t write(uint8_t b) override {
    _logByte(_txLine, _txLen, (char)b, ">>>");
    return _real.write(b);
  }
  size_t write(const uint8_t* buf, size_t len) override {
    for (size_t i = 0; i < len; i++) write(buf[i]);
    return len;
  }

private:
  Stream& _real;
  static const size_t LINE_MAX = 128;
  char   _txLine[LINE_MAX]; size_t _txLen = 0;
  char   _rxLine[LINE_MAX]; size_t _rxLen = 0;

  void _logByte(char* buf, size_t& len, char c, const char* dir) {
    if (c == '\n' || len >= LINE_MAX - 1) {
      buf[len] = '\0';
      if (len > 0) Serial.printf("[%lu] %s %s\n", (unsigned long)millis(), dir, buf);
      len = 0;
    } else if (c != '\r') {
      buf[len++] = c;
    }
  }
};
#endif
```

## Integration call sites (13 total)

**1. Modem object construction** — swaps the wrapped stream in when the flag is set:
```cpp
#ifdef DEBUG_AT_RAW
ATTraceStream g_atTrace(SerialAT);
TinyGsm modem(g_atTrace);
#else
TinyGsm modem(SerialAT);
#endif
```

**2. TCP connect start marker:**
```cpp
#ifdef DEBUG_AT_RAW
    Serial.printf("[%lu] === START TCP CONNECT ===\n", (unsigned long)millis());
#endif
```

**3. TCP connect failure diagnostic dump** (failure-path-only, bounded cost):
```cpp
#ifdef DEBUG_AT_RAW
      // [Requirement 5] Diagnostic dump on TCP connect failure. All of these
      // are read-only status queries fired ONLY on this (already-rare)
      // failure path, not in any loop -- each is a real AT exchange with a
      // real, bounded cost, acceptable exactly because it's failure-path-only.
      // "client.getLastError()" has no equivalent at this layer: TinyGsmClient
      // exposes no such method (that concept only exists one layer up, on
      // MQTTClient::lastError(), which this class has no reference to) --
      // reporting that plainly rather than fabricating a call that isn't there.
      Serial.printf("[%lu] === TCP CONNECT FAILED -- diagnostic dump ===\n", (unsigned long)millis());
      Serial.printf("[%lu]   _tcp.connected()      = %u\n", (unsigned long)millis(), (unsigned)_tcp.connected());
      Serial.printf("[%lu]   client.getLastError() = not available on TinyGsmClient (no such method at this layer)\n", (unsigned long)millis());
      Serial.printf("[%lu]   TinyGSM mux (socket)  = %d\n", (unsigned long)millis(), 0);
      Serial.printf("[%lu]   GPRS connected        = %u\n", (unsigned long)millis(), (unsigned)_modem.isGprsConnected());
      Serial.printf("[%lu]   Local IP              = %s\n", (unsigned long)millis(), _modem.localIP().toString().c_str());
      Serial.printf("[%lu]   Signal quality (CSQ)  = %d\n", (unsigned long)millis(), _modem.getSignalQuality());
      Serial.printf("[%lu]   Operator              = %s\n", (unsigned long)millis(), _modem.getOperator().c_str());
      Serial.printf("[%lu] === END diagnostic dump ===\n", (unsigned long)millis());
#endif
```

**4. TCP connect done / TLS handshake start markers:**
```cpp
#ifdef DEBUG_AT_RAW
    Serial.printf("[%lu] === TCP CONNECT DONE ===\n", (unsigned long)millis());
    Serial.printf("[%lu] === START TLS HANDSHAKE ===\n", (unsigned long)millis());
#endif
```

**5. TLS handshake done marker:**
```cpp
#ifdef DEBUG_AT_RAW
    Serial.printf("[%lu] === TLS HANDSHAKE DONE ===\n", (unsigned long)millis());
#endif
```

**6. `_ciprxget` step 1 — query bytes available (request):**
```cpp
#ifdef DEBUG_AT_RAW
    Serial.printf("[%lu] >>> AT+CIPRXGET=4,0\n", (unsigned long)millis());
#endif
```

**7. `_ciprxget` step 1 — parsed response:**
```cpp
#ifdef DEBUG_AT_RAW
    Serial.printf("[%lu] <<< (parsed) +CIPRXGET: mode=4 count=%d\n", (unsigned long)millis(), modemCount);
#endif
```

**8. `_ciprxget` step 2 — fetch bytes (request):**
```cpp
#ifdef DEBUG_AT_RAW
    Serial.printf("[%lu] >>> AT+CIPRXGET=2,0,%d\n", (unsigned long)millis(), fetch);
#endif
```

**9. `_ciprxget` step 2 — parsed response:**
```cpp
#ifdef DEBUG_AT_RAW
    Serial.printf("[%lu] <<< (parsed) +CIPRXGET: mode=2 actual=%d\n", (unsigned long)millis(), actual);
#endif
```

**10. `_atReadInt()` reader-loop line logging** (every line scanned, matched or not):
```cpp
#ifdef DEBUG_AT_RAW
          // [Requirement 2] Log every line scanned here, matched or not --
          // "do not suppress anything" -- since this is the raw-UART reader
          // path TinyGSM's own engine never sees (see the module-level
          // DEBUG_AT_RAW comment).
          if (llen > 0) Serial.printf("[%lu] <<< %s\n", (unsigned long)millis(), line);
#endif
```

**11. `_atReadInt()` timeout-with-no-match logging:**
```cpp
#ifdef DEBUG_AT_RAW
    Serial.printf("[%lu] <<< (timeout after %lums, no '%s' match)\n",
                  (unsigned long)millis(), (unsigned long)timeoutMs, prefix);
#endif
```

**12. MQTT connect start marker:**
```cpp
#ifdef DEBUG_AT_RAW
          Serial.printf("[%lu] === MQTT CONNECT ===\n", (unsigned long)millis());
#endif
```

**13. MQTT connect result marker:**
```cpp
#ifdef DEBUG_AT_RAW
          Serial.printf("[%lu] === MQTT CONNECT RESULT: %d ===\n", (unsigned long)millis(), (int)connected);
#endif
```

---

## Facts recorded for the future

- **`ATTraceStream` and `DEBUG_AT_RAW` do not exist anywhere else in this repository.** Confirmed by a repository-wide grep (excluding vendor trees) at extraction time — zero other matches, in production, any other RC/backup/worktree copy, or any prior evidence document.
- **The underlying TLS production issue this diagnostic was built to chase has already been resolved**, via two separate, already-committed mainline fixes:
  - `f07cb8a` — `fix(tls): remove false _tcp.connected() abort during TLS handshake`
  - `6650776` — `fix(tls): remove redundant modem connected check`
  Both are ancestors of current production HEAD (`c55853a`). The fix landed through direct root-cause correction, not through use of this trace tool's captured output (no evidence ties this diagnostic's captured data to either fix's commit).
- **This diagnostic implementation is preserved for future investigation only.** It has no open task or defect tied to it currently, and is not scheduled for use.
- **It must NOT be treated as a production firmware source, built, or flashed as-is** — it lives only in a stale RC snapshot (see `claude/docs/evidence` sibling record on `Commit8_CurrentEvidenceFix_RC`'s overall disposition), predates S21/current-freshness/NTP/`[v16.5g-j]` entirely, and was never part of any verified production baseline.
