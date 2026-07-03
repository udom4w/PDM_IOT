// ============================================================================
// FAULT LATCH v3 — COMPLETE PRODUCTION CODE
// Target: WTVB02_ESP32S3_V16_CM.ino
// All symbols verified against actual V16 source.
// ============================================================================


// ============================================================================
// SECTION 1 — CONSTANTS
// Insert after: #define NVS_SAVE_INTERVAL_MS  30000UL
// ============================================================================

#define FL_NS           "fault_latch"
#define FL_KEY_CODE     "ev_code"
#define FL_KEY_TS       "ev_ts"
#define FL_KEY_RMS      "ev_rms"
#define FL_KEY_KURT     "ev_kurt"
#define FL_KEY_PENDING  "ev_pending"
#define FL_KEY_MAGIC    "ev_magic"
#define FL_MAGIC_VALUE  0xF401A7CDUL
#define FL_EVT_NONE      0u
#define FL_EVT_WARNING   1u
#define FL_EVT_CRITICAL  2u
#define FL_EVT_BEARING   3u
#define FL_EVT_HEALTH    4u
#define FL_EVT_MAX_VALID 4u
#define FL_SEV_NONE     0u
#define FL_SEV_WARNING  1u
#define FL_SEV_HEALTH   2u
#define FL_SEV_CRITICAL 3u
#define FL_SEV_BEARING  4u
#define FL_HEALTH_LOW_THOLD  30
#define FL_TS_MIN_VALID  1577836800UL
#define FL_TS_UNKNOWN    0UL


// ============================================================================
// SECTION 2 — STRUCT
// Insert after SECTION 1 constants.
// MANDATORY FIX applied: volatile bool pending
// ============================================================================

struct FaultLatch_t {
  uint32_t      ts;
  float         rms;
  float         kurtosis;
  uint8_t       code;
  volatile bool pending;
};


// ============================================================================
// SECTION 3 — GLOBALS
// Insert after SECTION 2 struct.
// ============================================================================

static FaultLatch_t g_fl = { FL_TS_UNKNOWN, 0.0f, 0.0f, FL_EVT_NONE, false };
static volatile uint32_t g_flCount = 0u;
static bool           g_flPrevBearing   = false;
static bool           g_flPrevHealthLow = false;
static MachineState_t g_flPrevState     = STATE_NORMAL;


// ============================================================================
// SECTION 4 — faultSeverity()
// Insert after: closing brace of saveRuntimeHour() function
// ============================================================================

static uint8_t faultSeverity(uint8_t code) {
  switch (code) {
    case FL_EVT_BEARING:  return FL_SEV_BEARING;
    case FL_EVT_CRITICAL: return FL_SEV_CRITICAL;
    case FL_EVT_HEALTH:   return FL_SEV_HEALTH;
    case FL_EVT_WARNING:  return FL_SEV_WARNING;
    default:              return FL_SEV_NONE;
  }
}


// ============================================================================
// SECTION 5 — faultEventStr()
// Insert after: faultSeverity()
// ============================================================================

static const char* faultEventStr(uint8_t code) {
  switch (code) {
    case FL_EVT_BEARING:  return "BEARING_CONFIRMED";
    case FL_EVT_CRITICAL: return "ALARM_CRITICAL";
    case FL_EVT_HEALTH:   return "HEALTH_LOW";
    case FL_EVT_WARNING:  return "ALARM_WARNING";
    default:              return "UNKNOWN";
  }
}


// ============================================================================
// SECTION 6 — saveFaultLatchNVS()
// Insert after: faultEventStr()
// ============================================================================

static void saveFaultLatchNVS() {
  Preferences p;
  p.begin(FL_NS, false);
  p.putUChar(FL_KEY_PENDING, 0u);              // Step 1: disarm
  p.putUChar(FL_KEY_CODE,    g_fl.code);       // Step 2: data
  p.putUInt (FL_KEY_TS,      g_fl.ts);
  p.putFloat(FL_KEY_RMS,     g_fl.rms);
  p.putFloat(FL_KEY_KURT,    g_fl.kurtosis);
  p.putUInt (FL_KEY_MAGIC,   FL_MAGIC_VALUE);  // Step 3: commit marker
  p.putUChar(FL_KEY_PENDING, 1u);              // Step 4: arm
  p.end();
  Serial.printf("[LATCH] SAVE code=%u(%s) sev=%u ts=%lu rms=%.2f kurt=%.3f n=%lu\n",
                g_fl.code, faultEventStr(g_fl.code), faultSeverity(g_fl.code),
                (unsigned long)g_fl.ts, g_fl.rms, g_fl.kurtosis,
                (unsigned long)g_flCount);
}


// ============================================================================
// SECTION 7 — loadFaultLatchNVS()
// Insert after: saveFaultLatchNVS()
// ============================================================================

static void loadFaultLatchNVS() {
  Preferences p;
  p.begin(FL_NS, true);
  uint8_t  pending = p.getUChar(FL_KEY_PENDING, 0u);
  uint32_t magic   = p.getUInt (FL_KEY_MAGIC,   0u);
  uint8_t  code    = p.getUChar(FL_KEY_CODE,    FL_EVT_NONE);
  uint32_t ts      = p.getUInt (FL_KEY_TS,      0xFFFFFFFFUL);
  float    rms     = p.getFloat(FL_KEY_RMS,     -1.0f);
  float    kurt    = p.getFloat(FL_KEY_KURT,    -1.0f);
  p.end();

  if (!pending) {
    Serial.println("[LATCH] RECOVER — no pending latch");
    return;
  }
  if (magic != FL_MAGIC_VALUE) {
    Serial.printf("[LATCH] CORRUPT — magic=0x%08lX expected=0x%08lX\n",
                  (unsigned long)magic, (unsigned long)FL_MAGIC_VALUE);
    goto fl_discard;
  }
  if (code == FL_EVT_NONE || code > FL_EVT_MAX_VALID) {
    Serial.printf("[LATCH] CORRUPT — code=%u out of [1,%u]\n",
                  code, FL_EVT_MAX_VALID);
    goto fl_discard;
  }
  if (ts == 0xFFFFFFFFUL) {
    Serial.println("[LATCH] CORRUPT — ts=0xFFFFFFFF (erased cell)");
    goto fl_discard;
  }
  if (ts != FL_TS_UNKNOWN && ts < FL_TS_MIN_VALID) {
    Serial.printf("[LATCH] CORRUPT — ts=%lu below min valid\n", (unsigned long)ts);
    goto fl_discard;
  }
  if (rms != rms || rms < 0.0f || rms > 100.0f) {
    Serial.printf("[LATCH] CORRUPT — rms=%.3f implausible\n", rms);
    goto fl_discard;
  }
  if (kurt != kurt || kurt < 0.0f || kurt > 1000.0f) {
    Serial.printf("[LATCH] CORRUPT — kurt=%.3f implausible\n", kurt);
    goto fl_discard;
  }

  g_fl.code     = code;
  g_fl.ts       = ts;
  g_fl.rms      = rms;
  g_fl.kurtosis = kurt;
  g_fl.pending  = true;
  Serial.printf("[LATCH] RECOVER OK — code=%u(%s) sev=%u ts=%lu rms=%.2f kurt=%.3f\n",
                code, faultEventStr(code), faultSeverity(code),
                (unsigned long)ts, rms, kurt);
  return;

fl_discard:
  {
    Preferences pe;
    pe.begin(FL_NS, false);
    pe.clear();
    pe.end();
  }
  Serial.println("[LATCH] CORRUPT — namespace erased, system continues normally");
}


// ============================================================================
// SECTION 8 — clearFaultLatchNVS()
// Insert after: loadFaultLatchNVS()
// ============================================================================

static void clearFaultLatchNVS() {
  Preferences p;
  p.begin(FL_NS, false);
  p.putUChar(FL_KEY_PENDING, 0u);
  p.putUInt (FL_KEY_MAGIC,   0u);
  p.end();
  g_fl.pending = false;
  Serial.println("[LATCH] CLEAR — delivered, NVS latch released");
}


// ============================================================================
// SECTION 9 — checkAndLatchFault()
// Insert after: clearFaultLatchNVS()
// ============================================================================

static void checkAndLatchFault(const VibrationData_t* data,
                                MachineState_t         newState,
                                int                    healthScore,
                                bool                   bearingConfirmed)
{
  const bool healthLowNow = (healthScore <= FL_HEALTH_LOW_THOLD);

  uint8_t evCode = FL_EVT_NONE;

  if (bearingConfirmed && !g_flPrevBearing) {
    evCode = FL_EVT_BEARING;
  } else if (newState == STATE_CRITICAL &&
             g_flPrevState != STATE_CRITICAL) {
    evCode = FL_EVT_CRITICAL;
  } else if (healthLowNow && !g_flPrevHealthLow) {
    evCode = FL_EVT_HEALTH;
  } else if (newState == STATE_WARNING &&
             g_flPrevState == STATE_NORMAL) {
    evCode = FL_EVT_WARNING;
  }

  // Update trackers UNCONDITIONALLY — must run even when no event fires.
  g_flPrevBearing   = bearingConfirmed;
  g_flPrevHealthLow = healthLowNow;
  g_flPrevState     = newState;

  if (evCode == FL_EVT_NONE) return;

  if (g_fl.pending) {
    const uint8_t inSev = faultSeverity(evCode);
    const uint8_t exSev = faultSeverity(g_fl.code);
    if (inSev <= exSev) {
      Serial.printf("[LATCH] SKIP ev=%u(%s) sev=%u — pending ev=%u(%s) sev=%u\n",
                    evCode, faultEventStr(evCode), inSev,
                    g_fl.code, faultEventStr(g_fl.code), exSev);
      return;
    }
    Serial.printf("[LATCH] OVERWRITE pending ev=%u sev=%u -> ev=%u sev=%u\n",
                  g_fl.code, exSev, evCode, inSev);
  }

  uint32_t epochNow = FL_TS_UNKNOWN;
  if (g_rtcValid && g_timeSync.synced) {
    const uint32_t cand = rtc.now().unixtime();
    if (cand >= FL_TS_MIN_VALID) {
      epochNow = cand;
    } else {
      Serial.printf("[LATCH] ts candidate=%lu < min — storing UNKNOWN\n",
                    (unsigned long)cand);
    }
  } else {
    Serial.printf("[LATCH] ts=UNKNOWN (rtcValid=%d synced=%d)\n",
                  (int)g_rtcValid, (int)g_timeSync.synced);
  }

  g_fl.code     = evCode;
  g_fl.ts       = epochNow;
  g_fl.rms      = data->rms_overall;
  g_fl.kurtosis = data->kurtosis_max;
  g_flCount++;

  saveFaultLatchNVS();  // issues MEMW barrier via Preferences::end()

  g_fl.pending = true;  // set LAST — after NVS write and barrier

  Serial.printf("[LATCH] LATCHED ev=%u(%s) sev=%u ts=%lu rms=%.2f kurt=%.3f n=%lu\n",
                evCode, faultEventStr(evCode), faultSeverity(evCode),
                (unsigned long)epochNow,
                data->rms_overall, data->kurtosis_max,
                (unsigned long)g_flCount);
}


// ============================================================================
// SECTION 10 — setup() MODIFICATION
// Find:   loadRuntimeHour();
// After that line, add:
// ============================================================================

//  loadRuntimeHour();                                       // existing line
//  loadFaultLatchNVS();   // v3: restore pending fault event from previous session


// ============================================================================
// SECTION 11 — taskStateMachine() MODIFICATION
//
// Find the end of taskStateMachine(), specifically this block:
//
//         xSemaphoreGive(mutexSystemState);
//       }
//     }   <- closes: if (xQueueReceive(queueSensorData, &sensorData, portMAX_DELAY) == pdPASS)
//   }     <- closes: while (1)
// }       <- closes: void taskStateMachine(void* parameter)
//
// Replace those 5 lines with the block below.
// ============================================================================

/*
        xSemaphoreGive(mutexSystemState);
      }

      // ── Fault Latch v3: evaluate fault transitions ────────────────────────
      {
        int latchHealth = 100;
        if (sensorData.motor_state == 2 && sensorData.rms_overall > BASELINE_RMS) {
          float norm = (sensorData.rms_overall - BASELINE_RMS) /
                       (CRITICAL_RMS - BASELINE_RMS) * 100.0f;
          latchHealth = (int)max(0.0f, min(100.0f, roundf(100.0f - norm)));
        }
        const bool latchBearing =
          (sensorData.motor_state == 2) &&
          (g_bearingStableCnt >= BEARING_STABLE_CYCLES) &&
          (sensorData.kurtosis_max >= KURTOSIS_CONFIRMED);

        checkAndLatchFault(&sensorData, newState, latchHealth, latchBearing);
      }
      // ── End Fault Latch ───────────────────────────────────────────────────
    }
  }
}
*/


// ============================================================================
// SECTION 12 — taskNetwork() REPLAY BLOCK
//
// Find this line in taskNetwork() (~line 3035):
//   // -- 100ms sleep -- ให้ FreeRTOS scheduler ทำงาน tasks อื่น --
//
// Insert the following block BEFORE that line.
// ============================================================================

/*
    // ── Fault Latch v3: replay pending event on /status ──────────────────────
    if (mqttClient.connected() && g_fl.pending) {

      char flTs[26] = {};
      const bool flTsKnown = (g_fl.ts >= FL_TS_MIN_VALID);
      if (flTsKnown) {
        DateTime flEv(g_fl.ts);
        snprintf(flTs, sizeof(flTs), "%04d-%02d-%02dT%02d:%02d:%02dZ",
                 flEv.year(), flEv.month(), flEv.day(),
                 flEv.hour(), flEv.minute(), flEv.second());
      }

      int         snapAlarmCode  = 0;
      const char* snapAlarmLevel = "NORMAL";
      int         snapHealth     = 100;
      {
        VibrationData_t snapVib = {};
        MachineState_t  snapState = STATE_NORMAL;
        if (xSemaphoreTake(mutexVibData, pdMS_TO_TICKS(5)) == pdTRUE) {
          memcpy(&snapVib, &g_vibData, sizeof(VibrationData_t));
          xSemaphoreGive(mutexVibData);
        }
        if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(5)) == pdTRUE) {
          snapState = g_systemState.state;
          xSemaphoreGive(mutexSystemState);
        }
        if (snapVib.motor_state == 2) {
          snapAlarmCode  = (snapState == STATE_CRITICAL) ? 2 :
                           (snapState == STATE_WARNING)  ? 1 : 0;
          snapAlarmLevel = (snapAlarmCode == 2) ? "CRITICAL" :
                           (snapAlarmCode == 1) ? "WARNING"  : "NORMAL";
          float flNorm = (snapVib.rms_overall - BASELINE_RMS) /
                         (CRITICAL_RMS - BASELINE_RMS) * 100.0f;
          snapHealth = (int)max(0.0f, min(100.0f, roundf(100.0f - flNorm)));
        }
      }

      char flNowTs[26] = "not_available";
      if (g_rtcValid) {
        DateTime rtcNow = rtc.now();
        snprintf(flNowTs, sizeof(flNowTs), "%04d-%02d-%02dT%02d:%02d:%02dZ",
                 rtcNow.year(), rtcNow.month(), rtcNow.day(),
                 rtcNow.hour(), rtcNow.minute(), rtcNow.second());
      }

      StaticJsonDocument<1024> flDoc;
      flDoc["plant"]              = PLANT_ID;
      flDoc["machine_id"]         = MACHINE_ID;
      flDoc["sensor_id"]          = SENSOR_ID;
      flDoc["stage"]              = "status";
      flDoc["execution_location"] = "edge";
      flDoc["alarm_code"]         = snapAlarmCode;
      flDoc["alarm_level"]        = snapAlarmLevel;
      flDoc["health_score"]       = snapHealth;
      flDoc["timestamp"]          = flNowTs;
      flDoc["fault_latch_pending"]= true;
      flDoc["fault_event"]        = faultEventStr(g_fl.code);
      flDoc["fault_severity"]     = faultSeverity(g_fl.code);
      if (flTsKnown) {
        flDoc["fault_ts"]         = g_fl.ts;
        flDoc["fault_ts_iso"]     = flTs;
        flDoc["fault_ts_unknown"] = false;
      } else {
        flDoc["fault_ts"]         = (uint32_t)0;
        flDoc["fault_ts_iso"]     = (char*)nullptr;
        flDoc["fault_ts_unknown"] = true;
      }
      flDoc["fault_rms"]          = roundf(g_fl.rms      * 100.0f)  / 100.0f;
      flDoc["fault_kurtosis"]     = roundf(g_fl.kurtosis * 1000.0f) / 1000.0f;
      flDoc["fault_latch_count"]  = g_flCount;

      char   flBuf[1024];
      size_t flSz = serializeJson(flDoc, flBuf, sizeof(flBuf));
      if (flSz == 0 || flSz >= sizeof(flBuf) - 1) {
        Serial.printf("[LATCH] JSON overflow sz=%u — skipping\n", (unsigned)flSz);
      } else {
        if (mqttClient.publish(g_mqttTopicDecision, flBuf, (int)flSz,
                               false, MQTT_QOS)) {
          g_network.publishCount++;
          clearFaultLatchNVS();
          Serial.printf("[LATCH] PUBLISH+CLEAR %s\n", flBuf);
        } else {
          g_network.publishFailures++;
          Serial.println("[LATCH] PUBLISH failed — retry next loop");
        }
      }
    }
    // ── End Fault Latch replay ────────────────────────────────────────────────
*/


// ============================================================================
// SECTION 13 — publishTelemetry() /status BLOCK REPLACEMENT
//
// Find PUBLISH 2 of 3 /status block in publishTelemetry().
// Replace the ENTIRE block with this. Changes: size 512->768, 2 new fields.
// ============================================================================

/*
  // ──────────────────────────────────────────────────────────────────────────
  // PUBLISH 2 of 3 — /status  (alarm + health, CM V1)
  // v3: StaticJsonDocument 512->768, char buf 512->768, added latch diagnostics
  // ──────────────────────────────────────────────────────────────────────────
  {
    StaticJsonDocument<768> d;
    d["plant"]               = PLANT_ID;
    d["machine_id"]          = MACHINE_ID;
    d["sensor_id"]           = SENSOR_ID;
    d["stage"]               = "status";
    d["execution_location"]  = "edge";

    d["alarm_code"]          = alarmCode;
    d["alarm_level"]         = alarmLevel;
    d["health_score"]        = healthScore;

    d["bearing_alert"]              = bearingAlert;
    d["kurtosis_max"]               = kmax;
    d["kurtosis_axis"]              = kaxis;
    d["kurtosis_valid"]             = kurtosisValid;
    d["dominant_vibration_axis"]    = domVibAxis;

    d["freq_alert"]          = freqGateOpen && g_trendResult.freq_alert;
    d["freq_drift_x"]        = freqGateOpen ? g_trendResult.freq_drift_x : 0.0f;
    d["freq_drift_y"]        = freqGateOpen ? g_trendResult.freq_drift_y : 0.0f;
    d["freq_drift_z"]        = freqGateOpen ? g_trendResult.freq_drift_z : 0.0f;

    d["trend_dir"]           = trendDirStr;
    d["rms_slope"]           = g_trendResult.rms_slope;
    d["spike_count"]         = g_trendResult.spike_count;
    if (g_trendResult.ttw_hours > 0.0f)
      d["ttw_estimate_h"]    = g_trendResult.ttw_hours;

    d["timestamp"]           = tsBuf;

    d["fault_latch_pending"] = (bool)g_fl.pending;
    d["fault_latch_count"]   = g_flCount;

    char buf[768];
    size_t szStatus = serializeJson(d, buf, sizeof(buf));
    if (szStatus == 0 || szStatus >= sizeof(buf) - 1)
      Serial.printf("[WARN] /status JSON truncated! sz=%u\n", (unsigned)szStatus);
    if (mqttClient.publish(g_mqttTopicDecision, buf, (int)szStatus, false, MQTT_QOS))
      Serial.printf("[MQTT] /status %u B\n", (unsigned)szStatus);
    else
      Serial.printf("[MQTT] /status FAILED (err=%d)\n", mqttClient.lastError());
  }
*/


// ============================================================================
// SECTION 14 — JSON CAPACITY REFERENCE
// ============================================================================

// /status regular (SECTION 13):
//   24 fields x 8B/slot + 26B pool (tsBuf) = 218B needed
//   StaticJsonDocument<768>: 550B headroom  PASS
//   char buf[768]: output ~531B             PASS
//
// /status replay (SECTION 12):
//   18 fields x 8B/slot + 52B pool (flTs+flNowTs) = 196B needed
//   StaticJsonDocument<1024>: 828B headroom PASS
//   char buf[1024]: output ~425B            PASS
//
// Stack impact (taskNetwork, 24576B allocated):
//   Section 12 block: 1024 + 1024 = 2048B transient    PASS
//   Section 13 block: 768 + 768 = 1536B (vs 1024 before): +512B delta  PASS
//
// MANDATORY FIX (apply to base firmware before compiling):
//   Line ~1399: static uint8_t g_bearingStableCnt = 0;
//   Change to:  static volatile uint8_t g_bearingStableCnt = 0;
