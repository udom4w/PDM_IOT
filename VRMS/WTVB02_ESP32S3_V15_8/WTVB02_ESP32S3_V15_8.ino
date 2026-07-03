/*
 * ============================================================================
 * Industrial Vibration Monitoring System - Dual Core FreeRTOS
 * ============================================================================
 * Hardware: LilyGO T-Vending S3 (ESP32-S3, 16MB Flash)
 * Modem: SIMCom A7670 (SIM7600 Compatible) - 4G LTE
 * RTC: DS3231
 * 
 * Core Assignment Strategy:
 * +--------------------------------------------------------------+
 * | CORE 0 (PRO_CPU) - Time-Critical Tasks                      |
 * |  - Modbus RTU Communication (highest priority)              |
 * |  - Sensor Data Processing                                   |
 * |  - State Machine & Safety Logic                             |
 * +--------------------------------------------------------------+
 * | CORE 1 (APP_CPU) - User Interface & Network                 |
 * |  - OLED Display Rendering                                   |
 * |  - 4G Modem/MQTT Communication                              |
 * |  - Button Input Handling                                    |
 * |  - Buzzer Control                                           |
 * +--------------------------------------------------------------+
 * 
 * Features:
 * - 4G LTE connectivity via SIMCom A7670
 * - MQTT over mTLS (port 8883, client certificate auth)
 * - RTC DS3231 for accurate timestamps
 * - Automatic NTP time sync via 4G GSM network time
 * - Periodic drift detection and auto-correction
 * - Zero blocking operations
 * - Task-based architecture
 * - Queue-based inter-task communication
 * - Mutex protection for shared resources
 * - Optimized Modbus read (5 transactions: VEL+TEMP+FREQ+CFX/KX+CFY/KY+CFZ/KZ)
 * - Vy stuck detection & auto sensor restart via Modbus
 * 
 * Author: Senior Real-Time Embedded Engineer
 * Version: 15.8 (pdm_accuracy_fix_patch)
 *
 * v15.8 Changes (pdm_accuracy_fix_patch):
 * +--------------------------------------------------------------------+
 * | Fix #1: True 3-axis vector RMS (ISO 10816 / ISO 20816 compliant)   |
 * |   Before: rms_overall = max(rms_x, rms_y, rms_z)                  |
 * |           → underestimates combined energy by up to 42%            |
 * |   After:  rms_overall = sqrt(rms_x²+rms_y²+rms_z²)                |
 * |           → true vector magnitude                                  |
 * |   Thresholds BASELINE=2.1/WARNING=4.5/CRITICAL=7.1 unchanged:      |
 * |   ISO 10816-3 already specifies them as vector RMS.                |
 * |                                                                    |
 * | Fix #2: Dominant-axis freq_ratio (preserves fault signatures)      |
 * |   Before: ratioMean = (frx+fry+frz)/3 → destroys axis patterns    |
 * |   After:  ratioDom  = max(frx, fry, frz)                           |
 * |           ratioRadial = max(frx, fry) for imbalance                |
 * |           ratioAxial  = frz with 1.3× boost for misalignment       |
 * |   classifyFaultProbabilistic() now accepts all 3 axis ratios.      |
 * |                                                                    |
 * | Fix #3: Bearing score decoupled from velocity spike_count          |
 * |   Before: 0.35×spk + 0.25×kurt + 0.20×cf + 0.10×nonI + 0.10×vol  |
 * |           spk≈0 always (velocity RMS cannot capture impulses)      |
 * |   After:  0.55×kurtNorm + 0.30×cfNorm + 0.15×vol                  |
 * |           kurtNorm = clamp((kurt-3)/3, 0,1) baseline=3.0           |
 * |           cfNorm   = clamp((CF-2.5)/4.5, 0,1)                      |
 * |           Bearing kurtosis path runs even when ratioDom=0           |
 * |           (bearing damage does not require a dominant harmonic)    |
 * +--------------------------------------------------------------------+
 *
 * v15.7 Changes (vrms_direct_read_patch):
 * +--------------------------------------------------------------------+
 * | Root cause A: VX/VY/VZ (0x3A~0x3C) ส่งค่า Peak amplitude ไม่ใช่  |
 * |   RMS โค้ดเดิมแปลง peak→RMS เอง (×0.7071) ซึ่งเป็น approximation |
 * |   สำหรับ sinusoidal เท่านั้น                                        |
 * |   Datasheet §6.4.14~16: sensor มี VRMSX/Y/Z register แยกต่างหาก   |
 * |   คำนวณจาก raw FIFO 16KHz ภายใน chip -- ถูกต้องกว่า               |
 * |                                                                    |
 * | Fix A: เปลี่ยน Transaction 1 อ่าน VRMS โดยตรงจาก sensor           |
 * |   VRMSX: reg 0x50 → rms_x (mm/s) = raw / 1000                    |
 * |   VRMSY: reg 0x5C → rms_y (mm/s) = raw / 1000                    |
 * |   VRMSZ: reg 0x68 → rms_z (mm/s) = raw / 1000                    |
 * |   หมายเหตุ: address ไม่ต่อเนื่อง → อ่านแยก 3 transactions         |
 * |                                                                    |
 * | ลบออก: vel_peak_x/y/z/overall, g_velPeakHold, peak field          |
 * |   ไม่มี peak register บน sensor -- ไม่ publish peak อีกต่อไป      |
 * |                                                                    |
 * | Root cause B: VRMS sensor noise floor ~0.5~1 mm/s ขณะ motor หยุด  |
 * |   ทำให้ rms ≠ 0 และ kurtosis พุ่งสูง (เช่น 33.6) ขณะ state=STOPPED|
 * |   เนื่องจากไม่มี mechanical damping ของ rotating mass             |
 * |                                                                    |
 * | Fix B: Motor state gate ใน taskModbusRead (Step 1)                |
 * |   motorActive = (MOTOR_RUNNING || MOTOR_STOPPING)                 |
 * |   !motorActive → rms_x/y/z/overall = 0.0, CF/Kurtosis = 0        |
 * |   ป้องกัน false alarm และ false fault classification               |
 * |   ขณะ state=STOPPED/STARTING                                       |
 * +--------------------------------------------------------------------+
 *
 * v15.6 Changes (cf_kurtosis_unsigned_fix):
 * +--------------------------------------------------------------------+
 * | Root cause: CF & Kurtosis = 0.000 ตลอด แม้ transaction สำเร็จ     |
 * |   Datasheet §6.4.14~16: register เป็น unsigned 16-bit (/1000)     |
 * |   โค้ดเดิม cast getResponseBuffer() เป็น int16_t                   |
 * |   ถ้า raw = 0 (sensor ยังไม่ warm-up) guard >0 ตัดออก → 0.0f     |
 * |   ถ้า raw > 32767 (CF สูง) wrap เป็น negative → guard ตัดออก      |
 * |                                                                    |
 * | Fix: เปลี่ยน declaration + cast เป็น uint16_t ทั้ง 6 ตัว          |
 * |   raw_cfx, raw_kx  (Transaction 3, REG_CFX 0x47~0x48)             |
 * |   raw_cfy, raw_ky  (Transaction 4, REG_CFY 0x53~0x54)             |
 * |   raw_cfz, raw_kz  (Transaction 5, REG_CFZ 0x5F~0x60)             |
 * |   Guard > 0 ยังคงไว้เพื่อป้องกัน transaction fail (raw stays 0)  |
 * +--------------------------------------------------------------------+
 *
 * v15.5 Changes (kurt_cf_decision_integration_patch):
 * +--------------------------------------------------------------------+
 * | R1: classifyFaultProbabilistic() — ส่ง kurtMax/cfMax เข้า scoring  |
 * |   FAULT_BEARING score (เดิม 3 terms → ใหม่ 5 terms):              |
 * |     เดิม: 0.50*spk + 0.30*nonI + 0.20*vol                        |
 * |     ใหม่: 0.35*spk + 0.25*kurtNorm + 0.20*cfNorm                 |
 * |            + 0.10*nonI + 0.10*vol  (sum=1.0 ✅)                   |
 * |     kurtNorm = clamp((kurtMax-3.0)/3.0, 0,1) -- baseline=3.0     |
 * |     cfNorm   = clamp((cfMax-2.5)/4.5, 0,1)   -- starts at CF>2.5 |
 * |                                                                    |
 * |   FAULT_LOOSENESS score (เพิ่ม cfLow):                            |
 * |     เดิม: 0.35*h3 + 0.30*sub + 0.35*vol                          |
 * |     ใหม่: 0.30*h3 + 0.25*sub + 0.30*vol + 0.15*cfLow (sum=1.0 ✅)|
 * |     cfLow = clamp(1-cfMax/4, 0,1) -- CF ต่ำ+vol สูง=looseness    |
 * |                                                                    |
 * |   FAULT_IMBALANCE, FAULT_MISALIGNMENT, FAULT_RESONANCE: ไม่เปลี่ยน|
 * |                                                                    |
 * | R2: runDecisionEngine() — อ่าน kurtMax/cfMax/motorState            |
 * |   - เพิ่ม kurtMax, cfMax, motorState ใน mutexVibData block        |
 * |   - Gate: motorState != 2 → kurtMax=cfMax=0 (transient protection) |
 * |   - ส่ง kurtMax/cfMax เข้า classifyFaultProbabilistic()           |
 * |                                                                    |
 * | R3: Type E conflict resolution + action_needed bearing gate        |
 * |   3a. Type E: kurtMax≥6 && motorState==2 && finalPred<1           |
 * |       → finalPred=1 (WARNING) เพื่อให้ action_needed ประเมินได้   |
 * |   3b. action_needed เพิ่ม:                                        |
 * |       bearingConfirmed = (kurtMax≥6 && motorState==2)             |
 * |       bearingEarlyWarn = (kurtMax≥4 && motorState==2              |
 * |                          && severity≥30 && fault==BEARING)        |
 * |                                                                    |
 * | R4: reason string เพิ่ม bearing cases (priority > freq_drift)     |
 * |     "bearing_confirmed+kurt=X.XX"                                 |
 * |     "bearing_early+kurt=X.XX+sc=Y.YY"                            |
 * +--------------------------------------------------------------------+
 *
 * v15.4 Changes (tls_wdt_reconnect_patch):
 * +--------------------------------------------------------------------+
 * | Root cause: TASK_WDT เพราะ taskNetwork ค้างใน reconnect loop       |
 * |   Sequence: MQTT publish fail → reconnect → TLS handshake ค้าง    |
 * |   > 30s → TASK_WDT fires → reboot (#40 ครั้ง!)                    |
 * |   Underlying: 4G IP เปลี่ยนบ่อย → TCP drop → mbedTLS context เก่า |
 * |   → decryption failed / bad record MAC ที่ broker                  |
 * |                                                                    |
 * | Fix A: esp_task_wdt_reset() ใน reconnect loop ทุก iteration       |
 * |   - เพิ่ม wdt reset ทุก 5 วินาที ระหว่างรอ backoff               |
 * |   - เพิ่ม wdt reset ก่อน/หลัง TLS connect attempt                 |
 * |   - ป้องกัน WDT ตอน reconnect ค้างนาน                            |
 * |                                                                    |
 * | Fix B: Full mbedTLS context reset ก่อน reconnect ทุกครั้ง         |
 * |   - เพิ่ม GsmTLSClient::resetTLS() -- free + reinit ssl + conf    |
 * |   - เรียก gsmClient.resetTLS() ก่อน mqttClient.connect()          |
 * |   - แก้ปัญหา stale cipher state จาก session เดิม                  |
 * |   - เดิม stop() reinit เฉพาะตอน TCP close ซึ่งช้าเกินไป         |
 * |                                                                    |
 * | Fix C: Exponential backoff สำหรับ MQTT reconnect                  |
 * |   - แทน fixed 30s retry → adaptive 30s→60s→120s→max 300s         |
 * |   - reset backoff เมื่อ connect สำเร็จ                           |
 * |   - ลด reconnect storm เมื่อ network unstable                     |
 * |   - แสดง next retry countdown ใน Serial log                       |
 * +--------------------------------------------------------------------+
 *
 * v15.3 Changes (reset_reason_logging_patch):
 * +--------------------------------------------------------------------+
 * | Feature: Reset Reason Logging                                      |
 * |   - เพิ่ม logResetReason() เรียกแรกสุดใน setup() หลัง Serial.begin |
 * |   - ใช้ esp_reset_reason() อ่านสาเหตุ reboot จาก ESP32 hardware   |
 * |   - สาเหตุที่ detect ได้:                                          |
 * |     POWER_ON     : เปิดไฟปกติ / กด EN button                      |
 * |     BROWNOUT ⚠️  : ไฟตกชั่วคราว (4G current spike → Vcc drop)    |
 * |     SW_RESET     : esp_restart() หรือ OTA                         |
 * |     TASK_WDT 🔴  : Task ค้างเกิน watchdog timeout (30s)           |
 * |     INT_WDT  🔴  : Interrupt watchdog (interrupt ค้าง)            |
 * |     PANIC    🔴  : Exception / Stack overflow / Assertion          |
 * |     EXT_PIN     : กดปุ่ม RESET                                    |
 * |   - เก็บ reset reason + reboot count ลง NVS (persistent)          |
 * |     reboot_count สะสมข้ามรอบ ไม่ reset เมื่อ power cycle         |
 * |   - Publish ใน /sensor payload: reset_reason + reboot_count       |
 * |   - ถ้า reason = BROWNOUT/WDT/PANIC → Serial warning ชัดเจน      |
 * +--------------------------------------------------------------------+
 *
 * v15.2 Changes (bearing_alert_gate_patch):
 * +--------------------------------------------------------------------+
 * | Fix 16: bearing_alert = CONFIRMED ขณะ motor หยุด (false alarm)     |
 * |   - Root cause: Kurtosis พุ่งสูงระหว่าง motor deceleration/stop    |
 * |     เพราะ impulse จาก shaft decel + noise floor ต่ำมาก (rms~0)    |
 * |     ทำให้ kurt_max = 19.77 ตอน state=STOPPING, rpm=0 → CONFIRMED   |
 * |     ซึ่งไม่ใช่ bearing fault จริง                                  |
 * |   - Fix: Gate bearing_alert ด้วย motor_state == 2 (RUNNING)       |
 * |     state != 2 → bearing_alert = "INVALID_STATE"                  |
 * |     (STOPPED=0, STARTING=1, STOPPING=3 → ไม่ evaluate Kurtosis)   |
 * |                                                                    |
 * | Fix 17: freq_alert trigger ขณะ RPM ต่ำ/หยุด (false alarm)         |
 * |   - Root cause: freq_drift คำนวณจาก freq/rot_freq                 |
 * |     ตอน RPM < threshold: rot_freq เกือบ 0 → ratio ไม่มีความหมาย  |
 * |     driftX/Y/Z พุ่งสูงมาก (-48, +18 ฯลฯ) ระหว่าง start/stop      |
 * |   - Fix: Gate freq_alert ด้วย RPM_FREQ_GATE (default 400 rpm)     |
 * |     rpm < 400 → freq_alert = false, freq_ratio_x/y/z = 0.0       |
 * |     ป้องกัน freq_alert trigger ระหว่าง startup/shutdown            |
 * |                                                                    |
 * | Fix 18: peak hold สะสมค่า impulse จาก motor transient             |
 * |   - Root cause: g_velPeakHold ไม่ reset ตอน motor หยุด           |
 * |     spike จาก deceleration ค้างอยู่ใน peak จนถึง publish ถัดไป   |
 * |   - Fix: Reset g_velPeakHold เมื่อ motor state เปลี่ยนเป็น        |
 * |     STOPPED (state=0) ใน taskStateMachine                         |
 * |     เพื่อให้ peak reflect เฉพาะช่วง RUNNING จริง                  |
 * +--------------------------------------------------------------------+
 *
 * v15.1 Changes (full_3axis_cf_kurtosis_patch):
 * +--------------------------------------------------------------------+
 * | Feature: เพิ่ม CF และ Kurtosis ครบทั้ง 3 แกน                       |
 * |   v15.0 อ่านแค่ CFX/KX (0x47~0x48) -- Y และ Z ยังขาด              |
 * |   Bearing fault อาจ radiate เฉพาะแกน radial หรือ axial             |
 * |   ถ้าดูแค่ X อาจพลาด fault ที่ Y หรือ Z                            |
 * |                                                                    |
 * |   - เพิ่ม Transaction 4: CFY (0x53) + KY (0x54)                   |
 * |   - เพิ่ม Transaction 5: CFZ (0x5F) + KZ (0x60)                   |
 * |   - เพิ่ม fields: cf_y, cf_z, kurtosis_y, kurtosis_z              |
 * |   - เพิ่ม kurtosis_max = max(KX, KY, KZ) -- ใช้ใน bearing alert   |
 * |   - เพิ่ม kurtosis_dominant_axis (0=X, 1=Y, 2=Z) -- fault localize |
 * |   - เพิ่ม cf_max = max(CFX, CFY, CFZ)                             |
 * |   - timing: cycle ~30ms → ~50ms (ยังอยู่ใน 250ms budget)          |
 * |                                                                    |
 * |   Bearing alert logic (ใน publishTelemetry):                       |
 * |   - kurtosis_max > 4.0  → BEARING early warning                   |
 * |   - kurtosis_max > 6.0  → BEARING confirmed                       |
 * |   - dominant axis บอก bearing ตัวไหน (radial vs axial)            |
 * |                                                                    |
 * |   Register map ที่เพิ่ม (Datasheet §6.4.15~16):                    |
 * |   0x53 CFY  /1000 → cf_y                                          |
 * |   0x54 KY   /1000 → kurtosis_y                                    |
 * |   0x5F CFZ  /1000 → cf_z                                          |
 * |   0x60 KZ   /1000 → kurtosis_z                                    |
 * +--------------------------------------------------------------------+
 *
 * v15.0 Changes (true_peak_sensor_cf_patch):
 * +--------------------------------------------------------------------+
 * | Fix 14: crestFactor คำนวณผิดหลักการ -- ใช้ค่า sensor แทน          |
 * |   - เดิม: crestFactor = max(RMS) / RMS_ปัจจุบัน                   |
 * |     ปัญหา 1: ตัวตั้ง (peak hold) เป็น RMS ที่แปลงแล้ว ไม่ใช่ Peak  |
 * |     ปัญหา 2: ตัวตั้ง-ตัวหาร คนละ time window (~30s vs instant)    |
 * |   - Fix: ลบการคำนวณ crestFactor เอง ใช้ reg 0x47 (CFX) จาก sensor |
 * |     sensor คำนวณจาก raw 16KHz FIFO ภายใน chip ถูกต้องตามหลักการ   |
 * |                                                                    |
 * | Fix 15: g_velPeakHold เก็บ RMS (ที่แปลงแล้ว) ไม่ใช่ Peak จริง     |
 * |   - เดิม: g_velPeakHold = max(rms_overall) = max(raw/100 x 0.7071) |
 * |   - Fix: g_velPeakHold = max(vel_peak_overall) = max(raw/100)      |
 * |     ค่า "peak" ใน MQTT payload จึงเป็น velocity peak จริง [mm/s]   |
 * |                                                                    |
 * | Feature: เพิ่ม True Peak Velocity fields ใน VibrationData_t        |
 * |   - vel_peak_x/y/z     : peak velocity ต่อแกน [mm/s] = raw/100    |
 * |   - vel_peak_overall   : max(vel_peak_x, y, z)                    |
 * |                                                                    |
 * | Feature: เพิ่ม Sensor-computed CF และ Kurtosis                     |
 * |   - cf_x       : Acceleration Crest Factor X (reg 0x47, /1000)    |
 * |   - kurtosis_x : Acceleration Kurtosis X     (reg 0x48, /1000)    |
 * |   - เพิ่ม Modbus Transaction 3: อ่าน REG_CFX (0x47) 2 registers   |
 * |     timing เพิ่ม ~10ms: cycle ~20ms → ~30ms (ยังอยู่ใน 250ms)    |
 * |   - publish kurtosis_x ใน /sensor payload สำหรับ bearing diagnosis |
 * +--------------------------------------------------------------------+
 *
 * v14.9 Changes (wdt_tls_debug_patch):
 * +--------------------------------------------------------------------+
 * | Fix 12: WDT "task not found" / "Network4G did not reset WDT" crash  |
 * |   - Root cause A: Network task was never subscribed to TWDT, yet   |
 * |     the handshake loop called esp_task_wdt_reset() (no-op → spam). |
 * |   - Root cause B: Arduino ESP32 default TWDT timeout = 5s. When   |
 * |     Network4G blocks CPU1 during modem ops, TLS handshake, or MQTT |
 * |     publish (all via 4G), IDLE task on CPU1 starves → WDT fires    |
 * |     "Network4G did not reset watchdog" → panic + reboot loop.      |
 * |   - Fix A: esp_task_wdt_reconfigure() in setup() sets timeout to   |
 * |     30s (covers worst-case modem re-registration + TLS handshake). |
 * |   - Fix B: esp_task_wdt_add(NULL) at taskNetwork() start to        |
 * |     subscribe the task explicitly. esp_task_wdt_reset() called at  |
 * |     every 100ms loop iteration (end of main while(1) loop).        |
 * |   - Removed stale esp_task_wdt_reset() calls from TLS loop;        |
 * |     they were no-ops (task not subscribed) and are now unnecessary  |
 * |     because handshake completes well within the 30s timeout.        |
 * |                                                                    |
 * | Fix 13: TLS verbose debug threshold 3 → 0 (production)           |
 * |   - mbedtls_debug_set_threshold(3) was left from debug session,   |
 * |     generating ~10KB of [mbedTLS L3] serial output on every       |
 * |     reconnect — wasting CPU on Serial.printf, polluting the log.  |
 * |   - Fix: threshold set to 0 (silent). To re-enable: change 0 → 1 |
 * |     (errors only) or 3 (full trace) and recompile.                |
 * |   - Removed [TLS-DBG] verbose prints from handshake loop (2s     |
 * |     progress tick, maintain() counters). Error-path prints        |
 * |     ([TLS] Handshake FAILED, TCP lost, etc.) retained.            |
 * +--------------------------------------------------------------------+
 *
 * v14.8 Changes (velPeakHold_slotDur_reset_patch):
 * +--------------------------------------------------------------------+
 * | Fix 9: g_velPeakHold not reset on maintenance                     |
 * |   - Peak hold persisted after reset: first post-maintenance        |
 * |     /vibration publish (taskNetwork PUB-1) snapshots               |
 * |     g_velPeakHold BEFORE overwriting it → broker receives the      |
 * |     pre-maintenance peak (e.g. 3.2 mm/s from a prior fault run)    |
 * |     as the reported peak for the very first post-reset cycle.      |
 * |   - Fix: g_velPeakHold = 0.0f; in maintenance reset block,        |
 * |     placed after variance reset (mirrors sensor-offline branch     |
 * |     ~line 2547 and per-publish reset in taskNetwork ~line 4185).   |
 * |                                                                    |
 * | Fix 10: g_slotDur1sMs not reset on maintenance                    |
 * |   - RPM-adaptive slot duration retains pre-maintenance value.      |
 * |     If machine was at low RPM before reset (e.g. slow-spin →       |
 * |     slotDur = 2800ms), analytics task waits 2.8s before first      |
 * |     buf1s flush instead of the expected 1.0s — buf1s appears       |
 * |     stuck for nearly 2 extra seconds post-warmup.                  |
 * |   - Fix: g_slotDur1sMs = 1000UL; in maintenance reset block.      |
 * |     taskAnalytics overwrites this on its first tick after resume   |
 * |     (~line 4499) so the cost is at most one 1s tick at default     |
 * |     rate — correct and safe.                                       |
 * |                                                                    |
 * | Fix 11: Serial2 undeclared — ESP32 Arduino core 3.x compile error  |
 * |   - core 2.x pre-declared Serial0/1/2 as global HardwareSerial     |
 * |     objects. core 3.x removed the implicit Serial2 global →        |
 * |     'Serial2' was not declared in this scope (setup() line 5351).  |
 * |   - Fix: HardwareSerial SerialRS485(2); declared alongside         |
 * |     SerialAT(1). All Serial2 references in setup() replaced with   |
 * |     SerialRS485. UART number (2), baud, and pins unchanged.        |
 * +--------------------------------------------------------------------+
 * v14.7 Changes (button_stack_i2c_patch):
 * +--------------------------------------------------------------------+
 * | Fix 7: STACK_SIZE_BUTTON 2048 → 4096                              |
 * |   - Live watermark trend: sess1=88B, sess2=316B, sess3=172B        |
 * |     (declining each patch as maintenance block grows)              |
 * |   - Root cause: rtc.now() → Wire I2C call stack depth inside       |
 * |     maintenance reset block + Serial.printf + FreeRTOS overhead    |
 * |   - At 172B remaining (92% used), any future addition or stack     |
 * |     alignment variance risks silent overflow / data corruption.    |
 * |   - Fix: 2048 → 4096 (2220B safety margin post-fix)               |
 * |                                                                    |
 * | Fix 8: rtc.now() in taskButtonHandler without mutexI2C            |
 * |   - Line 3126: called inside 8-second SELECT maintenance block     |
 * |   - taskDisplayUpdate holds mutexI2C during OLED writes (10Hz)     |
 * |   - Both tasks on Core 1; FreeRTOS preemption can interleave      |
 * |     Wire I2C transactions → bus corruption / wrong timestamp       |
 * |   - Fix: wrap rtc.now() with xSemaphoreTake(mutexI2C, 100ms)      |
 * |     If mutex not obtained in 100ms, fill event with zeros (safe    |
 * |     fallback — audit event still queued, timestamp shows 0000).   |
 * +--------------------------------------------------------------------+
 *
 * v14.6 Changes (decision_state_reset_patch):
 * +--------------------------------------------------------------------+
 * | Fix 3: g_prevFinalState not reset → false stateChanged on first    |
 * |   post-maintenance decision cycle.                                 |
 * |   - If prev session ended at finalPred=1 or 2, first cycle after  |
 * |     reset sees (0 != 1/2) → stateChanged=true → g_stateChangeCyc  |
 * |     =3 → publish interval unnecessarily drops to 5 s for 3 cycles.|
 * |   - Fix: g_prevFinalState = 0; in maintenance reset block.         |
 * |                                                                    |
 * | Fix 4: g_stateChangeCyc not reset → stale boost cycles carry over  |
 * |   - Fix: g_stateChangeCyc = 0; in maintenance reset block.         |
 * |                                                                    |
 * | Fix 5: g_prevTtwBest not reset → wrong ttwRoC in first post-reset  |
 * |   decision cycle (ttwRoC = 0 - old_ttw_best ≠ 0).                 |
 * |   - Fix: g_prevTtwBest = 0.0f; in maintenance reset block.         |
 * |                                                                    |
 * | Fix 6: prevTtwRoC was function-local static inside runDecisionEngine|
 * |   → unreachable by maintenance reset.                              |
 * |   - Promoted to global g_prevTtwRoC = 0.0f (same semantics).       |
 * |   - g_prevTtwRoC = 0.0f; added to maintenance reset block.         |
 * +--------------------------------------------------------------------+
 *
 * v14.5 Changes (accumulator_reset_patch):
 * +--------------------------------------------------------------------+
 * | Fix 1: g_decision not reset on maintenance → stale fault type      |
 * |   - After reset, old fault (e.g. RESONANCE sc=1.00) persisted in  |
 * |     g_decision until runDecisionEngine() ran with new data.        |
 * |   - Fix: memset(&g_decision,0) + re-init fields to boot defaults  |
 * |     (FAULT_UNKNOWN, unc=1.0, ALARM_SILENT, pub=30s, "init")        |
 * |                                                                    |
 * | Fix 2: g_osg_r_s / g_osg_r_m not reset on maintenance             |
 * |   - OSG reliability multipliers carried stale TTW weights into     |
 * |     the first post-warmup decision cycle (r_s/r_m < 1.0).         |
 * |   - Fix: g_osg_r_s = 1.0f; g_osg_r_m = 1.0f; (neutral weights)   |
 * +--------------------------------------------------------------------+
 *
 * v14.3 Changes (buf1024_patch):
 * +--------------------------------------------------------------------+
 * | Root cause discovered by live MQTT log measurement (v14.3):        |
 * |   /fusion/aedf actual JSON = 486 B  → doc<384> drops 102 B of     |
 * |     fields silently BEFORE serializeJson() is even called.         |
 * |   /fusion/fvri actual JSON = 388 B  → doc<384> drops fields too.  |
 * |   buf[450] (v14.2 fix) was still smaller than actual payload.      |
 * |                                                                    |
 * | Strategy from v14.3: buf[1024] + adequately sized doc for all     |
 * |   analytics publish blocks (PUB-B … PUB-G).                       |
 * |   Each block is in its own {} scope → only one buf live at once   |
 * |   → no stack explosion (analytics task stack = 6144 B).           |
 * |                                                                    |
 * | Actual JSON sizes measured from live MQTT log:                     |
 * |   PUB-B /fusion/raw   : ~480 B  doc<768>  buf[1024]               |
 * |   PUB-C /fusion/aedf  : ~486 B  doc<768>  buf[1024]  (was <384!)  |
 * |   PUB-D /fusion/osg   : ~339 B  doc<512>  buf[1024]               |
 * |   PUB-E /fusion/fvri  : ~388 B  doc<512>  buf[1024]  (was <384!)  |
 * |   PUB-F /ttw/model    : ~265 B  doc<512>  buf[1024]               |
 * |   PUB-G /output       : ~541 B  doc<768>  buf[1024]               |
 * +--------------------------------------------------------------------+
 *
 * v14.2 Changes (buffer_safety_patch_2):
 * +--------------------------------------------------------------------+
 * | Fix 4:  PUB-B /fusion/raw   buf[640]→buf[700]  (off-by-one)       |
 * | Fix 5:  PUB-C /fusion/aedf  buf[384]→buf[450]  (confirmed trunc)  |
 * | Fix 6:  PUB-D /fusion/osg   buf[384]→buf[450]  (off-by-one)       |
 * | Fix 7:  PUB-E /fusion/fvri  buf[384]→buf[450]  (confirmed trunc)  |
 * | Fix 8:  PUB-F /ttw/model    buf[512]→buf[580]  (off-by-one)       |
 * | Fix 9:  PUB-G /output       buf[640]→buf[700]  (off-by-one)       |
 * | Fix 10: PUB-2 comment mismatch corrected                           |
 * +--------------------------------------------------------------------+
 *
 * v14.1 Changes (buffer_safety_patch):
 * +--------------------------------------------------------------------+
 * | Fix 1: PUB-1 /sensor — char buf[560] → buf[680]                   |
 * |   - Root cause: buf was 80 B SHORTER than StaticJsonDocument<640>  |
 * |   - serializeJson() silently truncated at 559 B (invalid JSON)     |
 * |   - Fix: buf[680] > doc(640) with 40 B safety margin              |
 * |   - Added: size_t sz + truncation guard + Serial.printf size log   |
 * |                                                                    |
 * | Fix 2: PUB-2 /decision — char buf[420] → buf[600]                 |
 * |   - Root cause: buf was 28 B SHORTER than StaticJsonDocument<448>  |
 * |   - Final fix: doc<576> buf[600] with 24 B safety margin           |
 * |   - Added: size_t sz + truncation guard + Serial.printf size log   |
 * |                                                                    |
 * | Fix 3: PUB-A /trend — char buf[640] → buf[700]                    |
 * |   - Root cause: buf == doc → off-by-one (max writable = 639 B)    |
 * |   - Observed max 585 B, headroom was only ~55 B before fix         |
 * |   - Fix: buf[700] gives 115 B headroom for future field additions  |
 * +--------------------------------------------------------------------+
 *
 * v14.0 Changes (Patent Claim 2 — Dynamic Slot Duration):
 * +--------------------------------------------------------------------+
 * | Claim 2: g_slotDur1sMs — RPM-adaptive buf1s slot duration         |
 * |   - computeSlotDurMs(rpm): slot = clamp(SLOT_REVS_TARGET*60000/rpm,|
 * |     SLOT_DUR_MIN_MS, SLOT_DUR_MAX_MS)                             |
 * |   - SLOT_REVS_TARGET=24 revs/slot (1450RPM → 993ms ≈ original 1s) |
 * |   - SLOT_DUR_MIN_MS=400  SLOT_DUR_MAX_MS=3000                     |
 * |   - taskAnalytics: replaced cnt10s/cnt60s with millis accumulators |
 * |     acc1sMs / acc10sMs / acc60sMs driven by g_slotDur1sMs         |
 * |   - buf10s threshold = 10×slotDur1s, buf60s = 60×slotDur1s        |
 * |   - FreeRTOS tick period unchanged (1000ms); RPM read per tick     |
 * |   - /trend payload: added slot_dur_ms + slot_revs_target fields    |
 * |                                                                    |
 * v13.0 Patch Changes (mqtt_pipeline_patch):
 * | Patch A: g_rawScores[FAULT_TYPE_COUNT] — pre-AEDF Gaussian scores  |
 * |   - Added global float[7] stored after classifyFaultProbabilistic  |
 * |   - Published as "raw_scores" nested object in /fusion/raw         |
 * |   - StaticJsonDocument<512> -> <640> in PUB-B                     |
 * |                                                                    |
 * | Patch B: g_osg_r_s / g_osg_r_m — OSG*FVRI reliability multipliers |
 * |   - Added volatile float globals stored after OSG suppression      |
 * |   - Published as r_s / r_m fields in /fusion/osg                  |
 * |   - StaticJsonDocument<320> -> <384> in PUB-D                     |
 * |   - Removed stale comment "local to runDecisionEngine()"           |
 * |                                                                    |
 * | Patch C: Sensor offline alert routed to /sensor topic              |
 * |   - Changed publish target from g_mqttTopic to g_mqttTopicSensor   |
 * |   - Added stage="sensor" + execution_location="edge" fields        |
 * |   - StaticJsonDocument<256> -> <296>; char buf[256] -> [296]       |
 * +--------------------------------------------------------------------+
 * ============================================================================
 */

// ============================================================================
// MODEM CONFIGURATION (Must be before TinyGSM include)
// ============================================================================
#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER 4096

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <TinyGsmClient.h>
#include <MQTT.h>  // joel-gaehwiler/MQTT (arduino-mqtt) -- supports QoS 0/1/2
#include <ModbusMaster.h>
#include <ArduinoJson.h>
#include <RTClib.h>
#include <Preferences.h>   // NVS Flash -- runtime_hour persistence

// ============================================================================
// FAULT & ALARM ENUMERATIONS
// MUST be declared before any function that uses them. Arduino IDE performs a
// full-file prototype scan before compilation; if FaultType_t / AlarmClass_t
// are not visible during that scan, auto-generated prototypes for
// faultTypeStr() and alarmClassStr() will fail with "not declared in scope".
// ============================================================================
typedef enum {
  FAULT_NORMAL       = 0,  // No fault signature detected
  FAULT_IMBALANCE    = 1,  // 1x dominant -- mass imbalance or shaft bow
  FAULT_MISALIGNMENT = 2,  // 2x dominant -- angular/parallel misalignment
  FAULT_LOOSENESS    = 3,  // 3x or sub-harmonic / high stddev -- structural looseness
  FAULT_BEARING      = 4,  // Non-integer ratio + spikes -- developing bearing fault
  FAULT_RESONANCE    = 5,  // High harmonic (>3x, non-BPF) -- structural resonance
  FAULT_UNKNOWN      = 6   // Insufficient data or ambiguous pattern
} FaultType_t;

typedef enum {
  ALARM_SILENT        = 0,  // pred=0, low severity -- no alert
  ALARM_INFORMATIONAL = 1,  // pred=1, severity < 50 -- informational
  ALARM_WARNING       = 2,  // pred=1, severity >= 50 OR specific fault types
  ALARM_CRITICAL      = 3   // pred=2 confirmed by fusion engine
} AlarmClass_t;

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// --- Pin Definitions (LilyGO T-Vending S3) ---
#define PIN_BUTTON 5        // SELECT button (page navigation)
#define PIN_BUTTON_ENTER 6  // ENTER button (alarm acknowledge)
#define PIN_BUZZER 7        // Buzzer output

// --- RS485 Pins ---
#define RS485_RX_PIN 38
#define RS485_TX_PIN 39
#define RS485_EN_PIN 42

// --- I2C Pins ---
#define I2C_SDA_PIN 44
#define I2C_SCL_PIN 43

// --- 4G Modem Pins (SIMCom A7670) ---
#define MODEM_TX 3
#define MODEM_RX 46
#define MODEM_POWER_ON 4
#define MODEM_RESET_PIN 9
#define MODEM_RESET_LEVEL HIGH
#define BUILTIN_LED 10

// --- Display Configuration ---
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_ADDRESS 0x3C

// --- Modbus Configuration ---
#define MODBUS_BAUDRATE 9600
#define MODBUS_SLAVE_ID 0x50
// v15.7: อ่าน VRMS โดยตรงจาก sensor (Datasheet §6.4.14~16)
// address ไม่ต่อเนื่อง → อ่านแยก 3 transactions
#define REG_VRMS_X 0x50  // VRMSX: velocity RMS X [mm/s] = raw/1000 (uint16)
#define REG_VRMS_Y 0x5C  // VRMSY: velocity RMS Y [mm/s] = raw/1000 (uint16)
#define REG_VRMS_Z 0x68  // VRMSZ: velocity RMS Z [mm/s] = raw/1000 (uint16)
#define REG_TEMPERATURE 0x40
#define REG_FREQ_X 0x44  // Frequency X,Y,Z (0x44~0x46) per WTVB02 manual
#define REG_CFX    0x47  // CFX=Accel Crest Factor X, KX=Kurtosis X (0x47~0x48) §6.4.14
                         // CFY=0x53, CFZ=0x5F (ไม่ต่อเนื่อง -- อ่านแยก transaction ถ้าต้องการ)
#define REG_CFY    0x53  // CFY=Accel Crest Factor Y, KY=Kurtosis Y (0x53~0x54) §6.4.15
#define REG_CFZ    0x5F  // CFZ=Accel Crest Factor Z, KZ=Kurtosis Z (0x5F~0x60) §6.4.16
// ????? error ??????????????????? OFFLINE (3 x 250ms = 750ms)
#define MODBUS_OFFLINE_THRESHOLD 3

// --- 4G Network Configuration ---
static constexpr const char* APN = "internet";
static constexpr const char* GPRS_USER = "";
static constexpr const char* GPRS_PASS = "";

// --- Identity Configuration (Phase 1) ---
// *** ??? 4 ???????????????? -- ?????????????????? ***
#define PLANT_ID "plant01"   // Plant / Site identity
#define MACHINE_ID "pump01"  // Machine identity (tag-level)
#define SENSOR_ID "vb01"     // Sensor identity
#define NAMEPLATE_RPM 3000   // Motor nameplate RPM (used as RATED_RPM reference)

// --- Proximity / RPM Sensor Configuration ---
#define PIN_RPM               17      // Proximity sensor pulse input (PC817 or NPN)
#define PULSE_PER_REV         1       // Pulses per revolution
#define MAX_RPM               3000   // Spike reject ceiling
#define MIN_RPM_VALID         300      // Below this -> treat as zero
#define RATED_RPM             NAMEPLATE_RPM   // Rated speed (centre of RUNNING band)
#define RATED_RPM_TOL         100      // +/-50 RPM around RATED_RPM -> RUNNING state
#define RPM_SMOOTH_ALPHA      0.25f   // EMA filter coefficient (0=heavy,1=none)
#define SPIKE_REJECT_FACTOR   1.1f    // Reject pulses > MAX_RPM x factor
#define NO_PULSE_STOPPING_MS  400     // No pulse > 400 ms -> STOPPING
#define FORCE_STOP_TIMEOUT_MS 2000    // No pulse > 2 s   -> STOPPED
#define FAULT_WINDOW_MS       3000    // RUNNING but no pulse > 3 s -> prox=0 (Fault)
#define RPM_FREQ_GATE         400     // v15.2: RPM ขั้นต่ำสำหรับ freq_alert / freq_ratio
                                      // rpm < 400 → freq_ratio = 0, freq_alert suppressed
                                      // ป้องกัน false alert ระหว่าง startup/shutdown
#define NVS_SAVE_INTERVAL_MS  30000UL // Save runtime_hour to NVS every 30 s

// --- MQTT Configuration (mTLS / Mosquitto) ---
#define MQTT_SERVER "iot.promlogix.com"  // Broker Public IP
#define MQTT_PORT 8883                           // TLS port
#define MQTT_CLIENT_ID "pump01"           // -> "PLANT01-ESP01"
#define MQTT_QOS 1                               // QoS 1 -- at-least-once delivery
// Mosquitto config:  require_certificate = true
//                    use_identity_as_username = true
//                    allow_anonymous = false
// -> Username is extracted from client certificate CN by the broker.
//   No explicit username/password required in CONNECT packet.
// Topic built at runtime: factory/{PLANT_ID}/machine/{MACHINE_ID}/vibration

// ============================================================================
// mTLS CERTIFICATES (PEM format)
// ============================================================================
// Replace the placeholder blocks below with your actual certificate contents.
// Use OpenSSL to generate:
//   openssl genrsa -out PLANT01-ESP01.key 2048
//   openssl req -new -key PLANT01-ESP01.key -out PLANT01-ESP01.csr \
//           -subj "/CN=PLANT01-ESP01"
//   openssl x509 -req -in PLANT01-ESP01.csr -CA industrial-root-ca.crt \
//           -CAkey industrial-root-ca.key -CAcreateserial \
//           -out PLANT01-ESP01.crt -days 3650
// ============================================================================

// Root CA -- industrial-root-ca.crt
static const char* root_ca = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDdzCCAl+gAwIBAgIUaeT/6iBLHWm30hAItq+9q9lv3ZMwDQYJKoZIhvcNAQEL
BQAwSzELMAkGA1UEBhMCVEgxEjAQBgNVBAoMCVByb21sb2dpeDEMMAoGA1UECwwD
SW9UMRowGAYDVQQDDBFQcm9tbG9naXgtUm9vdC1DQTAeFw0yNjA1MDIxMTQ1MTJa
Fw0zNjA0MjkxMTQ1MTJaMEsxCzAJBgNVBAYTAlRIMRIwEAYDVQQKDAlQcm9tbG9n
aXgxDDAKBgNVBAsMA0lvVDEaMBgGA1UEAwwRUHJvbWxvZ2l4LVJvb3QtQ0EwggEi
MA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDAupZ3ySVJ3qok6/ZmezLSDJsu
zp9eYGryO3w8ezbgyAvjE6nyaImmRUsTIwWR4V3J2XZz5G4Bzm32avDYqrMm4pnr
45UfVvldwLQFhMlkDUYFpWwxNDPxKvIkoJIzPKg/U5XT9glHsM5nvhpTKLpPrR7q
o1yfANtncwxRFQOwVrBqXjdmaveVqwOv/9n+J6+bELh3wKoOb5LTU1ZfISfE5PxE
YulvbVcPOcNTrIkmdyQP+arjiX4cNO3eQZo6OxdMYnenHlDaNbahv1qZBzwMHFUn
U9IGYsmmw/UIz+B1DUEUvuFdSbynqq6awfY76XPqDQidnXRHYo3Q+JPeY9uHAgMB
AAGjUzBRMB0GA1UdDgQWBBR/RPU+w/yXvpIX9CWZ0e8FUsEGxjAfBgNVHSMEGDAW
gBR/RPU+w/yXvpIX9CWZ0e8FUsEGxjAPBgNVHRMBAf8EBTADAQH/MA0GCSqGSIb3
DQEBCwUAA4IBAQC2k9C9TVMehB8N2UoW91+IacoJDBRSpHhH+U55EtJLSg4sU9RI
sfaA6VDNdttxdRgG6O8Vp1KNCw5NrUeICHei6boJyI0a3Fz3SOKC8y0hkclx6bcc
mv9jhc4OyCUTMat8ojjq3ozYrYSLlHAmpvc2rhhCn9Z39Fp+8mdPd42eLCy7x0sE
vGmxO01oyezvwJAZineNUn6e26DvuDYX+FrV4BxZBa++28Ab4iZSqBaE/XgQVMf7
gvuvFI/Aqftgjk0yv8wA6WcANh9stUDkI0gMBEmAzwl4y0v9G8ktgHrl1nk2WApy
GqxIBNFsqLdGN189BwBlBPnWscWJg+oGqLtT
-----END CERTIFICATE-----
)EOF";

// Client Certificate -- PLANT01-ESP01.crt  (CN must match MQTT_CLIENT_ID)
static const char* client_crt = R"EOF(
-----BEGIN CERTIFICATE-----
MIIC4zCCAcsCFHrIJfxa9kIp8xl2OiGamsQiUjWrMA0GCSqGSIb3DQEBCwUAMEsx
CzAJBgNVBAYTAlRIMRIwEAYDVQQKDAlQcm9tbG9naXgxDDAKBgNVBAsMA0lvVDEa
MBgGA1UEAwwRUHJvbWxvZ2l4LVJvb3QtQ0EwHhcNMjYwNTAyMTE0NTM0WhcNMjcw
NTAyMTE0NTM0WjARMQ8wDQYDVQQDDAZwdW1wMDEwggEiMA0GCSqGSIb3DQEBAQUA
A4IBDwAwggEKAoIBAQCcFCUyXJIAHj1m5LJNFSzrR9trx4ELBYq4UpAZZtiY618T
jYCecCqNRDIeaGNLadXjAqZRADE1cV1NZzIDnMw4DUA8fqEJ+8GURiCkvlypwDcf
Zv4Gdto20uQFSKfFvvoBRu7ze89VVNY/0uc4Qpip04hzK4fspmI/zDz8JUPxDELt
Rp769mKgmoNsUO14DSMWJbAolbXLDpBaGuD61Svlm5o/YKNfcLDbm35+6aDG7+/J
FzM9DYYPQxkvw+Nu+t2gkHfs+qcgLo+7oget/P2JXOy1Ysc2jMqQNDco8M1rob8T
NZzb4mKcW4LtgNGudsyel5p0utmlxKeTevXvIx5HAgMBAAEwDQYJKoZIhvcNAQEL
BQADggEBAF+pIJv97lLnSRRAwTo6UTxWEPhY32BCWsN09me6bDbq87dogqtZTL1H
c2wZClOea+jE0U/hBIo/Vh/f5YNCDH3nMEuIBgs6fNM8Y2DaIM/Wz6hh1iQap6FE
dwDbDMckfpGM4Sn5valLzYwkqe4aflaDY5Uas9FzolPOCItGsc4g2gafOaDoOrFo
XDjfTO3o2eznKC/Ou/ft/tVcdEX/BdFKu1Sqw8UntlkjaDtM4T9bz+RfoYTL0UPT
zKcQVPxhvKKbyiCe62gBYl5tLD1vriUBZwioCJLHfV2XMJx47msJiOLtyz6SdDc0
dzc7CkmVKekWfJ+jVC0bgIONpwLNEWo=
-----END CERTIFICATE-----
)EOF";

// Client Private Key -- PLANT01-ESP01.key
static const char* client_key = R"EOF(
-----BEGIN PRIVATE KEY-----
MIIEvQIBADANBgkqhkiG9w0BAQEFAASCBKcwggSjAgEAAoIBAQCcFCUyXJIAHj1m
5LJNFSzrR9trx4ELBYq4UpAZZtiY618TjYCecCqNRDIeaGNLadXjAqZRADE1cV1N
ZzIDnMw4DUA8fqEJ+8GURiCkvlypwDcfZv4Gdto20uQFSKfFvvoBRu7ze89VVNY/
0uc4Qpip04hzK4fspmI/zDz8JUPxDELtRp769mKgmoNsUO14DSMWJbAolbXLDpBa
GuD61Svlm5o/YKNfcLDbm35+6aDG7+/JFzM9DYYPQxkvw+Nu+t2gkHfs+qcgLo+7
oget/P2JXOy1Ysc2jMqQNDco8M1rob8TNZzb4mKcW4LtgNGudsyel5p0utmlxKeT
evXvIx5HAgMBAAECggEAAuJQaaTSQdRNOCiDru70PIjAYjZ2iPiaPpuv8/g1imXX
BOp5dPQHpUKcVnmBVDRpcl9rKVYCksU8fyCoCO8Nyv9br4J7gU64nf/JvKGT3sMh
gaAKk54AnEC7W+miyAGmZv2jjrY794ywxM8l3KFGZuT0wYQNZ+8PI7Snb9VUcxDK
jHGUQrXM+fbKGLXRkhfeFzfUD7aNqYZWMkt2KwzZsSh8M+ctp+G5IdcNEQOS2GVo
wX6d2KGkNudd9aCKi50XQPFem2TVOllFQYijld2SABYK1pQiPXR+NrLSnb6RUm15
3w/g3eax0qa52XdD+KTixxi1oPe8yxe4qDAz6lLEqQKBgQDK5U/Bt4Jo4geCV0Za
izoNnLcqa6+nHlsphYXw5pUqLSLq4RCGJMcbojkYXxXxJYsmlm0+OQ9S9WW6rJ8d
z2jPUWS3MKVW3KZ338oS9AgBr1aNhggkCQpg2SEbsmM1khs+t8HoN8LIFLGVb7PB
dZKLw8VctsSlxwPgsxsont4qHQKBgQDE7e+HKhKWplAwwIBDCUUy1xjZH+nIeYsV
hWF4iE1hDuUN/dYFVClDS4aRPDrgeB4lzzqw5NCSHR/Gu0TeHMspJj1k1CDBiI12
LruakLOw4P3ul13Yv23EJuUq/In8Q3otYF3yaks1nwM9/hgEMRmlsbUgbSGW1tKW
RkvorwqcswKBgHGEaOIuRPVfeOoQ4FjqSpmxE73VMBqlXkXV4cGNkOlfBYk6UN9s
lkW8tosPMBySb88wHIDSteMpTzhpOkEYeUB8/oeL3QXDQBQTjmCaThx7OEbINafL
sxXKhb6USPOBAmNNtlyxTfZZtZ2xOHZFzK8L4lFkJJPHzECclNZeRFh1AoGBAJML
t9eNquOivC4rD5r+yRT1WDCIi+COITSoq+d8n4rhvFd+Otkvxr/hHVJFTxFdn+VL
n9+Ge9ceuCOEoh/YEDthumYXn33joP2mV59KfWKOHg6SKBk4l5XoFSbL+5zKJejM
FFp21EHtwlX/7Z7zqtr2nvDfjD09m3FqfDP6wEnRAoGAK90MergErflRfvNqGmSf
BUkR4hIjiNHCnNizxJUtuwG+HDf7bg0Lvo5KYALYnwNuSEzE1EsLJEFl22a7zCJ8
EZ5Dxy12SrP6E+hbMhuV1KeMax45WU4beFu88wTlbz8A8is2KKjKbS9+8e79R1I/
l0PCpmCF8SZ8OXd/UfRIbLk=
-----END PRIVATE KEY-----
)EOF";

// --- Machine Configuration ---
#define MACHINE_NAME MACHINE_ID  // Display uses MACHINE_ID for consistency
#define BASELINE_RMS 2.1f
#define WARNING_RMS 4.5f
#define CRITICAL_RMS 7.1f

// --- FreeRTOS Configuration ---
#define STACK_SIZE_MODBUS    4096   // Modbus task stack
#define STACK_SIZE_DISPLAY   6144   // Display task stack (larger for U8g2)
#define STACK_SIZE_NETWORK  24576   // Phase2: 24KB -- RSA-2048 + JSON 2200B + TinyGSM peak
#define STACK_SIZE_BUTTON    4096   // V14.7: 2048→4096 (watermark was 172B=92% used; rtc+Wire+Serial.printf depth)
#define STACK_SIZE_STATE     6144   // Phase 4: 4096->6144 (watermark was 2236 = 54% used)
#define STACK_SIZE_ANALYTICS 6144   // Phase 3: +decision engine +classifyFault on stack

#define PRIORITY_MODBUS    5  // Highest priority (time-critical)
#define PRIORITY_STATE     4  // State machine
#define PRIORITY_DISPLAY   3  // Display updates
#define PRIORITY_ANALYTICS 3  // Analytics (same as display -- non-critical, 1s cadence)
#define PRIORITY_NETWORK   2  // Network (can tolerate delays)
#define PRIORITY_BUTTON    2  // Button handling
#define PRIORITY_BUZZER    1  // Lowest priority

// --- Queue Sizes ---
#define QUEUE_SIZE_SENSOR 5
#define QUEUE_SIZE_BUTTON 3
#define QUEUE_SIZE_DISPLAY 3
#define QUEUE_SIZE_MAINT 2   // V14.4: maintenance reset events (Button -> Network)

// --- Modem Timeouts ---
#define MODEM_INIT_TIMEOUT 30000    // 30 seconds for modem init
#define GPRS_CONNECT_TIMEOUT 60000  // 60 seconds for GPRS connect
#define MODEM_RETRY_DELAY 10000     // 10 seconds between retries

// --- Time Sync Configuration ---
#define NTP_SYNC_INTERVAL 86400000UL       // 24 hours between NTP syncs (ms)
#define NTP_SYNC_RETRY_INTERVAL 1800000UL  // 30 minutes retry on failure (ms)
#define NTP_DRIFT_WARN_SEC 5               // Warn if drift exceeds 5 seconds
#define NTP_DRIFT_MAX_SEC 30               // Force-correct if drift > 30 seconds
#define TIMEZONE_OFFSET_SEC (7 * 3600)     // UTC+7 (Bangkok/Thailand) -- adjust per site

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

// Hardware Serial for Modem (Serial1)
HardwareSerial SerialAT(1);

// Hardware Serial for Modbus RS485 (Serial2 / UART2)
// ESP32 Arduino core 3.x does not pre-declare Serial2 — must instantiate explicitly,
// same pattern as SerialAT above. UART number must match: 2 = UART2 (RX=38, TX=39).
HardwareSerial SerialRS485(2);

// TinyGSM objects
TinyGsm modem(SerialAT);
TinyGsmClient rawClient(modem, 1);  // Plain TCP on mux 1 (used for TCP reachability test)

// -----------------------------------------------------------------------------
// GsmTLSClient -- mirrors WiFiClientSecure but over 4G modem TCP transport
// -----------------------------------------------------------------------------
// Key design decisions vs previous versions:
//
// 1. CERTS PARSED ONCE -- entropy/drbg/x509/pk are initialized once at startup
//    via loadCerts() and never freed/re-parsed. This eliminates:
//    - "Private key parse failed: -0x3E80" (heap fragmentation after reconnects)
//    - ~42KB heap drop per attempt from repeated cert parsing
//
// 2. SSL CONTEXT RESET PER CONNECTION -- only _ssl and _conf are freed/re-init
//    on each stop()/connect() cycle, keeping the heavy cert data intact.
//
// 3. NON-BLOCKING bio_recv -- wait loop is in the handshake loop (not bio),
//    so mbedTLS drives timing correctly and large TLS records (server cert ~3KB)
//    have enough time to arrive over the slow 4G modem without hitting timeout.
// -----------------------------------------------------------------------------
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/x509_crt.h"
#include "mbedtls/pk.h"
#include "mbedtls/error.h"
#include "mbedtls/debug.h"  // TLS debug trace via mbedtls_debug_set_threshold()
#include "esp_task_wdt.h"   // WDT: reconfigure timeout, subscribe/reset Network task
#include "esp_system.h"     // v15.3: esp_reset_reason() -- detect reboot cause

class GsmTLSClient : public Client {
public:
  explicit GsmTLSClient(TinyGsm& modem)
    : _tcp(modem, 0), _modem(modem) {
    // Persistent contexts -- live for the lifetime of the object
    mbedtls_entropy_init(&_entropy);
    mbedtls_ctr_drbg_init(&_drbg);
    mbedtls_x509_crt_init(&_ca_crt);
    mbedtls_x509_crt_init(&_cli_crt);
    mbedtls_pk_init(&_pk);
    // Per-connection contexts
    mbedtls_ssl_init(&_ssl);
    mbedtls_ssl_config_init(&_conf);

    mbedtls_ctr_drbg_seed(&_drbg, mbedtls_entropy_func, &_entropy,
                          (const uint8_t*)"gsmtls1", 7);
  }

  ~GsmTLSClient() {
    _freeSession();
    mbedtls_pk_free(&_pk);
    mbedtls_x509_crt_free(&_cli_crt);
    mbedtls_x509_crt_free(&_ca_crt);
    mbedtls_ctr_drbg_free(&_drbg);
    mbedtls_entropy_free(&_entropy);
  }

  // -- Call once after construction -- parses certs into mbedTLS structures --
  bool loadCerts(const char* ca, const char* crt, const char* key) {
    int ret;
    if (ca) {
      ret = mbedtls_x509_crt_parse(&_ca_crt,
                                   (const uint8_t*)ca, strlen(ca) + 1);
      if (ret != 0) {
        Serial.printf("[TLS] CA parse failed: -0x%04X\n", -ret);
        return false;
      }
    }
    if (crt) {
      ret = mbedtls_x509_crt_parse(&_cli_crt,
                                   (const uint8_t*)crt, strlen(crt) + 1);
      if (ret != 0) {
        Serial.printf("[TLS] Cert parse failed: -0x%04X\n", -ret);
        return false;
      }
    }
    if (key) {
      ret = mbedtls_pk_parse_key(&_pk,
                                 (const uint8_t*)key, strlen(key) + 1,
                                 NULL, 0, mbedtls_ctr_drbg_random, &_drbg);
      if (ret != 0) {
        Serial.printf("[TLS] Key parse failed: -0x%04X\n", -ret);
        return false;
      }
    }
    _certsLoaded = true;
    Serial.println("[TLS] Certificates parsed and loaded +");

    // -- Diagnostic: print cert subjects to confirm CN = MQTT client ID --
    // If CN does not match MQTT_CLIENT_ID, broker will reject with rc=5.
    // Requires mbedtls/x509.h which is included transitively via x509_crt.h.
    {
      char buf[128];
      mbedtls_x509_dn_gets(buf, sizeof(buf), &_cli_crt.subject);
      Serial.printf("[TLS] Client cert subject: %s\n", buf);
      mbedtls_x509_dn_gets(buf, sizeof(buf), &_cli_crt.issuer);
      Serial.printf("[TLS] Client cert issuer:  %s\n", buf);
      mbedtls_x509_dn_gets(buf, sizeof(buf), &_ca_crt.subject);
      Serial.printf("[TLS] CA cert subject:     %s\n", buf);
      // Confirm key matches cert by checking key type
      Serial.printf("[TLS] Key type: %s, bitlen: %u\n",
                    mbedtls_pk_get_name(&_pk),
                    (unsigned)mbedtls_pk_get_bitlen(&_pk));
    }
    return true;
  }

  // Compatibility shims for existing setupTLS() call pattern
  void setCACert(const char* ca) {
    _ca_pem = ca;
  }
  void setCertificate(const char* c) {
    _crt_pem = c;
  }
  void setPrivateKey(const char* k) {
    _key_pem = k;
  }

  // Called by setupTLS() -- triggers actual cert loading
  bool applyCredentials() {
    return loadCerts(_ca_pem, _crt_pem, _key_pem);
  }

  int connect(IPAddress ip, uint16_t port) override {
    return connect(ip.toString().c_str(), port);
  }

  int connect(const char* host, uint16_t port) override {
    if (!_certsLoaded) {
      Serial.println("[TLS] ERROR: loadCerts() not called before connect()");
      return 0;
    }

    // -- Always reset SSL session before connecting --
    // arduino-mqtt does NOT call stop() before retry, so _ssl/_conf may be
    // dirty from a previous failed handshake. Reset them unconditionally.
    if (_tcp.connected()) _tcp.stop();
    _freeSession();
    mbedtls_ssl_init(&_ssl);
    mbedtls_ssl_config_init(&_conf);
    _connected = false;

    // -- Step 1: TCP via modem --
    Serial.printf("[TLS] TCP connecting to %s:%d ...\n", host, port);
    if (!_tcp.connect(host, port)) {
      Serial.println("[TLS] TCP connect failed");
      return 0;
    }
    Serial.println("[TLS] TCP OK, starting TLS handshake...");

    // -- Step 2: configure SSL (TLS 1.2 to match broker mosquitto.conf) --
    mbedtls_ssl_config_defaults(&_conf,
                                MBEDTLS_SSL_IS_CLIENT,
                                MBEDTLS_SSL_TRANSPORT_STREAM,
                                MBEDTLS_SSL_PRESET_DEFAULT);
    mbedtls_ssl_conf_rng(&_conf, mbedtls_ctr_drbg_random, &_drbg);

    // Force TLS 1.2 -- broker mosquitto.conf: tls_version tlsv1.2
    mbedtls_ssl_conf_max_tls_version(&_conf, MBEDTLS_SSL_VERSION_TLS1_2);
    mbedtls_ssl_conf_min_tls_version(&_conf, MBEDTLS_SSL_VERSION_TLS1_2);

    // Verify server cert, and present our client cert (mTLS)
    mbedtls_ssl_conf_authmode(&_conf, MBEDTLS_SSL_VERIFY_REQUIRED);
    mbedtls_ssl_conf_ca_chain(&_conf, &_ca_crt, NULL);

    int ret = mbedtls_ssl_conf_own_cert(&_conf, &_cli_crt, &_pk);
    if (ret != 0) {
      Serial.printf("[TLS] own_cert failed: -0x%04X\n", -ret);
      return 0;
    }

    // -- FIX-1 (REVISED): DO NOT configure sig_algs or sig_hashes here --
    //
    // Root cause analysis of original code:
    //   mbedtls_ssl_conf_sig_algs()  -> only exists with CONFIG_MBEDTLS_SSL_PROTO_TLS1_3=y
    //   MBEDTLS_TLS1_3_SIG_*        -> only defined with TLS 1.3 enabled
    //   mbedtls_ssl_conf_sig_hashes()-> REMOVED in mbedTLS 3.x (Arduino ESP32 core 3.x)
    //
    // For TLS 1.2 with RSA keys, mbedTLS ALWAYS uses PKCS#1 v1.5 by default.
    // RSA-PSS is strictly TLS 1.3 only. Explicitly setting sig algorithms is
    // unnecessary and causes compile failures across mbedTLS versions.
    // Remove the block entirely -- defaults are correct for this use case.

    ret = mbedtls_ssl_setup(&_ssl, &_conf);
    if (ret != 0) {
      Serial.printf("[TLS] ssl_setup failed: -0x%04X\n", -ret);
      return 0;
    }

    // -- TLS DEBUG: disabled in production --
    // Set threshold: 0=off 1=error 2=state_change 3=info 4=verbose
    // To enable: change 0 -> 1 (errors only) or 3 (full trace) and recompile.
    // At threshold=3 generates ~10KB of serial output per handshake.
#if defined(MBEDTLS_DEBUG_C)
    mbedtls_ssl_conf_dbg(&_conf, _tls_debug_cb, NULL);
    mbedtls_debug_set_threshold(0);  // PRODUCTION: 0=silent (change to 1/3 to debug)
#endif

    mbedtls_ssl_set_hostname(&_ssl, host);
    // BioCtx carries self so static _bio_recv can access _rxBuf
    _bioCtx = { &_tcp, &_modem, this };
    mbedtls_ssl_set_bio(&_ssl, &_bioCtx, _bio_send, _bio_recv, NULL);

    // -- Step 3: TLS handshake --
    // NOTE: Network task (taskNetwork) is intentionally NOT subscribed to the
    // ESP-IDF Task Watchdog (TWDT). The task performs blocking operations
    // (4G modem TCP + TLS handshake ~1-3s, MQTT publish with modem I/O) that
    // cannot be interrupted. Subscribing would require resetting the WDT at
    // every blocking point, and any missed reset causes a reboot.
    // Network connectivity loss is handled at the application level via
    // reconnect logic (30s retry interval), not via TWDT.
    uint32_t t0 = millis();

    while ((ret = mbedtls_ssl_handshake(&_ssl)) != 0) {
      if (ret == MBEDTLS_ERR_SSL_WANT_READ) {
        uint32_t tw = millis();
        while (!_tcp.available()) {
          _modem.maintain();
          vTaskDelay(pdMS_TO_TICKS(5));
          if (!_tcp.connected()) {
            Serial.println("[TLS] TCP lost during handshake");
            return 0;
          }
          if (millis() - tw > 15000UL) {
            Serial.println("[TLS] No data from broker (15s) -- handshake stalled");
            return 0;
          }
        }
      } else if (ret == MBEDTLS_ERR_SSL_WANT_WRITE) {
        _modem.maintain();
        vTaskDelay(pdMS_TO_TICKS(5));
      } else {
        char errbuf[80];
        mbedtls_strerror(ret, errbuf, sizeof(errbuf));
        Serial.printf("[TLS] Handshake FAILED (-0x%04X): %s\n", -ret, errbuf);
        if (ret == MBEDTLS_ERR_SSL_CONN_EOF)
          Serial.println("[TLS] -> TCP layer: connection closed by remote");
        else if (ret == MBEDTLS_ERR_X509_CERT_VERIFY_FAILED)
          Serial.println("[TLS] -> AUTH: server cert verify failed (check CA)");
        else if (ret == MBEDTLS_ERR_SSL_FATAL_ALERT_MESSAGE)
          Serial.printf("[TLS] -> ALERT: broker sent fatal alert "
                        "(check CN=%s matches MQTT_CLIENT_ID)\n",
                        host);
        else if (ret == MBEDTLS_ERR_SSL_HANDSHAKE_FAILURE)
          Serial.println("[TLS] -> CIPHER: no common cipher suite");
        return 0;
      }

      if (millis() - t0 > 90000UL) {
        Serial.println("[TLS] Handshake timeout (90s)");
        return 0;
      }
    }

    // Log negotiated cipher suite for diagnostics
    Serial.printf("[TLS] mTLS handshake OK + (%lus) cipher: %s\n",
                  (unsigned long)(millis() - t0) / 1000,
                  mbedtls_ssl_get_ciphersuite(&_ssl));
    _connected = true;
    return 1;
  }

  size_t write(uint8_t b) override {
    return write(&b, 1);
  }

  size_t write(const uint8_t* buf, size_t sz) override {
    if (!_connected) return 0;
    int ret = mbedtls_ssl_write(&_ssl, buf, sz);
    return ret > 0 ? (size_t)ret : 0;
  }

  int available() override {
    if (!_connected) return 0;
    // FIX-3: mbedtls_ssl_get_bytes_avail() only returns bytes already decrypted
    // from the LAST ssl_read() call. A7670E additionally has the issue where
    // TinyGSM's tcp.available() reports modem buffer count but local rx is empty.
    // Report 1 if EITHER ssl layer OR TCP has pending data -- read() will handle it.
    int pending = mbedtls_ssl_get_bytes_avail(&_ssl);
    if (pending > 0) return pending;
    return (_tcp.available() > 0) ? 1 : 0;
  }

  int read() override {
    uint8_t b;
    return (read(&b, 1) == 1) ? b : -1;
  }

  int read(uint8_t* buf, size_t sz) override {
    if (!_connected) return -1;
    int ret = mbedtls_ssl_read(&_ssl, buf, sz);
    if (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE) return 0;
    if (ret <= 0) {
      _connected = false;
      return -1;
    }
    return ret;
  }

  int peek() override {
    return -1;
  }

  void flush() override {
    _tcp.flush();
  }

  void stop() override {
    if (_connected) {
      mbedtls_ssl_close_notify(&_ssl);
      _connected = false;
    }
    _tcp.stop();
    _freeSession();
    // Re-init only the per-connection contexts (certs stay parsed)
    mbedtls_ssl_init(&_ssl);
    mbedtls_ssl_config_init(&_conf);
    _rxReset();  // clear local rx buffer for next connection
  }

  // FIX-4: Do NOT call _tcp.connected() here -- that issues an AT command.
  // The MQTT library calls connected() every 100ms (10 AT commands/sec starves data RX).
  // _connected is set false by read()/write() errors, which is sufficient.
  uint8_t connected() override {
    return _connected ? 1 : 0;
  }
  operator bool() {
    return connected();
  }

  // Use this ONLY in the slow 10-second network status check, not in the MQTT poll path.
  bool tcpConnected() {
    return _tcp.connected();
  }

  // v15.4 Fix B: Full mbedTLS context reset ก่อน reconnect ทุกครั้ง
  // เรียกก่อน mqttClient.connect() เพื่อล้าง stale cipher state จาก session เดิม
  // Root cause: 4G IP เปลี่ยน → TCP drop → old sequence numbers → decryption failed
  // stop() reinit เฉพาะตอน TCP close ซึ่งช้าเกินไป -- ต้อง force reset ก่อน attempt
  void resetTLS() {
    // Stop TCP ก่อน (ถ้ายังเปิดอยู่)
    if (_connected) {
      mbedtls_ssl_close_notify(&_ssl);
      _connected = false;
    }
    _tcp.stop();

    // Free และ reinit per-connection contexts ทั้งหมด
    // (CA cert, client cert, private key ยังคงอยู่ -- parse ครั้งเดียวตอน setupTLS)
    _freeSession();
    mbedtls_ssl_free(&_ssl);
    mbedtls_ssl_config_free(&_conf);
    mbedtls_ssl_init(&_ssl);
    mbedtls_ssl_config_init(&_conf);
    _rxReset();

    Serial.println("[TLS] Context reset complete -- ready for fresh handshake");
  }

private:
  TinyGsm& _modem;
  TinyGsmClient _tcp;
  bool _connected = false;
  bool _certsLoaded = false;

  const char* _ca_pem = nullptr;
  const char* _crt_pem = nullptr;
  const char* _key_pem = nullptr;

  // Persistent mbedTLS contexts -- parsed ONCE, reused on every reconnect
  mbedtls_entropy_context _entropy;
  mbedtls_ctr_drbg_context _drbg;
  mbedtls_x509_crt _ca_crt;
  mbedtls_x509_crt _cli_crt;
  mbedtls_pk_context _pk;

  // Per-connection mbedTLS contexts -- reset on each stop()/connect()
  mbedtls_ssl_context _ssl;
  mbedtls_ssl_config _conf;

  // -- A7670E local RX buffer ------------------------------------------------
  // Root cause (confirmed): A7670E sends "+CADATAIND:0" URC but TinyGSM
  // SIM7600 driver expects "+CIPRXGET:1,0". Data sits in modem hardware buffer
  // (reported by AT+CIPRXGET=4 = available()) but TinyGSM local buffer is empty
  // (read() always returns 0).
  //
  // Previous fix (v1): call AT+CIPRXGET=2,0,N on each bio_recv call.
  // Problem: multiple small calls -> each response has "\r\nOK\r\n" trailer ->
  //          drain(delay=20ms) not sufficient -> "OK\r\n" bytes leak into the
  //          NEXT call's binary stream -> TLS record corrupted ->
  //          MBEDTLS_ERR_PK_INVALID_PUBKEY when parsing server cert public key.
  //
  // This fix (v2): ONE large CIPRXGET fills _rxBuf, bio_recv serves from it.
  //   No AT commands between bio_recv calls = zero corruption risk.
  // -------------------------------------------------------------------------
  static constexpr int RX_BUF_SIZE = 4096;
  uint8_t _rxBuf[RX_BUF_SIZE];
  int _rxHead = 0;  // next byte to serve to mbedTLS
  int _rxTail = 0;  // first free byte (buffered = _rxTail - _rxHead)

  void _rxReset() {
    _rxHead = _rxTail = 0;
  }

  // -- Compact buffer when head advances past halfway --
  void _rxCompact() {
    if (_rxHead == 0) return;
    int rem = _rxTail - _rxHead;
    if (rem > 0) memmove(_rxBuf, _rxBuf + _rxHead, rem);
    _rxHead = 0;
    _rxTail = (rem > 0) ? rem : 0;
  }

  // -- Fill _rxBuf from modem: 2-step CIPRXGET protocol (v3) -------------
  // Bug history:
  //   v1: single CIPRXGET=2 per bio_recv -> "OK\r\n" leaks into next binary read
  //   v2: wrong comma count (3 commas -> "remaining" not "actual")
  //       -> read loop waited 8s for 1460 bytes when only 107 arrived
  //       -> URC +CADATAIND consumed as binary -> data corruption
  //   v3: Step 1 AT+CIPRXGET=4,0 -> exact count; Step 2 fetch exactly that many
  //       -> no timeout, no partial reads, no URC pollution during binary read
  // ---------------------------------------------------------------------
  int _ciprxget(int /*hint*/) {
    _rxCompact();
    int freeSpace = RX_BUF_SIZE - _rxTail;
    if (freeSpace <= 0) return 0;

    // -- Step 1: Query exact bytes in modem hardware buffer --
    SerialAT.printf("AT+CIPRXGET=4,0\r\n");
    int modemCount = _atReadInt("+CIPRXGET:", 2, 3000);
    _drainOK();
    if (modemCount <= 0) return 0;

    // A7670E max per CIPRXGET=2 call: 1460 bytes (one TCP MSS)
    int fetch = min({ modemCount, freeSpace, 1460 });

    // -- Step 2: Fetch exactly 'fetch' bytes --
    SerialAT.printf("AT+CIPRXGET=2,0,%d\r\n", fetch);
    // Response: "+CIPRXGET: 2,0,<actual>,<remaining>\r\n<binary>\r\nOK\r\n"
    // actual = bytes modem actually gives us (may be < fetch if chunk boundary)
    // Comma count for _atReadInt: "+CIPRXGET: 2,0,actual,remaining"
    //   after prefix "+CIPRXGET:": skip 2 commas -> lands on "actual"
    int actual = _atReadInt("+CIPRXGET:", 2, 3000);
    if (actual <= 0) {
      _drainOK();
      return 0;
    }

    // -- Read exactly 'actual' binary bytes into _rxBuf --
    // We know the exact count -- no timeout risk, no URC pollution.
    // 1460 bytes @ 115200 baud = ~127ms. Budget 2s for safety.
    int got = 0;
    uint32_t t = millis();
    while (got < actual && millis() - t < 2000) {
      if (SerialAT.available()) {
        _rxBuf[_rxTail + got] = (uint8_t)SerialAT.read();
        got++;
      } else {
        delayMicroseconds(100);
      }
    }
    _rxTail += got;

    // -- Drain trailing "\r\nOK\r\n" --
    _drainOK();

    // -- HEX DUMP: disabled in production (set to 1 to re-enable during TLS debug) --
#if 0
    {
      static uint32_t s_fetchNo = 0;
      s_fetchNo++;
      int dumpLen = min(got, 32);
      Serial.printf("[CIPRXGET #%lu] +%d/%d bytes (modem had %d)\n",
                    s_fetchNo, got, actual, modemCount);
      Serial.printf("[HEX #%lu] first %d bytes:\n  ", s_fetchNo, dumpLen);
      for (int i = 0; i < dumpLen; i++) {
        Serial.printf("%02X ", _rxBuf[_rxTail - got + i]);
        if ((i + 1) % 16 == 0 && i + 1 < dumpLen) Serial.print("\n  ");
      }
      Serial.println();

      uint8_t* p = _rxBuf + (_rxTail - got);
      if (got >= 5 && p[0] >= 0x14 && p[0] <= 0x17) {
        const char* rec = (p[0] == 0x16) ? "Handshake" : (p[0] == 0x15) ? "Alert"
                                                       : (p[0] == 0x14) ? "ChangeCipherSpec"
                                                                        : "AppData";
        uint16_t rlen = ((uint16_t)p[3] << 8) | p[4];
        Serial.printf("[HEX #%lu] TLS: %s(0x%02X) ver=%02X%02X reclen=%u\n",
                      s_fetchNo, rec, p[0], p[1], p[2], rlen);
        if (p[0] == 0x16 && got >= 6) {
          const char* hs = "?";
          switch (p[5]) {
            case 0x02: hs = "ServerHello"; break;
            case 0x0B: hs = "Certificate"; break;
            case 0x0C: hs = "ServerKeyExchange"; break;
            case 0x0D: hs = "CertificateRequest"; break;
            case 0x0E: hs = "ServerHelloDone"; break;
            case 0x14: hs = "Finished"; break;
          }
          Serial.printf("[HEX #%lu] Handshake: %s(0x%02X)\n",
                        s_fetchNo, hs, p[5]);
        }
        if (p[0] == 0x15 && got >= 7)
          Serial.printf("[HEX #%lu] Alert: lvl=%u desc=%u\n",
                        s_fetchNo, p[5], p[6]);
      } else if (got >= 1) {
        Serial.printf("[HEX #%lu] ! first byte=0x%02X '%c' -- mid-record chunk\n",
                      s_fetchNo, p[0], isprint(p[0]) ? p[0] : '.');
      }
    }
#endif
    return got;
  }

  // -- Helper: read integer from modem AT response after N commas --
  // Waits for a line containing 'prefix', then counts commasBefore commas,
  // returns atoi() of the field that follows.
  int _atReadInt(const char* prefix, int commasBefore, uint32_t timeoutMs) {
    char line[128];
    int llen = 0;
    uint32_t t = millis();
    while (millis() - t < timeoutMs) {
      while (SerialAT.available() && llen < 126) {
        char ch = (char)SerialAT.read();
        if (ch == '\n') {
          line[llen] = '\0';
          const char* p = strstr(line, prefix);
          if (p) {
            p += strlen(prefix);
            int c = 0;
            while (*p && c < commasBefore) {
              if (*p++ == ',') c++;
            }
            return atoi(p);
          }
          llen = 0;
        } else if (ch != '\r') {
          line[llen++] = ch;
        }
      }
      delayMicroseconds(500);
    }
    return -1;
  }

  // -- Helper: drain UART until 80ms of silence --
  void _drainOK() {
    uint32_t quiet = millis();
    while (millis() - quiet < 80) {
      if (SerialAT.available()) {
        SerialAT.read();
        quiet = millis();
      }
    }
  }

  void _freeSession() {
    mbedtls_ssl_free(&_ssl);
    mbedtls_ssl_config_free(&_conf);
  }

  // -- BioCtx carries self pointer so static callback can access buffer --
  struct BioCtx {
    TinyGsmClient* tcp;
    TinyGsm* modem;
    GsmTLSClient* self;
  };
  BioCtx _bioCtx;

  // ==========================================================================
  // _bio_recv -- Buffered A7670E implementation (v2)
  //
  // Flow:
  //  1. Serve bytes from _rxBuf if available       <- no AT command, fast
  //  2. Pump modem + check tcp.available()
  //  3. Fill _rxBuf via ONE CIPRXGET call          <- single AT round-trip
  //  4. Serve from freshly filled buffer
  //
  // This eliminates the "OK\r\n leaks into next call's binary data" corruption
  // that caused MBEDTLS_ERR_PK_INVALID_PUBKEY in v1.
  // ==========================================================================
  static int _bio_recv(void* ctx, unsigned char* buf, size_t len) {
    BioCtx* c = (BioCtx*)ctx;
    GsmTLSClient* self = c->self;

    // -- Step 1: Serve from local buffer --
    int buffered = self->_rxTail - self->_rxHead;
    if (buffered > 0) {
      int n = (int)min((size_t)buffered, len);
      memcpy(buf, self->_rxBuf + self->_rxHead, n);
      self->_rxHead += n;
      if (self->_rxHead >= self->_rxTail) self->_rxReset();
      return n;
    }

    // -- Step 2: Pump modem, check for data --
    for (int i = 0; i < 5; i++) c->modem->maintain();
    int avail = c->tcp->available();
    if (avail <= 0) return MBEDTLS_ERR_SSL_WANT_READ;

    // -- Step 3: Fill local buffer via one CIPRXGET call --
    int fetched = self->_ciprxget(avail);
    if (fetched <= 0) return MBEDTLS_ERR_SSL_WANT_READ;

    // -- Step 4: Serve from freshly filled buffer --
    int n = (int)min((size_t)fetched, len);
    memcpy(buf, self->_rxBuf + self->_rxHead, n);
    self->_rxHead += n;
    if (self->_rxHead >= self->_rxTail) self->_rxReset();
    return n;
  }

  // -- TLS debug callback -- routes mbedTLS trace to Arduino Serial --
  static void _tls_debug_cb(void* /*ctx*/, int level,
                            const char* file, int line, const char* str) {
    const char* f = strrchr(file, '/');
    if (!f) f = strrchr(file, '\\');
    f = f ? f + 1 : file;
    Serial.printf("[mbedTLS L%d] %s:%d: %s", level, f, line, str);
  }

  static int _bio_send(void* ctx, const unsigned char* buf, size_t len) {
    BioCtx* c = (BioCtx*)ctx;
    size_t written = c->tcp->write(buf, len);
    return (written > 0) ? (int)written : MBEDTLS_ERR_SSL_WANT_WRITE;
  }
};

// Global TLS client -- ESP32 mbedTLS over 4G modem TCP (same engine as WiFiClientSecure)
GsmTLSClient gsmClient(modem);

// MQTT TX/RX buffer must be larger than the biggest PUBLISH packet:
// Phase 3 JSON payload <= 2600 bytes + topic (~55) + MQTT header (4) + QoS1 msgid (2) = ~2661 bytes
// Set to 2800 for comfortable headroom.
MQTTClient mqttClient(3600);  // v15.7: 2800→3600 (prevent err=-5/-6 BUFFER_TOO_SHORT on burst publish)

// Display and Modbus
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, I2C_SCL_PIN, I2C_SDA_PIN);
ModbusMaster modbus;

// RTC
RTC_DS3231 rtc;

// MQTT Topic (built from identity defines at compile time)
static char g_mqttTopic[128];  // populated in setup()

// ============================================================================
// ENUMERATIONS
// ============================================================================

typedef enum {
  STATE_NORMAL = 0,
  STATE_WARNING,
  STATE_CRITICAL,
  STATE_MAINTENANCE,
  STATE_WARMUP
} MachineState_t;

typedef enum {
  PAGE_MACHINE = 0,
  PAGE_AXIS,
  PAGE_NETWORK,
  PAGE_MAX
} DisplayPage_t;

typedef enum {
  BTN_NONE = 0,
  BTN_SHORT_PRESS,
  BTN_LONG_PRESS,
  BTN_VLONG_PRESS
} ButtonEvent_t;

typedef enum {
  MODEM_STATE_OFF = 0,
  MODEM_STATE_INITIALIZING,
  MODEM_STATE_SEARCHING,
  MODEM_STATE_REGISTERED,
  MODEM_STATE_GPRS_CONNECTING,
  MODEM_STATE_GPRS_CONNECTED,
  MODEM_STATE_ERROR
} ModemState_t;

// Motor run-state (derived from proximity sensor pulse timing)
typedef enum {
  MOTOR_STOPPED  = 0,   // ???? / ????? pulse
  MOTOR_STARTING = 1,   // ????? start (rpm ????????? rated band)
  MOTOR_RUNNING  = 2,   // RUNNING -- rpm ?????? RATED_RPM +/- RATED_RPM_TOL
  MOTOR_STOPPING = 3    // ????? stop (pulse ??????????????)
} MotorRunState_t;

// ============================================================================
// DATA STRUCTURES
// ============================================================================

// Sensor data (shared between cores)
typedef struct {
  // v15.7: rms_x/y/z อ่านจาก sensor VRMS register โดยตรง (0x50/0x5C/0x68 ÷ 1000)
  // ไม่ต้องคำนวณเอง -- sensor คำนวณจาก raw FIFO 16KHz ภายใน chip
  float rms_x;           // velocity RMS X [mm/s] = VRMSX reg / 1000
  float rms_y;           // velocity RMS Y [mm/s] = VRMSY reg / 1000
  float rms_z;           // velocity RMS Z [mm/s] = VRMSZ reg / 1000
  float rms_overall;     // true 3-axis vector RMS [mm/s] = sqrt(x²+y²+z²) [v15.8]

  // --- Sensor-computed features [v15.0/15.1] ---
  // คำนวณภายใน chip จาก 16KHz FIFO ถูกต้องกว่าคำนวณบน ESP32
  float cf_x;            // Acceleration Crest Factor X (reg 0x47 / 1000) -- Peak/RMS acc
  float cf_y;            // Acceleration Crest Factor Y (reg 0x53 / 1000) [v15.1]
  float cf_z;            // Acceleration Crest Factor Z (reg 0x5F / 1000) [v15.1]
  float cf_max;          // max(cf_x, cf_y, cf_z)                         [v15.1]

  float kurtosis_x;      // Acceleration Kurtosis X (reg 0x48 / 1000) -- bearing impact
  float kurtosis_y;      // Acceleration Kurtosis Y (reg 0x54 / 1000) [v15.1]
  float kurtosis_z;      // Acceleration Kurtosis Z (reg 0x60 / 1000) [v15.1]
  float kurtosis_max;    // max(kurtosis_x, y, z) -- ใช้ใน bearing alert  [v15.1]
  uint8_t kurtosis_dominant_axis; // 0=X, 1=Y, 2=Z (แกนที่ kurtosis_max มาจาก) [v15.1]

  // v15.7: peak field ลบออก -- ไม่มี peak register บน sensor
  float freq_x;
  float freq_y;
  float freq_z;
  float temperature;
  uint32_t timestamp;
  bool valid;
  // --- RPM / Motor fields (from proximity sensor) ---
  float    rpm;           // Actual filtered RPM (0.0 when stopped)
  uint8_t  motor_state;   // MotorRunState_t: 0=STOPPED,1=STARTING,2=RUNNING,3=STOPPING
  float    runtime_hour;  // Accumulated running hours (NVS persistent)
  uint8_t  prox;          // 1=pulse normal, 0=Fault/?????? pulse
} VibrationData_t;

// System state (shared between cores)
typedef struct {
  MachineState_t state;
  DisplayPage_t currentPage;
  bool alarmAcknowledged;
  bool buzzerActive;
  bool blinkState;
  uint32_t stateEntryTime;
} SystemState_t;

// Network status (Core 1 only) - Modified for 4G
typedef struct {
  bool modemReady;
  bool gprsConnected;
  ModemState_t modemState;
  int16_t signalQuality;  // CSQ value (0-31, 99=unknown)
  int8_t signalPercent;   // Signal strength in %
  uint32_t lastPublishTime;
  uint16_t publishCount;
  uint16_t publishFailures;
  char operatorName[20];
  char imei[20];
} NetworkStatus_t;

// Time synchronization status
typedef struct {
  bool synced;              // Has time been synced at least once?
  bool ntpReachable;        // Was last NTP/network-time fetch successful?
  uint32_t lastSyncMillis;  // millis() of last successful sync
  uint32_t syncCount;       // Total successful syncs
  uint32_t syncFailures;    // Total failed sync attempts
  int32_t lastDriftSec;     // Drift detected at last sync (seconds)
  char lastSyncTime[25];    // Human-readable last sync time (ISO 8601)
} TimeSyncStatus_t;

// Display update command
typedef struct {
  DisplayPage_t page;
  MachineState_t state;
  bool forceUpdate;
} DisplayCommand_t;

// Maintenance reset event (Button task -> Network task via queue)
// Carries all data needed for MQTT audit publish, built on Button stack-free.
typedef struct {
  uint32_t triggerMillis;   // millis() when reset was triggered
  bool     rtcValid;        // snapshot of g_rtcValid at trigger time
  uint16_t year;            // RTC fields (copied to avoid RTC access on Network task)
  uint8_t  month;
  uint8_t  day;
  uint8_t  hour;
  uint8_t  minute;
  uint8_t  second;
} MaintenanceEvent_t;

// ============================================================================
// FREERTOS HANDLES
// ============================================================================

// Task Handles
TaskHandle_t taskHandleModbus    = NULL;
TaskHandle_t taskHandleState     = NULL;
TaskHandle_t taskHandleDisplay   = NULL;
TaskHandle_t taskHandleNetwork   = NULL;
TaskHandle_t taskHandleButton    = NULL;
TaskHandle_t taskHandleBuzzer    = NULL;
TaskHandle_t taskHandleAnalytics = NULL;  // Phase 2: analytics task

// Queue Handles
QueueHandle_t queueSensorData = NULL;
QueueHandle_t queueButtonEvent = NULL;
QueueHandle_t queueDisplayUpdate = NULL;
QueueHandle_t queueMaintEvent = NULL;   // V14.4: maintenance reset (Button -> Network)

// Mutex Handles
SemaphoreHandle_t mutexVibData    = NULL;
SemaphoreHandle_t mutexSystemState = NULL;
SemaphoreHandle_t mutexI2C        = NULL;
SemaphoreHandle_t mutexModem      = NULL;  // Mutex for modem access
SemaphoreHandle_t mutexAggBufs    = NULL;  // Phase 2: protects g_buf1s/10s/60s (taskAnalytics ? taskNetwork)

// ============================================================================
// SHARED VARIABLES (protected by mutex)
// ============================================================================

static VibrationData_t g_vibData = { 0 };
static SystemState_t g_systemState = {
  .state = STATE_NORMAL,
  .currentPage = PAGE_MACHINE,
  .alarmAcknowledged = false,
  .buzzerActive = false,
  .blinkState = false,
  .stateEntryTime = 0
};
static NetworkStatus_t g_network = { 0 };
static TimeSyncStatus_t g_timeSync = { 0 };

// Statistics (atomic operations, no mutex needed)
static volatile uint32_t g_sensorReads = 0;
static volatile uint32_t g_sensorErrors = 0;
static volatile uint32_t g_displayUpdates = 0;

// v15.3: Reset reason (อ่านจาก hardware ตอน boot, persistent via NVS)
static char     g_resetReasonStr[24] = "UNKNOWN";  // human-readable string
static uint8_t  g_resetReasonCode    = 0;           // esp_reset_reason_t value
static uint32_t g_rebootCount        = 0;           // สะสมข้ามรอบ (NVS)

// Modbus offline detection
static volatile uint8_t  g_modbusConsecErrors = 0;  // ??? error ?????????
static volatile bool     g_sensorOffline      = false; // true = sensor ?????/??????????

// v15.7: g_velPeakHold ลบออก -- peak field ถูกลบออกจาก VibrationData_t แล้ว

// ============================================================================
// TREND BUFFER -- Phase 1: Raw Circular Buffer (Core 0 writes / Core 1 reads)
// ============================================================================
//
// Layer 1 (Raw Circular Buffer):
//   ???? sample ??? 250ms ??? taskStateMachine (Core 0)
//   ???? TREND_BUF_SIZE = 240 samples = 60 ?????? @ 4 Hz
//   RAM: 240 x (6 float) x 4 bytes = 5,760 bytes (~5.6 KB)
//
// Layer 2 (Trend Engine -- ???????? publish, Core 1):
//   ??? window ???????? TREND_WINDOW_SAMPLES samples ??? buffer
//   ?????: RMS slope (Linear Regression), Peak spike count,
//           Freq ratio drift (X/Y/Z), Temp slope, TTW estimate
//
// Thread safety:
//   g_trendBuf ???????? Core 0 (taskStateMachine) ????????
//   ??????? Core 1 (taskAnalytics, publishTelemetry) ?????? snapshot ??? head+count
//   ???????? -- float write ???? atomic ?? ESP32 (Xtensa LX7)
// ============================================================================

// --- Trend Buffer Configuration ---
#define TREND_BUF_SIZE        240    // samples (60s @ 4Hz) -- Raw circular buffer
#define TREND_WINDOW_SAMPLES  120    // samples ??????????? trend (30s window)
#define TREND_MIN_SAMPLES      20    // ??????????????? 20 samples (5s) ?????????

// Thresholds ?????? Trend Engine
#define TREND_SLOPE_UP      0.002f   // mm/s per sample -> "UP"   (0.008 mm/s/s)
#define TREND_SLOPE_DOWN   -0.002f   // mm/s per sample -> "DOWN"
#define SPIKE_RMS_FACTOR    1.5f     // peak > WARNING_RMS x 1.5 -> ??? spike
#define FREQ_DRIFT_THRESH   1.00f    // freq_ratio drift > 1.0x -> drift detected
                                     // v15.7: 0.15→1.0 -- sensor FREQ register resolution
                                     // ~0.1 Hz at 3000 RPM = 0.002x/step, observed
                                     // natural variation ±0.36x → 0.15 was too tight
                                     // 1.0x means true load/speed shift, not noise
#define TEMP_SLOPE_WARN     0.001f   //  degC per sample -> temp rising (0.004 degC/s)

// --- Trend Sample Struct (Layer 1) ---
typedef struct {
  float rms;           // rms_overall [mm/s] -- sustained vibration level
  float rms_max;       // max rms in push window [mm/s] -- for impulse/spike detection
                       // v15.7: renamed from 'peak' (was incorrectly set = rms_overall)
                       // impulse detection requires momentary peak, not sustained rms
  float temp;          // temperature [ degC]
  float freq_ratio_x;  // freq_x / (rpm/60) -- harmonic order
  float freq_ratio_y;
  float freq_ratio_z;
} TrendSample_t;

// --- Trend Buffer Globals (Core 0 writes / Core 1 reads) ---
static TrendSample_t     g_trendBuf[TREND_BUF_SIZE];
static volatile uint16_t g_trendHead  = 0;
static volatile uint16_t g_trendCount = 0;

// ============================================================================
// MULTI-RESOLUTION AGGREGATION BUFFERS -- Phase 2
// ============================================================================
//
// taskAnalytics (Core 1) reads g_trendBuf every 1 s -> aggregates -> pushes to
// 3 circular buffers:
//
//   g_buf1s [60]  -- 1 slot = 1 s   -> 60 s of 1-second averages
//   g_buf10s[60]  -- 1 slot = 10 s  -> 10 min of 10-second averages
//   g_buf60s[60]  -- 1 slot = 60 s  -> 60 min of 60-second averages
//
// RAM: sizeof(AggSample_t) x 60 x 3 ? 8.6 KB
//
// Thread safety:
//   taskAnalytics WRITES g_buf* (Core 1)
//   taskNetwork (calcTrend) READS g_buf* (Core 1)
//   Both on same core -> FreeRTOS preemption CAN interleave struct writes
//   -> protected by mutexAggBufs (lightweight, held <1 ms each direction)
// ============================================================================

// --- Aggregated Sample Struct ---
typedef struct {
  float   mean_rms;      // mean rms_overall per slot [mm/s]
  float   max_rms;       // max rms_overall per slot
  float   stddev_rms;    // standard deviation of rms (volatility)
  float   mean_temp;     // mean temperature [degC]
  float   max_temp;      // max temperature
  float   mean_rms_max;  // mean of per-sample rms_max (impulse envelope) [mm/s]
                         // v15.7: renamed from mean_peak -- now carries true impulse max
  float   max_rms_max;   // max of per-sample rms_max in slot -- peak impulse
                         // v15.7: renamed from max_peak
  float   mean_frx;      // mean freq_ratio_x
  float   mean_fry;
  float   mean_frz;
  uint8_t spike_count;   // samples where rms_max > WARNING_RMS x SPIKE_RMS_FACTOR
                         // v15.7: counts impulse spikes (rms_max), not sustained rms
  uint8_t n_samples;     // raw samples in aggregate (debug)
  // pad to 4-byte aligned: 10xfloat(40) + 2xuint8(2) + 2 pad = 44 bytes
} AggSample_t;

// --- Buffer sizes ---
#define AGG_BUF_1S_SIZE    60   // 60 slots x 1 s  =  60 s  history
#define AGG_BUF_10S_SIZE   60   // 60 slots x 10 s = 600 s  history (10 min)
#define AGG_BUF_60S_SIZE   60   // 60 slots x 60 s = 3600 s history (60 min)

// --- Aggregated circular buffers (taskAnalytics writes / calcTrend reads, both Core 1) ---
static AggSample_t       g_buf1s [AGG_BUF_1S_SIZE];
static volatile uint16_t g_buf1sHead  = 0;
static volatile uint16_t g_buf1sCount = 0;

static AggSample_t       g_buf10s[AGG_BUF_10S_SIZE];
static volatile uint16_t g_buf10sHead  = 0;
static volatile uint16_t g_buf10sCount = 0;

static AggSample_t       g_buf60s[AGG_BUF_60S_SIZE];
static volatile uint16_t g_buf60sHead  = 0;
static volatile uint16_t g_buf60sCount = 0;

// --- EMA state (taskAnalytics writes, Core 1 only -- no cross-core issue) ---
#define EMA_ALPHA          0.20f   // ? = 0.20 -> ? ? 4 samples (4s @ 1Hz)
#define EMA_DIR_THRESHOLD  0.003f  // |delta| > 3 ?m/s per 1s update -> direction
static float   g_emaRms     = 0.0f;  // EMA ??? rms_overall [mm/s]
static float   g_emaPrevRms = 0.0f;  // EMA ??????? (???????? direction)
static float   g_emaDelta   = 0.0f;  // g_emaRms ? g_emaPrevRms [mm/s per 1s]
static int8_t  g_emaDir     = 0;     // +1=UP  0=STABLE  -1=DOWN

// --- Analytics MQTT topic (built at setup) ---
static char g_mqttAnalyticsTopic[128];

// ── MQTT Pipeline Topics (mqtt_pipeline_patch §A) ────────────────────────────
static char g_mqttTopicSensor    [128];  // factory/.../sensor
static char g_mqttTopicDecision  [128];  // factory/.../decision
static char g_mqttTopicTrend     [128];  // factory/.../trend
static char g_mqttTopicFusionRaw [128];  // factory/.../fusion/raw
static char g_mqttTopicFusionAedf[128];  // factory/.../fusion/aedf
static char g_mqttTopicFusionOsg [128];  // factory/.../fusion/osg
static char g_mqttTopicFusionFvri[128];  // factory/.../fusion/fvri
static char g_mqttTopicTtwModel  [128];  // factory/.../ttw/model
static char g_mqttTopicEvent     [128];  // factory/.../vibration/event  (V14.4)
// /output (5-min) reuses g_mqttPredictionTopic — no new array needed
// ─────────────────────────────────────────────────────────────────────────────

// ============================================================================
// TREND ENGINE OUTPUT -- populated by calcTrend(), read by publishTelemetry()
// ============================================================================

// -- Phase 3: Fault Classification thresholds -----------------------------
// Freq ratio tolerance bands -- ratio = dominant_freq / (rpm/60)
#define FAULT_RATIO_TOL      0.15f   // +/-0.15x tolerance for integer harmonic match (1x/2x/3x)
#define BPF_RATIO_TOL_WIDE   0.50f   // wider tolerance for BPF matching (pump blade harmonics)
                                     // BPF freq varies with load/speed → wider window needed
#define FAULT_SUB_LOW        0.35f   // sub-harmonic band low  (oil whirl ~0.40-0.48x)
#define FAULT_SUB_HIGH       0.55f   // sub-harmonic band high
#define FAULT_IMBALANCE_CTR  1.00f   // 1x = imbalance / bow
#define FAULT_MISALIGN_CTR   2.00f   // 2x = misalignment / looseness
#define FAULT_LOOSE_CTR      3.00f   // 3x = looseness
// Blade Pass Frequency: for N-blade pump, BPF = N x shaft freq -> ratio = N
// NAMEPLATE_RPM already defined -- use PUMP_BLADES to identify expected BPF ratio
#define PUMP_BLADES          4       // number of pump impeller blades (adjust per machine)
// RESONANCE: only fire above BPF+1x -- prevents inter-harmonic pump noise from scoring
// For 4-blade pump: ratio must exceed 5.0x (above BPF=4x zone)
// Observed: normal pump operation gives frx=4.7-5.0x which is inter-BPF noise, not resonance
#define RESONANCE_RATIO_MIN  ((float)PUMP_BLADES + 1.0f)  // 5.0x for 4-blade pump
// Bearing: non-integer ratio + spike pattern (no BSF/BPFI without bearing DB)
#define FAULT_BEARING_SPIKE  5       // spike_count >= 5 within window -> bearing candidate
#define FAULT_VOLAT_LOOSE    0.30f   // stddev_1min > 0.30 mm/s -> looseness candidate
// Multi-resolution agreement: how many resolutions must trend UP for rising_multiRes
#define FAULT_AGREE_THRESH   2       // >=2 of 3 slopes positive -> multi-res rising

// Minimum buffer depth before slope is considered valid
#define SLOPE_1S_MIN_SLOTS    10   // 10 s warmup
#define SLOPE_10S_MIN_SLOTS   10   // ~100 s warmup
#define SLOPE_60S_MIN_SLOTS   20   // ~20 min warmup

// -- Phase 4: TTW ensemble weights ---------------------------------------
// short=1s window (noisy, reactive),  medium=10s (balanced),  long=60s (stable)
// When all 3 available: long gets highest weight (most reliable)
#define TTW_W_SHORT   0.20f    // 20% weight for slope_1s-derived TTW
#define TTW_W_MEDIUM  0.30f    // 30% weight for slope_10s-derived TTW
#define TTW_W_LONG    0.50f    // 50% weight for slope_60s-derived TTW
#define TTW_MAX_HOURS 168.0f   // Cap at 1 week (168h) -- beyond this TTW is not useful

// Minimum rate for long-term slope to compute TTW (lower than short -- less noise)
#define TTW_LONG_MIN_RATE_H   0.10f  // mm/s/h -- half of kMinRateH (more sensitive)

// Phase 4: /prediction topic publish interval
#define PUBLISH_PREDICTION_INTERVAL_S  300  // publish /prediction every 5 min

// -- Phase 5: Probabilistic Fault Classifier --------------------------------
#define FAULT_TYPE_COUNT     7      // NORMAL=0 ? UNKNOWN=6 (must match FaultType_t)

// -- Phase 5: Adaptive Evidence Decay Function (AEDF) ----------------------
// Decay tau in "number of publish intervals" for each fault type.
// Faster tau = evidence forgets quickly (acute faults like BEARING).
// Slower tau = evidence persists (chronic faults like IMBALANCE).
#define AEDF_TAU_NORMAL     10.0f   // normal state persists
#define AEDF_TAU_IMBALANCE   8.0f
#define AEDF_TAU_MISALIGN    6.0f
#define AEDF_TAU_LOOSENESS   4.0f
#define AEDF_TAU_BEARING     2.0f   // bearing onset is acute -- forget fast
#define AEDF_TAU_RESONANCE   5.0f
#define AEDF_TAU_UNKNOWN     1.0f   // unknown is transient
#define AEDF_BLEND           0.40f  // IIR: 60% history + 40% new score
#define AEDF_CONTRADICT      0.35f  // rival score margin triggers accelerated reset
#define AEDF_RESET_MUL       0.30f  // multiply evidence on contradiction detected
#define AEDF_PERSIST_THRESH  0.50f  // evidence above this boosts effective score

// -- Phase 5: Oscillatory Suppression Gate (OSG) ---------------------------
#define OSG_FLOOR            0.05f  // minimum suppression multiplier (never full block)

// -- Phase 5: FVRI variance thresholds (in (mm/s per 1s slot)^2) ----------
// These are empirical thresholds for the g_slopeVar_* values
#define FVRI_VAR_BEARING     1e-8f  // above this -> BEARING var is signal, invert penalty
#define FVRI_VAR_LOOSENESS   5e-9f  // above this -> LOOSENESS suppress short TTW
#define FVRI_VAR_IMBALANCE   2e-9f  // above this -> IMBALANCE penalize (unexpected noise)

// -- Phase 5: Typed Conflict Resolution ------------------------------------
#define FUSION_FAULT_SCORE_MIN    0.60f  // fault must exceed this to override pred=0
#define FUSION_HIGH_SEV           70     // severity >= this forces pred>=1 (Type C)
#define FUSION_UNCERTAINTY_MAX    0.40f  // fault_uncertainty above this -> Type D
#define FUSION_UNCERTAINTY_PENALTY 0.85f // Type D severity multiplier (15% cut)
#define FUSION_TYPE_D_CONF_MIN    60     // Type D: conf must exceed for pred=2

// -- Phase 5: Context-Conditioned TTW Threshold ----------------------------
#define CTX_THRESH_BASE       1.0f  // base TTW threshold for pred=2 [hours]
#define CTX_THRESH_BEARING    2.0f  // BEARING: lower urgency threshold (longer lead time)
#define CTX_THRESH_CRITICAL   0.5f  // very high severity: require TTW < 30 min for pred=2

// -- Fault type & Alarm class string helpers --------------------------------
// (Enums declared at top of file, before all #defines, to fix Arduino IDE
//  prototype-scan ordering. See "FAULT & ALARM ENUMERATIONS" section above.)
static const char* faultTypeStr(FaultType_t ft) {
  switch (ft) {
    case FAULT_NORMAL:       return "NORMAL";
    case FAULT_IMBALANCE:    return "IMBALANCE";
    case FAULT_MISALIGNMENT: return "MISALIGNMENT";
    case FAULT_LOOSENESS:    return "LOOSENESS";
    case FAULT_BEARING:      return "BEARING";
    case FAULT_RESONANCE:    return "RESONANCE";
    default:                 return "UNKNOWN";
  }
}

static const char* alarmClassStr(AlarmClass_t ac) {
  switch (ac) {
    case ALARM_SILENT:        return "SILENT";
    case ALARM_INFORMATIONAL: return "INFO";
    case ALARM_WARNING:       return "WARNING";
    case ALARM_CRITICAL:      return "CRITICAL";
    default:                  return "SILENT";
  }
}

// -- Decision Engine output struct ----------------------------------------
typedef struct {
  FaultType_t  fault_type;          // probabilistic top candidate
  float        fault_score;         // top candidate Gaussian score 0-1
  float        fault_uncertainty;   // 1 ? (top ? second) score gap 0-1
  uint8_t      severity_score;      // 0-100 adaptive weighted
  uint8_t      confidence;          // 0-100% orthogonal geometric mean
  uint8_t      predicted_state;     // 0/1/2 raw pre-fusion
  uint8_t      final_state;         // 0/1/2 post-fusion (typed conflict resolved)
  AlarmClass_t alarm_class;         // SILENT/INFO/WARNING/CRITICAL
  float        ttw_short_h;         // TTW from slope_1s  [hours]
  float        ttw_medium_h;        // TTW from slope_10s [hours]
  float        ttw_long_h;          // TTW from slope_60s [hours]
  float        ttw_best_h;          // Variance-weighted ensemble TTW
  float        ttw_w_s;             // adaptive weight for short
  float        ttw_w_m;             // adaptive weight for medium
  float        ttw_w_l;             // adaptive weight for long
  bool         action_needed;
  uint32_t     publish_interval_ms; // Phase 5: adaptive interval
  char         reason[48];          // increased from 40
} DecisionResult_t;

static DecisionResult_t g_decision = {
  FAULT_UNKNOWN, 0.0f, 1.0f, 0, 0, 0, 0,
  ALARM_SILENT, 0, 0, 0, 0, 0, 0, 0, false, 30000, "init"
};

// Phase 4: /prediction topic
static char g_mqttPredictionTopic[128];

// -- Phase 5: Slope variance trackers (updated in taskAnalytics) ----------
// Units: (mm/s per 1s slot)^2  -- used by OSG and FVRI
static volatile float g_slopeVar_1s  = 1e-6f;  // initialized to small nonzero
static volatile float g_slopeVar_10s = 1e-6f;
static volatile float g_slopeVar_60s = 1e-6f;

// -- Patch B: OSG*FVRI reliability multipliers (stored for /fusion/osg pub) --
// r_s and r_m are the post-FVRI, post-OSG reliability weights fed into TTW.
// Written by runDecisionEngine() after OSG suppression is applied.
static volatile float g_osg_r_s = 1.0f;  // reliability for short (1s) TTW tier
static volatile float g_osg_r_m = 1.0f;  // reliability for medium (10s) TTW tier

// -- Maintenance reset: warmup phase start timestamp -----------------------
static volatile uint32_t g_warmupStartTs = 0;  // millis() at last maintenance reset

// -- Analytics task internal state -- promoted to globals for maintenance reset --
// V14.4: must be resettable by taskButtonHandler (via vTaskSuspend guard).
// acc* : millis-based slot accumulators (Patent Claim 2)
static volatile uint32_t g_accMs_1s  = 0;   // ms accumulated since last buf1s flush
static volatile uint32_t g_accMs_10s = 0;   // ms accumulated since last buf10s flush
static volatile uint32_t g_accMs_60s = 0;   // ms accumulated since last buf60s flush
// lastHead / firstRun: raw-buffer consumption bookkeeping
static volatile uint16_t g_anaLastHead  = 0;
static volatile bool     g_anaFirstRun  = true;
static volatile uint8_t  g_anaPublishCnt = 0;  // ticks since last /trend publish
// Per-slot running accumulators (flushed every g_slotDur1sMs ms)
static volatile float    g_sl_sumRms   = 0.0f, g_sl_sumSqRms = 0.0f, g_sl_maxRms  = 0.0f;
static volatile float    g_sl_sumTemp  = 0.0f, g_sl_maxTemp  = 0.0f;
static volatile float    g_sl_sumPeak  = 0.0f, g_sl_maxPeak  = 0.0f;
static volatile float    g_sl_sumFrx   = 0.0f, g_sl_sumFry   = 0.0f, g_sl_sumFrz  = 0.0f;
static volatile uint8_t  g_sl_spikes   = 0;
static volatile uint8_t  g_sl_n        = 0;

// ============================================================================
// PATENT CLAIM 2: Dynamic Slot Duration (RPM-Adaptive Buffer Resolution)
// ============================================================================
// Patent ข้อ 2 ระบุ: "ปรับระยะเวลา Slot อย่างน้อยหนึ่งรายการแบบพลวัตตามความเร็วรอบ
// ที่วัดได้ของเครื่องจักร เพื่อให้ความละเอียดเชิงเวลาของ Buffer ปรับตามความเร็วการทำงาน
// ปัจจุบัน จึงให้ครอบคลุมรอบฮาร์โมนิกที่สม่ำเสมอสำหรับเครื่องจักรที่ทำงานที่ความเร็วพิกัดต่างกัน"
//
// Implementation strategy:
//   - g_slotDur1sMs: effective duration of one buf1s slot in ms (adaptive)
//   - At RATED_RPM → 1000ms (1 revolution per ~41ms × ~24 revs/slot = good harmonic coverage)
//   - Below RATED_RPM → slot widens (more ms) to cover same number of revolutions
//   - Above RATED_RPM → slot narrows (fewer ms) to avoid over-sampling
//   - Clamped: [SLOT_DUR_MIN_MS .. SLOT_DUR_MAX_MS] to stay sane
//   - buf10s and buf60s slot durations scale proportionally via their cascade counters
//
// The taskAnalytics loop runs at fixed 1000ms ticks (FreeRTOS unchanged).
// Adaptive behaviour is implemented with a millis()-based accumulator:
//   acc1s += 1000 each tick; flush when acc1s >= g_slotDur1sMs.
// This keeps FreeRTOS timing deterministic while delivering variable-length slots.
// ============================================================================
#define SLOT_DUR_MIN_MS     400UL   // floor: ~24 Hz max-RPM machines
#define SLOT_DUR_MAX_MS    3000UL   // ceiling: very slow machines / startup
#define SLOT_REVS_TARGET      24    // target revolutions covered per 1s slot
// Derived: slotDur = clamp(SLOT_REVS_TARGET * 60000 / rpm, MIN, MAX)
// At 1450 RPM → 60000/1450*24 ≈ 993 ms ≈ 1000ms (matches original default)
// At  900 RPM → 60000/ 900*24 ≈ 1600 ms  (widens to cover 24 revs)
// At 1800 RPM → 60000/1800*24 ≈  800 ms  (narrows)

static volatile uint32_t g_slotDur1sMs = 1000UL;  // effective 1s-slot duration [ms]

// -- Phase 5: AEDF -- per-fault evidence accumulators ----------------------
static float    g_faultEvidence[FAULT_TYPE_COUNT] = {0.0f};
static uint32_t g_lastEvidenceMs = 0;

// -- Patch A: pre-AEDF raw Gaussian scores (stored for /fusion/raw pub) ----
// Written by runDecisionEngine() immediately after classifyFaultProbabilistic().
static float g_rawScores[FAULT_TYPE_COUNT] = {0.0f};

// -- Phase 5: TTW hysteresis and state damping -----------------------------
static float   g_prevTtwBest      = 0.0f;
static float   g_prevTtwRoC       = 0.0f;  // V14.6: promoted from static local in runDecisionEngine
static uint8_t g_prevFinalState   = 0;
static uint8_t g_stateChangeCyc   = 0;    // cycles since last state change (for publish boost)

// -- TrendResult_t -- complete output struct -------------------------------
typedef struct {
  // -- Phase 1 fields (30s single-resolution) ------------------------------
  float    rms_slope;       // mm/s per sample (+= rising, -= falling)
  float    temp_slope;      //  degC per sample
  int8_t   trend_dir;       // +1=UP  0=STABLE  -1=DOWN  (from linreg slope)
  uint16_t spike_count;     // peak > WARNINGx1.5 ?? 30s window
  float    freq_drift_x;    // harmonic drift X
  float    freq_drift_y;
  float    freq_drift_z;
  bool     freq_alert;      // drift > FREQ_DRIFT_THRESH >=1 axis
  float    ttw_hours;       // Phase1 TTW from raw slope [h]
  uint16_t window_samples;  // ????? samples ?????????? (debug)

  // -- Phase 2 fields (multi-resolution) -----------------------------------
  float  slope_1s;          // linreg ?? g_buf1s 30 slots  [mm/s per 1s slot]
  float  slope_10s;         // linreg ?? g_buf10s 30 slots [mm/s per 10s slot]
  float  slope_60s;         // linreg ?? g_buf60s 30 slots [mm/s per 60s slot]
  bool   slope_ready_1s;    // true = buf1s >= SLOPE_1S_MIN_SLOTS
  bool   slope_ready_10s;   // true = buf10s >= SLOPE_10S_MIN_SLOTS
  bool   slope_ready_60s;   // true = buf60s >= SLOPE_60S_MIN_SLOTS
  int8_t ema_dir;           // snapshot g_emaDir  (+1/0/-1)
  float  ema_rms;           // snapshot g_emaRms  [mm/s]
  float  ema_delta;         // snapshot g_emaDelta [mm/s per 1s]
  float  stddev_1min;       // mean stddev_rms ??? g_buf1s 60 slots
  float  max_rms_10min;     // peak max_rms ??? g_buf10s 60 slots

  // -- Phase 3 fields (decision engine output) -----------------------------
  FaultType_t fault_type;   // classified fault (mirrors g_decision.fault_type)
} TrendResult_t;

static TrendResult_t g_trendResult = { 0 };  // ?? trend ??????

// RTC valid flag
static bool g_rtcValid = false;

// ============================================================================
// PROXIMITY / RPM -- ISR Variables (volatile, written in IRAM ISR)
// ============================================================================

static const uint32_t RPM_MIN_INTERVAL_US =
    60000000UL / (MAX_RPM * PULSE_PER_REV);
static const uint32_t RPM_DEBOUNCE_US =
    RPM_MIN_INTERVAL_US / 2;

volatile uint32_t g_rpmLastPulseTime  = 0;
volatile uint32_t g_rpmPulseInterval  = 0;
volatile uint32_t g_rpmTotalPulses    = 0;

// RPM processing state (Core 0 only -- no mutex needed)
static float           g_rpmFiltered       = 0.0f;
static uint32_t        g_rpmLastPulseCount  = 0;
static uint32_t        g_rpmLastPulseMillis = 0;
static MotorRunState_t g_motorRunState      = MOTOR_STOPPED;
static MotorRunState_t g_prevMotorRunState  = MOTOR_STOPPED;  // v15.2: track transition

// Runtime hour accumulation (NVS persistent)
static Preferences g_motorPrefs;
static float       g_runtimeHour    = 0.0f;
static uint32_t    g_runningStartMs = 0;
static bool        g_prevWasRunning = false;
static uint32_t    g_lastNvsSaveMs  = 0;

// ISR -- IRAM_ATTR: runs from IRAM, safe from Cache miss
void IRAM_ATTR rpmISR() {
  uint32_t now      = micros();
  uint32_t interval = now - g_rpmLastPulseTime;
  if (interval > RPM_DEBOUNCE_US) {
    g_rpmPulseInterval = interval;
    g_rpmLastPulseTime = now;
    g_rpmTotalPulses++;
  }
}

// NVS: ???? runtime_hour ??? Flash
static void loadRuntimeHour() {
  g_motorPrefs.begin("motor_nvs", false);
  g_runtimeHour = g_motorPrefs.getFloat("runtime_h", 0.0f);
  g_motorPrefs.end();
  Serial.printf("[RPM] Loaded runtime_hour = %.4f h\n", g_runtimeHour);
}

// NVS: ?????? runtime_hour ?? Flash
static void saveRuntimeHour(float value) {
  g_motorPrefs.begin("motor_nvs", false);
  g_motorPrefs.putFloat("runtime_h", value);
  g_motorPrefs.end();
}

// ?????? runtime_hour ??? state transition + periodic save (30 s)
static void updateRuntimeHour() {
  bool isRunning = (g_motorRunState == MOTOR_RUNNING);
  if (isRunning && !g_prevWasRunning) {
    g_runningStartMs = millis();
    g_lastNvsSaveMs  = millis();
  }
  if (!isRunning && g_prevWasRunning) {
    uint32_t elapsed = millis() - g_runningStartMs;
    g_runtimeHour += elapsed / 3600000.0f;
    saveRuntimeHour(g_runtimeHour);
  }
  if (isRunning && (millis() - g_lastNvsSaveMs >= NVS_SAVE_INTERVAL_MS)) {
    uint32_t elapsed     = millis() - g_runningStartMs;
    float    totalToSave = g_runtimeHour + (elapsed / 3600000.0f);
    saveRuntimeHour(totalToSave);
    g_lastNvsSaveMs = millis();
  }
  g_prevWasRunning = isRunning;
}

// ?????? runtime ??????? session ????????
static float getCurrentRuntimeHour() {
  if (g_motorRunState == MOTOR_RUNNING) {
    return g_runtimeHour + (millis() - g_runningStartMs) / 3600000.0f;
  }
  return g_runtimeHour;
}

// Process RPM -- ???? ISR vars -> ????? rpm / motor_state / prox / runtime_hour
// ???????? taskStateMachine ??? 250 ms (Core 0, no mutex needed)
static void processRPM(VibrationData_t* data) {
  uint32_t interval, pulseCopy;
  noInterrupts();
  interval  = g_rpmPulseInterval;
  pulseCopy = g_rpmTotalPulses;
  interrupts();

  bool newPulse = (pulseCopy != g_rpmLastPulseCount);
  if (newPulse) g_rpmLastPulseMillis = millis();
  g_rpmLastPulseCount = pulseCopy;

  uint32_t timeSincePulseMs = millis() - g_rpmLastPulseMillis;

  // ---------- RPM Calculation (EMA filtered) ----------
  if (newPulse && interval >= RPM_MIN_INTERVAL_US) {
    float rpmRaw = (60000000.0f / interval) / PULSE_PER_REV;
    if (rpmRaw <= MAX_RPM * SPIKE_REJECT_FACTOR) {
      g_rpmFiltered = RPM_SMOOTH_ALPHA * rpmRaw
                    + (1.0f - RPM_SMOOTH_ALPHA) * g_rpmFiltered;
    }
  }

  // ---------- Motor State Machine ----------
  if (timeSincePulseMs > FORCE_STOP_TIMEOUT_MS) {
    g_motorRunState = MOTOR_STOPPED;
    g_rpmFiltered   = 0.0f;
  } else if (timeSincePulseMs > NO_PULSE_STOPPING_MS) {
    g_motorRunState  = MOTOR_STOPPING;
    g_rpmFiltered   *= 0.80f;
    if (g_rpmFiltered < MIN_RPM_VALID) g_rpmFiltered = 0.0f;
  } else {
    bool inBand = (g_rpmFiltered >= (RATED_RPM - RATED_RPM_TOL)) &&
                  (g_rpmFiltered <= (RATED_RPM + RATED_RPM_TOL));
    g_motorRunState = inBand ? MOTOR_RUNNING : MOTOR_STARTING;
  }

  // v15.7: g_velPeakHold ถูกลบออกแล้ว -- peak hold ไม่มีอีกต่อไป
  g_prevMotorRunState = g_motorRunState;

  // ---------- Prox Signal Quality ----------
  // 1 = pulse ???? | 0 = Fault / ?????? pulse
  uint8_t prox;
  if (g_motorRunState == MOTOR_STOPPED) {
    prox = 0;
  } else if (g_motorRunState == MOTOR_RUNNING &&
             timeSincePulseMs > FAULT_WINDOW_MS) {
    prox = 0;
  } else {
    prox = 1;
  }

  // ---------- Runtime Hour Update ----------
  updateRuntimeHour();

  // ---------- Write to shared VibrationData_t ----------
  data->rpm          = roundf(g_rpmFiltered * 10.0f) / 10.0f;
  data->motor_state  = (uint8_t)g_motorRunState;
  data->runtime_hour = roundf(getCurrentRuntimeHour() * 10000.0f) / 10000.0f;
  data->prox         = prox;
}


// ============================================================================
// RS485 CONTROL (Core 0)
// ============================================================================

static inline void rs485Enable() {
  digitalWrite(RS485_EN_PIN, LOW);
}

static inline void rs485Disable() {
  digitalWrite(RS485_EN_PIN, HIGH);
}

// ============================================================================
// VY STUCK DETECTION & AUTO-RECOVERY (Core 0)
// ============================================================================
// ?????: ???? hard reset ????? ??????????????? register VY (0x3B) = 0 ????
//         ????????? power cycle ???????? -> ??? Modbus restart ???
// ??????: ??????? Vx/Vy/Vz = 0 ????????? 5 ????? (1.25s) ????????????? > 0
//         -> ???? restart ???????????? Modbus (Unlock 0x69 -> Restart 0x00)
// ============================================================================

// --- Vy Stuck ---
static volatile uint16_t g_vyStuckCount   = 0;  // ??? Vy=0 ?????????
static volatile uint16_t g_vyRestartCount = 0;  // ???????????????? restart ????? Vy

// --- Vz Stuck (NEW) ---
static volatile uint16_t g_vzStuckCount   = 0;  // ??? Vz=0 ?????????
static volatile uint16_t g_vzRestartCount = 0;  // ???????????????? restart ????? Vz

// --- Vx Stuck (NEW) ---
static volatile uint16_t g_vxStuckCount   = 0;  // ??? Vx=0 ?????????
static volatile uint16_t g_vxRestartCount = 0;  // ???????????????? restart ????? Vx

static uint32_t g_lastSensorRestart = 0;        // millis() ??? restart ?????? (shared cooldown)

static const uint16_t STUCK_THRESHOLD        = 5;      // 5 ????? x 250ms = 1.25 ?????? (????? 10)
static const int16_t  STUCK_MIN_RAW          = 5;      // |raw| > 5 (0.05 mm/s) ?????? "?????"
static const uint32_t SENSOR_RESTART_COOLDOWN = 30000; // ???? restart ???????? 30 ??????

/**
 * Restart WTVB02-485 via Modbus เมื่อ axis stuck ถึง threshold
 *
 * ปัญหา: Reboot (0x00FF) โหลดค่าจาก EEPROM -- ถ้า EEPROM ยังเป็น default
 *   (MODE=0x00, LowFreq) CF/Kurtosis จะเป็น 0 หลัง reboot
 *   → ต้อง re-apply config (SR=16K + MODE=FreqDomain) หลัง reboot ทุกครั้ง
 *
 * Sequence (พิสูจน์แล้วจาก WTVB02_SensorReset_Test.ino):
 *   unlock(300ms) → reboot(5s) → verify alive
 *   → unlock(300ms) → SR=16K → MODE=0x0002 → unlock(300ms) → save(300ms)
 *
 * หมายเหตุ:
 *   - Reboot x FAIL expected -- sensor reset ก่อนส่ง ACK กลับ
 *   - unlock ครั้งเดียวครอบคลุม SR + MODE ได้ (valid 10 วินาที)
 *   - เรียกจาก taskModbusRead (Core 0) -- ใช้ vTaskDelay ไม่ใช่ delay()
 *   - rs485Enable() ต้องถูกเรียกก่อนเรียก function นี้
 *
 * @param axisLabel  แกนที่ trigger ("Vx", "Vy", หรือ "Vz")
 */
static bool restartSensorViaModbus(const char* axisLabel) {
  uint32_t now = millis();

  // Cooldown check -- ป้องกัน restart loop (shared across all axes)
  if ((now - g_lastSensorRestart) < SENSOR_RESTART_COOLDOWN && g_lastSensorRestart != 0) {
    Serial.printf("[SENSOR] %s stuck -- Cooldown active (%lu s remaining)\n",
                  axisLabel,
                  (SENSOR_RESTART_COOLDOWN - (now - g_lastSensorRestart)) / 1000);
    return false;
  }

  Serial.println("[SENSOR] ========================================");
  Serial.printf("[SENSOR] ! %s STUCK -> Reboot + re-config...\n", axisLabel);
  Serial.println("[SENSOR] ========================================");

  uint8_t result;

  // ── Step 1: Unlock → Reboot ─────────────────────────────────────
  Serial.print("[SENSOR] [Unlock#1]... ");
  result = modbus.writeSingleRegister(0x0069, 0xB588);
  Serial.println(result == modbus.ku8MBSuccess ? "+ OK" : "x FAIL");
  if (result != modbus.ku8MBSuccess) {
    Serial.println("[SENSOR] x Unlock failed -- abort");
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(300));

  Serial.print("[SENSOR] Reboot (0x00=0x00FF)... ");
  result = modbus.writeSingleRegister(0x0000, 0x00FF);
  Serial.printf("%s  (FAIL expected)\n",
                result == modbus.ku8MBSuccess ? "+ OK" : "x FAIL");
  // ไม่ abort เมื่อ FAIL -- sensor reset ก่อนส่ง ACK กลับ (ปกติ)

  Serial.println("[SENSOR] Waiting 5 s for sensor reboot...");
  vTaskDelay(pdMS_TO_TICKS(5000));

  // ── Step 2: Verify alive ────────────────────────────────────────
  bool alive = false;
  for (int i = 0; i < 3 && !alive; i++) {
    if (modbus.readHoldingRegisters(0x003A, 3) == modbus.ku8MBSuccess) {
      float vx = (int16_t)modbus.getResponseBuffer(0) / 100.0f;
      Serial.printf("[SENSOR] + Alive! VX=%.2f mm/s\n", vx);
      alive = true;
    } else {
      Serial.printf("[SENSOR] x Not yet, retry %d/3...\n", i + 1);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
  if (!alive) {
    Serial.println("[SENSOR] x Sensor not responding after reboot");
    return false;
  }

  // ── Step 3: Re-apply config (ป้องกัน MODE กลับเป็น default=0x00) ──
  // Unlock ครั้งเดียวครอบคลุม SR + MODE (valid 10 วินาที)
  Serial.print("[SENSOR] [Unlock#2 for re-config]... ");
  result = modbus.writeSingleRegister(0x0069, 0xB588);
  Serial.println(result == modbus.ku8MBSuccess ? "+ OK" : "x FAIL");
  if (result != modbus.ku8MBSuccess) {
    Serial.println("[SENSOR] x Re-config unlock failed -- sensor alive but config may be wrong");
    // ไม่ return false -- sensor ยัง alive แต่ MODE อาจผิด
  } else {
    vTaskDelay(pdMS_TO_TICKS(300));

    Serial.print("[SENSOR] SR=16K (0x29=0x0001)... ");
    result = modbus.writeSingleRegister(0x0029, 0x0001);
    Serial.println(result == modbus.ku8MBSuccess ? "+ OK" : "x FAIL");
    vTaskDelay(pdMS_TO_TICKS(100));

    Serial.print("[SENSOR] MODE=FreqDomain (0x07=0x0002)... ");
    result = modbus.writeSingleRegister(0x0007, 0x0002);
    Serial.println(result == modbus.ku8MBSuccess ? "+ OK" : "x FAIL");
    vTaskDelay(pdMS_TO_TICKS(100));

    // Unlock ก่อน Save
    Serial.print("[SENSOR] [Unlock#3 before Save]... ");
    result = modbus.writeSingleRegister(0x0069, 0xB588);
    Serial.println(result == modbus.ku8MBSuccess ? "+ OK" : "x FAIL");
    vTaskDelay(pdMS_TO_TICKS(300));

    Serial.print("[SENSOR] Save (0x00=0x0000)... ");
    result = modbus.writeSingleRegister(0x0000, 0x0000);
    Serial.println(result == modbus.ku8MBSuccess ? "+ OK" : "x FAIL");
    vTaskDelay(pdMS_TO_TICKS(300));

    Serial.println("[SENSOR] + Re-config complete (SR=16K, MODE=FreqDomain)");
  }

  // ── Done ────────────────────────────────────────────────────────
  g_lastSensorRestart = millis();
  g_vxStuckCount = 0;
  g_vyStuckCount = 0;
  g_vzStuckCount = 0;

  Serial.printf("[SENSOR] + Restart+reconfig done (axis=%s)\n", axisLabel);
  Serial.println("[SENSOR] ========================================");
  return true;
}

// ============================================================================
// FORCE SENSOR CONFIG -- ตั้งค่า WTVB02-485 ทุกครั้งที่ boot
// ============================================================================
//
// ลำดับที่พิสูจน์แล้วจาก WTVB02_SensorReset_Test.ino (test log v2):
//
//   สิ่งที่ค้นพบจากการทดสอบจริง:
//   1. Unlock ครั้งเดียวครอบคลุม write หลายอันได้ (valid 10 วินาที)
//      → Unlock#2 ก่อน write MODE ในไฟล์ test x FAIL แต่ MODE write + OK
//        เพราะ Unlock#1 ยังค้างอยู่ -- ไม่จำเป็นต้อง unlock ก่อนทุก write
//   2. Save (0x00=0x0000) ต้องการ unlock ก่อน (Unlock#3 + OK)
//   3. Reboot (0x00=0x00FF) x FAIL แต่ sensor reboot จริงเสมอ
//      → sensor reset ก่อนส่ง Modbus response ครบ จึง ModbusMaster รับ timeout
//      → ไม่ต้อง abort เมื่อ Reboot FAIL
//   4. CF/Kurtosis ≠ 0 ต้องการ MODE=0x02 (FreqDomain, reg 0x07=0x0002)
//      MODE=0x00 และ 0x01 ให้ CF=0 ทุก sample
//
// ลำดับที่ใช้:
//   unlock(300ms) → write SR=16K(100ms) → write MODE=0x02(100ms)
//   → unlock(300ms) → save(300ms)
//   → unlock(300ms) → reboot(5s รอ sensor)
//
// Registers:
//   0x29 (SR)   = 0x0001 → 16K Hz (§6.4.12)
//   0x07 (MODE) = 0x0002 → FreqDomain -- เงื่อนไขที่ CF/Kurtosis ≠ 0
//
// ============================================================================
static bool forceSensorConfig() {
  uint8_t result;
  Serial.println("[SENSOR-CFG] ========================================");
  Serial.println("[SENSOR-CFG] Applying forced sensor configuration...");
  Serial.println("[SENSOR-CFG] Sequence: unlock→SR / unlock→MODE / unlock→save / unlock→reboot");
  Serial.println("[SENSOR-CFG] ========================================");

  // Helper: flush RS485 RX buffer ระหว่าง write ป้องกัน stale bytes จาก response ก่อน
  auto flushRS485 = []() {
    uint32_t t = millis();
    while (SerialRS485.available() && millis() - t < 50) SerialRS485.read();
  };

  // ── Unlock#1 → SR=16K ────────────────────────────────────────────
  flushRS485();
  Serial.print("[SENSOR-CFG] [Unlock#1 for SR]... ");
  result = modbus.writeSingleRegister(0x0069, 0xB588);
  Serial.println(result == modbus.ku8MBSuccess ? "+ OK" : "x FAIL");
  if (result != modbus.ku8MBSuccess) {
    Serial.println("[SENSOR-CFG] ABORT: cannot unlock sensor");
    return false;
  }
  flushRS485();
  delay(300);

  Serial.print("[SENSOR-CFG] SR=16K (0x29=0x0001)... ");
  result = modbus.writeSingleRegister(0x0029, 0x0001);
  Serial.println(result == modbus.ku8MBSuccess ? "+ OK" : "x FAIL");
  if (result != modbus.ku8MBSuccess) {
    Serial.println("[SENSOR-CFG] ABORT: SR write failed");
    return false;
  }
  flushRS485();
  delay(300);  // เพิ่มจาก 100 → 300ms: รอ sensor process SR แล้วค่อย unlock ใหม่

  // ── Unlock#2 → MODE=FreqDomain ───────────────────────────────────
  Serial.print("[SENSOR-CFG] [Unlock#2 for MODE]... ");
  result = modbus.writeSingleRegister(0x0069, 0xB588);
  Serial.println(result == modbus.ku8MBSuccess ? "+ OK" : "x FAIL");
  if (result != modbus.ku8MBSuccess) {
    Serial.println("[SENSOR-CFG] ABORT: unlock#2 failed");
    return false;
  }
  flushRS485();
  delay(300);

  // reg 0x07 = MODE register (verified by test sketch: readback = 0x0002 after write)
  // MODE=0x0002 (FreqDomain) เงื่อนไขที่ CF/Kurtosis ≠ 0
  // MODE=0x00/0x01 ให้ CF=0 ทุก sample (พิสูจน์จาก WTVB02_SensorReset_Test.ino)
  Serial.print("[SENSOR-CFG] MODE=FreqDomain (0x07=0x0002)... ");
  result = modbus.writeSingleRegister(0x0007, 0x0002);
  Serial.println(result == modbus.ku8MBSuccess ? "+ OK" : "x FAIL");
  if (result != modbus.ku8MBSuccess) {
    Serial.println("[SENSOR-CFG] ABORT: MODE write failed");
    return false;
  }
  flushRS485();
  delay(300);  // เพิ่มจาก 100 → 300ms

  // ── Unlock#3 → Save ──────────────────────────────────────────────
  Serial.print("[SENSOR-CFG] [Unlock#3 before Save]... ");
  result = modbus.writeSingleRegister(0x0069, 0xB588);
  Serial.println(result == modbus.ku8MBSuccess ? "+ OK" : "x FAIL");
  if (result != modbus.ku8MBSuccess) {
    Serial.println("[SENSOR-CFG] ABORT: unlock before save failed");
    return false;
  }
  flushRS485();
  delay(300);

  Serial.print("[SENSOR-CFG] Save (0x00=0x0000)... ");
  result = modbus.writeSingleRegister(0x0000, 0x0000);
  Serial.println(result == modbus.ku8MBSuccess ? "+ OK" : "x FAIL");
  if (result != modbus.ku8MBSuccess) {
    Serial.println("[SENSOR-CFG] ABORT: save failed");
    return false;
  }
  flushRS485();
  delay(300);

  // ── Unlock#4 → Reboot ────────────────────────────────────────────
  Serial.print("[SENSOR-CFG] [Unlock#4 before Reboot]... ");
  result = modbus.writeSingleRegister(0x0069, 0xB588);
  Serial.println(result == modbus.ku8MBSuccess ? "+ OK" : "x FAIL");
  if (result != modbus.ku8MBSuccess) {
    Serial.println("[SENSOR-CFG] ABORT: unlock before reboot failed");
    return false;
  }
  flushRS485();
  delay(300);

  // Reboot -- sensor resets ก่อนส่ง ACK → ModbusMaster timeout → FAIL expected
  Serial.print("[SENSOR-CFG] Reboot (0x00=0x00FF)... ");
  result = modbus.writeSingleRegister(0x0000, 0x00FF);
  Serial.printf("%s  (FAIL expected -- sensor resets before ACK)\n",
                result == modbus.ku8MBSuccess ? "+ OK" : "x FAIL");

  Serial.println("[SENSOR-CFG] Waiting 5 s for sensor reboot...");
  delay(5000);

  // ── Verify alive + MODE ──────────────────────────────────────────
  bool alive = false;
  for (int i = 0; i < 3 && !alive; i++) {
    if (modbus.readHoldingRegisters(0x003A, 3) == modbus.ku8MBSuccess) {
      float vx = (int16_t)modbus.getResponseBuffer(0) / 100.0f;
      Serial.printf("[SENSOR-CFG] + Alive! VX=%.2f mm/s\n", vx);
      alive = true;
    } else {
      Serial.printf("[SENSOR-CFG] x not yet, retry %d/3...\n", i + 1);
      delay(1000);
    }
  }
  if (!alive) {
    Serial.println("[SENSOR-CFG] WARNING: sensor not responding after reboot");
    return false;
  }

  if (modbus.readHoldingRegisters(0x0007, 1) == modbus.ku8MBSuccess) {
    uint16_t mode = modbus.getResponseBuffer(0);
    Serial.printf("[SENSOR-CFG] MODE verify = 0x%04X %s\n",
                  mode, mode == 0x0002 ? "(FreqDomain OK)" : "(!!! MISMATCH -- CF will be 0)");
  }

  Serial.println("[SENSOR-CFG] + Config complete");
  Serial.println("[SENSOR-CFG] ========================================");
  return true;
}

// ============================================================================
// TLS SETUP -- parse certs ONCE into mbedTLS structures (not on every connect)
// ============================================================================
void setupTLS() {
  Serial.println("[TLS] Parsing mTLS certificates into ESP32 mbedTLS...");
  gsmClient.setCACert(root_ca);
  gsmClient.setCertificate(client_crt);
  gsmClient.setPrivateKey(client_key);
  if (!gsmClient.applyCredentials()) {
    Serial.println("[TLS] WARNING: Certificate load failed!");
  }
}

// ============================================================================
// MODEM CONTROL FUNCTIONS
// ============================================================================

void modemPowerOn() {
  Serial.println("[Modem] Starting power on sequence...");

  digitalWrite(MODEM_POWER_ON, LOW);
  digitalWrite(MODEM_RESET_PIN, LOW);
  delay(2000);

  digitalWrite(MODEM_POWER_ON, HIGH);
  delay(100);
  digitalWrite(MODEM_RESET_PIN, HIGH);
  delay(1200);
  digitalWrite(MODEM_RESET_PIN, LOW);
  delay(5000);

  Serial.println("[Modem] Power sequence completed");
}

bool checkModemResponse(int timeoutMs = 10000) {
  unsigned long startTime = millis();
  SerialAT.flush();

  while (millis() - startTime < timeoutMs) {
    if (modem.testAT()) {
      Serial.println("[Modem] Modem responded");
      return true;
    }
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[Modem] No response from modem");
  return false;
}

bool modemInit() {
  Serial.println("[Modem] Starting initialization...");
  g_network.modemState = MODEM_STATE_INITIALIZING;

  // Initialize serial for modem
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(1000);
  SerialAT.flush();

  // Setup GPIO for modem control
  pinMode(MODEM_RESET_PIN, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);

  bool modemInitialized = false;
  int modemRetryCount = 0;
  const int MAX_MODEM_RETRIES = 3;

  while (!modemInitialized && modemRetryCount < MAX_MODEM_RETRIES) {
    modemRetryCount++;
    Serial.printf("[Modem] Initialization attempt %d/%d\n", modemRetryCount, MAX_MODEM_RETRIES);

    // Power on modem
    modemPowerOn();
    delay(3000);

    // Check modem response
    if (checkModemResponse(15000)) {
      if (modem.init()) {
        modemInitialized = true;
        Serial.println("[Modem] Initialized successfully");

        // Get modem info
        String modemInfo = modem.getModemInfo();
        Serial.printf("[Modem] Info: %s\n", modemInfo.c_str());

        // Get IMEI
        String imei = modem.getIMEI();
        strncpy(g_network.imei, imei.c_str(), sizeof(g_network.imei) - 1);
        Serial.printf("[Modem] IMEI: %s\n", g_network.imei);

        g_network.modemReady = true;
        // TLS certs are loaded into ESP32 mbedTLS via setupTLS() in taskNetwork init.
        // No modem-side SSL configuration needed.
      } else {
        Serial.println("[Modem] Library initialization failed");
      }
    }

    if (!modemInitialized) {
      Serial.println("[Modem] Retrying...");
      delay(2000);
    }
  }

  if (!modemInitialized) {
    Serial.println("[Modem] Init failed after all retries!");
    g_network.modemState = MODEM_STATE_ERROR;
    return false;
  }

  return true;
}

bool modemConnectGPRS() {
  if (!g_network.modemReady) {
    return false;
  }

  Serial.println("[Modem] Waiting for network registration...");
  g_network.modemState = MODEM_STATE_SEARCHING;

  // Wait for network registration (30 seconds)
  modem.waitForNetwork(30000L);

  if (modem.isNetworkConnected()) {
    Serial.println("[Modem] Network registered!");
    g_network.modemState = MODEM_STATE_REGISTERED;

    // Get operator name
    String op = modem.getOperator();
    strncpy(g_network.operatorName, op.c_str(), sizeof(g_network.operatorName) - 1);
    Serial.printf("[Modem] Operator: %s\n", g_network.operatorName);

    // Get signal quality
    g_network.signalQuality = modem.getSignalQuality();
    g_network.signalPercent = map(g_network.signalQuality, 0, 31, 0, 100);
    Serial.printf("[Modem] Signal: %d (%d%%)\n", g_network.signalQuality, g_network.signalPercent);

    // Connect GPRS
    Serial.printf("[Modem] Connecting to GPRS APN: %s\n", APN);
    g_network.modemState = MODEM_STATE_GPRS_CONNECTING;

    if (modem.gprsConnect(APN, GPRS_USER, GPRS_PASS)) {
      Serial.println("[Modem] GPRS connected!");
      g_network.gprsConnected = true;
      g_network.modemState = MODEM_STATE_GPRS_CONNECTED;

      // Print IP address
      String ip = modem.localIP().toString();
      Serial.printf("[Modem] IP: %s\n", ip.c_str());

      return true;
    } else {
      Serial.println("[Modem] GPRS connection failed!");
      g_network.modemState = MODEM_STATE_ERROR;
      return false;
    }
  } else {
    Serial.println("[Modem] Network registration failed!");
    g_network.modemState = MODEM_STATE_ERROR;
    return false;
  }
}

void updateSignalQuality() {
  if (g_network.modemReady) {
    g_network.signalQuality = modem.getSignalQuality();
    if (g_network.signalQuality != 99) {
      g_network.signalPercent = map(g_network.signalQuality, 0, 31, 0, 100);
    } else {
      g_network.signalPercent = 0;
    }
  }
}

// ============================================================================
// NTP / NETWORK TIME SYNCHRONIZATION
// ============================================================================
// The SIMCom A7670 provides GSM network time via AT+CCLK?.
// TinyGSM wraps this as modem.getGSMDateTime().
// We parse it, compare with DS3231, and correct if needed.
// ============================================================================

/**
 * Enable automatic network time update on the modem.
 * AT+CTZU=1 enables auto-update of RTC from network.
 * AT+CLTS=1 enables getting local timestamp.
 * Should be called once after modem.init().
 */
void modemEnableNetworkTime() {
  // Enable network time auto-update (CTZU) and local timestamp (CLTS)
  SerialAT.println("AT+CTZU=1");
  delay(300);
  // Drain response
  while (SerialAT.available()) SerialAT.read();

  SerialAT.println("AT+CLTS=1");
  delay(300);
  while (SerialAT.available()) SerialAT.read();

  // Some modems need AT+COPS? or network re-registration to take effect
  // Give the modem a moment to receive network time
  Serial.println("[NTP] Network time auto-update enabled (CTZU=1, CLTS=1)");
}

/**
 * Parse GSM date-time string from modem.
 * Formats observed from SIMCom A7670:
 *   "24/12/25,14:30:00+28"   (YY/MM/DD,HH:MM:SS+/-TZ_quarters)
 *   "2024/12/25,14:30:00+28"
 * TZ is in quarter-hours from UTC (e.g. +28 = +7h = UTC+7).
 * Returns true if parsing succeeded and fills 'dt' with UTC time.
 */
bool parseGSMDateTime(const String& gsmTime, DateTime& dt) {
  // Expect at least "YY/MM/DD,HH:MM:SS"
  if (gsmTime.length() < 17) {
    Serial.printf("[NTP] GSM time too short: '%s'\n", gsmTime.c_str());
    return false;
  }

  // Find the date/time delimiters to handle 2-digit or 4-digit year
  int slash1 = gsmTime.indexOf('/');
  int slash2 = gsmTime.indexOf('/', slash1 + 1);
  int comma = gsmTime.indexOf(',');
  int colon1 = gsmTime.indexOf(':');
  int colon2 = gsmTime.indexOf(':', colon1 + 1);

  if (slash1 < 0 || slash2 < 0 || comma < 0 || colon1 < 0 || colon2 < 0) {
    Serial.printf("[NTP] Cannot parse GSM time: '%s'\n", gsmTime.c_str());
    return false;
  }

  int year = gsmTime.substring(0, slash1).toInt();
  int month = gsmTime.substring(slash1 + 1, slash2).toInt();
  int day = gsmTime.substring(slash2 + 1, comma).toInt();
  int hour = gsmTime.substring(comma + 1, colon1).toInt();
  int minute = gsmTime.substring(colon1 + 1, colon2).toInt();

  // Seconds may be followed by +/-TZ
  String secPart = gsmTime.substring(colon2 + 1);
  int second = 0;
  int tzQuarters = 0;

  int plusIdx = secPart.indexOf('+');
  int minusIdx = secPart.indexOf('-');
  int tzIdx = (plusIdx >= 0) ? plusIdx : minusIdx;

  if (tzIdx >= 0) {
    second = secPart.substring(0, tzIdx).toInt();
    tzQuarters = secPart.substring(tzIdx).toInt();  // includes sign
  } else {
    second = secPart.toInt();
  }

  // Handle 2-digit year
  if (year < 100) year += 2000;

  // Sanity check
  if (year < 2024 || year > 2099 || month < 1 || month > 12 || day < 1 || day > 31 || hour > 23 || minute > 59 || second > 59) {
    Serial.printf("[NTP] Invalid GSM time values: %04d-%02d-%02d %02d:%02d:%02d\n",
                  year, month, day, hour, minute, second);
    return false;
  }

  // Convert local time to UTC by subtracting timezone offset
  // tzQuarters is in 15-minute increments (e.g. +28 = +7 hours)
  DateTime localTime(year, month, day, hour, minute, second);
  uint32_t unixLocal = localTime.unixtime();
  int32_t tzOffsetSec = tzQuarters * 15 * 60;
  uint32_t unixUTC = unixLocal - tzOffsetSec;

  dt = DateTime(unixUTC);

  Serial.printf("[NTP] Parsed: local=%04d-%02d-%02dT%02d:%02d:%02d (TZ=%+d quarters = %+dh)\n",
                year, month, day, hour, minute, second, tzQuarters, tzQuarters / 4);
  Serial.printf("[NTP] UTC  : %04d-%02d-%02dT%02d:%02d:%02dZ\n",
                dt.year(), dt.month(), dt.day(),
                dt.hour(), dt.minute(), dt.second());

  return true;
}

/**
 * Sync RTC from 4G modem network time.
 * Returns true if RTC was synced (or already accurate).
 */
bool syncRTCFromModem() {
  if (!g_network.modemReady) {
    Serial.println("[NTP] Modem not ready, skip sync");
    return false;
  }

  // Get GSM date-time from modem (TinyGSM wrapper for AT+CCLK?)
  String gsmDateTime = modem.getGSMDateTime(DATE_FULL);
  Serial.printf("[NTP] Raw GSM time: '%s'\n", gsmDateTime.c_str());

  if (gsmDateTime.length() < 10 || gsmDateTime.startsWith("80/01/06")) {
    // "80/01/06" is the default uninitialized time on many SIMCom modems
    Serial.println("[NTP] Modem has no valid network time yet");
    g_timeSync.syncFailures++;
    g_timeSync.ntpReachable = false;
    return false;
  }

  // Parse the GSM time string
  DateTime networkUTC;
  if (!parseGSMDateTime(gsmDateTime, networkUTC)) {
    g_timeSync.syncFailures++;
    g_timeSync.ntpReachable = false;
    return false;
  }

  g_timeSync.ntpReachable = true;

  // Compare with current RTC
  if (g_rtcValid) {
    DateTime rtcNow = rtc.now();
    int32_t drift = (int32_t)networkUTC.unixtime() - (int32_t)rtcNow.unixtime();
    g_timeSync.lastDriftSec = drift;

    Serial.printf("[NTP] RTC drift: %+d seconds\n", drift);

    if (abs(drift) <= 1) {
      // RTC is accurate, no adjustment needed
      Serial.println("[NTP] RTC is accurate (drift <= 1s), no adjustment");
      g_timeSync.synced = true;
      g_timeSync.lastSyncMillis = millis();
      g_timeSync.syncCount++;
      snprintf(g_timeSync.lastSyncTime, sizeof(g_timeSync.lastSyncTime),
               "%04d-%02d-%02dT%02d:%02d:%02dZ",
               networkUTC.year(), networkUTC.month(), networkUTC.day(),
               networkUTC.hour(), networkUTC.minute(), networkUTC.second());
      return true;
    }

    if (abs(drift) > NTP_DRIFT_WARN_SEC) {
      Serial.printf("[NTP] ! WARNING: RTC drift = %+d seconds!\n", drift);
    }

    if (abs(drift) > NTP_DRIFT_MAX_SEC) {
      Serial.printf("[NTP] ! CRITICAL: RTC drift = %+d seconds -- FORCE CORRECTION\n", drift);
    }

    // Correct RTC
    Serial.printf("[NTP] Adjusting RTC: %04d-%02d-%02dT%02d:%02d:%02dZ (was off by %+ds)\n",
                  networkUTC.year(), networkUTC.month(), networkUTC.day(),
                  networkUTC.hour(), networkUTC.minute(), networkUTC.second(), drift);

    rtc.adjust(networkUTC);

  } else {
    // RTC was not valid -- set it from network time
    Serial.println("[NTP] RTC was invalid -- setting from network time");
    rtc.adjust(networkUTC);
    g_rtcValid = true;
    g_timeSync.lastDriftSec = 0;
  }

  g_timeSync.synced = true;
  g_timeSync.lastSyncMillis = millis();
  g_timeSync.syncCount++;
  snprintf(g_timeSync.lastSyncTime, sizeof(g_timeSync.lastSyncTime),
           "%04d-%02d-%02dT%02d:%02d:%02dZ",
           networkUTC.year(), networkUTC.month(), networkUTC.day(),
           networkUTC.hour(), networkUTC.minute(), networkUTC.second());

  Serial.printf("[NTP] + RTC synced successfully (total syncs: %lu)\n", g_timeSync.syncCount);
  return true;
}

/**
 * Check if NTP sync is due and perform it if needed.
 * Call this periodically from the network task.
 */
void checkAndSyncTime() {
  uint32_t now = millis();
  uint32_t interval = g_timeSync.synced ? NTP_SYNC_INTERVAL : NTP_SYNC_RETRY_INTERVAL;

  // On first run or after interval
  if (g_timeSync.lastSyncMillis == 0 || (now - g_timeSync.lastSyncMillis >= interval)) {
    Serial.printf("[NTP] Time sync check (interval=%lus, synced=%s)\n",
                  interval / 1000, g_timeSync.synced ? "yes" : "no");
    syncRTCFromModem();
  }
}

// ============================================================================
// CORE 0 TASKS - TIME CRITICAL OPERATIONS
// ============================================================================

/**
 * Task 1: Modbus RTU Communication (CORE 0, Priority 5)
 * Runs every 250ms
 * Reads sensor data via RS485 and sends to queue
 * v15.7: Transaction 1 เปลี่ยนเป็นอ่าน VRMS X/Y/Z จาก reg 0x50/0x5C/0x68 โดยตรง
 */
void taskModbusRead(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(250);  // 250ms = 4Hz

  VibrationData_t localData;
  // v15.7: raw_vrms_* = uint16, อ่านจาก VRMS register (÷1000 → mm/s)
  // raw_x/y/z ถูกลบออก -- ไม่ใช้ VX/VY/VZ (0x3A~0x3C) อีกต่อไป
  uint16_t raw_vrms_x = 0, raw_vrms_y = 0, raw_vrms_z = 0;
  int16_t raw_temp;
  int16_t raw_fx, raw_fy, raw_fz;
  uint16_t raw_cfx = 0, raw_kx = 0;  // v15.0: CFX (0x47), KX (0x48) -- unsigned per datasheet §6.4.14
  uint16_t raw_cfy = 0, raw_ky = 0;  // v15.1: CFY (0x53), KY (0x54) -- unsigned per datasheet §6.4.15
  uint16_t raw_cfz = 0, raw_kz = 0;  // v15.1: CFZ (0x5F), KZ (0x60) -- unsigned per datasheet §6.4.16

  Serial.println("[CORE 0] Modbus task started");

  while (1) {
    g_sensorReads++;

    bool success = true;

    // Read all registers (blocking I/O, but isolated to this task)
    // v15.0: 3 Modbus transactions -- VEL + TEMP + FREQ + CF/K
    rs485Enable();
    vTaskDelay(pdMS_TO_TICKS(5));  // 5ms stabilization

    // v15.7: Transaction 1a: VRMS X (reg 0x50) -- sensor-computed velocity RMS
    if (modbus.readHoldingRegisters(REG_VRMS_X, 1) == modbus.ku8MBSuccess) {
      raw_vrms_x = (uint16_t)modbus.getResponseBuffer(0);
    } else {
      raw_vrms_x = 0;  // reset on fail -- don't carry stale value
      success = false;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // v15.7: Transaction 1b: VRMS Y (reg 0x5C)
    if (modbus.readHoldingRegisters(REG_VRMS_Y, 1) == modbus.ku8MBSuccess) {
      raw_vrms_y = (uint16_t)modbus.getResponseBuffer(0);
    } else {
      raw_vrms_y = 0;  // reset on fail
      success = false;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // v15.7: Transaction 1c: VRMS Z (reg 0x68)
    if (modbus.readHoldingRegisters(REG_VRMS_Z, 1) == modbus.ku8MBSuccess) {
      raw_vrms_z = (uint16_t)modbus.getResponseBuffer(0);
    } else {
      raw_vrms_z = 0;  // reset on fail
      success = false;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // Transaction 2a: Temperature (0x40)
    if (modbus.readHoldingRegisters(REG_TEMPERATURE, 1) == modbus.ku8MBSuccess) {
      raw_temp = (int16_t)modbus.getResponseBuffer(0);
    } else {
      success = false;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // Transaction 2b: Frequency X, Y, Z (3 consecutive registers 0x44~0x46)
    if (modbus.readHoldingRegisters(REG_FREQ_X, 3) == modbus.ku8MBSuccess) {
      raw_fx = (int16_t)modbus.getResponseBuffer(0);
      raw_fy = (int16_t)modbus.getResponseBuffer(1);
      raw_fz = (int16_t)modbus.getResponseBuffer(2);
    } else {
      raw_fx = 0;
      raw_fy = 0;
      raw_fz = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // Transaction 3: CFX (0x47) + KX (0x48) -- Accel Crest Factor & Kurtosis [v15.0]
    // Optional -- ถ้า fail ปล่อยค่าเดิม (0) ไม่กระทบ success หลัก
    if (modbus.readHoldingRegisters(REG_CFX, 2) == modbus.ku8MBSuccess) {
      raw_cfx = (uint16_t)modbus.getResponseBuffer(0); // v15.6 fix: unsigned per datasheet §6.4.14
      raw_kx  = (uint16_t)modbus.getResponseBuffer(1);
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // Transaction 4: CFY (0x53) + KY (0x54) -- Y-axis [v15.1]
    if (modbus.readHoldingRegisters(REG_CFY, 2) == modbus.ku8MBSuccess) {
      raw_cfy = (uint16_t)modbus.getResponseBuffer(0); // v15.6 fix: unsigned per datasheet §6.4.15
      raw_ky  = (uint16_t)modbus.getResponseBuffer(1);
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // Transaction 5: CFZ (0x5F) + KZ (0x60) -- Z-axis [v15.1]
    if (modbus.readHoldingRegisters(REG_CFZ, 2) == modbus.ku8MBSuccess) {
      raw_cfz = (uint16_t)modbus.getResponseBuffer(0); // v15.6 fix: unsigned per datasheet §6.4.16
      raw_kz  = (uint16_t)modbus.getResponseBuffer(1);
    }
    // ทั้ง T3/T4/T5 เป็น optional -- ไม่ set success = false ถ้า fail

    rs485Disable();

    if (success) {
      // -- + ??????????: Reset consecutive error counter --
      if (g_modbusConsecErrors > 0) {
        Serial.printf("[MODBUS] + Sensor back ONLINE (was offline for %u consecutive reads, "
                      "total errors: %lu)\n",
                      g_modbusConsecErrors, g_sensorErrors);
        g_modbusConsecErrors = 0;
        g_sensorOffline      = false;
      }

      // ============================================================
      // ============================================================
      // STUCK DETECTION -- ทำงานเฉพาะตอน MOTOR_RUNNING เท่านั้น
      // ============================================================
      // หลักการ: VRMS=0 บน sensor ที่ RUNNING อยู่ = channel ค้าง/fault
      // Gate: ตอน STOPPED/STARTING raw_vrms จะเป็น noise floor (~500 raw)
      //   ไม่ใช่ค่า 0 จริง → ถ้าไม่ gate จะไม่เคย trigger stuck
      //   แต่ถ้า motor หยุดและ raw_vrms บังเอิญ == 0 จะ false trigger
      //   ทางออกที่สะอาดที่สุด: รัน stuck detection เฉพาะ MOTOR_RUNNING
      // ถ้า motor ไม่ RUNNING → reset counter ทุกแกน (ป้องกัน count ค้าง
      //   ข้ามช่วง start/stop)
      // STUCK_MIN_RAW (= 5 raw = 0.005 mm/s) ใช้ยืนยันว่าอีกแกน "มีชีวิต"
      // threshold = 5 reads × 250 ms = 1.25 วินาที
      // ============================================================

      bool needRestart  = false;
      const char* stuckAxis = "";

      if (g_motorRunState == MOTOR_RUNNING) {

        // ── ALL-ZERO DETECTION ──────────────────────────────────────────
        // เมื่อ sensor ยัง MODE=default (0x00) → raw_vrms ทุกแกน = 0 พร้อมกัน
        // เงื่อนไข othersAlive ใน per-axis check จะ false → ไม่เคย trigger
        // ต้องตรวจ all-zero แยก: ถ้า RUNNING นาน > threshold แต่ทุกแกง = 0
        // → sensor ไม่ได้ config MODE ถูก → restart + re-config
        {
          bool allZero = (raw_vrms_x == 0) && (raw_vrms_y == 0) && (raw_vrms_z == 0);
          if (allZero) {
            g_vyStuckCount++;  // ใช้ Vy counter เป็น representative สำหรับ all-zero
            if (g_vyStuckCount == 1) {
              Serial.printf("[SENSOR] all-zero start: count=1/%u while RUNNING (rpm=%.0f)\n",
                            STUCK_THRESHOLD, g_rpmFiltered);
            }
            if (g_vyStuckCount == STUCK_THRESHOLD) {
              Serial.printf("[SENSOR] ! ALL axes=0 for %u reads while RUNNING"
                            " -- sensor MODE not configured -> restart+reconfig\n",
                            g_vyStuckCount);
              needRestart = true;
              stuckAxis   = "ALL";
              g_vyRestartCount++;
            }
          } else {
            if (g_vyStuckCount > 0) {
              Serial.printf("[SENSOR] + All-zero cleared (was %u reads)\n", g_vyStuckCount);
            }
            g_vyStuckCount = 0;
          }
        }

        if (!needRestart) {
        // --- Vy STUCK (single-axis) ---
        {
          bool vyIsZero    = (raw_vrms_y == 0);
          bool othersAlive = (raw_vrms_x > (uint16_t)STUCK_MIN_RAW) ||
                             (raw_vrms_z > (uint16_t)STUCK_MIN_RAW);

          if (vyIsZero && othersAlive) {
            g_vyStuckCount++;
            if (g_vyStuckCount == STUCK_THRESHOLD) {
              Serial.printf("[SENSOR] ! Vy=0 for %u reads (Vx=%u Vz=%u) -> restart\n",
                            g_vyStuckCount, raw_vrms_x, raw_vrms_z);
              needRestart = true;
              stuckAxis   = "Vy";
              g_vyRestartCount++;
            }
          } else {
            if (g_vyStuckCount > 0 && !vyIsZero) {
              Serial.printf("[SENSOR] + Vy recovered! (stuck %u reads, now raw_vrms_y=%u)\n",
                            g_vyStuckCount, raw_vrms_y);
            }
            g_vyStuckCount = 0;
          }
        }

        // --- Vz STUCK ---
        if (!needRestart) {
          bool vzIsZero    = (raw_vrms_z == 0);
          bool othersAlive = (raw_vrms_x > (uint16_t)STUCK_MIN_RAW) ||
                             (raw_vrms_y > (uint16_t)STUCK_MIN_RAW);

          if (vzIsZero && othersAlive) {
            g_vzStuckCount++;
            if (g_vzStuckCount == STUCK_THRESHOLD) {
              Serial.printf("[SENSOR] ! Vz=0 for %u reads (Vx=%u Vy=%u) -> restart\n",
                            g_vzStuckCount, raw_vrms_x, raw_vrms_y);
              needRestart = true;
              stuckAxis   = "Vz";
              g_vzRestartCount++;
            }
          } else {
            if (g_vzStuckCount > 0 && !vzIsZero) {
              Serial.printf("[SENSOR] + Vz recovered! (stuck %u reads, now raw_vrms_z=%u)\n",
                            g_vzStuckCount, raw_vrms_z);
            }
            g_vzStuckCount = 0;
          }
        }

        // --- Vx STUCK ---
        if (!needRestart) {
          bool vxIsZero    = (raw_vrms_x == 0);
          bool othersAlive = (raw_vrms_y > (uint16_t)STUCK_MIN_RAW) ||
                             (raw_vrms_z > (uint16_t)STUCK_MIN_RAW);

          if (vxIsZero && othersAlive) {
            g_vxStuckCount++;
            if (g_vxStuckCount == STUCK_THRESHOLD) {
              Serial.printf("[SENSOR] ! Vx=0 for %u reads (Vy=%u Vz=%u) -> restart\n",
                            g_vxStuckCount, raw_vrms_y, raw_vrms_z);
              needRestart = true;
              stuckAxis   = "Vx";
              g_vxRestartCount++;
            }
          } else {
            if (g_vxStuckCount > 0 && !vxIsZero) {
              Serial.printf("[SENSOR] + Vx recovered! (stuck %u reads, now raw_vrms_x=%u)\n",
                            g_vxStuckCount, raw_vrms_x);
            }
            g_vxStuckCount = 0;
          }
        }
        } // !needRestart (per-axis block)

      } else {
        // Motor ไม่ RUNNING → reset ทุก counter ป้องกัน count ค้างข้าม start/stop
        g_vxStuckCount = 0;
        g_vyStuckCount = 0;
        g_vzStuckCount = 0;
      }

      // --- ??? Restart ????? axis ??? stuck ??? threshold ---
      if (needRestart) {
        rs485Enable();
        vTaskDelay(pdMS_TO_TICKS(5));

        if (restartSensorViaModbus(stuckAxis)) {
          Serial.printf("[SENSOR] + Auto-restart OK (axis=%s), monitoring recovery...\n", stuckAxis);
        } else {
          Serial.printf("[SENSOR] x Auto-restart FAILED (axis=%s), retry after cooldown\n", stuckAxis);
          // reset counters ????????????????????????? cooldown
          g_vxStuckCount = 0;
          g_vyStuckCount = 0;
          g_vzStuckCount = 0;
        }

        rs485Disable();

        // Reset timing ????? restart ??????? ~3 ??????
        xLastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        continue;  // ?????????? ?????????????
      }

      // ============================================================
      // DATA PROCESSING [v15.7]
      // ============================================================

      // Step 1: Velocity RMS จาก sensor โดยตรง (VRMS register ÷ 1000)
      // sensor คำนวณจาก raw FIFO 16KHz ภายใน chip -- ไม่ต้องคำนวณเอง
      //
      // Motor state gate: ตอน STOPPED/STARTING sensor ยังให้ noise floor
      // (ประมาณ 0.5~1 mm/s) ซึ่งไม่ใช่ค่าจริง → force เป็น 0 เพื่อ
      // ป้องกัน false alarm และ false fault classification
      // RUNNING (2) และ STOPPING (3) เท่านั้นที่ให้ค่าผ่าน
      {
        bool motorActive = (g_motorRunState == MOTOR_RUNNING ||
                            g_motorRunState == MOTOR_STOPPING);
        localData.rms_x = motorActive ? (raw_vrms_x / 1000.0f) : 0.0f;
        localData.rms_y = motorActive ? (raw_vrms_y / 1000.0f) : 0.0f;
        localData.rms_z = motorActive ? (raw_vrms_z / 1000.0f) : 0.0f;
        // CF/Kurtosis ก็ gate เช่นกัน -- noise floor ทำให้ kurtosis พุ่งสูงผิดปกติ
        // (เช่น kurtosis=33 ขณะ state=STOPPED ซึ่งไม่มีความหมาย)
        if (!motorActive) {
          raw_cfx = 0; raw_kx = 0;
          raw_cfy = 0; raw_ky = 0;
          raw_cfz = 0; raw_kz = 0;
        }
      }
      // v15.8 Fix #1: True 3-axis vector RMS magnitude per ISO 10816 / ISO 20816
      // Before: rms_overall = max(rms_x, rms_y, rms_z)  -- underestimates by up to 42%
      // After:  rms_overall = sqrt(x² + y² + z²)        -- true energy magnitude
      // Thresholds (BASELINE=2.1, WARNING=4.5, CRITICAL=7.1 mm/s) do NOT require
      // adjustment: ISO 10816-3 specifies them as vector RMS -- they were already
      // correct for this formula.
      localData.rms_overall = sqrtf(localData.rms_x * localData.rms_x +
                                    localData.rms_y * localData.rms_y +
                                    localData.rms_z * localData.rms_z);

      // Step 3: Sensor-computed CF & Kurtosis -- ครบ 3 แกน [v15.1]
      // คำนวณจาก 16KHz raw FIFO ภายใน chip
      // raw = 0 ถ้า transaction fail (ปลอดภัย -- guard > 0)
      localData.cf_x       = (raw_cfx > 0) ? raw_cfx / 1000.0f : 0.0f;
      localData.cf_y       = (raw_cfy > 0) ? raw_cfy / 1000.0f : 0.0f;
      localData.cf_z       = (raw_cfz > 0) ? raw_cfz / 1000.0f : 0.0f;

      localData.kurtosis_x = (raw_kx  > 0) ? raw_kx  / 1000.0f : 0.0f;
      localData.kurtosis_y = (raw_ky  > 0) ? raw_ky  / 1000.0f : 0.0f;
      localData.kurtosis_z = (raw_kz  > 0) ? raw_kz  / 1000.0f : 0.0f;

      // Derived: max CF และ max Kurtosis พร้อม dominant axis [v15.1]
      // kurtosis_max ใช้ใน bearing alert: > 4.0 = early warning, > 6.0 = confirmed
      // kurtosis_dominant_axis ใช้ localize fault: 0=X(radial), 1=Y(radial), 2=Z(axial)
      localData.cf_max = max(localData.cf_x, max(localData.cf_y, localData.cf_z));

      if (localData.kurtosis_x >= localData.kurtosis_y &&
          localData.kurtosis_x >= localData.kurtosis_z) {
        localData.kurtosis_max            = localData.kurtosis_x;
        localData.kurtosis_dominant_axis  = 0;  // X
      } else if (localData.kurtosis_y >= localData.kurtosis_z) {
        localData.kurtosis_max            = localData.kurtosis_y;
        localData.kurtosis_dominant_axis  = 1;  // Y
      } else {
        localData.kurtosis_max            = localData.kurtosis_z;
        localData.kurtosis_dominant_axis  = 2;  // Z
      }
      // ถ้าทุกแกน = 0 (transactions ทั้งหมด fail) kurtosis_max = 0 -- ไม่ trigger alert

      // Step 2 (misc): Temperature, Frequency
      localData.temperature = raw_temp / 100.0f;
      localData.freq_x = raw_fx / 10.0f;
      localData.freq_y = raw_fy / 10.0f;
      localData.freq_z = raw_fz / 10.0f;

      // v15.7: Step 5 (peak hold) ถูกลบออก -- ไม่มี peak field อีกต่อไป

      localData.timestamp = millis();
      localData.valid = true;

      // Send to queue (non-blocking)
      if (xQueueSend(queueSensorData, &localData, 0) != pdPASS) {
        Serial.println("[CORE 0] Sensor queue full!");
      }
    } else {
      // -- x Modbus ?????????? (timeout / no response) --
      g_sensorErrors++;
      g_modbusConsecErrors++;

      if (g_modbusConsecErrors >= MODBUS_OFFLINE_THRESHOLD) {
        // ?????? OFFLINE ?????????????? threshold
        if (!g_sensorOffline) {
          g_sensorOffline = true;
          Serial.printf("[MODBUS] ! SENSOR OFFLINE -- %u consecutive errors "
                        "(total: %lu). Check sensor power / RS485 wiring.\n",
                        g_modbusConsecErrors, g_sensorErrors);
        }

        // ??? invalid packet ????? queue ??? cycle ??? offline
        // ???????? taskStateMachine ??????? g_vibData ???????????? ERROR ?? display
        memset(&localData, 0, sizeof(VibrationData_t));
        localData.valid     = false;
        localData.timestamp = millis();
        if (xQueueSend(queueSensorData, &localData, 0) != pdPASS) {
          // queue ???? -- ??? critical, ??????????
        }
      } else {
        // ????????? threshold -- log warning ??????????????? offline
        Serial.printf("[MODBUS] x Read failed (consec=%u/%d, total=%lu)\n",
                      g_modbusConsecErrors, MODBUS_OFFLINE_THRESHOLD, g_sensorErrors);
      }
    }

    // Wait until next cycle (precise timing)
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/**
 * Task 2: State Machine & Safety Logic (CORE 0, Priority 4)
 * Processes sensor data and updates machine state
 * Triggered by queue from Modbus task
 */
void taskStateMachine(void* parameter) {
  VibrationData_t sensorData;

  Serial.println("[CORE 0] State machine task started");

  while (1) {
    // Wait for new sensor data (blocking on queue)
    if (xQueueReceive(queueSensorData, &sensorData, portMAX_DELAY) == pdPASS) {

      // -- ???????: sensor offline (valid = false) --
      if (!sensorData.valid) {
        // ???? g_vibData ??????? 0 ??? mark invalid
        // ????????????????????????????? display / MQTT
        if (xSemaphoreTake(mutexVibData, pdMS_TO_TICKS(10)) == pdTRUE) {
          memset(&g_vibData, 0, sizeof(VibrationData_t));
          g_vibData.valid     = false;
          g_vibData.timestamp = sensorData.timestamp;
          xSemaphoreGive(mutexVibData);
        }

        // v15.7: peak hold ถูกลบออกแล้ว -- ไม่ต้อง reset

        // ?????? state ???? NORMAL -- ???? trigger alarm ??? sensor ???????
        if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(10)) == pdTRUE) {
          if (g_systemState.state != STATE_MAINTENANCE) {
            if (g_systemState.state != STATE_NORMAL) {
              Serial.println("[STATE] Sensor OFFLINE -> forced STATE_NORMAL, buzzer OFF");
            }
            g_systemState.state       = STATE_NORMAL;
            g_systemState.buzzerActive = false;
          }
          xSemaphoreGive(mutexSystemState);
        }
        continue;  // ??????????????? RPM / state machine ?????????
      }

      // -- + ?????????????: ???????????? --
      processRPM(&sensorData);

      // Update shared vibration data (with mutex)
      if (xSemaphoreTake(mutexVibData, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(&g_vibData, &sensorData, sizeof(VibrationData_t));
        xSemaphoreGive(mutexVibData);
      }

      // -- Push sample to Trend Buffer (Core 0 only, no mutex needed) --
      // freq_ratio gate: ใช้ RPM_FREQ_GATE (=400 rpm) เหมือนกับ publish
      // และถ้า rms=0 (motor active แต่ VRMS=0 เพราะ MODE ผิด) ให้ freq_ratio=0
      // เพื่อป้องกัน RESONANCE classification จาก freq noise ล้วนๆ
      //
      // rms_max: track max(rms_overall) ในช่วง TREND_SAMPLE_WINDOW (4 reads = 1s)
      // เพื่อให้ spike_count นับ impulse จริง ไม่ใช่ sustained rms
      {
        static float   s_peakWindow   = 0.0f;  // accumulate max rms in window
        static uint8_t s_peakWinCount = 0;
        const  uint8_t TREND_SAMPLE_WINDOW = 4; // 4 reads x 250ms = 1 sample/s

        float curRms = sensorData.rms_overall;
        if (curRms > s_peakWindow) s_peakWindow = curRms;
        s_peakWinCount++;

        float ratX = 0.0f, ratY = 0.0f, ratZ = 0.0f;
        bool freqValid = (sensorData.rpm >= (float)RPM_FREQ_GATE) &&
                         (sensorData.rms_overall > 0.0f);
        if (freqValid) {
          float rf = sensorData.rpm / 60.0f;
          ratX = sensorData.freq_x / rf;
          ratY = sensorData.freq_y / rf;
          ratZ = sensorData.freq_z / rf;
        }

        if (s_peakWinCount >= TREND_SAMPLE_WINDOW) {
          g_trendBuf[g_trendHead] = {
            curRms,        // rms -- current sustained rms
            s_peakWindow,  // rms_max -- max impulse in window (for spike detection)
            sensorData.temperature,
            ratX, ratY, ratZ
          };
          g_trendHead  = (g_trendHead + 1) % TREND_BUF_SIZE;
          if (g_trendCount < TREND_BUF_SIZE) g_trendCount++;
          s_peakWindow   = 0.0f;
          s_peakWinCount = 0;
        }
      }

      // -- Flush trend buffer on motor STOP transition ─────────────────────
      // เมื่อ motor หยุด freq_ratio ใน trend buffer จาก RUNNING phase
      // จะก่อให้เกิด freq_drift alert ผิดพลาดในรอบถัดไป
      // flush เมื่อ transition → STOPPED เพื่อให้ trend เริ่มต้นใหม่
      {
        static MotorRunState_t s_prevStateForFlush = MOTOR_STOPPED;
        MotorRunState_t cur = g_motorRunState;
        if (s_prevStateForFlush != MOTOR_STOPPED && cur == MOTOR_STOPPED) {
          g_trendHead  = 0;
          g_trendCount = 0;
          Serial.println("[TREND] Motor→STOPPED: trend buffer flushed");
        }
        s_prevStateForFlush = cur;
      }

      // Determine new state based on RMS
      MachineState_t newState;
      float rms = sensorData.rms_overall;

      if (rms < WARNING_RMS) {
        newState = STATE_NORMAL;
      } else if (rms >= WARNING_RMS && rms < CRITICAL_RMS) {
        newState = STATE_WARNING;
      } else {
        newState = STATE_CRITICAL;
      }

      // Update system state (with mutex)
      if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(10)) == pdTRUE) {
        MachineState_t oldState = g_systemState.state;

        // Skip if in maintenance mode
        if (g_systemState.state != STATE_MAINTENANCE) {
          if (newState != oldState) {
            g_systemState.state = newState;
            g_systemState.stateEntryTime = millis();
            g_systemState.alarmAcknowledged = false;

            // Activate buzzer on WARNING/CRITICAL
            if (newState >= STATE_WARNING) {
              g_systemState.buzzerActive = true;
            } else {
              g_systemState.buzzerActive = false;
            }

            Serial.printf("[CORE 0] State: %d -> %d (RMS: %.2f)\n",
                          oldState, newState, rms);
          }
        }

        xSemaphoreGive(mutexSystemState);
      }
    }
  }
}

// ============================================================================
// CORE 1 TASKS - USER INTERFACE & NETWORK
// ============================================================================

/**
 * Task 3: OLED Display Rendering (CORE 1, Priority 3)
 * Updates display at 10 Hz (100ms)
 * Uses I2C mutex to prevent conflicts
 */
void taskDisplayUpdate(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 100ms = 10Hz

  VibrationData_t localVibData;
  MachineState_t localState;
  DisplayPage_t localPage;
  bool blinkState = false;
  uint32_t lastBlink = 0;

  Serial.println("[CORE 1] Display task started");

  while (1) {
    uint32_t now = millis();

    // Update blink state
    uint32_t blinkInterval = (localState == STATE_CRITICAL) ? 300 : 1000;
    if (now - lastBlink >= blinkInterval) {
      blinkState = !blinkState;
      lastBlink = now;
    }

    // Get current data (with mutex)
    if (xSemaphoreTake(mutexVibData, pdMS_TO_TICKS(5)) == pdTRUE) {
      memcpy(&localVibData, &g_vibData, sizeof(VibrationData_t));
      xSemaphoreGive(mutexVibData);
    }

    if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(5)) == pdTRUE) {
      localState = g_systemState.state;
      localPage = g_systemState.currentPage;
      xSemaphoreGive(mutexSystemState);
    }

    // Render display (with I2C mutex)
    if (xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(50)) == pdTRUE) {
      u8g2.clearBuffer();

      // Route to appropriate page renderer
      if (!localVibData.valid) {
        // -- Sensor offline -- ???????? ERROR ?????????? --
        drawSensorOfflineScreen();
      } else if (localPage == PAGE_MACHINE) {
        if (localState == STATE_CRITICAL) {
          drawCriticalScreen(&localVibData, blinkState);
        } else if (localState == STATE_WARNING) {
          drawWarningScreen(&localVibData, blinkState);
        } else {
          drawMachineScreen(&localVibData);
        }
      } else if (localPage == PAGE_AXIS) {
        drawAxisScreen(&localVibData);
      } else if (localPage == PAGE_NETWORK) {
        drawNetworkScreen();
      }

      u8g2.sendBuffer();
      xSemaphoreGive(mutexI2C);

      g_displayUpdates++;
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/**
 * Task 4: 4G Modem & MQTT Network (CORE 1, Priority 2)
 * Handles 4G connectivity and telemetry publishing
 * Based on working code from ESPSS3_in_ro_R11_final.ino
 */
void taskNetwork(void* parameter) {
  Serial.println("[CORE 1] Network task started (4G Modem)");

  // FIX-WDT (v14.9): Subscribe Network task to TWDT with extended timeout.
  // Arduino ESP32 core auto-subscribes IDLE tasks to the 5s TWDT. When
  // Network4G blocks CPU1 (modem I/O, TLS handshake, MQTT publish over 4G)
  // the IDLE task on CPU1 starves → "Network4G did not reset watchdog" crash.
  // Solution: subscribe this task explicitly and reset at every loop iteration.
  // Timeout set to 30s to cover worst-case modem re-registration + handshake.
  // The 100ms vTaskDelay at end of main loop resets the WDT each tick.
  esp_task_wdt_add(NULL);

  vTaskDelay(pdMS_TO_TICKS(5000));  // Wait for system to stabilize

  // Initialize modem
  if (!modemInit()) {
    Serial.println("[CORE 1] Modem init failed!");
    // Continue running but in error state
  } else {
    // Enable automatic network time update on the modem
    modemEnableNetworkTime();
  }

  // Connect to GPRS if modem is ready
  if (g_network.modemReady) {
    modemConnectGPRS();

    // Perform initial time sync after GPRS connects
    if (g_network.gprsConnected) {
      Serial.println("[CORE 1] Performing initial NTP time sync...");
      // Wait a moment for modem to receive network time
      vTaskDelay(pdMS_TO_TICKS(3000));
      syncRTCFromModem();
    }
  }

  Serial.printf("[CORE 1] Client ID: %s\n", MQTT_CLIENT_ID);

  // FIX-2: Removed raw TCP test against port 8883.
  // Port 8883 is TLS-only. A plaintext TCP connect causes Mosquitto's OpenSSL to start
  // a TLS handshake; the broker then receives garbage (no ClientHello), logs an SSL
  // error, and may rate-limit or flag subsequent connections from the same IP.
  // If port reachability testing is required, test a non-TLS port (e.g. port 80 HTTP).


  // -- Load mTLS certs into ESP32 mbedTLS (same as aws_test.ino setupTLS) --
  setupTLS();

  // -- MQTT over mTLS (arduino-mqtt + GsmTLSClient + ESP32 mbedTLS) --
  mqttClient.begin(MQTT_SERVER, MQTT_PORT, gsmClient);
  mqttClient.setKeepAlive(60);

  // Connection state tracking
  bool lastGprsState = false;
  bool lastMqttState = false;
  uint32_t lastConnectionAttempt = 0;
  uint32_t lastPublish = 0;
  uint32_t lastSignalCheck = 0;
  uint32_t lastStatusCheck = 0;
  const uint32_t CONNECTION_RETRY_INTERVAL = 30000;

  // v15.4 Fix C: Exponential backoff สำหรับ MQTT reconnect
  // 30s → 60s → 120s → 300s (max) -- reset เมื่อ connect สำเร็จ
  uint32_t mqttBackoffMs   = 30000;   // เริ่มต้นที่ 30s
  const uint32_t BACKOFF_MIN =  30000;
  const uint32_t BACKOFF_MAX = 300000; // max 5 นาที
  uint8_t  mqttFailCount   = 0;       // นับ fail ต่อเนื่อง

  VibrationData_t localVibData;
  MachineState_t localState;

  while (1) {
    uint32_t now = millis();

    // v15.4 Fix A: WDT reset ทุก iteration -- ป้องกัน WDT ตอน network check ค้าง
    // (modem AT commands, GPRS check, NTP sync อาจใช้เวลา > 1s ต่อ call)
    esp_task_wdt_reset();

    // Check network status every 10 seconds
    if (now - lastStatusCheck > 10000) {
      lastStatusCheck = now;

      if (g_network.modemReady) {
        bool gprs = modem.isGprsConnected();
        bool network = modem.isNetworkConnected();

        // Try to reconnect network if lost
        if (!network) {
          Serial.println("[CORE 1] Network not connected, waiting...");
          esp_task_wdt_reset();   // v15.4: waitForNetwork อาจค้าง 10s
          modem.waitForNetwork(10000L);
          esp_task_wdt_reset();
          network = modem.isNetworkConnected();
        }

        // Try to reconnect GPRS if network is up but GPRS is down
        if (network && !gprs) {
          Serial.println("[CORE 1] Reconnecting GPRS...");
          esp_task_wdt_reset();   // v15.4: gprsConnect อาจใช้เวลา
          modem.gprsConnect(APN, GPRS_USER, GPRS_PASS);
          vTaskDelay(pdMS_TO_TICKS(5000));
          esp_task_wdt_reset();
          gprs = modem.isGprsConnected();
        }

        g_network.gprsConnected = gprs;

        // Update signal quality
        if (gprs) {
          g_network.signalQuality = modem.getSignalQuality();
          if (g_network.signalQuality != 99) {
            g_network.signalPercent = map(g_network.signalQuality, 0, 31, 0, 100);
          } else {
            g_network.signalPercent = 0;
          }
        }

        // Try to connect MQTT if GPRS is up but MQTT is down
        if (gprs && !mqttClient.connected() &&
            (now - lastConnectionAttempt > mqttBackoffMs)) {

          lastConnectionAttempt = now;

          // v15.4 Fix B: Full TLS reset ก่อน attempt ทุกครั้ง
          // ล้าง stale cipher state จาก session เดิม (IP เปลี่ยน → old context)
          esp_task_wdt_reset();   // Fix A: reset WDT ก่อน TLS operation
          gsmClient.resetTLS();
          esp_task_wdt_reset();   // Fix A: reset WDT หลัง TLS reset

          Serial.printf("[CORE 1] Connecting MQTT (mTLS, ID=%s) backoff=%lus attempt#%u...\n",
                        MQTT_CLIENT_ID,
                        (unsigned long)(mqttBackoffMs / 1000),
                        (unsigned)mqttFailCount + 1);

          esp_task_wdt_reset();   // Fix A: reset WDT ก่อน TLS handshake (อาจใช้เวลา ~2s)
          bool connected = mqttClient.connect(MQTT_CLIENT_ID);
          esp_task_wdt_reset();   // Fix A: reset WDT หลัง TLS handshake

          if (connected) {
            Serial.println("[CORE 1] MQTT Connected (mTLS) +");
            // Fix C: reset backoff เมื่อ connect สำเร็จ
            mqttBackoffMs = BACKOFF_MIN;
            mqttFailCount = 0;
          } else {
            int mqttErr = mqttClient.lastError();
            int mqttRc  = mqttClient.returnCode();
            mqttFailCount++;

            // Fix C: exponential backoff -- 30s → 60s → 120s → 300s (max)
            mqttBackoffMs = min(mqttBackoffMs * 2, (uint32_t)BACKOFF_MAX);

            Serial.printf("[CORE 1] MQTT connect failed | err=%d rc=%d fail#%u next_retry=%lus\n",
                          mqttErr, mqttRc,
                          (unsigned)mqttFailCount,
                          (unsigned long)(mqttBackoffMs / 1000));

            if (mqttErr == LWMQTT_NETWORK_FAILED_CONNECT)
              Serial.println("[CORE 1]   -> Layer: TCP connect failed (modem/network issue)");
            else if (mqttErr == LWMQTT_NETWORK_TIMEOUT)
              Serial.println("[CORE 1]   -> Layer: TLS handshake timed out");
            else if (mqttRc == 5)
              Serial.println("[CORE 1]   -> Layer: MQTT AUTH refused (check CN=client ID)");
            else if (mqttRc == 4)
              Serial.println("[CORE 1]   -> Layer: MQTT bad credentials");
            else
              Serial.println("[CORE 1]   -> Layer: TLS handshake failed (see [TLS] lines above)");

            // Fix A: ถ้า fail มากกว่า 3 ครั้งต่อเนื่อง ให้ reset modem ด้วย
            if (mqttFailCount >= 3) {
              Serial.printf("[CORE 1] %u consecutive MQTT failures -- reinit modem\n",
                            (unsigned)mqttFailCount);
              esp_task_wdt_reset();
              modem.restart();
              esp_task_wdt_reset();
              modemConnectGPRS();
              esp_task_wdt_reset();
              mqttFailCount = 0;
              mqttBackoffMs = BACKOFF_MIN;
            }
          }
        }


        // Update modem state
        if (gprs && mqttClient.connected()) {
          g_network.modemState = MODEM_STATE_GPRS_CONNECTED;
        } else if (gprs) {
          g_network.modemState = MODEM_STATE_GPRS_CONNECTED;
        } else if (network) {
          g_network.modemState = MODEM_STATE_REGISTERED;
        } else {
          g_network.modemState = MODEM_STATE_SEARCHING;
        }

        // Log state changes
        if (gprs != lastGprsState) {
          Serial.printf("[CORE 1] GPRS: %s\n", gprs ? "CONNECTED" : "DISCONNECTED");
          // Trigger time sync when GPRS comes back up
          if (gprs && !lastGprsState) {
            Serial.println("[CORE 1] GPRS reconnected -- scheduling NTP sync");
            g_timeSync.lastSyncMillis = 0;  // Force immediate sync check
          }
          lastGprsState = gprs;
        }

        if (mqttClient.connected() != lastMqttState) {
          Serial.printf("[CORE 1] MQTT: %s\n", mqttClient.connected() ? "CONNECTED" : "DISCONNECTED");
          lastMqttState = mqttClient.connected();
        }
      }
    }

    // -- mqttClient.loop() ??? iteration = ??? 100ms --
    // ????????????? publish ????? process ACK/PINGREQ ??????
    if (mqttClient.connected()) {
      mqttClient.loop();
    }

    // Periodic NTP time sync check
    if (g_network.gprsConnected) {
      checkAndSyncTime();
    }

    // Get current sensor data
    if (xSemaphoreTake(mutexVibData, pdMS_TO_TICKS(5)) == pdTRUE) {
      memcpy(&localVibData, &g_vibData, sizeof(VibrationData_t));
      xSemaphoreGive(mutexVibData);
    }

    if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(5)) == pdTRUE) {
      localState = g_systemState.state;
      xSemaphoreGive(mutexSystemState);
    }

    // Phase 5: Adaptive publish interval from Fusion Decision Engine
    // g_decision.publish_interval_ms accounts for final_state, TTW derivative,
    // state-change boost, and minimum floor (3 s).
    uint32_t publishInterval = g_decision.publish_interval_ms;
    if (publishInterval == 0) {
      // Fallback if decision engine not yet initialized
      switch (localState) {
        case STATE_WARNING:  publishInterval = 10000; break;
        case STATE_CRITICAL: publishInterval =  5000; break;
        default:             publishInterval = 30000; break;
      }
    }

    // -- Publish telemetry (?? FreeRTOS task ???????? ???????? ISR) --
    if (mqttClient.connected() && (now - lastPublish >= publishInterval)) {
      if (localVibData.valid) {
        // -- Normal telemetry --
        if (publishTelemetry(&localVibData, localState)) {
          lastPublish = now;
          g_network.publishCount++;
          g_network.lastPublishTime = now;
        } else {
          g_network.publishFailures++;
        }
      } else {
        // -- Sensor offline -- publish status alert ??? (??? 30s) --
        static uint32_t lastOfflinePublish = 0;
        if (now - lastOfflinePublish >= 30000) {
          StaticJsonDocument<296> offlineDoc;
          offlineDoc["plant"]              = PLANT_ID;
          offlineDoc["machine_id"]         = MACHINE_ID;
          offlineDoc["sensor_id"]          = SENSOR_ID;
          // Patch C: pipeline stage fields for /sensor topic consistency
          offlineDoc["stage"]              = "sensor";
          offlineDoc["execution_location"] = "edge";
          offlineDoc["sensor_status"]      = "OFFLINE";
          offlineDoc["error_count"]        = g_sensorErrors;
          offlineDoc["uptime_s"]           = millis() / 1000;
          // Timestamp
          if (g_rtcValid) {
            DateTime rtcNow = rtc.now();
            char tsBuf[25];
            snprintf(tsBuf, sizeof(tsBuf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
                     rtcNow.year(), rtcNow.month(),  rtcNow.day(),
                     rtcNow.hour(), rtcNow.minute(), rtcNow.second());
            offlineDoc["ts"] = tsBuf;
          }
          char offlineJson[296];
          serializeJson(offlineDoc, offlineJson, sizeof(offlineJson));
          // Patch C: publish to /sensor topic (not /vibration) so consumers
          // receive offline alerts on the same topic as normal sensor data.
          if (mqttClient.publish(g_mqttTopicSensor, offlineJson, false, MQTT_QOS)) {
            lastOfflinePublish = now;
            g_network.publishCount++;
            Serial.printf("[MQTT] ! Offline alert published: %s\n", offlineJson);
          } else {
            g_network.publishFailures++;
          }
        }
      }
    }

    // ── V14.4: Maintenance reset MQTT audit (queued from taskButtonHandler) ──
    // Drains up to QUEUE_SIZE_MAINT events per loop tick (should be ≤1 normally).
    {
      MaintenanceEvent_t mEvt;
      while (xQueueReceive(queueMaintEvent, &mEvt, 0) == pdPASS) {
        if (mqttClient.connected()) {
          char tsBuf[32];
          if (mEvt.rtcValid) {
            snprintf(tsBuf, sizeof(tsBuf),
                     "%04d-%02d-%02dT%02d:%02d:%02d",
                     mEvt.year, mEvt.month, mEvt.day,
                     mEvt.hour, mEvt.minute, mEvt.second);
          } else {
            snprintf(tsBuf, sizeof(tsBuf), "millis:%lu", mEvt.triggerMillis);
          }

          StaticJsonDocument<256> evDoc;
          char evBuf[280];

          evDoc["plant_id"]   = PLANT_ID;
          evDoc["machine_id"] = MACHINE_ID;
          evDoc["event"]      = "maintenance_reset";
          evDoc["timestamp"]  = tsBuf;
          evDoc["state"]      = "WARMUP";

          serializeJson(evDoc, evBuf, sizeof(evBuf));

          if (mqttClient.publish(g_mqttTopicEvent, evBuf, false, MQTT_QOS)) {
            g_network.publishCount++;
            Serial.println("[MAINT] MQTT audit event published +");
          } else {
            g_network.publishFailures++;
            Serial.println("[MAINT] MQTT audit event FAILED");
          }
        } else {
          // MQTT not connected — event is dropped (already dequeued).
          // Acceptable: maintenance reset is a manual operator action.
          Serial.println("[MAINT] MQTT audit event dropped (not connected)");
        }
      }
    }

    // -- 100ms sleep -- ให้ FreeRTOS scheduler ทำงาน tasks อื่น --
    // v15.4: WDT reset ย้ายไปอยู่ที่ TOP ของ loop + ทุก blocking operation
    // บรรทัดนี้เป็น safety net สำหรับ normal operation path
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/**
 * Task 5: Button Input Handling (CORE 1, Priority 2)
 * Non-blocking button debounce and event detection
 * Supports 2 buttons: SELECT (PIN_BUTTON) and ENTER (PIN_BUTTON_ENTER)
 */
void taskButtonHandler(void* parameter) {
  // SELECT button state tracking
  bool lastStateSelect = HIGH;
  uint32_t pressStartSelect = 0;
  bool processedSelect = false;

  // ENTER button state tracking
  bool lastStateEnter = HIGH;
  uint32_t pressStartEnter = 0;
  bool processedEnter = false;

  Serial.println("[CORE 1] Button task started (SELECT + ENTER)");

  while (1) {
    bool currentStateSelect = digitalRead(PIN_BUTTON);
    bool currentStateEnter = digitalRead(PIN_BUTTON_ENTER);
    uint32_t now = millis();

    // ===== SELECT BUTTON HANDLING =====
    // Detect press
    if (currentStateSelect == LOW && lastStateSelect == HIGH) {
      pressStartSelect = now;
      processedSelect = false;
    }

    // Detect hold duration
    if (currentStateSelect == LOW && !processedSelect) {
      uint32_t duration = now - pressStartSelect;

      if (duration >= 8000) {
        if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(10)) == pdTRUE) {
          if (g_systemState.state != STATE_CRITICAL) {
            g_systemState.state = STATE_MAINTENANCE;
            g_systemState.buzzerActive = false;
            xSemaphoreGive(mutexSystemState);  // release before suspend

            // ── Freeze processing to avoid race condition ──
            if (taskHandleAnalytics != NULL) {
              vTaskSuspend(taskHandleAnalytics);
            }

            // --- Reset Evidence ---
            for (int i = 0; i < FAULT_TYPE_COUNT; i++) {
              g_faultEvidence[i] = 0.0f;
              g_rawScores[i]     = 0.0f;
            }

            // --- Reset Trend Buffers ---
            memset(g_trendBuf, 0, sizeof(g_trendBuf));
            g_trendHead  = 0;
            g_trendCount = 0;

            memset(g_buf1s,  0, sizeof(g_buf1s));
            g_buf1sHead  = 0; g_buf1sCount  = 0;

            memset(g_buf10s, 0, sizeof(g_buf10s));
            g_buf10sHead = 0; g_buf10sCount = 0;

            memset(g_buf60s, 0, sizeof(g_buf60s));
            g_buf60sHead = 0; g_buf60sCount = 0;

            // --- Reset Variance (must NOT be zero) ---
            g_slopeVar_1s  = 1e-6f;
            g_slopeVar_10s = 1e-6f;
            g_slopeVar_60s = 1e-6f;

            // v15.7: g_velPeakHold ถูกลบออกแล้ว -- ไม่ต้อง reset

            // --- Reset EMA ---
            g_emaRms     = 0.0f;
            g_emaPrevRms = 0.0f;
            g_emaDelta   = 0.0f;
            g_emaDir     = 0;

            memset(&g_trendResult, 0, sizeof(g_trendResult));

            // --- Reset Decision Engine state (V14.5) ---
            // g_decision persists across the reset without this block, causing
            // stale fault/score (e.g. RESONANCE sc=1.00) to remain published
            // until runDecisionEngine() accumulates enough new data.
            memset(&g_decision, 0, sizeof(g_decision));
            g_decision.fault_type          = FAULT_UNKNOWN;
            g_decision.fault_uncertainty   = 1.0f;
            g_decision.alarm_class         = ALARM_SILENT;
            g_decision.publish_interval_ms = 30000UL;
            strncpy(g_decision.reason, "init", sizeof(g_decision.reason));

            // --- Reset OSG reliability multipliers (V14.5) ---
            // g_osg_r_s / g_osg_r_m are written by runDecisionEngine() after
            // OSG suppression; without reset they carry stale TTW weights into
            // the first post-warmup decision cycle.
            g_osg_r_s = 1.0f;
            g_osg_r_m = 1.0f;

            // --- Reset Decision Engine inter-cycle state (V14.6) ---
            // g_prevFinalState: stale value (e.g. 1/2) triggers false
            //   stateChanged=true → g_stateChangeCyc=3 → publish drops to 5s
            //   for 3 unnecessary cycles post-warmup.
            g_prevFinalState  = 0;
            g_stateChangeCyc  = 0;
            // g_prevTtwBest: stale TTW causes wrong ttwRoC = (0 - old_ttw)
            //   on first cycle, skewing adaptive publish interval.
            g_prevTtwBest     = 0.0f;
            // g_prevTtwRoC: promoted from static local (V14.6); must be
            //   zeroed so ttwRoC is suppressed (condition: prevTtwRoC>0)
            //   on the first post-warmup call.
            g_prevTtwRoC      = 0.0f;
            // g_lastEvidenceMs: reset so AEDF dt is measured from now,
            //   not from before the maintenance event (harmless when
            //   evidence[]=0 but avoids a spuriously large dt on first update).
            g_lastEvidenceMs  = 0;

            // --- Reset Analytics Task Internal State (V14.4) ---
            // millis accumulators: reset so flush cadence restarts from zero,
            // preventing the immediate re-flush storm after maintenance reset.
            g_accMs_1s  = 0;
            g_accMs_10s = 0;
            g_accMs_60s = 0;
            // g_slotDur1sMs: reset to default 1000ms (V14.8).
            // taskAnalytics recomputes this from live RPM on its very first
            // tick after resume (line ~4499). Without this reset, if the pre-
            // maintenance RPM produced a wide slot (e.g. slow-spin = 2800ms),
            // the first flush threshold after reset is 2800ms instead of the
            // expected 1000ms — analytics appears "stuck" for up to 2.8s.
            g_slotDur1sMs = 1000UL;
            // Raw-buffer bookkeeping: force firstRun so lastHead re-syncs
            g_anaLastHead   = 0;
            g_anaFirstRun   = true;
            g_anaPublishCnt = 0;
            // Per-slot running accumulators
            g_sl_sumRms   = 0.0f;  g_sl_sumSqRms = 0.0f;  g_sl_maxRms  = 0.0f;
            g_sl_sumTemp  = 0.0f;  g_sl_maxTemp  = 0.0f;
            g_sl_sumPeak  = 0.0f;  g_sl_maxPeak  = 0.0f;
            g_sl_sumFrx   = 0.0f;  g_sl_sumFry   = 0.0f;  g_sl_sumFrz  = 0.0f;
            g_sl_spikes   = 0;
            g_sl_n        = 0;

            // Resume processing
            if (taskHandleAnalytics != NULL) {
              vTaskResume(taskHandleAnalytics);
            }

            // ── Enter Warm-up phase ──
            g_systemState.state = STATE_WARMUP;
            g_warmupStartTs = millis();

            // ── Queue maintenance event for Network task to publish (MQTT audit) ──
            // All JSON/MQTT work is done in taskNetwork to keep Button stack lean.
            {
              MaintenanceEvent_t mEvt;
              mEvt.triggerMillis = g_warmupStartTs;
              mEvt.rtcValid      = g_rtcValid;
              if (g_rtcValid) {
                // V14.7: take mutexI2C — taskDisplayUpdate holds it during OLED
                // writes (10 Hz); without the mutex, Wire transactions can interleave
                // causing bus corruption or a wrong timestamp in the audit event.
                // Timeout 100ms is safe: OLED render is < 20ms per frame.
                if (xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(100)) == pdTRUE) {
                  DateTime rtcNow   = rtc.now();
                  xSemaphoreGive(mutexI2C);
                  mEvt.year   = rtcNow.year();
                  mEvt.month  = rtcNow.month();
                  mEvt.day    = rtcNow.day();
                  mEvt.hour   = rtcNow.hour();
                  mEvt.minute = rtcNow.minute();
                  mEvt.second = rtcNow.second();
                } else {
                  // I2C mutex timeout — fill with zeros; audit event is still
                  // queued (timestamp will show 0000-00-00 00:00:00, acceptable).
                  mEvt.year = mEvt.month = mEvt.day = 0;
                  mEvt.hour = mEvt.minute = mEvt.second = 0;
                  Serial.println("[MAINT] mutexI2C timeout — timestamp zeroed");
                }
              } else {
                mEvt.year = mEvt.month = mEvt.day = 0;
                mEvt.hour = mEvt.minute = mEvt.second = 0;
              }
              xQueueSend(queueMaintEvent, &mEvt, 0);  // non-blocking, drop if full
            }

            Serial.println("[MAINT] Reset complete → WARMUP (MQTT event queued)");

          } else {
            xSemaphoreGive(mutexSystemState);
          }
        }
        processedSelect = true;
      }
    }

    // Detect release (short press)
    if (currentStateSelect == HIGH && lastStateSelect == LOW) {
      uint32_t duration = now - pressStartSelect;

      if (duration >= 50 && duration < 800 && !processedSelect) {
        // Short press - Change page
        if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(10)) == pdTRUE) {
          g_systemState.currentPage = (DisplayPage_t)((g_systemState.currentPage + 1) % PAGE_MAX);
          Serial.printf("[CORE 1] SELECT: Page %d\n", g_systemState.currentPage);
          xSemaphoreGive(mutexSystemState);
        }
      }
    }

    lastStateSelect = currentStateSelect;

    // ===== ENTER BUTTON HANDLING =====
    // Detect press
    if (currentStateEnter == LOW && lastStateEnter == HIGH) {
      pressStartEnter = now;
      processedEnter = false;
    }

    // Detect hold duration (long press for alarm ACK)
    if (currentStateEnter == LOW && !processedEnter) {
      uint32_t duration = now - pressStartEnter;

      if (duration >= 2000) {
        // Long press (2s) - Acknowledge alarm
        if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(10)) == pdTRUE) {
          g_systemState.alarmAcknowledged = true;
          g_systemState.buzzerActive = false;
          Serial.println("[CORE 1] ENTER: Alarm ACK (2s)");
          xSemaphoreGive(mutexSystemState);
        }
        processedEnter = true;
      }
    }

    // Detect release (short press - can be used for other functions)
    if (currentStateEnter == HIGH && lastStateEnter == LOW) {
      uint32_t duration = now - pressStartEnter;

      if (duration >= 50 && duration < 800 && !processedEnter) {
        // Short press ENTER - Currently unused, can add functionality
        Serial.println("[CORE 1] ENTER: Short press");
        // Future: Toggle logging, reset stats, etc.
      }
    }

    lastStateEnter = currentStateEnter;

    vTaskDelay(pdMS_TO_TICKS(10));  // 10ms polling
  }
}

/**
 * Task 6: Buzzer Control (CORE 1, Priority 1)
 * Lowest priority, non-critical
 */
void taskBuzzerControl(void* parameter) {
  bool beepState = false;
  uint32_t lastBeep = 0;

  Serial.println("[CORE 1] Buzzer task started");

  while (1) {
    bool buzzerActive = false;
    bool acknowledged = false;
    MachineState_t state;

    if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(5)) == pdTRUE) {
      buzzerActive = g_systemState.buzzerActive;
      acknowledged = g_systemState.alarmAcknowledged;
      state = g_systemState.state;
      xSemaphoreGive(mutexSystemState);
    }

    if (buzzerActive && !acknowledged) {
      uint32_t now = millis();
      uint32_t interval = (state == STATE_CRITICAL) ? 250 : 500;

      if (now - lastBeep >= interval) {
        beepState = !beepState;
        digitalWrite(PIN_BUZZER, beepState ? HIGH : LOW);
        lastBeep = now;
      }
    } else {
      digitalWrite(PIN_BUZZER, LOW);
    }

    vTaskDelay(pdMS_TO_TICKS(50));  // 50ms check
  }
}

// ============================================================================
// DISPLAY RENDERING FUNCTIONS (CORE 1)
// ============================================================================

/**
 * drawSensorOfflineScreen -- ????????? sensor ????? / ?????????? Modbus
 * ???????????????????????????? sensor ???????? online
 */
void drawSensorOfflineScreen() {
  char buf[32];

  // Header -- ????????????????????????????
  u8g2.drawBox(0, 0, 128, 14);          // ?????????
  u8g2.setDrawColor(0);                 // ????? (invert)
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(12, 10, "! SENSOR OFFLINE !");
  u8g2.setDrawColor(1);                 // ??????????

  // Divider
  u8g2.drawHLine(0, 16, 128);

  // Line 2: ??????????
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 28, "Modbus: No Response");

  // Line 3: error count
  snprintf(buf, sizeof(buf), "RS485 Err: %lu", g_sensorErrors);
  u8g2.drawStr(0, 40, buf);

  // Line 4: uptime ??????????????? board ??? alive
  snprintf(buf, sizeof(buf), "Uptime: %lu s", millis() / 1000);
  u8g2.drawStr(0, 52, buf);

  // Line 5: hint
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(0, 63, "Check power / RS485 wiring");
}

void drawMachineScreen(VibrationData_t* data) {
  char buf[32];

  // Line 1: Header (y=10)
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 10, MACHINE_NAME);
  u8g2.drawStr(100, 10, "[1/3]");

  // Line 2: STATUS (y=22)
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(15, 24, "STATUS: NORMAL");

  // Line 3: RMS value (y=38) - LARGE
  u8g2.setFont(u8g2_font_ncenB10_tr);
  snprintf(buf, sizeof(buf), "%.2f", data->rms_overall);
  uint8_t w = u8g2.getStrWidth(buf);
  u8g2.drawStr((128 - w) / 2, 40, buf);

  // Unit
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr((128 - w) / 2 + w + 2, 40, "mm/s");

  // Line 4-5: Thresholds (y=50)
  u8g2.setFont(u8g2_font_6x10_tr);
  snprintf(buf, sizeof(buf), "WARN %.1f", WARNING_RMS);
  u8g2.drawStr(0, 52, buf);

  snprintf(buf, sizeof(buf), "CRIT %.1f", CRITICAL_RMS);
  u8g2.drawStr(80, 52, buf);

  // Line 6: Footer (y=64)
  u8g2.setFont(u8g2_font_5x7_tr);
  snprintf(buf, sizeof(buf), "T:%.1fC", data->temperature);
  u8g2.drawStr(0, 64, buf);

  // Show 4G status instead of MQTT
  if (g_network.gprsConnected) {
    snprintf(buf, sizeof(buf), "4G:%d%%", g_network.signalPercent);
  } else {
    snprintf(buf, sizeof(buf), "4G:--");
  }
  u8g2.drawStr(80, 64, buf);
}

void drawWarningScreen(VibrationData_t* data, bool blink) {
  char buf[32];

  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 10, MACHINE_NAME);
  u8g2.drawStr(100, 10, "[1/3]");

  if (blink) {
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(30, 26, "! WARNING !");
  }

  u8g2.setFont(u8g2_font_ncenB10_tr);
  snprintf(buf, sizeof(buf), "%.2f", data->rms_overall);
  u8g2.drawStr(40, 40, buf);
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(90, 40, "mm/s");

  u8g2.setFont(u8g2_font_6x10_tr);
  snprintf(buf, sizeof(buf), "CRIT: %.1f", CRITICAL_RMS);
  u8g2.drawStr(20, 54, buf);

  u8g2.setFont(u8g2_font_5x7_tr);
  snprintf(buf, sizeof(buf), "X:%.1f Y:%.1f Z:%.1f",
           data->rms_x, data->rms_y, data->rms_z);
  u8g2.drawStr(0, 64, buf);
}

void drawCriticalScreen(VibrationData_t* data, bool blink) {
  char buf[32];

  if (blink) {
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(30, 16, "! CRITICAL !");
  }

  u8g2.setFont(u8g2_font_ncenB10_tr);
  snprintf(buf, sizeof(buf), "%.2f", data->rms_overall);
  u8g2.drawStr(40, 30, buf);

  u8g2.setFont(u8g2_font_5x7_tr);
  snprintf(buf, sizeof(buf), "LIMIT: %.1f mm/s", CRITICAL_RMS);
  u8g2.drawStr(20, 40, buf);

  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(20, 52, "ACTION: STOP");

  u8g2.setFont(u8g2_font_5x7_tr);
  snprintf(buf, sizeof(buf), "T:%.1fC", data->temperature);
  u8g2.drawStr(0, 64, buf);
  u8g2.drawStr(100, 64, "[1/3]");
}

void drawAxisScreen(VibrationData_t* data) {
  char buf[20];

  // === Column headers (bold font) ===
  u8g2.setFont(u8g2_font_6x12_tf);  // bold-ish for headers
  u8g2.drawStr(3, 10, "VEL.(mm/s)");
  u8g2.drawStr(75, 10, "FRE.(Hz)");

  // Underline below each header
  u8g2.drawHLine(3, 13, 58);   // left column underline
  u8g2.drawHLine(70, 13, 55);  // right column underline

  // Vertical divider (full height below header)
  u8g2.drawVLine(64, 0, 64);

  // === Data rows (normal font) ===
  u8g2.setFont(u8g2_font_6x10_tr);

  // Row 1: VX / FX   (y=26)
  snprintf(buf, sizeof(buf), "VX = %03.2f", data->rms_x);
  u8g2.drawStr(5, 26, buf);
  snprintf(buf, sizeof(buf), "FX = %02.0f", data->freq_x);
  u8g2.drawStr(70, 26, buf);

  // Row 2: VY / FY   (y=37)
  snprintf(buf, sizeof(buf), "VY = %03.2f", data->rms_y);
  u8g2.drawStr(5, 37, buf);
  snprintf(buf, sizeof(buf), "FY = %02.0f", data->freq_y);
  u8g2.drawStr(70, 37, buf);

  // Row 3: VZ / FZ   (y=48)
  snprintf(buf, sizeof(buf), "VZ = %03.2f", data->rms_z);
  u8g2.drawStr(5, 48, buf);
  snprintf(buf, sizeof(buf), "FZ = %02.0f", data->freq_z);
  u8g2.drawStr(70, 48, buf);

  // Row 4: MAX / [2/3]  (y=60)
  snprintf(buf, sizeof(buf), "MAX= %03.2f", data->rms_overall);
  u8g2.drawStr(5, 60, buf);
  u8g2.drawStr(98, 60, "[2/3]");
}

void drawNetworkScreen() {
  char buf[32];

  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 10, "4G NETWORK");
  u8g2.drawStr(100, 10, "[3/3]");

  // Modem status
  const char* modemStatus = "UNKNOWN";
  switch (g_network.modemState) {
    case MODEM_STATE_OFF: modemStatus = "OFF"; break;
    case MODEM_STATE_INITIALIZING: modemStatus = "INIT..."; break;
    case MODEM_STATE_SEARCHING: modemStatus = "SEARCH"; break;
    case MODEM_STATE_REGISTERED: modemStatus = "REG OK"; break;
    case MODEM_STATE_GPRS_CONNECTING: modemStatus = "GPRS..."; break;
    case MODEM_STATE_GPRS_CONNECTED: modemStatus = "ONLINE"; break;
    case MODEM_STATE_ERROR: modemStatus = "ERROR"; break;
  }
  snprintf(buf, sizeof(buf), "Status: %s", modemStatus);
  u8g2.drawStr(5, 22, buf);

  // Operator and Signal
  if (g_network.gprsConnected) {
    snprintf(buf, sizeof(buf), "Op: %.10s", g_network.operatorName);
    u8g2.drawStr(5, 32, buf);

    snprintf(buf, sizeof(buf), "Signal: %d%% (CSQ:%d)",
             g_network.signalPercent, g_network.signalQuality);
    u8g2.drawStr(5, 42, buf);
  } else {
    u8g2.drawStr(5, 32, "Op: ---");
    u8g2.drawStr(5, 42, "Signal: ---");
  }

  // MQTT status
  snprintf(buf, sizeof(buf), "MQTT: %s",
           mqttClient.connected() ? "CONN" : "DISC");
  u8g2.drawStr(5, 52, buf);

  // Publish stats + NTP status
  snprintf(buf, sizeof(buf), "Tx:%d %s",
           g_network.publishCount,
           g_timeSync.synced ? "NTP:OK" : "NTP:--");
  u8g2.drawStr(5, 62, buf);
}

// ============================================================================
// ============================================================================
// PHASE 5: Helper functions -- Gaussian scorer, AEDF, TSI, OSG, FVRI
// ============================================================================

// Gaussian proximity: score = 1.0 at x=center, decays symmetrically with sigma
static float gaussianScore(float x, float center, float sigma) {
  float d = (x - center) / sigma;
  return expf(-0.5f * d * d);
}

// Compute rolling variance of mean_rms over last n slots in an AggSample buffer
static float computeRmsVariance(const AggSample_t* buf, uint16_t head,
                                 uint16_t count, uint16_t bufSize, uint16_t n) {
  uint16_t use = (count < n) ? count : n;
  if (use < 4) return 1e-6f;
  uint16_t startIdx = (head + bufSize - use) % bufSize;
  float sum = 0.0f, sum2 = 0.0f;
  for (uint16_t i = 0; i < use; i++) {
    float v = buf[(startIdx + i) % bufSize].mean_rms;
    sum  += v;
    sum2 += v * v;
  }
  float mean = sum / use;
  float var  = (sum2 / use) - (mean * mean);
  return (var < 0.0f) ? 0.0f : var;
}

// -- Probabilistic Fault Classifier ---------------------------------------
// v15.8 Fix #2: accepts ratioDom (dominant axis) and individual axis ratios.
// ratioDom drives primary harmonic scoring.
// ratioX/Y/Z allow axis-specific checks: imbalance (radial = max of X/Y),
// misalignment (axial = Z dominant), looseness (sub-harmonic on any axis).
static void classifyFaultProbabilistic(float ratioDom,
                                        float ratioX, float ratioY, float ratioZ,
                                        float rpm,
                                        float stddev,
                                        float kurtMax, float cfMax,
                                        float currentRms,
                                        float* scores) {
  // v15.8 Fix #2: kurtosis/CF-based bearing scoring works even when ratioDom=0
  // (bearing damage can occur without a dominant harmonic).
  // Only suppress ratio-dependent faults (imbalance, misalignment, resonance)
  // when ratioDom is not valid.
  bool ratioValid = (rpm >= 100.0f && ratioDom >= 0.01f);

  if (!ratioValid) {
    // Still evaluate kurtosis and CF for bearing detection
    for (int i = 0; i < FAULT_TYPE_COUNT; i++) scores[i] = 0.0f;
    scores[FAULT_UNKNOWN] = 0.5f;

    float kurtNorm2 = constrain((kurtMax - 3.0f) / 3.0f, 0.0f, 1.0f);
    float cfNorm2   = constrain((cfMax   - 2.5f) / 4.5f, 0.0f, 1.0f);
    float vol2      = fminf(stddev / FAULT_VOLAT_LOOSE, 1.0f);
    // v15.8 Fix #3 formula applied here too (see bearing score below)
    scores[FAULT_BEARING] = constrain(0.55f*kurtNorm2 + 0.30f*cfNorm2 + 0.15f*vol2,
                                       0.0f, 1.0f);
    return;
  }

  // v15.8: radial ratio = dominant of X/Y (imbalance = radial vibration)
  // axial ratio = Z (misalignment has strong axial component)
  float ratioRadial = fmaxf(ratioX, ratioY);
  float ratioAxial  = ratioZ;

  float sub  = gaussianScore(ratioDom, 0.45f, 0.08f);
  float vol  = fminf(stddev / FAULT_VOLAT_LOOSE, 1.0f);

  // Harmonic scores: imbalance uses radial dominant, misalignment uses axial
  float h1   = gaussianScore(ratioRadial, FAULT_IMBALANCE_CTR,  FAULT_RATIO_TOL);
  float h2   = fmaxf(gaussianScore(ratioRadial, FAULT_MISALIGN_CTR, FAULT_RATIO_TOL),
                     gaussianScore(ratioAxial,  FAULT_MISALIGN_CTR, FAULT_RATIO_TOL) * 1.3f);
  h2 = constrain(h2, 0.0f, 1.0f);  // boost axial 2x for misalignment
  float h3   = gaussianScore(ratioDom, FAULT_LOOSE_CTR, FAULT_RATIO_TOL);
  float bpf  = gaussianScore(ratioDom, (float)PUMP_BLADES,      BPF_RATIO_TOL_WIDE);
  float bpf2 = gaussianScore(ratioDom, (float)PUMP_BLADES*2.0f, BPF_RATIO_TOL_WIDE);
  float bpf3 = gaussianScore(ratioDom, (float)PUMP_BLADES*3.0f, BPF_RATIO_TOL_WIDE);
  float nonI = constrain(1.0f - fmaxf(h1, fmaxf(h2, fmaxf(h3, fmaxf(bpf, fmaxf(bpf2, bpf3))))), 0.0f, 1.0f);

  float kurtNorm = constrain((kurtMax - 3.0f) / 3.0f, 0.0f, 1.0f);
  float cfNorm   = constrain((cfMax   - 2.5f) / 4.5f, 0.0f, 1.0f);
  float cfLow    = constrain(1.0f - cfMax / 4.0f,      0.0f, 1.0f);

  scores[FAULT_NORMAL]       = constrain(0.70f*fmaxf(bpf,fmaxf(bpf2,bpf3)) + 0.20f*(1.0f-kurtNorm) + 0.10f*(1.0f-vol), 0.0f, 1.0f);
  scores[FAULT_IMBALANCE]    = constrain(0.70f*h1  + 0.20f*(1.0f-kurtNorm) + 0.10f*vol, 0.0f, 1.0f);
  scores[FAULT_MISALIGNMENT] = constrain(0.75f*h2  + 0.25f*vol*0.5f, 0.0f, 1.0f);
  scores[FAULT_LOOSENESS]    = constrain(0.30f*h3  + 0.25f*sub + 0.30f*vol + 0.15f*cfLow, 0.0f, 1.0f);

  // v15.8 Fix #3: bearing score -- see below
  scores[FAULT_BEARING]      = constrain(0.55f*kurtNorm + 0.30f*cfNorm + 0.15f*vol, 0.0f, 1.0f);

  if (ratioDom > RESONANCE_RATIO_MIN && currentRms >= BASELINE_RMS) {
    scores[FAULT_RESONANCE] = constrain(0.60f*nonI + 0.20f*(1.0f-bpf) + 0.20f*(1.0f-bpf2), 0.0f, 1.0f);
  } else {
    scores[FAULT_RESONANCE] = 0.0f;
  }

  scores[FAULT_UNKNOWN]      = constrain(0.30f*(1.0f - fmaxf(h1, fmaxf(h2, fmaxf(h3,bpf))))
                               * (1.0f-kurtNorm*0.5f) * (1.0f-vol*0.5f), 0.0f, 1.0f);
}

// -- AEDF: Adaptive Evidence Decay Function --------------------------------
static void updateAEDF(const float* rawScores, uint32_t now_ms) {
  static const float kTau[FAULT_TYPE_COUNT] = {
    AEDF_TAU_NORMAL*30.0f, AEDF_TAU_IMBALANCE*30.0f, AEDF_TAU_MISALIGN*30.0f,
    AEDF_TAU_LOOSENESS*30.0f, AEDF_TAU_BEARING*30.0f,
    AEDF_TAU_RESONANCE*30.0f, AEDF_TAU_UNKNOWN*30.0f,
  };
  float dt = (now_ms > g_lastEvidenceMs)
             ? (float)(now_ms - g_lastEvidenceMs) / 1000.0f : 30.0f;
  g_lastEvidenceMs = now_ms;
  if (dt < 0.1f) return;

  float topRival = 0.0f; int topI = 0;
  for (int i = 0; i < FAULT_TYPE_COUNT; i++)
    if (rawScores[i] > topRival) { topRival = rawScores[i]; topI = i; }

  for (int i = 0; i < FAULT_TYPE_COUNT; i++) {
    g_faultEvidence[i] *= expf(-dt / kTau[i]);
    g_faultEvidence[i]  = (1.0f-AEDF_BLEND)*g_faultEvidence[i] + AEDF_BLEND*rawScores[i];
    if (i != topI && (topRival - rawScores[i]) > AEDF_CONTRADICT)
      g_faultEvidence[i] *= AEDF_RESET_MUL;
    g_faultEvidence[i] = constrain(g_faultEvidence[i], 0.0f, 1.0f);
  }
}

static float applyAEDF(int faultIdx, float rawScore) {
  float ev = g_faultEvidence[faultIdx];
  float evB = (ev > AEDF_PERSIST_THRESH) ? fminf(ev*1.15f, 1.0f) : ev;
  return 0.5f*rawScore + 0.5f*evB;
}

// -- TSI: Trend Stability Index --------------------------------------------
static float computeTSI(void) {
  int total=0, pos=0, neg=0;
  if (g_trendResult.slope_ready_1s)  { total++; if (g_trendResult.slope_1s >0) pos++; else neg++; }
  if (g_trendResult.slope_ready_10s) { total++; if (g_trendResult.slope_10s>0) pos++; else neg++; }
  if (g_trendResult.slope_ready_60s) { total++; if (g_trendResult.slope_60s>0) pos++; else neg++; }
  if (total==0) return 0.0f;
  return (float)((pos>=neg)?pos:neg) / (float)total;
}

// -- Adaptive Severity Weights ---------------------------------------------
static void adaptWeights(FaultType_t fault, float tsi, float wOut[4]) {
  float w[4] = { 0.35f, 0.25f, 0.25f, 0.15f };
  switch (fault) {
    case FAULT_BEARING:     w[2]*=1.5f; w[3]*=1.4f; w[0]*=0.8f; break;
    case FAULT_LOOSENESS:   w[3]*=1.6f; w[1]*=0.7f; w[0]*=0.9f; break;
    case FAULT_IMBALANCE:   w[1]*=1.3f; w[0]*=1.1f;              break;
    case FAULT_MISALIGNMENT:w[1]*=1.2f; w[2]*=1.1f;              break;
    default: break;
  }
  w[1] *= tsi;
  float total = w[0]+w[1]+w[2]+w[3];
  if (total < 0.01f) { wOut[0]=0.4f; wOut[1]=0.2f; wOut[2]=0.2f; wOut[3]=0.2f; return; }
  for (int i=0;i<4;i++) wOut[i]=w[i]/total;
}

// -- OSG: Oscillatory Suppression Gate ------------------------------------
static float osgSuppression(float s_target, float s_contra,
                             uint8_t conf_contra, bool ready_contra) {
  if (!ready_contra)    return 1.0f;
  if (s_contra >= 0.0f) return 1.0f;
  if (s_target <= 0.0f) return 1.0f;
  float disagree = fabsf(s_contra) / (fabsf(s_target)+fabsf(s_contra)+1e-9f);
  float mul = 1.0f - (disagree * ((float)conf_contra/100.0f));
  return fmaxf(mul, OSG_FLOOR);
}

// -- FVRI: Fault-Volatility Resonance Index --------------------------------
static float fvriAdjust(float baseRel, float slopeVar,
                         FaultType_t fault, float faultScore) {
  if (faultScore < 0.50f) return baseRel;
  switch (fault) {
    case FAULT_BEARING:
      if (slopeVar > FVRI_VAR_BEARING)
        return baseRel + (1.0f-baseRel)*faultScore*0.5f;
      return baseRel;
    case FAULT_LOOSENESS:
      if (slopeVar > FVRI_VAR_LOOSENESS)
        return baseRel*(1.0f-faultScore*0.4f);
      return baseRel;
    case FAULT_IMBALANCE:
      return (slopeVar > FVRI_VAR_IMBALANCE) ? baseRel*0.7f : baseRel*1.1f;
    default:
      return baseRel;
  }
}

static float varToReliability(float variance) {
  return 1.0f / (1.0f + variance * 1e7f);
}

// -- Adaptive publish interval ---------------------------------------------
static uint32_t computeAdaptiveInterval(uint8_t finalState,
                                         float ttwRoC, bool stateChanged) {
  uint32_t base = (finalState==2)?5000UL:(finalState==1)?10000UL:30000UL;
  float mod = 1.0f;
  if      (ttwRoC < -0.5f) mod = 0.50f;
  else if (ttwRoC < -0.1f) mod = 0.75f;
  if (stateChanged) mod *= 0.5f;
  uint32_t iv = (uint32_t)((float)base*mod);
  return (iv < 3000UL) ? 3000UL : iv;
}

// ============================================================================
// PHASE 5: runDecisionEngine() -- full adaptive fusion pipeline
// ============================================================================
static void runDecisionEngine(float currentRms, MachineState_t currentState) {

  // -- 0. AEDF flush เมื่อ rms กลับมาจาก 0 ────────────────────────────────
  // กรณี: sensor เพิ่ง config MODE=FreqDomain สำเร็จ → rms เปลี่ยนจาก 0 → มีค่า
  // AEDF อาจสะสม evidence จาก freq_ratio noise ขณะ rms=0 (STARTING, MODE ผิด)
  // → reset evidence ทันทีเพื่อให้ decision เริ่มต้นใหม่จาก data จริง
  {
    static bool s_prevRmsWasZero = true;
    bool curRmsZero = (currentRms <= 0.0f);
    if (s_prevRmsWasZero && !curRmsZero) {
      // rms transition 0 → non-zero: flush stale AEDF evidence
      for (int i = 0; i < FAULT_TYPE_COUNT; i++) {
        g_faultEvidence[i] = 0.0f;
        g_rawScores[i]     = 0.0f;
      }
      g_lastEvidenceMs = 0;
      Serial.println("[DEC5] rms 0→nonzero: AEDF evidence flushed (sensor MODE now valid)");
    }
    s_prevRmsWasZero = curRmsZero;
  }

  // -- 1. Probabilistic fault classification + AEDF ----------------------
  float rawScores[FAULT_TYPE_COUNT];
  float latestRpm = 0.0f;
  float kurtMax   = 0.0f;
  float cfMax     = 0.0f;
  uint8_t motorState = 0;
  if (xSemaphoreTake(mutexVibData, pdMS_TO_TICKS(3)) == pdTRUE) {
    latestRpm  = g_vibData.rpm;
    kurtMax    = g_vibData.kurtosis_max;  // v15.5 R2
    cfMax      = g_vibData.cf_max;        // v15.5 R2
    motorState = g_vibData.motor_state;   // v15.5 R2: MotorRunState_t (0=STOP,1=START,2=RUN,3=STOP)
    xSemaphoreGive(mutexVibData);
  }
  // v15.5 R2: Gate kurtMax/cfMax — ไม่ใช้ค่าขณะ transient (start/stop)
  // เหมือน bearing_alert gate ใน publishTelemetry (Fix 16 v15.2)
  if (motorState != 2) {
    kurtMax = 0.0f;   // STOPPED/STARTING/STOPPING → kurtosis ไม่ valid
    cfMax   = 0.0f;
  }
  // v15.8 Fix #2: Use dominant-axis freq_ratio instead of 3-axis mean.
  // Before: ratioMean = (frx + fry + frz) / 3  -- destroys axis-specific signatures
  // After:  ratioDominant = max(frx, fry, frz) -- preserves the most significant
  //         harmonic pattern. Pass individual ratios so classifier can use them.
  // ratioDom is passed as the primary ratio; individual values are also forwarded
  // for per-fault harmonic scoring inside classifyFaultProbabilistic.
  float ratioDom = 0.0f;
  float ratioX   = 0.0f;
  float ratioY   = 0.0f;
  float ratioZ   = 0.0f;
  {
    uint16_t lastIdx = (g_trendHead + TREND_BUF_SIZE - 1) % TREND_BUF_SIZE;
    const TrendSample_t* ls = &g_trendBuf[lastIdx];
    bool freqRatioValid = (ls->rms > 0.0f) &&
                          (ls->freq_ratio_x > 0.01f ||
                           ls->freq_ratio_y > 0.01f ||
                           ls->freq_ratio_z > 0.01f);
    if (freqRatioValid) {
      ratioX   = ls->freq_ratio_x;
      ratioY   = ls->freq_ratio_y;
      ratioZ   = ls->freq_ratio_z;
      ratioDom = fmaxf(ratioX, fmaxf(ratioY, ratioZ));  // dominant axis
    }
  }
  classifyFaultProbabilistic(ratioDom, ratioX, ratioY, ratioZ, latestRpm,
                              g_trendResult.stddev_1min,
                              kurtMax, cfMax,
                              currentRms,
                              rawScores);
  // Patch A: snapshot pre-AEDF scores globally for /fusion/raw publication
  memcpy(g_rawScores, rawScores, sizeof(g_rawScores));
  updateAEDF(rawScores, millis());

  float finalScores[FAULT_TYPE_COUNT];
  for (int i=0;i<FAULT_TYPE_COUNT;i++) finalScores[i]=applyAEDF(i,rawScores[i]);

  int top1=0, top2=1;
  for (int i=1;i<FAULT_TYPE_COUNT;i++) {
    if (finalScores[i]>finalScores[top1]) { top2=top1; top1=i; }
    else if (finalScores[i]>finalScores[top2]) top2=i;
  }
  FaultType_t bestFault   = (FaultType_t)top1;
  float       bestScore   = finalScores[top1];
  float       faultUncert = constrain(1.0f-(finalScores[top1]-finalScores[top2]),0.0f,1.0f);
  g_trendResult.fault_type = bestFault;

  // -- 2. Adaptive severity with TSI-modulated weights --------------------
  float tsi = computeTSI();
  float s1_ps  = g_trendResult.slope_ready_1s  ? g_trendResult.slope_1s          : 0.0f;
  float s10_ps = g_trendResult.slope_ready_10s ? g_trendResult.slope_10s/10.0f   : 0.0f;
  float s60_ps = g_trendResult.slope_ready_60s ? g_trendResult.slope_60s/60.0f   : 0.0f;
  const float kRisingThresh = 0.0005f;
  uint8_t risingCount = 0;
  if (s1_ps  > kRisingThresh) risingCount++;
  if (s10_ps > kRisingThresh) risingCount++;
  if (s60_ps > kRisingThresh) risingCount++;
  bool emaRising = (g_trendResult.ema_dir==1 && g_trendResult.ema_delta>kRisingThresh);

  float rmsNorm   = constrain((currentRms-BASELINE_RMS)/(CRITICAL_RMS-BASELINE_RMS),0.0f,1.0f);
  float volNorm   = constrain(g_trendResult.stddev_1min/0.35f,0.0f,1.0f);
  float trendNorm = fminf((float)risingCount/3.0f,1.0f);
  float faultNorm;
  switch (bestFault) {
    case FAULT_NORMAL:       faultNorm=0.00f; break;
    case FAULT_UNKNOWN:      faultNorm=0.10f; break;
    case FAULT_RESONANCE:    faultNorm=0.25f; break;
    case FAULT_IMBALANCE:    faultNorm=0.40f; break;
    case FAULT_MISALIGNMENT: faultNorm=0.60f; break;
    case FAULT_LOOSENESS:    faultNorm=0.75f; break;
    case FAULT_BEARING:      faultNorm=1.00f; break;
    default:                 faultNorm=0.10f; break;
  }
  float w[4]; adaptWeights(bestFault, tsi, w);
  uint8_t severity = (uint8_t)constrain((w[0]*rmsNorm+w[1]*trendNorm+w[2]*faultNorm+w[3]*volNorm)*100.0f, 0.0f, 100.0f);

  // -- 3. Variance-based adaptive TTW ensemble with OSG + FVRI ----------
  bool medReady    = g_trendResult.slope_ready_10s;
  bool longReady   = g_trendResult.slope_ready_60s;
  bool medFalling  = medReady  && (g_trendResult.slope_10s < 0.0f);
  bool longFalling = longReady && (g_trendResult.slope_60s < 0.0f);
  bool allFalling  = medFalling && longFalling;

  float r_s = fvriAdjust(varToReliability((float)g_slopeVar_1s),  (float)g_slopeVar_1s,  bestFault, bestScore);
  float r_m = fvriAdjust(varToReliability((float)g_slopeVar_10s), (float)g_slopeVar_10s, bestFault, bestScore);
  float r_l = fvriAdjust(varToReliability((float)g_slopeVar_60s), (float)g_slopeVar_60s, bestFault, bestScore);

  uint8_t conf_10s = (uint8_t)constrain((float)g_buf10sCount/30.0f*60.0f, 0.0f, 60.0f);
  uint8_t conf_60s = (uint8_t)constrain((float)g_buf60sCount/40.0f*60.0f, 0.0f, 60.0f);
  r_s *= osgSuppression(s1_ps,  s10_ps, conf_10s, medReady);
  r_m *= osgSuppression(s10_ps, s60_ps, conf_60s, longReady);
  // Patch B: snapshot OSG*FVRI reliability multipliers for /fusion/osg publication
  g_osg_r_s = r_s;
  g_osg_r_m = r_m;

  float gap = WARNING_RMS - currentRms;
  float ttw_short=0.0f, ttw_medium=0.0f, ttw_long=0.0f, ttw_best=0.0f;

  if (!medFalling && !allFalling && g_trendResult.slope_ready_1s && s1_ps>0.0001f) {
    float rh=s1_ps*3600.0f;
    if (rh>=0.30f && gap>0.0f) ttw_short=constrain(gap/rh,0.0f,TTW_MAX_HOURS);
    else r_s=0.0f;
  } else { r_s=0.0f; }

  if (!longFalling && medReady && s10_ps>0.0001f) {
    float rh=s10_ps*3600.0f;
    if (rh>=0.15f && gap>0.0f) ttw_medium=constrain(gap/rh,0.0f,TTW_MAX_HOURS);
    else r_m=0.0f;
  } else { r_m=0.0f; }

  if (longReady && s60_ps>0.0001f) {
    float rh=s60_ps*3600.0f;
    if (rh>=TTW_LONG_MIN_RATE_H && gap>0.0f) ttw_long=constrain(gap/rh,0.0f,TTW_MAX_HOURS);
    else r_l=0.0f;
  } else { r_l=0.0f; }

  float rTotal=r_s+r_m+r_l;
  float w_s=0.0f, w_m=0.0f, w_l=0.0f;
  if (rTotal>0.001f) {
    w_s=r_s/rTotal; w_m=r_m/rTotal; w_l=r_l/rTotal;
    ttw_best=w_s*ttw_short+w_m*ttw_medium+w_l*ttw_long;
  } else {
    ttw_best=g_trendResult.ttw_hours;
  }
  // Hysteresis: dampen jumps >20%
  if (g_prevTtwBest>0.0f && ttw_best>0.0f) {
    float delta=fabsf(ttw_best-g_prevTtwBest)/g_prevTtwBest;
    if (delta>0.20f) ttw_best=0.30f*ttw_best+0.70f*g_prevTtwBest;
  }
  g_prevTtwBest=ttw_best;

  // -- 4. Orthogonal geometric-mean confidence ----------------------------
  float depthScore=0.0f;
  if (g_buf1sCount  >= SLOPE_1S_MIN_SLOTS)  depthScore+=0.20f;
  if (g_buf1sCount  >= 30)                  depthScore+=0.05f;
  if (g_buf10sCount >= SLOPE_10S_MIN_SLOTS) depthScore+=0.20f;
  if (g_buf10sCount >= 30)                  depthScore+=0.05f;
  if (g_buf60sCount >= SLOPE_60S_MIN_SLOTS) depthScore+=0.35f;
  if (g_buf60sCount >= 40)                  depthScore+=0.15f;
  depthScore=constrain(depthScore,0.0f,1.0f);
  float agreeScore=tsi;
  if (emaRising && risingCount>=2) agreeScore=fminf(agreeScore+0.1f,1.0f);
  float sepScore=constrain(1.0f-faultUncert,0.0f,1.0f);
  float geoMean=powf(depthScore*agreeScore*sepScore+1e-6f, 1.0f/3.0f);
  uint8_t confidence=(uint8_t)constrain(geoMean*100.0f,0.0f,100.0f);

  // -- 5. Context-conditioned dynamic TTW threshold ----------------------
  float ctxThresh=CTX_THRESH_BASE;
  if (bestFault==FAULT_BEARING) ctxThresh=CTX_THRESH_BEARING;
  else if (severity>=75 && confidence>=60) ctxThresh=CTX_THRESH_CRITICAL;
  ctxThresh *= (1.0f+(50.0f-(float)severity)/100.0f);
  ctxThresh *= (1.0f+(50.0f-(float)confidence)/200.0f);
  ctxThresh  = constrain(ctxThresh, 0.3f, 8.0f);

  uint8_t rawPred=0;
  if (currentState==STATE_CRITICAL) {
    rawPred=2;
  } else if (currentState==STATE_WARNING) {
    rawPred=1;
  } else {
    bool longConf=(w_l>0.0f)||(longReady&&risingCount>=2);
    bool medConf =(w_m>0.0f)||(medReady &&risingCount>=2);
    if (ttw_best>0.0f && ttw_best<ctxThresh && (longConf||medConf)) rawPred=2;
    else if (ttw_best>0.0f && ttw_best<ctxThresh)                   rawPred=1;
    else if (ttw_best>0.0f && ttw_best<8.0f)                        rawPred=1;
    else if (risingCount>=2 && severity>=50)                         rawPred=1;
    else                                                              rawPred=0;
  }

  // -- 6. Typed conflict resolution + state damping ---------------------
  uint8_t finalPred=rawPred;
  // Type A: fault abnormal but pred=0
  if (bestFault!=FAULT_NORMAL && bestFault!=FAULT_UNKNOWN
      && finalPred==0 && bestScore>FUSION_FAULT_SCORE_MIN) finalPred=1;
  // Type B: fault normal but pred=2, low confidence
  if (bestFault==FAULT_NORMAL && finalPred==2 && confidence<70) finalPred=1;
  // Type C: high severity overrides pred=0
  if (severity>=FUSION_HIGH_SEV && finalPred==0) finalPred=1;
  // Type D: ambiguous fault -- penalize severity and cap pred
  if (faultUncert>FUSION_UNCERTAINTY_MAX) {
    severity=(uint8_t)((float)severity*FUSION_UNCERTAINTY_PENALTY);
    if (finalPred==2 && confidence<FUSION_TYPE_D_CONF_MIN) finalPred=1;
  }
  // v15.5 R3a: Type E — bearing_alert CONFIRMED (kurtMax≥6) ขณะ RUNNING
  // ยก finalPred อย่างน้อยเป็น WARNING (1) เพื่อให้ action_needed ได้รับการพิจารณา
  // gate ด้วย motorState==2 (อยู่ใน runDecisionEngine scope จาก R2 read ข้างต้น)
  if (kurtMax >= 6.0f && motorState == MOTOR_RUNNING && finalPred < 1) {
    finalPred = 1;   // v15.5 R3a: Type E bearing override
  }
  // State damping: max 1 step per cycle
  if (finalPred>g_prevFinalState+1) finalPred=g_prevFinalState+1;
  if (g_prevFinalState>0 && finalPred<g_prevFinalState-1) finalPred=g_prevFinalState-1;

  bool stateChanged=(finalPred!=g_prevFinalState);
  if (stateChanged) g_stateChangeCyc=3;
  else if (g_stateChangeCyc>0) g_stateChangeCyc--;
  g_prevFinalState=finalPred;

  // -- 7. Alarm class -----------------------------------------------------
  AlarmClass_t alarmCls;
  if      (finalPred==2)                  alarmCls=ALARM_CRITICAL;
  else if (finalPred==1 && severity>=50)  alarmCls=ALARM_WARNING;
  else if (finalPred==1)                  alarmCls=ALARM_INFORMATIONAL;
  else                                    alarmCls=ALARM_SILENT;

  // -- 8. action_needed -------------------------------------------------
  // gate ด้วย MOTOR_RUNNING เสมอ ป้องกัน false alarm ระหว่าง transient
  bool bearingConfirmed = (kurtMax >= 6.0f && motorState == MOTOR_RUNNING);
  bool bearingEarlyWarn = (kurtMax >= 4.0f && motorState == MOTOR_RUNNING
                           && severity >= 30 && bestFault == FAULT_BEARING);
  bool action=(severity>=60)||(finalPred>=2)||
              (bestFault==FAULT_BEARING)||
              bearingConfirmed ||
              (bearingEarlyWarn && bestFault==FAULT_BEARING) ||
              (bestFault==FAULT_LOOSENESS && severity>=40)||
              (g_trendResult.freq_alert && risingCount>=2);

  // -- 9. Adaptive publish interval -------------------------------------
  float ttwRoC=(g_prevTtwRoC>0.0f&&ttw_best>0.0f)?(ttw_best-g_prevTtwRoC):0.0f;
  g_prevTtwRoC=ttw_best;
  uint32_t pubInterval=computeAdaptiveInterval(finalPred,ttwRoC,g_stateChangeCyc>0);

  // -- 10. Reason string -------------------------------------------------
  // v15.7 Bug #3 fix: bearingConfirmed/bearingEarlyWarn must be evaluated
  // BEFORE the generic "bearing+spk" check (bestScore>0.50) because that check
  // would mask the more specific bearing_confirmed reason when score is high.
  char reason[48]="normal";
  if      (bearingConfirmed)
    snprintf(reason,sizeof(reason),"bearing_confirmed+kurt=%.2f",kurtMax);
  else if (bearingEarlyWarn)
    snprintf(reason,sizeof(reason),"bearing_early+kurt=%.2f+sc=%.2f",kurtMax,bestScore);
  else if (bestFault==FAULT_BEARING && bestScore>0.50f)
    snprintf(reason,sizeof(reason),"bearing+kurt=%.2f+sc=%.2f",kurtMax,bestScore);
  else if (bestFault==FAULT_LOOSENESS && bestScore>0.50f)
    snprintf(reason,sizeof(reason),"looseness+sd=%.2f+sc=%.2f",g_trendResult.stddev_1min,bestScore);
  else if (bestFault==FAULT_MISALIGNMENT && risingCount>=2)
    snprintf(reason,sizeof(reason),"misalign+rise=%u",risingCount);
  else if (bestFault==FAULT_IMBALANCE && risingCount>=1)
    snprintf(reason,sizeof(reason),"imbalance+rise=%u",risingCount);
  else if (finalPred>=2 && ttw_best>0.0f)
    snprintf(reason,sizeof(reason),"ttw_%.1fh_CRIT_c=%u",ttw_best,confidence);
  else if (risingCount>=FAULT_AGREE_THRESH)
    snprintf(reason,sizeof(reason),"rising_%u_tsi=%.2f",risingCount,tsi);
  else if (finalPred>=1 && ttw_best>0.0f)
    snprintf(reason,sizeof(reason),"ttw_%.1fh_WARN_c=%u",ttw_best,confidence);
  else if (g_trendResult.freq_alert)
    snprintf(reason,sizeof(reason),"freq_drift");
  else if (severity>=40)
    snprintf(reason,sizeof(reason),"sev=%u_tsi=%.2f",severity,tsi);

  // -- Commit -------------------------------------------------------------
  g_decision.fault_type          = bestFault;
  g_decision.fault_score         = roundf(bestScore*100.0f)/100.0f;
  g_decision.fault_uncertainty   = roundf(faultUncert*100.0f)/100.0f;
  g_decision.severity_score      = severity;
  g_decision.confidence          = confidence;
  g_decision.predicted_state     = rawPred;
  g_decision.final_state         = finalPred;
  g_decision.alarm_class         = alarmCls;
  g_decision.ttw_short_h         = roundf(ttw_short *10.0f)/10.0f;
  g_decision.ttw_medium_h        = roundf(ttw_medium*10.0f)/10.0f;
  g_decision.ttw_long_h          = roundf(ttw_long  *10.0f)/10.0f;
  g_decision.ttw_best_h          = roundf(ttw_best  *10.0f)/10.0f;
  g_decision.ttw_w_s             = roundf(w_s *100.0f)/100.0f;
  g_decision.ttw_w_m             = roundf(w_m *100.0f)/100.0f;
  g_decision.ttw_w_l             = roundf(w_l *100.0f)/100.0f;
  g_decision.action_needed       = action;
  g_decision.publish_interval_ms = pubInterval;
  strncpy(g_decision.reason, reason, sizeof(g_decision.reason)-1);
  g_decision.reason[sizeof(g_decision.reason)-1]='\0';

  Serial.printf("[DEC5] %s(sc=%.2f unc=%.2f) sev=%u tsi=%.2f conf=%u%% "
                "raw=%u fin=%u %s ttw=%.1fh(s=%.1f*%.2f m=%.1f*%.2f l=%.1f*%.2f) "
                "act=%d pub=%lus | %s\n",
                faultTypeStr(bestFault),bestScore,faultUncert,
                severity,tsi,confidence,rawPred,finalPred,alarmClassStr(alarmCls),
                ttw_best,ttw_short,w_s,ttw_medium,w_m,ttw_long,w_l,
                (int)action,(unsigned long)(pubInterval/1000),reason);
}

// ANALYTICS HELPERS -- ????????????????? taskAnalytics ??? calcTrend
// ============================================================================

// Generic push ??? circular buffer (?????????? mutex -- caller ???? hold mutex)
static void pushAggBuf(AggSample_t* buf, volatile uint16_t* head,
                       volatile uint16_t* count, uint16_t maxSize,
                       const AggSample_t* sample) {
  buf[*head] = *sample;
  *head = (*head + 1) % maxSize;
  if (*count < maxSize) (*count)++;
}

// Linear regression slope ?? mean_rms ??? AggSample_t buffer
// ??????: mm/s ??? 1 slot (caller ?????? 1 slot = ?????????)
// windowSlots: ????? slots ?????????????????? (??????????? head)
static float aggLinRegSlope(const AggSample_t* buf, uint16_t bufHead,
                             uint16_t bufCount, uint16_t bufSize,
                             uint16_t windowSlots) {
  uint16_t n = (bufCount < windowSlots) ? bufCount : windowSlots;
  if (n < 4) return 0.0f;
  uint16_t startIdx = (bufHead + bufSize - n) % bufSize;
  double sumX=0.0, sumX2=0.0, sumY=0.0, sumXY=0.0;
  for (uint16_t i = 0; i < n; i++) {
    uint16_t idx = (startIdx + i) % bufSize;
    double x = (double)i;
    double y = (double)buf[idx].mean_rms;
    sumX  += x;
    sumX2 += x * x;
    sumY  += y;
    sumXY += x * y;
  }
  double denom = (double)n * sumX2 - sumX * sumX;
  if (denom == 0.0) return 0.0f;
  return (float)(((double)n * sumXY - sumX * sumY) / denom);
}

// ============================================================================
// TREND ENGINE -- calcTrend()
// ============================================================================
// ???????? publishTelemetry() (Core 1) ???? build JSON
// ???? snapshot ??? g_trendBuf ? ???????????? -- thread-safe ???????????
// ????? float write ?? ESP32 (Xtensa LX7) ???? atomic 4-byte aligned
//
// ????? Phase 1:
//   1. rms_slope    -- Linear Regression (Least Squares) ??? rms ?? 30s window
//   2. temp_slope   -- Linear Regression ??? temperature
//   3. trend_dir    -- +1/0/-1 ??? rms_slope vs threshold
//   4. spike_count  -- ??? peak > WARNING_RMS x SPIKE_RMS_FACTOR ?? window
//   5. freq_drift   -- ???????????? freq_ratio ??????????????? window
//   6. ttw_hours    -- Time-to-Warning estimate ??? rms_slope + gap
//
// ????? Phase 2 (new):
//   7. slope_1s/10s/60s -- linreg ?? g_buf* multi-resolution
//   8. ema_dir/rms/delta -- snapshot ??? g_ema*
//   9. stddev_1min / max_rms_10min -- volatility indicators
// ============================================================================
static void calcTrend() {
  // -- Snapshot head + count ???????? --
  uint16_t snapHead  = g_trendHead;
  uint16_t snapCount = g_trendCount;

  // -- Phase 1: Single-resolution (30s window) -----------------------------
  if (snapCount >= TREND_MIN_SAMPLES) {
    uint16_t n = (snapCount < TREND_WINDOW_SAMPLES) ? snapCount : TREND_WINDOW_SAMPLES;
    uint16_t startIdx = (snapHead + TREND_BUF_SIZE - n) % TREND_BUF_SIZE;

    double sumX=0, sumX2=0;
    double sumRms=0, sumXRms=0;
    double sumTemp=0, sumXTemp=0;
    double sumFrX=0, sumFrY=0, sumFrZ=0;
    uint16_t spike_count = 0;

    for (uint16_t i = 0; i < n; i++) {
      uint16_t idx = (startIdx + i) % TREND_BUF_SIZE;
      TrendSample_t* s = &g_trendBuf[idx];
      sumX     += i;
      sumX2    += (double)i * i;
      sumRms   += s->rms;
      sumXRms  += (double)i * s->rms;
      sumTemp  += s->temp;
      sumXTemp += (double)i * s->temp;
      sumFrX   += s->freq_ratio_x;
      sumFrY   += s->freq_ratio_y;
      sumFrZ   += s->freq_ratio_z;
      if (s->rms_max > WARNING_RMS * SPIKE_RMS_FACTOR) spike_count++;
    }

    double denom = (double)n * sumX2 - sumX * sumX;
    float rmsSlope  = (denom != 0.0) ? (float)((n * sumXRms  - sumX * sumRms)  / denom) : 0.0f;
    float tempSlope = (denom != 0.0) ? (float)((n * sumXTemp - sumX * sumTemp) / denom) : 0.0f;

    float driftX = 0.0f, driftY = 0.0f, driftZ = 0.0f;
    if (n >= 20) {
      uint16_t half = n / 2;
      // v15.7: skip samples with freq_ratio=0 (motor stopped/STARTING, sensor gate)
      // prevents startup artifact where first-half=0, second-half=5.5x → drift=5.5
      double s1X=0, s1Y=0, s1Z=0;
      uint16_t n1X=0, n1Y=0, n1Z=0;
      double s2X=0, s2Y=0, s2Z=0;
      uint16_t n2X=0, n2Y=0, n2Z=0;
      for (uint16_t i = 0; i < half; i++) {
        uint16_t idx = (startIdx + i) % TREND_BUF_SIZE;
        float fx = g_trendBuf[idx].freq_ratio_x;
        float fy = g_trendBuf[idx].freq_ratio_y;
        float fz = g_trendBuf[idx].freq_ratio_z;
        if (fx > 0.01f) { s1X += fx; n1X++; }
        if (fy > 0.01f) { s1Y += fy; n1Y++; }
        if (fz > 0.01f) { s1Z += fz; n1Z++; }
      }
      for (uint16_t i = half; i < n; i++) {
        uint16_t idx = (startIdx + i) % TREND_BUF_SIZE;
        float fx = g_trendBuf[idx].freq_ratio_x;
        float fy = g_trendBuf[idx].freq_ratio_y;
        float fz = g_trendBuf[idx].freq_ratio_z;
        if (fx > 0.01f) { s2X += fx; n2X++; }
        if (fy > 0.01f) { s2Y += fy; n2Y++; }
        if (fz > 0.01f) { s2Z += fz; n2Z++; }
      }
      // only compute drift when both halves have enough valid samples (>=5)
      if (n1X >= 5 && n2X >= 5) driftX = (float)(s2X/n2X - s1X/n1X);
      if (n1Y >= 5 && n2Y >= 5) driftY = (float)(s2Y/n2Y - s1Y/n1Y);
      if (n1Z >= 5 && n2Z >= 5) driftZ = (float)(s2Z/n2Z - s1Z/n1Z);
    }
    bool freqAlert = (fabsf(driftX) > FREQ_DRIFT_THRESH ||
                      fabsf(driftY) > FREQ_DRIFT_THRESH ||
                      fabsf(driftZ) > FREQ_DRIFT_THRESH);

    int8_t trendDir = (rmsSlope >  TREND_SLOPE_UP)  ?  1 :
                      (rmsSlope <  TREND_SLOPE_DOWN) ? -1 : 0;

    float ttwHours  = 0.0f;
    float currentRms = g_trendBuf[(snapHead + TREND_BUF_SIZE - 1) % TREND_BUF_SIZE].rms;
    if (trendDir == 1 && currentRms < WARNING_RMS) {
      float ratePerHour = rmsSlope * 4.0f * 3600.0f;
      float gap = WARNING_RMS - currentRms;
      if (ratePerHour > 0.001f) {
        ttwHours = gap / ratePerHour;
        if (ttwHours > 9999.0f) ttwHours = 9999.0f;
      }
    }

    g_trendResult.rms_slope      = roundf(rmsSlope  * 100000.0f) / 100000.0f;
    g_trendResult.temp_slope     = roundf(tempSlope * 100000.0f) / 100000.0f;
    g_trendResult.trend_dir      = trendDir;
    g_trendResult.spike_count    = spike_count;
    g_trendResult.freq_drift_x   = roundf(driftX * 1000.0f) / 1000.0f;
    g_trendResult.freq_drift_y   = roundf(driftY * 1000.0f) / 1000.0f;
    g_trendResult.freq_drift_z   = roundf(driftZ * 1000.0f) / 1000.0f;
    g_trendResult.freq_alert     = freqAlert;
    g_trendResult.ttw_hours      = roundf(ttwHours * 10.0f) / 10.0f;
    g_trendResult.window_samples = n;

    Serial.printf("[TREND] n=%u slope=%.5f dir=%+d spikes=%u | "
                  "driftX=%.3f driftY=%.3f driftZ=%.3f alert=%d | "
                  "ttw=%.1fh tempSlope=%.5f\n",
                  n, rmsSlope, trendDir, spike_count,
                  driftX, driftY, driftZ, (int)freqAlert,
                  ttwHours, tempSlope);
  } else {
    // ????????????? -- zero Phase 1 fields ??????????? Phase 2 ???
    g_trendResult.rms_slope      = 0.0f;
    g_trendResult.temp_slope     = 0.0f;
    g_trendResult.trend_dir      = 0;
    g_trendResult.spike_count    = 0;
    g_trendResult.freq_drift_x   = 0.0f;
    g_trendResult.freq_drift_y   = 0.0f;
    g_trendResult.freq_drift_z   = 0.0f;
    g_trendResult.freq_alert     = false;
    g_trendResult.ttw_hours      = 0.0f;
    g_trendResult.window_samples = snapCount;
  }

  // -- Phase 2: Multi-Resolution Slopes ------------------------------------
  // hold mutexAggBufs ??????? g_buf* ???????????? taskAnalytics ?????????????
  if (xSemaphoreTake(mutexAggBufs, pdMS_TO_TICKS(5)) == pdTRUE) {

    // slope ready flags -- consumer knows when data is trustworthy
    g_trendResult.slope_ready_1s  = (g_buf1sCount  >= SLOPE_1S_MIN_SLOTS);
    g_trendResult.slope_ready_10s = (g_buf10sCount >= SLOPE_10S_MIN_SLOTS);
    g_trendResult.slope_ready_60s = (g_buf60sCount >= SLOPE_60S_MIN_SLOTS);

    // slope_1s: 30s window -- compute once buf1s >= 10 slots
    g_trendResult.slope_1s  = g_trendResult.slope_ready_1s
        ? aggLinRegSlope(g_buf1s,  g_buf1sHead,  g_buf1sCount, AGG_BUF_1S_SIZE,  30)
        : 0.0f;

    // slope_10s: 5-min window -- compute once buf10s >= 10 slots (~100 s)
    g_trendResult.slope_10s = g_trendResult.slope_ready_10s
        ? aggLinRegSlope(g_buf10s, g_buf10sHead, g_buf10sCount, AGG_BUF_10S_SIZE, 30)
        : 0.0f;

    // slope_60s: 30-min window -- compute once buf60s >= 20 slots (~20 min)
    g_trendResult.slope_60s = g_trendResult.slope_ready_60s
        ? aggLinRegSlope(g_buf60s, g_buf60sHead, g_buf60sCount, AGG_BUF_60S_SIZE, 30)
        : 0.0f;

    // stddev_1min and max_rms_10min
    {
      uint16_t n60 = (g_buf1sCount < AGG_BUF_1S_SIZE) ? g_buf1sCount : AGG_BUF_1S_SIZE;
      float sumSd = 0.0f;
      for (uint16_t i = 0; i < n60; i++) {
        sumSd += g_buf1s[(g_buf1sHead + AGG_BUF_1S_SIZE - n60 + i) % AGG_BUF_1S_SIZE].stddev_rms;
      }
      g_trendResult.stddev_1min = (n60 > 0) ? (sumSd / n60) : 0.0f;

      float maxRms10m = 0.0f;
      uint16_t n10m = (g_buf10sCount < AGG_BUF_10S_SIZE) ? g_buf10sCount : AGG_BUF_10S_SIZE;
      for (uint16_t i = 0; i < n10m; i++) {
        float mr = g_buf10s[(g_buf10sHead + AGG_BUF_10S_SIZE - n10m + i) % AGG_BUF_10S_SIZE].max_rms;
        if (mr > maxRms10m) maxRms10m = mr;
      }
      g_trendResult.max_rms_10min = maxRms10m;
    }

    xSemaphoreGive(mutexAggBufs);
  }

  // -- EMA snapshot ------------------------------------------------------
  g_trendResult.ema_dir   = g_emaDir;
  g_trendResult.ema_rms   = roundf(g_emaRms   * 1000.0f) / 1000.0f;
  g_trendResult.ema_delta = roundf(g_emaDelta * 100000.0f) / 100000.0f;

  Serial.printf("[TREND-P2] slope_1s=%s%.5f slope_10s=%s%.5f slope_60s=%s%.5f | "
                "ema=%.3f dir=%+d | stddev1m=%.3f maxRms10m=%.2f\n",
                g_trendResult.slope_ready_1s  ? "" : "~",  g_trendResult.slope_1s,
                g_trendResult.slope_ready_10s ? "" : "~",  g_trendResult.slope_10s,
                g_trendResult.slope_ready_60s ? "" : "~",  g_trendResult.slope_60s,
                g_trendResult.ema_rms, g_trendResult.ema_dir,
                g_trendResult.stddev_1min, g_trendResult.max_rms_10min);

  // -- Phase 3: Fault Classification ----------------------------------------
  // Needs freq_ratio from most recent raw sample (same snapshot approach)
  {
    uint16_t lastIdx = (g_trendHead + TREND_BUF_SIZE - 1) % TREND_BUF_SIZE;
    const TrendSample_t* latest = &g_trendBuf[lastIdx];
    // current RPM from g_vibData (Core 1 read with mutex)
    float latestRpm = 0.0f;
    if (xSemaphoreTake(mutexVibData, pdMS_TO_TICKS(5)) == pdTRUE) {
      latestRpm = g_vibData.rpm;
      xSemaphoreGive(mutexVibData);
    }
    // Phase 5: fault classification is now handled inside runDecisionEngine()
    // via classifyFaultProbabilistic() + AEDF, which sets g_trendResult.fault_type
    (void)latestRpm;  // still read for mutex timing; used again below
  }

  // -- Phase 5: Decision Engine (includes probabilistic classification) ------
  {
    float latestRms = g_trendBuf[(g_trendHead + TREND_BUF_SIZE - 1) % TREND_BUF_SIZE].rms;
    MachineState_t latestState = STATE_NORMAL;
    if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(5)) == pdTRUE) {
      latestState = g_systemState.state;
      xSemaphoreGive(mutexSystemState);
    }
    runDecisionEngine(latestRms, latestState);
  }
}

// ============================================================================
// MQTT PUBLISHING (CORE 1)
// ============================================================================

bool publishTelemetry(VibrationData_t* data, MachineState_t state) {
  if (!mqttClient.connected()) return false;

  // ── Shared pre-computes (identical to original) ───────────────────────────
  int alarmCode = (state == STATE_CRITICAL) ? 2 :
                  (state == STATE_WARNING)  ? 1 : 0;
  const char* alarmLevel = (alarmCode == 2) ? "CRITICAL" :
                           (alarmCode == 1) ? "WARNING"  : "NORMAL";

  float normalized  = (data->rms_overall - BASELINE_RMS) /
                      (CRITICAL_RMS - BASELINE_RMS) * 100.0f;
  int healthScore   = (int)max(0.0f, min(100.0f, roundf(100.0f - normalized)));

  // v15.1: ใช้ cf_max (max ของทั้ง 3 แกน) แทน cf_x เพียงแกนเดียว
  // sensor คำนวณจาก raw 16KHz FIFO ภายใน chip:  CF = Peak_acc / RMS_acc
  float crestFactor = (data->cf_max > 0.0f)
                      ? roundf(data->cf_max * 100.0f) / 100.0f
                      : 0.0f;

  // v15.2 Fix 16: Gate bearing_alert ด้วย motor_state == 2 (RUNNING)
  // state != 2 → Kurtosis ไม่ valid (decel impulse / noise floor ต่ำ)
  // Gaussian baseline = 3.0 | early warning > 4.0 | confirmed > 6.0
  const char* bearingAlert;
  if (data->motor_state != 2) {
    bearingAlert = "INVALID_STATE";   // STOPPED/STARTING/STOPPING → ไม่ประเมิน
  } else if (data->kurtosis_max >= 6.0f) {
    bearingAlert = "CONFIRMED";
  } else if (data->kurtosis_max >= 4.0f) {
    bearingAlert = "EARLY_WARNING";
  } else {
    bearingAlert = "NORMAL";
  }
  const char* dominantAxis = (data->kurtosis_dominant_axis == 0) ? "X" :
                             (data->kurtosis_dominant_axis == 1) ? "Y" : "Z";

  // v15.2 Fix 17: Gate freq_ratio และ freq_alert ด้วย RPM_FREQ_GATE
  // rpm < RPM_FREQ_GATE (400) → ratio = 0, ไม่คำนวณ (rot_freq เกือบ 0 → ratio ไม่มีความหมาย)
  float freqX = roundf(data->freq_x * 10.0f) / 10.0f;
  float freqY = roundf(data->freq_y * 10.0f) / 10.0f;
  float freqZ = roundf(data->freq_z * 10.0f) / 10.0f;
  float freqRatioX = 0.0f, freqRatioY = 0.0f, freqRatioZ = 0.0f;
  bool  freqGateOpen = (data->rpm >= (float)RPM_FREQ_GATE);  // true = rpm เพียงพอ
  if (freqGateOpen) {
    float rotFreq  = data->rpm / 60.0f;
    freqRatioX = roundf((freqX / rotFreq) * 100.0f) / 100.0f;
    freqRatioY = roundf((freqY / rotFreq) * 100.0f) / 100.0f;
    freqRatioZ = roundf((freqZ / rotFreq) * 100.0f) / 100.0f;
  }

  calcTrend();   // same call as original
  const char* trendDirStr = (g_trendResult.trend_dir ==  1) ? "UP"   :
                            (g_trendResult.trend_dir == -1) ? "DOWN" : "STABLE";

  // Shared timestamp (built once, used in all three payloads)
  char tsBuf[26] = "not_available";
  if (g_rtcValid) {
    DateTime now = rtc.now();
    snprintf(tsBuf, sizeof(tsBuf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second());
  }

  bool success = false;

  // ──────────────────────────────────────────────────────────────────────────
  // PUBLISH 1 of 3 — /sensor
  // Stage  : raw acquisition
  // Fields : sensor readings + motor context + harmonic feature extraction
  // v15.0 additions: vel_peak_x/y/z, vel_peak, kurtosis_x
  // v15.3 additions: reset_reason, reboot_count
  // Size   : doc 960 B (stack), char buf[1000], JSON ~530 B estimated
  // ──────────────────────────────────────────────────────────────────────────
  {
    StaticJsonDocument<960> s;
    s["plant"]               = PLANT_ID;
    s["machine_id"]          = MACHINE_ID;
    s["sensor_id"]           = SENSOR_ID;
    s["stage"]               = "sensor";
    s["execution_location"]  = "edge";
    s["sensor_status"]       = "ONLINE";

    // v15.7: velocity RMS จาก sensor VRMS register โดยตรง (÷1000)
    s["rms"]   = round(data->rms_overall * 100) / 100.0f;
    s["vx"]    = round(data->rms_x       * 100) / 100.0f;
    s["vy"]    = round(data->rms_y       * 100) / 100.0f;
    s["vz"]    = round(data->rms_z       * 100) / 100.0f;
    // v15.7: peak/vel_peak_x/y/z ลบออก -- ไม่มี peak register บน sensor

    s["temp"]  = round(data->temperature *  10) /  10.0f;
    s["rpm"]   = data->rpm;

    // Harmonic feature extraction
    s["freq_x"]       = freqX;
    s["freq_y"]       = freqY;
    s["freq_z"]       = freqZ;
    s["freq_ratio_x"] = freqRatioX;
    s["freq_ratio_y"] = freqRatioY;
    s["freq_ratio_z"] = freqRatioZ;

    // v15.1: CF ครบ 3 แกน + max
    s["crest_factor"]   = crestFactor;                           // = cf_max
    s["cf_x"]           = round(data->cf_x * 100) / 100.0f;
    s["cf_y"]           = round(data->cf_y * 100) / 100.0f;
    s["cf_z"]           = round(data->cf_z * 100) / 100.0f;

    // v15.1: Kurtosis ครบ 3 แกน + derived fields
    s["kurtosis_x"]     = round(data->kurtosis_x   * 1000) / 1000.0f;
    s["kurtosis_y"]     = round(data->kurtosis_y   * 1000) / 1000.0f;
    s["kurtosis_z"]     = round(data->kurtosis_z   * 1000) / 1000.0f;
    s["kurtosis_max"]   = round(data->kurtosis_max * 1000) / 1000.0f;
    s["kurtosis_axis"]  = dominantAxis;   // "X" / "Y" / "Z"
    s["bearing_alert"]  = bearingAlert;   // "NORMAL" / "EARLY_WARNING" / "CONFIRMED"

    s["motor_state"]           = data->motor_state;
    s["rotation_signal_ok"]    = data->prox;
    s["operating_hours_total"] = data->runtime_hour;

    // v15.3: Reset reason — ช่วยวินิจฉัย unexpected reboot
    s["reset_reason"]  = g_resetReasonStr;   // "POWER_ON" / "BROWNOUT" / "PANIC" etc.
    s["reboot_count"]  = g_rebootCount;      // สะสมทุก boot (NVS persistent)

    s["timestamp"]   = tsBuf;
    s["time_synced"] = g_timeSync.synced;

    char buf[1000];
    size_t szSensor = serializeJson(s, buf, sizeof(buf));
    if (szSensor == 0 || szSensor >= sizeof(buf) - 1) {
      Serial.printf("[WARN] /sensor JSON truncated! sz=%u buf=%u\n",
                    (unsigned)szSensor, (unsigned)sizeof(buf));
    }
    if (mqttClient.publish(g_mqttTopicSensor, buf, false, MQTT_QOS))
      Serial.printf("[MQTT] /sensor %u B\n", (unsigned)szSensor);
    else
      Serial.printf("[MQTT] /sensor FAILED (err=%d)\n", mqttClient.lastError());
  }

  // ──────────────────────────────────────────────────────────────────────────
  // PUBLISH 2 of 3 — /decision
  // Stage  : final alarm + fault + health (human-facing output)
  // Fields : alarm class, health, fault, confidence, action, state transitions
  // Size   : doc 576 B (stack), char buf[600], JSON ~330 B (v14.2: corrected comment; was doc<448>/buf[480] in comment but code already had doc<576>/buf[600])
  // ──────────────────────────────────────────────────────────────────────────
  {
    StaticJsonDocument<576> d;
    d["plant"]               = PLANT_ID;
    d["machine_id"]          = MACHINE_ID;
    d["sensor_id"]           = SENSOR_ID;
    d["stage"]               = "decision";
    d["execution_location"]  = "edge";

    d["alarm_code"]          = alarmCode;
    d["alarm_level"]         = alarmLevel;
    d["alarm_class"]         = alarmClassStr(g_decision.alarm_class);
    d["health_score"]        = healthScore;

    d["fault_type"]          = faultTypeStr(g_decision.fault_type);
    d["fault_score"]         = g_decision.fault_score;
    d["fault_uncertainty"]   = g_decision.fault_uncertainty;
    d["severity_score"]      = g_decision.severity_score;
    d["confidence"]          = g_decision.confidence;

    d["predicted_state"]     = g_decision.predicted_state;
    d["final_state"]         = g_decision.final_state;
    // Patent field: did state damping change the outcome?
    d["state_rate_limited"]      = (g_decision.final_state != g_decision.predicted_state);
    d["state_change_boost_cyc"]  = g_stateChangeCyc;

    if (g_decision.ttw_best_h > 0.0f)
      d["ttw_best_h"]        = g_decision.ttw_best_h;

    d["action_needed"]       = g_decision.action_needed;
    d["reason"]              = g_decision.reason;
    d["pub_interval_s"]      = g_decision.publish_interval_ms / 1000;

    d["timestamp"] = tsBuf;

    // buf[600] > StaticJsonDocument<576> + 24 B safety margin
    // (v14.1 original bug: buf[420] was 28 B SHORTER than old doc<448>; v14.2: comment corrected to reflect actual doc<576>/buf[600])
    char buf[600];
    size_t szDecision = serializeJson(d, buf, sizeof(buf));
    if (szDecision == 0 || szDecision >= sizeof(buf) - 1) {
      Serial.printf("[WARN] /decision JSON truncated! sz=%u buf=%u\n",
                    (unsigned)szDecision, (unsigned)sizeof(buf));
    }
    if (mqttClient.publish(g_mqttTopicDecision, buf, false, MQTT_QOS))
      Serial.printf("[MQTT] /decision %u B\n", (unsigned)szDecision);
    else
      Serial.printf("[MQTT] /decision FAILED (err=%d)\n", mqttClient.lastError());
  }

  // ──────────────────────────────────────────────────────────────────────────
  // PUBLISH 3 of 3 — /vibration  (BACKWARD-COMPATIBLE Grafana payload)
  // ALL original field names kept verbatim.
  // StaticJsonDocument reduced 3200 -> 2048 B; all fields still fit.
  // ──────────────────────────────────────────────────────────────────────────
  {
    StaticJsonDocument<2048> doc;

    doc["plant"]      = PLANT_ID;
    doc["machine_id"] = MACHINE_ID;
    doc["sensor_id"]  = SENSOR_ID;

    doc["rms"]   = round(data->rms_overall * 100) / 100.0f;
    doc["vx"]    = round(data->rms_x       * 100) / 100.0f;
    doc["vy"]    = round(data->rms_y       * 100) / 100.0f;
    doc["vz"]    = round(data->rms_z       * 100) / 100.0f;
    // v15.7: peak/vel_peak_x/y/z ลบออก
    doc["temp"]  = round(data->temperature *  10) /  10.0f;
    doc["rpm"]   = data->rpm;
    doc["freq_x"] = freqX; doc["freq_y"] = freqY; doc["freq_z"] = freqZ;
    doc["freq_ratio_x"] = freqRatioX;
    doc["freq_ratio_y"] = freqRatioY;
    doc["freq_ratio_z"] = freqRatioZ;
    doc["state"]               = data->motor_state;
    doc["operating_hours_total"] = data->runtime_hour;
    doc["rotation_signal_ok"]  = data->prox;
    doc["alarm_code"]          = alarmCode;
    doc["alarm_level"]         = alarmLevel;
    doc["health_score"]        = healthScore;
    // v15.1: CF และ Kurtosis ครบ 3 แกน + derived
    doc["crest_factor"]     = crestFactor;                           // = cf_max
    doc["cf_x"]             = round(data->cf_x * 100) / 100.0f;
    doc["cf_y"]             = round(data->cf_y * 100) / 100.0f;
    doc["cf_z"]             = round(data->cf_z * 100) / 100.0f;
    doc["kurtosis_x"]       = round(data->kurtosis_x   * 1000) / 1000.0f;
    doc["kurtosis_y"]       = round(data->kurtosis_y   * 1000) / 1000.0f;
    doc["kurtosis_z"]       = round(data->kurtosis_z   * 1000) / 1000.0f;
    doc["kurtosis_max"]     = round(data->kurtosis_max * 1000) / 1000.0f;
    doc["kurtosis_axis"]    = dominantAxis;   // "X" / "Y" / "Z"
    doc["bearing_alert"]    = bearingAlert;   // "NORMAL" / "EARLY_WARNING" / "CONFIRMED"
    doc["sensor_status"]    = "ONLINE";

    doc["rms_slope"]      = g_trendResult.rms_slope;
    doc["temp_slope"]     = g_trendResult.temp_slope;
    doc["trend_dir"]      = trendDirStr;
    doc["spike_count"]    = g_trendResult.spike_count;
    // v15.2 Fix 17: suppress freq fields เมื่อ RPM < RPM_FREQ_GATE
    doc["freq_drift_x"]   = freqGateOpen ? g_trendResult.freq_drift_x : 0.0f;
    doc["freq_drift_y"]   = freqGateOpen ? g_trendResult.freq_drift_y : 0.0f;
    doc["freq_drift_z"]   = freqGateOpen ? g_trendResult.freq_drift_z : 0.0f;
    doc["freq_alert"]     = freqGateOpen && g_trendResult.freq_alert;
    doc["freq_gate_open"] = freqGateOpen;  // debug: บอกว่า gate เปิด/ปิด
    if (g_trendResult.ttw_hours > 0.0f)
      doc["ttw_hours"]    = g_trendResult.ttw_hours;
    doc["trend_window_s"] = (g_trendResult.window_samples * 250) / 1000;

    doc["slope_1s"]          = roundf(g_trendResult.slope_1s  * 100000.0f) / 100000.0f;
    doc["slope_10s"]         = roundf(g_trendResult.slope_10s * 100000.0f) / 100000.0f;
    doc["slope_60s"]         = roundf(g_trendResult.slope_60s * 100000.0f) / 100000.0f;
    doc["slope_ready_1s"]    = g_trendResult.slope_ready_1s;
    doc["slope_ready_10s"]   = g_trendResult.slope_ready_10s;
    doc["slope_ready_60s"]   = g_trendResult.slope_ready_60s;
    doc["ema_dir"]           = g_trendResult.ema_dir;
    doc["ema_rms"]           = g_trendResult.ema_rms;
    doc["stddev_1min"]       = roundf(g_trendResult.stddev_1min   * 1000.0f) / 1000.0f;
    doc["max_rms_10min"]     = roundf(g_trendResult.max_rms_10min * 100.0f)  / 100.0f;
    doc["agg_buf_1s"]        = g_buf1sCount;
    doc["agg_buf_10s"]       = g_buf10sCount;
    doc["agg_buf_60s"]       = g_buf60sCount;

    doc["fault_type"]        = faultTypeStr(g_decision.fault_type);
    doc["fault_score"]       = g_decision.fault_score;
    doc["fault_uncertainty"] = g_decision.fault_uncertainty;
    doc["severity_score"]    = g_decision.severity_score;
    doc["confidence"]        = g_decision.confidence;
    doc["predicted_state"]   = g_decision.predicted_state;
    doc["final_state"]       = g_decision.final_state;
    doc["alarm_class"]       = alarmClassStr(g_decision.alarm_class);
    if (g_decision.ttw_best_h > 0.0f) {
      doc["ttw_best_h"]    = g_decision.ttw_best_h;
      doc["ttw_short_h"]   = g_decision.ttw_short_h;
      doc["ttw_medium_h"]  = g_decision.ttw_medium_h;
      if (g_decision.ttw_long_h > 0.0f)
        doc["ttw_long_h"]  = g_decision.ttw_long_h;
      doc["ttw_w_s"]       = g_decision.ttw_w_s;
      doc["ttw_w_m"]       = g_decision.ttw_w_m;
      doc["ttw_w_l"]       = g_decision.ttw_w_l;
    }
    doc["action_needed"]   = g_decision.action_needed;
    doc["reason"]          = g_decision.reason;
    doc["pub_interval_s"]  = g_decision.publish_interval_ms / 1000;

    doc["timestamp"]   = tsBuf;
    doc["time_synced"] = g_timeSync.synced;
    if (g_timeSync.synced)
      doc["sync_age_s"] = (millis() - g_timeSync.lastSyncMillis) / 1000;

    char   jsonBuffer[2048];
    size_t jsonSize = serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));
    success = mqttClient.publish(g_mqttTopic, jsonBuffer, false, MQTT_QOS);

    if (success) {
      Serial.printf("[MQTT] /vibration %d B | %s rms=%.2f(vx=%.2f vy=%.2f vz=%.2f) rpm=%.1f "
                    "state=%d | health=%d%% | frx=%.2f fry=%.2f frz=%.2f | "
                    "cf=%.2f kurt_max=%.3f(%s) bear=%s | "
                    "fault=%s sc=%.2f sev=%u conf=%u%% pred=%u fin=%u ttw=%.1fh\n",
                    jsonSize, alarmLevel,
                    data->rms_overall, data->rms_x, data->rms_y, data->rms_z, data->rpm,
                    data->motor_state,
                    healthScore, freqRatioX, freqRatioY, freqRatioZ,
                    crestFactor, data->kurtosis_max, dominantAxis, bearingAlert,
                    faultTypeStr(g_decision.fault_type),
                    g_decision.fault_score, g_decision.severity_score,
                    g_decision.confidence, g_decision.predicted_state,
                    g_decision.final_state, g_decision.ttw_best_h);
    } else {
      Serial.printf("[MQTT] Publish FAILED (err=%d)\n", mqttClient.lastError());
    }
  }

  return success;
}

// ============================================================================
// TASK: ANALYTICS (CORE 1, Priority 3) -- Phase 2
// ============================================================================
//
// ???????:
//   1. ??????? 1,000 ms (1 Hz) — FreeRTOS tick unchanged
//   2. ???? g_trendBuf snapshot (Core 0 ????? / Core 1 ???? -- atomic float)
//   3. ????? AggSample_t ??? 4 raw samples ?????? -> push g_buf1s
//      (Claim 2: flush triggered by acc1sMs >= g_slotDur1sMs, not fixed counter)
//   4. Cascade -> g_buf10s ??? ~10s real time
//   5. Cascade -> g_buf60s ??? ~60s real time
//   6. ?????? EMA + g_emaDir
//   7. Publish /analytics MQTT topic ??? 60 ?????? (??? MQTT connected)
//
// Stack: 5120 bytes (?????? linreg loop + float arrays ?? stack)
// Priority: 3 (??????? display -- ??? block network/modbus)
// ============================================================================

// ── Patent Claim 2 helper ────────────────────────────────────────────────────
// Compute RPM-adaptive slot duration for buf1s.
// Returns ms per slot clamped to [SLOT_DUR_MIN_MS, SLOT_DUR_MAX_MS].
// When rpm < MIN_RPM_VALID (motor stopped/starting) returns fixed 1000ms
// to avoid division near zero and spurious wide slots during transients.
static uint32_t computeSlotDurMs(float rpm) {
  if (rpm < (float)MIN_RPM_VALID) return 1000UL;  // motor not running -- keep default
  // target: SLOT_REVS_TARGET full revolutions per slot
  float ms = ((float)SLOT_REVS_TARGET * 60000.0f) / rpm;
  uint32_t dur = (uint32_t)ms;
  if (dur < SLOT_DUR_MIN_MS) dur = SLOT_DUR_MIN_MS;
  if (dur > SLOT_DUR_MAX_MS) dur = SLOT_DUR_MAX_MS;
  return dur;
}

void taskAnalytics(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(1000);  // 1 Hz — unchanged

  // ── Patent Claim 2: millis-based accumulators replace fixed counters ─────
  // acc*Ms and lastHead/firstRun are now globals (g_accMs_*, g_anaLastHead,
  // g_anaFirstRun) so taskButtonHandler can atomically reset them during
  // maintenance while taskAnalytics is suspended. Local aliases for readability.
  uint32_t& acc1sMs  = (uint32_t&)g_accMs_1s;
  uint32_t& acc10sMs = (uint32_t&)g_accMs_10s;
  uint32_t& acc60sMs = (uint32_t&)g_accMs_60s;

  // lastHead: ?? head ?????????????? -> ?? samples ???????????????? aggregate
  uint16_t& lastHead = (uint16_t&)g_anaLastHead;
  bool&     firstRun = (bool&)g_anaFirstRun;

  // Publish /analytics counter
  uint8_t&  analyticsPublishCnt = (uint8_t&)g_anaPublishCnt;

  Serial.println("[CORE 1] Analytics task started (Phase 2+Claim2)");

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);

    // ── Patent Claim 2: update slot duration from current RPM ────────────
    // Read RPM (written atomically by Core 0 processRPM).
    // Update g_slotDur1sMs every tick so slot width adapts to operating speed.
    {
      float latestRpm = 0.0f;
      if (xSemaphoreTake(mutexVibData, pdMS_TO_TICKS(3)) == pdTRUE) {
        latestRpm = g_vibData.rpm;
        xSemaphoreGive(mutexVibData);
      }
      g_slotDur1sMs = computeSlotDurMs(latestRpm);
    }

    // Accumulate elapsed ms this tick (task period = 1000ms fixed)
    const uint32_t tickMs = 1000UL;
    acc1sMs  += tickMs;
    acc10sMs += tickMs;
    acc60sMs += tickMs;

    // -- Snapshot raw buffer state (Core 0 writes, float-atomic) --
    uint16_t snapHead  = g_trendHead;   // volatile read (atomic on Xtensa)
    uint16_t snapCount = g_trendCount;

    if (snapCount == 0) continue;

    // -- ?????????? new samples ??????? last read --
    // newSamples = ????? samples ??? Core 0 push ?????? 1 ?????? (??????? 4 +/-1)
    uint16_t newSamples;
    if (firstRun) {
      // ??????: ??? min(snapCount, 4) -- ???????????? 1 ??????
      newSamples = (snapCount < 4) ? snapCount : 4;
      lastHead   = (snapHead + TREND_BUF_SIZE - newSamples) % TREND_BUF_SIZE;
      firstRun   = false;
    } else {
      // ????: head ??????????? step ??? lastHead
      newSamples = (snapHead - lastHead + TREND_BUF_SIZE) % TREND_BUF_SIZE;
      if (newSamples == 0) {
        // Core 0 ?????? push ????? 1 ?????? (sensor offline?) -- ????
        analyticsPublishCnt++;
        goto analytics_publish;
      }
      // ???????????? 4 (buffer ??? overrun ??? wakeup ???)
      if (newSamples > 8) newSamples = 8;
    }

    // ── Patent Claim 2: rolling accumulator for variable-width slot ──────
    // Instead of building one a1s per tick and flushing immediately,
    // we accumulate raw samples into a_slot across multiple ticks.
    // When acc1sMs >= g_slotDur1sMs (the RPM-derived threshold), we
    // finalise and push the accumulated slot, then reset for the next one.
    // This ensures each buf1s slot always covers SLOT_REVS_TARGET revolutions
    // regardless of how many ticks fit within that time window.
    {
      // Per-slot running accumulators (persist across ticks between flushes)
      // V14.4: promoted to globals (g_sl_*) for maintenance reset support.
      float&   sl_sumRms   = (float&)g_sl_sumRms;
      float&   sl_sumSqRms = (float&)g_sl_sumSqRms;
      float&   sl_maxRms   = (float&)g_sl_maxRms;
      float&   sl_sumTemp  = (float&)g_sl_sumTemp;
      float&   sl_maxTemp  = (float&)g_sl_maxTemp;
      float&   sl_sumPeak  = (float&)g_sl_sumPeak;
      float&   sl_maxPeak  = (float&)g_sl_maxPeak;
      float&   sl_sumFrx   = (float&)g_sl_sumFrx;
      float&   sl_sumFry   = (float&)g_sl_sumFry;
      float&   sl_sumFrz   = (float&)g_sl_sumFrz;
      uint8_t& sl_spikes   = (uint8_t&)g_sl_spikes;
      uint8_t& sl_n        = (uint8_t&)g_sl_n;

      // Append this tick's samples into the running slot accumulator
      for (uint16_t i = 0; i < newSamples; i++) {
        uint16_t idx = (lastHead + i) % TREND_BUF_SIZE;
        const TrendSample_t* s = &g_trendBuf[idx];

        float rms     = s->rms;
        float rms_max = s->rms_max;  // v15.7: impulse peak (was 'peak' = rms_overall, bug)
        float temp    = s->temp;

        sl_sumRms   += rms;
        sl_sumSqRms += rms * rms;
        if (rms  > sl_maxRms)  sl_maxRms  = rms;
        sl_sumTemp  += temp;
        if (temp > sl_maxTemp) sl_maxTemp = temp;
        sl_sumPeak  += rms_max;                          // mean_rms_max = mean impulse envelope
        if (rms_max > sl_maxPeak) sl_maxPeak = rms_max; // max_rms_max  = peak impulse
        sl_sumFrx   += s->freq_ratio_x;
        sl_sumFry   += s->freq_ratio_y;
        sl_sumFrz   += s->freq_ratio_z;
        if (rms_max > WARNING_RMS * SPIKE_RMS_FACTOR) sl_spikes++;  // impulse spike count
        if (sl_n < 255) sl_n++;  // guard uint8_t overflow (max 255 raw samples/slot)
      }

      // Advance lastHead — consumed up to snapHead
      lastHead = snapHead;

      if (sl_n == 0) goto analytics_ema;

      // ── Flush when slot duration threshold reached ──────────────────────
      if (acc1sMs >= g_slotDur1sMs) {
        acc1sMs = 0;

        float inv      = 1.0f / sl_n;
        float meanRms  = sl_sumRms * inv;
        float variance = (sl_sumSqRms * inv) - (meanRms * meanRms);
        float stddev   = (variance > 0.0f) ? sqrtf(variance) : 0.0f;

        AggSample_t a1s = {
          .mean_rms     = meanRms,
          .max_rms      = sl_maxRms,
          .stddev_rms   = stddev,
          .mean_temp    = sl_sumTemp * inv,
          .max_temp     = sl_maxTemp,
          .mean_rms_max = sl_sumPeak * inv,  // mean impulse envelope
          .max_rms_max  = sl_maxPeak,        // peak impulse in slot
          .mean_frx     = sl_sumFrx * inv,
          .mean_fry     = sl_sumFry * inv,
          .mean_frz     = sl_sumFrz * inv,
          .spike_count  = sl_spikes,
          .n_samples    = sl_n
        };

        // Reset slot accumulators for the next slot
        sl_sumRms = sl_sumSqRms = sl_maxRms  = 0.0f;
        sl_sumTemp = sl_maxTemp = 0.0f;
        sl_sumPeak = sl_maxPeak = 0.0f;
        sl_sumFrx  = sl_sumFry  = sl_sumFrz  = 0.0f;
        sl_spikes  = 0;
        sl_n       = 0;

        // -- Push 1s aggregate (hold mutex) --
        if (xSemaphoreTake(mutexAggBufs, pdMS_TO_TICKS(5)) == pdTRUE) {
          pushAggBuf(g_buf1s, &g_buf1sHead, &g_buf1sCount, AGG_BUF_1S_SIZE, &a1s);
          xSemaphoreGive(mutexAggBufs);
        }

        // -- Cascade -> 10s (target: 10 × slotDur1s real ms) ─────────────
        // acc10sMs threshold = 10 × g_slotDur1sMs, so the 10s slot always
        // covers exactly 10 × SLOT_REVS_TARGET revolutions regardless of RPM.
        if (acc10sMs >= 10UL * g_slotDur1sMs) {
          acc10sMs = 0;

          // Average ??? g_buf1s 10 slots ??????
          if (xSemaphoreTake(mutexAggBufs, pdMS_TO_TICKS(5)) == pdTRUE) {
            uint16_t take = (g_buf1sCount < 10) ? g_buf1sCount : 10;

            if (take > 0) {
              AggSample_t a10s = { 0 };
              for (uint16_t i = 0; i < take; i++) {
                uint16_t idx = (g_buf1sHead + AGG_BUF_1S_SIZE - take + i) % AGG_BUF_1S_SIZE;
                const AggSample_t* q = &g_buf1s[idx];
                a10s.mean_rms    += q->mean_rms;
                if (q->max_rms  > a10s.max_rms)  a10s.max_rms  = q->max_rms;
                a10s.stddev_rms  += q->stddev_rms;
                a10s.mean_temp   += q->mean_temp;
                if (q->max_temp > a10s.max_temp) a10s.max_temp = q->max_temp;
                a10s.mean_rms_max   += q->mean_rms_max;
                if (q->max_rms_max > a10s.max_rms_max) a10s.max_rms_max = q->max_rms_max;
                a10s.mean_frx    += q->mean_frx;
                a10s.mean_fry    += q->mean_fry;
                a10s.mean_frz    += q->mean_frz;
                a10s.spike_count += q->spike_count;
                a10s.n_samples   += q->n_samples;
              }
              float inv10 = 1.0f / take;
              a10s.mean_rms   *= inv10;
              a10s.stddev_rms *= inv10;
              a10s.mean_temp  *= inv10;
              a10s.mean_rms_max  *= inv10;
              a10s.mean_frx   *= inv10;
              a10s.mean_fry   *= inv10;
              a10s.mean_frz   *= inv10;
              pushAggBuf(g_buf10s, &g_buf10sHead, &g_buf10sCount, AGG_BUF_10S_SIZE, &a10s);
            }
            xSemaphoreGive(mutexAggBufs);
          }

          // -- Cascade -> 60s (target: 60 × slotDur1s real ms) ─────────
          // acc60sMs threshold = 60 × g_slotDur1sMs ≈ 60 s wall-clock
          // (adapts proportionally: same harmonic coverage at every speed)
          if (acc60sMs >= 60UL * g_slotDur1sMs) {
            acc60sMs = 0;

            if (xSemaphoreTake(mutexAggBufs, pdMS_TO_TICKS(5)) == pdTRUE) {
              uint16_t take6 = (g_buf10sCount < 6) ? g_buf10sCount : 6;

              if (take6 > 0) {
                AggSample_t a60s = { 0 };
                for (uint16_t i = 0; i < take6; i++) {
                  uint16_t idx = (g_buf10sHead + AGG_BUF_10S_SIZE - take6 + i) % AGG_BUF_10S_SIZE;
                  const AggSample_t* q = &g_buf10s[idx];
                  a60s.mean_rms    += q->mean_rms;
                  if (q->max_rms  > a60s.max_rms)  a60s.max_rms  = q->max_rms;
                  a60s.stddev_rms  += q->stddev_rms;
                  a60s.mean_temp   += q->mean_temp;
                  if (q->max_temp > a60s.max_temp) a60s.max_temp = q->max_temp;
                  a60s.mean_rms_max   += q->mean_rms_max;
                  if (q->max_rms_max > a60s.max_rms_max) a60s.max_rms_max = q->max_rms_max;
                  a60s.mean_frx    += q->mean_frx;
                  a60s.mean_fry    += q->mean_fry;
                  a60s.mean_frz    += q->mean_frz;
                  a60s.spike_count += q->spike_count;
                  a60s.n_samples   += q->n_samples;
                }
                float inv6 = 1.0f / take6;
                a60s.mean_rms   *= inv6;
                a60s.stddev_rms *= inv6;
                a60s.mean_temp  *= inv6;
                a60s.mean_rms_max  *= inv6;
                a60s.mean_frx   *= inv6;
                a60s.mean_fry   *= inv6;
                a60s.mean_frz   *= inv6;
                pushAggBuf(g_buf60s, &g_buf60sHead, &g_buf60sCount, AGG_BUF_60S_SIZE, &a60s);

                Serial.printf("[ANALYTICS] 60s flush -> buf1s=%u buf10s=%u buf60s=%u slot_ms=%lu\n",
                              g_buf1sCount, g_buf10sCount, g_buf60sCount, g_slotDur1sMs);
              }

              // -- Phase 5: Update slope variance trackers (OSG + FVRI inputs) --
              // Compute rolling variance of mean_rms for each buffer tier.
              // Called inside mutexAggBufs critical section -- safe to access buffers.
              // Use last 20 slots (coverage: 20×slotDur1s, 200×slotDur1s, 1200×slotDur1s).
              if (g_buf1sCount  >= 4)
                g_slopeVar_1s  = computeRmsVariance(g_buf1s,  g_buf1sHead,  g_buf1sCount,  AGG_BUF_1S_SIZE,  20);
              if (g_buf10sCount >= 4)
                g_slopeVar_10s = computeRmsVariance(g_buf10s, g_buf10sHead, g_buf10sCount, AGG_BUF_10S_SIZE, 20);
              if (g_buf60sCount >= 4)
                g_slopeVar_60s = computeRmsVariance(g_buf60s, g_buf60sHead, g_buf60sCount, AGG_BUF_60S_SIZE, 20);

              xSemaphoreGive(mutexAggBufs);
            }
          }
        }
      }
      // If acc1sMs < g_slotDur1sMs: samples already appended to sl_* accumulators above;
      // nothing else to do this tick — wait for the slot threshold to be reached.
    }

analytics_ema:
    // -- EMA update (??? 1 ??????) ------------------------------------------
    // ?????? rms ????????? g_trendBuf (1 sample ??????????? EMA ???????)
    {
      uint16_t lastIdx = (snapHead + TREND_BUF_SIZE - 1) % TREND_BUF_SIZE;
      float latestRms  = g_trendBuf[lastIdx].rms;

      float prevEma = g_emaRms;
      g_emaRms      = EMA_ALPHA * latestRms + (1.0f - EMA_ALPHA) * prevEma;
      g_emaDelta    = g_emaRms - g_emaPrevRms;
      g_emaPrevRms  = g_emaRms;

      g_emaDir = (g_emaDelta >  EMA_DIR_THRESHOLD) ?  1 :
                 (g_emaDelta < -EMA_DIR_THRESHOLD) ? -1 : 0;
    }

analytics_publish:
    analyticsPublishCnt++;
    if (analyticsPublishCnt < 60) continue;
    analyticsPublishCnt = 0;

    if (!mqttClient.connected()) continue;
    if (g_buf1sCount < 4)        continue;

    // Shared timestamp for this publish round (all 7 topics use same value)
    char tsA[26] = "not_available";
    if (g_rtcValid) {
      DateTime nowTs = rtc.now();
      snprintf(tsA, sizeof(tsA), "%04d-%02d-%02dT%02d:%02d:%02dZ",
               nowTs.year(), nowTs.month(), nowTs.day(),
               nowTs.hour(), nowTs.minute(), nowTs.second());
    }

    // ──────────────────────────────────────────────────────────────────────
    // PUB-A  /trend   (replaces /analytics, same 60-s cadence)
    // Stage : multi-resolution buffer + slope + EMA + spike + variance
    // Doc   : 640 B stack
    // ──────────────────────────────────────────────────────────────────────
    {
      StaticJsonDocument<640> t;
      t["plant"]               = PLANT_ID;
      t["machine_id"]          = MACHINE_ID;
      t["sensor_id"]           = SENSOR_ID;
      t["stage"]               = "trend";
      t["execution_location"]  = "edge";

      t["buf_1s"]    = g_buf1sCount;
      t["buf_10s"]   = g_buf10sCount;
      t["buf_60s"]   = g_buf60sCount;
      t["ready_1s"]  = g_trendResult.slope_ready_1s;
      t["ready_10s"] = g_trendResult.slope_ready_10s;
      t["ready_60s"] = g_trendResult.slope_ready_60s;

      if (xSemaphoreTake(mutexAggBufs, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (g_trendResult.slope_ready_1s)
          t["slope_1s"]  = roundf(aggLinRegSlope(g_buf1s,  g_buf1sHead,  g_buf1sCount,
                                                 AGG_BUF_1S_SIZE,  30) * 100000.0f) / 100000.0f;
        if (g_trendResult.slope_ready_10s)
          t["slope_10s"] = roundf(aggLinRegSlope(g_buf10s, g_buf10sHead, g_buf10sCount,
                                                 AGG_BUF_10S_SIZE, 30) * 100000.0f) / 100000.0f;
        if (g_trendResult.slope_ready_60s)
          t["slope_60s"] = roundf(aggLinRegSlope(g_buf60s, g_buf60sHead, g_buf60sCount,
                                                 AGG_BUF_60S_SIZE, 30) * 100000.0f) / 100000.0f;

        float maxRms10m = 0.0f;
        uint16_t n10m = (g_buf10sCount < AGG_BUF_10S_SIZE) ? g_buf10sCount : AGG_BUF_10S_SIZE;
        for (uint16_t i = 0; i < n10m; i++)
          maxRms10m = max(maxRms10m,
            g_buf10s[(g_buf10sHead + AGG_BUF_10S_SIZE - n10m + i) % AGG_BUF_10S_SIZE].max_rms);
        t["max_rms_10min"] = roundf(maxRms10m * 100.0f) / 100.0f;

        float maxRms60m = 0.0f;
        uint16_t n60m = (g_buf60sCount < AGG_BUF_60S_SIZE) ? g_buf60sCount : AGG_BUF_60S_SIZE;
        for (uint16_t i = 0; i < n60m; i++)
          maxRms60m = max(maxRms60m,
            g_buf60s[(g_buf60sHead + AGG_BUF_60S_SIZE - n60m + i) % AGG_BUF_60S_SIZE].max_rms);
        if (n60m > 0) t["max_rms_60min"] = roundf(maxRms60m * 100.0f) / 100.0f;

        float sumSd = 0.0f;
        uint16_t n60s = (g_buf1sCount < AGG_BUF_1S_SIZE) ? g_buf1sCount : AGG_BUF_1S_SIZE;
        for (uint16_t i = 0; i < n60s; i++)
          sumSd += g_buf1s[(g_buf1sHead + AGG_BUF_1S_SIZE - n60s + i) % AGG_BUF_1S_SIZE].stddev_rms;
        t["stddev_1min"] = (n60s > 0) ? roundf((sumSd / n60s) * 1000.0f) / 1000.0f : 0.0f;

        // OSG + FVRI inputs — patent-relevant
        t["slope_var_1s"]  = (float)g_slopeVar_1s;
        t["slope_var_10s"] = (float)g_slopeVar_10s;
        t["slope_var_60s"] = (float)g_slopeVar_60s;

        xSemaphoreGive(mutexAggBufs);
      }

      t["ema_rms"]   = roundf(g_emaRms   * 1000.0f)  / 1000.0f;
      t["ema_dir"]   = g_emaDir;
      t["ema_delta"] = roundf(g_emaDelta * 100000.0f) / 100000.0f;
      t["spike_count"]  = g_trendResult.spike_count;
      // v15.2 Fix 17: suppress freq fields เมื่อ RPM < RPM_FREQ_GATE
      // คำนวณ gate จาก g_vibData.rpm โดยตรง (taskAnalytics คนละ scope กับ publishTelemetry)
      {
        bool trendFreqGate = (g_vibData.rpm >= (float)RPM_FREQ_GATE);
        t["freq_alert"]   = trendFreqGate && g_trendResult.freq_alert;
        t["freq_drift_x"] = trendFreqGate ? g_trendResult.freq_drift_x : 0.0f;
        t["freq_drift_y"] = trendFreqGate ? g_trendResult.freq_drift_y : 0.0f;
        t["freq_drift_z"] = trendFreqGate ? g_trendResult.freq_drift_z : 0.0f;
      }
      // Patent Claim 2: expose adaptive slot duration for external verification
      t["slot_dur_ms"]       = (uint32_t)g_slotDur1sMs;
      t["slot_revs_target"]  = SLOT_REVS_TARGET;
      t["timestamp"] = tsA;

      // buf[1024] >> StaticJsonDocument<640>  (v14.3: was buf[700])
      char buf[1024];
      size_t sz = serializeJson(t, buf, sizeof(buf));
      if (sz == 0 || sz >= sizeof(buf) - 1)
        Serial.printf("[WARN] /trend JSON truncated! sz=%u buf=%u\n",
                      (unsigned)sz, (unsigned)sizeof(buf));
      if (mqttClient.publish(g_mqttTopicTrend, buf, false, MQTT_QOS))
        Serial.printf("[TREND] /trend %u B\n", (unsigned)sz);
      else
        Serial.printf("[TREND] FAILED -> %s\n", g_mqttTopicTrend);
    }

    // ──────────────────────────────────────────────────────────────────────
    // PUB-B  /fusion/raw
    // Stage : pre-AEDF Gaussian scores + post-AEDF accumulated evidence
    // Note  : g_rawScores[] = pre-AEDF (Patch A); g_faultEvidence[] = post-AEDF
    // Doc   : 768 B stack (v14.3: was 640; actual JSON ~480 B → needs ≥544 B doc)
    // Size  : doc 768 B, char buf[1024] (v14.3: was buf[700]; uniform 1024)
    // ──────────────────────────────────────────────────────────────────────
    {
      StaticJsonDocument<768> r;
      r["plant"]               = PLANT_ID;
      r["machine_id"]          = MACHINE_ID;
      r["sensor_id"]           = SENSOR_ID;
      r["stage"]               = "fusion_raw";
      r["execution_location"]  = "edge";
      r["top_fault"]           = faultTypeStr(g_decision.fault_type);
      r["top_score"]           = g_decision.fault_score;
      r["fault_uncertainty"]   = g_decision.fault_uncertainty;

      // Patch A: pre-AEDF raw Gaussian scores per fault hypothesis
      JsonObject rs = r.createNestedObject("raw_scores");
      rs["NORMAL"]       = roundf(g_rawScores[FAULT_NORMAL]       * 1000.0f) / 1000.0f;
      rs["IMBALANCE"]    = roundf(g_rawScores[FAULT_IMBALANCE]    * 1000.0f) / 1000.0f;
      rs["MISALIGNMENT"] = roundf(g_rawScores[FAULT_MISALIGNMENT] * 1000.0f) / 1000.0f;
      rs["LOOSENESS"]    = roundf(g_rawScores[FAULT_LOOSENESS]    * 1000.0f) / 1000.0f;
      rs["BEARING"]      = roundf(g_rawScores[FAULT_BEARING]      * 1000.0f) / 1000.0f;
      rs["RESONANCE"]    = roundf(g_rawScores[FAULT_RESONANCE]    * 1000.0f) / 1000.0f;
      rs["UNKNOWN"]      = roundf(g_rawScores[FAULT_UNKNOWN]      * 1000.0f) / 1000.0f;

      JsonObject ev = r.createNestedObject("aedf_evidence");
      ev["NORMAL"]       = roundf(g_faultEvidence[FAULT_NORMAL]       * 1000.0f) / 1000.0f;
      ev["IMBALANCE"]    = roundf(g_faultEvidence[FAULT_IMBALANCE]    * 1000.0f) / 1000.0f;
      ev["MISALIGNMENT"] = roundf(g_faultEvidence[FAULT_MISALIGNMENT] * 1000.0f) / 1000.0f;
      ev["LOOSENESS"]    = roundf(g_faultEvidence[FAULT_LOOSENESS]    * 1000.0f) / 1000.0f;
      ev["BEARING"]      = roundf(g_faultEvidence[FAULT_BEARING]      * 1000.0f) / 1000.0f;
      ev["RESONANCE"]    = roundf(g_faultEvidence[FAULT_RESONANCE]    * 1000.0f) / 1000.0f;
      ev["UNKNOWN"]      = roundf(g_faultEvidence[FAULT_UNKNOWN]      * 1000.0f) / 1000.0f;

      r["timestamp"] = tsA;

      // buf[1024] >> StaticJsonDocument<768>  (v14.3: was buf[700])
      // Actual JSON ~480 B → 544+ B headroom against future field additions
      char buf[1024];
      size_t sz = serializeJson(r, buf, sizeof(buf));
      if (sz == 0 || sz >= sizeof(buf) - 1) {
        Serial.printf("[WARN] /fusion/raw JSON truncated! sz=%u buf=%u\n",
                      (unsigned)sz, (unsigned)sizeof(buf));
      }
      if (mqttClient.publish(g_mqttTopicFusionRaw, buf, false, MQTT_QOS))
        Serial.printf("[FUSION/RAW] %u B\n", (unsigned)sz);
      else
        Serial.printf("[FUSION/RAW] FAILED -> %s\n", g_mqttTopicFusionRaw);
    }

    // ──────────────────────────────────────────────────────────────────────
    // PUB-C  /fusion/aedf
    // Stage : AEDF decay configuration + evidence snapshot
    // Doc   : 768 B stack (v14.3: was 384 — actual JSON ~486 B → doc<384>
    //         silently dropped fields BEFORE serializeJson() was even called!)
    // Size  : doc 768 B, char buf[1024] (v14.3: was buf[450] still too small)
    // ──────────────────────────────────────────────────────────────────────
    {
      StaticJsonDocument<768> a;
      a["plant"]               = PLANT_ID;
      a["machine_id"]          = MACHINE_ID;
      a["stage"]               = "fusion_aedf";
      a["execution_location"]  = "edge";

      JsonObject cfg = a.createNestedObject("config");
      cfg["blend_alpha"]       = AEDF_BLEND;
      cfg["contradict_margin"] = AEDF_CONTRADICT;
      cfg["reset_multiplier"]  = AEDF_RESET_MUL;
      cfg["persist_threshold"] = AEDF_PERSIST_THRESH;

      JsonObject tau = a.createNestedObject("decay_tau_s");
      tau["NORMAL"]       = AEDF_TAU_NORMAL      * 30.0f;
      tau["IMBALANCE"]    = AEDF_TAU_IMBALANCE   * 30.0f;
      tau["MISALIGNMENT"] = AEDF_TAU_MISALIGN    * 30.0f;
      tau["LOOSENESS"]    = AEDF_TAU_LOOSENESS   * 30.0f;
      tau["BEARING"]      = AEDF_TAU_BEARING     * 30.0f;
      tau["RESONANCE"]    = AEDF_TAU_RESONANCE   * 30.0f;
      tau["UNKNOWN"]      = AEDF_TAU_UNKNOWN     * 30.0f;

      JsonObject ev2 = a.createNestedObject("evidence_now");
      ev2["NORMAL"]       = roundf(g_faultEvidence[FAULT_NORMAL]       * 1000.0f) / 1000.0f;
      ev2["IMBALANCE"]    = roundf(g_faultEvidence[FAULT_IMBALANCE]    * 1000.0f) / 1000.0f;
      ev2["MISALIGNMENT"] = roundf(g_faultEvidence[FAULT_MISALIGNMENT] * 1000.0f) / 1000.0f;
      ev2["LOOSENESS"]    = roundf(g_faultEvidence[FAULT_LOOSENESS]    * 1000.0f) / 1000.0f;
      ev2["BEARING"]      = roundf(g_faultEvidence[FAULT_BEARING]      * 1000.0f) / 1000.0f;
      ev2["RESONANCE"]    = roundf(g_faultEvidence[FAULT_RESONANCE]    * 1000.0f) / 1000.0f;
      ev2["UNKNOWN"]      = roundf(g_faultEvidence[FAULT_UNKNOWN]      * 1000.0f) / 1000.0f;

      a["timestamp"] = tsA;

      // buf[1024] >> StaticJsonDocument<768>  (v14.3: was buf[450] — still too small)
      // doc<384> in v14.2 was silently dropping fields; doc<768> gives full headroom
      char buf[1024];
      size_t szAedf = serializeJson(a, buf, sizeof(buf));
      if (szAedf == 0 || szAedf >= sizeof(buf) - 1) {
        Serial.printf("[WARN] /fusion/aedf JSON truncated! sz=%u buf=%u\n",
                      (unsigned)szAedf, (unsigned)sizeof(buf));
      }
      if (mqttClient.publish(g_mqttTopicFusionAedf, buf, false, MQTT_QOS))
        Serial.printf("[FUSION/AEDF] %u B\n", (unsigned)szAedf);
      else
        Serial.printf("[FUSION/AEDF] FAILED -> %s\n", g_mqttTopicFusionAedf);
    }

    // ──────────────────────────────────────────────────────────────────────
    // PUB-D  /fusion/osg
    // Stage : Oscillatory Suppression Gate inputs + floor constant + multipliers
    // Note  : g_osg_r_s / g_osg_r_m are stored by runDecisionEngine (Patch B)
    // Doc   : 512 B stack (v14.3: was 384; actual JSON ~339 B → doc<384> ok but
    //         increased to 512 for future field additions)
    // Size  : doc 512 B, char buf[1024] (v14.3: was buf[450])
    // ──────────────────────────────────────────────────────────────────────
    {
      StaticJsonDocument<512> o;
      o["plant"]               = PLANT_ID;
      o["machine_id"]          = MACHINE_ID;
      o["stage"]               = "fusion_osg";
      o["execution_location"]  = "edge";
      o["floor_value"]         = OSG_FLOOR;

      JsonObject vi = o.createNestedObject("slope_var_inputs");
      vi["var_1s"]  = (float)g_slopeVar_1s;
      vi["var_10s"] = (float)g_slopeVar_10s;
      vi["var_60s"] = (float)g_slopeVar_60s;

      o["slope_ready_1s"]       = g_trendResult.slope_ready_1s;
      o["slope_ready_10s"]      = g_trendResult.slope_ready_10s;
      o["slope_ready_60s"]      = g_trendResult.slope_ready_60s;
      // Patch B: actual OSG*FVRI reliability multipliers (post suppression)
      o["r_s"]                  = roundf((float)g_osg_r_s * 1000.0f) / 1000.0f;
      o["r_m"]                  = roundf((float)g_osg_r_m * 1000.0f) / 1000.0f;
      o["post_osg_weights_in"]  = "ttw/model";
      o["timestamp"] = tsA;

      // buf[1024] >> StaticJsonDocument<512>  (v14.3: was buf[450])
      char buf[1024];
      size_t szOsg = serializeJson(o, buf, sizeof(buf));
      if (szOsg == 0 || szOsg >= sizeof(buf) - 1) {
        Serial.printf("[WARN] /fusion/osg JSON truncated! sz=%u buf=%u\n",
                      (unsigned)szOsg, (unsigned)sizeof(buf));
      }
      if (mqttClient.publish(g_mqttTopicFusionOsg, buf, false, MQTT_QOS))
        Serial.printf("[FUSION/OSG] %u B\n", (unsigned)szOsg);
      else
        Serial.printf("[FUSION/OSG] FAILED -> %s\n", g_mqttTopicFusionOsg);
    }

    // ──────────────────────────────────────────────────────────────────────
    // PUB-E  /fusion/fvri
    // Stage : Fault-Volatility Resonance Index
    // Doc   : 512 B stack (v14.3: was 384; actual JSON ~388 B → doc<384> was
    //         silently dropping fields — only 4 B gap, any float formatting
    //         expansion would push it over)
    // Size  : doc 512 B, char buf[1024] (v14.3: was buf[450])
    // ──────────────────────────────────────────────────────────────────────
    {
      StaticJsonDocument<512> f;
      f["plant"]               = PLANT_ID;
      f["machine_id"]          = MACHINE_ID;
      f["stage"]               = "fusion_fvri";
      f["execution_location"]  = "edge";

      JsonObject thr = f.createNestedObject("thresholds");
      thr["var_bearing"]   = FVRI_VAR_BEARING;
      thr["var_looseness"] = FVRI_VAR_LOOSENESS;
      thr["var_imbalance"] = FVRI_VAR_IMBALANCE;

      f["active_fault"]  = faultTypeStr(g_decision.fault_type);
      f["fault_score"]   = g_decision.fault_score;
      f["slope_var_1s"]  = (float)g_slopeVar_1s;
      f["slope_var_10s"] = (float)g_slopeVar_10s;
      f["slope_var_60s"] = (float)g_slopeVar_60s;

      // Patent field: FVRI bearing variance inversion active?
      bool bearingInv = (g_decision.fault_type == FAULT_BEARING) &&
                        (g_decision.fault_score >= 0.50f) &&
                        ((float)g_slopeVar_1s > FVRI_VAR_BEARING);
      f["bearing_inversion_active"] = bearingInv;

      // Post-FVRI normalized weights (committed to g_decision)
      f["w_short"]  = g_decision.ttw_w_s;
      f["w_medium"] = g_decision.ttw_w_m;
      f["w_long"]   = g_decision.ttw_w_l;

      f["timestamp"] = tsA;

      // buf[1024] >> StaticJsonDocument<512>  (v14.3: was buf[450] — still too small)
      // doc<384> in v14.2 was silently dropping fields (only 4 B gap)
      char buf[1024];
      size_t szFvri = serializeJson(f, buf, sizeof(buf));
      if (szFvri == 0 || szFvri >= sizeof(buf) - 1) {
        Serial.printf("[WARN] /fusion/fvri JSON truncated! sz=%u buf=%u\n",
                      (unsigned)szFvri, (unsigned)sizeof(buf));
      }
      if (mqttClient.publish(g_mqttTopicFusionFvri, buf, false, MQTT_QOS))
        Serial.printf("[FUSION/FVRI] %u B\n", (unsigned)szFvri);
      else
        Serial.printf("[FUSION/FVRI] FAILED -> %s\n", g_mqttTopicFusionFvri);
    }

    // ──────────────────────────────────────────────────────────────────────
    // PUB-F  /ttw/model
    // Stage : TTW derivation transparency
    // Fields: per-tier TTW + weights + ensemble + hysteresis state
    // Doc   : 512 B stack (actual JSON ~265 B → ample headroom)
    // Size  : doc 512 B, char buf[1024] (v14.3: was buf[580])
    // ──────────────────────────────────────────────────────────────────────
    {
      StaticJsonDocument<512> m;
      m["plant"]               = PLANT_ID;
      m["machine_id"]          = MACHINE_ID;
      m["stage"]               = "ttw_model";
      m["execution_location"]  = "edge";

      if (g_decision.ttw_short_h  > 0.0f) m["ttw_short_h"]  = g_decision.ttw_short_h;
      if (g_decision.ttw_medium_h > 0.0f) m["ttw_medium_h"] = g_decision.ttw_medium_h;
      if (g_decision.ttw_long_h   > 0.0f) m["ttw_long_h"]   = g_decision.ttw_long_h;

      m["w_short"]  = g_decision.ttw_w_s;
      m["w_medium"] = g_decision.ttw_w_m;
      m["w_long"]   = g_decision.ttw_w_l;
      m["ttw_best_h"] = g_decision.ttw_best_h;

      // Patent fields: hysteresis state
      m["ttw_prev_h"]         = roundf(g_prevTtwBest * 10.0f) / 10.0f;
      bool hystApplied = (g_prevTtwBest > 0.0f && g_decision.ttw_best_h > 0.0f)
                         && (fabsf(g_decision.ttw_best_h - g_prevTtwBest) /
                             g_prevTtwBest > 0.20f);
      m["hysteresis_applied"] = hystApplied;

      JsonObject sr = m.createNestedObject("slope_ready");
      sr["s1"]  = g_trendResult.slope_ready_1s;
      sr["s10"] = g_trendResult.slope_ready_10s;
      sr["s60"] = g_trendResult.slope_ready_60s;

      m["timestamp"] = tsA;

      // buf[1024] >> StaticJsonDocument<512>  (v14.3: was buf[580])
      char buf[1024];
      size_t sz = serializeJson(m, buf, sizeof(buf));
      if (sz == 0 || sz >= sizeof(buf) - 1) {
        Serial.printf("[WARN] /ttw/model JSON truncated! sz=%u buf=%u\n",
                      (unsigned)sz, (unsigned)sizeof(buf));
      }
      if (mqttClient.publish(g_mqttTopicTtwModel, buf, false, MQTT_QOS))
        Serial.printf("[TTW] /ttw/model %u B best=%.1fh prev=%.1fh hyst=%d\n",
                      (unsigned)sz, g_decision.ttw_best_h, g_prevTtwBest, (int)hystApplied);
      else
        Serial.printf("[TTW] FAILED -> %s\n", g_mqttTopicTtwModel);
    }

    // ──────────────────────────────────────────────────────────────────────
    // PUB-G  /output   (replaces /prediction — same 5-min cadence)
    // Stage : integrated final output for external consumers
    // Doc   : 768 B stack (v14.3: was 640; actual JSON ~541 B → needs ≥605 B doc)
    // Size  : doc 768 B, char buf[1024] (v14.3: was buf[700])
    // ──────────────────────────────────────────────────────────────────────
    {
      static uint32_t lastOutputMs = 0;
      uint32_t nowMs = millis();
      bool outputDue = (lastOutputMs == 0) ||
                       ((nowMs - lastOutputMs) >=
                        (uint32_t)PUBLISH_PREDICTION_INTERVAL_S * 1000UL);

      if (outputDue && mqttClient.connected()) {
        StaticJsonDocument<768> op;
        op["plant"]               = PLANT_ID;
        op["machine_id"]          = MACHINE_ID;
        op["sensor_id"]           = SENSOR_ID;
        op["stage"]               = "output";
        op["execution_location"]  = "edge";
        op["rms_ema"]             = roundf(g_emaRms * 100.0f) / 100.0f;

        op["fault_type"]          = faultTypeStr(g_decision.fault_type);
        op["fault_score"]         = g_decision.fault_score;
        op["fault_uncertainty"]   = g_decision.fault_uncertainty;
        op["severity_score"]      = g_decision.severity_score;
        op["confidence"]          = g_decision.confidence;

        op["predicted_state"]     = g_decision.predicted_state;
        op["final_state"]         = g_decision.final_state;
        op["state_rate_limited"]  = (g_decision.final_state != g_decision.predicted_state);
        op["alarm_class"]         = alarmClassStr(g_decision.alarm_class);
        op["action_needed"]       = g_decision.action_needed;
        op["reason"]              = g_decision.reason;

        JsonObject ttwO = op.createNestedObject("ttw");
        ttwO["best_h"] = g_decision.ttw_best_h;
        if (g_decision.ttw_short_h  > 0.0f) ttwO["short_h"]  = g_decision.ttw_short_h;
        if (g_decision.ttw_medium_h > 0.0f) ttwO["medium_h"] = g_decision.ttw_medium_h;
        if (g_decision.ttw_long_h   > 0.0f) ttwO["long_h"]   = g_decision.ttw_long_h;

        JsonObject slO = op.createNestedObject("slopes");
        slO["ready_1s"]  = g_trendResult.slope_ready_1s;
        slO["ready_10s"] = g_trendResult.slope_ready_10s;
        slO["ready_60s"] = g_trendResult.slope_ready_60s;
        if (g_trendResult.slope_ready_1s)
          slO["s1"]  = roundf(g_trendResult.slope_1s  * 100000.0f) / 100000.0f;
        if (g_trendResult.slope_ready_10s)
          slO["s10"] = roundf(g_trendResult.slope_10s * 100000.0f) / 100000.0f;
        if (g_trendResult.slope_ready_60s)
          slO["s60"] = roundf(g_trendResult.slope_60s * 100000.0f) / 100000.0f;

        JsonObject bufsO = op.createNestedObject("bufs");
        bufsO["s1"]  = g_buf1sCount;
        bufsO["s10"] = g_buf10sCount;
        bufsO["s60"] = g_buf60sCount;

        op["timestamp"] = tsA;

        // buf[1024] >> StaticJsonDocument<768>  (v14.3: was buf[700])
        // Actual JSON ~541 B; doc<640> in v14.2 gave only 99 B headroom
        char buf[1024];
        size_t sz = serializeJson(op, buf, sizeof(buf));
        if (sz == 0 || sz >= sizeof(buf) - 1) {
          Serial.printf("[WARN] /output JSON truncated! sz=%u buf=%u\n",
                        (unsigned)sz, (unsigned)sizeof(buf));
        }
        if (mqttClient.publish(g_mqttPredictionTopic, buf, false, MQTT_QOS)) {
          lastOutputMs = nowMs;
          Serial.printf("[OUTPUT] /output %u B pred=%u fin=%u ttw=%.1fh conf=%u%%\n",
                        (unsigned)sz, g_decision.predicted_state, g_decision.final_state,
                        g_decision.ttw_best_h, g_decision.confidence);
        } else {
          Serial.printf("[OUTPUT] FAILED -> %s\n", g_mqttPredictionTopic);
        }
      }
    }

  }  // end while(1)
}  // end taskAnalytics

// ============================================================================
// SETUP
// ============================================================================

// ── v15.3: Reset Reason Logger ──────────────────────────────────────────────
// เรียกแรกสุดใน setup() หลัง Serial.begin
// อ่าน esp_reset_reason() จาก hardware register (ไม่ขึ้นกับ UART/Serial)
// บันทึก reboot count ลง NVS (ไม่ reset เมื่อ power cycle)
// ──────────────────────────────────────────────────────────────────────────

void logResetReason() {
  esp_reset_reason_t reason = esp_reset_reason();
  g_resetReasonCode = (uint8_t)reason;

  // Map reason code → human-readable string + severity
  const char* emoji = "";
  switch (reason) {
    case ESP_RST_POWERON:
      strncpy(g_resetReasonStr, "POWER_ON",    sizeof(g_resetReasonStr));
      emoji = "✓";
      break;
    case ESP_RST_EXT:
      strncpy(g_resetReasonStr, "EXT_PIN",     sizeof(g_resetReasonStr));
      emoji = "✓";  // กดปุ่ม RESET
      break;
    case ESP_RST_SW:
      strncpy(g_resetReasonStr, "SW_RESET",    sizeof(g_resetReasonStr));
      emoji = "✓";  // esp_restart() / OTA
      break;
    case ESP_RST_BROWNOUT:
      strncpy(g_resetReasonStr, "BROWNOUT",    sizeof(g_resetReasonStr));
      emoji = "⚠";   // ไฟตก -- ตรวจ PSU / 4G current spike
      break;
    case ESP_RST_TASK_WDT:
      strncpy(g_resetReasonStr, "TASK_WDT",    sizeof(g_resetReasonStr));
      emoji = "!!";  // Task ค้าง > 30s
      break;
    case ESP_RST_INT_WDT:
      strncpy(g_resetReasonStr, "INT_WDT",     sizeof(g_resetReasonStr));
      emoji = "!!";  // Interrupt ค้าง
      break;
    case ESP_RST_PANIC:
      strncpy(g_resetReasonStr, "PANIC",       sizeof(g_resetReasonStr));
      emoji = "!!";  // Exception / Stack overflow
      break;
    case ESP_RST_DEEPSLEEP:
      strncpy(g_resetReasonStr, "DEEP_SLEEP",  sizeof(g_resetReasonStr));
      emoji = "✓";
      break;
    default:
      snprintf(g_resetReasonStr, sizeof(g_resetReasonStr), "UNKNOWN_%d", (int)reason);
      emoji = "?";
      break;
  }

  // อ่าน + อัปเดต reboot count จาก NVS
  Preferences resetPrefs;
  if (resetPrefs.begin("boot", false)) {
    g_rebootCount = resetPrefs.getUInt("count", 0) + 1;
    resetPrefs.putUInt("count", g_rebootCount);
    resetPrefs.end();
  }

  // Print banner
  Serial.println();
  Serial.println("+========================================================+");
  Serial.printf( "|  [BOOT] Reset Reason : %-6s %s\n", g_resetReasonStr, emoji);
  Serial.printf( "|  [BOOT] Reboot Count : #%lu\n", (unsigned long)g_rebootCount);
  Serial.println("+========================================================+");

  // Extra warning สำหรับสาเหตุที่ต้องสอบสวน
  if (reason == ESP_RST_BROWNOUT) {
    Serial.println("[BOOT] !! BROWNOUT detected -- ตรวจสอบ PSU / 4G current spike");
    Serial.println("[BOOT] !! ควรเพิ่ม capacitor หรือ separate power rail สำหรับ modem");
  } else if (reason == ESP_RST_TASK_WDT) {
    Serial.println("[BOOT] !! TASK WATCHDOG -- task ค้างเกิน 30s");
    Serial.println("[BOOT] !! ดู task ที่ไม่ได้ reset WDT ก่อน timeout");
  } else if (reason == ESP_RST_INT_WDT) {
    Serial.println("[BOOT] !! INTERRUPT WATCHDOG -- ISR ค้างนานเกินไป");
  } else if (reason == ESP_RST_PANIC) {
    Serial.println("[BOOT] !! PANIC/EXCEPTION -- ดู backtrace ใน log ก่อนหน้า");
    Serial.println("[BOOT] !! สาเหตุที่พบบ่อย: stack overflow, null ptr, heap corruption");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // v15.3: อ่าน reset reason แรกสุด ก่อน peripheral init ใดๆ
  // ต้องอยู่หลัง Serial.begin เพื่อให้ print ได้
  logResetReason();

  // FIX-WDT (v14.9): Extend Task Watchdog timeout to 30s.
  // Arduino ESP32 core default is 5s -- too short for 4G modem operations
  // (network registration ~10s, TLS handshake ~2s, MQTT publish over 4G).
  // esp_task_wdt_reconfigure() replaces the IDF default config at runtime.
  // trigger=true: panic+reboot on timeout (same as default behavior).
  {
    esp_task_wdt_config_t wdt_cfg = {
      .timeout_ms    = 30000,  // 30 seconds
      .idle_core_mask = (1 << 0) | (1 << 1),  // Watch both CPU0 and CPU1 IDLE
      .trigger_panic  = true,
    };
    esp_task_wdt_reconfigure(&wdt_cfg);
  }

  Serial.println("\n\n");
  Serial.println("+========================================================+");
  Serial.println("|  ESP32-S3 VIBRATION MONITOR v12.0 (Phase 5 Fusion AI)  |");
  Serial.println("|        LilyGO T-Vending S3 + SIMCom A7670             |");
  Serial.println("+========================================================+\n");

  // Print CPU info
  Serial.printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("Flash Size: %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));
  Serial.printf("Free Heap: %d bytes\n\n", ESP.getFreeHeap());

  // Build MQTT topic from identity (Phase 1)
  // ── MQTT Pipeline Topic Init (mqtt_pipeline_patch §D) ───────────────────────
  // Legacy backward-compat topic (/vibration) — field names UNCHANGED
  snprintf(g_mqttTopic, sizeof(g_mqttTopic),
           "factory/%s/machine/%s/vibration", PLANT_ID, MACHINE_ID);

  // Repurpose existing char arrays (no memory increase)
  //   g_mqttAnalyticsTopic  -> /trend   (60-s cadence, was /analytics)
  //   g_mqttPredictionTopic -> /output  (5-min cadence, was /prediction)
  snprintf(g_mqttAnalyticsTopic, sizeof(g_mqttAnalyticsTopic),
           "factory/%s/machine/%s/trend",  PLANT_ID, MACHINE_ID);
  snprintf(g_mqttPredictionTopic, sizeof(g_mqttPredictionTopic),
           "factory/%s/machine/%s/output", PLANT_ID, MACHINE_ID);

  // New pipeline-stage topics (declared in §A)
  snprintf(g_mqttTopicSensor,     sizeof(g_mqttTopicSensor),
           "factory/%s/machine/%s/sensor",      PLANT_ID, MACHINE_ID);
  snprintf(g_mqttTopicDecision,   sizeof(g_mqttTopicDecision),
           "factory/%s/machine/%s/decision",    PLANT_ID, MACHINE_ID);
  snprintf(g_mqttTopicTrend,      sizeof(g_mqttTopicTrend),
           "factory/%s/machine/%s/trend",       PLANT_ID, MACHINE_ID);
  snprintf(g_mqttTopicFusionRaw,  sizeof(g_mqttTopicFusionRaw),
           "factory/%s/machine/%s/fusion/raw",  PLANT_ID, MACHINE_ID);
  snprintf(g_mqttTopicFusionAedf, sizeof(g_mqttTopicFusionAedf),
           "factory/%s/machine/%s/fusion/aedf", PLANT_ID, MACHINE_ID);
  snprintf(g_mqttTopicFusionOsg,  sizeof(g_mqttTopicFusionOsg),
           "factory/%s/machine/%s/fusion/osg",  PLANT_ID, MACHINE_ID);
  snprintf(g_mqttTopicFusionFvri, sizeof(g_mqttTopicFusionFvri),
           "factory/%s/machine/%s/fusion/fvri", PLANT_ID, MACHINE_ID);
  snprintf(g_mqttTopicTtwModel,   sizeof(g_mqttTopicTtwModel),
           "factory/%s/machine/%s/ttw/model",        PLANT_ID, MACHINE_ID);
  snprintf(g_mqttTopicEvent,      sizeof(g_mqttTopicEvent),
           "factory/%s/machine/%s/vibration/event",  PLANT_ID, MACHINE_ID);  // V14.4

  Serial.println("[Init] MQTT Pipeline Topics:");
  Serial.printf("  /vibration (compat): %s\n", g_mqttTopic);
  Serial.printf("  /sensor:             %s\n", g_mqttTopicSensor);
  Serial.printf("  /decision:           %s\n", g_mqttTopicDecision);
  Serial.printf("  /trend:              %s\n", g_mqttTopicTrend);
  Serial.printf("  /fusion/raw:         %s\n", g_mqttTopicFusionRaw);
  Serial.printf("  /fusion/aedf:        %s\n", g_mqttTopicFusionAedf);
  Serial.printf("  /fusion/osg:         %s\n", g_mqttTopicFusionOsg);
  Serial.printf("  /fusion/fvri:        %s\n", g_mqttTopicFusionFvri);
  Serial.printf("  /ttw/model:          %s\n", g_mqttTopicTtwModel);
  Serial.printf("  /output:             %s\n", g_mqttPredictionTopic);
  Serial.printf("  /event:              %s\n", g_mqttTopicEvent);            // V14.4
  Serial.printf("[Init] Identity: plant=%s  machine=%s  sensor=%s  rated_rpm=%d\n",
                PLANT_ID, MACHINE_ID, SENSOR_ID, RATED_RPM);
  // ─────────────────────────────────────────────────────────────────────────────

  // Initialize GPIO
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BUTTON_ENTER, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(RS485_EN_PIN, OUTPUT);
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(BUILTIN_LED, LOW);
  rs485Enable();

  Serial.println("[Init] GPIO configured");

  // Initialize I2C and OLED
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);  // 400 kHz

  u8g2.begin();
  u8g2.setContrast(255);

  // Splash screen
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.drawStr(25, 20, "4G LTE");
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(10, 40, "Vib Monitor");
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(30, 55, MACHINE_NAME);
  u8g2.drawStr(20, 64, "SIMCom A7670");
  u8g2.sendBuffer();

  Serial.println("[Init] OLED initialized");

  // Initialize RTC
  if (rtc.begin()) {
    g_rtcValid = true;
    DateTime now = rtc.now();
    DateTime compiled(F(__DATE__), F(__TIME__));

    if (rtc.lostPower()) {
      // RTC battery died -- set compile time as temporary fallback
      // This will be corrected by NTP sync once 4G connects
      Serial.println("[Init] RTC lost power! Setting compile time as fallback...");
      Serial.println("[Init] ! Time will be corrected via NTP after 4G connects");
      rtc.adjust(compiled);
      g_timeSync.synced = false;  // Mark as not synced
    } else if (now.year() < 2024 || now.unixtime() < compiled.unixtime()) {
      // RTC has invalid/old time -- likely battery issue
      Serial.println("[Init] RTC time appears invalid, setting compile time as fallback");
      rtc.adjust(compiled);
      g_timeSync.synced = false;
    } else {
      Serial.println("[Init] RTC time looks reasonable (will verify via NTP)");
    }

    now = rtc.now();
    Serial.printf("[Init] RTC: %04d-%02d-%02d %02d:%02d:%02d (sync pending)\n",
                  now.year(), now.month(), now.day(),
                  now.hour(), now.minute(), now.second());
  } else {
    Serial.println("[Init] RTC not found! Timestamps will be unavailable until NTP sync.");
    g_rtcValid = false;
  }

  // Initialize Modbus
  SerialRS485.begin(MODBUS_BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  modbus.begin(MODBUS_SLAVE_ID, SerialRS485);

  Serial.printf("[Init] Modbus @ %d baud, ID: 0x%02X\n",
                MODBUS_BAUDRATE, MODBUS_SLAVE_ID);

  // Force sensor config every boot (sampling rate, baud rate) then reboot sensor
  // ทำก่อน FreeRTOS tasks เริ่ม -- ใช้ delay() ได้ปกติ
  rs485Enable();
  delay(100);  // รอ RS485 transceiver stable
  if (!forceSensorConfig()) {
    Serial.println("[Init] WARNING: forceSensorConfig() failed -- sensor may use defaults");
  }
  rs485Disable();

  // -- Initialize Proximity / RPM Sensor (ISR-based, GPIO17) --
  loadRuntimeHour();                                       // ???? runtime ??????? NVS Flash
  pinMode(PIN_RPM, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_RPM), rpmISR, FALLING);
  g_rpmLastPulseMillis = millis();                         // ??????? false STOPPING ??? boot
  Serial.printf("[Init] RPM sensor GPIO%d | RATED=%d RPM +/-%d | PPR=%d\n",
                PIN_RPM, RATED_RPM, RATED_RPM_TOL, PULSE_PER_REV);

  // Create Mutexes
  mutexVibData    = xSemaphoreCreateMutex();
  mutexSystemState = xSemaphoreCreateMutex();
  mutexI2C        = xSemaphoreCreateMutex();
  mutexModem      = xSemaphoreCreateMutex();
  mutexAggBufs    = xSemaphoreCreateMutex();  // Phase 2: guard g_buf1s/10s/60s

  if (mutexVibData == NULL || mutexSystemState == NULL ||
      mutexI2C == NULL || mutexModem == NULL || mutexAggBufs == NULL) {
    Serial.println("[FATAL] Failed to create mutexes!");
    while (1) delay(1000);
  }

  Serial.println("[Init] Mutexes created");

  // Create Queues
  queueSensorData = xQueueCreate(QUEUE_SIZE_SENSOR, sizeof(VibrationData_t));
  queueButtonEvent = xQueueCreate(QUEUE_SIZE_BUTTON, sizeof(ButtonEvent_t));
  queueDisplayUpdate = xQueueCreate(QUEUE_SIZE_DISPLAY, sizeof(DisplayCommand_t));
  queueMaintEvent = xQueueCreate(QUEUE_SIZE_MAINT, sizeof(MaintenanceEvent_t));  // V14.4

  if (queueSensorData == NULL || queueButtonEvent == NULL || queueDisplayUpdate == NULL
      || queueMaintEvent == NULL) {
    Serial.println("[FATAL] Failed to create queues!");
    while (1) delay(1000);
  }

  Serial.println("[Init] Queues created");

  delay(2000);

  // ========================================================================
  // CREATE TASKS - CORE ASSIGNMENT
  // ========================================================================

  Serial.println("\n[Init] Creating FreeRTOS tasks...\n");

  // CORE 0 - Time-critical operations
  Serial.println("+--- CORE 0 (PRO_CPU) - Time Critical ---------------+");

  xTaskCreatePinnedToCore(
    taskModbusRead,
    "ModbusRead",
    STACK_SIZE_MODBUS,
    NULL,
    PRIORITY_MODBUS,
    &taskHandleModbus,
    0);
  Serial.printf("| [+] Modbus RTU       (Priority %d, Stack %d)      |\n",
                PRIORITY_MODBUS, STACK_SIZE_MODBUS);

  xTaskCreatePinnedToCore(
    taskStateMachine,
    "StateMachine",
    STACK_SIZE_STATE,
    NULL,
    PRIORITY_STATE,
    &taskHandleState,
    0);
  Serial.printf("| [+] State Machine    (Priority %d, Stack %d)      |\n",
                PRIORITY_STATE, STACK_SIZE_STATE);

  Serial.println("+----------------------------------------------------+\n");

  // CORE 1 - UI and network
  Serial.println("+--- CORE 1 (APP_CPU) - User Interface --------------+");

  xTaskCreatePinnedToCore(
    taskDisplayUpdate,
    "DisplayUpdate",
    STACK_SIZE_DISPLAY,
    NULL,
    PRIORITY_DISPLAY,
    &taskHandleDisplay,
    1);
  Serial.printf("| [+] OLED Display     (Priority %d, Stack %d)      |\n",
                PRIORITY_DISPLAY, STACK_SIZE_DISPLAY);

  xTaskCreatePinnedToCore(
    taskNetwork,
    "Network4G",
    STACK_SIZE_NETWORK,
    NULL,
    PRIORITY_NETWORK,
    &taskHandleNetwork,
    1);
  Serial.printf("| [+] 4G Modem/MQTT    (Priority %d, Stack %d)     |\n",
                PRIORITY_NETWORK, STACK_SIZE_NETWORK);

  xTaskCreatePinnedToCore(
    taskAnalytics,
    "Analytics",
    STACK_SIZE_ANALYTICS,
    NULL,
    PRIORITY_ANALYTICS,
    &taskHandleAnalytics,
    1);
  Serial.printf("| [+] Analytics        (Priority %d, Stack %d)      |\n",
                PRIORITY_ANALYTICS, STACK_SIZE_ANALYTICS);

  xTaskCreatePinnedToCore(
    taskButtonHandler,
    "ButtonHandler",
    STACK_SIZE_BUTTON,
    NULL,
    PRIORITY_BUTTON,
    &taskHandleButton,
    1);
  Serial.printf("| [+] Button Input     (Priority %d, Stack %d)      |\n",
                PRIORITY_BUTTON, STACK_SIZE_BUTTON);

  xTaskCreatePinnedToCore(
    taskBuzzerControl,
    "BuzzerControl",
    2048,
    NULL,
    PRIORITY_BUZZER,
    &taskHandleBuzzer,
    1);
  Serial.printf("| [+] Buzzer Control   (Priority %d, Stack %d)      |\n",
                PRIORITY_BUZZER, 2048);

  Serial.println("+----------------------------------------------------+\n");

  Serial.println("+========================================================+");
  Serial.println("|  SYSTEM READY -- 4G LTE + mTLS + Phase 5 Fusion AI     |");
  Serial.println("+========================================================+\n");

  Serial.println("Network Configuration:");
  Serial.printf("  APN:       %s\n", APN);
  Serial.printf("  MQTT:      %s:%d (mTLS)\n", MQTT_SERVER, MQTT_PORT);
  Serial.printf("  Client ID: %s\n", MQTT_CLIENT_ID);
  Serial.printf("  QoS:       %d\n", MQTT_QOS);
  Serial.printf("  Topic:     %s\n\n", g_mqttTopic);

  Serial.println("Vibration Thresholds:");
  Serial.printf("  Baseline: %.1f mm/s\n", BASELINE_RMS);
  Serial.printf("  Warning:  %.1f mm/s\n", WARNING_RMS);
  Serial.printf("  Critical: %.1f mm/s\n\n", CRITICAL_RMS);

  Serial.println("Adaptive Sending:");
  Serial.println("  NORMAL:   30 seconds");
  Serial.println("  WARNING:  10 seconds");
  Serial.println("  CRITICAL: 5 seconds\n");

  Serial.println("Phase 2 Analytics:");
  Serial.println("  taskAnalytics: Core 1, 1 Hz, Priority 3");
  Serial.println("  Buffers: g_buf1s[60]=60s  g_buf10s[60]=10min  g_buf60s[60]=60min");
  Serial.printf("  RAM overhead: ~%d bytes  (~%.1f KB)\n",
                (int)(sizeof(g_buf1s) + sizeof(g_buf10s) + sizeof(g_buf60s)),
                (sizeof(g_buf1s) + sizeof(g_buf10s) + sizeof(g_buf60s)) / 1024.0f);
  Serial.printf("  Analytics topic: %s\n\n", g_mqttAnalyticsTopic);

  Serial.println("Phase 5 Adaptive Fusion Decision Engine:");
  Serial.println("  classifyFaultProbabilistic(): Gaussian multi-candidate (7 hypotheses)");
  Serial.println("  AEDF: per-fault evidence decay (fault-type-specific tau)");
  Serial.println("  TSI: Trend Stability Index -> adaptive severity weight modulation");
  Serial.println("  OSG: Oscillatory Suppression Gate (continuous, confidence-weighted)");
  Serial.println("  FVRI: Fault-Volatility Resonance Index (cross-module feedback)");
  Serial.println("  TTW: variance-derived reliability weights + hysteresis");
  Serial.println("  Confidence: orthogonal geometric mean (depth x agreement x separation)");
  Serial.println("  Fusion: 4-typed conflict resolution + state damping");
  Serial.printf("  Pump blades: %d  BPF ratio: %.1fx\n", PUMP_BLADES, (float)PUMP_BLADES);
  Serial.printf("  Slope guards: 1s>=%d  10s>=%d  60s>=%d slots\n",
                SLOPE_1S_MIN_SLOTS, SLOPE_10S_MIN_SLOTS, SLOPE_60S_MIN_SLOTS);
  Serial.printf("  TTW long min rate: %.2f mm/s/h  cap: %.0fh\n",
                TTW_LONG_MIN_RATE_H, TTW_MAX_HOURS);
  Serial.printf("  Prediction topic: %s  (every %ds)\n\n",
                g_mqttPredictionTopic, PUBLISH_PREDICTION_INTERVAL_S);

  Serial.println("NTP Time Sync:");
  Serial.printf("  Sync Interval:  %lu hours\n", NTP_SYNC_INTERVAL / 3600000UL);
  Serial.printf("  Retry Interval: %lu minutes\n", NTP_SYNC_RETRY_INTERVAL / 60000UL);
  Serial.printf("  Drift Warn:     %d seconds\n", NTP_DRIFT_WARN_SEC);
  Serial.printf("  Drift Max:      %d seconds\n\n", NTP_DRIFT_MAX_SEC);

  Serial.println("FreeRTOS scheduler will now take over...\n");
}

// ============================================================================
// MAIN LOOP (Runs on Core 1 by default)
// ============================================================================

void loop() {
  // Statistics and monitoring (low priority background task)
  static uint32_t lastStats = 0;
  uint32_t now = millis();

  // Blink LED to show system is alive
  static uint32_t lastBlink = 0;
  if (now - lastBlink >= 1000) {
    digitalWrite(BUILTIN_LED, !digitalRead(BUILTIN_LED));
    lastBlink = now;
  }

  if (now - lastStats >= 30000) {  // Every 30 seconds
    lastStats = now;

    Serial.println("\n+========================================================+");
    Serial.println("|              SYSTEM STATUS REPORT (4G mTLS)            |");
    Serial.println("+========================================================+");

    // Get current data
    VibrationData_t localVib;
    MachineState_t localState;

    if (xSemaphoreTake(mutexVibData, pdMS_TO_TICKS(10)) == pdTRUE) {
      memcpy(&localVib, &g_vibData, sizeof(VibrationData_t));
      xSemaphoreGive(mutexVibData);
    }

    if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(10)) == pdTRUE) {
      localState = g_systemState.state;
      xSemaphoreGive(mutexSystemState);
    }

    Serial.printf("| Machine: %-45s |\n", MACHINE_NAME);

    const char* stateStr = "UNKNOWN";
    if (localState == STATE_NORMAL) stateStr = "NORMAL";
    else if (localState == STATE_WARNING) stateStr = "WARNING";
    else if (localState == STATE_CRITICAL) stateStr = "CRITICAL";
    else if (localState == STATE_MAINTENANCE) stateStr = "MAINTENANCE";
    Serial.printf("| State:   %-45s |\n", stateStr);

    Serial.printf("| RMS:     %-42.2f mm/s |\n", localVib.rms_overall);
    Serial.printf("| Temp:    %-43.1f  degC |\n", localVib.temperature);

    Serial.println("+========================================================+");
    Serial.printf("| Modem:   %-45s |\n", g_network.modemReady ? "READY" : "NOT READY");
    Serial.printf("| GPRS:    %-45s |\n", g_network.gprsConnected ? "CONNECTED" : "DISCONNECTED");
    Serial.printf("| MQTT:    %-45s |\n", mqttClient.connected() ? "CONNECTED (mTLS)" : "DISCONNECTED");
    Serial.printf("| Signal:  %d%% (CSQ: %d)                                |\n",
                  g_network.signalPercent, g_network.signalQuality);
    Serial.printf("| Operator: %-44s |\n", g_network.operatorName);

    Serial.println("+========================================================+");
    Serial.printf("| Sensor Reads:    %8lu (Errors: %8lu)       |\n",
                  g_sensorReads, g_sensorErrors);
    Serial.printf("| Display Updates: %8lu                          |\n",
                  g_displayUpdates);
    Serial.printf("| MQTT Publishes:  %8u (Failures: %8u)     |\n",
                  g_network.publishCount, g_network.publishFailures);

    // Vx / Vy / Vz Stuck Recovery Status
    Serial.println("+========================================================+");
    Serial.printf("| Stuck Threshold: %2u reads x 250ms = %.2f s           |\n",
                  STUCK_THRESHOLD, STUCK_THRESHOLD * 0.25f);
    Serial.printf("| Vx Stuck Count:  %3u / %u   (Restarts: %3u)           |\n",
                  g_vxStuckCount, STUCK_THRESHOLD, g_vxRestartCount);
    Serial.printf("| Vy Stuck Count:  %3u / %u   (Restarts: %3u)           |\n",
                  g_vyStuckCount, STUCK_THRESHOLD, g_vyRestartCount);
    Serial.printf("| Vz Stuck Count:  %3u / %u   (Restarts: %3u)           |\n",
                  g_vzStuckCount, STUCK_THRESHOLD, g_vzRestartCount);
    if (g_lastSensorRestart > 0) {
      uint32_t ageSec = (millis() - g_lastSensorRestart) / 1000;
      Serial.printf("| Last Restart:    %lu sec ago                       |\n", ageSec);
    } else {
      Serial.printf("| Last Restart:    never                              |\n");
    }

    Serial.println("+========================================================+");
    Serial.printf("| Free Heap:       %8d bytes                     |\n",
                  ESP.getFreeHeap());
    Serial.printf("| Min Free Heap:   %8d bytes                     |\n",
                  ESP.getMinFreeHeap());
    Serial.printf("| Uptime:          %8lu seconds                  |\n",
                  millis() / 1000);

    // RTC time if available
    if (g_rtcValid) {
      DateTime rtcNow = rtc.now();
      Serial.printf("| RTC Time: %04d-%02d-%02d %02d:%02d:%02d                    |\n",
                    rtcNow.year(), rtcNow.month(), rtcNow.day(),
                    rtcNow.hour(), rtcNow.minute(), rtcNow.second());
    }

    // NTP Time Sync status
    Serial.println("+========================================================+");
    Serial.printf("| NTP Synced:  %-41s |\n", g_timeSync.synced ? "YES +" : "NO x");
    if (g_timeSync.synced) {
      uint32_t ageSec = (millis() - g_timeSync.lastSyncMillis) / 1000;
      Serial.printf("| Last Sync:   %-41s |\n", g_timeSync.lastSyncTime);
      Serial.printf("| Sync Age:    %lu seconds ago                        |\n", ageSec);
      Serial.printf("| Last Drift:  %+d seconds                            |\n", g_timeSync.lastDriftSec);
    }
    Serial.printf("| Sync Count:  %lu  (Failures: %lu)                    |\n",
                  g_timeSync.syncCount, g_timeSync.syncFailures);

    Serial.println("+========================================================+\n");

    // Task stack watermarks (debug info)
    Serial.println("Task Stack High Water Marks (bytes remaining):");
    Serial.printf("  Modbus:      %u\n", uxTaskGetStackHighWaterMark(taskHandleModbus));
    Serial.printf("  State:       %u\n", uxTaskGetStackHighWaterMark(taskHandleState));
    Serial.printf("  Display:     %u\n", uxTaskGetStackHighWaterMark(taskHandleDisplay));
    Serial.printf("  Network:     %u\n", uxTaskGetStackHighWaterMark(taskHandleNetwork));
    Serial.printf("  Analytics:   %u\n", uxTaskGetStackHighWaterMark(taskHandleAnalytics));
    Serial.printf("  Button:      %u\n", uxTaskGetStackHighWaterMark(taskHandleButton));
    Serial.printf("  Buzzer:      %u\n\n", uxTaskGetStackHighWaterMark(taskHandleBuzzer));

    // Phase 2: Analytics buffer fill status
    Serial.printf("Analytics Buffer Fill: buf1s=%u/60  buf10s=%u/60  buf60s=%u/60\n",
                  g_buf1sCount, g_buf10sCount, g_buf60sCount);
    Serial.printf("EMA: rms=%.3f delta=%.5f dir=%+d\n",
                  g_emaRms, g_emaDelta, (int)g_emaDir);

    // Phase 3+4: Decision Engine last result
    Serial.printf("Decision: fault=%-12s sc=%.2f unc=%.2f sev=%3u conf=%3u%% "
                  "raw=%u fin=%u %s ttw=%.1fh(s=%.1f*%.2f m=%.1f*%.2f l=%.1f*%.2f) "
                  "act=%d pub=%lus\n",
                  faultTypeStr(g_decision.fault_type),
                  g_decision.fault_score, g_decision.fault_uncertainty,
                  g_decision.severity_score, g_decision.confidence,
                  g_decision.predicted_state, g_decision.final_state,
                  alarmClassStr(g_decision.alarm_class),
                  g_decision.ttw_best_h,
                  g_decision.ttw_short_h, g_decision.ttw_w_s,
                  g_decision.ttw_medium_h, g_decision.ttw_w_m,
                  g_decision.ttw_long_h, g_decision.ttw_w_l,
                  (int)g_decision.action_needed,
                  (unsigned long)(g_decision.publish_interval_ms/1000));
    Serial.printf("Reason: %s\n\n", g_decision.reason);
  }

  // This loop runs at low priority on Core 1
  // Main work is done by FreeRTOS tasks
  vTaskDelay(pdMS_TO_TICKS(1000));  // Sleep for 1 second
}
