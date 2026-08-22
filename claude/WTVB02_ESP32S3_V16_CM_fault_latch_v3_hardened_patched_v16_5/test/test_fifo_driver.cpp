// ============================================================================
// [v16.6.14-fifo] test_fifo_driver.cpp
//
// [Task 3.7] Host-only integration test suite for the complete FifoDriver
// public API (Task 3.6) and everything it composes (Session Controller,
// FifoArena, FrameCodec, the S0-S14 state machine, Tasks 3.4/3.5's
// timeout/retry/circuit-breaker logic). Matches the frozen Implementation
// Plan's own naming for this exact deliverable ("test/test_fifo_driver.cpp").
//
// HOST-ONLY -- lives under test/, matching log_replay_transport.h's own
// convention, never compiled into the firmware.
//
// Hand-rolled assertion harness, no external test-framework dependency --
// matches this driver's own zero-extra-dependency philosophy and avoids
// assuming a framework (Unity/gtest/etc.) is available in this environment.
//
// [IMPORTANT -- read before trusting a PASS] This file has been written
// and reasoned through carefully, tracing the production code tick-by-
// tick by hand, but it has NOT been compiled or executed -- "Do not
// build" applied to this task exactly as it did to every implementation
// task before it. Treat every test below as "expected to pass, based on
// a careful manual trace," not as "proven to pass." One real defect WAS
// found this way (see fifo_driver.cpp's "[Task 3.7 defect fix]" comments
// in HandleS11ResultReady()/FifoDriver_TryAcquireResult()) and corrected
// in the production file before this suite was finalized around it.
//
// Frame fixtures are built using the SAME frozen StreamingCrc16
// (fifo_codec.h, Task 2.1) production code uses, rather than hand-
// computed/hardcoded CRC bytes -- eliminates an entire class of "the
// test fixture's own CRC is wrong" risk.
//
// Coverage gap, disclosed: the SS17.5 result-hold watchdog's 30 s
// threshold is measured against FifoArena_HeldSinceMs(), which reads
// std::chrono::steady_clock directly (a disclosed Task 3.3A compromise --
// its own zero-parameter frozen signature has no injection point). This
// is the ONE timing value in the entire driver NOT controllable via
// LogReplayTransport's simulated clock. Test_WatchdogReclaim() below is
// written but marked SLOW and is not exercised by RunFastSuite() -- see
// that test's own comment and this file's "Remaining architectural
// risks" writeup in the delivered Integration Report.
// ============================================================================

#include "../fifo_driver.h"
#include "../fifo_types.h"
#include "../fifo_codec.h"
#include "../fifo_arena.h"
#include "log_replay_transport.h"

#include <cstdio>
#include <cstring>
#include <vector>
#include <thread>
#include <chrono>

// ----------------------------------------------------------------------------
// Minimal assertion harness.
// ----------------------------------------------------------------------------
static int g_testsRun = 0;
static int g_testsFailed = 0;
static int g_assertionsInCurrentTest = 0;
static const char* g_currentTestName = "";

#define TEST_BEGIN(name)                    \
  do {                                      \
    g_currentTestName = name;               \
    g_assertionsInCurrentTest = 0;          \
    std::printf("--- %s ---\n", name);      \
    g_testsRun++;                           \
  } while (0)

#define TEST_ASSERT(cond, msg)                                              \
  do {                                                                      \
    g_assertionsInCurrentTest++;                                            \
    if (!(cond)) {                                                          \
      std::printf("  FAIL [%s]: %s (line %d)\n", g_currentTestName, msg,    \
                   __LINE__);                                               \
      g_testsFailed++;                                                      \
    }                                                                       \
  } while (0)

#define TEST_END()                                                     \
  do {                                                                 \
    std::printf("  (%d assertions)\n", g_assertionsInCurrentTest);     \
  } while (0)

// ----------------------------------------------------------------------------
// Simulated clock, shared by every test via LogReplayTransport's
// injection points. Reset at the start of every test.
// ----------------------------------------------------------------------------
static uint32_t g_simNowMs = 0;
static bool g_simOverflow = false;

static void ResetClock() {
  g_simNowMs = 0;
  g_simOverflow = false;
}

static void AdvanceMs(uint32_t deltaMs) {
  g_simNowMs += deltaMs;
}

// Ticks the driver, advancing simulated time in small steps so internal
// timeouts/quiet-periods are observed at realistic granularity rather
// than jumped over in one shot.
static void RunFor(uint32_t totalMs, uint32_t stepMs = 50) {
  for (uint32_t elapsed = 0; elapsed < totalMs; elapsed += stepMs) {
    AdvanceMs(stepMs);
    FifoDriver_Service();
  }
}

// [R-4.1] Ticks one service call at a time until `want` is observed, or
// maxTicks elapse. Returns true if it was observed.
//
// Why this replaces "RunFor(N) then assert phase": a transient phase can be
// entered and left inside a single RunFor() window, so sampling only at the
// END of the window silently misses it. S12_COOLDOWN is exactly that case --
// ADR-0006 D-3 amended T_COOLDOWN_MS from 60000 to 0, so COOLDOWN now lasts
// ONE service tick. The old test asserted COOLDOWN after RunFor(100,50) (two
// ticks) and therefore always sampled one tick too late, after IDLE.
//
// Checking BEFORE each tick makes this policy-agnostic: it passes whether the
// dwell is 0 ms (current) or 60000 ms (if ADR-0006 is ever reverted) -- only
// maxTicks needs to be generous enough for the long case.
static bool RunUntilPhase(FifoPhase want, uint32_t maxTicks, uint32_t stepMs = 50) {
  for (uint32_t i = 0; i < maxTicks; i++) {
    if (FifoDriver_GetPhase() == want) {
      return true;
    }
    AdvanceMs(stepMs);
    FifoDriver_Service();
  }
  return FifoDriver_GetPhase() == want;
}

// ----------------------------------------------------------------------------
// Frame construction helpers.
// ----------------------------------------------------------------------------
static void AppendCrc(std::vector<uint8_t>& frame) {
  uint16_t crc;
  Crc16_Init(&crc);
  for (uint8_t b : frame) {
    Crc16_Update(&crc, b);
  }
  uint16_t final = Crc16_Final(crc);
  frame.push_back(static_cast<uint8_t>(final & 0xFF));         // CRC_lo (K-6)
  frame.push_back(static_cast<uint8_t>((final >> 8) & 0xFF));  // CRC_hi
}

// PROGRESS frame (K-3): 50 03 01 <fill_hi> <fill_lo> <CRC_lo> <CRC_hi>
static std::vector<uint8_t> BuildProgressFrame(uint16_t fill) {
  std::vector<uint8_t> frame = {0x50, 0x03, 0x01,
                                 static_cast<uint8_t>((fill >> 8) & 0xFF),
                                 static_cast<uint8_t>(fill & 0xFF)};
  AppendCrc(frame);
  return frame;
}

// FULL_DUMP frame (K-4/K-8): 50 03 00 <6144 payload bytes> <CRC_lo> <CRC_hi>
// 1024 samples x 3 axes x int16 big-endian, stride 6, X/Y/Z order.
static std::vector<uint8_t> BuildDumpFrame(const int16_t xVals[1024],
                                            const int16_t yVals[1024],
                                            const int16_t zVals[1024]) {
  std::vector<uint8_t> frame = {0x50, 0x03, 0x00};
  frame.reserve(3 + 6144 + 2);
  for (int i = 0; i < 1024; i++) {
    uint16_t x = static_cast<uint16_t>(xVals[i]);
    uint16_t y = static_cast<uint16_t>(yVals[i]);
    uint16_t z = static_cast<uint16_t>(zVals[i]);
    frame.push_back(static_cast<uint8_t>((x >> 8) & 0xFF));
    frame.push_back(static_cast<uint8_t>(x & 0xFF));
    frame.push_back(static_cast<uint8_t>((y >> 8) & 0xFF));
    frame.push_back(static_cast<uint8_t>(y & 0xFF));
    frame.push_back(static_cast<uint8_t>((z >> 8) & 0xFF));
    frame.push_back(static_cast<uint8_t>(z & 0xFF));
  }
  AppendCrc(frame);
  return frame;
}

// Deliberately CRC-corrupted FULL_DUMP frame, for the frame-level
// self-heal (CRC failure) test.
static std::vector<uint8_t> BuildCorruptedDumpFrame(const int16_t xVals[1024],
                                                      const int16_t yVals[1024],
                                                      const int16_t zVals[1024]) {
  std::vector<uint8_t> frame = BuildDumpFrame(xVals, yVals, zVals);
  frame.back() ^= 0xFF;  // corrupt CRC_hi -- guaranteed mismatch
  return frame;
}

// Anchor + BAD type byte (0x02 is neither 0x00 PROGRESS nor 0x01... wait,
// K-5: 0x00=full dump, 0x01=progress -- 0x02 is neither. No CRC tail
// needed: BAD_TYPE_BYTE fires the instant the type byte itself is read,
// before any further bytes are consumed or any CRC comparison happens.
static std::vector<uint8_t> BuildBadTypeByteFrame() {
  return {0x50, 0x03, 0x02};
}

static void AppendSegment(std::vector<uint8_t>& allBytes,
                           std::vector<uint32_t>& allTimestamps,
                           const std::vector<uint8_t>& segment, uint32_t atMs) {
  for (uint8_t b : segment) {
    allBytes.push_back(b);
    allTimestamps.push_back(atMs);
  }
}

static void SampleXYZ(int seed, int16_t out[1024]) {
  for (int i = 0; i < 1024; i++) {
    out[i] = static_cast<int16_t>((i * 7 + seed) % 2000 - 1000);
  }
}

static FifoCaptureRequest MakeAdmissibleRequest() {
  FifoCaptureRequest req{};
  req.triggerSource = FifoTriggerSource::SCHEDULED;
  const char* tag = "t7";
  for (size_t i = 0; i < FIFO_TAG_MAXLEN && tag[i] != '\0'; i++) {
    req.tag[i] = tag[i];
  }
  req.requirePermissive = true;
  req.maxRetries = 2;  // informational on the request; FIFO_MAX_RETRIES is
                        // the driver's own fixed constant, not read from here
  req.admissionContext.motorStable = true;
  req.admissionContext.sensorHealthy = true;
  req.admissionContext.mqttReconnecting = false;
  // [R-3] Machine-state provenance. Deliberately NON-ZERO and distinctive so a
  // regression to the old "declared but never assigned" behaviour (which
  // published motor_state=0 rpm=0 temp_c=0) fails loudly instead of silently
  // matching a zero-initialised result. Values mirror the real hardware
  // observation that exposed the defect: RUNNING @ ~1484 rpm, ~51.1 degC.
  req.motorStateAtCapture = 2;        // MOTOR_RUNNING
  req.rpmAtCapture        = 1484.5f;
  req.tempCAtCapture      = 51.1f;
  return req;
}

// Drains a fully-acquired, fully-released result all the way back to
// IDLE, for tests that don't care about COOLDOWN's own timing in detail.
// Requires FifoDriver_GetPhase() == RESULT_READY on entry.
static void AcquireReleaseAndCooldown() {
  FifoCaptureResult result{};
  bool acquired = FifoDriver_TryAcquireResult(&result);
  TEST_ASSERT(acquired, "acquire succeeds when RESULT_READY");
  FifoDriver_ReleaseResult();
  RunFor(100, 50);  // one tick suffices post-[Task 3.7 defect fix]
  TEST_ASSERT(FifoDriver_GetPhase() == FifoPhase::COOLDOWN, "enters COOLDOWN after release");
  RunFor(62000, 500);  // T_COOLDOWN_MS = 60000
}

// ============================================================================
// 1. End-to-end success flow
//    Request -> Driver -> Session Controller -> Arena -> RESULT_READY ->
//    Acquire -> Release -> Idle
// ============================================================================
static void Test_EndToEndSuccess() {
  TEST_BEGIN("EndToEndSuccess");
  ResetClock();

  static int16_t xVals[1024], yVals[1024], zVals[1024];
  SampleXYZ(1, xVals);
  SampleXYZ(2, yVals);
  SampleXYZ(3, zVals);

  std::vector<uint8_t> bytes;
  std::vector<uint32_t> timestamps;
  AppendSegment(bytes, timestamps, BuildDumpFrame(xVals, yVals, zVals), 260);

  FifoTransport transport;
  LogReplayTransport_Init(&transport, bytes.data(), bytes.size(), timestamps.data(),
                           &g_simNowMs, &g_simOverflow);

  FifoDriver_Init(&transport);
  TEST_ASSERT(FifoDriver_GetPhase() == FifoPhase::IDLE, "starts IDLE");

  FifoCaptureRequest req = MakeAdmissibleRequest();
  uint32_t handle = 0;
  FifoError verdict = FifoDriver_Request(&req, &handle);
  TEST_ASSERT(verdict == FifoError::NONE, "request admitted");
  TEST_ASSERT(handle == 1, "first handle is 1");
  // Request() only sets s_requestPending synchronously -- s_state itself
  // does not move off S1_IDLE until HandleS1Idle() consumes that flag on
  // the driver's own next tick, so phase is still IDLE at this exact
  // point, not yet ACTIVE.
  TEST_ASSERT(FifoDriver_GetPhase() == FifoPhase::IDLE, "still IDLE immediately after Request() returns");

  RunFor(50, 50);  // consume the pending request -> S2 ARMED
  TEST_ASSERT(FifoDriver_GetPhase() == FifoPhase::ACTIVE, "ACTIVE once armed");

  RunFor(3000, 50);  // request send + 260ms anchor wait + streaming decode
                      // (bounded by FIFO_SERVICE_MAX_BYTES=512/tick, so
                      // ceil(6149/512)=13 ticks minimum once bytes arrive)

  TEST_ASSERT(FifoDriver_GetPhase() == FifoPhase::RESULT_READY, "reaches RESULT_READY");

  FifoCaptureResult result{};
  TEST_ASSERT(FifoDriver_TryAcquireResult(&result), "acquire succeeds");
  TEST_ASSERT(result.error == FifoError::NONE, "result reports success");
  TEST_ASSERT(result.status == FifoPhase::RESULT_READY, "status reflects RESULT_READY, not the zero-init default IDLE");
  TEST_ASSERT(result.captureId == handle, "captureId matches the handle Request() returned");
  TEST_ASSERT(result.sampleCount == 1024, "sampleCount == 1024");
  // [R-4.1] fifo_types.h: "FifoError::NONE required before x/y/z may be read".
  if (result.error == FifoError::NONE &&
      result.x != NULL && result.y != NULL && result.z != NULL) {
    TEST_ASSERT(result.x[0] == xVals[0] && result.x[1023] == xVals[1023], "x samples decode correctly");
    TEST_ASSERT(result.y[512] == yVals[512], "y samples decode correctly");
    TEST_ASSERT(result.z[100] == zVals[100], "z samples decode correctly");
  } else {
    TEST_ASSERT(false, "samples unreadable: error != NONE or x/y/z == NULL");
  }
  TEST_ASSERT(result.retryCount == 0, "no retries needed");
  // [R-3] Machine-state provenance copied verbatim from the request at the
  // provenance latch, exactly as srIndexAtCapture/srHz already were. Before
  // R-3 these three had no producer and stayed at their zero-init defaults.
  TEST_ASSERT(result.motorStateAtCapture == 2, "motorStateAtCapture copied from request (not zero-init 0)");
  TEST_ASSERT(result.rpmAtCapture > 1484.4f && result.rpmAtCapture < 1484.6f, "rpmAtCapture copied from request (not zero-init 0)");
  TEST_ASSERT(result.tempCAtCapture > 51.0f && result.tempCAtCapture < 51.2f, "tempCAtCapture copied from request (not zero-init 0)");

  FifoCaptureResult second{};
  TEST_ASSERT(!FifoDriver_TryAcquireResult(&second), "double-acquire rejected while first holder active");

  FifoDriver_ReleaseResult();
  // [R-4.1] Observe the TRANSITION into COOLDOWN rather than a dwell. With
  // T_COOLDOWN_MS = 0 (ADR-0006 D-3) the state is entered on the first tick
  // after release and left on the next, so it must be sampled per-tick.
  TEST_ASSERT(RunUntilPhase(FifoPhase::COOLDOWN, 10), "enters COOLDOWN after release");
  // Return to IDLE verified separately. maxTicks sized for the pre-ADR-0006
  // 60 s dwell too (60000/50 = 1200 ticks), so this assertion holds under
  // either cooldown policy.
  TEST_ASSERT(RunUntilPhase(FifoPhase::IDLE, 2000), "returns to IDLE after cooldown");

  TEST_END();
}

// ============================================================================
// 2. CRC failure -- session-level self-heal (SS17.1 "byte level RESYNC"),
//    never reaching the driver's retry-attempt policy at all: a
//    corrupted frame is rejected, Session Controller auto-re-arms to
//    WAIT_FRAME within the SAME attempt, and a subsequent valid frame in
//    the same response stream still succeeds.
// ============================================================================
static void Test_CrcFailureSelfHeals() {
  TEST_BEGIN("CrcFailureSelfHeals");
  ResetClock();

  static int16_t xVals[1024], yVals[1024], zVals[1024];
  SampleXYZ(11, xVals);
  SampleXYZ(12, yVals);
  SampleXYZ(13, zVals);

  std::vector<uint8_t> bytes;
  std::vector<uint32_t> timestamps;
  AppendSegment(bytes, timestamps, BuildCorruptedDumpFrame(xVals, yVals, zVals), 260);
  AppendSegment(bytes, timestamps, BuildDumpFrame(xVals, yVals, zVals), 260);  // same burst, next frame

  FifoTransport transport;
  LogReplayTransport_Init(&transport, bytes.data(), bytes.size(), timestamps.data(),
                           &g_simNowMs, &g_simOverflow);
  FifoDriver_Init(&transport);

  FifoCaptureRequest req = MakeAdmissibleRequest();
  uint32_t handle = 0;
  TEST_ASSERT(FifoDriver_Request(&req, &handle) == FifoError::NONE, "request admitted");

  RunFor(3200, 50);

  TEST_ASSERT(FifoDriver_GetPhase() == FifoPhase::RESULT_READY, "still reaches RESULT_READY despite the leading corrupt frame");

  FifoCaptureResult result{};
  TEST_ASSERT(FifoDriver_TryAcquireResult(&result), "acquire succeeds");
  TEST_ASSERT(result.error == FifoError::NONE, "final result is a success (self-healed)");
  TEST_ASSERT(result.crcErrorCount == 1, "exactly one CRC error counted");
  TEST_ASSERT(result.retryCount == 0, "self-heal never invoked driver-level retry");
  FifoDriver_ReleaseResult();

  TEST_END();
}

// ============================================================================
// 3. No response -- T_REQUEST_RESPONSE_MS (1000ms) pre-anchor silence.
// ============================================================================
static void Test_NoResponseTimeout() {
  TEST_BEGIN("NoResponseTimeout");
  ResetClock();

  std::vector<uint8_t> bytes;      // empty -- nothing ever arrives
  std::vector<uint32_t> timestamps;

  FifoTransport transport;
  LogReplayTransport_Init(&transport, bytes.data(), bytes.size(), timestamps.data(),
                           &g_simNowMs, &g_simOverflow);
  FifoDriver_Init(&transport);

  FifoCaptureRequest req = MakeAdmissibleRequest();
  uint32_t handle = 0;
  TEST_ASSERT(FifoDriver_Request(&req, &handle) == FifoError::NONE, "request admitted");

  RunFor(100, 50);  // consume request (tick 1: S1->S2), arm+send (tick 2: S2->S3,
                     // stamps the stall clock) -- 2 ticks, not 1; see this test's
                     // own history for why margins below are kept generous rather
                     // than boundary-precise

  RunFor(700, 100);  // comfortably under the 1000ms budget from either
                      // plausible stamp tick
  TEST_ASSERT(FifoDriver_GetPhase() == FifoPhase::ACTIVE, "still ACTIVE comfortably under T_REQUEST_RESPONSE_MS");

  RunFor(600, 100);  // comfortably cross the 1000ms budget regardless of
                      // which exact tick the clock was stamped on
  // Retries follow automatically (retryCount 0 < FIFO_MAX_RETRIES=2) --
  // drain, one-tick idle, re-arm, re-request -- all into the same empty
  // stream, so this repeats twice more before the session finalizes as a
  // failure. Budget generously for 3 full attempts worth of drain+wait+timeout.
  RunFor(3 * 1600, 100);

  TEST_ASSERT(FifoDriver_GetPhase() == FifoPhase::RESULT_READY, "session finalizes (retries exhausted)");
  FifoCaptureResult result{};
  TEST_ASSERT(FifoDriver_TryAcquireResult(&result), "acquire succeeds");
  TEST_ASSERT(result.error == FifoError::ERR_NO_RESPONSE, "final error is ERR_NO_RESPONSE");
  TEST_ASSERT(result.retryCount == 2, "FIFO_MAX_RETRIES (2) retries were attempted");
  FifoDriver_ReleaseResult();

  TEST_END();
}

// ============================================================================
// 4. Inter-byte timeout -- T_INTER_BYTE_MS (500ms) mid-frame stall.
//    Anchor + type byte arrive, then silence.
// ============================================================================
static void Test_InterByteTimeout() {
  TEST_BEGIN("InterByteTimeout");
  ResetClock();

  std::vector<uint8_t> bytes;
  std::vector<uint32_t> timestamps;
  AppendSegment(bytes, timestamps, {0x50, 0x03, 0x00}, 260);  // anchor + FULL_DUMP type, then nothing

  FifoTransport transport;
  LogReplayTransport_Init(&transport, bytes.data(), bytes.size(), timestamps.data(),
                           &g_simNowMs, &g_simOverflow);
  FifoDriver_Init(&transport);

  FifoCaptureRequest req = MakeAdmissibleRequest();
  req.admissionContext.motorStable = true;
  uint32_t handle = 0;
  TEST_ASSERT(FifoDriver_Request(&req, &handle) == FifoError::NONE, "request admitted");

  RunFor(500, 50);   // reach t=260+, consume anchor+type -> RECEIVING
  TEST_ASSERT(FifoDriver_GetPhase() == FifoPhase::ACTIVE, "receiving, mid-frame");

  RunFor(300, 50);   // comfortably under the 500ms inter-byte budget from
                      // whichever tick actually last saw activity
  TEST_ASSERT(FifoDriver_GetPhase() == FifoPhase::ACTIVE, "still receiving, not yet timed out");

  RunFor(600, 100);  // comfortably cross the 500ms budget

  // As with no-response, this retries twice more before finalizing.
  RunFor(3 * 1600, 100);

  TEST_ASSERT(FifoDriver_GetPhase() == FifoPhase::RESULT_READY, "session finalizes (retries exhausted)");
  FifoCaptureResult result{};
  TEST_ASSERT(FifoDriver_TryAcquireResult(&result), "acquire succeeds");
  TEST_ASSERT(result.error == FifoError::ERR_INTER_BYTE_TIMEOUT, "final error is ERR_INTER_BYTE_TIMEOUT");
  FifoDriver_ReleaseResult();

  TEST_END();
}

// ============================================================================
// 5 & 6. Retry (successful) and Retry exhausted -- SS17.1 "Attempt level
//    -> RETRY_ATTEMPT", driven by BAD_TYPE_BYTE (SESSION_FAILED).
// ============================================================================
static void Test_RetrySucceeds() {
  TEST_BEGIN("RetrySucceeds");
  ResetClock();

  static int16_t xVals[1024], yVals[1024], zVals[1024];
  SampleXYZ(21, xVals);
  SampleXYZ(22, yVals);
  SampleXYZ(23, zVals);

  std::vector<uint8_t> bytes;
  std::vector<uint32_t> timestamps;
  // Attempt 1 (t=260): bad type byte -> BAD_TYPE_BYTE -> SESSION_FAILED.
  AppendSegment(bytes, timestamps, BuildBadTypeByteFrame(), 260);
  // [R-4.1] Attempt 2's valid dump, placed INSIDE attempt 2's actual response
  // window. Measured attempt boundaries (50 ms ticks): attempt 1 t=50..700,
  // attempt 2 t=750..2250, attempt 3 t=2300..3800 -- the whole session is
  // terminal by t=3800. The previous t=5000 placement was therefore never
  // reachable: attempts 2 and 3 both hit T_REQUEST_RESPONSE_MS (1000 ms) and
  // the session ended ERR_NO_RESPONSE before this segment was ever replayed.
  AppendSegment(bytes, timestamps, BuildDumpFrame(xVals, yVals, zVals), 1000);

  FifoTransport transport;
  LogReplayTransport_Init(&transport, bytes.data(), bytes.size(), timestamps.data(),
                           &g_simNowMs, &g_simOverflow);
  FifoDriver_Init(&transport);

  FifoCaptureRequest req = MakeAdmissibleRequest();
  uint32_t handle = 0;
  TEST_ASSERT(FifoDriver_Request(&req, &handle) == FifoError::NONE, "request admitted");

  RunFor(8000, 50);  // generous budget for attempt 1 (fail+drain+idle+
                      // re-request) plus attempt 2 (anchor wait+decode)

  TEST_ASSERT(FifoDriver_GetPhase() == FifoPhase::RESULT_READY, "reaches RESULT_READY after one retry");
  FifoCaptureResult result{};
  TEST_ASSERT(FifoDriver_TryAcquireResult(&result), "acquire succeeds");
  TEST_ASSERT(result.error == FifoError::NONE, "second attempt succeeded");
  TEST_ASSERT(result.retryCount == 1, "exactly one retry was needed");
  TEST_ASSERT(result.sampleCount == 1024, "full dump decoded on the successful attempt");
  TEST_ASSERT(result.captureId == handle, "captureId preserved across the retry (same logical capture)");
  // [R-4.1] fifo_types.h: "FifoError::NONE required before x/y/z may be read".
  // Guarded so a future failure reports the real cause instead of dereferencing
  // a stale arena pointer (which previously yielded a misleading x[0] = -999).
  if (result.error == FifoError::NONE && result.x != NULL) {
    TEST_ASSERT(result.x[0] == xVals[0], "samples from the SECOND (successful) attempt are what's reported");
  } else {
    TEST_ASSERT(false, "samples unreadable: error != NONE or x == NULL");
  }
  FifoDriver_ReleaseResult();

  TEST_END();
}

static void Test_RetryExhausted() {
  TEST_BEGIN("RetryExhausted");
  ResetClock();

  std::vector<uint8_t> bytes;
  std::vector<uint32_t> timestamps;
  // [R-4.1] Three attempts (initial + FIFO_MAX_RETRIES=2), one bad frame inside
  // EACH attempt's actual response window. Measured boundaries (50 ms ticks):
  // attempt 1 t=50..700, attempt 2 t=750..2250, attempt 3 t=2300..3800. The
  // previous 260/5000/10000 spacing put segments 2 and 3 past the session's own
  // terminal point (t=3800), so attempts 2 and 3 timed out ERR_NO_RESPONSE and
  // the terminal error was never the ERR_BAD_TYPE_BYTE this test asserts.
  AppendSegment(bytes, timestamps, BuildBadTypeByteFrame(), 260);
  AppendSegment(bytes, timestamps, BuildBadTypeByteFrame(), 1000);
  AppendSegment(bytes, timestamps, BuildBadTypeByteFrame(), 2500);

  FifoTransport transport;
  LogReplayTransport_Init(&transport, bytes.data(), bytes.size(), timestamps.data(),
                           &g_simNowMs, &g_simOverflow);
  FifoDriver_Init(&transport);

  FifoCaptureRequest req = MakeAdmissibleRequest();
  uint32_t handle = 0;
  TEST_ASSERT(FifoDriver_Request(&req, &handle) == FifoError::NONE, "request admitted");

  RunFor(12000, 50);

  TEST_ASSERT(FifoDriver_GetPhase() == FifoPhase::RESULT_READY, "session finalizes after exhausting retries");
  FifoCaptureResult result{};
  TEST_ASSERT(FifoDriver_TryAcquireResult(&result), "acquire succeeds");
  TEST_ASSERT(result.error == FifoError::ERR_BAD_TYPE_BYTE, "final error preserved from the last attempt");
  TEST_ASSERT(result.status == FifoPhase::RESULT_READY, "status reflects RESULT_READY even on a failure outcome");
  TEST_ASSERT(result.retryCount == 2, "FIFO_MAX_RETRIES (2) retries were attempted, then stopped");
  FifoDriver_ReleaseResult();

  TEST_END();
}

// ============================================================================
// 7. Abort -- explicit termination mid-capture, no retry, drain still
//    happens.
// ============================================================================
static void Test_Abort() {
  TEST_BEGIN("Abort");
  ResetClock();

  std::vector<uint8_t> bytes;       // driver will be waiting; nothing arrives
  std::vector<uint32_t> timestamps;

  FifoTransport transport;
  LogReplayTransport_Init(&transport, bytes.data(), bytes.size(), timestamps.data(),
                           &g_simNowMs, &g_simOverflow);
  FifoDriver_Init(&transport);

  FifoCaptureRequest req = MakeAdmissibleRequest();
  uint32_t handle = 0;
  TEST_ASSERT(FifoDriver_Request(&req, &handle) == FifoError::NONE, "request admitted");

  RunFor(100, 50);  // reach S4, waiting for an anchor that will never come
  TEST_ASSERT(FifoDriver_GetPhase() == FifoPhase::ACTIVE, "capture in progress");

  FifoDriver_Abort(FifoError::ERR_ABORTED);

  RunFor(600, 50);  // S13 -> S10 (drain, quiet immediately since nothing arrives) -> S11

  TEST_ASSERT(FifoDriver_GetPhase() == FifoPhase::RESULT_READY, "aborted capture still reaches RESULT_READY (drain performed)");
  FifoCaptureResult result{};
  TEST_ASSERT(FifoDriver_TryAcquireResult(&result), "acquire succeeds");
  TEST_ASSERT(result.error == FifoError::ERR_ABORTED, "result reports ERR_ABORTED");
  TEST_ASSERT(result.status == FifoPhase::RESULT_READY, "status reflects RESULT_READY even for an aborted capture (routes through the same S10->S11 transition)");
  TEST_ASSERT(result.retryCount == 0, "abort does not trigger a retry");
  FifoDriver_ReleaseResult();

  // Abort with nothing in progress is a no-op.
  RunFor(62000, 500);  // finish cooldown, reach IDLE
  TEST_ASSERT(FifoDriver_GetPhase() == FifoPhase::IDLE, "back to idle");
  FifoDriver_Abort(FifoError::ERR_ABORTED);
  TEST_ASSERT(FifoDriver_GetPhase() == FifoPhase::IDLE, "Abort() with no capture in progress is a no-op");

  TEST_END();
}

// ============================================================================
// 8. Circuit breaker -- FIFO_BREAKER_THRESHOLD (5) consecutive failed
//    SESSIONS trips S14 DISABLED; ERR_CIRCUIT_OPEN thereafter.
// ============================================================================
static void RunOneFailingSessionToIdle(uint32_t requestFrameAtMs) {
  std::vector<uint8_t> bytes;
  std::vector<uint32_t> timestamps;
  AppendSegment(bytes, timestamps, BuildBadTypeByteFrame(), requestFrameAtMs);
  AppendSegment(bytes, timestamps, BuildBadTypeByteFrame(), requestFrameAtMs + 5000);
  AppendSegment(bytes, timestamps, BuildBadTypeByteFrame(), requestFrameAtMs + 10000);

  FifoTransport transport;
  LogReplayTransport_Init(&transport, bytes.data(), bytes.size(), timestamps.data(),
                           &g_simNowMs, &g_simOverflow);
  // Re-binds the transport singleton only -- FifoDriver's own state is
  // deliberately NOT re-initialized here, so s_consecutiveFailedSessions
  // (and s_captureIdCounter) persist across calls, matching how these
  // sessions are meant to run back-to-back within one boot.

  FifoCaptureRequest req = MakeAdmissibleRequest();
  uint32_t handle = 0;
  FifoError verdict = FifoDriver_Request(&req, &handle);
  TEST_ASSERT(verdict == FifoError::NONE, "each of the 5 sessions is itself admitted normally");

  RunFor(12000, 50);
  FifoCaptureResult result{};
  if (FifoDriver_TryAcquireResult(&result)) {
    FifoDriver_ReleaseResult();
  }
  RunFor(62000, 500);  // cooldown (or straight to DISABLED on the 5th)
}

static void Test_CircuitBreaker() {
  TEST_BEGIN("CircuitBreaker");
  ResetClock();

  // Bind an initial (unused) transport just to call Init() once.
  std::vector<uint8_t> empty;
  std::vector<uint32_t> emptyTs;
  FifoTransport transport;
  LogReplayTransport_Init(&transport, empty.data(), empty.size(), emptyTs.data(),
                           &g_simNowMs, &g_simOverflow);
  FifoDriver_Init(&transport);

  for (int session = 0; session < 5; session++) {
    RunOneFailingSessionToIdle(g_simNowMs + 260);
  }

  TEST_ASSERT(FifoDriver_GetPhase() == FifoPhase::BREAKER_DISABLED,
              "breaker trips to BREAKER_DISABLED after 5 consecutive failed sessions");

  FifoCaptureRequest req = MakeAdmissibleRequest();
  uint32_t handle = 0;
  FifoError verdict = FifoDriver_Request(&req, &handle);
  TEST_ASSERT(verdict == FifoError::ERR_CIRCUIT_OPEN, "requests rejected with ERR_CIRCUIT_OPEN while disabled");

  TEST_END();
}

// ============================================================================
// 9. Reset circuit breaker -- the only way S14 exits.
// ============================================================================
static void Test_ResetCircuitBreaker() {
  TEST_BEGIN("ResetCircuitBreaker");
  ResetClock();

  std::vector<uint8_t> empty;
  std::vector<uint32_t> emptyTs;
  FifoTransport transport;
  LogReplayTransport_Init(&transport, empty.data(), empty.size(), emptyTs.data(),
                           &g_simNowMs, &g_simOverflow);
  FifoDriver_Init(&transport);

  for (int session = 0; session < 5; session++) {
    RunOneFailingSessionToIdle(g_simNowMs + 260);
  }
  TEST_ASSERT(FifoDriver_GetPhase() == FifoPhase::BREAKER_DISABLED, "breaker tripped (precondition for this test)");

  bool didReset = FifoDriver_ResetCircuitBreaker();
  TEST_ASSERT(didReset, "reset succeeds while disabled");
  TEST_ASSERT(FifoDriver_GetPhase() == FifoPhase::IDLE, "returns to IDLE");

  FifoCaptureRequest req = MakeAdmissibleRequest();
  uint32_t handle = 0;
  TEST_ASSERT(FifoDriver_Request(&req, &handle) == FifoError::NONE,
              "a request is admitted normally again after reset");

  bool secondReset = FifoDriver_ResetCircuitBreaker();
  TEST_ASSERT(!secondReset, "resetting again when not disabled is a no-op, returns false");

  TEST_END();
}

// ============================================================================
// 10. Watchdog reclaim -- SS17.5, held > T_RESULT_HOLD_MAX_MS (30 s).
//
//     [SLOW / NOT RUN BY DEFAULT] FifoArena_HeldSinceMs() reads
//     std::chrono::steady_clock directly (Task 3.3A's disclosed
//     compromise) -- it is NOT driven by g_simNowMs/LogReplayTransport's
//     injected clock, so this test genuinely sleeps ~30 real seconds. It
//     is written for completeness and correctness review, but is
//     excluded from RunFastSuite() below. See the delivered Integration
//     Report's "Remaining architectural risks" for the recommended fix
//     (inject a test-controllable clock into FifoArena, in a future,
//     separately-reviewed change).
// ============================================================================
static void Test_WatchdogReclaim_SLOW() {
  TEST_BEGIN("WatchdogReclaim_SLOW");
  ResetClock();

  static int16_t xVals[1024], yVals[1024], zVals[1024];
  SampleXYZ(31, xVals);
  SampleXYZ(32, yVals);
  SampleXYZ(33, zVals);

  std::vector<uint8_t> bytes;
  std::vector<uint32_t> timestamps;
  AppendSegment(bytes, timestamps, BuildDumpFrame(xVals, yVals, zVals), 260);

  FifoTransport transport;
  LogReplayTransport_Init(&transport, bytes.data(), bytes.size(), timestamps.data(),
                           &g_simNowMs, &g_simOverflow);
  FifoDriver_Init(&transport);

  FifoCaptureRequest req = MakeAdmissibleRequest();
  uint32_t handle = 0;
  TEST_ASSERT(FifoDriver_Request(&req, &handle) == FifoError::NONE, "request admitted");
  RunFor(3000, 50);
  TEST_ASSERT(FifoDriver_GetPhase() == FifoPhase::RESULT_READY, "reaches RESULT_READY");

  FifoCaptureResult result{};
  TEST_ASSERT(FifoDriver_TryAcquireResult(&result), "acquire succeeds");
  // Deliberately never call FifoDriver_ReleaseResult().

  std::this_thread::sleep_for(std::chrono::seconds(31));  // > T_RESULT_HOLD_MAX_MS (30000ms), REAL time
  RunFor(500, 50);  // let the driver observe the reclaim on a real tick

  TEST_ASSERT(FifoDriver_GetPhase() == FifoPhase::IDLE,
              "watchdog force-reclaims and returns DIRECTLY to IDLE (SS17.5's literal text, bypassing COOLDOWN)");
  TEST_ASSERT(!FifoArena_IsOwned(), "arena ownership was force-reclaimed");

  FifoCaptureRequest req2 = MakeAdmissibleRequest();
  uint32_t handle2 = 0;
  TEST_ASSERT(FifoDriver_Request(&req2, &handle2) == FifoError::NONE,
              "a new request is admitted immediately (no cooldown wait) -- see the gate-3 "
              "'cooldown elapsed' discussion in the Integration Report for why this is "
              "arguably a SECOND finding worth a closer look, not just SS17.5's own tension");

  TEST_END();
}

// ----------------------------------------------------------------------------
// main() -- runs the fast, deterministic suite by default. The slow
// watchdog test is available but not included in the default run.
// ----------------------------------------------------------------------------
static void RunFastSuite() {
  Test_EndToEndSuccess();
  Test_CrcFailureSelfHeals();
  Test_NoResponseTimeout();
  Test_InterByteTimeout();
  Test_RetrySucceeds();
  Test_RetryExhausted();
  Test_Abort();
  Test_CircuitBreaker();
  Test_ResetCircuitBreaker();
}

int main(int argc, char** argv) {
  bool runSlow = (argc > 1 && std::strcmp(argv[1], "--slow") == 0);

  RunFastSuite();
  if (runSlow) {
    Test_WatchdogReclaim_SLOW();
  } else {
    std::printf("--- WatchdogReclaim_SLOW ---\n  SKIPPED (pass --slow to run; takes ~31 real seconds)\n");
  }

  std::printf("\n%d tests run, %d failed.\n", g_testsRun, g_testsFailed);
  return g_testsFailed == 0 ? 0 : 1;
}
