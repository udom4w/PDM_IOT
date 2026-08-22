// ============================================================================
// [R-2] test_vib_ema.cpp -- host-only tests for VibEma freshness semantics
//
// HOST-ONLY -- lives under test/, matching log_replay_transport.h's own
// convention, never compiled into the firmware.
//
// Hand-rolled assertion harness, no external test-framework dependency --
// matches test_fifo_driver.cpp's zero-extra-dependency philosophy.
//
// Build + run (WSL / any host g++):
//   g++ -std=c++17 -I.. -o test_vib_ema test_vib_ema.cpp ../vib_ema.cpp ../vib_history.cpp
//   ./test_vib_ema
//
// Covers the R-2 contract: velocity_ema_valid means "current usable EMA"
// (seeded AND fresh), not merely "seeded".
// ============================================================================

#include "../vib_ema.h"
#include "../vib_history.h"

#include <cstdio>
#include <cmath>

static int g_checks = 0;
static int g_failed = 0;

static void check(bool cond, const char* what) {
  g_checks++;
  if (!cond) {
    g_failed++;
    std::printf("  FAIL: %s\n", what);
  } else {
    std::printf("  ok  : %s\n", what);
  }
}

static void checkNear(float got, float want, const char* what) {
  check(std::fabs(got - want) < 1e-4f, what);
  if (std::fabs(got - want) >= 1e-4f) {
    std::printf("        got=%.6f want=%.6f\n", got, want);
  }
}

static const uint32_t MAX_AGE = 10000u;  // mirrors VIB_VELOCITY_MAX_AGE_MS_TBD

static void freshState(void) {
  VibHistory_Init();
  VibEma_Init();
}

// ---------------------------------------------------------------------------
// 1. never seeded -> valid=false regardless of freshness
// ---------------------------------------------------------------------------
static void test_never_seeded(void) {
  std::printf("[1] never seeded\n");
  freshState();
  VibEmaState s = VibEma_Get(0u, MAX_AGE);
  check(!s.valid, "valid=false before any sample");
  checkNear(s.ema_mms, 0.0f, "ema_mms=0 before any sample");
}

// ---------------------------------------------------------------------------
// 2. seeded + fresh -> valid=true
// ---------------------------------------------------------------------------
static void test_seeded_fresh(void) {
  std::printf("[2] seeded + fresh\n");
  freshState();
  VibHistory_Append(1.500f, 1000u, 1u);
  VibEma_Update();

  VibEmaState s = VibEma_Get(1000u, MAX_AGE);
  check(s.valid, "valid=true at age 0");
  checkNear(s.ema_mms, 1.500f, "seed takes the sample value exactly");

  s = VibEma_Get(1000u + MAX_AGE, MAX_AGE);
  check(s.valid, "valid=true at age == maxAgeMs (inclusive boundary)");
}

// ---------------------------------------------------------------------------
// 3. seeded + stale -> valid=false, value RETAINED (never zeroed)
// ---------------------------------------------------------------------------
static void test_seeded_stale(void) {
  std::printf("[3] seeded + stale\n");
  freshState();
  VibHistory_Append(1.500f, 1000u, 1u);
  VibEma_Update();

  VibEmaState s = VibEma_Get(1000u + MAX_AGE + 1u, MAX_AGE);
  check(!s.valid, "valid=false one ms past maxAgeMs");
  checkNear(s.ema_mms, 1.500f, "stale ema_mms RETAINED, not zeroed");
  check(s.timestampMs == 1000u, "stale timestampMs retained");

  // The regression that motivated R-2: 565 s of no data must not read as valid.
  s = VibEma_Get(1000u + 565000u, MAX_AGE);
  check(!s.valid, "valid=false after 565 s outage (observed field case)");
  checkNear(s.ema_mms, 1.500f, "value still retained after long outage");
}

// ---------------------------------------------------------------------------
// 4. recovery: a new fresh sample restores valid=true
// ---------------------------------------------------------------------------
static void test_recovery(void) {
  std::printf("[4] recovery after staleness\n");
  freshState();
  VibHistory_Append(1.500f, 1000u, 1u);
  VibEma_Update();
  check(!VibEma_Get(60000u, MAX_AGE).valid, "stale before recovery");

  // New sample 20 s later -- inside VIB_EMA_RESET_GAP_MS (90 s), so this is a
  // DECAY step, not a reseed.
  VibHistory_Append(2.500f, 21000u, 2u);
  VibEma_Update();

  VibEmaState s = VibEma_Get(21000u, MAX_AGE);
  check(s.valid, "valid=true again after a fresh sample");
  check(!s.reseeded, "decay step, not a reseed (gap < VIB_EMA_RESET_GAP_MS)");
  check(s.ema_mms > 1.500f && s.ema_mms < 2.500f,
        "ema decayed toward the new sample rather than jumping to it");
  check(s.timestampMs == 21000u, "timestamp advanced to the new sample");
}

// ---------------------------------------------------------------------------
// 5. REGRESSION GUARD (requirement 6): reading a stale snapshot must not
//    corrupt the stored seeded flag, i.e. must not turn the next sample into
//    a spurious reseed.
// ---------------------------------------------------------------------------
static void test_stale_read_does_not_reseed(void) {
  std::printf("[5] stale read must not cause a spurious reseed\n");
  freshState();
  VibHistory_Append(1.500f, 1000u, 1u);
  VibEma_Update();

  // Read stale MANY times -- each call must leave the stored state untouched.
  for (int i = 0; i < 25; i++) {
    (void)VibEma_Get(1000u + 60000u + (uint32_t)i, MAX_AGE);
  }

  VibHistory_Append(2.500f, 21000u, 2u);
  VibEma_Update();

  VibEmaState s = VibEma_Get(21000u, MAX_AGE);
  check(!s.reseeded, "still a decay step after 25 stale reads");
  check(s.ema_mms < 2.500f, "value decayed, proving the seed flag survived");
}

// ---------------------------------------------------------------------------
// 6. reseed behaviour after a genuinely long gap is PRESERVED (requirement 6)
// ---------------------------------------------------------------------------
static void test_long_gap_still_reseeds(void) {
  std::printf("[6] long gap still reseeds\n");
  freshState();
  VibHistory_Append(1.500f, 1000u, 1u);
  VibEma_Update();
  (void)VibEma_Get(500000u, MAX_AGE);  // observed stale in between

  // > VIB_EMA_RESET_GAP_MS (90 s) since the last VALID sample -> reseed.
  VibHistory_Append(5.000f, 1000u + VIB_EMA_RESET_GAP_MS + 1000u, 2u);
  VibEma_Update();

  VibEmaState s = VibEma_Get(1000u + VIB_EMA_RESET_GAP_MS + 1000u, MAX_AGE);
  check(s.valid, "valid=true after reseed");
  check(s.reseeded, "reseeded=true after a gap > VIB_EMA_RESET_GAP_MS");
  checkNear(s.ema_mms, 5.000f, "reseed takes the new sample value exactly");
}

// ---------------------------------------------------------------------------
// 7. gaps in the ring are not EMA samples and do not refresh validity
// ---------------------------------------------------------------------------
static void test_gap_does_not_refresh(void) {
  std::printf("[7] gap entries do not refresh validity\n");
  freshState();
  VibHistory_Append(1.500f, 1000u, 1u);
  VibEma_Update();

  VibHistory_AppendGap(30000u, 0u);
  VibEma_Update();

  VibEmaState s = VibEma_Get(30000u, MAX_AGE);
  check(!s.valid, "gap did not make a stale EMA look fresh");
  check(s.timestampMs == 1000u, "timestamp still that of the last VALID sample");
}

// ---------------------------------------------------------------------------
// 8. unsigned wrap-around across the millis() rollover
// ---------------------------------------------------------------------------
static void test_wrap_around(void) {
  std::printf("[8] millis() wrap-around\n");
  freshState();
  const uint32_t nearMax = 0xFFFFFF00u;
  VibHistory_Append(1.500f, nearMax, 1u);
  VibEma_Update();

  // now has wrapped past 0; true elapsed = 0x200 (512 ms) -> still fresh.
  VibEmaState s = VibEma_Get(0x00000100u, MAX_AGE);
  check(s.valid, "valid=true across wrap when true age < maxAgeMs");

  // true elapsed = 0x100 + 20000 -> stale.
  s = VibEma_Get(0x00000100u + 20000u, MAX_AGE);
  check(!s.valid, "valid=false across wrap when true age > maxAgeMs");
}

int main(void) {
  std::printf("=== [R-2] VibEma freshness tests ===\n");
  test_never_seeded();
  test_seeded_fresh();
  test_seeded_stale();
  test_recovery();
  test_stale_read_does_not_reseed();
  test_long_gap_still_reseeds();
  test_gap_does_not_refresh();
  test_wrap_around();
  std::printf("=== %d checks, %d failed ===\n", g_checks, g_failed);
  return g_failed == 0 ? 0 : 1;
}
