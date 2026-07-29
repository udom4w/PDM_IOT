// ============================================================================
// [v16.6.10-fifo] fifo_arena.cpp
//
// See fifo_arena.h for the public contract, scope note, ownership model,
// and thread-model justification. [Task 3.3A] Now implements all eight
// frozen-SDS entry points (Init/WriteHandleX/Y/Z/TryAcquire/Release/
// IsOwned/HeldSinceMs) -- still no retry/protocol/transport/session/
// driver-state-machine logic of any kind. No dynamic allocation anywhere
// in this file.
// ============================================================================

#include "fifo_arena.h"
#include <chrono>

// ----------------------------------------------------------------------------
// Single static storage (SDS SS9/SS10.1's own budget row: "Sample arena
// x[1024]+y[1024]+z[1024] (int16) | 6144 B | .bss, static"). 1024 samples
// x 3 axes x 2 bytes (int16_t) = 6144 bytes total, matching K-4/K-8's
// payload size exactly. File-static (internal linkage) -- nothing outside
// this translation unit can name these arrays; every access from outside
// this file goes through TryAcquire()'s published const pointers only.
// ----------------------------------------------------------------------------
static int16_t s_x[1024];
static int16_t s_y[1024];
static int16_t s_z[1024];

// ----------------------------------------------------------------------------
// Ownership state. `volatile` because FifoArena_IsOwned() may be read from
// a different task/core than whichever single call site writes it --
// matching CLAUDE.md's cross-core rule that a single-word flag with
// exactly one writer is safe to read across cores without a mutex (unlike
// a multi-writer boolean, which CLAUDE.md explicitly forbids). See
// fifo_arena.h's Thread model note for the single-call-site assumption
// this relies on.
// ----------------------------------------------------------------------------
static volatile bool s_held = false;

// ----------------------------------------------------------------------------
// [Task 3.3A] Timestamp of the current hold's start, milliseconds, per
// NowMsInternal() below. `volatile` for the same reason s_held is: written
// once (inside TryAcquire()'s success path) by whatever single call site
// the frozen SDS names for the consumer side, read from FifoArena_HeldSinceMs()
// which may be called from a different task/core (SDS SS9.1's
// "exactly one call site" guarantee covers the writer; it says nothing
// about the reader, so this is treated with the same cross-core caution
// as s_held). Meaningless while !s_held; FifoArena_HeldSinceMs() returns 0
// in that case rather than reading this value.
// ----------------------------------------------------------------------------
static volatile uint32_t s_acquiredAtMs = 0;

// ----------------------------------------------------------------------------
// NowMsInternal() -- [Task 3.3A] the one live-clock read in this entire
// driver family; see FifoArena_HeldSinceMs()'s doc comment in
// fifo_arena.h for why it exists and why it could not be injected like
// every other timing value in this codebase. std::chrono::steady_clock is
// pure ISO C++ (not Arduino's millis()) specifically so this file's
// "compiles standalone under a plain C++ compiler" claim stays true; it is
// monotonic non-decreasing, matching the same guarantee FifoTransport's
// own nowMs() contract already requires of its callers.
// ----------------------------------------------------------------------------
static uint32_t NowMsInternal() {
  using namespace std::chrono;
  return static_cast<uint32_t>(
      duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
}

void FifoArena_Init() {
  s_held = false;
  s_acquiredAtMs = 0;
}

int16_t* FifoArena_WriteHandleX() {
  return s_x;
}

int16_t* FifoArena_WriteHandleY() {
  return s_y;
}

int16_t* FifoArena_WriteHandleZ() {
  return s_z;
}

bool FifoArena_TryAcquire(const int16_t** outX, const int16_t** outY, const int16_t** outZ) {
  if (s_held) {
    // Double-acquire guard: reject outright, never publish a second set
    // of pointers while the first holder is still active.
    *outX = nullptr;
    *outY = nullptr;
    *outZ = nullptr;
    return false;
  }

  s_held = true;
  s_acquiredAtMs = NowMsInternal();  // [Task 3.3A] starts the HeldSinceMs() clock
  *outX = s_x;
  *outY = s_y;
  *outZ = s_z;
  return true;
}

void FifoArena_Release() {
  // Release-without-ownership is a safe no-op -- s_held is already false,
  // setting it to false again changes nothing and cannot desynchronize a
  // future TryAcquire()'s correctness.
  s_held = false;
}

bool FifoArena_IsOwned() {
  return s_held;
}

uint32_t FifoArena_HeldSinceMs() {
  if (!s_held) {
    return 0;
  }
  // Unsigned subtraction wraps correctly across a NowMsInternal() rollover
  // as long as true elapsed time stays under ~49.7 days -- the same
  // assumption every millis()-style duration calculation elsewhere in this
  // codebase already relies on, and SS17.5's T_RESULT_HOLD_MAX_MS (30 s)
  // is far inside that bound.
  return NowMsInternal() - s_acquiredAtMs;
}
