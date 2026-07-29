#pragma once
// ============================================================================
// [v16.6.10-fifo] fifo_arena.h
//
// L3 -- Sample Arena (Implementation Plan v1.4, Task 3.3; originally
// specified as Task 3.2 under Implementation Plan v1.0/v1.3, renumbered by
// ADR-0004's Session Controller promotion -- see
// CM-100_ADR-0004_Session_Controller_Promotion_v1.0.md and
// CM-100_FIFO_Implementation_Plan_v1.4_Delta.md SS2. Renumbered only; the
// arena's own design (SDS D-10, SS9) is unchanged by that ADR.
//
// The single 6144B static arena and the strict ownership-transfer protocol
// that makes PRR C-2 (stale data survives a failed capture) structurally
// impossible (SDS D-10, SS9.2).
//
// [SCOPE NOTE -- Task 3.3A] Task 3.3 originally shipped only
// Init/TryAcquire/Release/IsOwned, deliberately deferring two frozen-SDS
// functions. Task 3.3A (this revision) closes that gap by adding
// FifoArena_WriteHandleX()/Y()/Z() and FifoArena_HeldSinceMs() -- see each
// declaration below for how it maps to the frozen SDS. All eight functions
// the frozen Task 3.2/3.3 table specifies now exist. Two names still
// deliberately differ from the frozen literal text (FifoArena_Init() vs.
// the frozen FifoArena_Reset(); FifoArena_IsOwned() vs. the frozen
// FifoArena_IsHeld()) -- disclosed and justified in Task 3.3's own
// delivery, unchanged and not revisited here per "do not change existing
// APIs."
//
// Zero Arduino/ESP32/FreeRTOS dependency, matching every other L1-L3 file
// in this driver. Depends on nothing else in the FIFO driver -- no
// fifo_transport.h, fifo_codec.h, fifo_session.h, or fifo_types.h include
// -- this file is a leaf, exactly like fifo_types.h.
//
// Ownership model (SDS SS9.1, D-10): the frozen phase table is
//   IDLE(driver,none) -> CAPTURING(driver,none,driver-only-mutable) ->
//   RESULT_RDY(driver,frozen,none-until-acquire) -> HELD(consumer,
//   read-only) -> back to driver on release.
// From THIS file's perspective, the first three rows are indistinguishable
// -- all three are "not held by a consumer" -- and collapsing them into
// one unowned state is correct: distinguishing IDLE from CAPTURING from
// RESULT_RDY is the driver state machine's job (Task 3.4's internal
// FifoState S0..S14), never this file's. This file implements exactly the
// outer boundary: unowned <-> HELD.
//
// Thread model (SDS D-16, single-writer by construction): SS9.1
// [v1.1, Freeze Review Finding 2.1] names "exactly one call site" for the
// consumer side -- handleFifoCaptureCompletion() is the sole function that
// calls TryAcquireResult()/ReleaseResult(). This file's TryAcquire()/
// Release() inherit that same single-call-site guarantee, which is what
// makes a single, non-atomic `volatile bool` ownership flag sufficient
// here without a mutex. SDS SS7.1 A-6 attributes "the only new
// synchronization primitive in the entire driver" (mutexFifoResult) to the
// FifoDriver public-API layer (Task 3.6), not to this file -- this file
// deliberately does not add a second one. This is a load-bearing
// assumption: if a future task ever gives TryAcquire()/Release() more than
// one call site, this design must be revisited (CLAUDE.md's cross-core
// rule: a single-word volatile flag is safe with exactly one writer; it
// stops being safe the moment a second writer exists).
//
// [Task 3.3A] WriteHandleX()/Y()/Z() add no new cross-core concern: per
// SDS SS9.1's phase table, they are callable only during CAPTURING, which
// is exclusively driver-owned (D-1: FifoDriver_Service(), taskModbusRead,
// Core 0 only) -- a single caller, same as every other L2/L3 byte-level
// entry point in this driver. HeldSinceMs() DOES introduce one new,
// disclosed exception to this file's "no live clock" posture: see its own
// doc comment below.
// ============================================================================

#include <stdint.h>

// FifoArena_Init() -- initialize the arena to its empty, unowned state.
//   Precondition:  none.
//   Postcondition: ownership state is unowned (FifoArena_IsOwned() ==
//                  false). Storage contents are unspecified -- static
//                  storage duration already zero-initializes the
//                  underlying arrays before this ever runs; this function
//                  does not additionally re-zero them, since ownership
//                  gating (not byte content) is this arena's actual
//                  validity mechanism (SDS D-10, SS9.2: "there is no path
//                  to the samples that does not pass through a
//                  status == OK result").
//   Intended to be called once, at boot (matching FifoDriver_Init()'s own
//   "once per boot" convention, SDS SS7.1 A-1) -- but idempotent: calling
//   it again simply resets ownership state to unowned, safely.
void FifoArena_Init();

// FifoArena_WriteHandleX()/Y()/Z() -- [Task 3.3A] driver-internal write
// access to the arena's per-axis storage, exactly as the frozen SDS names
// them: "driver-internal write access, only during S8" (Task 3.2/3.3's
// public interface table). This is the destination D-11's streaming
// decode writes into -- FrameCodec_Step()'s xOut/yOut/zOut parameters are
// meant to be bound to these three pointers by the caller (Task 3.4's
// core state machine), stride-by-stride, as each 6-byte payload chunk
// completes. Closing this gap is what makes the arena actually usable for
// capture; Task 3.3 shipped without it, deliberately, per that task's own
// scope.
//   Precondition:  callable only while the arena is driver-owned and not
//                  currently CAPTURING-complete-and-consumer-HELD (SDS
//                  SS9.1's phase table: WriteHandle access is valid only
//                  during CAPTURING, never during HELD). This file does
//                  NOT enforce that precondition -- doing so would require
//                  knowing which internal S0..S14 state the caller is in,
//                  which is the driver state machine's own knowledge
//                  (Task 3.4), never this file's (matches the same
//                  trust-the-precondition convention already used by
//                  FrameCodecState_Reset() and FifoSession_StartAttempt()
//                  elsewhere in this driver). Callable unconditionally;
//                  correctness of *when* to call it is the caller's
//                  responsibility.
//   Postcondition: returns a non-const pointer to this file's own static,
//                  1024-int16_t per-axis storage. The pointer value never
//                  changes across calls (fixed static address) -- this
//                  function is a pure accessor, not a factory; it neither
//                  allocates nor mutates ownership state.
//   Thread model:  single caller by construction (FifoDriver_Service(),
//                  Core 0/taskModbusRead only, per D-1) -- see the file
//                  header's Thread model note.
int16_t* FifoArena_WriteHandleX();
int16_t* FifoArena_WriteHandleY();
int16_t* FifoArena_WriteHandleZ();

// FifoArena_TryAcquire() -- attempt to acquire exclusive read access to the
// arena's current contents.
//   Precondition:  `outX`, `outY`, `outZ` are valid, non-null,
//                  caller-owned pointers.
//   Postcondition: if the arena was unowned, ownership transfers to the
//                  caller (FifoArena_IsOwned() becomes true), *outX/*outY/
//                  *outZ are set to point at the arena's static per-axis
//                  storage (1024 int16_t samples each, read-only from the
//                  caller's side -- SDS SS9.1: "Zero copies. The consumer
//                  reads the arena in place"), and this function returns
//                  true. If the arena was already owned, this call is
//                  rejected -- *outX/*outY/*outZ are set to nullptr,
//                  ownership is unchanged, and this function returns
//                  false. This is the double-acquire guard: a second
//                  concurrent TryAcquire() can never observe the first
//                  caller's pointers.
//   Ownership:     the returned pointers are non-owning views into this
//                  file's own static storage -- the caller never frees
//                  them, and they remain valid only until the caller's
//                  matching FifoArena_Release() call.
bool FifoArena_TryAcquire(const int16_t** outX, const int16_t** outY, const int16_t** outZ);

// FifoArena_Release() -- release a previously acquired exclusive hold.
//   Precondition:  none enforced -- see Postcondition for the
//                  release-without-ownership case.
//   Postcondition: if the arena was owned, ownership transfers back to
//                  unowned (FifoArena_IsOwned() becomes false), making the
//                  arena eligible for a future TryAcquire() again. If the
//                  arena was NOT owned, this call is a safe no-op --
//                  release-without-ownership can never corrupt the
//                  ownership invariant or affect a future TryAcquire()'s
//                  correctness. This function is `void` (matching the
//                  frozen SDS signature) -- a caller relying on this
//                  function to report misuse will not be notified; the
//                  guarantee it makes is safety, not diagnostics. A
//                  consumer that never calls this at all stalls all
//                  future captures -- an accepted tradeoff per SDS SS9.2,
//                  not something this function can detect or prevent.
void FifoArena_Release();

// FifoArena_IsOwned() -- query the arena's current ownership state.
//   Precondition:  none.
//   Postcondition: returns true if a caller currently holds the arena via
//                  an unreleased FifoArena_TryAcquire(), false otherwise.
//                  Safe to call from any task/core (single-word volatile
//                  read) -- see the file header's Thread model note.
bool FifoArena_IsOwned();

// FifoArena_HeldSinceMs() -- [Task 3.3A] milliseconds elapsed since the
// CURRENT hold began, exactly matching the frozen SDS's signature
// (`uint32_t FifoArena_HeldSinceMs();`, no parameters) and its stated
// purpose: "for the SS17.5 watchdog." SS17.5's result-hold watchdog
// (Task 3.5, not this file) compares this value against its own
// T_RESULT_HOLD_MAX_MS constant to convert SS9.2's accepted
// forgot-to-release tradeoff into "a logged, self-clearing event." This
// file has no knowledge of that threshold and performs no comparison,
// alarm, or retry of its own -- it only ever reports the raw elapsed
// value; policy lives entirely in Task 3.5.
//   Precondition:  none.
//   Postcondition: if the arena is currently owned (FifoArena_IsOwned() ==
//                  true), returns milliseconds elapsed since the
//                  TryAcquire() call that produced the current hold.
//                  Returns 0 if the arena is not currently owned -- "held
//                  since" is meaningless without an active hold, and 0
//                  can never spuriously trigger a threshold comparison in
//                  a future watchdog.
//   [DISCLOSED DEVIATION] Every other timing-relevant value anywhere in
//   this driver (L1's FifoTransport::nowMs(), consumed by L2's
//   FrameCodec_Step()) is INJECTED, specifically so host-side tests can
//   control and replay time (fifo_transport.h's own stated rationale:
//   "not a bare millis() call baked into L2/L3"). This function cannot
//   follow that pattern: its signature is frozen at zero parameters by
//   the SDS itself, and FifoArena's other frozen entry points
//   (TryAcquire(), Init()) are equally parameter-frozen by Task 3.3's
//   "do not change existing APIs" constraint, so there is no injection
//   point anywhere in this file's contract through which a caller-
//   controlled clock could reach it. This implementation therefore reads
//   `std::chrono::steady_clock` directly (see fifo_arena.cpp) -- pure
//   ISO C++, so this file's "zero Arduino/ESP32/FreeRTOS dependency" and
//   "compiles standalone under a plain C++ compiler" claims both remain
//   true, but this is nonetheless the one function in this entire driver
//   that is NOT deterministically controllable by LogReplayTransport's
//   simulated clock in a host-side test. Flagged here deliberately,
//   inherited from the frozen SDS's own signature choice, not introduced
//   silently by this task.
uint32_t FifoArena_HeldSinceMs();
