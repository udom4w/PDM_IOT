#pragma once
// ============================================================================
// [v16.6.4-fifo] log_replay_transport.h
//
// L1 -- Host-only test-fixture FifoTransport implementation
// (CM-100_FIFO_Implementation_Plan_v1.3 Task 1.3; CM-100_FIFO_DRIVER_SDS_v1.0
// SS23).
//
// Replays a pre-recorded byte stream, with its recorded per-byte arrival
// timing, against the FifoTransport contract defined in fifo_transport.h
// (Task 1.1, unmodified, zero Arduino dependency). This turns the 7 recorded
// hardware logs -- including all 7 failures -- into deterministic, host-
// runnable regression fixtures for L2/L3 (PRR C-14).
//
// HOST-ONLY. Lives under test/ specifically so Arduino's sketch builder
// never sweeps it into the production firmware build (SDS SS2.4, Plan SS1/
// SS2.4) -- this file must never be referenced from any file that ships to
// the device.
//
// Zero Arduino/ESP32/HardwareSerial/RS485/Modbus dependency by design --
// compiles standalone under a plain host C++ compiler, exactly like
// fifo_transport.h itself.
//
// Out of scope for this file (deliberately): parsing serial_log*.txt.
// Turning the hex-dump-style recorded logs into the plain
// (bytes[], timestamps[]) arrays this fixture replays is a separate,
// one-time conversion step (Implementation Plan Task 2.4's first sub-task),
// not this transport's job. This file only ever replays arrays a caller has
// already constructed.
// ============================================================================

#include "../fifo_transport.h"

// LogReplayTransport_Init() -- populate `out` with callbacks that replay
// `recordedBytes` against `byteTimestampsMs`, driven by an externally-owned
// simulated clock.
//
//   recordedBytes:     the byte stream to replay, caller-owned.
//   recordedLen:       length of recordedBytes / byteTimestampsMs (both
//                      arrays are the same length, index-for-index).
//   byteTimestampsMs:  recordedBytes[i]'s simulated arrival time, caller-
//                      owned, must be non-decreasing -- this is exactly the
//                      recorded K-13 timing a fixture test needs to
//                      reproduce faithfully (SDS K-13: <=2ms mid-dump gaps,
//                      up to 239ms across poll boundaries).
//   simulatedNowMs:    pointer to a single, external, test-owned uint32_t
//                      "clock". This transport only ever reads through this
//                      pointer -- it never writes to it, and there is no
//                      setter function on this transport. The test advances
//                      time by writing to *simulatedNowMs directly, between
//                      FifoDriver_Service() calls. This mirrors
//                      Uart485Transport_NowMs() delegating to an external
//                      time source (millis()) that transport doesn't own
//                      either -- same shape, a test-controlled source
//                      instead of a hardware one, matching the production
//                      abstraction rather than adding a transport-specific
//                      control API.
//   simulatedOverflow: pointer to a single, external, test-owned bool,
//                      following the exact same pattern as simulatedNowMs.
//                      This transport only ever reads through this pointer
//                      -- it never writes to it, and there is no setter
//                      function. A test that wants to exercise
//                      ERR_RX_OVERFLOW (SDS SS6.1: hadOverflow() is the sole
//                      source of that error code) sets *simulatedOverflow
//                      to true directly, then clears it directly, exactly
//                      as it drives *simulatedNowMs -- there is no
//                      clear-on-read here, unlike Uart485Transport_HadOverflow(),
//                      because clearing would mean this transport writing to
//                      caller-owned state, which the "external state, not
//                      an owned mutable API" pattern deliberately avoids. A
//                      test not exercising overflow paths can simply pass a
//                      pointer to a local `bool` left at false.
//
//   Precondition: all four array/pointer arguments must outlive the
//                 returned FifoTransport -- in a typical test these are
//                 stack locals or globals owned by the test function
//                 itself, alive for the whole scenario.
//   Lifetime:     matches Uart485Transport_Init()'s own contract -- bind
//                 once per test scenario, before driving
//                 FifoDriver_Service().
//   Reentrancy:   not reentrant, not thread-safe against concurrent calls to
//                 itself, and backed by a single static instance (mirroring
//                 Uart485Transport_Init() -- there is exactly one production
//                 instance, ever; here, exactly one active replay scenario
//                 at a time). Call again to reset/rebind for the next test
//                 scenario; do not use two simultaneously active replays.
void LogReplayTransport_Init(FifoTransport* out,
                              const uint8_t* recordedBytes,
                              size_t recordedLen,
                              const uint32_t* byteTimestampsMs,
                              const uint32_t* simulatedNowMs,
                              const bool* simulatedOverflow);
