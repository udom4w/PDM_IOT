#pragma once
// ============================================================================
// [v16.6.2-fifo] fifo_transport.h
//
// L1 -- Transport Abstraction (CM-100_FIFO_DRIVER_SDS_v1.1 SS6, D-7).
//
// A non-blocking, injected byte-transport interface. The FIFO driver (L2/L3)
// talks to this interface only -- it never references SerialRS485, Serial,
// or modbus directly. This is what makes unarbitrated bus access (PRR C-3)
// structurally impossible rather than a rule someone has to remember, and
// what makes L2/L3 host-testable against a recorded byte stream instead of
// real hardware (PRR C-14).
//
// This file is the CONTRACT only. No implementation lives here -- see
// fifo_transport_uart485.h (production, Task 1.2) and
// test/log_replay_transport.h (test fixture, Task 1.3).
//
// Zero Arduino dependency by design: this header must compile standalone
// under a plain host C++ compiler (verified below), not just arduino-cli.
// ============================================================================

#include <stdint.h>
#include <stddef.h>

// ----------------------------------------------------------------------------
// struct FifoTransport
//
// A plain function-pointer struct, not a C++ abstract base class + virtual
// dispatch (SDS SS6.2 "Alternatives considered": avoids .ino auto-prototype
// friction and vtable indirection on a hot byte loop, for the same seam).
//
// Ownership: the struct instance and the storage behind `ctx` are owned and
// populated by whichever concrete transport binds to it (Uart485Transport_Init
// for production, LogReplayTransport_Init for tests) -- never by this file,
// never by the driver that consumes it. Both must remain valid for the
// lifetime of the driver (effectively the process lifetime: FifoDriver_Init()
// runs once at boot and the driver runs until reset) -- the concrete
// transport must back `ctx` with static/global storage, never a stack-local.
//
// Thread/task assumptions: every function pointer below is called exclusively
// from taskModbusRead(), exclusively via FifoDriver_Service(), exclusively
// once per 250ms tick (SDS D-15/SS13.2). No ISR, no other task, and no other
// core ever calls through this interface at runtime -- single writer, single
// reader, by construction (SDS D-16). Host-side unit tests call it
// single-threaded, off-target; that is not a runtime concern.
//
// Blocking contract: every member is non-blocking EXCEPT `write`, which has
// one named, bounded exception -- see its own doc comment below. This is a
// deliberate, minimal exception (SDS SS6.2 tradeoffs), not a general
// allowance to add more blocking calls later.
//
// Error-reporting responsibility: deliberately narrow. `read`/`write`/
// `available` return byte counts, never an error code -- a 0-byte `read()`
// is the normal "nothing arrived yet" case, not an error (SDS SS6.1). The
// only condition this layer itself reports is a receive-buffer overflow, via
// `hadOverflow()`. Every other failure class (no response, corrupted frame,
// desync, mid-frame timeout) is inferred by the CALLER (L2's FrameCodec /
// L3's state machine) from timing (`nowMs()` deltas) and byte content, never
// by this layer -- this split is what SDS D-18's error taxonomy assumes.
// ----------------------------------------------------------------------------
struct FifoTransport {
  // Opaque, owner-supplied context pointer. Passed unmodified to every call
  // below. This file never dereferences, allocates, or frees it.
  void* ctx;

  // read() -- copy up to maxLen already-received bytes into dst.
  //   Returns: number of bytes actually copied, 0..maxLen.
  //   Contract: MUST NOT block. MUST NOT wait for more bytes to arrive.
  //             Returning 0 is normal (nothing buffered yet), not an error --
  //             the caller distinguishes "nothing yet" from "nothing for too
  //             long" itself, via nowMs(), never by this function spinning.
  size_t (*read)(void* ctx, uint8_t* dst, size_t maxLen);

  // write() -- transmit exactly len bytes from src.
  //   Returns: number of bytes actually written (0..len).
  //   Contract: MAY block, but ONLY for the physical duration of transmitting
  //             the bytes themselves (the FIFO protocol's request frame is a
  //             fixed 8 bytes, ~8.3ms at 9600 baud -- SDS SS6.2). MUST NOT
  //             block waiting for a response; this call returns once the
  //             bytes are on the wire, not once anything answers them.
  size_t (*write)(void* ctx, const uint8_t* src, size_t len);

  // available() -- how many bytes are currently buffered and ready for
  // read() to return without blocking.
  //   Contract: MUST NOT block.
  size_t (*available)(void* ctx);

  // flushRx() -- discard all currently buffered RX bytes.
  //   Contract: MUST NOT block. Used for bus-settle draining (SDS SS17,
  //             K-14's 0xE2 cascade finding) -- never a substitute for
  //             reading data that matters.
  void (*flushRx)(void* ctx);

  // nowMs() -- monotonic milliseconds. Injected (not a bare millis() call
  // baked into L2/L3) specifically so host-side tests can control and replay
  // time instead of depending on wall-clock behavior.
  //   Contract: MUST NOT block. MUST be monotonic non-decreasing.
  uint32_t (*nowMs)(void* ctx);

  // hadOverflow() -- [v1.1, Freeze Review Finding 7.1] true if the underlying
  // transport detected a receive-buffer overflow (bytes lost before this
  // layer could read them) since the last call to this function. This is the
  // SOLE source of FifoError::ERR_RX_OVERFLOW (SDS SS16.1) -- without this
  // member that error code has no way to ever be raised. The detection
  // mechanism itself is implementation-specific (e.g. a platform UART
  // overflow flag, where one is exposed) and is not specified at this layer.
  //   Contract: MUST NOT block. Semantics of "since the last call" are the
  //             concrete transport's responsibility to implement correctly
  //             (e.g. clear-on-read), not this interface's to enforce.
  bool (*hadOverflow)(void* ctx);
};
