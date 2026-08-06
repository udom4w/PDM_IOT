// ============================================================================
// [v16.6.3-fifo] fifo_transport_uart485.cpp
//
// Production FifoTransport implementation. See fifo_transport_uart485.h for
// the public contract and CM-100_FIFO_DRIVER_SDS_v1.1 SS6 for the interface
// this satisfies. No FIFO protocol, framing, CRC, retry, or state-machine
// logic exists in this file -- it moves bytes and reports transport-level
// conditions only (SDS SS6.1's "error-reporting responsibility" split).
// ============================================================================

#include "fifo_transport_uart485.h"
#include <Arduino.h>

// ----------------------------------------------------------------------------
// Context: this transport wraps exactly one HardwareSerial object. Static
// storage, not heap -- there is exactly one production instance, ever
// (SDS D-12, zero-heap).
// ----------------------------------------------------------------------------
struct Uart485TransportCtx {
  HardwareSerial* serial;
};

static Uart485TransportCtx s_ctx = {nullptr};

// ----------------------------------------------------------------------------
// Overflow flag. Written only by the UART driver's internal event task (via
// the onReceiveError callback below), read-and-cleared only by
// hadOverflow(), which per SDS D-15/SS14.1 is called only from
// taskModbusRead(). Single writer (the event task), single reader
// (taskModbusRead) -- a plain volatile bool matches this codebase's existing
// convention for single-word cross-task flags (CLAUDE.md: single-word values
// are atomic on Xtensa). A theoretical read/clear race against a
// same-instant overflow is accepted: worst case is deferring a diagnostic
// flag to the next call, never a data-integrity issue.
// ----------------------------------------------------------------------------
static volatile bool s_rxOverflow = false;

// ----------------------------------------------------------------------------
// [Task 7.1 -- UART receive-error instrumentation, diagnostic only, not a
// permanent production feature] Cumulative, per-error-type counters for all
// five hardwareSerial_error_t values Uart485Transport_OnReceiveError() can
// receive. Same single-writer (event task) / single-reader
// (taskModbusRead, via Uart485Transport_GetErrorCounts()) pattern as
// s_rxOverflow above -- plain volatile uint32_t, matching this codebase's
// established single-word cross-task convention. Reset only by
// Uart485Transport_ResetErrorCounts(), never implicitly.
// ----------------------------------------------------------------------------
static volatile uint32_t s_countFifoOvf    = 0;
static volatile uint32_t s_countBufferFull = 0;
static volatile uint32_t s_countBreak      = 0;
static volatile uint32_t s_countFrameErr   = 0;
static volatile uint32_t s_countParityErr  = 0;

static void Uart485Transport_OnReceiveError(hardwareSerial_error_t err) {
  // [Task 7.1] Count every one of the five detectable error types --
  // Task 6.2's audit found UART_BREAK_ERROR/UART_FRAME_ERROR/
  // UART_PARITY_ERROR reached this function and were silently dropped.
  // s_rxOverflow/hadOverflow()'s existing behavior, directly below,
  // is completely unchanged.
  switch (err) {
    case UART_FIFO_OVF_ERROR:    s_countFifoOvf++;    break;
    case UART_BUFFER_FULL_ERROR: s_countBufferFull++; break;
    case UART_BREAK_ERROR:       s_countBreak++;       break;
    case UART_FRAME_ERROR:       s_countFrameErr++;    break;
    case UART_PARITY_ERROR:      s_countParityErr++;   break;
    default:                     break;  // UART_NO_ERROR never reaches this callback
  }
  // [Task 7.3 -- TEMPORARY DIAGNOSTIC ONLY, NVS/flash correlation
  // investigation, not a permanent production feature] Print immediately,
  // from this callback's own context (HardwareSerial's _uartEventTask,
  // HardwareSerial.cpp:275-322), the instant a FIFO_OVF/BUFFER_FULL event
  // is detected -- this is the earliest possible point to timestamp the
  // event for correlation against [NVS_BEGIN]/[NVS_END] markers printed
  // elsewhere. Uses this file's existing direct Serial.printf() access
  // (already established as safe in this file -- unlike fifo_driver.cpp,
  // this is the one file in the whole FIFO driver that is Arduino-
  // dependent by design, see this file's own top-of-file comment).
  if (err == UART_FIFO_OVF_ERROR || err == UART_BUFFER_FULL_ERROR) {
    Serial.printf("[UART_ERR]\ntimestamp_ms=%lu\ntype=%s\nfifo_ovf_count=%lu\nbuffer_full_count=%lu\n",
                  (unsigned long)millis(),
                  (err == UART_FIFO_OVF_ERROR) ? "UART_FIFO_OVF" : "UART_BUFFER_FULL",
                  (unsigned long)s_countFifoOvf, (unsigned long)s_countBufferFull);
  }
  if (err == UART_FIFO_OVF_ERROR || err == UART_BUFFER_FULL_ERROR) {
    s_rxOverflow = true;
  }
}

// ----------------------------------------------------------------------------
// read() -- FifoTransport contract: non-blocking, copies up to maxLen
// already-received bytes, 0 is normal.
//
//   Ownership:  reads from the UART driver's own internal RX ring buffer
//               (sized by setup()'s existing setRxBufferSize(2048), Task
//               0.1) -- this function owns none of that storage, only the
//               copy destination `dst`, which the CALLER owns.
//   Blocking:   non-blocking. HardwareSerial::read(uint8_t*, size_t) calls
//               into the IDF driver with an explicit 0-tick timeout --
//               verified by direct inspection of esp32-hal-uart.c /
//               HardwareSerial.cpp this session, not assumed. This is a
//               DIFFERENT overload from Stream::readBytes(), which is
//               timeout-based and blocking -- readBytes() is never used
//               anywhere in this file.
//   Error:      none reported by this function itself -- 0 returned means
//               "nothing buffered right now," which the FifoTransport
//               contract defines as normal, not an error (SDS SS6.1).
//   Assumption: `ctx` is always the address of s_ctx, populated by
//               Uart485Transport_Init(); never null once initialized.
// ----------------------------------------------------------------------------
static size_t Uart485Transport_Read(void* ctx, uint8_t* dst, size_t maxLen) {
  Uart485TransportCtx* c = static_cast<Uart485TransportCtx*>(ctx);
  return c->serial->read(dst, maxLen);
}

// ----------------------------------------------------------------------------
// write() -- FifoTransport contract: may block only for the physical
// duration of transmitting `len` bytes (SDS SS6.1, ~8.3ms for the 8-byte
// FIFO request frame at 9600 baud).
//
//   Ownership:  pure byte transmission only. [v1.3, Architecture Freeze]
//               This function does not assert, deassert, or reference the
//               RS485 EN pin in any form -- no EN-pin symbol exists
//               anywhere in this file. Per ADR-1 ("Bus ownership includes
//               EN ownership") and SDS SS13.2.1, EN ownership belongs
//               entirely to taskModbusRead()'s OwnsBus() boundary
//               (Implementation Plan Task 4.2), not this transport.
//   Blocking:   HardwareSerial::write() blocks only until the bytes are
//               queued/transmitted -- it does not wait for a response. This
//               matches the SDS's stated ~8.3ms budget.
//   Error:      returns the actual byte count written (0..len); no separate
//               error code, matching the FifoTransport contract's narrow
//               error-reporting scope (SDS SS6.1) -- a short write is
//               visible to the caller as actualLen < len.
//   Precondition (SDS SS6.1): the caller already owns the RS485 bus and has
//               already asserted EN before this is called. This function
//               neither checks nor enforces that precondition -- per D-7,
//               the transport has no visibility into bus arbitration by
//               design. Ordering is instead guaranteed by the FIFO driver's
//               own state-machine reachability (SDS SS13.2.1): this
//               function is only reachable once FifoDriver_OwnsBus() has
//               already become true.
// ----------------------------------------------------------------------------
static size_t Uart485Transport_Write(void* ctx, const uint8_t* src, size_t len) {
  Uart485TransportCtx* c = static_cast<Uart485TransportCtx*>(ctx);
  return c->serial->write(src, len);
}

// ----------------------------------------------------------------------------
// available() -- FifoTransport contract: non-blocking byte count.
//   Ownership:  read-only query of the UART driver's own RX buffer state.
//   Blocking:   non-blocking -- HardwareSerial::available() is a direct
//               query, no wait.
//   Error:      none -- returns 0 if nothing buffered, which is not an
//               error. A defensively-handled negative return from the
//               underlying int available() (not expected on a validly
//               initialized UART, but not assumed away either) is also
//               treated as 0, never underflowed into a huge size_t.
//   Assumption: same as read().
// ----------------------------------------------------------------------------
static size_t Uart485Transport_Available(void* ctx) {
  Uart485TransportCtx* c = static_cast<Uart485TransportCtx*>(ctx);
  int n = c->serial->available();
  return (n > 0) ? static_cast<size_t>(n) : 0;
}

// ----------------------------------------------------------------------------
// flushRx() -- FifoTransport contract: discard all buffered RX bytes,
// non-blocking.
//   Ownership:  discards from the UART driver's own RX buffer; owns nothing
//               itself.
//   Blocking:   non-blocking BY CONSTRUCTION, not by using
//               HardwareSerial::flush(false). That function was checked
//               against source this session (esp32-hal-uart.c,
//               uartFlushTxOnly()) and found to internally spin-wait for
//               TX-idle before discarding RX, for BOTH flush() overloads --
//               using it here would violate this callback's non-blocking
//               contract in the (unlikely but real) case TX has not yet
//               drained. This function instead discards via the
//               already-verified-non-blocking read() path: repeatedly
//               draining into a small scratch buffer until a read returns
//               less than a full buffer. The loop is bounded by "bytes
//               already buffered" (at most the configured 2048-byte RX
//               buffer, Task 0.1), never by "wait for more to arrive," so it
//               terminates without ever blocking on new bytes.
//   Error:      none.
//   Assumption: same as read().
// ----------------------------------------------------------------------------
static void Uart485Transport_FlushRx(void* ctx) {
  Uart485TransportCtx* c = static_cast<Uart485TransportCtx*>(ctx);
  uint8_t scratch[64];
  size_t got;
  do {
    got = c->serial->read(scratch, sizeof(scratch));
  } while (got == sizeof(scratch));
}

// ----------------------------------------------------------------------------
// nowMs() -- FifoTransport contract: monotonic milliseconds, non-blocking.
//   Ownership:  delegates to the global Arduino millis() -- owns nothing.
//   Blocking:   non-blocking.
//   Error:      none.
//   Assumption: `ctx` is unused; millis() is a free function, not tied to a
//               specific serial instance.
// ----------------------------------------------------------------------------
static uint32_t Uart485Transport_NowMs(void* ctx) {
  (void)ctx;
  return millis();
}

// ----------------------------------------------------------------------------
// hadOverflow() -- FifoTransport contract: true if an RX overflow occurred
// since the last call, non-blocking.
//   Ownership:  reads and clears s_rxOverflow (file-scope, above).
//   Blocking:   non-blocking -- a single volatile read plus write.
//   Error:      this callback IS the error report -- see s_rxOverflow's own
//               doc comment for the cross-task write/read reasoning.
//   Assumption: Uart485Transport_Init() has registered
//               Uart485Transport_OnReceiveError via serial->onReceiveError()
//               -- without that registration this would always read false,
//               not because overflow cannot happen, but because nothing
//               would ever set the flag.
//
//   -- Verified against source, not assumed --
//   This ESP32 core's UART driver disables its overflow interrupt ONLY when
//   the separate, explicitly opt-in uartSetFastReading() is called
//   (esp32-hal-uart.c). Confirmed by a project-wide search this session that
//   this function is never called anywhere in this codebase or its vendored
//   libraries. The standard begin()/uart_driver_install() path this project
//   actually uses leaves overflow detection enabled, delivered via the UART
//   driver's internal event task through HardwareSerial::onReceiveError()
//   (UART_FIFO_OVF_ERROR / UART_BUFFER_FULL_ERROR). This is a genuine
//   detection mechanism, not a permanently-false stub.
// ----------------------------------------------------------------------------
static bool Uart485Transport_HadOverflow(void* ctx) {
  (void)ctx;
  bool had = s_rxOverflow;
  s_rxOverflow = false;
  return had;
}

// [Task 7.1] See fifo_transport_uart485.h for the full contract. Read-only --
// unlike hadOverflow() above, does not clear anything.
void Uart485Transport_GetErrorCounts(uint32_t* outFifoOvf, uint32_t* outBufferFull,
                                      uint32_t* outBreak, uint32_t* outFrameErr,
                                      uint32_t* outParityErr) {
  *outFifoOvf    = s_countFifoOvf;
  *outBufferFull = s_countBufferFull;
  *outBreak      = s_countBreak;
  *outFrameErr   = s_countFrameErr;
  *outParityErr  = s_countParityErr;
}

// [Task 7.1] See fifo_transport_uart485.h for the full contract -- caller's
// responsibility to invoke this only at a genuinely new FIFO session's
// admission, never between retry attempts of the same session.
void Uart485Transport_ResetErrorCounts() {
  s_countFifoOvf    = 0;
  s_countBufferFull = 0;
  s_countBreak      = 0;
  s_countFrameErr   = 0;
  s_countParityErr  = 0;
}

// ----------------------------------------------------------------------------
void Uart485Transport_Init(FifoTransport* out, HardwareSerial* serial) {
  s_ctx.serial = serial;
  s_rxOverflow = false;

  out->ctx = &s_ctx;
  out->read = Uart485Transport_Read;
  out->write = Uart485Transport_Write;
  out->available = Uart485Transport_Available;
  out->flushRx = Uart485Transport_FlushRx;
  out->nowMs = Uart485Transport_NowMs;
  out->hadOverflow = Uart485Transport_HadOverflow;

  serial->onReceiveError(Uart485Transport_OnReceiveError);
}
