#pragma once
// ============================================================================
// [v16.6.3-fifo] fifo_transport_uart485.h
//
// L1 -- Production FifoTransport implementation, wrapping the existing
// SerialRS485 HardwareSerial object (CM-100_FIFO_Implementation_Plan_v1.3
// Task 1.2).
//
// This is the ONLY file in the entire FIFO driver that touches Arduino's
// HardwareSerial API, or any platform-specific symbol, directly.
// fifo_transport.h (Task 1.1) stays platform-independent.
//
// [v1.3, Architecture Freeze] Uart485Transport is a PURE UART BYTE
// TRANSPORT. It never owns the RS485 bus, and it never owns the RS485 EN
// pin -- no EN-pin symbol, digitalWrite(), or pinMode() call for that pin
// exists anywhere in this file or its .cpp. The caller must already own
// the RS485 bus, and must already have asserted EN, before calling
// write() -- this is a documented precondition of the FifoTransport
// contract (SDS SS6.1), not something this file checks or enforces. Per
// ADR-1 ("Bus ownership includes EN ownership"), bus/EN ownership belongs
// entirely to taskModbusRead()'s OwnsBus() boundary (Implementation Plan
// Task 4.2), never to this file.
// ============================================================================

#include "fifo_transport.h"
#include <HardwareSerial.h>

// Uart485Transport_Init() -- populate `out` with callbacks bound to `serial`.
//
//   Precondition: `serial->begin(...)` must already have been called. This
//                 function registers an error callback on the live UART
//                 driver instance; it does not configure baud/pins itself --
//                 that remains setup()'s existing responsibility, unchanged
//                 by this task.
//   Precondition: this file provides only a UART byte transport
//                 implementation -- it never calls digitalWrite() or
//                 pinMode() for the RS485 EN pin, and owns neither the EN
//                 pin nor the RS485 bus in any form. Bus ownership and EN
//                 ownership belong entirely to the caller (SDS SS6.1,
//                 ADR-1): the caller must already own the RS485 bus and
//                 must already have asserted EN before any write() call
//                 this transport makes.
//   Lifetime:     `serial` must outlive the returned FifoTransport -- in
//                 production this is the global SerialRS485 object, which
//                 lives for the process lifetime.
//   Reentrancy:   not reentrant, not thread-safe against concurrent calls to
//                 itself -- call exactly once, from setup(), before any
//                 FifoDriver_Service() call (matches FifoDriver_Init()'s own
//                 "once per boot" contract, SDS A-1).
void Uart485Transport_Init(FifoTransport* out, HardwareSerial* serial);

// [Task 7.1 -- UART receive-error instrumentation, diagnostic only, not a
// permanent production feature] Cumulative, per-error-type counters for
// every hardwareSerial_error_t this transport's onReceiveError() callback
// can receive. Closes the gap Task 6.2 identified: the existing
// Uart485Transport_OnReceiveError() only ever acted on UART_FIFO_OVF_ERROR/
// UART_BUFFER_FULL_ERROR (setting s_rxOverflow, read via the unchanged
// hadOverflow() transport-contract member) -- UART_BREAK_ERROR/
// UART_FRAME_ERROR/UART_PARITY_ERROR reached the callback and were silently
// dropped. These counters now record all five, independently of and
// without altering s_rxOverflow/hadOverflow()'s existing behavior.

// Uart485Transport_GetErrorCounts() -- read the current cumulative counts.
//   Precondition:  all five out-pointers non-null.
//   Postcondition: each *out receives the lifetime count of that error type
//                  since the last Uart485Transport_ResetErrorCounts() call
//                  (or since boot, if never reset). Read-only -- does not
//                  clear or otherwise modify the counters (unlike
//                  hadOverflow()'s existing read-and-clear contract).
void Uart485Transport_GetErrorCounts(uint32_t* outFifoOvf, uint32_t* outBufferFull,
                                      uint32_t* outBreak, uint32_t* outFrameErr,
                                      uint32_t* outParityErr);

// Uart485Transport_ResetErrorCounts() -- zero all five counters.
//   Precondition:  none. Intended call site: exactly once per NEW FIFO
//                  session admission, never between retry attempts of the
//                  same session (the caller, not this function, is
//                  responsible for calling this only at the correct
//                  boundary).
void Uart485Transport_ResetErrorCounts();
