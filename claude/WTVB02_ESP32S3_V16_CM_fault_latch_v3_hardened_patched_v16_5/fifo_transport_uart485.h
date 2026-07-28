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
