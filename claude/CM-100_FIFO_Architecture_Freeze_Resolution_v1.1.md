Each of the 7 Must-Fix findings was resolved with the same shape: describe the contradiction, state one final decision, apply it consistently everywhere it appears, then verify by re-grepping rather than assuming.

| Finding | Final decision |
|---|---|
| 1.1 (L2/L3 leak) | `FrameCodec_Step()` takes plain `int16_t*` pointers, never `FifoArena*` — matching the pattern `SampleDecoder` already used correctly |
| 2.1 (multi-consumer) | One function, `handleFifoCaptureCompletion()`, is the sole `TryAcquireResult`/`ReleaseResult` call site; telemetry and (later) storage are steps inside it, not separate acquirers |
| 3.1 (hidden state) | Explicit `FrameCodecState`, owned by the session, passed by pointer — the plan's own `/* internal scan state */ ...` placeholder is gone |
| 3.2 (admission coupling) | `FifoCaptureRequest` gains an `admissionContext` field, populated by the caller — `Request()`'s signature is unchanged, `fifo_driver.cpp` never touches a `.ino` global |
| 4.1 (EN-pin contradiction) | The SDS was right; the Implementation Plan was wrong and is corrected to match it — the transport owns EN toggling around its `write()` |
| 5.1 (false timing justification) | §15.3's reasoning is rewritten — `ABSENT_STOPPING_MS` is RPM-derived, not Modbus-derived. No constant changed, only the stated justification. **Verifying this one turned up an eighth, previously unflagged inconsistency** — §22 still said "implementation must not begin until both gates clear," contradicting the corrected position. Fixed in the same pass. |
| 7.1 (orphaned error codes) | `FifoTransport` gains `hadOverflow()`; `S6`'s handler is explicitly assigned the `fill`-regression/overrun comparison, reusing the already-tracked `lastProgressFill` |

The two Should-Improve items (`outHandle`'s unused output, `FifoDriverConfig`'s copy-vs-retain lifetime) were deliberately left untouched — they were never blockers, and going beyond the 7 Must-Fix items wasn't asked for. Both are logged as intentionally deferred in the Implementation Plan so they read as a decision, not an oversight.

Final consistency audit confirmed zero Must-Fix items remain, zero contradictory ownership definitions remain, zero undefined public interfaces remain, and — checked explicitly — no previously-approved decision was reversed and no new feature was introduced.

# ARCHITECTURE FROZEN

**Recommend beginning Phase 1 implementation** — `CM-100_FIFO_Implementation_Plan_v1.1`, Tasks 0.1–1.3 (RX buffer sizing, transport interface, transport implementations).
