# CT_TURNS Commissioning — Evidence Results

## 1. Objective

Execute physical commissioning of `CT_TURNS` per
`P2_CT_TURNS_VERIFICATION_PLAN.md` and
`P2_CT_TURNS_COMMISSIONING_CHECKLIST.md`, and record the evidence
collected during this session.

## 2. Test Configuration

- `CT_TURNS` = 2 (physically confirmed)
- Firmware build used: commit `6160c05-dirty` (`FW_VERSION` "16.5", per
  compiled source). Boot-banner capture (`Firmware`/`Git Commit`/
  `BUILD_ID` lines) was not successfully obtained on Serial during this
  session; build identity is corroborated instead by the flash hash
  verification and the live `ctTurns=2` cross-check below (§3).

## 3. Firmware Verification

- Compile successful.
- Flash successful.
- Flash hash verified.
- `CURRENT_DIAG` confirmed active on Serial; observed `ctTurns=2`,
  matching source.

## 4. Measurement Results

### Load Point 1

- Reference Clamp = 0.85 A
- rawA = 1.79 A
- engineeringA = 0.82 A
- Error = 3.5%
- Acceptance = ±5%
- Result = PASS

## 5. Limitations

- Only one load point was available.
- Medium and high load verification were not performed.

## 6. Conclusion

Partial Verification — Load Point 1 passed within the defined
acceptance criterion. Additional load points are required before
declaring full CT_TURNS verification.

## 7. Follow-Up Note (Addendum)

A later consistency review found that the recorded `rawA`/`engineeringA`
pair for Load Point 1 was not fully consistent with the expected
`CT_TURNS` scaling (`rawA (1.79) ÷ CT_TURNS (2) = 0.895`, not the
recorded `engineeringA = 0.82`).

**The discrepancy was not conclusively explained.** A plausible
contributing mechanism was identified (two independently-timed
diagnostic print sites active at the time of this measurement, since
consolidated under P3-01's single-source-of-truth redesign), but this
explains only that a mismatch of this kind was *possible* — it does not
confirm that this specific discrepancy is benign.

**A repeat measurement is recommended** under the now-current P3-01
diagnostic architecture, which guarantees `rawA`/`engineeringA` always
originate from the same sample.

The original values above are preserved unmodified, per the
evidence-only principle established for this engagement. **This result
should be treated as Partial Verification, not final commissioning
evidence**, until re-measured.
