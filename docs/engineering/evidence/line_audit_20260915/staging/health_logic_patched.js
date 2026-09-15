// =============================================
// 🧠 Health Logic — FIFO/DSP authoritative
// [Phase0-R3] Legacy VRMS (p.rms) is no longer read anywhere in this node.
//   Authoritative vibration source: p.velocity_rms_overall + p.velocity_data_valid
//   Legacy p.rms is NEVER used as a gate and NEVER used as a fallback.
// =============================================

var p = msg.payload;

if (!p || typeof p !== 'object') {
    node.warn('[Health] ❌ DROP — invalid payload');
    node.status({
        fill: 'red',
        shape: 'ring',
        text: '❌ DROP: invalid payload'
    });
    return null;
}

// ─────────────────────────────────────────────
// Mandatory identity / decision gates (NOT legacy — unchanged)
// ─────────────────────────────────────────────

if (!p.machine_id) {
    node.warn('[Health] ❌ DROP — machine_id missing');
    node.status({
        fill: 'red',
        shape: 'ring',
        text: '❌ DROP: machine_id'
    });
    return null;
}

if (!['NORMAL', 'WARNING', 'CRITICAL'].includes(p.alarm_level)) {
    node.warn(
        '[Health] ❌ DROP — alarm_level missing/invalid (got: "' +
        p.alarm_level +
        '") | machine=' +
        p.machine_id
    );

    node.status({
        fill: 'red',
        shape: 'ring',
        text: '❌ DROP: alarm_level'
    });

    return null;
}

// ─────────────────────────────────────────────
// [Phase0-R3] FIFO/DSP velocity — the ONE vibration source
//
// EXACT BEHAVIOUR:
//   velocity_data_valid === true (or 1) AND velocity_rms_overall is a
//   finite number >= 0
//       -> VIB_OK. p.vibration_measure_mms is set, source = 'fifo_dsp'.
//
//   anything else (flag false/0/absent, field absent, NaN, negative)
//       -> VIB_UNAVAILABLE. The message is PASSED THROUGH, not dropped.
//          p.vibration_measure_mms = null, source = 'unavailable'.
//
// Why pass-through and not drop: velocity_data_valid === false is the
// NORMAL steady state whenever the motor is not running (firmware omits
// velocity_rms_* entirely in that case). Dropping here would silently
// delete every stopped-machine sample from InfluxDB and from the LINE
// path -- a regression versus the legacy gate, which accepted rms = 0.
// Unavailability is therefore reported explicitly downstream instead of
// being turned into message loss.
//
// There is NO automatic fallback to p.rms. If a genuinely pre-Product-1
// firmware ever appears (no velocity_data_valid key at all) that fact is
// logged as LEGACY_FALLBACK below and the sample is still marked
// unavailable -- the legacy number is not substituted.
// ─────────────────────────────────────────────

var velFlagPresent = (p.velocity_data_valid !== undefined && p.velocity_data_valid !== null);
var velFlagOk      = (p.velocity_data_valid === true || p.velocity_data_valid === 1);
var velNum         = Number(p.velocity_rms_overall);
var velNumOk       = (p.velocity_rms_overall !== undefined &&
                      p.velocity_rms_overall !== null &&
                      isFinite(velNum) &&
                      velNum >= 0);

var vibOk = velFlagOk && velNumOk;

if (vibOk) {
    p.vibration_measure_mms    = velNum;
    p.vibration_measure_source = 'fifo_dsp';
    p.vibration_measure_status = 'OK';
} else {
    p.vibration_measure_mms    = null;
    p.vibration_measure_source = 'unavailable';
    p.vibration_measure_status = 'UNAVAILABLE';

    if (!velFlagPresent) {
        // Pre-Product-1 firmware. Reported loudly; value NOT substituted.
        node.warn(
            '[Health] ⚠ LEGACY_FALLBACK — velocity_data_valid absent ' +
            '(pre-Product-1 firmware). Vibration marked UNAVAILABLE; ' +
            'legacy p.rms deliberately NOT used. | machine=' + p.machine_id
        );
    } else {
        node.warn(
            '[Health] ⚠ VIB_UNAVAILABLE — velocity_data_valid=' +
            p.velocity_data_valid +
            ' velocity_rms_overall=' + p.velocity_rms_overall +
            ' | passing through | machine=' + p.machine_id
        );
    }
}

// ─────────────────────────────────────────────
// [Verdict-Fix-1] Authoritative live-verdict flag.
// Mirrors contract.py's Dashboard formula EXACTLY:
//   alarm_live = (vibration_status == "OK") and (motor_code == 2)
// so LINE and the Dashboard agree on when alarm_level may be presented
// as a trustworthy, live measurement rather than a held/unconfirmed
// value. Purely additive -- nothing upstream or in alarm_level/alarm_code
// itself is changed, and nothing previously read this field.
// rpm_valid plays no part in this formula, unchanged from before: RPM
// validity must not alter the vibration verdict.
// ─────────────────────────────────────────────
var motorCode = Number(p.motor_state);
p.alarm_verdict_live = (p.vibration_measure_status === 'OK') && (motorCode === 2);

// ─────────────────────────────────────────────
// alarm_code
// ─────────────────────────────────────────────

if (p.alarm_code === undefined) {
    p.alarm_code =
        p.alarm_level === 'CRITICAL' ? 2 :
            p.alarm_level === 'WARNING' ? 1 : 0;
}

// Preserve existing UI/display thresholds.
// These are NOT Product-1 vibration alarm thresholds.
p.thresholds = {
    warning: 4.5,
    critical: 7.1
};

// ─────────────────────────────────────────────
// Status badge
// ─────────────────────────────────────────────

var velocityText = vibOk ? velNum.toFixed(2) + ' mm/s' : 'UNAVAILABLE';

node.status({
    fill:
        p.alarm_level === 'CRITICAL' ? 'red' :
            p.alarm_level === 'WARNING' ? 'yellow' :
                vibOk ? 'green' : 'grey',

    shape: 'dot',

    text:
        '⚡ ' + p.machine_id +
        ' V:' + velocityText +
        ' [' + (p.vibration_status || 'UNKNOWN') + ']' +
        ' [' + p.alarm_level + ']'
});

return msg;
