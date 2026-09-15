// Read-only test harness for the SME-wording LINE Message Builder patch.
// Loads the ACTUAL patched function-node source (verbatim, no
// reimplementation) and the ACTUAL unchanged Health Logic source, and
// executes both with a minimal Node-RED-compatible msg/flow/node shim.
// No network access, no LINE Push is ever invoked (the harness never
// calls anything resembling an HTTP request — it only inspects the
// resulting msg.payload.messages[0].text string).

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const healthLogicSrc = fs.readFileSync(path.join(__dirname, 'health_logic_UNCHANGED_reference_copy.js'), 'utf-8');
const lineBuilderSrc = fs.readFileSync(path.join(__dirname, 'line_message_builder_SME_WORDING_V3.js'), 'utf-8');

function makeFlowContext(initial) {
    const store = Object.assign({}, initial);
    return {
        get: (key) => store[key],
        set: (key, val) => { store[key] = val; },
    };
}

function makeNodeRecorder() {
    const warnings = [];
    const statuses = [];
    return {
        warn: (m) => warnings.push(String(m)),
        status: (s) => statuses.push(s),
        _warnings: warnings,
        _statuses: statuses,
    };
}

function runFunctionNode(src, msg, flowCtx, nodeRec) {
    const sandbox = {
        msg: msg,
        flow: flowCtx,
        node: nodeRec,
        console: console,
        Date: Date,
        Number: Number,
        isFinite: isFinite,
        Object: Object,
        String: String,
        Math: Math,
        result: undefined,
    };
    vm.createContext(sandbox);
    const wrapped = `(function(){ ${src}\n})()`;
    sandbox.result = vm.runInContext(wrapped, sandbox, { filename: 'function-node.js' });
    return sandbox.result;
}

const PLANT_LINE_TOKENS = { plant01: 'TEST-TOKEN-NOT-REAL-NEVER-SENT' };
const PLANT_GROUP_IDS = { plant01: 'TEST-GROUP-NOT-REAL-NEVER-SENT' };

function runOne(payload, alertType, initialMachines) {
    const flowStore = {
        PLANT_LINE_TOKENS, PLANT_GROUP_IDS,
        machines: initialMachines || {},
    };
    const flowCtx = makeFlowContext(flowStore);

    const msg1 = { payload: JSON.parse(JSON.stringify(payload)) };
    const afterHealth = runFunctionNode(healthLogicSrc, msg1, flowCtx, makeNodeRecorder());

    if (afterHealth === null) {
        return { dropped: true };
    }

    afterHealth._machineId = afterHealth.payload.machine_id;
    afterHealth._alertType = alertType;

    const afterLine = runFunctionNode(lineBuilderSrc, afterHealth, flowCtx, makeNodeRecorder());

    return {
        dropped: false,
        alarm_verdict_live: afterHealth.payload.alarm_verdict_live,
        vibration_measure_status: afterHealth.payload.vibration_measure_status,
        text: afterLine && afterLine.payload && afterLine.payload.messages
            ? afterLine.payload.messages[0].text : null,
    };
}

const results = {};

// A. NORMAL + vibration valid
results.A_NORMAL_valid = runOne(
    { machine_id: 'pump01', plant: 'plant01', alarm_level: 'NORMAL', motor_state: 2,
      velocity_data_valid: true, velocity_rms_overall: 0.34, health_score: -1, rpm_valid: true },
    'recovery'
);

// B. WARNING + vibration valid
results.B_WARNING_valid = runOne(
    { machine_id: 'pump01', plant: 'plant01', alarm_level: 'WARNING', motor_state: 2,
      velocity_data_valid: true, velocity_rms_overall: 2.25, health_score: -1, rpm_valid: true },
    'alert'
);

// C. CRITICAL + vibration valid
results.C_CRITICAL_valid = runOne(
    { machine_id: 'pump01', plant: 'plant01', alarm_level: 'CRITICAL', motor_state: 2,
      velocity_data_valid: true, velocity_rms_overall: 4.68, health_score: -1, rpm_valid: true },
    'alert'
);

// D. RUNNING + vibration unavailable
results.D_RUNNING_unavailable = runOne(
    { machine_id: 'pump01', plant: 'plant01', alarm_level: 'CRITICAL', motor_state: 2,
      velocity_data_valid: false, health_score: -1, rpm_valid: true },
    'alert'
);

// D2. Same fact pattern, but arriving as a 'recovery'-tagged payload (e.g.
// alarm_level=NORMAL while vibration is unavailable) -- must render
// IDENTICALLY to D, never as a confirmed recovery.
results.D2_RUNNING_unavailable_recoveryTagged = runOne(
    { machine_id: 'pump01', plant: 'plant01', alarm_level: 'NORMAL', motor_state: 2,
      velocity_data_valid: false, health_score: -1, rpm_valid: true },
    'recovery'
);

// E. STOPPED
results.E_STOPPED = runOne(
    { machine_id: 'pump01', plant: 'plant01', alarm_level: 'NORMAL', motor_state: 0,
      velocity_data_valid: false, health_score: -1, rpm_valid: true },
    'recovery'
);

// F. RUNNING + RPM invalid + vibration valid (NORMAL) -- compare against A
results.F_RUNNING_rpmInvalid_vibValid_NORMAL = runOne(
    { machine_id: 'pump01', plant: 'plant01', alarm_level: 'NORMAL', motor_state: 2,
      velocity_data_valid: true, velocity_rms_overall: 0.34, health_score: -1, rpm_valid: false },
    'recovery'
);

// F2. Same, but CRITICAL + RPM invalid -- verdict/wording must be
// identical to C (RPM invalidity must not soften a CRITICAL verdict).
results.F2_RUNNING_rpmInvalid_vibValid_CRITICAL = runOne(
    { machine_id: 'pump01', plant: 'plant01', alarm_level: 'CRITICAL', motor_state: 2,
      velocity_data_valid: true, velocity_rms_overall: 4.68, health_score: -1, rpm_valid: false },
    'alert'
);

// G. CRITICAL -> automatic recovery NORMAL, genuine two-step sequence
// sharing the same flow context, exactly like two consecutive real MQTT
// messages would arrive. Two variants:
//   G1: vibration valid at BOTH instants (clean, fully-confirmed recovery)
//   G2: vibration UNAVAILABLE at the recovery instant (the disputed shape
//       from the real forensic-trace incident) -- must NOT say the
//       confirmed-recovery header.
(function runCaseG() {
    // G1
    (function () {
        const flowCtx = makeFlowContext({ PLANT_LINE_TOKENS, PLANT_GROUP_IDS, machines: {} });
        const critPayload = { machine_id: 'pump01', plant: 'plant01', alarm_level: 'CRITICAL',
            motor_state: 2, velocity_data_valid: true, velocity_rms_overall: 6.93, health_score: -1 };
        const h1 = runFunctionNode(healthLogicSrc, { payload: critPayload }, flowCtx, makeNodeRecorder());
        h1._machineId = h1.payload.machine_id; h1._alertType = 'alert';
        const l1 = runFunctionNode(lineBuilderSrc, h1, flowCtx, makeNodeRecorder());

        const normPayload = { machine_id: 'pump01', plant: 'plant01', alarm_level: 'NORMAL',
            motor_state: 2, velocity_data_valid: true, velocity_rms_overall: 0.34, health_score: -1 };
        const h2 = runFunctionNode(healthLogicSrc, { payload: normPayload }, flowCtx, makeNodeRecorder());
        h2._machineId = h2.payload.machine_id; h2._alertType = 'recovery';
        const l2 = runFunctionNode(lineBuilderSrc, h2, flowCtx, makeNodeRecorder());

        results.G1_CRITICAL_to_recovery_NORMAL_vibValidBoth = {
            step1_text: l1.payload.messages[0].text,
            step1_verdict_live: h1.payload.alarm_verdict_live,
            step2_text: l2.payload.messages[0].text,
            step2_verdict_live: h2.payload.alarm_verdict_live,
        };
    })();

    // G2 (disputed / forensic-trace shape)
    (function () {
        const flowCtx = makeFlowContext({ PLANT_LINE_TOKENS, PLANT_GROUP_IDS, machines: {} });
        const critPayload = { machine_id: 'pump01', plant: 'plant01', alarm_level: 'CRITICAL',
            motor_state: 2, velocity_data_valid: false, health_score: -1 };
        const h1 = runFunctionNode(healthLogicSrc, { payload: critPayload }, flowCtx, makeNodeRecorder());
        h1._machineId = h1.payload.machine_id; h1._alertType = 'alert';
        const l1 = runFunctionNode(lineBuilderSrc, h1, flowCtx, makeNodeRecorder());

        const normPayload = { machine_id: 'pump01', plant: 'plant01', alarm_level: 'NORMAL',
            motor_state: 2, velocity_data_valid: false, health_score: -1 };
        const h2 = runFunctionNode(healthLogicSrc, { payload: normPayload }, flowCtx, makeNodeRecorder());
        h2._machineId = h2.payload.machine_id; h2._alertType = 'recovery';
        const l2 = runFunctionNode(lineBuilderSrc, h2, flowCtx, makeNodeRecorder());

        results.G2_CRITICAL_to_recovery_NORMAL_vibUnavailableAtRecovery = {
            step1_text: l1.payload.messages[0].text,
            step1_verdict_live: h1.payload.alarm_verdict_live,
            step2_text: l2.payload.messages[0].text,
            step2_verdict_live: h2.payload.alarm_verdict_live,
        };
    })();
})();

// ---- Automated assertions (H and I, plus spot checks for A-G) ----
const allTexts = [];
for (const k of Object.keys(results)) {
    const r = results[k];
    if (r.text) allTexts.push([k, r.text]);
    if (r.step1_text) allTexts.push([k + '.step1', r.step1_text]);
    if (r.step2_text) allTexts.push([k + '.step2', r.step2_text]);
}

const assertions = [];

// H. Ensure no "Health: RUNNING" (or any "Health:" label) anywhere.
for (const [k, t] of allTexts) {
    assertions.push({
        case: k, check: 'H_no_Health_label',
        pass: !t.includes('Health:'),
    });
}

// I. Ensure unavailable vibration can never produce a confirmed-recovery
// header, in either wording (old "กลับสู่ NORMAL" or new
// "กลับสู่สถานะปกติ").
for (const key of ['D_RUNNING_unavailable', 'D2_RUNNING_unavailable_recoveryTagged']) {
    const t = results[key].text;
    assertions.push({
        case: key, check: 'I_no_confirmed_recovery_header_when_unavailable',
        pass: !t.includes('กลับสู่ NORMAL') && !t.includes('กลับสู่สถานะปกติ'),
    });
}
assertions.push({
    case: 'G2.step2', check: 'I_no_confirmed_recovery_header_when_unavailable',
    pass: !results.G2_CRITICAL_to_recovery_NORMAL_vibUnavailableAtRecovery.step2_text.includes('กลับสู่ NORMAL')
        && !results.G2_CRITICAL_to_recovery_NORMAL_vibUnavailableAtRecovery.step2_text.includes('กลับสู่สถานะปกติ'),
});

// F / F2: RPM invalidity must not change wording or verdict vs the
// RPM-valid equivalent, and "rpm" must never appear in the text.
assertions.push({
    case: 'F_vs_A', check: 'F_identical_to_RPM_valid_equivalent',
    pass: results.F_RUNNING_rpmInvalid_vibValid_NORMAL.text === results.A_NORMAL_valid.text,
});
assertions.push({
    case: 'F2_vs_C', check: 'F2_identical_to_RPM_valid_equivalent',
    pass: results.F2_RUNNING_rpmInvalid_vibValid_CRITICAL.text === results.C_CRITICAL_valid.text,
});
for (const [k, t] of allTexts) {
    assertions.push({
        case: k, check: 'F_no_rpm_word_in_any_message',
        pass: !/rpm/i.test(t),
    });
}

// D vs E must be textually distinct (never the same wording).
assertions.push({
    case: 'D_vs_E', check: 'D_and_E_are_distinct_wording',
    pass: results.D_RUNNING_unavailable.text !== results.E_STOPPED.text,
});

// E must never claim a vibration reading was taken.
assertions.push({
    case: 'E_STOPPED', check: 'E_states_no_measurement_taken',
    pass: results.E_STOPPED.text.includes('ไม่มีการวัด'),
});

// A/B/C must show a real numeric mm/s value, never "ไม่มีข้อมูล".
for (const k of ['A_NORMAL_valid', 'B_WARNING_valid', 'C_CRITICAL_valid']) {
    assertions.push({
        case: k, check: 'ABC_show_real_numeric_value',
        pass: /\d+\.\d{2} mm\/s/.test(results[k].text) && !results[k].text.includes('ไม่มีข้อมูล'),
    });
}

// No message anywhere may contain "FIFO-DSP" as user-facing text.
for (const [k, t] of allTexts) {
    assertions.push({
        case: k, check: 'no_FIFO_DSP_in_user_text',
        pass: !t.includes('FIFO-DSP'),
    });
}

// ---- V3 regression assertions ----
// Per the V3 task's explicit requirement list. The V2-specific literal
// wording checks (which asserted V2's exact STOPPED/NORMAL text) are
// intentionally NOT carried forward unchanged here: V3 deliberately
// rewords STOPPED and NORMAL again (approved, operator-reviewed target
// text), so asserting V2's literal strings would be a stale check, not a
// real regression guard. They are replaced below by checks against the
// underlying INVARIANT each one was protecting (no ambiguous "ปกติ", a
// distinct severity marker per state, etc.), which V3 must still satisfy.

// UNAVAILABLE (D) starts with ❓ — unchanged from V2.
for (const key of ['D_RUNNING_unavailable', 'D2_RUNNING_unavailable_recoveryTagged']) {
    assertions.push({
        case: key, check: 'V3_D_starts_with_question_mark_emoji',
        pass: results[key].text.startsWith('❓'),
    });
}
assertions.push({
    case: 'G2.step1', check: 'V3_D_starts_with_question_mark_emoji',
    pass: results.G2_CRITICAL_to_recovery_NORMAL_vibUnavailableAtRecovery.step1_text.startsWith('❓'),
});
assertions.push({
    case: 'G2.step2', check: 'V3_D_starts_with_question_mark_emoji',
    pass: results.G2_CRITICAL_to_recovery_NORMAL_vibUnavailableAtRecovery.step2_text.startsWith('❓'),
});

// UNAVAILABLE ไม่สามารถแสดง NORMAL — strengthened to check against every
// confirmed-recovery header string used across this engagement's history
// (V1 "กลับสู่ NORMAL", V2 "กลับสู่สถานะปกติ", V3 "การสั่นสะเทือนกลับสู่ระดับปกติ"),
// so this stays a real regression guard even if the recovery wording is
// revised again in the future.
const CONFIRMED_RECOVERY_PHRASES = ['กลับสู่ NORMAL', 'กลับสู่สถานะปกติ', 'การสั่นสะเทือนกลับสู่ระดับปกติ'];
for (const key of ['D_RUNNING_unavailable', 'D2_RUNNING_unavailable_recoveryTagged']) {
    assertions.push({
        case: key, check: 'V3_UNAVAILABLE_never_confirmed_recovery',
        pass: CONFIRMED_RECOVERY_PHRASES.every(ph => !results[key].text.includes(ph)),
    });
}
assertions.push({
    case: 'E_STOPPED', check: 'V3_STOPPED_never_confirmed_recovery',
    pass: CONFIRMED_RECOVERY_PHRASES.every(ph => !results.E_STOPPED.text.includes(ph)),
});

// STOPPED never claims a normal vibration reading: the word "ปกติ" must
// not appear anywhere in the STOPPED message at all (V3's STOPPED text
// no longer needs a "ธรรมดา"-style workaround because the ambiguous
// clause was removed outright — this asserts that stronger outcome).
assertions.push({
    case: 'E_STOPPED', check: 'V3_STOPPED_never_uses_pokati',
    pass: !results.E_STOPPED.text.includes('ปกติ'),
});

// Motor State ยังถูกต้อง — the "สถานะเครื่อง:" line, when present, must
// carry the correct Thai text for the case's motor_state code.
const MOTOR_STATE_EXPECTED = { A_NORMAL_valid: 'กำลังทำงาน', B_WARNING_valid: 'กำลังทำงาน',
    C_CRITICAL_valid: 'กำลังทำงาน', D_RUNNING_unavailable: 'กำลังทำงาน',
    D2_RUNNING_unavailable_recoveryTagged: 'กำลังทำงาน', E_STOPPED: 'หยุดทำงาน',
    F_RUNNING_rpmInvalid_vibValid_NORMAL: 'กำลังทำงาน', F2_RUNNING_rpmInvalid_vibValid_CRITICAL: 'กำลังทำงาน' };
for (const [k, expected] of Object.entries(MOTOR_STATE_EXPECTED)) {
    assertions.push({
        case: k, check: 'V3_motor_state_line_correct',
        pass: results[k].text.includes('สถานะเครื่อง: ' + expected),
    });
}

// WARNING/CRITICAL/NORMAL/UNAVAILABLE อ่านต่างกันชัดเจน — all four titles
// distinct, and (new in V3) all four use a DIFFERENT leading emoji from
// each other (V1/V2 had D and B sharing ⚠️ until the V2 fix; V3's design
// gives every one of the four its own glyph: ⚠️/🚨/✅/❓).
(function () {
    const titles = {
        WARNING: results.B_WARNING_valid.text.split('\n')[0],
        CRITICAL: results.C_CRITICAL_valid.text.split('\n')[0],
        NORMAL: results.A_NORMAL_valid.text.split('\n')[0],
        UNAVAILABLE: results.D_RUNNING_unavailable.text.split('\n')[0],
    };
    const values = Object.values(titles);
    const allTitlesDistinct = new Set(values).size === values.length;
    const emojis = values.map(t => Array.from(t)[0]);
    const allEmojisDistinct = new Set(emojis).size === emojis.length;
    assertions.push({ case: 'ABCD_titles', check: 'V3_four_states_titles_distinct', pass: allTitlesDistinct });
    assertions.push({ case: 'ABCD_titles', check: 'V3_four_states_emojis_distinct', pass: allEmojisDistinct });
})();

// WARNING still starts with ⚠️. CRITICAL still starts with 🚨. NORMAL now
// starts with ✅ (re-checked explicitly, since V3 rewords NORMAL's title
// text but the leading severity emoji must still be the recovery emoji).
assertions.push({ case: 'B_WARNING_valid', check: 'V3_WARNING_starts_with_warning_emoji', pass: results.B_WARNING_valid.text.startsWith('⚠️') });
assertions.push({ case: 'C_CRITICAL_valid', check: 'V3_CRITICAL_starts_with_siren_emoji', pass: results.C_CRITICAL_valid.text.startsWith('🚨') });
assertions.push({ case: 'A_NORMAL_valid', check: 'V3_NORMAL_starts_with_check_emoji', pass: results.A_NORMAL_valid.text.startsWith('✅') });

// CRITICAL must never assert that damage is actively occurring from a
// single reading — no speculative "ความเสียหายที่กำลังเกิดขึ้น" (or any
// "เสียหาย"/"damage" word) anywhere in the CRITICAL message.
assertions.push({
    case: 'C_CRITICAL_valid', check: 'V3_CRITICAL_no_damage_claim',
    pass: !results.C_CRITICAL_valid.text.includes('เสียหาย'),
});
assertions.push({
    case: 'F2_RUNNING_rpmInvalid_vibValid_CRITICAL', check: 'V3_CRITICAL_no_damage_claim',
    pass: !results.F2_RUNNING_rpmInvalid_vibValid_CRITICAL.text.includes('เสียหาย'),
});

// No threshold number (2.1 / 4.5 mm/s) is ever hard-coded into any
// message text — only the real payload-carried measured value appears.
for (const [k, t] of allTexts) {
    assertions.push({
        case: k, check: 'V3_no_hardcoded_threshold_numbers',
        pass: !/\b2\.1\b/.test(t) && !/\b4\.5\b/.test(t),
    });
}

// RPM invalid ไม่เปลี่ยนข้อความของ vibration verdict — re-asserted against
// the V3 wording specifically (same technique as V1/V2, repeated here
// explicitly per the V3 task's requirement list).
assertions.push({
    case: 'F_vs_A_v3', check: 'V3_RPM_invalid_still_identical_to_valid_equivalent',
    pass: results.F_RUNNING_rpmInvalid_vibValid_NORMAL.text === results.A_NORMAL_valid.text,
});
assertions.push({
    case: 'F2_vs_C_v3', check: 'V3_RPM_invalid_still_identical_to_valid_equivalent',
    pass: results.F2_RUNNING_rpmInvalid_vibValid_CRITICAL.text === results.C_CRITICAL_valid.text,
});

// automatic recovery logic unchanged — G1 (vibration valid both steps)
// must still show a genuine CRITICAL-then-confirmed-NORMAL sequence, and
// G2 (vibration unavailable at recovery) must still NEVER show a
// confirmed-recovery header at step 2. This is a behavioral check on the
// same two-step sequence used in every prior validation pass — it does
// not re-verify Health Logic/Unified State Engine source itself (done
// separately by file-hash comparison, see the deployment-report-style
// verification performed alongside this harness run), only that this
// harness's own recovery SEQUENCE still behaves correctly end-to-end
// under the new wording.
assertions.push({
    case: 'G1.step1', check: 'V3_automatic_recovery_sequence_intact',
    pass: results.G1_CRITICAL_to_recovery_NORMAL_vibValidBoth.step1_text.startsWith('🚨'),
});
assertions.push({
    case: 'G1.step2', check: 'V3_automatic_recovery_sequence_intact',
    pass: results.G1_CRITICAL_to_recovery_NORMAL_vibValidBoth.step2_text.startsWith('✅'),
});
assertions.push({
    case: 'G2.step2', check: 'V3_automatic_recovery_sequence_intact',
    pass: CONFIRMED_RECOVERY_PHRASES.every(ph => !results.G2_CRITICAL_to_recovery_NORMAL_vibUnavailableAtRecovery.step2_text.includes(ph)),
});

const allPass = assertions.every(a => a.pass);

console.log(JSON.stringify({ results, assertions, allPass }, null, 2));
