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
const lineBuilderSrc = fs.readFileSync(path.join(__dirname, 'line_message_builder_SME_WORDING.js'), 'utf-8');

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

// ---- V2 regression assertions: the two review-recommended wording fixes ----
// Per LINE_SME_WORDING_FINAL_REVIEW_20260915.md Defects 1 and 2, and the
// V2-task's explicit regression-assertion requirements.

// STOPPED contains "ธรรมดา" and does not use the ambiguous "ปกติ" in that
// clause (the specific parenthetical explaining why no measurement exists).
(function () {
    const t = results.E_STOPPED.text;
    const hasThammada = t.includes('ธรรมดา');
    const parenClauseMatch = t.match(/\(([^)]*)\)/);
    const parenClause = parenClauseMatch ? parenClauseMatch[1] : '';
    const parenHasNoBarePokati = !parenClause.includes('สถานะปกติ');
    assertions.push({
        case: 'E_STOPPED', check: 'V2_contains_thammada',
        pass: hasThammada,
    });
    assertions.push({
        case: 'E_STOPPED', check: 'V2_parenthetical_no_ambiguous_pokati',
        pass: parenHasNoBarePokati,
    });
})();

// UNAVAILABLE (D) starts with ❓.
for (const key of ['D_RUNNING_unavailable', 'D2_RUNNING_unavailable_recoveryTagged']) {
    assertions.push({
        case: key, check: 'V2_D_starts_with_question_mark_emoji',
        pass: results[key].text.startsWith('❓'),
    });
}
assertions.push({
    case: 'G2.step1', check: 'V2_D_starts_with_question_mark_emoji',
    pass: results.G2_CRITICAL_to_recovery_NORMAL_vibUnavailableAtRecovery.step1_text.startsWith('❓'),
});
assertions.push({
    case: 'G2.step2', check: 'V2_D_starts_with_question_mark_emoji',
    pass: results.G2_CRITICAL_to_recovery_NORMAL_vibUnavailableAtRecovery.step2_text.startsWith('❓'),
});

// UNAVAILABLE never contains "กลับสู่ NORMAL" (re-asserted post-fix, both
// old and new confirmed-recovery header strings).
for (const key of ['D_RUNNING_unavailable', 'D2_RUNNING_unavailable_recoveryTagged']) {
    assertions.push({
        case: key, check: 'V2_UNAVAILABLE_never_confirmed_recovery',
        pass: !results[key].text.includes('กลับสู่ NORMAL') && !results[key].text.includes('กลับสู่สถานะปกติ'),
    });
}

// WARNING still starts with ⚠️.
assertions.push({
    case: 'B_WARNING_valid', check: 'V2_WARNING_starts_with_warning_emoji',
    pass: results.B_WARNING_valid.text.startsWith('⚠️'),
});

// CRITICAL still starts with 🚨.
assertions.push({
    case: 'C_CRITICAL_valid', check: 'V2_CRITICAL_starts_with_siren_emoji',
    pass: results.C_CRITICAL_valid.text.startsWith('🚨'),
});

// NORMAL recovery wording unchanged (byte-for-byte vs the V1 baseline
// text recorded in LINE_SME_WORDING_FINAL_REVIEW_20260915.md / the
// original test_output_sme.json — neither review fix touched state A).
const A_EXPECTED_V1 = '✅ กลับสู่สถานะปกติ\n\n🏭 โรงงาน: plant01\n⚙️ เครื่อง: pump01\n\n🔎 เกิดอะไรขึ้น:\nค่าการสั่นสะเทือนของเครื่องนี้กลับสู่ระดับปกติแล้ว หลังจากที่เคยตรวจพบความผิดปกติก่อนหน้านี้\n\n📌 ตอนนี้:\nค่าการสั่นสะเทือน: 0.34 mm/s\nสถานะเครื่อง: กำลังทำงาน\n\n🔧 ควรทำอะไร:\nไม่ต้องดำเนินการเพิ่มเติม ระบบจะเฝ้าระวังค่าการสั่นสะเทือนต่อเนื่อง\n\n⏰ ';
assertions.push({
    case: 'A_NORMAL_valid', check: 'V2_NORMAL_wording_unchanged_from_V1',
    pass: results.A_NORMAL_valid.text.startsWith(A_EXPECTED_V1),
});

// RPM-invalid output remains byte-identical to its RPM-valid equivalent
// (re-asserted post-fix; same check as V1's F_vs_A / F2_vs_C, repeated
// here explicitly per the V2 task's requirement list).
assertions.push({
    case: 'F_vs_A_post_fix', check: 'V2_RPM_invalid_still_identical_to_valid_equivalent',
    pass: results.F_RUNNING_rpmInvalid_vibValid_NORMAL.text === results.A_NORMAL_valid.text,
});
assertions.push({
    case: 'F2_vs_C_post_fix', check: 'V2_RPM_invalid_still_identical_to_valid_equivalent',
    pass: results.F2_RUNNING_rpmInvalid_vibValid_CRITICAL.text === results.C_CRITICAL_valid.text,
});

const allPass = assertions.every(a => a.pass);

console.log(JSON.stringify({ results, assertions, allPass }, null, 2));
