// Read-only test harness. Loads the ACTUAL patched function-node source
// (verbatim, no rewriting) from the staged flow JSON and executes it with
// a minimal Node-RED-compatible msg/flow/node shim. No network access, no
// LINE Push is ever invoked (the harness intercepts msg after the LINE
// Message Builder node and never calls anything resembling an HTTP request).

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const FLOWS_PATH = path.join(__dirname, 'staged_flows_ISOLATED_COPY.json');
const flows = JSON.parse(fs.readFileSync(FLOWS_PATH, 'utf-8'));
const byId = {};
flows.forEach(n => { byId[n.id] = n; });

const healthLogicSrc = byId['adf3dc5f003a516c'].func;
const lineBuilderSrc = byId['e982d76b3ebe0b06'].func;

// ---- Minimal Node-RED-compatible flow-context store (per test run) ----
function makeFlowContext(initial) {
    const store = Object.assign({}, initial);
    return {
        get: (key) => store[key],
        set: (key, val) => { store[key] = val; },
    };
}

// ---- Minimal node.warn/node.status recorder (no console spam needed) ----
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
    // Wrap exactly like Node-RED does: a function body ending in `return msg;`
    const wrapped = `(function(){ ${src}\n})()`;
    sandbox.result = vm.runInContext(wrapped, sandbox, { filename: 'function-node.js' });
    return sandbox.result;
}

// ---- Test cases ----
// Each case supplies a synthetic /vibration-shaped payload exactly as
// Health Logic would receive it (post Identity Extractor / Latency
// Calculator -- i.e. plant/machine_id already resolved, as real MQTT
// traffic would look).
const cases = {
    A_RUNNING_valid_NORMAL: {
        machine_id: 'pump01', plant: 'plant01', alarm_level: 'NORMAL',
        motor_state: 2, velocity_data_valid: true, velocity_rms_overall: 0.51,
        health_score: -1, rpm_valid: true,
    },
    B_RUNNING_valid_WARNING: {
        machine_id: 'pump01', plant: 'plant01', alarm_level: 'WARNING',
        motor_state: 2, velocity_data_valid: true, velocity_rms_overall: 5.2,
        health_score: -1, rpm_valid: true,
    },
    C_RUNNING_valid_CRITICAL: {
        machine_id: 'pump01', plant: 'plant01', alarm_level: 'CRITICAL',
        motor_state: 2, velocity_data_valid: true, velocity_rms_overall: 8.9,
        health_score: -1, rpm_valid: true,
    },
    D_RUNNING_unavailable: {
        machine_id: 'pump01', plant: 'plant01', alarm_level: 'CRITICAL',
        motor_state: 2, velocity_data_valid: false,
        health_score: -1, rpm_valid: true,
    },
    E_STOPPED: {
        machine_id: 'pump01', plant: 'plant01', alarm_level: 'NORMAL',
        motor_state: 0, velocity_data_valid: false,
        health_score: -1, rpm_valid: true,
    },
    F_RUNNING_rpmInvalid_vibValid: {
        machine_id: 'pump01', plant: 'plant01', alarm_level: 'NORMAL',
        motor_state: 2, velocity_data_valid: true, velocity_rms_overall: 0.48,
        health_score: -1, rpm_valid: false,   // <-- RPM invalid, everything else identical to case A
    },
};

const PLANT_LINE_TOKENS = { plant01: 'TEST-TOKEN-NOT-REAL-NEVER-SENT' };
const PLANT_GROUP_IDS = { plant01: 'TEST-GROUP-NOT-REAL-NEVER-SENT' };

const results = {};

function runOne(name, payload, alertType, initialMachines) {
    const flowStore = {
        PLANT_LINE_TOKENS, PLANT_GROUP_IDS,
        machines: initialMachines || {},
    };
    const flowCtx = makeFlowContext(flowStore);

    // Step 1: Health Logic
    const healthRec = makeNodeRecorder();
    const msg1 = { payload: JSON.parse(JSON.stringify(payload)) };
    const afterHealth = runFunctionNode(healthLogicSrc, msg1, flowCtx, healthRec);

    if (afterHealth === null) {
        return { dropped: true, healthWarnings: healthRec._warnings };
    }

    // Step 2: simulate what the (untouched) Unified State Engine would set,
    // WITHOUT executing that node's own code (it is explicitly out of
    // scope / must not be modified or exercised as "the thing under test"
    // -- we only need its documented, unchanged output contract: it sets
    // msg._machineId, msg._alertType, and passes msg.payload through
    // untouched). This mirrors its real behavior exactly, verified against
    // the live node's source in the prior audits, without re-implementing
    // or altering its logic.
    afterHealth._machineId = afterHealth.payload.machine_id;
    afterHealth._alertType = alertType;

    // Step 3: LINE Message Builder (THE NODE UNDER TEST, patched version)
    const lineRec = makeNodeRecorder();
    const afterLine = runFunctionNode(lineBuilderSrc, afterHealth, flowCtx, lineRec);

    return {
        dropped: false,
        healthWarnings: healthRec._warnings,
        vibration_measure_status: afterHealth.payload.vibration_measure_status,
        alarm_verdict_live: afterHealth.payload.alarm_verdict_live,
        lineResult: afterLine, // null if snoozed/backoff/no-token (not expected here)
        lineText: afterLine && afterLine.payload && afterLine.payload.messages
            ? afterLine.payload.messages[0].text : null,
    };
}

// A, B, C, D, F -> alertType 'alert' (fresh CRITICAL/WARNING/NORMAL "alert" framing
// path is what the Unified State Engine would tag any *first* transition into a
// non-NORMAL... but per the real code, NORMAL transitions are tagged 'recovery' and
// non-NORMAL are tagged 'alert'. We mirror that exactly here, not inventing new logic.)
for (const [name, payload] of Object.entries(cases)) {
    const alertType = (payload.alarm_level === 'NORMAL') ? 'recovery' : 'alert';
    results[name] = runOne(name, payload, alertType);
}

// G: CRITICAL -> automatic recovery -> NORMAL, run as a genuine two-step
// sequence sharing the same flow context, exactly like two consecutive
// real MQTT messages would.
(function runCaseG() {
    const flowStore = { PLANT_LINE_TOKENS, PLANT_GROUP_IDS, machines: {} };
    const flowCtx = makeFlowContext(flowStore);

    // Step G1: CRITICAL, vibration unavailable (matches the forensic trace's
    // most probable reconstruction of the real 11:35:20 event)
    const critPayload = {
        machine_id: 'pump01', plant: 'plant01', alarm_level: 'CRITICAL',
        motor_state: 2, velocity_data_valid: false, health_score: -1,
    };
    const h1 = runFunctionNode(healthLogicSrc, { payload: critPayload }, flowCtx, makeNodeRecorder());
    h1._machineId = h1.payload.machine_id;
    h1._alertType = 'alert';
    const l1 = runFunctionNode(lineBuilderSrc, h1, flowCtx, makeNodeRecorder());

    // Step G2: NORMAL recovery ~53s later, vibration STILL unavailable at
    // this exact instant (the specific, disputed shape from the real
    // incident) -- proves the patch's behavior for the actual reported case.
    const normPayload = {
        machine_id: 'pump01', plant: 'plant01', alarm_level: 'NORMAL',
        motor_state: 2, velocity_data_valid: false, health_score: -1,
    };
    const h2 = runFunctionNode(healthLogicSrc, { payload: normPayload }, flowCtx, makeNodeRecorder());
    h2._machineId = h2.payload.machine_id;
    h2._alertType = 'recovery';
    const l2 = runFunctionNode(lineBuilderSrc, h2, flowCtx, makeNodeRecorder());

    results['G_CRITICAL_to_recovery_NORMAL'] = {
        step1_alert_text: l1.payload.messages[0].text,
        step1_verdict_live: h1.payload.alarm_verdict_live,
        step2_recovery_text: l2.payload.messages[0].text,
        step2_verdict_live: h2.payload.alarm_verdict_live,
    };
})();

console.log(JSON.stringify(results, null, 2));
