// =============================================
// LINE MESSAGE BUILDER — Multi-Tenant v2 — SME wording
// [SME-Wording-1] Wording/string-building only, per
// docs/engineering/evidence/line_audit_20260915/LINE_SME_MESSAGE_SPEC_20260915.md
// Messaging API — lookup token + groupId ต่อ plant
// =============================================

var tokens = flow.get('PLANT_LINE_TOKENS') || {};
var groupIds = flow.get('PLANT_GROUP_IDS') || {};
var p = msg.payload;
var plant = (msg.plant || p.plant || 'UNKNOWN').toLowerCase();
var mId = msg._machineId || (p.machine_id || 'N/A').toUpperCase();
var alertType = msg._alertType || 'alert';

// [Verdict-Fix-1] Authoritative live-verdict, read from the field Health
// Logic computes/carries — UNCHANGED. This patch never re-derives or
// alters this value; it only changes how it is worded below.
var verdictLive = p.alarm_verdict_live === true;
var motorCode = Number(p.motor_state);

// [SME-Wording-1] Plain-Thai motor-state vocabulary, replacing the prior
// English "RUNNING"/"STOPPED"/etc. labels. Same motorCode input, same
// four-way mapping — wording only.
var MOTOR_STATE_NAMES_TH = { 0: 'หยุดทำงาน', 1: 'เริ่มทำงาน', 2: 'กำลังทำงาน', 3: 'กำลังหยุด' };
var motorStateText = MOTOR_STATE_NAMES_TH[motorCode] || 'ไม่ทราบสถานะ';

// --- lookup token ตาม plant ---
var accessToken = tokens[plant];

if (!accessToken) {
    node.warn('[LINE] ไม่พบ token สำหรับ plant: ' + plant + ' — ข้ามการส่ง');
    node.status({ fill: 'red', shape: 'dot', text: '❌ ไม่มี token: ' + plant });
    return null;
}

// --- lookup groupId ตาม plant ---
var groupId = groupIds[plant];

if (!groupId) {
    node.warn('[LINE] ไม่พบ groupId สำหรับ plant: ' + plant + ' — ข้ามการส่ง');
    node.status({ fill: 'red', shape: 'dot', text: '❌ ไม่มี groupId: ' + plant });
    return null;
}

// --- ตรวจ snooze / backoff --- (UNCHANGED)
var machines = flow.get('machines') || {};
var m = machines[mId];

if (m) {
    var now = Date.now();

    if (now < m.snoozeUntil || now < m.backoffUntil) return null;

    m.lastAlertSentAt = now;
    flow.set('machines', machines);
}

// --- เวลาไทย --- (UNCHANGED)
var timeThai = new Date().toLocaleString('th-TH', {
    timeZone: 'Asia/Bangkok',
    hour12: false,
    year: 'numeric',
    month: 'short',
    day: 'numeric',
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit'
}) + ' น.';

// [SME-Wording-1] Plain-Thai vibration value text, used only in the
// CONFIRMED states (A/B/C). Same [Notify-Fix-1] guard as before — never
// fabricate a value when FIFO-DSP data is unavailable. "FIFO-DSP" itself
// is no longer surfaced as the primary user-facing explanation anywhere
// in this file (spec requirement 4).
var velValueText =
    (p.velocity_data_valid === true &&
        typeof p.velocity_rms_overall === 'number')
        ? p.velocity_rms_overall.toFixed(2) + ' mm/s'
        : 'ไม่มีข้อมูล';

var commandBlock = '\n\n📖 คำสั่ง:\n  ack ' + mId + '\n  snooze ' + mId + ' 30';

var text = '';

if (alertType === 'escalation') {

    // UNCHANGED — out of scope for this spec, escalation timer behavior
    // and wording untouched.
    text = '🔴 ESCALATION!\n'
        + '🏭 Plant: ' + plant + '\n'
        + '⚙️ Machine: ' + mId + '\n'
        + '⏱️ CRITICAL มานาน: ' + (msg._durationMin || '?') + ' นาที\n'
        + '⏰ ' + timeThai + '\n\n'
        + '⚠️ กรุณาตรวจสอบด่วน!';

} else if (!verdictLive) {

    // [SME-Wording-1] States D (VIBRATION UNAVAILABLE) and E (STOPPED).
    // Checked BEFORE the alertType-specific branches so this applies
    // identically whether the payload arrived tagged 'alert' or
    // 'recovery' — the fact being reported (no confirmed vibration
    // reading) is the same either way. Neither branch below can ever
    // produce the word "ปกติ" (normal) describing vibration — this is
    // the direct implementation of "VIBRATION UNAVAILABLE must NEVER be
    // described as NORMAL" and "STOPPED must not claim vibration is
    // normal when no valid measurement exists."
    if (motorCode === 2) {
        // D — machine confirmed RUNNING, but vibration cannot be confirmed.
        text = '⚠️ ไม่สามารถยืนยันสภาพการสั่นสะเทือนได้ในขณะนี้\n\n'
            + '🏭 โรงงาน: ' + plant + '\n'
            + '⚙️ เครื่อง: ' + mId + '\n\n'
            + '🔎 เกิดอะไรขึ้น:\n'
            + 'ระบบไม่สามารถอ่านค่าการสั่นสะเทือนจากเซนเซอร์ได้ในขณะนี้ ขณะที่เครื่องยังกำลังทำงานอยู่ '
            + 'จึงยังไม่สามารถยืนยันได้ว่าเครื่องอยู่ในสภาพปกติหรือไม่\n\n'
            + '📌 ตอนนี้:\n'
            + 'ค่าการสั่นสะเทือน: ไม่มีข้อมูล\n'
            + 'สถานะเครื่อง: ' + motorStateText + '\n\n'
            + '🔧 ควรทำอะไร:\n'
            + 'แนะนำให้ตรวจสอบการเชื่อมต่อของเซนเซอร์ หากสังเกตเห็นความผิดปกติของเครื่องด้วยวิธีอื่น (เสียง ความร้อน) ให้ตรวจสอบเครื่องเพิ่มเติม\n\n'
            + '⏰ ' + timeThai
            + commandBlock;
    } else {
        // E — machine not running (STOPPED/STARTING/STOPPING): no
        // measurement exists, and this is never described as an error or
        // as a normal vibration reading.
        text = 'ℹ️ เครื่องหยุดทำงาน\n\n'
            + '🏭 โรงงาน: ' + plant + '\n'
            + '⚙️ เครื่อง: ' + mId + '\n\n'
            + '🔎 เกิดอะไรขึ้น:\n'
            + 'เครื่องนี้อยู่ในสถานะหยุดทำงาน จึงไม่มีการวัดค่าการสั่นสะเทือนในขณะนี้ '
            + '(เป็นสถานะปกติของเครื่องที่หยุดทำงาน ไม่ใช่ความผิดปกติของระบบ)\n\n'
            + '📌 ตอนนี้:\n'
            + 'ค่าการสั่นสะเทือน: ไม่มีการวัด (เครื่องหยุดทำงาน)\n'
            + 'สถานะเครื่อง: ' + motorStateText + '\n\n'
            + '🔧 ควรทำอะไร:\n'
            + 'ไม่ต้องดำเนินการ ระบบจะเริ่มตรวจสอบค่าการสั่นสะเทือนอีกครั้งเมื่อเครื่องเริ่มทำงาน\n\n'
            + '⏰ ' + timeThai;
    }

} else if (alertType === 'alert') {

    // A (WARNING) / B (CRITICAL) — reached only when verdictLive === true:
    // motor confirmed RUNNING and vibration data confirmed genuinely
    // valid. alarm_level itself is read verbatim, unchanged — no new
    // severity calculation.
    var isCritical = p.alarm_level === 'CRITICAL';
    var title = isCritical
        ? '🚨 แจ้งเตือน: พบการสั่นสะเทือนระดับวิกฤต'
        : '⚠️ แจ้งเตือน: พบการสั่นสะเทือนสูงกว่าปกติ';
    var whatHappened = isCritical
        ? 'ระบบตรวจพบค่าการสั่นสะเทือนของเครื่องนี้สูงถึงระดับวิกฤต ซึ่งอาจเป็นสัญญาณของความเสียหายที่กำลังเกิดขึ้น'
        : 'ระบบตรวจพบค่าการสั่นสะเทือนของเครื่องนี้สูงกว่าระดับปกติ ขณะที่เครื่องกำลังทำงานอยู่';
    var whatToDo = isCritical
        ? 'ตรวจสอบเครื่องโดยด่วน หากเป็นไปได้ให้พิจารณาหยุดเครื่องเพื่อตรวจสอบก่อนเดินเครื่องต่อ'
        : 'เฝ้าระวังอย่างใกล้ชิด แนะนำให้ตรวจสอบเครื่องในรอบตรวจถัดไป';

    text = title + '\n\n'
        + '🏭 โรงงาน: ' + plant + '\n'
        + '⚙️ เครื่อง: ' + mId + '\n\n'
        + '🔎 เกิดอะไรขึ้น:\n'
        + whatHappened + '\n\n'
        + '📌 ตอนนี้:\n'
        + 'ค่าการสั่นสะเทือน: ' + velValueText + '\n'
        + 'สถานะเครื่อง: ' + motorStateText + '\n\n'
        + '🔧 ควรทำอะไร:\n'
        + whatToDo + '\n\n'
        + '⏰ ' + timeThai
        + commandBlock;

} else if (alertType === 'recovery') {

    // C — reached only when verdictLive === true: vibration confirmed
    // genuinely OK AND motor confirmed RUNNING at the recovery instant.
    // When verdictLive is false, execution never reaches here — it was
    // already handled by the D/E branch above. This is the structural
    // guarantee (unchanged from [Verdict-Fix-1], preserved by this
    // patch) that prevents "กลับสู่ NORMAL" from ever being paired with
    // an unavailable/unconfirmed reading (spec requirement 5 / test I).
    text = '✅ กลับสู่สถานะปกติ\n\n'
        + '🏭 โรงงาน: ' + plant + '\n'
        + '⚙️ เครื่อง: ' + mId + '\n\n'
        + '🔎 เกิดอะไรขึ้น:\n'
        + 'ค่าการสั่นสะเทือนของเครื่องนี้กลับสู่ระดับปกติแล้ว หลังจากที่เคยตรวจพบความผิดปกติก่อนหน้านี้\n\n'
        + '📌 ตอนนี้:\n'
        + 'ค่าการสั่นสะเทือน: ' + velValueText + '\n'
        + 'สถานะเครื่อง: ' + motorStateText + '\n\n'
        + '🔧 ควรทำอะไร:\n'
        + 'ไม่ต้องดำเนินการเพิ่มเติม ระบบจะเฝ้าระวังค่าการสั่นสะเทือนต่อเนื่อง\n\n'
        + '⏰ ' + timeThai;
}

// --- ส่ง Messaging API --- (UNCHANGED)
msg._machineId = mId;

msg.headers = {
    'Content-Type': 'application/json',
    'Authorization': 'Bearer ' + accessToken
};

msg.payload = {
    to: groupId,
    messages: [{ type: 'text', text: text }]
};

node.status({
    fill: 'blue',
    shape: 'dot',
    text: '[' + plant + '] [' + alertType + '] → ' + mId
});

return msg;
