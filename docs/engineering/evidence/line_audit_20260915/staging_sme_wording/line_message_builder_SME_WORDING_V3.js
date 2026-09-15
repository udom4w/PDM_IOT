// =============================================
// LINE MESSAGE BUILDER — Multi-Tenant v2 — SME wording (V3)
// [SME-Wording-2] Wording/string-building only. Real-production-facing
// language pass, per operator review of live output: lead with the
// plain-language finding, not the sensor reading; never speculate about
// "damage in progress"; SME technician vocabulary throughout.
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

// [SME-Wording-1] Plain-Thai motor-state vocabulary — UNCHANGED from V2.
var MOTOR_STATE_NAMES_TH = { 0: 'หยุดทำงาน', 1: 'เริ่มทำงาน', 2: 'กำลังทำงาน', 3: 'กำลังหยุด' };
var motorStateText = MOTOR_STATE_NAMES_TH[motorCode] || 'ไม่ทราบสถานะ';

// --- lookup token ตาม plant --- (UNCHANGED)
var accessToken = tokens[plant];

if (!accessToken) {
    node.warn('[LINE] ไม่พบ token สำหรับ plant: ' + plant + ' — ข้ามการส่ง');
    node.status({ fill: 'red', shape: 'dot', text: '❌ ไม่มี token: ' + plant });
    return null;
}

// --- lookup groupId ตาม plant --- (UNCHANGED)
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

// [SME-Wording-1] Plain-Thai vibration value text — UNCHANGED guard from
// V2/[Notify-Fix-1]: never fabricate a value when data is unavailable.
// The real payload value is always used verbatim; no threshold number
// (2.1 / 4.5 mm/s) is ever hard-coded into this or any message text.
var velValueText =
    (p.velocity_data_valid === true &&
        typeof p.velocity_rms_overall === 'number')
        ? p.velocity_rms_overall.toFixed(2) + ' mm/s'
        : 'ไม่มีข้อมูล';

var commandBlock = '\n\n📖 คำสั่ง:\n  ack ' + mId + '\n  snooze ' + mId + ' 30';

var text = '';

if (alertType === 'escalation') {

    // UNCHANGED — out of scope, escalation timer behavior and wording
    // untouched.
    text = '🔴 ESCALATION!\n'
        + '🏭 Plant: ' + plant + '\n'
        + '⚙️ Machine: ' + mId + '\n'
        + '⏱️ CRITICAL มานาน: ' + (msg._durationMin || '?') + ' นาที\n'
        + '⏰ ' + timeThai + '\n\n'
        + '⚠️ กรุณาตรวจสอบด่วน!';

} else if (!verdictLive) {

    // States D (VIBRATION UNAVAILABLE) and E (STOPPED). Checked BEFORE
    // the alertType-specific branches so this applies identically
    // whether the payload arrived tagged 'alert' or 'recovery' —
    // UNCHANGED structural guarantee from V1/V2. Neither branch below
    // can ever produce the word "ปกติ" describing vibration.
    if (motorCode === 2) {
        // D — machine confirmed RUNNING, but vibration cannot be
        // confirmed. [SME-Wording-2] Title and body reworded per the
        // operator-approved V3 target text; explicitly still refuses to
        // conclude the machine is in a normal condition.
        text = '❓ ยังไม่สามารถประเมินการสั่นสะเทือนได้\n\n'
            + '🏭 โรงงาน: ' + plant + '\n'
            + '⚙️ เครื่อง: ' + mId + '\n\n'
            + '🔎 เกิดอะไรขึ้น:\n'
            + 'ระบบยังไม่ได้รับข้อมูลการสั่นสะเทือนที่พร้อมใช้งาน\n\n'
            + '📌 ตอนนี้:\n'
            + 'ค่าการสั่นสะเทือน: ไม่พร้อมใช้งาน\n'
            + 'สถานะเครื่อง: ' + motorStateText + '\n\n'
            + '🔧 ควรทำอะไร:\n'
            + 'ตรวจสอบระบบวัดและรอข้อมูลกลับมา\n'
            + 'ยังไม่ควรสรุปว่าเครื่องอยู่ในภาวะปกติ\n\n'
            + '⏰ ' + timeThai
            + commandBlock;
    } else {
        // E — machine not running (STOPPED/STARTING/STOPPING): no
        // measurement exists. [SME-Wording-2] Simplified to match the
        // operator-approved V3 target: no vibration-value line at all
        // (nothing to report), motor-state line only.
        text = '⏹️ เครื่องหยุดทำงาน\n\n'
            + '🏭 โรงงาน: ' + plant + '\n'
            + '⚙️ เครื่อง: ' + mId + '\n\n'
            + '🔎 เกิดอะไรขึ้น:\n'
            + 'เครื่องไม่ได้กำลังทำงาน จึงไม่มีการวัดการสั่นสะเทือนในขณะนี้\n\n'
            + '📌 ตอนนี้:\n'
            + 'สถานะเครื่อง: ' + motorStateText + '\n\n'
            + '🔧 ควรทำอะไร:\n'
            + 'เมื่อเครื่องกลับมาทำงาน ระบบจะเริ่มประเมินการสั่นสะเทือนอีกครั้ง\n\n'
            + '⏰ ' + timeThai;
    }

} else if (alertType === 'alert') {

    // WARNING / CRITICAL — reached only when verdictLive === true: motor
    // confirmed RUNNING and vibration data confirmed genuinely valid.
    // alarm_level itself is read verbatim, unchanged — no new severity
    // calculation. [SME-Wording-2] Wording reworded to lead with the
    // plain-language finding rather than the sensor value, and to never
    // speculate that damage is actively occurring from a single reading
    // — CRITICAL now states only that a critical-level reading was
    // detected, and recommends inspection/shutdown per safety procedure,
    // not a conclusion about damage.
    var isCritical = p.alarm_level === 'CRITICAL';
    var title = isCritical
        ? '🚨 พบการสั่นสะเทือนระดับวิกฤต'
        : '⚠️ พบการสั่นสะเทือนสูงกว่าระดับเฝ้าระวัง';
    var whatHappened = isCritical
        ? 'ระบบตรวจพบการสั่นสะเทือนถึงระดับวิกฤต'
        : 'ระบบตรวจพบการสั่นสะเทือนสูงกว่าระดับเฝ้าระวัง';
    var whatToDo = isCritical
        ? 'ตรวจสอบเครื่องโดยด่วน\nและพิจารณาหยุดเครื่องตามขั้นตอนความปลอดภัยก่อนเดินเครื่องต่อ'
        : 'ติดตามค่าการสั่นสะเทือนต่อเนื่อง\nและตรวจสอบเครื่องในรอบตรวจถัดไป';

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

    // NORMAL / RECOVERY — reached only when verdictLive === true:
    // vibration confirmed genuinely OK AND motor confirmed RUNNING at
    // the recovery instant. When verdictLive is false, execution never
    // reaches here — already handled by the D/E branch above. UNCHANGED
    // structural guarantee (from [Verdict-Fix-1]) that prevents a
    // confirmed-recovery header from ever pairing with an
    // unavailable/unconfirmed reading. [SME-Wording-2] Wording reworded
    // per the operator-approved V3 target text.
    text = '✅ การสั่นสะเทือนกลับสู่ระดับปกติ\n\n'
        + '🏭 โรงงาน: ' + plant + '\n'
        + '⚙️ เครื่อง: ' + mId + '\n\n'
        + '🔎 เกิดอะไรขึ้น:\n'
        + 'ค่าการสั่นสะเทือนของเครื่องกลับเข้าสู่ระดับปกติ\nหลังจากก่อนหน้านี้ตรวจพบค่าที่สูงขึ้น\n\n'
        + '📌 ตอนนี้:\n'
        + 'ค่าการสั่นสะเทือน: ' + velValueText + '\n'
        + 'สถานะเครื่อง: ' + motorStateText + '\n\n'
        + '🔧 ควรทำอะไร:\n'
        + 'ไม่ต้องดำเนินการเพิ่มเติม\nระบบจะเฝ้าระวังค่าการสั่นสะเทือนต่อเนื่อง\n\n'
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
