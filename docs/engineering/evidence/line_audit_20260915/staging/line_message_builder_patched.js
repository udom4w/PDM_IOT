// =============================================
// LINE MESSAGE BUILDER — Multi-Tenant v2
// Messaging API — lookup token + groupId ต่อ plant
// =============================================

var tokens = flow.get('PLANT_LINE_TOKENS') || {};
var groupIds = flow.get('PLANT_GROUP_IDS') || {};
var p = msg.payload;
var plant = (msg.plant || p.plant || 'UNKNOWN').toLowerCase();
var mId = msg._machineId || (p.machine_id || 'N/A').toUpperCase();
var alertType = msg._alertType || 'alert';

// [Verdict-Fix-1] Authoritative live-verdict + Motor State, read from the
// field Health Logic now computes/carries. Absent (e.g. the synthetic
// Escalation Timer payload, which bypasses Health Logic) is treated as
// "not live" -- never as a false positive.
var verdictLive = p.alarm_verdict_live === true;
var MOTOR_STATE_NAMES = { 0: 'STOPPED', 1: 'STARTING', 2: 'RUNNING', 3: 'STOPPING' };
var motorStateText = MOTOR_STATE_NAMES[Number(p.motor_state)] || 'UNKNOWN';

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

// --- ตรวจ snooze / backoff ---
var machines = flow.get('machines') || {};
var m = machines[mId];

if (m) {
    var now = Date.now();

    if (now < m.snoozeUntil || now < m.backoffUntil) return null;

    m.lastAlertSentAt = now;
    flow.set('machines', machines);
}

// --- เวลาไทย ---
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

// [Notify-Fix-1]
// LINE presentation uses canonical FIFO-DSP velocity_rms_overall.
// Legacy p.rms remains untouched elsewhere in the flow.
// Never fabricate a velocity value when FIFO-DSP is unavailable.
var velRms =
    (p.velocity_data_valid === true &&
        typeof p.velocity_rms_overall === 'number')
        ? p.velocity_rms_overall.toFixed(2) + ' mm/s (Source: FIFO-DSP)'
        : 'N/A (FIFO-DSP unavailable)';

var text = '';

if (alertType === 'alert') {

    var emoji = p.alarm_level === 'CRITICAL' ? '🚨' : '⚠️';

    // [Verdict-Fix-1] Never present alarm_level as a clean, confirmed
    // measurement-backed alert unless verdictLive is true. When it is not
    // (vibration unavailable and/or motor not confirmed RUNNING), the
    // header explicitly says the condition is unconfirmed instead of a
    // plain "แจ้งเตือน <LEVEL>!" framing.
    var alertHeader = verdictLive
        ? (emoji + ' แจ้งเตือน ' + p.alarm_level + '!')
        : (emoji + ' ' + p.alarm_level +
           ' (ยืนยันสภาพจริงไม่ได้ — ข้อมูล vibration ไม่พร้อมใช้งาน / unconfirmed — vibration data unavailable)');

    text = alertHeader + '\n'
        + '🏭 Plant: ' + plant + '\n'
        + '⚙️ Machine: ' + mId + '\n'
        + '⚙️ Motor State: ' + motorStateText + '\n'
        + '📊 Velocity RMS: ' + velRms + '\n'
        + '⏰ ' + timeThai + '\n\n'
        + '📖 Commands:\n'
        + '  ack ' + mId + '\n'
        + '  snooze ' + mId + ' 30';

} else if (alertType === 'escalation') {

    text = '🔴 ESCALATION!\n'
        + '🏭 Plant: ' + plant + '\n'
        + '⚙️ Machine: ' + mId + '\n'
        + '⏱️ CRITICAL มานาน: ' + (msg._durationMin || '?') + ' นาที\n'
        + '⏰ ' + timeThai + '\n\n'
        + '⚠️ กรุณาตรวจสอบด่วน!';

} else if (alertType === 'recovery') {

    // [Verdict-Fix-1] Never say "กลับสู่ NORMAL" unless verdictLive is
    // true. The automatic recovery TRANSITION itself (when it fires, and
    // how often) is entirely unchanged -- only the wording of what this
    // specific message claims about the vibration condition changes.
    var recoveryHeader = verdictLive
        ? '✅ กลับสู่ NORMAL'
        : '⚠️ NORMAL (ยืนยันสภาพจริงไม่ได้ — ข้อมูล vibration ไม่พร้อมใช้งาน / unconfirmed — vibration data unavailable)';

    text = recoveryHeader + '\n'
        + '🏭 Plant: ' + plant + '\n'
        + '⚙️ Machine: ' + mId + '\n'
        + '⚙️ Motor State: ' + motorStateText + '\n'
        + '📊 Velocity RMS: ' + velRms + '\n'
        + '⏰ ' + timeThai;
}

// --- ส่ง Messaging API ---
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
