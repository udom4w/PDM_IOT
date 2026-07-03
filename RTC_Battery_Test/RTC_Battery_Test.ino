// ============================================================
//  RTC DS3231 Battery Backup Test
//  ทดสอบเฉพาะ battery backup — ไม่เขียนเวลาใหม่ทุก boot
//  SDA=GPIO44, SCL=GPIO43
// ============================================================

#include <Wire.h>
#include <RTClib.h>

#define I2C_SDA_PIN  44
#define I2C_SCL_PIN  43

RTC_DS3231 rtc;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  Serial.println("================================================");
  Serial.println("  DS3231 Battery Backup Test");
  Serial.println("================================================");

  if (!rtc.begin()) {
    Serial.println("!! ไม่พบ DS3231");
    return;
  }

  // ตรวจ lostPower ก่อน
  bool lost = rtc.lostPower();
  Serial.printf("[lostPower] = %s\n", lost ? "YES — battery ไม่ทำงาน" : "NO  — battery ดี");

  DateTime now = rtc.now();
  Serial.printf("[เวลาปัจจุบัน] %04d-%02d-%02d %02d:%02d:%02d\n",
    now.year(), now.month(), now.day(),
    now.hour(), now.minute(), now.second());

  // ถ้า lostPower หรือปีผิด → เซตเวลาใหม่ครั้งเดียว
  if (lost || now.year() < 2024 || now.year() > 2035) {
    Serial.println("[ACTION] เซตเวลาใหม่ → 2026-06-22 12:00:00");
    rtc.adjust(DateTime(2026, 6, 22, 12, 0, 0));
  } else {
    Serial.println("[OK] เวลายังอยู่ — ไม่เขียนใหม่");
  }

  Serial.println("------------------------------------------------");
  Serial.println("ถอด USB แล้วเสียบใหม่");
  Serial.println("ถ้าเวลาต่อเนื่อง → battery ดี");
  Serial.println("ถ้าเวลากลับ 12:00:00 → battery มีปัญหา");
  Serial.println("================================================");
}

void loop() {
  DateTime now = rtc.now();
  if (now.year() >= 2024 && now.year() <= 2035) {
    Serial.printf("[RTC] %04d-%02d-%02d %02d:%02d:%02d | Temp=%.2f C\n",
      now.year(), now.month(), now.day(),
      now.hour(), now.minute(), now.second(),
      rtc.getTemperature());
  } else {
    Serial.printf("[RTC] INVALID year=%u\n", now.year());
  }
  delay(1000);
}
