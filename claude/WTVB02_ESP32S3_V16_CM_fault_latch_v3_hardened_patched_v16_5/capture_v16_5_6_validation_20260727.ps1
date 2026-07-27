param(
  [string]$PortName = "COM5",
  [int]$BaudRate = 115200,
  [string]$LogPath = "C:\Users\HP\Downloads\PDM_IOT\claude\WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5\serial_capture_v16_5_6_validation_20260727.log",
  [int]$MaxSeconds = 330
)

# Early-stop crash signatures. Deliberately NARROW -- this firmware's normal
# application logs contain the bare word "Reset" in several benign lines
# (e.g. "[MOTOR] STOPPED transition -- peak hold reset", "[TLS] Context
# reset complete", "[MAINT] Reset complete -> WARMUP"), so matching on plain
# "Reset" here would trigger a false-positive stop on ordinary operation.
# These patterns instead match actual ESP32 ROM/panic-handler output.
# Same convention as capture_stopping_investigation_20260718.ps1.
$crashPatterns = @(
  'Guru Meditation',
  'Brownout detector',
  '^rst:0x',
  '^ets [A-Z][a-z]{2} +\d',   # ROM banner, e.g. "ets Jul 29 2019 12:21:46"
  'Rebooting',
  'Backtrace:',
  'abort\(\) was called',
  'Task watchdog got triggered',
  'task_wdt',
  '(TG0|TG1|RTC|INT)WDT'
  # NOTE: '[CONFIG] Press ENTER within 5 seconds' deliberately NOT included here
  # (unlike capture_stopping_investigation_20260718.ps1) -- this capture starts
  # immediately after flashing, so that banner is the expected first-boot
  # prompt, not evidence of a mid-session reboot. A genuine mid-session reboot
  # is still caught by the ROM/panic patterns above.
)

# Passive capture only. DTR/RTS are set exactly once, before Open(), and are
# never touched again for the lifetime of this script -- device is already
# running the freshly-flashed v16.5.6 firmware from the upload step.

$port = New-Object System.IO.Ports.SerialPort $PortName, $BaudRate, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
$port.ReadTimeout = 2000
$port.NewLine = "`n"
$port.DtrEnable = $true
$port.RtsEnable = $false

$sw = New-Object System.IO.StreamWriter($LogPath, $false)
$sw.AutoFlush = $true

$startTime = Get-Date
$status = "TIMEOUT"

try {
  $opened = $false
  $openDeadline = (Get-Date).AddSeconds(120)
  while (-not $opened -and (Get-Date) -lt $openDeadline) {
    try {
      $port.Open()
      $opened = $true
    } catch {
      Start-Sleep -Milliseconds 200
    }
  }
  if (-not $opened) {
    throw "Could not open $PortName within 120s (port busy or not present)"
  }

  $sw.WriteLine("###CAPTURE_OPENED $(Get-Date -Format o) port=$PortName baud=$BaudRate dtr=$($port.DtrEnable) rts=$($port.RtsEnable)")

  $crashHit = $false
  while ((New-TimeSpan -Start $startTime -End (Get-Date)).TotalSeconds -lt $MaxSeconds) {
    try {
      $line = $port.ReadLine()
    } catch [System.TimeoutException] {
      continue
    }
    $sw.WriteLine("$(Get-Date -Format 'HH:mm:ss.fff') $line")
    foreach ($pat in $crashPatterns) {
      if ($line -match $pat) {
        $sw.WriteLine("###CRASH_PATTERN_MATCHED pattern=`"$pat`" line=`"$line`"")
        $crashHit = $true
        break
      }
    }
    if ($crashHit) { break }
  }
  $status = if ($crashHit) { "CRASH_DETECTED" } else { "COMPLETE" }
} catch {
  $sw.WriteLine("###CAPTURE_ERROR $($_.Exception.Message)")
  $status = "ERROR"
} finally {
  if ($port.IsOpen) { $port.Close() }
  $sw.WriteLine("###CAPTURE_STATUS=$status elapsedSeconds=$((New-TimeSpan -Start $startTime -End (Get-Date)).TotalSeconds)")
  $sw.Close()
}

if ($status -eq "COMPLETE" -or $status -eq "CRASH_DETECTED") { exit 0 } else { exit 1 }
