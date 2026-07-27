param(
  [string]$PortName = "COM5",
  [int]$BaudRate = 115200,
  [string]$LogPath = "C:\Users\HP\Downloads\PDM_IOT\claude\WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5\serial_capture_p4_02_verification_run2_20260727.log",
  [int]$MaxSeconds = 700
)

# Early-stop crash signatures for THIS run only. '^rst:0x' is deliberately
# EXCLUDED here (unlike capture_v16_5_6_validation_20260727.ps1) because this
# specific session opens the port right after the Arduino IDE released COM5,
# and a diagnostic probe just confirmed opening a fresh connection triggers a
# normal ESP32-S3 "rst:0x15 (USB_UART_CHIP_RESET)" connect-reset -- expected
# DTR-triggered behavior, not a firmware fault, and it doubles as this run's
# genuine Test 1 (power-on) event. A REAL mid-session crash/panic is still
# caught by the remaining unambiguous signatures below (Guru Meditation,
# Backtrace, task_wdt, Brownout detector, abort()) -- none of those are ever
# produced by a benign connect-reset.
$crashPatterns = @(
  'Guru Meditation',
  'Brownout detector',
  '^ets [A-Z][a-z]{2} +\d',   # ROM banner, e.g. "ets Jul 29 2019 12:21:46"
  'Backtrace:',
  'abort\(\) was called',
  'Task watchdog got triggered',
  'task_wdt',
  '(TG0|TG1|RTC|INT)WDT'
)

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
