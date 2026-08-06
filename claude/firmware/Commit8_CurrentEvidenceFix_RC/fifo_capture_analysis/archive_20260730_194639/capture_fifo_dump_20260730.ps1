# capture_fifo_dump_20260730.ps1
# Ad hoc, read-only serial capture for the TEMP-DEBUG FIFO CSV dump added to
# taskModbusRead() (see the "[TEMP-DEBUG -- ad hoc, not part of the CM-100
# Implementation Plan]" block in the .ino). Does NOT build or flash anything
# -- assumes the modified firmware has already been uploaded to COM5 by the
# user. Mirrors this project's existing capture_*.ps1 convention
# (System.IO.Ports.SerialPort, timestamped lines, explicit crash-signature
# early stop) and adds one more early-stop condition: the
# "###FIFO_CSV_END" sentinel the new debug block prints once the one-shot
# capture's samples have been fully written to Serial.
param(
  [string]$PortName = "COM5",
  [int]$BaudRate = 115200,
  [string]$LogPath = "C:\Users\HP\Downloads\PDM_IOT\claude\WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5\fifo_capture_analysis\serial_raw_20260730.log",
  [int]$MaxSeconds = 400
)

$crashPatterns = @(
  'Guru Meditation',
  'Brownout detector',
  '^ets [A-Z][a-z]{2} +\d',
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
  $csvDone = $false
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
    if ($line -match '###FIFO_CSV_END') {
      $csvDone = $true
      # linger briefly to catch the FifoDriver_ReleaseResult() log line, if any
      Start-Sleep -Milliseconds 500
      break
    }
  }
  if ($crashHit) { $status = "CRASH_DETECTED" }
  elseif ($csvDone) { $status = "CSV_COMPLETE" }
  else { $status = "TIMEOUT" }
} catch {
  $sw.WriteLine("###CAPTURE_ERROR $($_.Exception.Message)")
  $status = "ERROR"
} finally {
  if ($port.IsOpen) { $port.Close() }
  $sw.WriteLine("###CAPTURE_STATUS=$status elapsedSeconds=$((New-TimeSpan -Start $startTime -End (Get-Date)).TotalSeconds)")
  $sw.Close()
}

if ($status -eq "CSV_COMPLETE" -or $status -eq "CRASH_DETECTED") { exit 0 } else { exit 1 }
