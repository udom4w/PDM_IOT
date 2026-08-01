# capture_fifo_dump_TEMP_DIAG.ps1
# Ad hoc, read-only serial capture for the first FIFO acquisition after the
# TEMP_DIAG transport-layer instrumentation (Point A in HandleReceivingImpl,
# Point B in Uart485Transport_OnReceiveError). Does NOT build or flash
# anything -- assumes the instrumented firmware has already been uploaded to
# COM5. Variant of capture_fifo_dump_20260730.ps1 with one additional
# early-stop condition: the "[FIFO-DUMP] result not usable, error=" line
# (the failure-path outcome), in addition to the existing "###FIFO_CSV_END"
# success-path sentinel -- stops on whichever of the two the first
# acquisition actually produces, per the request to stop "immediately" on
# either success or error=2.
param(
  [string]$PortName = "COM5",
  [int]$BaudRate = 115200,
  [string]$LogPath = "C:\Users\HP\Downloads\PDM_IOT\claude\WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5\fifo_capture_analysis\serial_raw_TEMP_DIAG.log",
  [int]$MaxSeconds = 600
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
  $finished = $false
  $finishReason = ""
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
      $finished = $true
      $finishReason = "CSV_COMPLETE"
      # linger briefly to catch the FifoDriver_ReleaseResult() log line, if any
      Start-Sleep -Milliseconds 500
      break
    }
    if ($line -match '\[FIFO-DUMP\] result not usable, error=') {
      $finished = $true
      $finishReason = "ERROR_RESULT"
      # linger briefly to catch any trailing TEMP_DIAG/state lines for this tick
      Start-Sleep -Milliseconds 500
      break
    }
  }
  if ($crashHit) { $status = "CRASH_DETECTED" }
  elseif ($finished) { $status = $finishReason }
  else { $status = "TIMEOUT" }
} catch {
  $sw.WriteLine("###CAPTURE_ERROR $($_.Exception.Message)")
  $status = "ERROR"
} finally {
  if ($port.IsOpen) { $port.Close() }
  $sw.WriteLine("###CAPTURE_STATUS=$status elapsedSeconds=$((New-TimeSpan -Start $startTime -End (Get-Date)).TotalSeconds)")
  $sw.Close()
}

if ($status -eq "CSV_COMPLETE" -or $status -eq "ERROR_RESULT" -or $status -eq "CRASH_DETECTED") { exit 0 } else { exit 1 }
