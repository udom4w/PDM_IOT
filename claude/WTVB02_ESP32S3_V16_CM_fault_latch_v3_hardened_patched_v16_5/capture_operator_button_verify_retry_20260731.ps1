# capture_operator_button_verify_retry_20260731.ps1
# Hardware verification capture RETRY for Phase 5 Commit 6 (OPERATOR_BUTTON
# production trigger). No reflash for this run -- board already runs the
# Commit 6 (revised) build from the prior flash. User drives gesture timing
# live after seeing CAPTURE_OPENED confirmed; 600s window to comfortably
# cover 7 scripted gestures including two natural 60s cooldown waits.
param(
  [string]$PortName = "COM5",
  [int]$BaudRate = 115200,
  [string]$LogPath = "C:\Users\HP\Downloads\PDM_IOT\claude\WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5\fifo_capture_analysis\serial_raw_OPERATOR_BUTTON_VERIFY_RETRY_20260731.log",
  [int]$MaxSeconds = 600
)

$port = New-Object System.IO.Ports.SerialPort $PortName, $BaudRate, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
$port.ReadTimeout = 2000
$port.NewLine = "`n"
$port.DtrEnable = $false
$port.RtsEnable = $false

$sw = New-Object System.IO.StreamWriter($LogPath, $false)
$sw.AutoFlush = $true
$startTime = Get-Date
$status = "TIMEOUT"

try {
  $opened = $false
  $openDeadline = (Get-Date).AddSeconds(30)
  while (-not $opened -and (Get-Date) -lt $openDeadline) {
    try { $port.Open(); $opened = $true } catch { Start-Sleep -Milliseconds 200 }
  }
  if (-not $opened) { throw "Could not open $PortName within 30s" }
  $startTime = Get-Date
  $sw.WriteLine("###CAPTURE_OPENED $(Get-Date -Format o) port=$PortName baud=$BaudRate dtr=$($port.DtrEnable) rts=$($port.RtsEnable)")
  while ((New-TimeSpan -Start $startTime -End (Get-Date)).TotalSeconds -lt $MaxSeconds) {
    try { $line = $port.ReadLine() } catch [System.TimeoutException] { continue }
    $sw.WriteLine("$(Get-Date -Format 'HH:mm:ss.fff') $line")
  }
  $status = "DURATION_COMPLETE"
} catch {
  $sw.WriteLine("###CAPTURE_ERROR $($_.Exception.Message)")
  $status = "ERROR"
} finally {
  if ($port.IsOpen) { $port.Close() }
  $sw.WriteLine("###CAPTURE_STATUS=$status elapsedSeconds=$((New-TimeSpan -Start $startTime -End (Get-Date)).TotalSeconds)")
  $sw.Close()
}
Write-Output "STATUS=$status"
