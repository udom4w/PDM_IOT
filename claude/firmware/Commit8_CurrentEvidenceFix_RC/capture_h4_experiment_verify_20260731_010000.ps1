# capture_h3_experiment_verify_20260731_003531.ps1
# Verification capture for the H4 experiment (FIFO-gated 10ms service cadence
# suppressed; PURELISTEN, all other subsystems unchanged from baseline).
# Board was already reset by the arduino-cli upload's own RTS-pin reset
# immediately before this script starts -- no DTR/RTS toggle here, just
# listen. Identical procedure to prior verification captures this session.
param(
  [string]$PortName = "COM5",
  [int]$BaudRate = 115200,
  [string]$LogPath = "C:\Users\HP\Downloads\PDM_IOT\claude\WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5\fifo_capture_analysis\serial_raw_H4_EXPERIMENT_VERIFY_20260731_010000.log",
  [int]$MaxSeconds = 160
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
