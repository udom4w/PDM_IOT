# capture_target_COM5_20260730_183449.ps1
# Target ESP32 Serial capture, paired session with the sniffer capture on
# COM4. DTR toggles on open, which resets the target board (no reflash --
# firmware is frozen/unchanged; this only power-cycles what's already
# flashed) so a fresh Task 4.4 one-shot FIFO trigger can fire.
param(
  [string]$PortName = "COM5",
  [int]$BaudRate = 115200,
  [string]$LogPath = "C:\Users\HP\Downloads\PDM_IOT\claude\WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5\fifo_capture_analysis\serial_raw_SNIFFER_VALIDATION_20260730_183449.log",
  [int]$MaxSeconds = 170
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
  $openDeadline = (Get-Date).AddSeconds(60)
  while (-not $opened -and (Get-Date) -lt $openDeadline) {
    try {
      $port.Open()
      $opened = $true
    } catch {
      Start-Sleep -Milliseconds 200
    }
  }
  if (-not $opened) {
    throw "Could not open $PortName within 60s (port busy or not present)"
  }

  $startTime = Get-Date
  $sw.WriteLine("###CAPTURE_OPENED $(Get-Date -Format o) port=$PortName baud=$BaudRate dtr=$($port.DtrEnable) rts=$($port.RtsEnable)")

  while ((New-TimeSpan -Start $startTime -End (Get-Date)).TotalSeconds -lt $MaxSeconds) {
    try {
      $line = $port.ReadLine()
    } catch [System.TimeoutException] {
      continue
    }
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
