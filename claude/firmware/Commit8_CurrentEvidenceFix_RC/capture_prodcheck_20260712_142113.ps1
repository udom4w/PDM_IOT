param(
  [string]$PortName = "COM5",
  [int]$BaudRate = 115200,
  [string]$LogPath = "C:\Users\HP\Downloads\PDM_IOT\claude\WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5\diag_prodcheck_20260712_142113.log",
  [int]$MaxSeconds = 180
)

# Bounded passive production-sanity capture only. DTR/RTS set exactly once,
# before Open(), never touched again. No reboot trigger of any kind.
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
  $port.Open()
  $sw.WriteLine("###CAPTURE_OPENED $(Get-Date -Format o) port=$PortName baud=$BaudRate dtr=$($port.DtrEnable) rts=$($port.RtsEnable)")

  while ((New-TimeSpan -Start $startTime -End (Get-Date)).TotalSeconds -lt $MaxSeconds) {
    try {
      $line = $port.ReadLine()
    } catch [System.TimeoutException] {
      continue
    }
    $sw.WriteLine($line)
  }
  $status = "TIMEOUT"
} catch {
  $sw.WriteLine("###CAPTURE_ERROR $($_.Exception.Message)")
  $status = "ERROR"
} finally {
  if ($port.IsOpen) { $port.Close() }
  $sw.WriteLine("###CAPTURE_STATUS=$status elapsedSeconds=$((New-TimeSpan -Start $startTime -End (Get-Date)).TotalSeconds)")
  $sw.Close()
}
