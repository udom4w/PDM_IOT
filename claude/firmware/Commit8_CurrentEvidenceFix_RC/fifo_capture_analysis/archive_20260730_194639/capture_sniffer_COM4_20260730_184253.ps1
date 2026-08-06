# capture_sniffer_COM4_20260730_184253.ps1
# Passive capture of the standalone ESP32 RS485 sniffer (COM4). No DTR/RTS
# toggle -- must not reset or otherwise disturb the sniffer.
param(
  [string]$PortName = "COM4",
  [int]$BaudRate = 115200,
  [string]$LogPath = "C:\Users\HP\Downloads\PDM_IOT\claude\WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5\fifo_capture_analysis\sniffer_raw_SNIFFER_VALIDATION_20260730_184253.log",
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
  $port.Open()
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
