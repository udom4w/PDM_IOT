param(
  [string]$PortName = "COM5",
  [int]$BaudRate = 115200,
  [string]$LogPath = "C:\Users\HP\Downloads\PDM_IOT\claude\WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5\serial_capture_rpmdiag_20260718.log",
  [int]$MaxSeconds = 480
)

# Passive capture only. DTR/RTS are set exactly once, before Open(), and are
# never touched again for the lifetime of this script -- same convention as
# capture_diag_*.ps1 / capture_modecheck_20260718.ps1 in this folder.
#
# No crash-signature early-stop here (unlike capture_stopping_investigation_*):
# this run is launched immediately before a deliberate flash, so the boot
# banner ("[CONFIG] Press ENTER...") is EXPECTED as the first thing captured,
# not a fault condition -- an early-stop on it would truncate the capture
# right after boot, before any [RPM-DIAG] lines could accumulate.
#
# Because arduino-cli/esptool/Arduino IDE needs exclusive access to the COM
# port during upload, this script retries Open() until the upload tool
# releases the port, so it can grab it and start logging as close to the
# post-flash reset as possible -- minimizing how much of the boot sequence is
# missed.

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

  while ((New-TimeSpan -Start $startTime -End (Get-Date)).TotalSeconds -lt $MaxSeconds) {
    try {
      $line = $port.ReadLine()
    } catch [System.TimeoutException] {
      continue
    }
    $sw.WriteLine("$(Get-Date -Format 'HH:mm:ss.fff') $line")
  }
  $status = "COMPLETE"
} catch {
  $sw.WriteLine("###CAPTURE_ERROR $($_.Exception.Message)")
  $status = "ERROR"
} finally {
  if ($port.IsOpen) { $port.Close() }
  $sw.WriteLine("###CAPTURE_STATUS=$status elapsedSeconds=$((New-TimeSpan -Start $startTime -End (Get-Date)).TotalSeconds)")
  $sw.Close()
}

if ($status -eq "COMPLETE") { exit 0 } else { exit 1 }
