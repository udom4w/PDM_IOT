# run_phase2_qualification.ps1
# Phase 2 repeatability qualification for the H4 cadence change.
# Runs the identical FIFO verification procedure N times. Each run needs its
# own boot because the Task 4.4 trigger is a one-shot per boot
# (s_fifoOneShotTriggered), so a hard reset per run IS the normal procedure.
# Reset uses esptool's read-only chip-id query (--before default-reset
# --after hard-reset) -- no flash write, firmware unchanged between runs.
param(
  [int]$StartRun = 1,
  [int]$EndRun = 5,
  [string]$PortName = "COM5",
  [int]$BaudRate = 115200,
  [int]$MaxSecondsPerRun = 150,
  [string]$OutDir = "C:\Users\HP\Downloads\PDM_IOT\claude\WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5\fifo_capture_analysis"
)

$esptool = "$env:LOCALAPPDATA\Arduino15\packages\esp32\tools\esptool_py\5.3.1\esptool.exe"

for ($run = $StartRun; $run -le $EndRun; $run++) {
  $logPath = Join-Path $OutDir ("serial_raw_QUAL_run{0:D2}.log" -f $run)
  Write-Output "=== RUN $run : resetting board ==="

  # Hard reset (read-only chip-id query; no flash write)
  & $esptool --port $PortName --before default-reset --after hard-reset chip-id *> $null
  Start-Sleep -Milliseconds 800

  $port = New-Object System.IO.Ports.SerialPort $PortName, $BaudRate, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
  $port.ReadTimeout = 2000
  $port.NewLine = "`n"
  $port.DtrEnable = $false
  $port.RtsEnable = $false

  $sw = New-Object System.IO.StreamWriter($logPath, $false)
  $sw.AutoFlush = $true
  $status = "TIMEOUT"
  $startTime = Get-Date

  try {
    $opened = $false
    $openDeadline = (Get-Date).AddSeconds(30)
    while (-not $opened -and (Get-Date) -lt $openDeadline) {
      try { $port.Open(); $opened = $true } catch { Start-Sleep -Milliseconds 200 }
    }
    if (-not $opened) { throw "Could not open $PortName within 30s" }

    $startTime = Get-Date
    $sw.WriteLine("###CAPTURE_OPENED $(Get-Date -Format o) run=$run port=$PortName baud=$BaudRate")

    $sawSessionEnd = $false
    $lingerUntil = $null
    while ((New-TimeSpan -Start $startTime -End (Get-Date)).TotalSeconds -lt $MaxSecondsPerRun) {
      try { $line = $port.ReadLine() } catch [System.TimeoutException] {
        if ($sawSessionEnd -and (Get-Date) -gt $lingerUntil) { break }
        continue
      }
      $sw.WriteLine("$(Get-Date -Format 'HH:mm:ss.fff') $line")

      if (-not $sawSessionEnd -and $line -match 'SESSION_END') {
        $sawSessionEnd = $true
        $status = "SESSION_COMPLETE"
        # linger to capture the CSV dump / FIFO-DUMP result lines that follow
        $lingerUntil = (Get-Date).AddSeconds(8)
      }
      if ($sawSessionEnd -and (Get-Date) -gt $lingerUntil) { break }
    }
  } catch {
    $sw.WriteLine("###CAPTURE_ERROR $($_.Exception.Message)")
    $status = "ERROR"
  } finally {
    if ($port.IsOpen) { $port.Close() }
    $sw.WriteLine("###CAPTURE_STATUS=$status elapsedSeconds=$((New-TimeSpan -Start $startTime -End (Get-Date)).TotalSeconds)")
    $sw.Close()
  }

  Write-Output "=== RUN $run : $status ==="
}
Write-Output "BATCH_DONE $StartRun..$EndRun"
