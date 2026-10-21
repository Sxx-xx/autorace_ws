# Windows serves its clock to the robot's Pi (w32time NTP server), and the Pi's chrony follows it
# (tools/pi/setup_chrony.sh <PC address>). WSL2 takes Windows' clock through Hyper-V, so the Pi,
# Windows and WSL agree.
# The other way round did not work (2026-10-22): Windows following the Pi polled every 1024 s and
# left a 47 ms offset in place for minutes; WSL's own chrony is undone by Hyper-V within seconds.
# Windows keeps its own source (time.windows.com).
# Run in an administrator PowerShell:
#   powershell -ExecutionPolicy Bypass -File \\wsl$\Ubuntu-24.04\home\songh\autorace_ws\tools\windows\time_server.ps1 [-Undo]
param([switch]$Undo)

[Console]::OutputEncoding = [Text.Encoding]::UTF8
$base = 'HKLM:\SYSTEM\CurrentControlSet\Services\W32Time'
$rule = 'AutoRace NTP for the Pi'
Set-Service w32time -StartupType Automatic
Start-Service w32time -ErrorAction SilentlyContinue
w32tm /config /manualpeerlist:"time.windows.com,0x9" /syncfromflags:manual /update
Set-ItemProperty "$base\TimeProviders\NtpClient" SpecialPollInterval 32768
if ($Undo) {
    Set-ItemProperty "$base\TimeProviders\NtpServer" Enabled 0
    Set-ItemProperty "$base\Config" AnnounceFlags 10
    Remove-NetFirewallRule -DisplayName $rule -ErrorAction SilentlyContinue
} else {
    Set-ItemProperty "$base\TimeProviders\NtpServer" Enabled 1
    # 5: always a time server, and a reliable one (so clients take it without another source).
    Set-ItemProperty "$base\Config" AnnounceFlags 5
    if (-not (Get-NetFirewallRule -DisplayName $rule -ErrorAction SilentlyContinue)) {
        New-NetFirewallRule -DisplayName $rule -Direction Inbound -Protocol UDP -LocalPort 123 `
            -RemoteAddress LocalSubnet -Action Allow -Profile Any | Out-Null
    }
}
Restart-Service w32time
Start-Sleep -Seconds 2
w32tm /query /configuration | Select-String 'Enabled|AnnounceFlags'
Get-NetFirewallRule -DisplayName $rule -ErrorAction SilentlyContinue | Select-Object DisplayName, Enabled
Get-NetUDPEndpoint -LocalPort 123 -ErrorAction SilentlyContinue | Select-Object LocalAddress, LocalPort
