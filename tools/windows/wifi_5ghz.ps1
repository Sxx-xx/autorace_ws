# The laptop's Wi-Fi for the robot on the phone's 5 GHz hotspot (iPhone: Maximize Compatibility off).
# Measured 2026-10-22: on 5 GHz, 90 fps at 10 Mbps came in at 90.2 Hz with no gap over 37 ms, where
# 2.4 GHz had 405-849 ms gaps. This keeps the adapter there and stops it dozing:
#   preferred band 5 GHz, roaming aggressiveness lowest (one AP: no roaming scans), packet
#   coalescing off (it holds back received packets, DDS among them), and the power plan's wireless
#   adapter setting at Maximum Performance on battery too (it was Medium Power Saving).
# Run in an administrator PowerShell (the adapter restarts: Wi-Fi drops for a few seconds):
#   powershell -ExecutionPolicy Bypass -File \\wsl$\Ubuntu-24.04\home\songh\autorace_ws\tools\windows\wifi_5ghz.ps1
# Undo: -Undo (no band preference, roaming medium, coalescing on, Medium Power Saving on battery).
param([string]$Adapter = 'Wi-Fi', [switch]$Undo)

[Console]::OutputEncoding = [Text.Encoding]::UTF8
$ErrorActionPreference = 'Stop'
$settings = if ($Undo) {
    @{ 'RoamingPreferredBandType' = 0; 'RoamAggressiveness' = 2; '*PacketCoalescing' = 1 }
} else {
    @{ 'RoamingPreferredBandType' = 2; 'RoamAggressiveness' = 0; '*PacketCoalescing' = 0 }
}
$battery = if ($Undo) { 2 } else { 0 }    # 0 Maximum Performance ... 3 Maximum Power Saving

function Show {
    Get-NetAdapterAdvancedProperty -Name $Adapter -RegistryKeyword @($settings.Keys) |
        Format-Table DisplayName, DisplayValue, RegistryKeyword -AutoSize | Out-String -Width 160
    powercfg /query SCHEME_CURRENT 19cbb8fa-5279-450e-9fac-8a3d5fedd0c1 12bbebe6-58d6-4636-95bb-3217ef867c1a |
        Select-String -Pattern '0x0000000[0-3]$'
}

Write-Host "== before"; Show
foreach ($k in $settings.Keys) {
    Set-NetAdapterAdvancedProperty -Name $Adapter -RegistryKeyword $k -RegistryValue $settings[$k] -NoRestart
}
powercfg /setacvalueindex SCHEME_CURRENT 19cbb8fa-5279-450e-9fac-8a3d5fedd0c1 12bbebe6-58d6-4636-95bb-3217ef867c1a 0
powercfg /setdcvalueindex SCHEME_CURRENT 19cbb8fa-5279-450e-9fac-8a3d5fedd0c1 12bbebe6-58d6-4636-95bb-3217ef867c1a $battery
powercfg /setactive SCHEME_CURRENT
Restart-NetAdapter -Name $Adapter
Start-Sleep -Seconds 10
Write-Host "== after"; Show
netsh wlan show interfaces | Select-String -Pattern 'SSID|밴드|Band|채널|Channel|신호|Signal'
