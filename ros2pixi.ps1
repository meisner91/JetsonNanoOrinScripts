param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]] $Args
)

# === Anpassung an deine Struktur ===
$ws = "C:\pixi_ws"
$rosSetup = "C:\pixi_ws\ros2-windows\local_setup.bat"

if (!(Test-Path $ws)) {
    Write-Error "Workspace not found: $ws"
    exit 1
}
if (!(Test-Path $rosSetup)) {
    Write-Error "ROS setup not found: $rosSetup"
    exit 1
}

Set-Location $ws

if ($Args.Count -eq 0) {
    Write-Host "Usage examples:"
    Write-Host "  .\ros2pixi.ps1 ros2 --help"
    Write-Host "  .\ros2pixi.ps1 ros2 run demo_nodes_cpp talker"
    Write-Host "  .\ros2pixi.ps1 colcon build --merge-install"
    exit 0
}

# Args sicher zu einem Command zusammenbauen (Quotes korrekt behandeln)
$joined = ($Args | ForEach-Object {
    # Nur doppelte Anführungszeichen escapen für cmd.exe
    $_.Replace('"', '\"')
}) -join ' '

# cmd.exe Kommando: zuerst ROS env, dann den gewünschten Befehl
$cmd = "call `"$rosSetup`" && $joined"

# Alles innerhalb pixi + cmd ausführen (stabile Umgebung)
pixi run -- cmd /c $cmd
