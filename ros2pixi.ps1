param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]] $Args
)

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

# Wenn keine Argumente übergeben wurden: Hilfe anzeigen
if ($Args.Count -eq 0) {
    Write-Host "Usage examples:"
    Write-Host "  .\ros2pixi.ps1 ros2 --help"
    Write-Host "  .\ros2pixi.ps1 ros2 run demo_nodes_cpp talker"
    Write-Host "  .\ros2pixi.ps1 colcon build --merge-install"
    exit 0
}

# Alles innerhalb pixi + ROS2 ausführen (eine stabile Umgebung)
$joined = ($Args | ForEach-Object { $_.Replace('"','\"') }) -join ' '

$cmd = "call `"$rosSetup`" && $joined"
pixi run -- cmd /c $cmd
