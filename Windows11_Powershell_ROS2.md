# Powershell add profile 

notepad $PROFILE




function ros2env {
    $rosSetup = "C:\pixi_ws\ros2-windows\local_setup.bat"

    if (!(Test-Path $rosSetup)) {
        Write-Error "ROS 2 setup not found: $rosSetup"
        return
    }

    cmd /c "call `"$rosSetup`" && set" | ForEach-Object {
        if ($_ -match "^(.*?)=(.*)$") {
            Set-Item -Path "env:$($matches[1])" -Value $matches[2]
        }
    }

    Write-Host "ROS 2 environment loaded into current pixi shell."
}


function ros2pixi {
    # startet eine neue PowerShell, aktiviert pixi, lädt ROS2 env und bleibt offen
    $ws = "C:\pixi_ws"
    $cmd = @"
Set-Location '$ws'
pixi shell
ros2env
"@

    # -NoExit: Fenster bleibt offen
    # -Command: führt Befehle aus
    powershell.exe -NoExit -Command $cmd
}
