<#
.SYNOPSIS
    Prepares the project to run the real device path under Meta XR Simulator.

.DESCRIPTION
    Wraps quest_app/Assets/Editor/SimulatorSetup.cs. Resolves the simulator
    package, assigns the Oculus XR loader for Standalone, pins the desktop
    graphics API to D3D11, and generates Assets/Scenes/TeleopSimulator.unity
    carrying TeleopBootstrap -- the same component the APK runs, not the
    mouse-driven preview mock.

    Run this once after pulling. Everything after it happens in the editor:

        1. open quest_app in Unity
        2. open Assets/Scenes/TeleopSimulator.unity
        3. Meta > Meta XR Simulator > Activate
        4. start the host on this machine (see -Host below)
        5. press Play

    What this gets you that tools/fake_quest.py cannot: fake_quest speaks the
    wire protocol directly and never runs a line of the app, so every defect
    that lives on the device is invisible to it. This runs OVRManager, the XR
    input subsystem, the console and passthrough for real, against a host on
    loopback -- no Wi-Fi bridge, no `adb reverse`, and Debug.Log in the editor
    console instead of a logcat ring VrApi is overwriting.

    What it cannot tell you: framerate, real tracking noise, doff/don timing,
    anything about the APK or the Android manifest, and whether the robot
    moves. It narrows what has to be checked on hardware.

.PARAMETER HostAddress
    Address baked into the generated scene. Defaults to 127.0.0.1, because the
    host runs on this same machine.

.EXAMPLE
    .\tools\simulator.ps1
#>
[CmdletBinding()]
param(
    [string]$HostAddress = "127.0.0.1",
    [int]$Port = 8443,
    [string]$UnityPath = ""
)

$ErrorActionPreference = "Stop"

$repo = Split-Path -Parent $PSScriptRoot
$project = Join-Path $repo "quest_app"
if (-not (Test-Path (Join-Path $project "Assets/Editor/SimulatorSetup.cs"))) {
    throw "quest_app does not look like the teleop Unity project: $project"
}

function Find-Unity {
    param([string]$Explicit, [string]$Project)
    if ($Explicit -ne "") {
        if (-not (Test-Path $Explicit)) { throw "no Unity at $Explicit" }
        return $Explicit
    }
    $versionFile = Join-Path $Project "ProjectSettings/ProjectVersion.txt"
    $wanted = $null
    if (Test-Path $versionFile) {
        $line = Get-Content $versionFile | Where-Object { $_ -match "^m_EditorVersion:" }
        if ($line) { $wanted = ($line -split ":\s*")[1].Trim() }
    }
    $root = "C:\Program Files\Unity\Hub\Editor"
    $exact = Join-Path $root "$wanted\Editor\Unity.exe"
    if ($wanted -and (Test-Path $exact)) { return $exact }
    $any = Get-ChildItem $root -Directory -ErrorAction SilentlyContinue |
        Where-Object { Test-Path (Join-Path $_.FullName "Editor\Unity.exe") } |
        Sort-Object Name -Descending | Select-Object -First 1
    if (-not $any) { throw "no Unity editor found under $root" }
    Write-Warning "$wanted not installed; using $($any.Name) instead"
    return (Join-Path $any.FullName "Editor\Unity.exe")
}

$unity = Find-Unity -Explicit $UnityPath -Project $project

$log = Join-Path $project "Build/simulator-setup.log"
New-Item -ItemType Directory -Force -Path (Split-Path -Parent $log) | Out-Null
if (Test-Path $log) { Remove-Item $log -Force }

# No -nographics: assigning the Standalone loader touches player settings that
# want a graphics device, and the failure mode without one is a silent no-op
# rather than an error.
$setupArgs = @(
    "-quit", "-batchmode",
    "-projectPath", $project,
    "-executeMethod", "WeGo.Teleop.Editor.SimulatorSetup.Prepare",
    "-logFile", $log,
    "--",
    "-host", $HostAddress,
    "-port", $Port
)

Write-Host "Unity   : $unity"
Write-Host "Host    : ${HostAddress}:${Port}"
Write-Host "Log     : $log"
Write-Host "Preparing the simulator scene (first run also downloads the package)..."

$started = Get-Date
$proc = Start-Process -FilePath $unity -ArgumentList $setupArgs -PassThru -Wait -NoNewWindow
$elapsed = (Get-Date) - $started

if ($proc.ExitCode -ne 0) {
    Write-Host "`n--- errors from the setup log ---" -ForegroundColor Yellow
    $hits = Select-String -Path $log -Pattern "error CS|\[SimulatorSetup\]|Exception:|Failed to resolve" |
        Select-Object -Last 40
    if ($hits) { $hits | ForEach-Object { Write-Host $_.Line } }
    else { Get-Content $log -Tail 40 | ForEach-Object { Write-Host $_ } }
    throw "simulator setup failed after $([int]$elapsed.TotalSeconds)s (exit $($proc.ExitCode))"
}

Write-Host "`nPrepared in $([int]$elapsed.TotalSeconds)s" -ForegroundColor Green
Write-Host @"

Next, in the editor:

  1. open $project in Unity
  2. open Assets/Scenes/TeleopSimulator.unity
  3. Meta > Meta XR Simulator > Activate      (console says "activated")
  4. start the host on this machine, listening on ${HostAddress}:${Port}
  5. press Play

In the simulator window, Input Bindings shows the keyboard and mouse mapping.
The two you will want first are the triggers (the confirm gesture) and the
face buttons X and A (the skip). Settings > Synthetic Environment loads a room,
which is what passthrough renders.
"@
