<#
.SYNOPSIS
    Builds the Quest 3 teleoperation APK, and optionally sideloads it.

.DESCRIPTION
    Wraps quest_app/Assets/Editor/QuestBuild.cs. Every build setting lives in
    that C# file; this script only locates the toolchain, passes the host
    address through, and makes a batchmode failure legible instead of leaving a
    12,000-line log for you to find the error in.

    The first build resolves the Meta XR SDK from Meta's npm registry and
    compiles IL2CPP for ARM64 -- expect 10-20 minutes and a network connection.
    Later builds reuse Library/ and take a couple of minutes.

.PARAMETER HostAddress
    The Host PC running teleop_hand_and_arm.py --xr-source xrlink. Baked into
    the APK, so a new address means a new build.

.PARAMETER Install
    adb install -r the result. Requires the headset in developer mode, plugged
    in, and 'Allow USB debugging' accepted on the device.

.EXAMPLE
    .\tools\build_quest_apk.ps1 -HostAddress 192.168.123.2 -Install
#>
[CmdletBinding()]
param(
    [string]$HostAddress = "192.168.123.2",
    [int]$Port = 8443,
    [switch]$Tls,
    [string]$Output = "",
    [switch]$Development,
    [switch]$Install,
    [int]$VersionCode = 1,
    [string]$UnityPath = ""
)

$ErrorActionPreference = "Stop"

$repo = Split-Path -Parent $PSScriptRoot
$project = Join-Path $repo "quest_app"
if (-not (Test-Path (Join-Path $project "Assets/Editor/QuestBuild.cs"))) {
    throw "quest_app does not look like the teleop Unity project: $project"
}
if ($Output -eq "") { $Output = Join-Path $project "Build/G1Teleop.apk" }

# ---------------------------------------------------------------- toolchain
function Find-Unity {
    param([string]$Explicit, [string]$Project)

    if ($Explicit -ne "") {
        if (-not (Test-Path $Explicit)) { throw "no Unity at $Explicit" }
        return $Explicit
    }

    # Prefer the exact version the project was authored against; a different
    # editor silently upgrades ProjectSettings and the next person gets a diff
    # they did not make.
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
    if (-not $any) {
        throw "no Unity editor found under $root. Install $wanted from Unity Hub."
    }
    Write-Warning "$wanted not installed; using $($any.Name) instead"
    return (Join-Path $any.FullName "Editor\Unity.exe")
}

$unity = Find-Unity -Explicit $UnityPath -Project $project
$editorRoot = Split-Path -Parent (Split-Path -Parent $unity)
$androidPlayer = Join-Path $editorRoot "Editor\Data\PlaybackEngines\AndroidPlayer"
if (-not (Test-Path $androidPlayer)) {
    throw ("Android Build Support is not installed for this editor. Install it with:`n" +
           "  & 'C:\Program Files\Unity Hub\Unity Hub.exe' -- --headless install-modules " +
           "--version <version> -m android --childModules")
}
$adb = Join-Path $androidPlayer "SDK\platform-tools\adb.exe"

# -------------------------------------------------------------------- build
$log = Join-Path $project "Build/unity-build.log"
New-Item -ItemType Directory -Force -Path (Split-Path -Parent $log) | Out-Null
if (Test-Path $log) { Remove-Item $log -Force }

$buildArgs = @(
    "-quit", "-batchmode", "-nographics",
    "-projectPath", $project,
    "-executeMethod", "WeGo.Teleop.Editor.QuestBuild.Build",
    "-logFile", $log,
    "--",
    "-host", $HostAddress,
    "-port", $Port,
    "-output", $Output,
    "-versionCode", $VersionCode
)
if ($Tls) { $buildArgs += "-tls" }
if ($Development) { $buildArgs += "-development" }

Write-Host "Unity   : $unity"
Write-Host "Project : $project"
Write-Host "Target  : $(if ($Tls) { 'wss' } else { 'ws' })://${HostAddress}:${Port}"
Write-Host "Output  : $Output"
Write-Host "Log     : $log"
Write-Host "Building. First run pulls the Meta XR SDK and can take 20 minutes..."

$started = Get-Date
$proc = Start-Process -FilePath $unity -ArgumentList $buildArgs -PassThru -Wait -NoNewWindow
$elapsed = (Get-Date) - $started

function Show-LogErrors {
    param([string]$Path)
    if (-not (Test-Path $Path)) { Write-Warning "no log at $Path"; return }
    Write-Host "`n--- errors from the build log ---" -ForegroundColor Yellow
    $hits = Select-String -Path $Path -Pattern "error CS|BuildFailedException|\[QuestBuild\]|Error building|Exception:" |
        Select-Object -Last 40
    if ($hits) { $hits | ForEach-Object { Write-Host $_.Line } }
    else { Get-Content $Path -Tail 40 | ForEach-Object { Write-Host $_ } }
    Write-Host "--- full log: $Path ---" -ForegroundColor Yellow
}

if ($proc.ExitCode -ne 0) {
    Show-LogErrors -Path $log
    throw "Unity exited with $($proc.ExitCode) after $([int]$elapsed.TotalSeconds)s"
}
if (-not (Test-Path $Output)) {
    Show-LogErrors -Path $log
    throw "Unity reported success but produced no APK at $Output"
}

$size = [math]::Round((Get-Item $Output).Length / 1MB, 1)
Write-Host "`nBuilt $Output ($size MB) in $([int]$elapsed.TotalSeconds)s" -ForegroundColor Green

# ------------------------------------------------------------------ install
if (-not $Install) {
    Write-Host "`nSideload with:"
    Write-Host "  & '$adb' install -r '$Output'"
    exit 0
}

if (-not (Test-Path $adb)) { throw "no adb at $adb" }
$devices = & $adb devices | Select-Object -Skip 1 | Where-Object { $_ -match "\sdevice$" }
if (-not $devices) {
    throw ("no authorised device. Plug the Quest in, enable developer mode, and " +
           "accept the USB debugging prompt inside the headset.")
}
Write-Host "Installing to: $($devices -join ', ')"
& $adb install -r $Output
if ($LASTEXITCODE -ne 0) { throw "adb install failed with $LASTEXITCODE" }
Write-Host "Installed. It appears under Library > Unknown Sources." -ForegroundColor Green
