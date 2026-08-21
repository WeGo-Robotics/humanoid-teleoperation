<#
.SYNOPSIS
    Builds and runs the Windows preview of the in-headset UI.

.DESCRIPTION
    Wraps quest_app/Assets/Editor/PreviewBuild.cs. Produces a windowed desktop
    exe running the same TeleopHud and TeleopAlignGuide the Quest build runs,
    driven by a mouse instead of a headset and by a synthetic align sequence
    instead of the host.

    Use this for every UI change, then build the APK once the layout is right.
    It cannot tell you anything about tracking, passthrough, controller input,
    the XrLink transport, or framerate -- those still need the device.

.PARAMETER NoRun
    Build without launching. Otherwise the exe starts as soon as it is built.

.EXAMPLE
    .\tools\build_preview.ps1
#>
[CmdletBinding()]
param(
    [string]$Output = "",
    [switch]$NoRun,
    [string]$UnityPath = ""
)

$ErrorActionPreference = "Stop"

$repo = Split-Path -Parent $PSScriptRoot
$project = Join-Path $repo "quest_app"
if (-not (Test-Path (Join-Path $project "Assets/Editor/PreviewBuild.cs"))) {
    throw "quest_app does not look like the teleop Unity project: $project"
}
if ($Output -eq "") { $Output = Join-Path $project "Build/preview/G1TeleopPreview.exe" }

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

$log = Join-Path $project "Build/preview-build.log"
New-Item -ItemType Directory -Force -Path (Split-Path -Parent $log) | Out-Null
if (Test-Path $log) { Remove-Item $log -Force }

# Note: no -nographics. The standalone player build needs a graphics device to
# compile shader variants; with -nographics the build succeeds and the exe
# renders nothing, which looks exactly like the bug this tool is meant to find.
$buildArgs = @(
    "-quit", "-batchmode",
    "-projectPath", $project,
    "-executeMethod", "WeGo.Teleop.Editor.PreviewBuild.Build",
    "-logFile", $log,
    "--",
    "-output", $Output
)

Write-Host "Unity   : $unity"
Write-Host "Output  : $Output"
Write-Host "Log     : $log"
Write-Host "Building the Windows preview..."

$started = Get-Date
$proc = Start-Process -FilePath $unity -ArgumentList $buildArgs -PassThru -Wait -NoNewWindow
$elapsed = (Get-Date) - $started

if ($proc.ExitCode -ne 0 -or -not (Test-Path $Output)) {
    Write-Host "`n--- errors from the build log ---" -ForegroundColor Yellow
    $hits = Select-String -Path $log -Pattern "error CS|BuildFailedException|\[PreviewBuild\]|Exception:" |
        Select-Object -Last 40
    if ($hits) { $hits | ForEach-Object { Write-Host $_.Line } }
    else { Get-Content $log -Tail 40 | ForEach-Object { Write-Host $_ } }
    throw "preview build failed after $([int]$elapsed.TotalSeconds)s (exit $($proc.ExitCode))"
}

Write-Host "`nBuilt $Output in $([int]$elapsed.TotalSeconds)s" -ForegroundColor Green

if ($NoRun) { exit 0 }
Write-Host "Launching. Right-drag to look; hold 1 or 2 and move the mouse to move a hand."
Start-Process -FilePath $Output
