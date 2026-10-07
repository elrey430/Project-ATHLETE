# Project ATHLETE - puts the MuJoCo C library into the Unreal build (Source/ThirdParty/MuJoCo).
#
# The learned athlete was trained in MuJoCo 3.5.0, so Unreal runs it in the same library. It comes from the
# LocoMuJoCo Python environment (Scripts/SetupLocoMuJoCoPython.ps1), whose MuJoCo package ships the DLL and
# the C headers: the exact version the policy was trained and tested with, no extra download. The import
# library Unreal links against is generated from the DLL's exports (dumpbin + lib from the MSVC toolchain).
# MuJoCo is Apache-2.0; its LICENSE files are copied alongside.
#
# Run once (and again after updating MuJoCo):  powershell -File Scripts\SetupMuJoCoUnreal.ps1

$ErrorActionPreference = 'Stop'
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$Package = Join-Path $ProjectRoot 'Intermediate\LocoMuJoCoPython\Lib\site-packages\mujoco'
$DistInfo = Join-Path $ProjectRoot 'Intermediate\LocoMuJoCoPython\Lib\site-packages\mujoco-3.5.0.dist-info'
$Target = Join-Path $ProjectRoot 'Source\ThirdParty\MuJoCo'

if (-not (Test-Path (Join-Path $Package 'mujoco.dll'))) {
    throw "MuJoCo 3.5.0 not found in the LocoMuJoCo Python environment ($Package). Run Scripts\SetupLocoMuJoCoPython.ps1 first."
}
if (-not (Test-Path $DistInfo)) { throw "Expected MuJoCo 3.5.0 ($DistInfo); the policy was trained with that version." }

# MSVC tools (the toolchain UBT uses).
$Msvc = Get-ChildItem 'C:\Program Files\Microsoft Visual Studio\*\*\VC\Tools\MSVC\*\bin\Hostx64\x64\lib.exe' |
    Sort-Object FullName | Select-Object -First 1
if (-not $Msvc) { throw 'lib.exe not found: install the Visual Studio C++ toolchain (.vsconfig).' }
$LibExe = $Msvc.FullName
$DumpbinExe = Join-Path $Msvc.DirectoryName 'dumpbin.exe'

foreach ($Dir in @('include\mujoco', 'lib\Win64', 'bin\Win64')) {
    New-Item -ItemType Directory -Force (Join-Path $Target $Dir) | Out-Null
}
Copy-Item (Join-Path $Package 'include\mujoco\*.h') (Join-Path $Target 'include\mujoco') -Force
Copy-Item (Join-Path $Package 'mujoco.dll') (Join-Path $Target 'bin\Win64') -Force
Copy-Item (Join-Path $DistInfo 'licenses\*') $Target -Force

# Import library from the DLL's exported names.
$Exports = & $DumpbinExe /exports (Join-Path $Target 'bin\Win64\mujoco.dll')
$Names = foreach ($Line in $Exports) {
    if ($Line -match '^\s+\d+\s+[0-9A-F]+\s+[0-9A-F]{8}\s+(\S+)') { $Matches[1] }
}
if ($Names.Count -lt 100) { throw "Only $($Names.Count) exports read from mujoco.dll: dumpbin output changed?" }
$Def = Join-Path $Target 'lib\Win64\mujoco.def'
@('LIBRARY mujoco', 'EXPORTS') + $Names | Set-Content -Encoding ascii $Def
$ImportLib = Join-Path $Target 'lib\Win64\mujoco.lib'
& $LibExe /nologo "/def:$Def" /machine:x64 "/out:$ImportLib" | Out-Null
if ($LASTEXITCODE -ne 0) { throw "lib.exe failed ($LASTEXITCODE)" }
Write-Host "MuJoCo 3.5.0 for Unreal: $Target ($($Names.Count) exports)"
