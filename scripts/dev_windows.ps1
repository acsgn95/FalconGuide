param(
  [ValidateSet("windows-vcpkg", "windows-arm64-vcpkg")]
  [string]$Preset = "windows-vcpkg",
  [switch]$SkipTests
)

$ErrorActionPreference = "Stop"

if (-not (Get-Command cmake -ErrorAction SilentlyContinue)) {
  throw "CMake was not found in PATH."
}

if (-not (Get-Command ninja -ErrorAction SilentlyContinue)) {
  throw "Ninja was not found in PATH."
}

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$localVcpkg = Join-Path $repoRoot ".tools\vcpkg"
$toolchainFile = Join-Path $localVcpkg "scripts\buildsystems\vcpkg.cmake"

if (-not (Test-Path $toolchainFile)) {
  throw "Local vcpkg was not found at $localVcpkg. Clone vcpkg into .tools\vcpkg and run bootstrap-vcpkg.bat."
}

$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path $vswhere)) {
  throw "vswhere.exe was not found. Install Visual Studio Build Tools with the C++ workload."
}

$vsPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vsPath) {
  throw "Visual Studio C++ build tools were not found. Install the Desktop development with C++ workload."
}

$devCmd = Join-Path $vsPath "Common7\Tools\VsDevCmd.bat"
if (-not (Test-Path $devCmd)) {
  throw "VsDevCmd.bat was not found at $devCmd."
}

$arch = "x64"
$triplet = "x64-windows"
if ($Preset -eq "windows-arm64-vcpkg") {
  $arch = "arm64"
  $triplet = "arm64-windows"
}

$commands = @(
  "call `"$devCmd`" -arch=$arch",
  "set VCPKG_DEFAULT_TRIPLET=$triplet",
  "set VCPKG_DEFAULT_HOST_TRIPLET=$triplet",
  "cmake --preset $Preset",
  "cmake --build --preset $Preset"
)

if (-not $SkipTests) {
  $commands += "ctest --preset $Preset"
}

& cmd.exe /d /s /c ($commands -join " && ")
if ($LASTEXITCODE -ne 0) {
  exit $LASTEXITCODE
}
