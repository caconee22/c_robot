# -V2 builds against pins_v2.hpp (HW_VERSION=2) with the AS7341 model only.
param([switch]$Demo, [switch]$Bench, [switch]$V2)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$compilerCommand = Get-Command g++ -ErrorAction SilentlyContinue
$compilerPath = if ($compilerCommand) { $compilerCommand.Source } else {
    Join-Path $env:USERPROFILE '.platformio\packages\toolchain-gccmingw32\bin\g++.exe'
}
if (-not (Test-Path -LiteralPath $compilerPath)) {
    $compilerPath = Join-Path $projectRoot '.pio/local-core/packages/toolchain-gccmingw32/bin/g++.exe'
}
if (-not (Test-Path -LiteralPath $compilerPath)) { throw 'A native g++ compiler is required.' }
$env:PATH = (Split-Path -Parent $compilerPath) + ';' + $env:PATH
$buildDirectory = Join-Path $projectRoot '.pio\host-tests'
New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null
# -Bench tests robot_bench_v2_manual (bench_v2_main.cpp + bench_v2/), which is V2-only.
if ($Bench) { $V2 = $true }
$hwVersion = if ($V2) { 2 } else { 1 }
$models = if ($V2) { @(1) } else { @(1, 2) }
$sourceFiles = @(Get-ChildItem -LiteralPath (Join-Path $projectRoot 'src') -Filter '*.cpp' -Recurse |
    Where-Object { $_.Name -ne 'sensor_serial_test.cpp' -and
        $_.Name -ne 'calibration_main.cpp' -and
        $(if ($Bench) { $_.Name -ne 'main.cpp' } else {
            $_.Name -ne 'bench_v2_main.cpp' -and $_.Directory.Name -ne 'bench_v2' }) } |
    ForEach-Object FullName)
$sourceFiles += Join-Path $PSScriptRoot 'host\fake_hardware.cpp'
$sourceFiles += Join-Path $PSScriptRoot $(if ($Bench) { 'host\test_bench.cpp' } else { 'host\test_firmware.cpp' })
foreach ($model in $models) {
    $testName = if ($Bench) { 'bench-tests' } else { 'firmware-tests' }
    $executable = Join-Path $buildDirectory "$testName-hw$hwVersion-$model.exe"
    $arguments = @('-std=c++17', '-O1', '-Wall', '-Wextra', '-Werror',
        '-Wno-unused-variable',
        "-DCOLOR_SENSOR_MODEL=$model", "-DHW_VERSION=$hwVersion",
        '-DBENCH_V2_MANUAL=1', '-I', (Join-Path $PSScriptRoot 'host'),
        '-I', (Join-Path $projectRoot 'include')) + $sourceFiles + @('-o', $executable)
    if ($Bench) { $arguments = @('-DBENCH_SOFTWARE_SWITCHES=1') + $arguments }
    & $compilerPath @arguments
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed for sensor model $model." }
    if ($Demo) { & $executable --demo } else { & $executable }
    if ($LASTEXITCODE -ne 0) { throw "Tests failed for sensor model $model." }
    if ($Bench -and $V2 -and -not $Demo) {
        & $executable --battery-missing
        if ($LASTEXITCODE -ne 0) { throw 'V2 battery start-block test failed.' }
    }
}
