param([switch]$Demo)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$compilerCommand = Get-Command g++ -ErrorAction SilentlyContinue
$compilerPath = if ($compilerCommand) { $compilerCommand.Source } else {
    Join-Path $env:USERPROFILE '.platformio\packages\toolchain-gccmingw32\bin\g++.exe'
}
if (-not (Test-Path -LiteralPath $compilerPath)) { throw 'A native g++ compiler is required.' }
$env:PATH = (Split-Path -Parent $compilerPath) + ';' + $env:PATH
$buildDirectory = Join-Path $projectRoot '.pio\host-tests'
New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null
$sourceFiles = @(Get-ChildItem -LiteralPath (Join-Path $projectRoot 'src') -Filter '*.cpp' -Recurse |
    Where-Object { $_.Name -ne 'sensor_serial_test.cpp' } | ForEach-Object FullName)
$sourceFiles += Join-Path $PSScriptRoot 'host\fake_hardware.cpp'
$sourceFiles += Join-Path $PSScriptRoot 'host\test_firmware.cpp'
foreach ($model in @(1, 2)) {
    $executable = Join-Path $buildDirectory "firmware-tests-$model.exe"
    $arguments = @('-std=c++17', '-O1', '-Wall', '-Wextra', '-Werror',
        '-Wno-unused-variable',
        "-DCOLOR_SENSOR_MODEL=$model", '-I', (Join-Path $PSScriptRoot 'host'),
        '-I', (Join-Path $projectRoot 'include')) + $sourceFiles + @('-o', $executable)
    & $compilerPath @arguments
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed for sensor model $model." }
    if ($Demo) { & $executable --demo } else { & $executable }
    if ($LASTEXITCODE -ne 0) { throw "Tests failed for sensor model $model." }
}
