@echo off
setlocal
rem Use an existing workspace-local cache when the shared installation is read-only.
rem A temporary drive keeps ESP toolchain paths below Windows path limits.
if defined PLATFORMIO_CORE_DIR goto run
if not exist "%~dp0.pio\local-core\platforms" goto run
for %%D in (Z Y X W V U T S R) do if not exist %%D:\ (
    subst %%D: "%~dp0" >nul 2>&1
    if exist %%D:\platformio.ini (
        set "BENCH_PIO_DRIVE=%%D:"
        goto mapped
    )
)
set "PLATFORMIO_CORE_DIR=%~dp0.pio\local-core"
goto run
:mapped
set "PLATFORMIO_CORE_DIR=%BENCH_PIO_DRIVE%/.pio/local-core"
pushd %BENCH_PIO_DRIVE%\
:run
"%USERPROFILE%\.platformio\penv\Scripts\python.exe" -m platformio %*
set "BENCH_PIO_RESULT=%errorlevel%"
if defined BENCH_PIO_DRIVE (
    popd
    subst %BENCH_PIO_DRIVE% /d >nul 2>&1
)
exit /b %BENCH_PIO_RESULT%
