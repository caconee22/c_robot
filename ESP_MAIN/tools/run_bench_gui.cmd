@echo off
setlocal
cd /d "%~dp0"
rem Field PC uses C:\Python311; other PCs fall back to the py launcher.
if exist "C:\Python311\python.exe" (
  "C:\Python311\python.exe" "%~dp0bench_gui.py"
) else (
  py -3 "%~dp0bench_gui.py"
)
if errorlevel 1 (
  echo.
  echo Bench GUI failed. See the error above.
  pause
)
