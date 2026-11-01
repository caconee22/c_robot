@echo off
setlocal
cd /d "%~dp0"
"C:\Python311\python.exe" "%~dp0bench_gui.py"
if errorlevel 1 (
  echo.
  echo Bench GUI failed. See the error above.
  pause
)
