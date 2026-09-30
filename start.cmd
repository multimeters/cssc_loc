@echo off
setlocal DisableDelayedExpansion
"%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe" -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0start.ps1" %*
set "launcher_exit=%errorlevel%"
if not "%launcher_exit%"=="0" (
  echo.
  pause
)
exit /b %launcher_exit%
