@echo off
rem OBD-Tool starten. Beim ersten Start wird pyserial installiert.
cd /d "%~dp0"
set PY=py -3
where py >nul 2>&1 || set PY=python
%PY% --version >nul 2>&1
if errorlevel 1 (
  echo Python wurde nicht gefunden.
  echo Bitte Python von https://www.python.org/downloads/ installieren
  echo und dabei "Add python.exe to PATH" anhaken.
  pause
  exit /b 1
)
%PY% -c "import serial" >nul 2>&1
if errorlevel 1 (
  echo Installiere pyserial ...
  %PY% -m pip install --user pyserial
)
%PY% obd_tool.py %*
if errorlevel 1 pause
