@echo off
chcp 65001 >nul
title Nazemnaya stanciya BPLA
cd /d "%~dp0station"
set PYTHONIOENCODING=utf-8

where python >nul 2>nul
if errorlevel 1 (
  echo.
  echo  Python не найден. Установите Python 3 с python.org,
  echo  при установке отметьте галочку "Add Python to PATH".
  echo.
  pause
  exit /b 1
)

echo.
echo  Наземная станция запускается. Интерфейс откроется в браузере:
echo      http://localhost:8080
echo  На дроне в SAR_GROUND_IP указать один из адресов ниже (не localhost).
echo  Остановить станцию - закрыть это окно или нажать Ctrl+C.
echo.
start "" cmd /c "timeout /t 2 >nul & start http://localhost:8080"
python ground_bridge.py %*
echo.
pause
