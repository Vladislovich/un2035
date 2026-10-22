@echo off
chcp 65001 >nul
title Proverka bez drona
cd /d "%~dp0station"
set PYTHONIOENCODING=utf-8

where python >nul 2>nul
if errorlevel 1 (
  echo  Python не найден. Установите Python 3 с python.org ^(галочка "Add Python to PATH"^).
  pause
  exit /b 1
)
python -c "import cv2" 2>nul
if errorlevel 1 (
  echo  Для имитатора нужен OpenCV, ставлю: pip install opencv-python
  python -m pip install opencv-python
)

echo.
echo  Запускаю станцию и имитатор дрона. Интерфейс: http://localhost:8080
echo  Имитатор летает по кругу и раз в 5 секунд присылает находку из demo.mp4.
echo  Остановить - закрыть оба окна.
echo.
start "Stanciya" cmd /k "set PYTHONIOENCODING=utf-8 & python ground_bridge.py --dir finds_sim"
timeout /t 3 >nul
start "" http://localhost:8080
python sim_drone.py
pause
