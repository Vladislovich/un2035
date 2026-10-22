#!/bin/bash
# run.sh - запуск пайплайна на Orange Pi. Собирает и запускает.
#
#   ./run.sh              боевой режим: настоящие координаты с GPS
#   ./run.sh test         проверка на столе: подставная поза, частые повторы ArUco
#   ./run.sh test 640     то же, но камера 640x480 - если 720p не тянет по кадрам
#
# Адрес наземной станции меняется здесь или переменной окружения:
#   SAR_GROUND_IP=192.168.3.99 ./run.sh
set -e
cd "$(dirname "$0")"

# ---- настройки, которые меняются чаще всего -------------------------------
GROUND_IP="${SAR_GROUND_IP:-192.168.3.24}"     # ноутбук с наземной станцией
FC="${SAR_FC_URI:-/dev/serial/by-id/usb-Betaflight_Betaflight_-_FOXEERF405V2_0x8000000-if00}"
WIDTH=1280; HEIGHT=720                          # MJPG, 30 кадров в секунду
# Тестовая поза: башня Политеха, высота 30 м. Применяется только в режиме test.
TEST_POSE="60.005388,30.373863,30"
# ---------------------------------------------------------------------------

MODE="${1:-}"
[ "${2:-}" = "640" ] && { WIDTH=640; HEIGHT=480; }

# Аппаратный декодер JPEG на RK3566, если он есть: освобождает процессор
# потоку ArUco. Иначе обычный jpegdec.
if gst-inspect-1.0 mppjpegdec >/dev/null 2>&1; then DEC=mppjpegdec; else DEC=jpegdec; fi

SRC="v4l2src device=/dev/video0 ! image/jpeg,width=$WIDTH,height=$HEIGHT,framerate=30/1 \
! $DEC ! videoconvert ! video/x-raw,format=NV12 \
! queue max-size-buffers=1 leaky=downstream ! appsink name=sink"

echo "сборка..."
cmake --build build -j4

# Проверяем ПЕРЕД запуском: порт полётника свободен? Иначе пайплайн
# откроет его и молча останется без телеметрии, а причина будет неочевидна.
if [ -e "$FC" ] && command -v fuser >/dev/null && fuser "$FC" >/dev/null 2>&1; then
    echo "ВНИМАНИЕ: порт полётника занят другой программой:"
    fuser -v "$FC" 2>&1 | sed 's/^/  /'
    echo "  закрой msp_probe.py или Betaflight Configurator и запусти снова"
    exit 1
fi
[ -e "$FC" ] || echo "ВНИМАНИЕ: полётник не найден ($FC) - координат не будет"

export SAR_GROUND_IP="$GROUND_IP"
export SAR_FC_URI="$FC"

if [ "$MODE" = "test" ]; then
    export SAR_FAKE_POSE="$TEST_POSE"   # координаты подставные
    export SAR_ARUCO_REPEAT_S=2         # маркер можно показывать раз в 2 с
    echo
    echo "  РЕЖИМ ПРОВЕРКИ: координаты подставные, В ПОЛЁТ НЕ ГОДИТСЯ"
    echo
fi

echo "камера: $WIDTH x $HEIGHT, декодер $DEC"
echo "земля:  $GROUND_IP"
echo
exec ./build/sar_yolo_pipeline "$SRC"
