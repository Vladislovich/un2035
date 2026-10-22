#!/bin/bash
# run.sh - запуск пайплайна на Orange Pi. Собирает и запускает.
#
#   ./run.sh              боевой режим: настоящие координаты с GPS
#   ./run.sh flight       проверка на столе: подставная поза (плавает +-0.00002°),
#                         частые повторы ArUco
#   ./run.sh flight 640   то же, но камера 640x480 - если 720p не тянет по кадрам
#
# Адрес наземной станции меняется здесь или переменной окружения:
#   SAR_GROUND_IP=192.168.3.99 ./run.sh
#
# Камера ищется сама (её номер /dev/videoN меняется после переподключения).
# Задать вручную: SAR_CAMERA=/dev/video1 ./run.sh flight
# Если камера пропала на ходу, программа выходит, а run.sh ждёт камеру и
# запускает программу снова. Остановить - Ctrl+C.
set -e
cd "$(dirname "$0")"

# ---- настройки, которые меняются чаще всего -------------------------------
GROUND_IP="${SAR_GROUND_IP:-192.168.3.24}"     # ноутбук с наземной станцией
FC="${SAR_FC_URI:-/dev/serial/by-id/usb-Betaflight_Betaflight_-_FOXEERF405V2_0x8000000-if00}"
WIDTH=1280; HEIGHT=720                          # желаемый размер; берётся, если камера умеет
# Подставная точка для режима flight: "широта, долгота" или "широта, долгота, высота".
# Без высоты - 30 м (FAKE_ALT в src/main.cpp). Разовая замена без правки файла:
#   SAR_FAKE_POSE="59.9386, 30.3141, 50" ./run.sh flight
TEST_POSE="60.007156, 30.376788"
# ---------------------------------------------------------------------------

MODE="${1:-}"
[ "${2:-}" = "640" ] && { WIDTH=640; HEIGHT=480; }
EXIT_CAMERA_LOST=3      # тот же код, что в src/main.cpp

# Аппаратный декодер JPEG на RK3566, если он есть: освобождает процессор
# потоку ArUco. Иначе обычный jpegdec.
if gst-inspect-1.0 mppjpegdec >/dev/null 2>&1; then DEC=mppjpegdec; else DEC=jpegdec; fi
HAVE_V4L2CTL=0
command -v v4l2-ctl >/dev/null && HAVE_V4L2CTL=1

# ---- поиск камеры ------------------------------------------------------------
# У USB-камеры несколько узлов /dev/videoN: кадры отдаёт узел с index 0,
# остальные - метаданные. Узлы /dev/video-dec0, -enc0 - кодеки Rockchip, не камеры.
node_index() {
    cat "/sys/class/video4linux/$(basename "$(readlink -f "$1")")/index" 2>/dev/null
}
node_driver() {
    basename "$(readlink -f "/sys/class/video4linux/$(basename "$(readlink -f "$1")")/device/driver")" 2>/dev/null
}
can_capture() {
    if [ "$HAVE_V4L2CTL" = 1 ]; then
        # Только Device Caps узла: Driver Caps общие на всё устройство.
        v4l2-ctl -d "$1" --info 2>/dev/null | sed -n '/Device Caps/,$p' | grep -q 'Video Capture$'
    else
        [ "$(node_index "$1")" = 0 ]
    fi
}
find_camera() {
    if [ -n "${SAR_CAMERA:-}" ]; then
        [ -e "$SAR_CAMERA" ] && echo "$SAR_CAMERA"
        return
    fi
    local d
    # 1) постоянное имя USB-камеры - не меняется при переподключении
    for d in /dev/v4l/by-id/*-video-index0; do
        [ -e "$d" ] && can_capture "$d" && { echo "$d"; return; }
    done
    # 2) любая USB-камера (uvcvideo), 3) любой узел захвата
    for d in /dev/video[0-9]*; do
        [ -e "$d" ] && [ "$(node_driver "$d")" = uvcvideo ] && can_capture "$d" && { echo "$d"; return; }
    done
    for d in /dev/video[0-9]*; do
        [ -e "$d" ] && can_capture "$d" && { echo "$d"; return; }
    done
}

# Пайплайн под то, что камера умеет: MJPG (лучше - 30 к/с на 720p) или
# несжатый YUYV, в нужном размере или 640x480. Частоту не задаём: v4l2src
# сам берёт наибольшую для выбранного размера.
TAIL="videoconvert ! video/x-raw,format=NV12 ! queue max-size-buffers=1 leaky=downstream ! appsink name=sink"
build_source() {
    local dev="$1" modes="" w h
    if [ "$HAVE_V4L2CTL" = 1 ]; then
        # строки вида "MJPG 1280x720"
        modes=$(v4l2-ctl -d "$dev" --list-formats-ext 2>/dev/null | awk '
            /\[[0-9]+\]: .[A-Z0-9]+./ { gsub(/\x27/, "", $2); f = $2 }
            /Size: Discrete/          { print f, $3 }')
    fi
    if [ -z "$modes" ]; then
        # Форматы не узнать (нет v4l-utils) - пробуем MJPG, как раньше.
        CAM_DESC="MJPG ${WIDTH}x${HEIGHT} (форматы не проверены: sudo apt install v4l-utils)"
        SRC="v4l2src device=$dev ! image/jpeg,width=$WIDTH,height=$HEIGHT ! $DEC ! $TAIL"
        return
    fi
    for size in "${WIDTH}x${HEIGHT}" 640x480; do
        w=${size%x*}; h=${size#*x}
        if grep -qx "MJPG $size" <<<"$modes"; then
            CAM_DESC="MJPG $size, декодер $DEC"
            SRC="v4l2src device=$dev ! image/jpeg,width=$w,height=$h ! $DEC ! $TAIL"
            return
        fi
        if grep -qx "YUYV $size" <<<"$modes"; then
            CAM_DESC="YUYV $size (без сжатия)"
            SRC="v4l2src device=$dev ! video/x-raw,format=YUY2,width=$w,height=$h ! $TAIL"
            return
        fi
    done
    # Ни один из привычных режимов - пусть GStreamer договорится сам.
    CAM_DESC="автовыбор GStreamer (камера умеет: $(echo $modes | tr '\n' ' '))"
    SRC="v4l2src device=$dev ! decodebin ! $TAIL"
}
# ------------------------------------------------------------------------------

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

if [ "$MODE" = "flight" ]; then
    export SAR_FAKE_POSE="${SAR_FAKE_POSE:-$TEST_POSE}"   # координаты подставные
    export SAR_ARUCO_REPEAT_S=2         # маркер можно показывать раз в 2 с
    echo
    echo "  РЕЖИМ ПРОВЕРКИ: координаты подставные, В ПОЛЁТ НЕ ГОДИТСЯ"
    echo "  точка: $SAR_FAKE_POSE"
    echo
fi
echo "земля:  $GROUND_IP"

set +e
while true; do
    CAM=$(find_camera)
    if [ -z "$CAM" ]; then
        echo "камера не найдена${SAR_CAMERA:+ ($SAR_CAMERA)}, жду... (есть: $(ls /dev/video* 2>/dev/null | tr '\n' ' '))"
        while [ -z "$CAM" ]; do sleep 2; CAM=$(find_camera); done
        sleep 1     # дать камере доинициализироваться после появления
    fi
    build_source "$CAM"
    echo "камера: $CAM ($(readlink -f "$CAM")), $CAM_DESC"
    echo

    ./build/sar_yolo_pipeline "$SRC"
    rc=$?
    # Камера не открылась или пропала - ищем заново. Любой другой выход
    # (Ctrl+C, pkill, ошибка) - заканчиваем.
    [ "$rc" = "$EXIT_CAMERA_LOST" ] || exit "$rc"
    echo
    echo "=== камера потеряна, перезапуск через 2 с ==="
    sleep 2
done
