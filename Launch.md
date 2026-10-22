# Запуск

## Первый раз (плата)
```bash
ssh orangepi@192.168.0.32
cd ~/un2035/onboard
cmake -B build -DCMAKE_BUILD_TYPE=Release -DRKNN_ROOT="$PWD/third_party/rknpu2"
```

## 1. Станция (ПК)
```bash
cd station
python3 ground_bridge.py --dir finds_3
```
Интерфейс: http://localhost:8080

## 2. Борт (плата)
```bash
ssh orangepi@192.168.0.32
cd ~/un2035/onboard
SAR_GROUND_IP=192.168.0.38 ./run.sh flight
```

| Команда | Режим |
|---|---|
| `./run.sh` | боевой: координаты с GPS |
| `./run.sh flight` | подставные координаты из `TEST_POSE` в `run.sh` |
| `./run.sh noflow` | без живого видео, только кадры находок |
| `./run.sh 640` | камера 640×480 |
| `./run.sh flight noflow 640` | слова сочетаются |

Перед `./run.sh` всегда `SAR_GROUND_IP=<IP ПК>`.

Дополнительно:
- `SAR_CAMERA=/dev/video1` — камера вручную;
- `SAR_FAKE_POSE="59.93, 30.31"` — другая подставная точка.

Остановить: `Ctrl+C`.

## Без дрона (ПК)
```bash
cd station
python3 ground_bridge.py --dir finds_sim
python3 sim_drone.py        # во втором окне
```

Подробно — [Readme.md](Readme.md).
