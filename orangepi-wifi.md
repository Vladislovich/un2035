# Настройка Wi‑Fi на Orange Pi (NetworkManager)

Короткая инструкция по подключению Orange Pi к Wi‑Fi, настройке IP и доступу по SSH с ПК.
Все команды выполняются на Orange Pi (например, через USB‑консоль).

## Схема сети

```
Интернет ── Роутер 192.168.0.1 (DHCP) ──┬── ПК 192.168.0.38 (кабель)
                                        └── Huawei B535 (192.168.0.100, точка доступа)
                                               └── Wi‑Fi HUAWEI_B535 ── Orange Pi
```

Huawei работает как точка доступа: адреса и шлюз в Wi‑Fi выдаёт главный роутер `192.168.0.1`.
Шлюз для Orange Pi — **192.168.0.1**, а не 192.168.0.100.

## 1. Диагностика

```bash
nmcli device status            # какие интерфейсы подключены и к чему
nmcli dev wifi list            # доступные сети
ip addr show wlan0             # адрес на Wi‑Fi
ip route | grep default        # маршрут по умолчанию (должен быть ОДИН)
ping -c 3 8.8.8.8              # есть ли интернет
```

Признаки статического адреса в `ip addr`: `valid_lft forever` и нет слова `dynamic`.

## 2. Подключение к Wi‑Fi

```bash
sudo nmcli dev wifi connect "HUAWEI_B535" password "ПАРОЛЬ"
```

Для каждой сети NetworkManager создаёт отдельный профиль (`nmcli connection show`).
Настройки IP хранятся в профиле, поэтому статику можно задать для одной сети, а в остальных оставить DHCP.

## 3. Адрес по DHCP (рекомендуется)

```bash
sudo nmcli connection modify "HUAWEI_B535" \
  ipv4.method auto ipv4.addresses "" ipv4.gateway "" ipv4.dns "" ipv4.routes ""
sudo nmcli connection up "HUAWEI_B535"
```

Чтобы адрес не менялся, удобнее закрепить его на роутере (DHCP‑резервирование по MAC `wlan0`).

## 4. Статический адрес (только для этой сети)

Адреса берите из того, что выдал DHCP (шаг 3): подсеть из `ip addr`, шлюз из `ip route`.

```bash
sudo nmcli connection modify "HUAWEI_B535" ipv4.method manual \
  ipv4.addresses 192.168.0.32/24 \
  ipv4.gateway 192.168.0.1 \
  ipv4.dns "192.168.0.1 8.8.8.8"
sudo nmcli connection up "HUAWEI_B535"
```

`nmcli connection modify` не применяет изменения сразу, для этого нужен `nmcli connection up`.

## 5. Лишний маршрут по умолчанию

Симптом: адрес есть, но `ping 8.8.8.8` выдаёт `Destination Host Unreachable`, а в `ip route` две строки `default`:

```
default via 192.168.8.1 dev wlan0 proto static metric 20100   ← старый, неверный
default via 192.168.0.1 dev wlan0 proto dhcp  metric 20600    ← правильный
```

Linux не проверяет, работает ли шлюз, и всегда идёт по маршруту с меньшей метрикой.
Убирать нужно сам лишний маршрут, приоритеты менять бесполезно.

```bash
# временно, до переподключения
sudo ip route del default via 192.168.8.1

# найти источник
nmcli -f ipv4.gateway,ipv4.routes connection show "HUAWEI_B535"
sudo grep -rn "192.168.8" /etc/netplan/ /etc/NetworkManager/system-connections/
```

- Если маршрут в профиле, очистите его командой из шага 3.
- Если в netplan, удалите строки с `gateway4:` или `routes:`/`via: 192.168.8.1`, затем выполните `sudo netplan apply`.

Проверка: `ip route | grep default` должен показывать одну строку через `192.168.0.1`.

## 6. Доступ с ПК

ПК и Orange Pi в одной сети `192.168.0.x`:

```bash
ssh orangepi@192.168.0.32
```

## 7. Git: клонирование с GitHub

```bash
git clone git@github.com:Vladislovich/un2035.git
```

- Ошибка `No route to host` обычно означает проблему с сетью (см. шаг 5). Сначала проверьте `ping 8.8.8.8`.
- Для SSH нужен ключ: `ssh-keygen -t ed25519`, затем добавьте `~/.ssh/id_ed25519.pub` в GitHub → Settings → SSH keys.
- Если закрыт порт 22, клонируйте по HTTPS (`https://github.com/...`) или настройте SSH через 443:

  ```
  # ~/.ssh/config
  Host github.com
      HostName ssh.github.com
      Port 443
      User git
  ```

## Шпаргалка по ошибкам

| Симптом | Причина | Решение |
|---|---|---|
| Устройства нет в списке клиентов роутера | Статический IP, роутер его не выдавал | Перейти на DHCP (шаг 3) |
| `Destination Host Unreachable` до шлюза | Шлюз не из этой сети | Взять шлюз из DHCP (шаг 3–4) |
| Адрес есть, интернета нет, два `default` | Старый статический маршрут | Удалить его (шаг 5) |
| `No route to host` при `git clone` | Нет интернета или закрыт порт 22 | Шаг 5 или HTTPS / порт 443 |
