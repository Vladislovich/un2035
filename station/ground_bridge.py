#!/usr/bin/env python3
"""
ground_bridge.py - наземная станция на ноутбуке: принимает находки с дрона
(код Иры, sar-drone-onboard-main) и показывает их в веб-интерфейсе Кати.

    дрон (C++ на Orange Pi)  --TCP 5700, протокол SARL-->  этот скрипт
    этот скрипт  --HTTP + WebSocket 8080-->  браузер (web/index.html)

Что делает:
  * принимает метки, кадры находок и живую телеметрию, подтверждает (ACK)
    только после записи на диск - как tools/ground_server.py у Иры;
  * по пикселю и позе дрона из метки считает точку на земле формулой Миши
    (geo_misha.py) и сверяет с тем, что посчитал борт;
  * отдаёт в браузер кадр с обведённой находкой, координаты дрона и объекта,
    точку на карте.

Запуск (только стандартная библиотека, ставить ничего не нужно):
    python ground_bridge.py
    python ground_bridge.py --tcp-port 5700 --port 8080 --dir finds_web

Потом в браузере: http://localhost:8080
На дроне адрес ноутбука задаётся так:  SAR_GROUND_IP=<IP ноутбука> ./sar_yolo_pipeline

ВАЖНО: порт 5700 слушает что-то одно - этот скрипт, ground_server.py или viewer.py.
"""
import argparse
import base64
import hashlib
import json
import os
import socket
import struct
import sys
import threading
import time
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import geo_misha as gm

ROOT = Path(__file__).resolve().parent
WEB = ROOT / 'web'

# ---- протокол борта (ReliableLink.hpp, LINK.md)
MAGIC, ACK_MAGIC, HDR = b'SARL', b'SARA', 18
REC_PING, REC_FRAME, REC_CROP, REC_META, REC_TELEMETRY = 0, 1, 2, 3, 4
MAX_PAYLOAD = 8 << 20
IDLE_S = 45            # борт шлёт проверку связи раз в 10 с

# kind с борта -> класс в интерфейсе (0 человек, 1 транспорт, 2 дым, 3 маркер)
KIND_CLS = {'person': 0, 'car': 1, 'smoke': 2, 'aruco': 3}
CLS_LETTER = 'ЧТДМ'


def parse_args():
    p = argparse.ArgumentParser(description='Мост: находки с дрона (SARL/TCP) -> веб-интерфейс')
    p.add_argument('--tcp-port', type=int, default=5700, help='порт, к которому подключается дрон')
    p.add_argument('--video-port', type=int, default=5701, help='порт живого видео с борта (LiveVideo.hpp)')
    p.add_argument('--port', type=int, default=8080, help='порт веб-интерфейса')
    p.add_argument('--dir', default=str(ROOT / 'finds_web'), help='куда сохранять принятое')
    g = p.add_argument_group('камера (если борт не прислал эти поля в метке)')
    g.add_argument('--hfov', type=float, default=62.0, help='угол обзора по горизонтали, градусы (CAMERA_HFOV_DEG)')
    g.add_argument('--tilt', type=float, default=10.0, help='наклон камеры от вертикали вперёд, градусы (CAMERA_TILT_DEG)')
    g.add_argument('--pan', type=float, default=0.0, help='поворот камеры вокруг вертикали, градусы (CAMERA_PAN_DEG)')
    g.add_argument('--aspect', type=float, default=16 / 9, help='W/H кадра для пятна обзора по телеметрии')
    p.add_argument('--gps-sigma', type=float, default=1.5, help='СКО GNSS, м (без RTK около 1.5)')
    return p.parse_args()


def jpeg_size(b):
    """(ширина, высота) из заголовка JPEG, без Pillow."""
    i = 2
    while i + 9 < len(b):
        if b[i] != 0xFF:
            i += 1
            continue
        m = b[i + 1]
        if m in (0xC0, 0xC1, 0xC2):
            h, w = struct.unpack('>HH', b[i + 5:i + 9])
            return w, h
        if m in (0xD8, 0x01) or 0xD0 <= m <= 0xD7:
            i += 2
            continue
        i += 2 + struct.unpack('>H', b[i + 2:i + 4])[0]
    return 0, 0


# ----------------------------------------------------------------------------- геометрия
def geolocate(meta, a):
    """Точка находки по формуле Миши. Возвращает dict или None.

    Поза дрона и пиксель приходят в метке (поля drone_lat, roll_deg, u, v ...),
    их добавляет Finding.hpp. Если борт старый и их нет - берём то, что
    посчитал борт (lat, lon при geo_valid)."""
    need = ('u', 'v', 'img_w', 'img_h', 'drone_lat', 'drone_lon', 'alt_m', 'roll_deg', 'pitch_deg', 'yaw_deg')
    if all(isinstance(meta.get(k), (int, float)) for k in need):
        W, H = meta['img_w'], meta['img_h']
        pose = dict(lat0=meta['drone_lat'], lon0=meta['drone_lon'], h=meta['alt_m'],
                    phi=meta['roll_deg'], theta=meta['pitch_deg'], psi=meta['yaw_deg'],
                    hfov_deg=meta.get('hfov_deg', a.hfov),
                    alpha_deg=90.0 - meta.get('cam_tilt_deg', a.tilt),
                    pan_deg=meta.get('cam_pan_deg', a.pan))
        g = gm.pixel_to_geo(meta['u'], meta['v'], W, H, **pose)
        if g and meta['alt_m'] > 0.5:
            g['src'] = 'misha'
            g['fp'] = gm.footprint(W, H, **pose)
            if meta.get('geo_valid') and 'lat' in meta:
                dn = (meta['lat'] - g['lat']) * gm.M_PER_DEG
                de = (meta['lon'] - g['lon']) * gm.M_PER_DEG * gm.math.cos(gm.math.radians(g['lat']))
                g['board_diff_m'] = round(gm.math.hypot(dn, de), 2)
            return g
    if meta.get('geo_valid') and isinstance(meta.get('lat'), (int, float)):
        return dict(lat=meta['lat'], lon=meta['lon'], rho=None, L=None, src='board', fp=[])
    return None


# ----------------------------------------------------------------------------- находки
class Store:
    """Находки на диске и в памяти. Повтор узнаём по хешу записи (как viewer.py
    Иры): номер события на борту сбрасывается при каждом перезапуске."""

    def __init__(self, out_dir, a, on_object, on_frame):
        self.a = a
        self.dir = Path(out_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.journal = self.dir / 'events.jsonl'
        self.lock = threading.Lock()
        self.seen = set()
        self.objects = {}            # id -> объект для интерфейса
        self.by_eid = {}             # номер события борта -> id объекта
        self.count = [0, 0, 0, 0]
        self.on_object, self.on_frame = on_object, on_frame
        self._load()

    # --- объект для интерфейса из метки
    def _object(self, meta, received):
        cls = KIND_CLS.get(meta.get('kind'), meta.get('class_id', 0))
        if cls not in (0, 1, 2, 3):
            cls = 0
        prev = None
        if cls == 3 and meta.get('aruco_id', -1) >= 0:
            # Маркер с номером один на местности: повторы обновляют его, а не плодят строки
            oid = f"М{meta['aruco_id']}"
            prev = self.objects.get(oid)
        else:
            self.count[cls] += 1
            oid = CLS_LETTER[cls] + str(self.count[cls])
        ts = meta.get('timestamp_us')
        t = ts / 1e6 if isinstance(ts, (int, float)) and ts > 1e15 else received
        g = geolocate(meta, self.a)
        o = dict(id=oid, cls=cls, first=t, last=t, conf=round(float(meta.get('confidence', 1.0)), 2),
                 eid=meta.get('event_id'), aruco_id=meta.get('aruco_id', -1),
                 fix=meta.get('fix_type'), sats=meta.get('sats'))
        if g:
            alt = meta.get('alt_m') or 0
            o.update(lat=round(g['lat'], 7), lon=round(g['lon'], 7), geo_src=g['src'],
                     err=round(gm.err95(alt, g['rho'] or 0, self.a.gps_sigma), 1) if g['rho'] is not None else 5.0,
                     rho=round(g['rho'], 1) if g['rho'] is not None else None,
                     L=round(g['L'], 1) if g.get('L') is not None else None,
                     fp=g.get('fp', []))
            if 'board_diff_m' in g:
                o['board_diff_m'] = g['board_diff_m']
        if isinstance(meta.get('drone_lat'), (int, float)):
            o['drone'] = dict(lat=meta['drone_lat'], lon=meta['drone_lon'], alt=meta.get('alt_m'),
                              roll=meta.get('roll_deg'), pitch=meta.get('pitch_deg'), hdg=meta.get('yaw_deg'))
        if isinstance(meta.get('bbox'), list) and meta.get('img_w'):
            x0, y0, x1, y1 = meta['bbox']
            W, H = meta['img_w'], meta['img_h']
            o['box'] = [round(x0 / W, 4), round(y0 / H, 4), round((x1 - x0) / W, 4), round((y1 - y0) / H, 4)]
        if prev:
            o['first'] = prev['first']
            o['hits'] = prev.get('hits', 1) + 1
            if 'frame' in prev:
                o['frame'] = prev['frame']
        return o

    def _load(self):
        if not self.journal.exists():
            return
        for line in self.journal.read_text(encoding='utf-8').splitlines():
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            self.seen.add(rec.pop('_sha1', None))
            received = rec.pop('_received', time.time())
            o = self._object(rec, received)
            self.objects[o['id']] = o
            self.by_eid[rec.get('event_id')] = o['id']
        for f in self.dir.glob('*_frame.jpg'):
            oid = f.stem[:-len('_frame')]
            if oid in self.objects:
                self.objects[oid]['frame'] = f'/frames/{f.name}?v={self.objects[oid]["eid"]}'
                self.seen.add(digest(REC_FRAME, self.objects[oid]['eid'] or 0, f.read_bytes()))

    def add(self, typ, eid, payload):
        """True - новое, False - повтор. OSError наружу: не записано, ACK нельзя."""
        sha = digest(typ, eid, payload)
        with self.lock:
            if sha in self.seen:
                return False
            now = time.time()
            if typ == REC_META:
                try:
                    meta = json.loads(payload.decode('utf-8'))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    meta = None
                if not isinstance(meta, dict):
                    self.seen.add(sha)
                    return True
                rec = dict(meta, _sha1=sha, _received=now)
                with self.journal.open('a', encoding='utf-8') as f:
                    f.write(json.dumps(rec, ensure_ascii=False) + '\n')
                    f.flush()
                    os.fsync(f.fileno())
                o = self._object(meta, now)
                self.objects[o['id']] = o
                self.by_eid[eid] = o['id']
                self.seen.add(sha)
                self.on_object(o)
            elif typ in (REC_FRAME, REC_CROP):
                oid = self.by_eid.get(eid)
                if oid is None:          # метка потерялась - кадр всё равно сохраняем
                    oid = f'без-метки-{eid}-{int(now)}'
                kind = 'frame' if typ == REC_FRAME else 'crop'
                path = self.dir / f'{oid}_{kind}.jpg'
                tmp = path.with_suffix('.part')
                with tmp.open('wb') as f:
                    f.write(payload)
                    f.flush()
                    os.fsync(f.fileno())
                tmp.replace(path)
                self.seen.add(sha)
                o = self.objects.get(oid)
                if o is not None and typ == REC_FRAME:
                    o['frame'] = f'/frames/{path.name}?v={eid}'   # новый кадр того же маркера - новый адрес
                    self.on_object(o)
                    self.on_frame(o, payload)
            return True


def digest(typ, eid, payload):
    return hashlib.sha1(struct.pack('<BI', typ, eid) + payload).hexdigest()


# ----------------------------------------------------------------------------- приём с борта
class DroneLink:
    def __init__(self, a, store, hub):
        self.a, self.store, self.hub = a, store, hub
        self.lock = threading.Lock()
        self.peer, self.last_rx = None, 0.0
        self.telemetry, self.telemetry_t = None, 0.0

    def start(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('0.0.0.0', self.a.tcp_port))
        s.listen(4)
        self.sock = s
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            conn, addr = self.sock.accept()
            threading.Thread(target=self._handle, args=(conn, addr[0]), daemon=True).start()

    def _handle(self, conn, who):
        conn.settimeout(IDLE_S)
        print(f'[борт {who}] подключился', flush=True)
        with self.lock:
            self.peer, self.last_rx = who, time.time()
        try:
            while True:
                head = recv_exactly(conn, HDR)
                if head is None:
                    break
                if head[:4] != MAGIC:
                    print(f'[борт {who}] мусор в потоке, рву соединение', flush=True)
                    break
                ver, typ = head[4], head[5]
                seq, eid, length = struct.unpack_from('<III', head, 6)
                if ver != 1 or length > MAX_PAYLOAD:
                    print(f'[борт {who}] непонятная запись (версия {ver}, длина {length})', flush=True)
                    break
                payload = recv_exactly(conn, length) if length else b''
                if payload is None:
                    print(f'[борт {who}] обрыв на середине записи {seq}, дрон пришлёт заново', flush=True)
                    break
                with self.lock:
                    self.last_rx = time.time()
                if typ == REC_TELEMETRY:
                    self._telemetry(payload)
                elif typ in (REC_META, REC_FRAME, REC_CROP):
                    try:
                        new = self.store.add(typ, eid, payload)
                    except OSError as e:
                        print(f'[событие {eid}] не записалось на диск ({e}), НЕ подтверждаю', flush=True)
                        break
                    if new and typ == REC_META:
                        print(f'[событие {eid}] метка: {payload[:200].decode("utf-8", "replace")}', flush=True)
                    elif new:
                        print(f'[событие {eid}] кадр {len(payload)} байт', flush=True)
                conn.sendall(ACK_MAGIC + struct.pack('<I', seq))   # ACK только после записи
        except (socket.timeout, OSError) as e:
            print(f'[борт {who}] ошибка связи: {e}', flush=True)
        finally:
            conn.close()
            print(f'[борт {who}] связь разорвана', flush=True)
            with self.lock:
                if self.peer == who:
                    self.peer = None

    def _telemetry(self, payload):
        try:
            t = json.loads(payload.decode('utf-8'))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return
        try:
            with (self.store.dir / 'telemetry.jsonl').open('a', encoding='utf-8') as f:
                f.write(json.dumps(t, ensure_ascii=False) + '\n')
        except OSError:
            pass
        # Пятно обзора по живой позе - та же формула Миши для четырёх углов кадра
        if t.get('geo_valid') and isinstance(t.get('alt_m'), (int, float)) and t['alt_m'] > 0.5:
            H = 720
            W = round(H * self.a.aspect)
            t['fp'] = gm.footprint(W, H, lat0=t['lat'], lon0=t['lon'], h=t['alt_m'],
                                   phi=t.get('roll_deg', 0.0), theta=t.get('pitch_deg', 0.0),
                                   psi=t.get('yaw_deg', 0.0), hfov_deg=self.a.hfov,
                                   alpha_deg=90.0 - self.a.tilt, pan_deg=self.a.pan)
        t['type'] = 'telemetry'
        t['rx'] = time.time()
        with self.lock:
            self.telemetry, self.telemetry_t = t, time.time()
        self.hub.event(t)

    def status(self):
        with self.lock:
            now = time.time()
            return dict(type='status', bridge=True, peer=self.peer,
                        rx_ago=round(now - self.last_rx, 1) if self.last_rx else None,
                        tel_ago=round(now - self.telemetry_t, 1) if self.telemetry_t else None,
                        objects=len(self.store.objects))


class VideoLink:
    """Живое превью с борта (LiveVideo.hpp): "SARV" + длины + JSON + JPEG.
    Без ACK и без записи на диск - важен только свежий кадр."""

    def __init__(self, a, link, hub):
        self.a, self.link, self.hub = a, link, hub
        self.peer, self.last, self.n, self.fps = None, 0.0, 0, 0.0

    def start(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('0.0.0.0', self.a.video_port))
        s.listen(2)
        self.sock = s
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            conn, addr = self.sock.accept()
            threading.Thread(target=self._handle, args=(conn, addr[0]), daemon=True).start()

    def _handle(self, conn, who):
        conn.settimeout(10)
        print(f'[видео {who}] подключился', flush=True)
        self.peer = who
        n, t_cnt = 0, time.time()
        try:
            while True:
                head = recv_exactly(conn, 12)
                if head is None or head[:4] != b'SARV':
                    break
                hl, jl = struct.unpack_from('<II', head, 4)
                if hl > 1 << 20 or jl > MAX_PAYLOAD:
                    break
                body = recv_exactly(conn, hl + jl)
                if body is None:
                    break
                try:
                    h = json.loads(body[:hl].decode('utf-8'))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    continue
                self.last = time.time()
                n += 1
                if time.time() - t_cnt >= 2:
                    self.fps, n, t_cnt = n / (time.time() - t_cnt), 0, time.time()
                self.hub.binary(self._header(h), body[hl:])
        except OSError as e:
            print(f'[видео {who}] ошибка связи: {e}', flush=True)
        finally:
            conn.close()
            print(f'[видео {who}] связь разорвана', flush=True)

    def _header(self, h):
        """Заголовок в формате, который понимает страница: рамки без номера
        объекта (пунктир), позиция и пятно обзора - из живой телеметрии."""
        t = self.link.telemetry or {}
        drone = None
        if t.get('geo_valid'):
            drone = dict(lat=t['lat'], lon=t['lon'], alt=t.get('alt_m'), hdg=t.get('yaw_deg'))
        ts = h.get('t_us')
        return dict(kind='live', t=time.time(), w=h.get('w'), h=h.get('h'),
                    dets=[d[:6] + [None] for d in h.get('dets', []) if len(d) >= 6],
                    drone=drone, fp=t.get('fp', []), video_fps=round(self.fps, 1),
                    cap_t=ts / 1e6 if isinstance(ts, (int, float)) else None)


def recv_exactly(conn, n):
    buf = b''
    while len(buf) < n:
        part = conn.recv(n - len(buf))
        if not part:
            return None
        buf += part
    return buf


# ----------------------------------------------------------------------------- WebSocket в браузер
def ws_frame(payload, opcode):
    n = len(payload)
    if n < 126:
        hdr = struct.pack('!BB', 0x80 | opcode, n)
    elif n < 65536:
        hdr = struct.pack('!BBH', 0x80 | opcode, 126, n)
    else:
        hdr = struct.pack('!BBQ', 0x80 | opcode, 127, n)
    return hdr + payload


class Client:
    def __init__(self, sock):
        self.sock = sock
        self.cv = threading.Condition()
        self.q = deque()
        self.video = None        # живое видео: только последний кадр, без очереди
        self.alive = True

    def push_video(self, data):
        with self.cv:
            self.video = data
            self.cv.notify()

    def push(self, data):
        with self.cv:
            self.q.append(data)
            while len(self.q) > 300:
                self.q.popleft()
            self.cv.notify()

    def reader(self):
        try:
            while self.alive:
                b = self.sock.recv(4096)
                if not b or (b[0] & 0x0F) == 0x8:
                    break
        except OSError:
            pass
        with self.cv:
            self.alive = False
            self.cv.notify()

    def writer(self):
        try:
            while True:
                with self.cv:
                    self.cv.wait_for(lambda: self.q or self.video or not self.alive, timeout=5)
                    if not self.alive:
                        return
                    items, self.q = list(self.q), deque()
                    vid, self.video = self.video, None
                for d in items:
                    self.sock.sendall(d)
                if vid:
                    self.sock.sendall(vid)
                if not items and not vid:
                    self.sock.sendall(ws_frame(b'', 0x9))
        except OSError:
            with self.cv:
                self.alive = False


class Hub:
    def __init__(self):
        self.lock = threading.Lock()
        self.clients = []

    def add(self, c):
        with self.lock:
            self.clients.append(c)

    def remove(self, c):
        with self.lock:
            if c in self.clients:
                self.clients.remove(c)

    def _send(self, data):
        with self.lock:
            for c in self.clients:
                c.push(data)

    def event(self, obj):
        self._send(ws_frame(json.dumps(obj, ensure_ascii=False, separators=(',', ':')).encode(), 0x1))

    def binary(self, header, jpg):
        h = json.dumps(header, ensure_ascii=False, separators=(',', ':')).encode()
        data = ws_frame(struct.pack('!I', len(h)) + h + jpg, 0x2)
        if header.get('kind') == 'live':
            with self.lock:
                for c in self.clients:
                    c.push_video(data)
        else:
            self._send(data)


def frame_header(o, jpg):
    """Заголовок кадра в том же формате, что у drone_server.py Кати."""
    w, h = jpeg_size(jpg)
    dets = []
    if 'box' in o:
        x, y, bw, bh = o['box']
        dets.append([o['cls'], o['conf'], x, y, bw, bh, o['id']])
    return dict(kind='finding', oid=o['id'], seq=o.get('eid'), t=o['first'], w=w, h=h, dets=dets,
                drone=o.get('drone'), fp=o.get('fp', []))


# ----------------------------------------------------------------------------- HTTP
TYPES = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript', '.css': 'text/css',
         '.woff2': 'font/woff2', '.png': 'image/png', '.svg': 'image/svg+xml', '.json': 'application/json',
         '.jpg': 'image/jpeg'}


def make_handler(app):
    class H(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *args):
            pass

        def do_GET(self):
            path = self.path.split('?')[0]
            if path == '/ws':
                return self.websocket()
            if path == '/status':
                return self.send_bytes(json.dumps(app.link.status(), ensure_ascii=False).encode(), TYPES['.json'])
            if path.startswith('/frames/'):
                base = app.store.dir.resolve()
                from urllib.parse import unquote
                f = (base / unquote(path[len('/frames/'):])).resolve()
                if base != f.parent or not f.is_file():
                    return self.send_error(404)
                return self.send_bytes(f.read_bytes(), TYPES['.jpg'])
            if path == '/':
                path = '/index.html'
            f = (WEB / path.lstrip('/')).resolve()
            if WEB not in f.parents or not f.is_file():
                return self.send_error(404)
            self.send_bytes(f.read_bytes(), TYPES.get(f.suffix, 'application/octet-stream'))

        def send_bytes(self, b, ctype):
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(b)))
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(b)

        def websocket(self):
            key = self.headers.get('Sec-WebSocket-Key')
            if not key or 'websocket' not in self.headers.get('Upgrade', '').lower():
                return self.send_error(400)
            acc = base64.b64encode(hashlib.sha1((key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()).decode()
            self.send_response(101)
            self.send_header('Upgrade', 'websocket')
            self.send_header('Connection', 'Upgrade')
            self.send_header('Sec-WebSocket-Accept', acc)
            self.end_headers()
            self.wfile.flush()
            sock = self.connection
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            c = Client(sock)
            c.push(ws_frame(json.dumps(app.hello(), ensure_ascii=False).encode(), 0x1))
            app.hub.add(c)
            threading.Thread(target=c.reader, daemon=True).start()
            c.writer()
            app.hub.remove(c)
            self.close_connection = True

    return H


class App:
    def __init__(self, a):
        self.a = a
        self.session = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.hub = Hub()
        self.store = Store(a.dir, a, self._on_object, self._on_frame)
        self.link = DroneLink(a, self.store, self.hub)
        self.video = VideoLink(a, self.link, self.hub)

    def _on_object(self, o):
        self.hub.event(dict(type='objects', items=[o]))

    def _on_frame(self, o, jpg):
        self.hub.binary(frame_header(o, jpg), jpg)

    def hello(self):
        a = self.a
        t, _ = self.link.telemetry, None
        return dict(type='hello', session='bridge-' + Path(a.dir).name, server_time=time.time(),
                    config=dict(mode='bridge', source=f'дрон по TCP :{a.tcp_port}', model='на борту (RKNN, NPU)',
                                hfov=a.hfov, tilt=a.tilt, pan=a.pan, gps='onboard', confirm=3),
                    objects=list(self.store.objects.values()), telemetry=t)

    def run(self):
        try:
            self.link.start()
            self.video.start()
        except OSError as e:
            sys.exit(f'Порт {self.a.tcp_port} или {self.a.video_port} занят ({e}). '
                     f'Закройте ground_server.py / viewer.py / второй мост.')
        threading.Thread(target=self._status_loop, daemon=True).start()
        srv = ThreadingHTTPServer(('0.0.0.0', self.a.port), make_handler(self))
        srv.daemon_threads = True
        print(f'Жду дрон на TCP-порту {self.a.tcp_port}, пишу в {self.store.dir.resolve()}', flush=True)
        print(f'Уже принято находок: {len(self.store.objects)}', flush=True)
        print('Интерфейс:', flush=True)
        for ip in ['localhost'] + local_ips():
            print(f'    http://{ip}:{self.a.port}', flush=True)
        print('На дроне:  SAR_GROUND_IP=<один из адресов выше> ./build/sar_yolo_pipeline\nОстановить: Ctrl+C\n',
              flush=True)
        try:
            srv.serve_forever()
        except KeyboardInterrupt:
            print('\nОстановлено', flush=True)

    def _status_loop(self):
        while True:
            time.sleep(1.0)
            st = self.link.status()
            v = self.video
            st['video_ago'] = round(time.time() - v.last, 1) if v.last else None
            st['video_fps'] = round(v.fps, 1)
            self.hub.event(st)


def local_ips():
    ips = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except OSError:
        pass
    ips.discard('127.0.0.1')
    return sorted(ips)


if __name__ == '__main__':
    if not (WEB / 'index.html').is_file():
        sys.exit(f'Не найдена страница {WEB / "index.html"}')
    App(parse_args()).run()
