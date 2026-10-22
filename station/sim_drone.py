#!/usr/bin/env python3
"""
sim_drone.py - имитатор борта для проверки наземной станции без дрона.

Говорит тем же протоколом, что ReliableLink.hpp у Иры: SARL-записи по TCP,
живая телеметрия раз в 0.5 с, находка = метка JSON + кадр с рамкой.
Метка в новом формате (с позой и пикселем, см. Finding.hpp).

    python ground_bridge.py          # в одном окне
    python sim_drone.py              # в другом
    python sim_drone.py --host 192.168.3.72 --video demo.mp4

Нужны opencv-python и numpy (кадры берутся из demo.mp4).
"""
import argparse
import json
import math
import random
import socket
import struct
import threading
import time

import cv2

import geo_misha as gm

REC_FRAME, REC_META, REC_TELEMETRY = 1, 3, 4


def rec(typ, seq, eid, payload):
    return b'SARL' + bytes([1, typ]) + struct.pack('<III', seq, eid, len(payload)) + payload


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=5700)
    ap.add_argument('--video', default='demo.mp4')
    ap.add_argument('--lat', type=float, default=60.005388)
    ap.add_argument('--lon', type=float, default=30.373863)
    ap.add_argument('--alt', type=float, default=30.0)
    ap.add_argument('--every', type=float, default=5.0, help='находка раз в столько секунд')
    a = ap.parse_args()

    cap = cv2.VideoCapture(a.video)
    sock = socket.create_connection((a.host, a.port))
    lock = threading.Lock()
    seq = [0]
    acks = [0]

    def send(typ, eid, payload):
        with lock:
            seq[0] += 1
            sock.sendall(rec(typ, seq[0], eid, payload))

    def ack_reader():
        while True:
            b = sock.recv(8)
            if not b:
                return
            acks[0] += len(b) // 8

    threading.Thread(target=ack_reader, daemon=True).start()
    t0 = time.time()

    def pose(t):
        # круг радиусом 40 м, курс по касательной, лёгкая качка
        w = 2 * math.pi / 120
        n, e = 40 * math.sin(w * t), 40 * (1 - math.cos(w * t))
        return dict(lat=a.lat + n / gm.M_PER_DEG, lon=a.lon + e / (gm.M_PER_DEG * math.cos(math.radians(a.lat))),
                    alt=a.alt + 2 * math.sin(t / 7), roll=4 * math.sin(t / 3), pitch=3 * math.sin(t / 5),
                    yaw=(math.degrees(w * t) + 90) % 360)

    eid, next_find = 0, time.time() + 2
    print(f'подключился к {a.host}:{a.port}, Ctrl+C - стоп')
    while True:
        t = time.time() - t0
        p = pose(t)
        tel = dict(kind='telemetry', timestamp_us=int(time.time() * 1e6), fc_open=True, fc_link=True,
                   msp_messages=int(t * 40), roll_deg=round(p['roll'], 2), pitch_deg=round(p['pitch'], 2),
                   yaw_deg=round(p['yaw'], 1), alt_m=round(p['alt'], 2), fix_type=3, sats=12,
                   lat=p['lat'], lon=p['lon'], geo_valid=True)
        send(REC_TELEMETRY, 0, json.dumps(tel).encode())

        if time.time() >= next_find:
            next_find = time.time() + a.every
            for _ in range(random.randint(10, 60)):
                ok, f = cap.read()
                if not ok:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ok, f = cap.read()
            H, W = f.shape[:2]
            kind = random.choice(['person', 'person', 'person', 'smoke', 'aruco'])
            bw, bh = (random.randint(24, 60), random.randint(40, 90)) if kind != 'smoke' else (180, 140)
            x0, y0 = random.randint(0, W - bw), random.randint(0, H - bh)
            cx, cy = x0 + bw / 2, y0 + bh / 2
            g = gm.pixel_to_geo(cx, cy, W, H, lat0=p['lat'], lon0=p['lon'], h=p['alt'], phi=p['roll'],
                                theta=p['pitch'], psi=p['yaw'], hfov_deg=62.0)
            eid += 1
            conf = round(random.uniform(0.45, 0.93), 2)
            meta = {'event_id': eid, 'kind': kind, 'class_id': {'person': 0, 'smoke': 2}.get(kind, -1),
                    'aruco_id': random.randint(0, 49) if kind == 'aruco' else -1,
                    'timestamp_us': int(time.time() * 1e6), 'confidence': conf if kind != 'aruco' else 1,
                    'geo_valid': g is not None, 'fix_type': 3, 'sats': 12}
            if g:
                meta.update(lat=g['lat'], lon=g['lon'])
            meta.update(u=cx, v=cy, bbox=[x0, y0, x0 + bw, y0 + bh], img_w=W, img_h=H, hfov_deg=62.0,
                        cam_tilt_deg=0.0, cam_pan_deg=0.0, drone_lat=p['lat'], drone_lon=p['lon'],
                        alt_m=p['alt'], roll_deg=p['roll'], pitch_deg=p['pitch'], yaw_deg=p['yaw'])
            send(REC_META, eid, json.dumps(meta).encode())
            # кадр как у FindingImage.hpp: зелёная рамка, подпись латиницей, 800 px, качество 65
            vis = f.copy()
            cv2.rectangle(vis, (x0, y0), (x0 + bw, y0 + bh), (0, 255, 0), 2)
            label = f"{kind} {meta['aruco_id']}" if kind == 'aruco' else f'{kind} {conf:.2f}'
            cv2.putText(vis, label, (x0, max(14, y0 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            if W > 800:
                vis = cv2.resize(vis, (800, int(H * 800 / W)), interpolation=cv2.INTER_AREA)
            jpg = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, 65])[1].tobytes()
            send(REC_FRAME, eid, jpg)
            print(f'находка {eid}: {kind}, {len(jpg)} байт, подтверждений всего {acks[0]}', flush=True)
        time.sleep(0.5)


if __name__ == '__main__':
    main()
