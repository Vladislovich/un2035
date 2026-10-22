"""
geo_misha.py - координаты человека по кадру с камеры дрона (формула Миши).

Обозначения те же, что на листе «Определение координат человека по кадру
с камеры дрона»:

    u, v          пиксель человека на кадре
    cx, cy        главная точка (центр кадра), пиксели
    fx, fy        фокусное расстояние, пиксели: fx = (W/2) / tg(HFOV/2)
    a, b          нормализованные координаты пикселя
    alpha         наклон камеры вниз от горизонта, градусы (90 = строго вниз)
    phi           крен, правое крыло вниз +, градусы
    theta         тангаж, нос вверх +, градусы
    psi           курс от севера по часовой, градусы
    h             высота дрона над землёй под целью, м
    lat0, lon0    координаты дрона

Оси дрона: X вперёд, Y вправо, Z вниз. Мир: N север, E восток, вниз.

На борту (sar-drone-onboard-main, GeoProject.hpp) считается то же самое
матрицами Rz(psi)*Ry(theta)*Rx(phi); у Иры угол камеры задан как
tilt от вертикали, поэтому alpha = 90 - tilt.
"""
import math

M_PER_DEG = 111320.0


def fx_from_hfov(W, hfov_deg):
    """6. fx = (W/2) / tg(HFOV/2)."""
    return (W / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)


def hfov_from_dfov(dfov_deg, W, H):
    """6. HFOV = 2*arctg(tg(DFOV/2) * W / sqrt(W^2 + H^2))."""
    return math.degrees(2 * math.atan(math.tan(math.radians(dfov_deg) / 2) * W / math.hypot(W, H)))


def ray_body(u, v, W, H, hfov_deg, alpha_deg, pan_deg=0.0):
    """1. Пиксель -> направление луча (dx, dy, dz) в осях дрона.

    Оптическая ось наклонена вниз на alpha: o = (cos a, 0, sin a).
    Вправо по кадру = +Y, вниз по кадру = (-sin a, 0, cos a),
    поэтому верх кадра смотрит вперёд по полёту.
    pan - поворот камеры вокруг вертикали (у Миши его нет, у Иры есть)."""
    fx = fy = fx_from_hfov(W, hfov_deg)
    cx, cy = W / 2.0, H / 2.0
    a = (u - cx) / fx
    b = (v - cy) / fy
    al = math.radians(alpha_deg)
    dx = math.cos(al) - b * math.sin(al)
    dy = a
    dz = math.sin(al) + b * math.cos(al)
    if pan_deg:
        p = math.radians(pan_deg)
        dx, dy = dx * math.cos(p) - dy * math.sin(p), dx * math.sin(p) + dy * math.cos(p)
    return dx, dy, dz


def rotate(dx, dy, dz, phi_deg, theta_deg, psi_deg):
    """3. Полная формула: крен, тангаж, курс. Возвращает (N_dir, E_dir, z2)."""
    phi, theta, psi = map(math.radians, (phi_deg, theta_deg, psi_deg))
    # Крен
    y1 = dy * math.cos(phi) - dz * math.sin(phi)
    z1 = dy * math.sin(phi) + dz * math.cos(phi)
    # Тангаж
    x2 = dx * math.cos(theta) + z1 * math.sin(theta)
    z2 = -dx * math.sin(theta) + z1 * math.cos(theta)
    y2 = y1
    # Курс
    n_dir = x2 * math.cos(psi) - y2 * math.sin(psi)
    e_dir = x2 * math.sin(psi) + y2 * math.cos(psi)
    return n_dir, e_dir, z2


def pixel_to_geo(u, v, W, H, *, lat0, lon0, h, phi=0.0, theta=0.0, psi=0.0,
                 hfov_deg=62.0, alpha_deg=90.0, pan_deg=0.0):
    """Пиксель -> точка на земле.

    Возвращает dict(lat, lon, dN, dE, rho, L, gamma) или None, если луч
    не встречает землю (z2 <= 0: дрон сильно накренён, луч выше горизонта)."""
    dx, dy, dz = ray_body(u, v, W, H, hfov_deg, alpha_deg, pan_deg)
    n_dir, e_dir, z2 = rotate(dx, dy, dz, phi, theta, psi)
    if z2 <= 1e-6 or h <= 0:
        return None
    # 4. Пересечение с землёй
    lam = h / z2
    dN, dE = lam * n_dir, lam * e_dir
    rho = math.hypot(dN, dE)
    # 5. GPS человека
    lat = lat0 + dN / M_PER_DEG
    lon = lon0 + dE / (M_PER_DEG * math.cos(math.radians(lat0)))
    return dict(lat=lat, lon=lon, dN=dN, dE=dE, rho=rho,
                L=math.sqrt(h * h + rho * rho),               # 6. наклонная дальность
                gamma=math.degrees(math.atan2(h, rho)))       # угол луча к горизонту


def simple(v, H, hfov_deg, alpha_deg, theta_deg, h, W=None):
    """2. Простые формулы (phi = 0, пиксель по центру по горизонтали):
    gamma = alpha - theta + arctg(b),  rho = h / tg(gamma),  L = h / sin(gamma)."""
    fy = fx_from_hfov(W or H * 16 / 9, hfov_deg)
    b = (v - H / 2.0) / fy
    gamma = math.radians(alpha_deg - theta_deg) + math.atan(b)
    return dict(gamma=math.degrees(gamma), rho=h / math.tan(gamma), L=h / math.sin(gamma))


def footprint(W, H, **pose):
    """Углы кадра на земле (для пятна обзора на карте). Угол, луч которого
    уходит выше горизонта, пропускается."""
    out = []
    for u, v in ((0, 0), (W, 0), (W, H), (0, H)):
        g = pixel_to_geo(u, v, W, H, **pose)
        if g:
            out.append([round(g['lat'], 7), round(g['lon'], 7)])
    return out


def err95(h, rho, gps_sigma=1.5):
    """Радиус ~95 %: GNSS, ошибка угла камеры 1°, ошибка высоты 0.7 м
    (та же оценка, что в drone_server.py у Кати)."""
    h = max(h, 1.0)
    return 2 * math.sqrt(gps_sigma ** 2 + (h * math.tan(math.radians(1.0))) ** 2 + (rho * 0.7 / h) ** 2)


def _selftest():
    """Сверка с бортовой геометрией Иры (GeoProject.hpp) на тех же числах."""
    W, H = 1280, 720
    base = dict(lat0=60.0, lon0=30.0, h=80.0, hfov_deg=62.0)
    # строго вниз, центр кадра -> под дроном
    g = pixel_to_geo(640, 360, W, H, **base)
    assert abs(g['rho']) < 1e-6, g
    # верх кадра = вперёд; курс 0 -> на север
    g = pixel_to_geo(640, 0, W, H, **base)
    assert g['dN'] > 0 and abs(g['dE']) < 1e-6, g
    # правый край -> на восток
    g = pixel_to_geo(1280, 360, W, H, **base)
    assert g['dE'] > 0 and abs(g['dN']) < 1e-6, g
    # курс 90 -> верх кадра смотрит на восток
    g = pixel_to_geo(640, 0, W, H, psi=90, **base)
    assert g['dE'] > 0 and abs(g['dN']) < 1e-6, g
    # крен 10° (правое крыло вниз): брюхо и камера поворачиваются влево -> запад
    g = pixel_to_geo(640, 360, W, H, phi=10, **base)
    assert g['dE'] < 0 and abs(g['dE'] + 80 * math.tan(math.radians(10))) < 1e-6, g
    # тангаж 10° (нос вверх): брюхо и камера поворачиваются вперёд -> север
    g = pixel_to_geo(640, 360, W, H, theta=10, **base)
    assert g['dN'] > 0 and abs(g['dN'] - 80 * math.tan(math.radians(10))) < 1e-6, g
    # простая формула совпадает с полной при phi = 0
    s = simple(500, H, 62.0, 45.0, 5.0, 80.0, W=W)
    f = pixel_to_geo(640, 500, W, H, lat0=60, lon0=30, h=80, theta=5, hfov_deg=62, alpha_deg=45)
    assert abs(s['rho'] - f['rho']) < 1e-6 and abs(s['L'] - f['L']) < 1e-6, (s, f)
    return True


if __name__ == '__main__':
    _selftest()
    print('формула Миши: самопроверка пройдена')
