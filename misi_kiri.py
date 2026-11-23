#!/usr/bin/env python3
import argparse

from lib.bagian_akhir import bagian_akhir
from lib.drone import buat_drone
from lib.jalur_kiri import jalur_kiri

ap = argparse.ArgumentParser()
ap.add_argument('--target', type=int, choices=[1, 2, 3], default=None)
a = ap.parse_args()

d = buat_drone()
try:
    d.takeoff(0.8)
    marker = jalur_kiri(d)
    nomor = a.target or marker
    if nomor not in (1, 2, 3):
        d.info('ArUco nggak kebaca, pakai bullseye 2')
        nomor = 2
    bagian_akhir(d, nomor)
    d.info('Misi selesai, bullseye %d' % nomor)
finally:
    d.mendarat()
