#!/usr/bin/env python3
import argparse
import json
import os

from lib.bagian_akhir import bagian_akhir
from lib.drone import buat_drone
from lib.jalur_kanan import jalur_kanan

ap = argparse.ArgumentParser()
ap.add_argument('--target', type=int, choices=[1, 2, 3], default=None)
ap.add_argument('--json', default=os.path.expanduser('~/oprec/gazebo_sim/worlds/world_oprec/target.json'))
a = ap.parse_args()

nomor = a.target
if nomor is None and os.path.exists(a.json):
    nomor = json.load(open(a.json))['target_bullseye']
if nomor not in (1, 2, 3):
    nomor = 2

d = buat_drone()
try:
    d.info('Target bullseye %d' % nomor)
    d.takeoff(1.3)
    jalur_kanan(d)
    bagian_akhir(d, nomor)
    d.info('Misi selesai, bullseye %d' % nomor)
finally:
    d.mendarat()
