#!/usr/bin/env python3
import atexit
import logging
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, StreamRate
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

LIDAR = ['left', 'right', 'up', 'down', 'front', 'back']
KAMERA = ['front', 'down']
LIDAR_MAX = 10.0
LIDAR_MIN = 0.12
DT = 0.05
SKALA_WAKTU = float(os.environ.get('SKALA_WAKTU', '1'))
LOG_DIR = Path(__file__).resolve().parent.parent / 'log'


def sekarang():
    return time.time() / SKALA_WAKTU


def batasi(nilai, batas):
    return max(-batas, min(batas, nilai))


class Drone(Node):
    def __init__(self):
        super().__init__('drone_oprec')
        self._log = self._buat_logger()
        self.state = State()
        self.x = self.y = self.z = None
        self.yaw = math.pi / 2
        self.yaw_target = math.pi / 2
        self.lidar = {n: LIDAR_MAX for n in LIDAR}
        self.gambar = {n: None for n in KAMERA}

        self.pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.create_subscription(State, '/mavros/state', self._cb_state, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self._cb_pose, qos_profile_sensor_data)
        for n in LIDAR:
            self.create_subscription(LaserScan, '/lidar_' + n,
                                     lambda m, n=n: self._cb_lidar(n, m), qos_profile_sensor_data)
        for n in KAMERA:
            self.create_subscription(Image, '/camera_' + n,
                                     lambda m, n=n: self._cb_gambar(n, m), 1)

        self.srv_arm = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.srv_mode = self.create_client(SetMode, '/mavros/set_mode')
        self.srv_takeoff = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.srv_rate = self.create_client(StreamRate, '/mavros/set_stream_rate')

    def _cb_state(self, m):
        self.state = m

    def _cb_pose(self, m):
        p, q = m.pose.position, m.pose.orientation
        self.x = p.x
        self.y = p.y
        self.z = p.z
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def _cb_lidar(self, nama, m):
        r = m.ranges[0]
        if math.isinf(r):
            r = LIDAR_MAX if r > 0 else LIDAR_MIN
        self.lidar[nama] = r

    def _cb_gambar(self, nama, m):
        a = np.array(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)
        self.gambar[nama] = a[:, :, ::-1].copy()

    def jeda(self, detik=DT):
        akhir = time.time() + detik * SKALA_WAKTU
        while True:
            sisa = akhir - time.time()
            if sisa <= 0:
                break
            rclpy.spin_once(self, timeout_sec=sisa)

    def _panggil(self, klien, req):
        while not klien.wait_for_service(timeout_sec=1.0):
            self.info('Nunggu service...')
        hasil = klien.call_async(req)
        rclpy.spin_until_future_complete(self, hasil)
        return hasil.result()

    def _tunggu(self, kondisi, timeout, nama):
        t = sekarang() + timeout
        while sekarang() < t:
            if kondisi():
                return True
            self.jeda(0.1)
        self.info('TIMEOUT nunggu ' + nama)
        return False

    def _buat_logger(self):
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        self.log_path = LOG_DIR / ('drone-' + datetime.now().strftime('%Y-%m-%d-%H-%M-%S') + '.log')

        log = logging.Logger('drone_oprec')
        log.setLevel(logging.INFO)
        bentuk = logging.Formatter('%(asctime)s %(levelname)s %(message)s',
                                   datefmt='%Y-%m-%d %H:%M:%S')
        berkas = logging.FileHandler(self.log_path)
        berkas.setFormatter(bentuk)
        layar = logging.StreamHandler(sys.stdout)
        layar.setFormatter(bentuk)
        log.addHandler(berkas)
        log.addHandler(layar)
        atexit.register(self._tutup_logger, log)
        return log

    def _tutup_logger(self, log):
        for h in list(log.handlers):
            h.flush()
            h.close()
            log.removeHandler(h)

    def info(self, teks):
        self._log.info(teks)

    def siap(self):
        self.info('Nunggu MAVROS nyambung...')
        self._tunggu(lambda: self.state.connected, 60, 'koneksi FCU')
        self._panggil(self.srv_rate, StreamRate.Request(stream_id=0, message_rate=20, on_off=True))
        self.info('Nunggu data posisi (EKF)...')
        if not self._tunggu(lambda: self.x is not None, 120, 'posisi lokal'):
            raise RuntimeError('Posisi lokal nggak masuk, cek SITL')

    def mode(self, nama):
        for _ in range(20):
            if self.state.mode == nama:
                return True
            self._panggil(self.srv_mode, SetMode.Request(custom_mode=nama))
            self.jeda(0.5)
        return False

    def takeoff(self, tinggi):
        self.mode('GUIDED')
        self.info('Arming...')
        for _ in range(60):
            if self.state.armed:
                break
            self._panggil(self.srv_arm, CommandBool.Request(value=True))
            self.jeda(2.0)
        else:
            raise RuntimeError('Gagal arm')
        self.info('Takeoff ke %.1f m' % tinggi)
        self._panggil(self.srv_takeoff, CommandTOL.Request(altitude=float(tinggi)))
        self._tunggu(lambda: self.z >= tinggi * 0.93, 30, 'naik')
        self.yaw_target = self.yaw
        self.info('Di ketinggian %.2f m' % self.z)

    def kecepatan_dunia(self, vx, vy, vz):
        m = PositionTarget()
        m.header.stamp = self.get_clock().now().to_msg()
        m.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        m.type_mask = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
                       | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
                       | PositionTarget.IGNORE_YAW_RATE)
        m.velocity.x = float(vx)
        m.velocity.y = float(vy)
        m.velocity.z = float(vz)
        m.yaw = float(self.yaw_target)
        self.pub.publish(m)

    def gerak(self, maju=0.0, kanan=0.0, naik=0.0):
        c, s = math.cos(self.yaw_target), math.sin(self.yaw_target)
        self.kecepatan_dunia(maju * c + kanan * s, maju * s - kanan * c, naik)

    def diam(self, detik=1.0):
        t = sekarang() + detik
        while sekarang() < t:
            self.gerak()
            self.jeda()

    def hadap(self, derajat, timeout=10):
        self.yaw_target = math.radians(derajat)
        t = sekarang() + timeout
        while sekarang() < t:
            self.gerak()
            selisih = math.atan2(math.sin(self.yaw_target - self.yaw), math.cos(self.yaw_target - self.yaw))
            if abs(selisih) < math.radians(3):
                break
            self.jeda()
        self.diam(0.5)

    def putar(self, derajat):
        self.hadap(math.degrees(self.yaw_target) + derajat)

    def _ke(self, x, y, z, kecepatan, toleransi, timeout):
        pas = 0
        t = sekarang() + timeout
        while sekarang() < t:
            ex, ey, ez = x - self.x, y - self.y, z - self.z
            jarak = math.hypot(ex, ey)
            pas = pas + 1 if jarak < toleransi and abs(ez) < toleransi else 0
            if pas >= 6:
                return True
            v = min(kecepatan, 1.0 * jarak)
            vx = v * ex / jarak if jarak > 1e-3 else 0.0
            vy = v * ey / jarak if jarak > 1e-3 else 0.0
            self.kecepatan_dunia(vx, vy, batasi(1.0 * ez, 1.0))
            self.jeda()
        self.info('TIMEOUT gerak relatif')
        return False

    def pindah(self, maju=0.0, kanan=0.0, naik=0.0, kecepatan=1.0, toleransi=0.12, timeout=60):
        c, s = math.cos(self.yaw_target), math.sin(self.yaw_target)
        return self._ke(self.x + maju * c + kanan * s, self.y + maju * s - kanan * c, self.z + naik,
                        kecepatan, toleransi, timeout)

    def naik_ke(self, tinggi, toleransi=0.08, timeout=40):
        return self._ke(self.x, self.y, tinggi, 0.5, toleransi, timeout)

    def jarak_dari(self, titik):
        return math.hypot(self.x - titik[0], self.y - titik[1])

    def titik(self):
        return (self.x, self.y)

    def tahan(self, titik, batas=0.3):
        ex, ey = titik[0] - self.x, titik[1] - self.y
        c, s = math.cos(self.yaw_target), math.sin(self.yaw_target)
        return batasi(ex * c + ey * s, batas), batasi(ex * s - ey * c, batas)

    def mendarat(self):
        self.info('Mendarat...')
        self.mode('LAND')
        self._tunggu(lambda: not self.state.armed, 60, 'mendarat')


def buat_drone():
    rclpy.init()
    d = Drone()
    d.siap()
    return d
