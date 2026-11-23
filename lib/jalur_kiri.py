#!/usr/bin/env python3
from lib import manuver as m
from lib import vision

TINGGI_GERBANG = 0.8
TINGGI_PIPA = 2.2
TINGGI_LORONG = 0.6
TINGGI_LINGKARAN = 1.5
JENDELA = ['cyan', 'hijau', 'magenta']


def lihat_gerbang(g):
    hasil = vision.lubang_tengah(g, 'oranye_gerbang', ulang=1)
    if hasil is None or not hasil[3] or hasil[2] < 40:
        return None
    return hasil


def gerbang_miring(d):
    d.info('K1: cari gerbang miring')
    d.naik_ke(TINGGI_GERBANG)
    m.geser_sampai(d, lambda d: lihat_gerbang(d.gambar['front']) is not None, kanan=-1.5, jarak_maks=30.0)
    m.sejajarkan_depan(d, lihat_gerbang, sumbu='x')
    m.tembus(d, kecepatan=0.8, pakai_atas=True, lebih=0.4)


def pipa_meliuk(d):
    d.info('K2: pipa meliuk')
    d.naik_ke(TINGGI_PIPA)
    m.sejajarkan_depan(d, lambda g: vision.pusat_lubang(g, 'merah', 'gelap'))
    m.maju(d, 3.0)
    if m.maju_sampai_lingkaran(d, jarak_maks=4.0):
        m.sejajarkan_bawah(d)


def lorong(d):
    d.info('K3: lorong L')
    d.naik_ke(TINGGI_LORONG)
    marker = m.susur_lorong(d, TINGGI_LORONG, lambda d: d.lidar['front'] < 0.68, baca_aruco=True)
    d.info('ArUco terbaca: %s' % marker)
    d.putar(-90)
    m.susur_lorong(d, TINGGI_LORONG, lambda d: d.lidar['left'] > 2.0 and d.lidar['right'] > 2.0)
    d.naik_ke(TINGGI_LINGKARAN)
    if m.maju_sampai_lingkaran(d, jarak_maks=4.0):
        m.sejajarkan_bawah(d)
    return marker


def jendela(d):
    d.info('K4: dinding oranye')
    m.dekati(d, 'oranye_dinding', ukuran_px=420)
    m.tembus(d, 'oranye_dinding', lebih=0.3)
    for nama in JENDELA:
        d.info('K4: cari jendela ' + nama)
        if m.cari_bingkai(d, nama, boleh_mundur=nama != 'cyan'):
            m.dekati(d, nama, boleh_mundur=nama != 'cyan')
            m.tembus(d, nama, lebih=0.2)


def jalur_kiri(d):
    gerbang_miring(d)
    pipa_meliuk(d)
    marker = lorong(d)
    jendela(d)
    return marker


if __name__ == '__main__':
    from lib.drone import buat_drone
    d = buat_drone()
    try:
        d.takeoff(TINGGI_GERBANG)
        marker = jalur_kiri(d)
        d.info('Jalur kiri selesai, target bullseye %s' % marker)
    finally:
        d.mendarat()
