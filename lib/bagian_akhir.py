#!/usr/bin/env python3
from lib import manuver as m
from lib import vision
from lib.drone import DT, batasi, sekarang

TINGGI_RING = 2.6
WARNA_TARGET = {1: 'bullseye1', 2: 'bullseye2', 3: 'bullseye3'}
ARAH_TARGET = {1: -1.0, 2: 0.0, 3: 1.0}


def lihat_lingkaran(g):
    return vision.pusat_benda(g, 'hijau', luas_min=300)


def lihat_ring(g):
    return vision.pusat_benda(g, 'ring', luas_min=20000)


def di_atas_lingkaran(d):
    return vision.pusat_benda(d.gambar['down'], 'hijau', luas_min=800) is not None


def lingkaran_lantai(d):
    d.info('C1: cari lingkaran lantai')
    d.naik_ke(TINGGI_RING)

    def kelihatan(d):
        hasil = lihat_lingkaran(d.gambar['front'])
        return hasil is not None and abs(hasil[0] - m.TENGAH_U) < 150
    if not di_atas_lingkaran(d):
        m.putar_cari(d, kelihatan, arah=1)
        awal = d.titik()
        t = sekarang() + 40
        while d.jarak_dari(awal) < 15.0 and not di_atas_lingkaran(d) and sekarang() < t:
            hasil = lihat_lingkaran(d.gambar['front'])
            kecepatan = 1.0
            if hasil is not None:
                d.yaw_target -= batasi(0.006 * (hasil[0] - m.TENGAH_U), 0.5) * DT
                if hasil[1] > 380:
                    kecepatan = 0.5
            else:
                kecepatan = 0.4
            d.gerak(kecepatan, 0.0, batasi(1.0 * (TINGGI_RING - d.z), 0.5))
            d.jeda()
        d.diam(0.3)
    m.sejajarkan_bawah(d)


def ring(d):
    d.info('C2: cari ring')
    m.putar_cari(d, lambda d: lihat_ring(d.gambar['front']) is not None, arah=1)
    m.putar_ke_target(d, lihat_ring)
    m.dekati(d, 'ring', ukuran_px=300, boleh_mundur=False)
    m.tembus(d, 'ring', lebih=0.8)


def bullseye(d, nomor, kecepatan=2.0):
    warna = WARNA_TARGET[nomor]
    d.info('Target: cari bullseye %d' % nomor)

    def lihat(g):
        return vision.pusat_benda(g, warna)

    def di_depan(d):
        hasil = lihat(d.gambar['front'])
        return hasil is not None and abs(hasil[0] - m.TENGAH_U) < 150
    if not di_depan(d):
        m.geser_sampai(d, di_depan, kanan=1.5 * ARAH_TARGET[nomor], jarak_maks=16.0)
    m.sejajarkan_depan(d, lihat, toleransi=5, gain=0.008, vmax=1.0, timeout=45)
    acuan = d.titik()
    tinggi = d.z
    d.info('Kamikaze dari ketinggian %.2f m, jarak %.1f m' % (d.z, d.lidar['front']))
    kena = None
    t = sekarang() + 20
    while sekarang() < t:
        _, geser = d.tahan(acuan)
        d.gerak(kecepatan, geser, batasi(1.0 * (tinggi - d.z), 0.5))
        d.jeda()
        if kena is None and d.lidar['front'] < 0.3:
            kena = (d.x, d.z, sekarang())
            d.info('Nabrak bullseye %d' % nomor)
        if kena is not None and sekarang() - kena[2] > 0.5:
            break
    d.diam(1.0)
    return kena


def bagian_akhir(d, nomor):
    lingkaran_lantai(d)
    ring(d)
    return bullseye(d, nomor)
