#!/usr/bin/env python3
import math
from collections import Counter

from lib import vision
from lib.drone import DT, batasi, sekarang

TENGAH_U = 320
TENGAH_V = 240
FOKUS = 554.0


def sejajarkan_depan(d, cari, sumbu='xy', toleransi=15, stabil=8, gain=0.004, vmax=0.5, timeout=25):
    pas = 0
    hilang = 0
    t = sekarang() + timeout
    while sekarang() < t:
        hasil = cari(d.gambar['front'])
        if hasil is None:
            hilang += 1
            d.gerak()
            if hilang > 60:
                d.info('sejajarkan_depan: target nggak kelihatan')
                return False
            d.jeda()
            continue
        hilang = 0
        eu = hasil[0] - TENGAH_U
        ev = hasil[1] - TENGAH_V
        kanan = batasi(gain * eu, vmax)
        naik = batasi(-gain * ev, vmax) if 'y' in sumbu else 0.0
        d.gerak(0.0, kanan, naik)
        beres = abs(eu) < toleransi and ('y' not in sumbu or abs(ev) < toleransi)
        pas = pas + 1 if beres else 0
        if pas >= stabil:
            d.diam(0.3)
            return True
        d.jeda()
    d.info('sejajarkan_depan: timeout')
    d.diam(0.3)
    return False


def sejajarkan_bawah(d, nama='hijau', toleransi=0.08, stabil=8, gain=0.5, vmax=0.3, halus=0.3, timeout=25):
    pas = 0
    hilang = 0
    kanan = depan = None
    t = sekarang() + timeout
    while sekarang() < t:
        hasil = vision.pusat_benda(d.gambar['down'], nama, luas_min=800)
        if hasil is None:
            hilang += 1
            d.gerak()
            if hilang > 60:
                d.info('sejajarkan_bawah: lingkaran nggak kelihatan')
                return False
            d.jeda()
            continue
        hilang = 0
        meter_per_px = d.lidar['down'] / FOKUS
        kanan_baru = (hasil[0] - TENGAH_U) * meter_per_px
        depan_baru = -(hasil[1] - TENGAH_V) * meter_per_px
        if kanan is None:
            kanan, depan = kanan_baru, depan_baru
        else:
            kanan += halus * (kanan_baru - kanan)
            depan += halus * (depan_baru - depan)
        d.gerak(batasi(gain * depan, vmax), batasi(gain * kanan, vmax), 0.0)
        pas = pas + 1 if abs(kanan) < toleransi and abs(depan) < toleransi else 0
        if pas >= stabil:
            d.diam(0.3)
            return True
        d.jeda()
    d.info('sejajarkan_bawah: timeout')
    d.diam(0.3)
    return False


def geser_sampai(d, cek, maju=0.0, kanan=0.0, naik=0.0, jarak_maks=30.0, tinggi_maks=None, tinggi_min=None, timeout=60):
    awal = d.titik()
    z0 = d.z
    t = sekarang() + timeout
    while d.jarak_dari(awal) < jarak_maks and abs(d.z - z0) < jarak_maks and sekarang() < t:
        if cek(d):
            d.diam(0.3)
            return True
        if tinggi_maks is not None and d.z >= tinggi_maks and naik > 0:
            break
        if tinggi_min is not None and d.z <= tinggi_min and naik < 0:
            break
        d.gerak(maju, kanan, naik)
        d.jeda()
    d.diam(0.3)
    return False


def maju_sampai_lingkaran(d, nama='hijau', kecepatan=0.4, jarak_maks=4.0):
    ketemu = geser_sampai(d, lambda d: vision.pusat_benda(d.gambar['down'], nama, luas_min=800) is not None,
                          maju=kecepatan, jarak_maks=jarak_maks)
    if not ketemu:
        d.info('maju_sampai_lingkaran: nggak ketemu')
    return ketemu


def maju(d, jarak, kecepatan=0.8):
    d.pindah(maju=jarak, kecepatan=kecepatan)


def putar_cari(d, cek, arah=1, laju=25.0, maks=360.0):
    awal = d.titik()
    diputar = 0.0
    while diputar < maks:
        if cek(d):
            d.diam(0.3)
            return True
        langkah = math.radians(laju) * DT
        d.yaw_target += arah * langkah
        diputar += math.degrees(langkah)
        maju, kanan = d.tahan(awal)
        d.gerak(maju, kanan, 0.0)
        d.jeda()
    d.diam(0.3)
    return False


def putar_ke_target(d, cari, toleransi=10, stabil=8, gain=0.006, timeout=20):
    awal = d.titik()
    pas = 0
    t = sekarang() + timeout
    while sekarang() < t:
        hasil = cari(d.gambar['front'])
        if hasil is not None:
            eu = hasil[0] - TENGAH_U
            d.yaw_target -= batasi(gain * eu, 0.5) * DT
            pas = pas + 1 if abs(eu) < toleransi else 0
            if pas >= stabil:
                break
        maju, kanan = d.tahan(awal)
        d.gerak(maju, kanan, 0.0)
        d.jeda()
    d.diam(0.3)


def dekati(d, nama, ukuran_px=330, toleransi=15, stabil=8, gain=0.004, vmax=0.5, timeout=45, boleh_mundur=True):
    pas = 0
    hilang = 0
    kunci = None
    belakang_kosong = 0
    t = sekarang() + timeout
    while sekarang() < t:
        g = d.gambar['front']
        k = vision.kotak(g, nama)
        if k is None:
            hilang += 1
            d.gerak()
            if hilang > 60:
                d.info('dekati %s: nggak kelihatan' % nama)
                return False
            d.jeda()
            continue
        hilang = 0
        x, y, w, h = k
        nempel = vision.nempel_tepi(k)
        belakang_kosong = belakang_kosong + 1 if d.lidar['back'] > 1.5 else 0
        bisa_mundur = boleh_mundur and belakang_kosong >= 10
        maju = 0.0
        target = None
        if kunci is not None:
            target = vision.lubang_terdekat(g, kunci, nama)
            if target is None:
                tengah = vision.lubang_tengah(g, nama)
                if tengah is not None and (tengah[3] or tengah[4]):
                    target = tengah
        elif nempel and bisa_mundur:
            maju = -0.4
        elif not nempel and max(w, h) < 0.6 * ukuran_px:
            if d.lidar['front'] > 0.7:
                maju = 0.4
        else:
            target = vision.lubang_utuh(g, nama)
            if target is None:
                tengah = vision.lubang_tengah(g, nama)
                if tengah is not None and (tengah[3] or tengah[4]):
                    target = tengah
        ada_lubang = target is not None
        if ada_lubang:
            kunci = (target[0], target[1])
        else:
            target = (x + w / 2, y + h / 2)
        eu = target[0] - TENGAH_U
        ev = target[1] - TENGAH_V
        naik = batasi(-gain * ev, vmax)
        if naik < 0 and d.lidar['down'] < 0.45:
            naik = 0.0
        d.gerak(maju, batasi(gain * eu, vmax), naik)
        beres = ada_lubang and abs(eu) < toleransi and abs(ev) < toleransi
        pas = pas + 1 if beres else 0
        if pas >= stabil:
            d.diam(0.3)
            return True
        d.jeda()
    d.info('dekati %s: timeout' % nama)
    d.diam(0.3)
    return False


def tembus(d, nama=None, kecepatan=0.6, lebih=0.5, jarak_maks=9.0, pakai_atas=False, gain=0.003):
    awal = d.titik()
    acuan = d.titik()
    tinggi = d.z
    pernah_di_dalam = False
    di_luar = 0
    t = sekarang() + 60
    while d.jarak_dari(awal) < jarak_maks and sekarang() < t:
        kiri, kanan_l = d.lidar['left'], d.lidar['right']
        atas, bawah = d.lidar['up'], d.lidar['down']
        di_dalam = atas < 1.6 if pakai_atas else (kiri < 1.6 and kanan_l < 1.6)
        geser = None
        naik = batasi(1.0 * (tinggi - d.z), 0.3)
        if kiri < 1.6 and kanan_l < 1.6:
            geser = batasi(1.0 * (kanan_l - kiri) / 2.0, 0.3)
            acuan = d.titik()
            if atas < 2.0 and bawah < 2.0 and not pakai_atas:
                naik = batasi(1.0 * (atas - bawah) / 2.0, 0.3)
                tinggi = d.z
        elif nama is not None and not pernah_di_dalam:
            lihat = vision.lubang_tengah(d.gambar['front'], nama)
            if lihat is not None and lihat[3]:
                geser = batasi(gain * (lihat[0] - TENGAH_U), 0.3)
                acuan = d.titik()
                if lihat[4]:
                    naik = batasi(-gain * (lihat[1] - TENGAH_V), 0.3)
                    tinggi = d.z
        if geser is None:
            _, geser = d.tahan(acuan)
        if di_dalam:
            pernah_di_dalam = True
            di_luar = 0
        elif pernah_di_dalam:
            di_luar += 1
            if di_luar >= 4:
                break
        d.gerak(kecepatan, geser, naik)
        d.jeda()
    if not pernah_di_dalam:
        d.info('tembus: lidar nggak pernah ngerasa di dalam bingkai')
    d.pindah(maju=lebih, kecepatan=kecepatan)
    return pernah_di_dalam


def cari_bingkai(d, nama, luas_min=5000, tinggi_min=1.0, tinggi_maks=4.6, mundur_maks=2.5, boleh_mundur=True):
    def cek(d):
        return vision.kotak(d.gambar['front'], nama, luas_min=luas_min) is not None
    if cek(d):
        return True
    if geser_sampai(d, cek, naik=-0.5, tinggi_min=tinggi_min):
        return True
    if geser_sampai(d, cek, naik=0.5, tinggi_maks=tinggi_maks):
        return True
    if boleh_mundur and d.lidar['back'] > 1.5:
        geser_sampai(d, lambda d: cek(d) or d.lidar['back'] < 1.2, maju=-0.4, jarak_maks=mundur_maks)
        if cek(d):
            return True
    if geser_sampai(d, cek, naik=-0.5, tinggi_min=tinggi_min):
        return True
    d.info('cari_bingkai %s: nggak ketemu' % nama)
    return False


def susur_lorong(d, tinggi, berhenti, kecepatan=0.6, baca_aruco=False, timeout=60):
    suara = Counter()
    acuan = d.titik()
    t = sekarang() + timeout
    while sekarang() < t:
        kiri, kanan, depan = d.lidar['left'], d.lidar['right'], d.lidar['front']
        if berhenti(d):
            break
        if kiri < 2.0 and kanan < 2.0:
            geser = batasi(0.8 * (kanan - kiri) / 2.0, 0.3)
            acuan = d.titik()
        else:
            _, geser = d.tahan(acuan)
        v = min(kecepatan, max(0.15, 0.8 * (depan - 0.5)))
        d.gerak(v, geser, batasi(1.0 * (tinggi - d.z), 0.5))
        if baca_aruco:
            hasil = vision.baca_aruco(d.gambar['front'])
            if hasil is not None:
                suara[hasil] += 1
        d.jeda()
    else:
        d.info('susur_lorong: timeout')
    d.diam(0.5)
    if suara:
        return suara.most_common(1)[0][0]
    return None


def susur(d, berhenti, tinggi=None, sisi=None, jarak=None, tengah_vertikal=False, kecepatan=0.5, timeout=60):
    acuan = d.titik()
    t = sekarang() + timeout
    while sekarang() < t:
        if berhenti(d):
            break
        kiri, kanan = d.lidar['left'], d.lidar['right']
        atas, bawah, depan = d.lidar['up'], d.lidar['down'], d.lidar['front']
        if sisi == 'right' and kanan < 1.5:
            geser = batasi(1.0 * (kanan - jarak), 0.3)
            acuan = d.titik()
        elif sisi == 'left' and kiri < 1.5:
            geser = batasi(-1.0 * (kiri - jarak), 0.3)
            acuan = d.titik()
        elif sisi is None and kiri < 1.5 and kanan < 1.5:
            geser = batasi(1.0 * (kanan - kiri) / 2.0, 0.3)
            acuan = d.titik()
        else:
            _, geser = d.tahan(acuan)
        if tengah_vertikal and atas < 2.5 and bawah < 2.5:
            naik = batasi(1.0 * (atas - bawah) / 2.0, 0.4)
        elif tinggi is not None:
            naik = batasi(1.0 * (tinggi - d.z), 0.5)
        else:
            naik = 0.0
        v = min(kecepatan, max(0.15, 0.8 * (depan - 0.45)))
        d.gerak(v, geser, naik)
        d.jeda()
    else:
        d.info('susur: timeout')
    d.diam(0.5)


def naik_sampai_depan_kosong(d, tambahan, batas_tinggi, kosong=3.0, kecepatan=0.3):
    geser_sampai(d, lambda d: d.lidar['front'] > kosong, naik=kecepatan, tinggi_maks=batas_tinggi)
    tujuan = min(d.z + tambahan, batas_tinggi)
    d.naik_ke(tujuan, toleransi=0.05)
    return tujuan


def lewat(sensor, dekat, jauh):
    keadaan = {'masuk': False}

    def berhenti(d):
        nilai = d.lidar[sensor]
        if nilai < dekat:
            keadaan['masuk'] = True
        return keadaan['masuk'] and nilai > jauh
    return berhenti


def geser_jaga_depan(d, arah, jarak_depan, berhenti, kecepatan=0.4, jarak_maks=4.0):
    awal = d.titik()
    acuan = d.titik()
    tinggi = d.z
    t = sekarang() + 40
    while d.jarak_dari(awal) < jarak_maks and sekarang() < t:
        if berhenti(d):
            break
        depan = d.lidar['front']
        if depan < 2.0:
            maju = batasi(1.0 * (depan - jarak_depan), 0.3)
            acuan = d.titik()
        else:
            maju, _ = d.tahan(acuan)
        d.gerak(maju, arah * kecepatan, batasi(1.0 * (tinggi - d.z), 0.3))
        d.jeda()
    d.diam(0.3)


def tengah_bukaan_tegak(d, batas=2.5, kecepatan=0.3, tinggi_maks=3.8, bukaan_min=0.4):
    acuan = d.titik()
    mulai = None
    t = sekarang() + 60
    while d.z < tinggi_maks and sekarang() < t:
        kosong = d.lidar['front'] > batas
        if kosong and mulai is None:
            mulai = d.z
        if not kosong and mulai is not None:
            if d.z - mulai >= bukaan_min:
                break
            mulai = None
        maju, geser = d.tahan(acuan)
        d.gerak(maju, geser, kecepatan)
        d.jeda()
    d.diam(0.3)
    if mulai is None:
        return 0.0
    tepi_atas = d.z
    d.naik_ke((mulai + tepi_atas) / 2, toleransi=0.05)
    return tepi_atas - mulai


def tengah_bukaan(d, batas=2.5, kecepatan=0.3, jarak_aman=0.4):
    geser_sampai(d, lambda d: d.lidar['front'] < batas or d.lidar['right'] < jarak_aman, kanan=kecepatan, jarak_maks=1.5)
    tepi_kanan = d.titik()
    geser_sampai(d, lambda d: d.lidar['front'] > batas, kanan=-kecepatan, jarak_maks=0.6)
    geser_sampai(d, lambda d: d.lidar['front'] < batas or d.lidar['left'] < jarak_aman, kanan=-kecepatan, jarak_maks=3.0)
    lebar = d.jarak_dari(tepi_kanan)
    d.pindah(kanan=lebar / 2, kecepatan=0.4, toleransi=0.05)
    return lebar
