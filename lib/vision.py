#!/usr/bin/env python3
import cv2
import numpy as np

WARNA = {
    'oranye_gerbang': [((14, 150, 50), (17, 255, 255))],
    'oranye_dinding': [((20, 170, 50), (24, 228, 255))],
    'hijau': [((50, 120, 40), (72, 255, 255))],
    'cyan': [((84, 120, 40), (100, 255, 255))],
    'magenta': [((145, 120, 40), (157, 255, 255))],
    'ungu': [((158, 120, 40), (168, 255, 255))],
    'ring': [((0, 130, 50), (8, 205, 255))],
    'bullseye1': [((115, 208, 60), (132, 255, 255))],
    'bullseye2': [((2, 215, 60), (15, 255, 255))],
    'bullseye3': [((24, 215, 60), (42, 255, 255))],
    'merah': [((0, 120, 40), (8, 255, 255)), ((172, 120, 40), (180, 255, 255))],
    'gelap': [((0, 0, 0), (180, 90, 70))],
}

KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
KAMUS = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def mask_warna(bgr, *nama):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(bgr.shape[:2], np.uint8)
    for n in nama:
        for bawah, atas in WARNA[n]:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, np.array(bawah), np.array(atas)))
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)


LEBAR = 640
TINGGI = 480


def kontur_terbesar(daftar_kontur, luas_min):
    terbaik = None
    luas_terbaik = luas_min
    for k in daftar_kontur:
        x, y, w, h = cv2.boundingRect(k)
        if w * h >= luas_terbaik:
            terbaik = k
            luas_terbaik = w * h
    return terbaik


def kotak(bgr, *nama, luas_min=1500):
    kontur, _ = cv2.findContours(mask_warna(bgr, *nama), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    k = kontur_terbesar(kontur, luas_min)
    if k is None:
        return None
    return cv2.boundingRect(k)


def nempel_tepi(k):
    x, y, w, h = k
    return x <= 2 or y <= 2 or x + w >= LEBAR - 2 or y + h >= TINGGI - 2


def pusat_benda(bgr, *nama, luas_min=1500):
    k = kotak(bgr, *nama, luas_min=luas_min)
    if k is None:
        return None
    x, y, w, h = k
    return x + w / 2, y + h / 2, max(w, h) / 2


def daftar_lubang(bgr, *nama, luas_min=800):
    kontur, hirarki = cv2.findContours(mask_warna(bgr, *nama), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    hasil = []
    if hirarki is None:
        return hasil
    for k, h in zip(kontur, hirarki[0]):
        if h[3] == -1:
            continue
        kotak_lubang = cv2.boundingRect(k)
        x, y, w, t = kotak_lubang
        if w * t >= luas_min and not nempel_tepi(kotak_lubang):
            hasil.append((x + w / 2, y + t / 2, w * t))
    return hasil


def lubang_utuh(bgr, *nama, luas_min=800):
    semua = daftar_lubang(bgr, *nama, luas_min=luas_min)
    if not semua:
        return None
    return max(semua, key=lambda target: target[2])


def lubang_terdekat(bgr, titik, *nama, luas_min=800):
    semua = daftar_lubang(bgr, *nama, luas_min=luas_min)
    if not semua:
        return None
    return min(semua, key=lambda target: (target[0] - titik[0]) ** 2 + (target[1] - titik[1]) ** 2)


def lubang_tengah(bgr, *nama, luas_min=1500, ulang=3):
    mask = mask_warna(bgr, *nama)
    if (mask > 0).sum() < luas_min:
        return None
    u, v = LEBAR // 2, TINGGI // 2
    for _ in range(ulang):
        if mask[v, u] > 0:
            return None
        kolom = np.where(mask[v, :] > 0)[0]
        kiri = kolom[kolom < u]
        kanan = kolom[kolom > u]
        tepi_kiri = kiri.max() if len(kiri) else 0
        tepi_kanan = kanan.min() if len(kanan) else LEBAR - 1
        u = int((tepi_kiri + tepi_kanan) // 2)
        baris = np.where(mask[:, u] > 0)[0]
        atas = baris[baris < v]
        bawah = baris[baris > v]
        tepi_atas = atas.max() if len(atas) else 0
        tepi_bawah = bawah.min() if len(bawah) else TINGGI - 1
        v = int((tepi_atas + tepi_bawah) // 2)
    lengkap_datar = len(kiri) > 0 and len(kanan) > 0
    lengkap_tegak = len(atas) > 0 and len(bawah) > 0
    return float(u), float(v), float(tepi_kanan - tepi_kiri) / 2, lengkap_datar, lengkap_tegak


def pusat_lubang(bgr, *nama):
    hasil = lubang_tengah(bgr, *nama)
    if hasil is None:
        return None
    return hasil[0], hasil[1], hasil[2]


def baca_aruco(bgr):
    abu = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        _, ids, _ = cv2.aruco.ArucoDetector(KAMUS).detectMarkers(abu)
    else:
        _, ids, _ = cv2.aruco.detectMarkers(abu, KAMUS)
    if ids is None:
        return None
    return int(ids.ravel()[0])
