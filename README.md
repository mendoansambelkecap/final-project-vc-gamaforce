# Final Proyek Oprec Gamaforce (Vision Control)

Kode misi drone Iris buat arena `world_oprec` dari repo oprec Gamaforce ([OPREC_Gamaforce](https://github.com/YournormalGuy1/OPREC_Gamaforce)). Drone lewat semua rintangan di jalur kiri (vision) atau jalur kanan (lidar), tembus ring, lalu nabrak bullseye yang nomornya sesuai ArUco.

Nggak ada koordinat arena di kode. Drone gerak pakai perintah kecepatan lewat MAVROS dan nemuin tiap rintangan sendiri dari kamera dan lidar.

![Peta arena](peta_arena.png)

## Isi folder

```text
.
├── misi_kiri.py      misi penuh jalur kiri (yang dijalanin)
├── misi_kanan.py     misi penuh jalur kanan (yang dijalanin)
├── peta_arena.png
├── log/              log tiap kali misi jalan (dibuat otomatis)
└── lib/              kode pendukung
```

| File di `lib/` | Isi |
|---|---|
| `jalur_kiri.py` | Rintangan jalur kiri aja (gerbang, pipa meliuk, lorong + ArUco, jendela) |
| `jalur_kanan.py` | Rintangan jalur kanan aja (sekat, pilar, terowongan) |
| `bagian_akhir.py` | Bagian yang dipakai dua jalur: lingkaran lantai, ring, kamikaze bullseye |
| `manuver.py` | Gerakan pakai sensor: nyari, ngepasin, nembus bingkai, nyusur dinding |
| `vision.py` | Deteksi warna, lubang, lingkaran, dan ArUco dari gambar kamera |
| `drone.py` | Koneksi MAVROS, takeoff, perintah kecepatan, baca lidar dan kamera |
| `tes_dasar.py` | Tes singkat: takeoff, gerak kotak, mendarat |

Satu jalur aja (tanpa bagian akhir) atau tes dasar bisa dijalanin dari folder utama:

```bash
python3 -m lib.jalur_kiri
```

```bash
python3 -m lib.tes_dasar
```

## Download

Kode di repo ini cuma misinya. Arena, model, dan drone ada di repo arena oprec Gamaforce, jadi butuh **dua repo**, masing-masing di-clone sendiri ke folder `~/oprec`:

```text
~/oprec/
├── gazebo_sim/                    repo arena (dipakai T1 sampai T3)
└── final-project-vc-gamaforce/    repo ini (dipakai T4)
```

**1. Repo arena**, ke `~/oprec/gazebo_sim`. Nama folder ini harus persis, karena semua perintah di README repo arena pakai path itu:

```bash
mkdir -p ~/oprec && cd ~/oprec && git clone https://github.com/YournormalGuy1/OPREC_Gamaforce.git gazebo_sim
```

Kalau repo arena udah ada di `~/oprec/gazebo_sim` dari pelatihan, langkah ini dilewatin.

**2. Repo ini**, di sebelahnya:

```bash
cd ~/oprec && git clone https://github.com/mendoansambelkecap/final-project-vc-gamaforce.git
```

Misinya selalu dijalanin dari dalam folder repo ini (`cd ~/oprec/final-project-vc-gamaforce`), biar folder `lib/` kebaca.

## Cara jalanin

Urutannya ngikutin README repo arena: tiga terminal buat simulasi (T1, T2, T3), ditambah satu terminal buat misi (T4). Repo arena diasumsikan ada di `~/oprec/gazebo_sim`, sama kayak di README-nya.

```text
T1  Randomizer + Gazebo     (world & target)
T2  SITL ArduCopter         (flight controller)
T3  Bridge sensor + MAVROS  (data sensor & kontrol)
T4  Misi                    (kode di folder ini)
```

### T1: Randomizer, lalu Gazebo

Randomizer dijalanin **dulu**, baru Gazebo. Gazebo cuma baca gambar marker waktu start, jadi kalau urutannya kebalik ArUco di arena nggak ikut berubah.

```bash
cd ~/oprec/gazebo_sim/worlds/world_oprec
python3 randomize.py --model-dir ../../models/Part/obstacles
```

```bash
sim_env
export GZ_SIM_RESOURCE_PATH=$HOME/oprec/gazebo_sim/models/Part:$GZ_SIM_RESOURCE_PATH
gz sim -r world_oprec.world
```

Dua baris itu beda sedikit dari README repo arena, dan dua-duanya udah dicek:

- **`--model-dir ../../models/Part/obstacles`.** Tanpa ini, `randomize.py` nulis gambar marker baru ke `worlds/models/obstacles/`, padahal arena makai `models/Part/obstacles/`. Akibatnya ArUco di arena nggak berubah, sementara `target.json` nyatet angka baru, jadi dua-duanya bisa nggak cocok.
- **`GZ_SIM_RESOURCE_PATH`.** Tanpa ini Gazebo nggak nemu model arena (`Unable to find uri[model://MAZE]` dan seterusnya). Kalau path ini udah ada di `~/.bashrc`, baris `export`-nya nggak perlu.

### T2: SITL

```bash
ardu
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console
```

Tunggu sampai muncul `EKF3 IMU origin set` dan status di console hijau. Sebelum itu drone nggak mau takeoff.

### T3: Bridge sensor + MAVROS

```bash
ros2 launch ~/oprec/gazebo_sim/scripts/sensors_mavros.launch.py
```

Cek sensor udah masuk:

```bash
ros2 topic list | grep -E "lidar|camera"
```

Harus muncul `/lidar_left`, `/lidar_right`, `/lidar_up`, `/lidar_down`, `/lidar_front`, `/lidar_back`, `/camera_front`, `/camera_down`.

Bisa juga pakai `cek_sensor.sh` buat nampilin sensor yang kedeteksi sekaligus ngecek kalau semuanya ada:

```bash
./cek_sensor.sh
```

Kalau ada yang kurang, script ini nampilin topic yang hilang dan keluar dengan kode error 1.

### T4: Misi

Setelah T1 sampai T3 jalan semua:

```bash
cd ~/oprec/final-project-vc-gamaforce
python3 misi_kiri.py
```

```bash
python3 misi_kanan.py --target 2
```

Misinya ngurus arming dan takeoff sendiri, jadi di console MAVProxy nggak perlu ngetik apa-apa.

Pesan dari drone tetap muncul di terminal, dan juga disimpan ke `log/drone-YYYY-MM-DD-HH-MM-SS.log`. Folder `log/` dibuat sendiri waktu misi jalan, dan tiap kali jalan dapat file baru.

### Ngulang misi

Di akhir misi drone nabrak bullseye dan biasanya jatuh, jadi simulasinya harus dimulai ulang dari T1:

1. Tutup Gazebo, SITL, dan MAVROS (`Ctrl+C` di tiap terminal).
2. Jalanin `randomize.py` lagi kalau mau ArUco baru.
3. Ulangi T1 sampai T4.

Kalau Gazebo lama masih nyangkut: `pkill -f "gz sim"`.

### Pilihan

| Pilihan | Dipakai di | Fungsi |
|---|---|---|
| `--target N` | `misi_kanan.py` | Nomor bullseye (1, 2, atau 3). Jalur kanan nggak punya ArUco, jadi targetnya diisi dari sini |
| `--target N` | `misi_kiri.py` | Nimpa hasil bacaan ArUco. Normalnya nggak diisi |
| `--json path` | `misi_kanan.py` | Lokasi `target.json`. Cuma dibaca kalau `--target` kosong |

Kalau `--target` di `misi_kanan.py` kosong, dia baca `target.json`, dan kalau file itu nggak ada dia ke bullseye 2. Buat penilaian, selalu isi `--target`.

### Kalau simulasinya lambat

Semua batas waktu di kode dihitung pakai jam biasa. Kalau simulasi jalan lebih lambat dari waktu asli (misalnya laptop tanpa GPU, atau menggunakan VM), kaliin batas waktunya pakai `SKALA_WAKTU`:

```bash
SKALA_WAKTU=4 python3 misi_kiri.py
```

Di mesin yang simulasinya real-time nggak perlu diisi.

## Cara kerja

Tiap rintangan polanya sama: cari sampai keliatan di sensor, pasin posisi, lewat.

### Jalur kiri

| Rintangan | Cara nemuin dan lewat |
|---|---|
| Gerbang miring | Geser ke kiri sampai dua kaki gerbang oranye keliatan di kamera depan. Pasin tengahnya, maju sampai lidar atas ngerasain palang lewat |
| Pipa meliuk | Naik, cari lubang dari tengah gambar, tembus. Maju sampai lingkaran hijau keliatan di kamera bawah, lalu pasin di atasnya |
| Lorong L | Lidar kiri-kanan jaga drone di tengah, lidar depan buat berhenti di ujung. Kamera depan baca ArUco berkali-kali, hasil terbanyak yang dipakai |
| Dinding oranye | Mundur sampai seluruh dinding keliatan, kunci lubang yang paling besar, pasin, tembus |
| Jendela cyan, hijau, magenta | Naik atau turun sampai warnanya keliatan, kalau perlu mundur dulu. Di dalam bingkai, lidar empat arah yang jaga tengah |

### Jalur kanan

| Rintangan | Cara nemuin dan lewat |
|---|---|
| Pintu maze | Geser ke kanan sampai lidar depan kena maze, terus sampai depan lega lagi. Itu celah sekat pertama |
| Sekat zig-zag | Maju sampai depan mentok, geser sampai depan lega, tempel dinding pakai lidar samping |
| Sekat rendah | Naik sampai lidar depan kosong, lompatin di bawah batas tinggi dinding. Lidar bawah yang ngasih tahu udah lewat |
| Lingkaran ungu, pilar | Pasin di atas lingkaran ungu pakai kamera bawah, turun, geser ke lajur kiri pilar pakai lidar kiri |
| Terowongan | Ukur tepi bawah-atas dan kiri-kanan mulut pipa pakai lidar depan, masuk di tengahnya. Di dalam, lidar atas-bawah jaga ketinggian. Di ruang akhir, naik sampai depan kosong lalu keluar |

### Bagian akhir

1. Muter sampai lingkaran hijau keliatan di kamera depan, terbang ke atasnya, pasin pakai kamera bawah.
2. Muter sampai ring keliatan, pasin, tembus.
3. Geser sampai bullseye yang warnanya cocok ada di depan (1 biru di kiri, 2 oranye di tengah, 3 hijau kekuningan di kanan).
4. Bidik pusatnya, lalu maju lurus 2 m/s sampai nabrak.

### Deteksi di `lib/vision.py`

Cuma pakai fungsi OpenCV dari materi pelatihan oprec: `cvtColor`, `inRange`, `bitwise_or`, `morphologyEx`, `findContours`, `boundingRect`, ditambah `cv2.aruco` buat baca marker.

- **Benda** (lingkaran, bullseye): mask warna HSV, ambil kontur terbesar, pusatnya dari kotak pembatas.
- **Lubang utuh** (jendela, ring): `findContours` mode `RETR_CCOMP`, kontur dalam = lubang.
- **Lubang yang kepotong tepi gambar** (gerbang, pipa, bingkai dari dekat): dari titik tengah gambar, cari tepi mask ke kiri-kanan dan atas-bawah, ambil tengahnya.

## Yang masih berupa angka di kode

Nggak ada posisi rintangan. Yang ditulis langsung cuma:

- ketinggian terbang, misalnya 0,6 m di lorong dan batas 1,96 m di maze (dinding 2,15 m),
- jarak relatif pendek, misalnya maju 0,3 m setelah lewat sekat,
- rentang warna HSV tiap rintangan,
- urutan rintangan dan posisi kasar bullseye (1 di kiri, 3 di kanan).

Posisi drone dari MAVROS cuma dipakai buat ketinggian dan ngukur jarak tempuh relatif.

## Hasil tes

Dites di simulasi (Gazebo Harmonic, ArduCopter SITL, ROS 2 Jazzy, MAVROS).

| Misi | Mesin | Target | Hasil | Waktu |
|---|---|---|---|---|
| Kiri | macOS | ArUco 1 | Kena 1 cm dari pusat (horizontal), 12 cm di bawah | 253 detik |
| Kiri | Ubuntu 24.04 | ArUco 3 | Kena 4 cm dari pusat, 10 cm di bawah | 959 detik (simulasi 0,26× kecepatan asli) |
| Kanan | macOS | 2 | Kena 1 cm dari pusat, 9 cm di bawah | 195 detik |
| Kanan | Ubuntu 24.04 | 1 | Kena 1 cm dari pusat, 12 cm di bawah | 802 detik (simulasi 0,26× kecepatan asli) |

Zona tengah bullseye (pengali ×1,0) radiusnya 1,58 m.

Jarak ke rintangan dari rekaman lintasan:

- **Kiri:** pipa meliuk 47 cm dari pipa, jendela meleset 1–6 cm dari tengah, tinggi di lorong maksimal 0,61 m.
- **Kanan:** titik paling sempit di sekat rendah, pusat drone 29 cm di atas puncak sekat dan puncak drone sekitar 2,06 m.

## Catatan

- Rentang warna disetel dari warna di simulasi. Kalau warna atau pencahayaan arena berubah, `WARNA` di `lib/vision.py` perlu disetel ulang.
- Gerbang, pipa meliuk, dan jendela di jalur kiri nggak punya collision di world, jadi drone nggak ketahan kalau nyenggol. Jarak di atas diukur dari rekaman posisi.
- Lingkaran hijau pertama (di bawah gerbang miring) posisinya sedikit di bawah lantai dan nggak keliatan kamera, jadi nggak dipakai.
- Lidar tiap arah cuma satu sinar, jadi benda yang nggak pas di garis sinar nggak kebaca.
- Setelah nabrak bullseye drone biasanya jatuh, dan di akhir muncul pesan `TIMEOUT nunggu mendarat`. Misinya udah selesai di titik itu.
- Tiap jalur baru lolos beberapa kali, belum diuji puluhan kali dengan ArUco acak.
