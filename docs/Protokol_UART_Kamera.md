# Protokol UART ESP32-S3 Utama <-> Modul Kamera (Deteksi Area Kotor)

Robot Pembersih Lantai Otonom, [NAMA TIM]. Versi protokol **1** (`PK_VERSI`), [TANGGAL].
Implementasi: `06 Implementasi\09 Firmware\main-esp32s3\protokol_kamera.h` dan salinan **identik** di
`06 Implementasi\09 Firmware\visual-esp32cam\protokol_kamera.h`. Bila protokol diubah, ubah kedua salinan dan naikkan `PK_VERSI`.

## 1. Lapisan fisik

| Item | Nilai |
|---|---|
| ESP32-S3 utama | UART1, **TX = GPIO39 -> U0R kamera**, **RX = GPIO40 <- U0T kamera** (CN19 pin 3/4, sudah disilang di PCB) |
| Kamera | UART0 (label header **U0R / U0T**): AI-Thinker = GPIO3 / GPIO1; ESP32-S3-CAM = GPIO44 / GPIO43 (**[CEK]** sablon papan) |
| Format | 115200 baud, 8N1, tanpa flow control, level 3,3 V, GND bersama wajib |
| Daya kamera | 5 V dari CN19 pin 1. **Jangan mencolok USB kamera saat CN19 tersambung dan 5 V robot menyala** (README PCB bagian 5) |

Catatan: pada AI-Thinker, UART0 juga dipakai log boot ROM dan konsol debug. Karena byte awal bingkai **0xA5 > 0x7F**,
teks ASCII tidak pernah dikira bingkai; ESP32-S3 mengabaikan semua byte di luar bingkai. Sebaliknya, kamera menerima
perintah teks (mis. `status`) di jalur yang sama untuk uji di meja.

## 2. Format bingkai

```
+------+------+------+------+---------------------+------+
| SOF  | TIPE | SEQ  | LEN  | PAYLOAD (LEN byte)  | CRC8 |
| 0xA5 | 1 B  | 1 B  | 0-32 | little-endian       | 1 B  |
+------+------+------+------+---------------------+------+
```

* **SOF** = 0xA5.
* **TIPE** = jenis pesan (bagian 3). Bit 7 = 1 untuk pesan dari kamera.
* **SEQ** = nomor urut pengirim (0-255, berulang). Hanya untuk diagnosis (deteksi bingkai hilang), tidak dibalas.
* **LEN** = panjang payload 0..32. LEN > 32 = bingkai rusak, pengurai kembali mencari SOF.
* **CRC8** = CRC-8/SMBUS: polinom 0x07, nilai awal 0x00, tanpa refleksi, tanpa XOR akhir, dihitung atas
  **TIPE, SEQ, LEN, PAYLOAD** (SOF tidak ikut). Nilai cek: CRC dari ASCII "123456789" = **0xF4**.
* Bilangan multi-byte **little-endian**. Panjang bingkai maksimum 37 byte (~3,2 ms pada 115200).
* Bingkai dengan CRC salah dibuang tanpa balasan (pengirim berikutnya otomatis memulihkan: heartbeat/hasil periodik,
  dan ESP32-S3 mengirim ulang MULAI/PING tiap 1 s bila perlu).
* Penerima **mengabaikan payload yang lebih panjang** dari yang ia kenal (perluasan masa depan menambah byte di akhir).

## 3. Daftar pesan

### 3.1 ESP32-S3 -> kamera

| TIPE | Nama | LEN | Payload | Balasan |
|---|---|---|---|---|
| 0x01 | PING | 0 | - | HEARTBEAT segera |
| 0x02 | MULAI (start deteksi) | 0 atau 2 | u16 periode_ms (50..1000, opsional, bawaan 150) | ACK, lalu KOTOR tiap periode |
| 0x03 | BERHENTI (stop deteksi) | 0 | - | ACK |
| 0x04 | MINTA_STATUS | 0 | - | STATUS |
| 0x05 | SET_PARAM (set threshold) | 3 | u8 id, i16 nilai (tabel 3.3) | ACK (status 0 = OK, 2 = nilai salah). Nilai disimpan di NVS kamera |
| 0x06 | KALIBRASI | 0 | - | ACK, lalu STATUS saat selesai (~6-10 s) atau GALAT kode 3 |
| 0x07 | HAPUS_KALIB | 0 | - | ACK |

### 3.2 Kamera -> ESP32-S3

| TIPE | Nama | LEN | Payload |
|---|---|---|---|
| 0x81 | HEARTBEAT (juga balasan PONG) | 8 | [0] u8 flags status, [1..2] u16 fps x10, [3..4] u16 PID sensor (0x0026 OV2640, 0x3660 OV3660), [5] u8 jumlah frame gagal (jenuh 255), [6..7] u16 uptime detik |
| 0x82 | STATUS | 12 | [0] flags status, [1] u8 floorMean, [2] u8 floorStd, [3] u8 darkThr, [4..5] u16 normalDark (px), [6..7] u16 dirtyLimit (px), [8..9] u16 periode_ms, [10] u8 progres kalibrasi 0-100, [11] u8 ID papan (1 AI-Thinker, 2 ESP32-S3-EYE, 3 kustom) |
| 0x83 | KOTOR (hasil deteksi) | 12 | [0] u8 **skor 0-100**, [1] u8 flags hasil, [2] u8 **region** (bit0 kiri, bit1 tengah, bit2 kanan), [3..5] u8 skor kolom kiri/tengah/kanan, [6..7] u16 jumlah dark pixel D, [8..9] u16 batas kotor L, [10] u8 rata-rata kecerahan ROI, [11] u8 nomor frame (berulang) |
| 0x8F | ACK | 3 | [0] TIPE perintah yang dibalas, [1] id parameter (SET_PARAM) atau 0, [2] status: 0 OK, 1 tidak dikenal, 2 nilai salah, 3 sibuk, 4 kamera gagal |
| 0xEE | GALAT (ERROR) | 2 | [0] kode: 1 init kamera gagal, 2 frame gagal beruntun, 3 kalibrasi gagal, 4 bingkai; [1] detail |

**Flags status** (HEARTBEAT/STATUS byte 0): bit0 deteksi aktif, bit1 terkalibrasi, bit2 kalibrasi sedang berjalan,
bit3 kamera OK, bit4 LED flash nyala.

**Flags hasil** (KOTOR byte 1): bit0 **KOTOR** (keputusan setelah histeresis), bit1 kandidat (frame ini D > L),
bit2 ambang dari kalibrasi, bit3 cahaya aneh (|mean - floorMean| > 40: silau/gelap, hasil kurang dapat dipercaya).

**Skor**: `skor = 50 x (D - normalDark) / (L - normalDark)`, dibatasi 0..100. Skor 50 = tepat di batas kotor,
0 = sama dengan lantai bersih, 100 = dua kali selisih batas. Region dan skor kolom memakai hitungan per sepertiga ROI
(kiri gambar = kiri robot setelah orientasi dicek dengan perintah `ascii`).

### 3.3 ID parameter SET_PARAM

| id | Nama | Rentang | Bawaan | Arti |
|---|---|---|---|---|
| 0x01 | PERIODE_MS | 50..1000 | 150 | periode hasil KOTOR |
| 0x02 | K_GELAP_X10 | 5..100 | 30 | darkThr = floorMean - max(k x floorStd, delta); k = nilai/10 |
| 0x03 | DELTA_GELAP | 5..120 | 20 | selisih minimum lantai - ambang gelap (level abu) |
| 0x04 | RASIO_BATAS_X100 | 105..400 | 145 | L = max(normal x rasio/100, normal + 4 sd, normal + 40) |
| 0x05 | HIST_NYALA | 1..20 | 3 | frame kandidat berturut-turut untuk KOTOR = 1 |
| 0x06 | HIST_MATI | 1..20 | 3 | frame bukan kandidat berturut-turut untuk KOTOR = 0 |
| 0x07 | BATAS_MANUAL | 0..32767 | 0 | L absolut (px); 0 = otomatis dari kalibrasi |
| 0x08 | GELAP_MANUAL | 0..255 | 0 | darkThr absolut; 0 = otomatis |
| 0x09 | KOMPENSASI | 0/1 | 1 | ambang digeser mengikuti (mean frame - floorMean), maks +-40 |
| 0x0A | LED_FLASH | 0/1 | **0** | LED flash kamera (mati: cegah silau di jendela) |

## 4. Metode deteksi (ringkas, sesuai uji tim)

Mengikuti `07 Dokumentasi\Software\Arsitektur_Dirt_Detection.png` dan hasil uji `Uji_Dirt_Detection_area_bersih/kotor.png`
(lantai keramik: normal 322 px, batas 478 px; area bersih 329-343 px -> DIRT 0, area kotor 516-555 px -> DIRT 1):

1. Frame 160 x 120 grayscale, ROI lantai 120 x 65 piksel (x 20..139, y 50..114, bagian bawah = dekat robot).
2. Kalibrasi fase 1 (20 frame lantai bersih): floorMean dan sebaran piksel -> **darkThr**.
3. Kalibrasi fase 2 (20 frame): rata-rata jumlah dark pixel -> **normalDark** (garis nat keramik ikut terhitung di sini) ->
   **dirtyLimit**. Disimpan di NVS kamera.
4. Runtime tiap 150 ms: D = jumlah piksel ROI < darkThr; D > dirtyLimit = kandidat; histeresis 3 nyala / 3 mati.

## 5. Waktu

| Item | Nilai |
|---|---|
| Hasil KOTOR | tiap 150 ms (~6,7 Hz) selama deteksi aktif; rentang yang diizinkan 50..1000 ms (target 5-10 Hz) |
| HEARTBEAT | 1 Hz selalu (juga saat deteksi berhenti dan saat kamera gagal init) |
| Kamera dianggap TIDAK ADA | tidak ada bingkai valid apa pun selama **3 s** |
| PING dari ESP32-S3 | bila tidak ada bingkai > 1,5 s, maksimal 1 PING per detik |
| Pemeliharaan | tiap 1 s ESP32-S3 membandingkan flag "deteksi aktif" dengan keinginannya; bila beda kirim MULAI/BERHENTI (memulihkan kamera yang reboot) |
| Hasil KOTOR dianggap basi | > 1 s sejak hasil terakhir -> diperlakukan "tidak kotor" |
| Kalibrasi | 40 frame, ~100 ms per frame -> ~4-8 s; robot harus diam di lantai bersih |
| Boot kamera | ~1-2 s sampai HEARTBEAT pertama |

## 6. Pemakaian oleh FSM ESP32-S3

* Saat **CLEANING** mulai, ESP32-S3 mengirim **MULAI (150 ms)**; saat IDLE/berhenti/galat mengirim **BERHENTI**.
* Hasil dipakai hanya bila kamera ada, hasil < 1 s, dan (parameter `kotorkalib` = 1) bit "terkalibrasi" = 1.
* Bila **KOTOR = 1** (parameter `modekotor`):
  * `1` (bawaan) **perlambat**: kecepatan lajur turun ke `vkotor` (0,10 m/s) sampai robot menempuh `jarakkamera`
    (0,30 m, jarak titik pandang kamera ke nozzle, **[UKUR]**) setelah deteksi kotor terakhir, sehingga nozzle
    melewati titik kotor dengan pelan.
  * `2` **perlambat + spot clean**: setelah episode kotor terlewati, robot mundur sepanjang episode (+0,10 m, maks 0,5 m)
    lalu melintas sekali lagi dengan pelan (sekali per titik).
  * `0` abaikan kamera.
* LED robot cyan saat kotor terdeteksi.
* **Kamera tidak ada** -> robot tetap membersihkan tanpa deteksi kotor (kecepatan lajur normal), status dicetak di `status`.
* Region (kiri/tengah/kanan) dikirim untuk pengembangan berikutnya (mis. geser lajur ke arah kotor); FSM saat ini hanya
  memakai bit KOTOR.

## 7. Contoh byte (CRC sudah dihitung)

| Arah | Pesan | Byte (hex) |
|---|---|---|
| S3 -> kamera | PING, seq 0 | `A5 01 00 00 6B` |
| S3 -> kamera | MULAI periode 150 ms (0x0096), seq 1 | `A5 02 01 02 96 00 9B` |
| S3 -> kamera | SET_PARAM RASIO_BATAS_X100 = 160 (0x00A0), seq 2 | `A5 05 02 03 04 A0 00 C0` |
| S3 -> kamera | KALIBRASI, seq 3 | `A5 06 03 00 42` |
| S3 -> kamera | BERHENTI, seq 4 | `A5 03 04 00 E9` |
| kamera -> S3 | HEARTBEAT: flags 0x0B (deteksi, terkalibrasi, kamera OK), 6,7 fps (67 = 0x0043), OV3660 (0x3660), 0 frame gagal, uptime 125 s | `A5 81 10 08 0B 43 00 60 36 00 7D 00 55` |
| kamera -> S3 | KOTOR (kotor): skor 71 (0x47), flags 0x07, region kanan (0x04), skor kolom 0/12/100, D 546 (0x0222), L 478 (0x01DE), mean 148 (0x94), frame 42 | `A5 83 11 0C 47 07 04 00 0C 64 22 02 DE 01 94 2A 06` |
| kamera -> S3 | KOTOR (bersih): skor 4, flags 0x04, region 0, skor kolom 2/5/3, D 336 (0x0150), L 478, mean 151, frame 43 | `A5 83 12 0C 04 04 00 02 05 03 50 01 DE 01 97 2B FB` |
| kamera -> S3 | ACK untuk SET_PARAM id 0x04 status OK | `A5 8F 13 03 05 04 00 46` |
| kamera -> S3 | STATUS: flags 0x0B, floorMean 150, floorStd 8, darkThr 126, normal 322, batas 478, periode 150, progres 100, papan 1 | `A5 82 14 0C 0B 96 08 7E 42 01 DE 01 96 00 64 01 05` |
| kamera -> S3 | GALAT init kamera gagal | `A5 EE 15 02 01 00 83` |

Contoh perhitungan CRC untuk PING: data = `01 00 00` -> CRC-8/SMBUS = `6B`.

## 8. Uji protokol

1. Kamera saja (USB-TTL / ESP32-CAM-MB, 115200): ketik `status`, `kalib`, `debug 1`, `mulai` -> baris
   `DIRT:0 dark:... (normal ..., batas ...)` seperti log uji tim; byte biner (karakter aneh) di terminal adalah bingkai.
2. Tersambung ke ESP32-S3: perintah CLI `kamera` harus menunjukkan "ada", fps, dan jumlah bingkai naik tanpa "CRC salah".
3. `kamera echo 1` lalu `kamera mulai`: hasil KOTOR tercetak; letakkan kotoran di depan kamera -> `KOTOR:1`.
4. Cabut kabel RX kamera selama > 3 s: `status` harus menyatakan kamera TIDAK ADA; robot tetap bisa `mulai`.
