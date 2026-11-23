# Firmware Modul Visual (`visual-esp32cam`)

Robot Pembersih Lantai Otonom, [NAMA TIM]. Versi firmware 0.1.0 (rancangan pertama, **belum diuji di papan**).

Deteksi area kotor di lantai depan robot dengan kamera OV3660/OV2640, hasil dikirim ke ESP32-S3 utama lewat UART
(protokol biner, `06 Implementasi\07 Integrasi\Protokol_UART_Kamera.md`). Metode sama dengan uji tim
(`07 Dokumentasi\Software\Arsitektur_Dirt_Detection.png`): frame 160x120 grayscale, ROI lantai 120x65, kalibrasi lantai bersih
2 fase (floorMean -> darkThr, normalDark -> dirtyLimit), hitung dark pixel tiap 150 ms, histeresis 3 nyala / 3 mati.
Tambahan: ambang mengikuti kecerahan frame (eksposur otomatis), skor 0-100, region kiri/tengah/kanan, kalibrasi tersimpan di NVS.
LED flash **mati** secara bawaan (silau pada jendela kamera).

## 1. Struktur

| Berkas | Isi |
|---|---|
| `visual-esp32cam.ino` | init kamera, kalibrasi, deteksi, protokol, konsol teks |
| `papan_kamera.h` | **pemilihan papan** + peta pin kamera dan UART |
| `protokol_kamera.h` | definisi protokol, **identik** dengan salinan di `main-esp32s3` |

## 2. Memilih papan

Papan yang ada berlabel header gaya AI-Thinker (3V3 IO16 IO0 GND VCC U0R U0T GND / 5V GND IO12 IO13 IO15 IO14 IO2 IO4)
dengan OV3660, tetapi bisa ESP32 klasik atau klon ESP32-S3. Cek tulisan di kaleng modul (ESP32-S / ESP32-S3-WROOM) lalu:

| Varian | Board Arduino | `PAPAN_KAMERA` | Peta pin |
|---|---|---|---|
| AI-Thinker ESP32-CAM (ESP32) | **AI Thinker ESP32-CAM** (`esp32:esp32:esp32cam`) | otomatis / `PAPAN_AITHINKER` | baku AI-Thinker (PWDN 32, XCLK 0, SIOD 26, SIOC 27, D0-D7 5/18/19/21/36/39/34/35, VSYNC 25, HREF 23, PCLK 22), LED flash GPIO4, link U0R=GPIO3 / U0T=GPIO1 |
| ESP32-S3-CAM | **ESP32S3 Dev Module** (`esp32:esp32:esp32s3`), PSRAM sesuai modul: N8R8/N16R8 = **OPI PSRAM**, N8R2/N4R2 = **QSPI PSRAM** | otomatis / `PAPAN_S3_EYE` | **ASUMSI** peta ESP32-S3-EYE / Freenove (XCLK 15, SIOD 4, SIOC 5, D0-D7 11/9/8/10/12/18/17/16, VSYNC 6, HREF 7, PCLK 13, tanpa PWDN/RESET), link U0R=GPIO44 / U0T=GPIO43 |
| Lainnya | sesuai chip | `PAPAN_KUSTOM` | isi sendiri dari skematik |

**Peta pin ESP32-S3 WAJIB diverifikasi** dengan skematik/penjual. Klon ESP32-S3-CAM berbentuk AI-Thinker bisa berbeda.
Gejala salah: konsol menulis `init kamera GAGAL 0x...`. Tanpa `#define`, pilihan otomatis dari target chip; bila papan dan
target tidak cocok, kompilasi berhenti dengan `#error`.

Pada ESP32-S3: USB CDC On Boot **Disabled** -> konsol dan link berbagi UART0 (sama seperti AI-Thinker). **Enabled** -> konsol
teks di USB native, link tetap di UART0 (berguna untuk debug sambil tersambung ke robot - **tetapi lihat peringatan USB di bagian 3**).

Kompilasi:

```
arduino-cli compile -b esp32:esp32:esp32cam --build-path %TEMP%\build_cam visual-esp32cam
arduino-cli compile -b "esp32:esp32:esp32s3:PSRAM=opi" --build-path %TEMP%\build_cam_s3 visual-esp32cam
```

Pustaka: hanya bawaan core (esp32-camera, Preferences).

## 3. Cara memprogram (flash)

> **PERINGATAN:** jangan pernah mencolok USB kamera (atau USB-TTL yang memberi 5 V) saat kabel **CN19** tersambung dan 5 V robot
> menyala: VBUS USB dan rel 5 V robot bisa saling mengisi (README PCB bagian 5). **Cabut CN19 dulu**, program kamera terpisah.

**AI-Thinker ESP32-CAM:**
1. Pakai papan **ESP32-CAM-MB** (tinggal colok, tekan IO0 + RST bila upload tidak mulai) **atau** USB-TTL 3,3 V:
   USB-TTL TX -> U0R, RX -> U0T, GND -> GND, 5 V -> 5V (kamera butuh 5 V stabil; catu 3,3 V sering brownout).
2. Hubungkan **IO0 ke GND**, tekan RST (atau cabut-pasang daya) -> mode boot unduh.
3. Upload (Board AI Thinker ESP32-CAM, Upload Speed 115200/460800).
4. **Lepas IO0 dari GND**, tekan RST -> program berjalan. Serial Monitor 115200, akhir baris Newline.

**ESP32-S3-CAM:** colok USB-C (port USB-UART atau native), pilih port, upload. Bila gagal: tahan BOOT (IO0), tekan RST, lepas BOOT.

## 4. Konfigurasi yang perlu diisi / dicek

| Item | Tempat | Nilai sementara | Cara |
|---|---|---|---|
| Papan dan peta pin S3 | `papan_kamera.h` | otomatis | skematik papan |
| ROI lantai `ROI_X0/Y0/W/H` | `.ino` | 20, 50, 120, 65 (sama dengan uji tim) | perintah `ascii` setelah kamera terpasang di bumper (ROI harus melihat lantai, bukan bumper/jendela) |
| `CAM_VFLIP`, `CAM_HMIRROR` | `.ino` | -1 (otomatis: OV3660 vflip 1) | `ascii`: atas gambar = jauh, kiri gambar = kiri robot |
| Ambang (`set <id> <nilai>`) | NVS kamera | k 3,0, delta 20, rasio 1,45, histeresis 3/3 | uji di lantai asli, bandingkan dengan log tim (normal 322, batas 478) |
| LED flash | `led 0` atau `led 1` | 0 (mati) | biarkan mati kecuali ruang gelap |
| Kalibrasi lantai | `kalib` | belum ada | robot diam di lantai bersih, ~6 s |

## 5. Konsol teks (115200)

`bantuan`, `status`, `mulai`, `berhenti`, `kalib`, `hapus`, `debug 0|1` (baris `DIRT:0 dark:336 px (normal 322, batas 478)
mean:150.7 (lantai 149.7) ...` seperti log uji tim), `led 0|1`, `set <id> <nilai>`, `ascii` (gambar ASCII 40x24 dengan bingkai
ROI untuk mengarahkan kamera tanpa Wi-Fi). Karakter aneh di terminal = bingkai biner protokol (normal).

## 6. Urutan uji

1. Papan saja lewat USB-TTL: boot -> `kamera OK: sensor PID 0x3660 (OV3660)`, `status` fps > 0 setelah `mulai`.
2. `ascii` di atas lantai: cek orientasi dan ROI.
3. `kalib` di lantai bersih -> `KALIBRASI SELESAI: normalDark ... dirtyLimit ...`.
4. `debug 1`, `mulai`: lantai bersih `DIRT:0`; taruh kotoran (debu, tanah, tumpahan) di ROI -> `DIRT:1` setelah 3 frame.
5. Pasang di robot (CN19, **USB dicabut**): CLI utama `kamera` -> ada, fps, bingkai naik tanpa CRC salah.
6. `kamera kalib` dari CLI utama di lantai bersih, lalu `kamera echo 1` dan `mulai` robot.

## 7. Asumsi yang perlu verifikasi

* Peta pin ESP32-S3 (bagian 2) dan pin UART0 pada header U0R/U0T.
* OV3660 mode grayscale QQVGA (bila gagal, kode otomatis mencoba RGB565 lalu mengonversi ke abu).
* ROI dan orientasi bergantung sudut kamera (`CAM_TILT` di desain 3D); kalibrasi ulang setiap posisi kamera / jenis lantai berubah.
* Lantai bermotif kuat atau berwarna gelap dapat memberi normalDark besar; naikkan `set 4 <rasio>` atau pakai `set 7 <batas>`.
