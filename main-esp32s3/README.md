# Firmware Utama ESP32-S3 (`main-esp32s3`)

Robot Pembersih Lantai Otonom, [NAMA TIM]. Versi firmware 0.1.0 (rancangan pertama, **belum diuji di robot**).

Firmware final untuk ESP32-S3 DevKit N16R8 di PCB utama rev 004: penggerak differential drive dengan PID kecepatan roda,
odometri + heading gyro, sensor jarak/tebing/IMU/RTC, baterai, suction, indikator, sambungan kamera deteksi kotor,
supervisor keselamatan, dan mesin keadaan (FSM) cakupan bolak-balik. Peta pin mengikuti
`06 Implementasi\07 Integrasi\SPEK_BERSAMA.md` bagian 3 dan `06 Implementasi\14 PCB Utama\README.md` bagian 3.
Kode turunan dari sketch `harness-uji` (Bringup_PCB, Uji_PID_Kecepatan_Roda_v2, Kalibrasi_IMU).

## 1. Struktur

| Berkas | Isi |
|---|---|
| `main-esp32s3.ino` | `setup()` (keadaan aman, selftest), `loop()` penjadwal millis (tanpa `delay`) |
| `config.h` | pin, alamat I2C, **semua konstanta yang wajib diukur (tanda `[UKUR]`)**, nilai bawaan parameter |
| `param.h/.cpp` | parameter runtime (`set`, `simpan` ke NVS namespace `robot`) |
| `i2c_bus.h/.cpp` | Wire 100 kHz + PCF8574 (shadow byte) |
| `penggerak.h/.cpp` | TB6612 PWM 20 kHz, encoder kuadratur 4x (interupsi), PID per-sampel Ts 50 ms (0,75/2,25/0,25), profil trapesium 0,5 m/s^2, rem darurat 2,0 m/s^2, odometri RK2 + fusi gyro, pengaman stall/tanda salah (fault terkunci) |
| `sensor_jarak.h/.cpp` | HC-SR04 x3 berurutan non-blocking (TRIG via PCF8574, ECHO interupsi), VL53L0X x2 (urutan alamat 0x30/0x31; jarak via pustaka Pololu atau STUB) |
| `sensor_imu.h/.cpp` | MPU9250 0x69: gyro z (+-500 dps) untuk heading, kemiringan dari akselerometer, bias dari NVS `imu` + estimasi ulang saat diam |
| `rtc_ds3231.h/.cpp` | baca/set waktu, dipakai jadwal mulai harian |
| `daya.h/.cpp` | VBAT (GPIO1, 100k/22k) dengan filter + histeresis; suction ON/OFF (GPIO10) |
| `indikator.h/.cpp` | LED RGB dan buzzer, pola non-blocking |
| `kamera_link.h/.cpp` + `protokol_kamera.h` | protokol UART kamera (`07 Integrasi\Protokol_UART_Kamera.md`). `protokol_kamera.h` **identik** dengan salinan di `visual-esp32cam` |
| `keselamatan.h/.cpp` | supervisor: tebing, miring, fault roda, baterai kritis; memaksa batas gerak/suction **setelah** FSM |
| `fsm.h/.cpp` | state BOOT, IDLE, BERSIH (CLEANING), HINDAR (AVOID), TEBING (CLIFF_ESCAPE), BATERAI (LOW_BATTERY_STOP), GALAT (ERROR), UJI; manuver (maju/mundur/putar dengan heading gyro); cakupan bolak-balik |
| `cli.h/.cpp` | perintah serial |

Penjadwal: tiap loop CLI/UART kamera/HC-SR04/indikator; 10 ms keselamatan; 20 ms IMU; 40 ms VL53L0X; 50 ms inner loop roda;
100 ms FSM (outer loop 10 Hz) lalu keselamatan lagi; 250 ms baterai; 1 s jadwal RTC. Watchdog loop aktif (macet > 5 s -> reset,
semua aktuator mati saat boot).

## 2. Pengaturan Arduino IDE

| Menu Tools | Nilai |
|---|---|
| Board | **ESP32S3 Dev Module** (paket esp32 Espressif 3.x; diuji kompilasi 3.3.12) |
| PSRAM | **OPI PSRAM** |
| Flash Size | **16MB (128Mb)** |
| USB CDC On Boot | **Disabled** (wajib, GPIO19/20 = LED G/B; sketch menolak kompilasi bila Enabled) |
| Port | USB-C berlabel **UART/COM (CH343)** |
| Serial Monitor | 115200, akhir baris **Newline** |

Baris perintah (build di luar folder proyek):

```
arduino-cli compile -b "esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,CDCOnBoot=default" --build-path %TEMP%\build_main main-esp32s3
```

**Pustaka:** hanya bawaan core (Wire, Preferences). Opsional: **"VL53L0X" oleh Pololu** (Library Manager) untuk jarak ToF
serong. Setelah dipasang, ubah `#define PAKAI_PUSTAKA_VL53L0X 0` menjadi `1` di `config.h` (tidak bisa otomatis: penemu pustaka
Arduino tidak mengikuti `__has_include`). Tanpa pustaka: alamat 0x30/0x31 tetap diatur, jarak ToF **STUB** (tidak dipakai navigasi).

## 3. Konfigurasi yang wajib diisi setelah pengukuran

Semua di `config.h` (tanda `[UKUR]`) kecuali yang bertanda "param" (ubah dengan `set <nama> <nilai>` lalu `simpan`).

| Item | Nilai sementara | Cara mengukur |
|---|---|---|
| `SIGN_MOT_KI/KA`, `SIGN_ENC_KI/KA` | +1/+1, +1/-1 (dari Uji_PID v2, **belum dikonfirmasi di PCB**) | Bringup_PCB menu `m` (roda diangkat), lalu `uji pwm 80 80 1000`: kedua roda maju, `sensor` tick naik positif |
| `SIGN_GYRO_Z` | +1 | putar robot ke kiri dengan tangan, `status`: th harus bertambah |
| `CPR_1X` | 517 (asumsi) | Bringup menu `e`, putar roda tepat 10 putaran |
| `R_RODA`, `TRACK_L` | 0,0335 / 0,231 m | `uji lurus 1.0` di lantai + meteran; `uji putar 360` + bandingkan gyro vs encoder |
| `U_MIN_GERAK` | 0 | PWM terkecil yang membuat roda berputar (`uji pwm`) |
| `ADDR_PCF` | 0x20 (PCF8574P) | `i2c`; 0x38 bila IC PCF8574**A**P |
| `TCRT_LEVEL_TEBING` | HIGH | Bringup menu `l` di atas lantai dan di tepi meja |
| `LED_AKTIF_HIGH` | true (katoda bersama) | `uji led` |
| param `vkoreksi` | 1,00 | bandingkan `status` VBAT dengan multimeter di TP1 |
| param `vwarn` / `vstop` / `vpulih` | 14,4 / 13,8 / 15,0 V | sesuai keputusan 24 Nov 2026 (berhenti 13,8 V berbeban) |
| param `ddepan`, `dlambat`, `ddarurat`, `dsamping`, `dtof`, `dtofdarurat` | 22/45/10/8 cm, 150/70 mm | posisi sensor di bumper, uji dinding |
| param `lajur` | 0,22 m | lebar sapuan nyata (overlap ~3 cm) |
| param `jarakkamera` | 0,30 m | jarak titik tengah pandangan kamera ke nozzle suction |
| param `kp`, `ki`, `kd`, `amaks` | 0,75/2,25/0,25, 0,5 | Uji_PID v2 di lantai (berbeban) |
| Kalibrasi IMU | NVS `imu` | sketch `Kalibrasi_IMU` perintah `g` lalu `s` (firmware ini memuatnya) |

## 4. Perintah CLI (ringkas, ketik `bantuan`)

`status`, `mulai`, `berhenti` / `x`, `reset`, `sensor`, `i2c`, `log 1 [ms]` (CSV), `param`, `set <nama> <nilai>`, `simpan`,
`bawaan`, `uji pwm|jalan|putar|lurus|suction|buzzer|led|tebing ...`, `kamera [ping|mulai|berhenti|status|kalib|hapus|echo|set]`,
`waktu` / `waktu set 2026-11-24 08:30:00`, `jadwal 08:30` / `jadwal off`, `gyro kalib`, `odo reset`, `versi`.

## 5. Perilaku

| State | LED | Buzzer | Keluar |
|---|---|---|---|
| BOOT (selftest, 2 s diam) | biru kedip cepat | 1 bunyi saat siap | -> IDLE |
| IDLE | hijau (kuning bila baterai peringatan) | - | `mulai` atau jadwal RTC |
| BERSIH | hijau kedip 1 Hz, cyan saat kotor | 1 bunyi pendek tiap 30 s bila baterai peringatan | selesai (2 bunyi) -> IDLE |
| HINDAR / TEBING | kuning kedip cepat | 1 bunyi saat tebing | -> BERSIH |
| BATERAI | merah kedip | 3 bunyi tiap 10 s | VBAT > 15,0 V selama 10 s -> IDLE |
| GALAT | merah tetap | 1 bunyi panjang tiap 5 s | `reset` |
| UJI | magenta | - | `berhenti` |

* **BERSIH**: diam ~1,5 s (bias gyro, suction spin-up 1,5 s, origin {W} = pose START), lalu lajur lurus pada heading 0 dengan
  kendali P heading; akhir lajur (US depan < 22 cm, ToF < 150 mm, atau 6 m) -> belok-U (putar 90, geser 0,22 m, putar 90) ke sisi
  bergantian. Geser terhalang -> pindah ke sisi sebaliknya; terhalang di kedua sisi -> selesai. Durasi maksimum 45 menit.
* **Keselamatan (menimpa semuanya)**: tebing -> rem darurat + larang maju (hanya mundur lurus), lalu mundur 10 cm dan putar
  120 derajat menjauh (180 bila kedua sensor); 5 tebing dalam 20 s -> GALAT. Fault stall/lari roda -> PWM 0 + GALAT. Miring
  > 20 derajat -> PWM 0 + GALAT. Baterai < 13,8 V selama 3 s -> rem, suction mati, alarm (BATERAI). HC-SR04 depan rusak saat
  bersih -> GALAT. Kamera tidak ada -> tetap bersih tanpa deteksi kotor.

## 6. Urutan uji yang disarankan

Lakukan dulu `harness-uji` (Bringup_PCB checklist, Kalibrasi_IMU, Uji_PID v2). Lalu dengan firmware ini:

1. **Roda diangkat**, catu daya lab berbatas arus. Boot: baca laporan SELFTEST (I2C harus 0x20, 0x30, 0x31, 0x57, 0x68, 0x69).
2. `uji tebing 1` (TCRT tidak melihat lantai saat diangkat), `uji pwm 80 80 1000`: kedua roda arah maju, `sensor` tick +.
3. `uji jalan 0.15 0 3000` lalu `uji putar 90`: tidak ada FAULT, `status` th berubah ~90.
4. `uji suction 3000`, `uji led`, `uji buzzer`, `waktu set ...`.
5. Robot di lantai, baterai penuh: `uji lurus 1.0` (ukur jarak, koreksi `R_RODA`), `uji putar 360` (koreksi `TRACK_L`).
6. Angkat robot dari lantai saat `uji jalan`: harus berhenti (tebing). Miringkan > 20 derajat: GALAT.
7. Kamera tersambung: `kamera`, lalu `kamera kalib` di lantai bersih, `kamera echo 1`.
8. `mulai` di ruang kecil berdinding, amati `log 1`. `x` untuk berhenti kapan saja.
9. Uji baterai: catu daya lab diturunkan ke 13,7 V -> state BATERAI, alarm, semua mati.

## 7. STUB dan asumsi yang perlu verifikasi perangkat keras

* VL53L0X: jarak STUB sampai pustaka Pololu dipasang dan `PAKAI_PUSTAKA_VL53L0X = 1` (cabang pustaka sudah dicek kompilasi
  terhadap API Pololu, belum dengan pustaka asli).
* Semua `SIGN_*`, CPR, geometri, polaritas TCRT/LED, ambang jarak: lihat bagian 3.
* PCF8574 sebagai TRIG HC-SR04 (keluaran quasi-bidirectional): README PCB bagian 7 butir 14.
* Algoritma cakupan sederhana tanpa peta: lajur yang sudah dibersihkan bisa dilewati lagi setelah pindah sisi.
* Tidak ada sensor belakang: semua gerak mundur dibatasi pendek (6-25 cm, maks 0,5 m pada spot clean).
