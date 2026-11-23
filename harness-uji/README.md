# Harness Uji Firmware - Robot Pembersih Lantai Otonom

Kumpulan sketch Arduino untuk menguji PCB utama (rev 004) dan subsistem robot satu per satu sebelum firmware final
(`main-esp32s3`). Peta pin mengikuti `06 Implementasi\07 Integrasi\SPEK_BERSAMA.md` bagian 3 dan
`06 Implementasi\14 PCB Utama\README.md` bagian 3.

## 1. Daftar sketch

| Sketch | Fungsi | Status |
|---|---|---|
| `Bringup_PCB/` | Menu serial untuk menyalakan PCB pertama kali dan tahap integrasi T1, T2, T3, T4, T5, T6, T8: info chip (flash/PSRAM), VBAT, pemindai I2C dengan PASS/FAIL, urutan alamat VL53L0X (0x30/0x31) + model ID, MPU9250 WHO_AM_I, DS3231 baca/set waktu, uji PCF8574, HC-SR04 (cm), TCRT5000, LED RGB, buzzer, suction 3 s, jog motor + penentuan konstanta `SIGN_*`, pantau encoder. Hanya memakai `Wire`. | baru |
| `uji_modul_b/` | Uji penggerak manual: maju/mundur/putar, ubah PWM, hitung encoder. Punya konstanta `SIGN_MOT_*`/`SIGN_ENC_*` dan berhenti otomatis 3 s setelah perintah gerak terakhir. | aktif |
| `Kalibrasi_IMU/` | Kalibrasi MPU9250 + AK8963 (bias gyro/accel, hard/soft iron magnetometer), simpan ke NVS, cetak konstanta untuk firmware final. | aktif |
| `Uji_PID_Kecepatan_Roda_v2/` | PID kecepatan roda, odometri, uji step/ramp + log CSV (`plot_log.py` untuk plot dan metrik). Punya pengaman stall / tanda salah (fault terkunci, lepas dengan `f`). | aktif |
| `Uji_Feedback_Encoder/` | Uji PID breadboard 15 Okt (arsip, lihat `Hasil_Uji_PID_2026-10-15.md`). | **USANG**: pin 36/37 bentrok PSRAM, pin 38 = LED R di PCB. Jangan dipakai di PCB. |

## 2. Urutan yang disarankan

1. **Bringup_PCB** - ikuti checklist README PCB bagian 9. Urutan menu yang disarankan:
   `i` (chip) -> `b` (VBAT) -> `s` (I2C) -> `t` (ToF) -> `s` lagi (harus 0x20, 0x30, 0x31, 0x57, 0x68, 0x69) ->
   `x`, `u`, `g`, `r`, `l` -> `c`, `z` -> `v` (suction) -> `e` (encoder dengan tangan) -> `m` (jog motor).
   Catat `SIGN_MOT_KI/KA` dan `SIGN_ENC_KI/KA` hasil menu `m`.
2. **uji_modul_b** - isi konstanta `SIGN_*` dari langkah 1, lalu cek: `w` (maju) harus membuat kedua roda maju
   dan `tickL`/`tickR` naik positif. Ukur CPR (putar roda 10 putaran) dan zona mati PWM.
3. **Kalibrasi_IMU** - setelah IMU terbaca di 0x69; kalibrasi magnetometer dilakukan setelah semua komponen terpasang di robot.
4. **Uji_PID_Kecepatan_Roda_v2** - isi konstanta `SIGN_*` yang sama, mulai dengan roda diangkat, baru kemudian di lantai.

Konvensi tanda encoder di ketiga sketch sama (hitungan naik bila C1 mendahului C2), jadi nilai `SIGN_*` bisa disalin langsung.

## 3. Pengaturan Arduino IDE

| Menu Tools | Nilai |
|---|---|
| Board | **ESP32S3 Dev Module** (paket esp32 oleh Espressif, core 2.x atau 3.x) |
| PSRAM | **OPI PSRAM** (modul N16R8 = PSRAM octal) |
| Flash Size | **16MB (128Mb)** |
| USB CDC On Boot | **Disabled** (wajib: GPIO19/20 dipakai LED G/B) |
| Upload Speed | 921600 (turunkan ke 115200 bila upload gagal) |
| Port | port **COM** dari USB-C berlabel **UART/COM (CH343)**, bukan port USB native |
| Serial Monitor | 115200 baud, akhir baris "Newline" atau "Both NL & CR" |

`Bringup_PCB` sengaja gagal dikompilasi (`#error`) bila USB CDC On Boot diaktifkan.

Kompilasi dari baris perintah (opsional):

```
arduino-cli compile -b "esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,CDCOnBoot=default" harness-uji/Bringup_PCB
```

Catatan: pada sebagian board, GPIO48 (SCL) juga tersambung ke LED RGB on-board; LED itu bisa ikut berkedip saat ada lalu lintas I2C. Tidak merusak.

## 4. Catatan keselamatan

* **Daya pertama kali dari catu daya lab dengan batas arus** (mis. 12-15 V, batas 0,3-0,5 A untuk PCB tanpa motor).
  Naikkan batas arus hanya setelah tidak ada arus tak wajar. Ikuti checklist README PCB bagian 9 (cek hubung singkat dulu).
* **Roda diangkat** (robot di atas balok, roda tidak menyentuh apa pun) untuk semua uji motor pertama:
  menu `m` Bringup_PCB, uji_modul_b, dan Uji_PID v2 sampai konstanta `SIGN_*` terbukti benar.
* Tanda motor/encoder yang salah membuat PID mendorong PWM penuh (roda "lari"). Uji_PID v2 menghentikan motor bila
  |PWM| >= 240 lebih dari 400 ms dengan roda macet atau berputar berlawanan arah target; tetap jangan mengandalkan pengaman ini.
* Suction hanya dinyalakan setelah konfirmasi `Y` dan mati otomatis setelah 3 s. Pastikan impeller bebas dan tidak ada jari/benda di saluran hisap.
* Tombol apa pun saat motor atau suction berjalan di Bringup_PCB = berhenti. Menu `0` mengembalikan semua keluaran ke keadaan aman.
* Jangan mencolok USB-C ESP32-S3-CAM saat kabel CN19 tersambung dan 5V robot menyala (lihat README PCB bagian 5).
* Pin VBAT sense (GPIO1) tidak boleh > 3,3 V; Bringup_PCB memberi peringatan bila Vadc > 3,2 V.
