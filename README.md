# Firmware Robot Pembersih Lantai Otonom

Firmware untuk robot pembersih lantai otonom berpenggerak diferensial: suction di depan, pel pasif mikrofiber di belakang, deteksi area kotor dengan kamera.

| Folder | Isi |
|---|---|
| [`main-esp32s3/`](main-esp32s3/) | Firmware utama ESP32-S3 DevKit N16R8. Penjadwal non-blocking, PID kecepatan roda, odometri dengan heading gyro, sensor jarak (3 x HC-SR04 lewat PCF8574, 2 x VL53L0X), sensor tebing TCRT5000, IMU MPU9250, RTC DS3231, monitor baterai 4S, suction ON/OFF, supervisor keselamatan, FSM, CLI serial |
| [`visual-esp32cam/`](visual-esp32cam/) | Firmware kamera (ESP32-CAM AI-Thinker atau ESP32-S3-CAM, OV2640/OV3660): deteksi area kotor berbasis piksel gelap dengan kalibrasi lantai bersih, hasil dikirim lewat UART |
| [`harness-uji/`](harness-uji/) | Sketch uji: `Bringup_PCB` (nyala pertama PCB, I2C, sensor, arah motor), `uji_modul_b`, `Kalibrasi_IMU`, `Uji_PID_Kecepatan_Roda_v2` (+ `plot_log.py`), `Uji_Feedback_Encoder` (usang) |
| [`docs/Protokol_UART_Kamera.md`](docs/Protokol_UART_Kamera.md) | Protokol berbingkai ESP32-S3 <-> kamera (0xA5, CRC-8) |

## Urutan pakai

1. `harness-uji/Bringup_PCB`: nyala pertama, cek I2C (0x20, 0x30, 0x31, 0x57, 0x68, 0x69), sensor, LED/buzzer, suction, dan tentukan tanda arah motor/encoder.
2. `harness-uji/uji_modul_b`: cek arah motor dengan roda terangkat.
3. `harness-uji/Kalibrasi_IMU`: kalibrasi gyro/akselerometer, disimpan ke NVS `imu`.
4. `harness-uji/Uji_PID_Kecepatan_Roda_v2`: uji dan setel PID di lantai.
5. `visual-esp32cam`: flash kamera **sebelum** dipasang, kalibrasi lantai bersih.
6. `main-esp32s3`: isi nilai bertanda `[UKUR]` di `config.h`, lalu uji per state lewat CLI.

## Setelan Arduino IDE (ESP32 core 3.x)

| | Utama | Kamera |
|---|---|---|
| Board | ESP32S3 Dev Module | AI Thinker ESP32-CAM, atau ESP32S3 Dev Module untuk varian S3 |
| PSRAM | OPI PSRAM | bawaan board / OPI (S3) |
| Flash | 16MB | bawaan |
| USB CDC On Boot | **Disabled** (GPIO19/20 dipakai LED) | sesuai README kamera |
| Upload | port USB-C **COM (CH343)** | USB-TTL / ESP32-CAM-MB (IO0 ke GND saat boot) |

Status: semua target lolos compile (esp32 core 3.3.12). **Belum diuji di perangkat keras.** Tanda arah, CPR encoder, dan ambang sensor masih asumsi (lihat `[UKUR]` di `main-esp32s3/config.h`). Jarak VL53L0X butuh pustaka "VL53L0X by Pololu" lalu `PAKAI_PUSTAKA_VL53L0X 1`.

**Keselamatan:** nyala pertama pakai catu daya lab dengan batas arus; uji motor dengan roda terangkat; jangan colok USB kamera saat konektor CN19 (5 V) tersambung.
