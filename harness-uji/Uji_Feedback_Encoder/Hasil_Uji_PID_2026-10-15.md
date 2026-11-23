# Hasil Uji PID Kecepatan Roda (15 Oktober 2026)

Sketch: `Uji_Feedback_Encoder.ino` (folder ini). Board: ESP32-S3 Dev Module, Serial 115200 baud.

## Susunan uji
- Dua motor JGA25-370 berencoder dipasang di papan uji, driver TB6612FNG di breadboard.
- Roda belum terpasang (uji tanpa beban).
- Foto: `07 Dokumentasi/Hardware/Foto_uji_PID_kecepatan_roda_2026-10-15.jpg`.
- Screenshot: `07 Dokumentasi/Software/Serial_monitor_uji_PID_kecepatan_roda_2026-10-15.png` dan `Serial_plotter_uji_PID_kecepatan_roda_2026-10-15.png`.

## Parameter
| Parameter | Nilai |
|---|---|
| Sampling | 50 ms |
| Target kecepatan | 30 tick per 50 ms (interrupt RISING pada C1 saja) |
| Kp, Ki, Kd | 3,0 ; 1,5 ; 2,0 |
| Anti-windup integral | ±255 |
| Batas PWM | 0–255 |

## Hasil
| Data | Motor kiri | Motor kanan |
|---|---|---|
| Serial monitor (12 sampel terakhir) | 29–31 tick, rata-rata 30,0 | 29–31 tick, rata-rata 30,0 |
| Serial plotter (sampel 394–443) | 29–31 tick | 28–32 tick (dua kali lonjakan ±2) |

Kecepatan kedua motor tertahan di target dengan simpangan ±1 tick (sekitar ±3%); motor kanan sesekali ±2 tick.

## Catatan
- Nilai tick kanan negatif saat maju: arah hitung encoder kanan terbalik terhadap kiri. Kode memakai `abs()` sehingga PID tidak terpengaruh, tetapi tanda harus dibalik untuk odometri.
- Uji ini tanpa beban. PID perlu diuji ulang dengan roda terpasang di lantai.
- Satuan masih tick per 50 ms; konversi ke rpm atau m/s menunggu data CPR encoder dan rasio gearbox.
- Pin di sketch belum sama dengan PCB rev 002:

| Sinyal | Sketch uji | PCB rev 002 (SPEK_BERSAMA) |
|---|---|---|
| PWMA, AIN2, AIN1 | 38, 37, 36 | 17, 16, 15 |
| Encoder kanan C1, C2 | 10, 11 | 6, 7 |
| Motor kanan, encoder kiri | sama | sama |

  Di PCB, GPIO10 = gate MOSFET suction, GPIO11 = ECHO US1, GPIO38 = LED R. GPIO35–37 dipakai PSRAM octal pada board N16R8. Ganti pin di sketch ke pemetaan PCB sebelum dipasang di PCB utama.
